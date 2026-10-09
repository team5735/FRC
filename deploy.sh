#!/bin/bash
set -eo pipefail
shopt -s lastpipe
PS4=$'D \t$EPOCHREALTIME\t '

function tic {
    before=$EPOCHREALTIME
}

function toc {
    after=$EPOCHREALTIME
    echo "$1:" $(<<<"printf '%.3f', $after-$before" perl)s
}

if ! command -v ssh > /dev/null; then echo missing ssh: sudo apt install ssh; exit 1; fi

where_rsync="/usr/local/frc/bin/rsync"
if [[ $1 = "--install-rsync" ]]; then
    wget -O /tmp/rsync-arm https://github.com/jbruechert/rsync-static/releases/download/continuous/rsync-arm
    scp /tmp/rsync-arm "admin@10.57.35.2:$where_rsync"
    rm /tmp/rsync-arm
fi

# find out what WPILib version to use bundled JDK of
grep "String wpilibYear" "$(git rev-parse --show-toplevel)/settings.gradle" |\
    sed "s/[[:space:]]*String wpilibYear = '\([0-9a-z_]*\)'/\1/" | read wpilib_version
wpilib_year="${wpilib_version::4}" 
if [[ "$wpilib_year" -le 2026 ]]; then
    export JAVA_HOME=$HOME/wpilib/${wpilib_year}/jdk
else
    export JAVA_HOME=$HOME/.local/share/wpilib/${wpilib_version}/jdk
fi

tic
echo start gradle
"$(git rev-parse --show-toplevel)/gradlew" --console plain jar | sed -n -e '/^$/q' -e 'p'
toc build
# bash more like stupid
exit 0

# trust gradle (BAD IDEA)
jarfile=(build/libs/*.jar)
if [[ "${#jarfile[@]}" -ne 1 ]]; then echo warn: not one jarfile, only using first; fi
deploy_files=($(find src/main/deploy -type f))
echo "deploying jarfile: ${jarfile}"
echo "  as well as ${#deploy_files[@]} files in src/main/deploy"

ctrl_path="ControlPath /tmp/deploy_socket"
remote="lvuser@10.57.35.2"
ssh_cmd=(ssh -o "$ctrl_path" "$remote")

# connect and start control multiplexer
tic
"${ssh_cmd[@]}" -fN -o 'ControlMaster yes' >/dev/null
toc login
# schedule disconnection to happen whether we exit normally or not
trap EXIT "${ssh_cmd[@]}" -O stop

# quick sanity check (untested)
if ! "${ssh_cmd[@]}" [[ -f "$where_rsync" ]]; then
    echo no rsync on target! use deploy.sh --install-rsync
    exit 1
fi

# source path file
"${ssh_cmd[@]}" . /etc/profile.d/natinst-path.sh

# stop robot program
"${ssh_cmd[@]}" /usr/local/frc/bin/frcKillRobot.sh -t 2> /dev/null

# transfer files
# don't use $deploy_files here because of rsync semantics
tic
rsync_display_opts="--info=name,progress,stats"
rsync -aiv "$rsync_display_opts" --rsh "ssh -o '$ctrl_path'" --rsync-path "$where_rsync" \
    "$jarfile" src/main/deploy "$remote":~/test-transfer
toc transfer

# restart robot program
"${ssh_cmd[@]}" /usr/local/frc/bin/frcKillRobot.sh -r -t 2> /dev/null

# the trap will handle disconnection
