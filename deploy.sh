#!/bin/bash
set -eo pipefail
PS4=$'D \t$EPOCHREALTIME\t '

if ! command -v ssh; then echo missing ssh: sudo apt install ssh; exit 1; fi

WHERE_RSYNC="/usr/local/frc/bin/rsync"
if [[ $1 = "--install-rsync" ]]; then
    wget -O /tmp/rsync-arm https://github.com/jbruechert/rsync-static/releases/download/continuous/rsync-arm
    scp /tmp/rsync-arm "admin@10.57.35.2:$WHERE_RSYNC"
    rm /tmp/rsync-arm
fi

# trust gradle (BAD IDEA)
JARFILE=(build/libs/*.jar)
if [[ "${#JARFILE[@]}" -ne 1 ]]; then echo warn: not one jarfile, only using first; fi
DEPLOY_FILES=($(find src/main/deploy -type f))
echo "deploying jarfile: ${JARFILE}"
echo "  as well as ${#DEPLOY_FILES[@]} files in src/main/deploy"

CTRL_PATH="ControlPath /tmp/deploy_socket"
TARGET="lvuser@10.57.35.2"
SSH_CMD=(ssh -o "$CTRL_PATH" "$TARGET")

# connect and start control multiplexer
"${SSH_CMD[@]}" -fN -o 'ControlMaster yes' >/dev/null
# schedule disconnection to happen whether we exit normally or not
trap EXIT "${SSH_CMD[@]}" -O stop

# quick sanity check (untested)
if ! "${SSH_CMD[@]}" [[ -f "$WHERE_RSYNC" ]]; then
    echo no rsync on target! use deploy.sh --install-rsync
    exit 1
fi

# source path file
"${SSH_CMD[@]}" . /etc/profile.d/natinst-path.sh

# stop robot program
"${SSH_CMD[@]}" /usr/local/frc/bin/frcKillRobot.sh -t 2> /dev/null

# transfer files
# don't use $DEPLOY_FILES here because of rsync semantics
rsync -aivP --rsh "ssh -o '$CTRL_PATH'" --rsync-path "$WHERE_RSYNC" \
    "$JARFILE" src/main/deploy "$TARGET":~/test-transfer

# restart robot program
"${SSH_CMD[@]}" /usr/local/frc/bin/frcKillRobot.sh -r -t 2> /dev/null

# the trap will handle disconnection
