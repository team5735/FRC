#!/bin/bash
function die {
    echo -e "$1"
    exit 1
}

[[ -z "$1" ]] && die "make sure you copied the command correctly. i need to know which WPILib version to install"

dir="WPILib_Linux-x64-$1"
file="$dir.tar.gz"
url="https://packages.wpilib.workers.dev/installer/v$1/$file"
err="$(wget --spider $url 2>&1)"
[[ $? -ne 0 ]] && die "$(echo "seems like i can't download WPILib right now. try again later\nerror:\n")$err"

set -e

echo installing necessary packages
sudo apt-get install --yes wget pv pigz tar git gh

echo downloading robot code to ~/robotics/FRC
if [[ ! -d ~/robotics/FRC ]]; then
    mkdir --parents ~/robotics
    git clone https://github.com/team5735/FRC ~/robotics/FRC
else
    cat <<END
~/robotics/FRC is already present, not overwriting
to redownload the repository, run the following in your terminal:
rm -rf ~/robotics/FRC; git clone https://github.com/team5735/FRC ~/robotics/FRC
END
fi

echo logging into GitHub\; if you do not have an account you can make it here
echo to cancel the login, use Ctrl+C in the terminal
trap "echo cancelled auth" SIGINT
gh auth login --git-protocol HTTPS --hostname github.com --web || true
trap SIGINT

echo downloading WPILib
wget --quiet --show-progress "$url" -O $file
rm --recursive --force $dir
size=$(pigz --list $file | cut --delimiter ' ' --fields 2)
unpigz --to-stdout $file | pv --interval 0.2 --name extract --size $size | tar --extract --file -
echo running the WPILib installer
"$dir"/WPILibInstaller-CLI --yes --install-mode all
