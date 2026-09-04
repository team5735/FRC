#!/bin/bash
set -ex -o pipefail

find . -name '*.java' -type f |\
    xargs clang-format --style="file:./.clang-format" -i
modified="$(git status --porcelain=v1 --untracked-files=no | grep '^ M ' | cut -b 4-)"
# exit if nothing changed
[[ $? -ne 0 ]] && exit 0
<<<"$modified" xargs --delim=\\n git add

count=$(<<<"$modified" wc -l)
bot="github-actions[bot]"
bot_email="$bot@users.noreply.github.com"
GIT_AUTHOR_NAME=$bot    GIT_AUTHOR_EMAIL=$bot_email\
GIT_COMMITTER_NAME=$bot GIT_COMMITTER_EMAIL=$bot_email\
git commit --message="reformat $count files"
