#!/bin/bash
set -x -o pipefail
PS4=$'F \t$EPOCHREALTIME\t '

cd $(git rev-parse --show-toplevel)

git stash push --quiet
trap "git stash pop --quiet" exit

files=$(find . -name '*.java' -type f)
[[ -z "$files" ]] && exit 0
<<<"$files" xargs clang-format --style="file:./.clang-format" -i

modified="$(git status --porcelain=v1 --untracked-files=no | grep '^ M ' | cut --bytes 4-)"
[[ -z "$modified" ]] && exit 0
count=$(<<<"$modified" wc -l)

if [[ "$1" = "--no-ask" ]]; then
    should_commit="y"
else
    set +x
    read -N 1 -p "commit auto-formatting of $count file(s) (Y/n/c)? " should_commit
    [[ "$should_commit" != $'\n' ]] && echo
    set -x
fi

case "$should_commit" in
    y|Y|$'\n')
        <<<"$modified" xargs --delim=\\n git add
        git commit --message="(format.java.sh) clang-format $count file(s)"
        ;;
    c|C)
        echo cancelling push
        git restore "$modified"
        ;;
    *)
        echo not committing
        git restore "$modified"
        ;;
esac
