#!/bin/bash
set -o pipefail
# add set -x for debugging and profiling
PS4=$'F \t$EPOCHREALTIME\t '

cd $(git rev-parse --show-toplevel)

if [[ "$1" = "--no-ask" ]]; then
    response="$(git stash push)"
    [[ "$response" = "No local changes to save" ]] || trap "git stash pop --quiet" exit
fi

files=$(find . -name '*.java' -type f)
[[ -z "$files" ]] && exit 0
<<<"$files" xargs clang-format --style="file:./.clang-format" -i

modified="$(git status --porcelain=v1 --untracked-files=no | grep '^ M ' | cut --bytes 4-)"
[[ -z "$modified" ]] && echo nothing formatted && exit 0
[[ "$1" == "--check" ]] && echo unformatted files: $'\n'"$modified" exit 1
count=$(<<<"$modified" wc -l)

if [[ "$1" = "--no-ask" ]]; then
    should_commit="y"
else
    read -N 1 -p "commit auto-formatting of $count file(s) (Y/n/c)? " should_commit
    [[ "$should_commit" != $'\n' ]] && echo
fi

case "$should_commit" in
    y|Y|$'\n')
        <<<"$modified" xargs git add
        git commit --message="(format.java.sh) clang-format $count file(s)"
        ;;
    c|C)
        echo restoring
        <<<"$modified" xargs git restore
        ;;
    *)
        echo $count files left formatted
        ;;
esac
