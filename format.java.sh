#!/bin/bash
set -ex -o pipefail

cd $(git rev-parse --show-toplevel)

current_branch="$(git symbolic-ref --short HEAD)"

git stash push --quiet

files=$(find . -name '*.java' -type f)
[[ -z "$files" ]] && continue
<<<"$files" xargs clang-format --style="file:./.clang-format" -i

modified="$(git status --porcelain=v1 --untracked-files=no | grep '^ M ' | cut --bytes 4-)"
[[ -z "$modified" ]] && continue
count=$(<<<"$modified" wc -l)

set +x
read -N 1 -p "commit auto-formatting of $count file(s) on $current_branch (Y/n/c)? " should_commit
[[ "$should_commit" != $'\n' ]] && echo
set -x

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

git stash pop --quiet
