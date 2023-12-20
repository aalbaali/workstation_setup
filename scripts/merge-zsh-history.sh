#!/bin/bash
# Merge two zsh history files, removing duplicates
#
# Usage: merge-zsh-history.sh <history1> <history2> <merged>
#
# Example:
#   merge-zsh-history.sh ~/.zsh_history ~/.zsh_history2 ~/.zsh_history_merged
# Details:
#  - Copied from https://gist.github.com/calexandre/63547c8dd0e08bf693d298c503e20aab
#  - Inspired on https://david-kerwick.github.io/2017-01-04-combining-zsh-history-files/
set -e
history1=$1
history2=$2
merged=$3

echo "Merging history files: $history1 + $history2"

test ! -f $history1 && echo "File $history1 not found" && exit 1
test ! -f $history2 && echo "File $history2 not found" && exit 1

cat $history1 $history2 | awk -v date="WILL_NOT_APPEAR$(date +"%s")" '{if (sub(/\\$/,date)) printf "%s", $0; else print $0}' | LC_ALL=C sort -u | awk -v date="WILL_NOT_APPEAR$(date +"%s")" '{gsub('date',"\\\n"); print $0}' > $merged

echo "Merged to: $merged"
