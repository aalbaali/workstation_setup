#!/bin/sh

# @Description
# This script finds the compile_commands.json in all packages after catkin build
# and merges them all to a single compile_commands.json for autocompletion in vscode.
# adaptod from https://github.com/catkin/catkin_tools/issues/551
#
# @Flags
# --headers Include header files in the concatendated compile_commands.json
# --symlink Add symbolic link from catkin_ws to build/compile_commands.json
# 
# @Examples
#   Run from anywhere in CASE
#   cdi; ./concat.sh --headers; cd -
#   
# @Authors
# ali.jahani@avidbots.com
# amro.al-baali@avidbots.com

help() {
    echo \
    "Arguments:
       --headers : concatenate headers
       --symlink : add symbolic link to cdi directory"
}

concat_headers=0
add_symlink=0
case "$1" in
    --headers ) concat_headers=1; shift ;;
    --symlink ) add_symlink=1; shift ;;
    * ) help; exit
esac

catkin_ws=$(catkin locate)
echo "\033[96;1mcatkin_ws: $catkin_ws\033[0m"
cd $catkin_ws

[ ! -d "build" ] && echo "\e[91mPlease compile first.\e[0m" &&exit 0

concatenated="build/compile_commands.json"

echo "[" > $concatenated

first=1
for d in build/*
do
    f="$d/compile_commands.json"

    if [ -f "$f" ]; then
        if [ $first -eq 0 ]; then
            echo "," >> $concatenated
        else
            first=0
        fi

        cat $f | sed '1d;$d' >> $concatenated
    fi
done

echo "]" >> $concatenated

echo "\e[92mConcatenation is finished.\e[0m"


# Add header files into `compile_commands.json`
if [ $concat_headers -eq 1 ]; then
    if [ ! $(command -v compdb &> /dev/null) ]
    then
        echo "\e[92;1mcompdb\e[0m is not installed"
        echo "Run \e[33;1msudo pip install compdb\e[0m to install"
    else
        echo "\e[92mAdding header files into compile_commands.json\e[0m"
        
        compdb -p build/ list > compile_commands_with_headers.json

        # Replace the compile commands file with the compdb version
        mv compile_commands_with_headers.json build/compile_commands.json
    fi
fi

cd $catkin_ws

if [ $add_symlink -eq 1 ]; then
    ln -s build/compile_commands.json compile_commands.json -f
fi
