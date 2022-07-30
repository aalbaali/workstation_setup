#!/bin/bash

# @brief Install custom latex scripts
# @detail Installing the custom latex scripts allows latex to be compiled from anywhere on the 
#         computer without needing to specify the custom commands location.
#         It is a better practice to keep custom commands within the same file.
#         A better alternative is to use Docker containers.
# @author Amro Al-Baali
#

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

wget -c https://raw.githubusercontent.com/aalbaali/latex_classes/master/install.sh -O $DIR/install_tex.sh
source $DIR/install_tex.sh
rm $DIR/install_tex.sh

# Copy `latexmk` config files, if it doesn't exist already
if [ ! -f $HOME/.latexmkrc ]
then
  cp $DIR/../user/.latexmkrc $HOME/
fi
