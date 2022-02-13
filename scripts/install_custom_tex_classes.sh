#!/bin/bash

# Install custom latex scripts
# Amro Al-Baali

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

wget -c https://raw.githubusercontent.com/aalbaali/latex_classes/master/install.sh
source $DIR/install_tex.sh
rm $DIR/install_tex.sh

# Copy `latexmk` config files, if it doesn't exist already
if [ ! -f $HOME/.latexmkrc ]
then
  cp $DIR/../user/.latexmkrc $HOME/
fi