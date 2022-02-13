#!/bin/bash

# Install custom latex scripts
# Amro Al-Baali

wget -c https://raw.githubusercontent.com/aalbaali/latex_classes/master/install.sh?token=GHSAT0AAAAAABRI2OOT3SC4C7IOHDIQJ5K4YQRPABA -O /workspace/install_tex.sh
source install_tex.sh
rm /workspace/install_tex.sh

# Copy `latexmk` config files, if it doesn't exist already
if [ ! -f $HOME/.latexmkrc ]
then
  DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
  cp $DIR/../user/.latexmkrc $HOME/
fi