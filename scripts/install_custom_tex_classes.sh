#!/bin/bash

# Install custom latex scripts
# Amro Al-Baali

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

wget -c https://raw.githubusercontent.com/aalbaali/latex_classes/master/install.sh?token=GHSAT0AAAAAABRI2OOT3SC4C7IOHDIQJ5K4YQRPABA -O $DIR/install_tex.sh
source $DIR/install_tex.sh
rm $DIR/install_tex.sh