#!/bin/bash

# Install VSCode custom configurations
# Amro Al-Baali

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"

# Create a backup
if [ -d $HOME/.config/Code/ ]; then
  cp -r $HOME/.config/Code $HOME/.config/Code.bak
else
  mkdir -p $HOME/.config/Code
fi

cp -r $DIR/../user/.config/Code/ $HOME/.config
