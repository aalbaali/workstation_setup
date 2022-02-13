#!/bin/bash

# Set up VSCode settings
# Amro Al-Baali

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null 2>&1 && pwd )"
# Create a backup
if [ -d $HOME/.config/Code/ ]; then
  cp -r $DIR/../user/.config/Code $DIR/../user/.config/Code.bak
fi
cp -r $DIR/../user/.config/Code/ $HOME/.config/Code/