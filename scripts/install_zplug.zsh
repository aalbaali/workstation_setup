#!/bin/zsh

# Install zplug non-interactively. Note that this script runs using zsh

ZSH_NONINTERACTIVE=true
source ~/.zshrc
zplug install
