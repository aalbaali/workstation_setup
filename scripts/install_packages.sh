#!/bin/bash

# Install packages I normally use
apt-get update

apt-get install -y \
  vim-gtk \
  neovim \
  tmux \
  git \
  htop \
  curl \
  silversearcher-ag \
  stow \
  unzip \
  libclang-dev \
  clang-format \
  clang-tidy \
  cmake \
  cmake-curses-gui \
  build-essential \
  gdb \
  ncdu \
  rsync \
  xclip \
  libpython3-dev \
  gawk \
  xsel \
  meld \
  tree \
  fd-find \
  zsh \
  brightnessctl \
  tilix \
  tmux \
  nodejs \
  npm \
  golang

# Install lazygit using golang
go install github.com/jesseduffield/lazygit@latest

# Install pre-commit
pip install pre-commit

# Install fzf if it doesn't already exist
if [ ! command -v fzf &> /dev/null ]; then
  read -p "Install fzf? (Y/n): " yn
  case $yn in
    "" | [Yy]* )
      # Check if repo already exists
      if [ ! -d $HOME/.fzf ]; then
        echo "Cloning fzf into ~/.fzf"
        git clone --depth 1 https://github.com/junegunn/fzf.git ~/.fzf
      fi

      if [ -f $HOME/.fzf/install ]; then
        echo "Installing fzf"
        y | $HOME/.fzf/install
      else
        echo "$HOME/.fzf/install script does not exist"
      fi
    ;;
  esac
fi
