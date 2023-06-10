#!/bin/bash

# Install packages I normally use
sudo apt-get update

sudo apt-get install -y \
  vim-gtk \
  tmux \
  git \
  htop \
  curl \
  ripgrep \
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
  nodejs \
  tree \
  fd-find \
  zsh \
  brightnessctl \
  tilix \
  kitty \
  exuberant-ctags \
  software-properties-common \
  bat \
  python3-venv \
  npm \
  autojump

# To install gnome-tweaks
# sudo apt-get install -y gnome-tweaks

# Add exa apt packages. Installation depends on the system version
if ! command dpkg -s exa > /dev/null 2>&1 && [ -f /etc/os-release ]; then
  . /etc/os-release
  if [ "$NAME" == "Ubuntu" ]; then
    VERSION=$(echo $VERSION_ID | cut -d '.' -f 1)
    if [ $VERSION -gt 20 ]; then
      # The current system is Ubuntu and its version is higher than 20.04"
      wget http://archive.ubuntu.com/ubuntu/pool/universe/r/rust-exa/exa_0.10.1-2_amd64.deb
      sudo apt install ./exa_0.10.1-2_amd64.deb
      rm exa_0.10.1-2_amd64.deb
    else
      # The current system is Ubuntu but its version is not higher than 20.04
      sudo add-apt-repository -y ppa:spvkgn/exa
      sudo apt-get update && sudo apt-get install exa
    fi
  fi
fi


# Installing `bat` using apt-get creates `batcat` as the default binary. To set `bat`, create a
# symbolic link
if command -v batcat &> /dev/null && ! command -v bat &> /dev/null
then
    sudo ln -s "$(command -v batcat)" "$(dirname $(command -v batcat))/bat"
    echo "Symbolic link created from bat to batcat"
fi

# Install nvim proper version
source ./install_nvim.sh

# Install node proper version
source ./install_node.sh

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
