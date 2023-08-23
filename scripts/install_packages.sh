#!/bin/bash

# Check if version $1 is larger than $2
function _is_version_larger {
  v1="$1"
  v2="$2"

  v1_major=$(echo "$v1" | cut -d. -f1)
  v1_minor=$(echo "$v1" | cut -d. -f2)
  v2_major=$(echo "$v2" | cut -d. -f1)
  v2_minor=$(echo "$v2" | cut -d. -f2)

  [[ $v1_major -lt $v2_major ]] && return -1 # False (return error)
  [[ $v1_major -gt $v2_major ]] && return 0 # True (return no error)
  [[ $v1_minor -gt $v2_minor ]] && return 0 # True (return no error)
  return -1 # False (return error)
}

# Add ppa repo to install latest version of git
sudo add-apt-repository -y ppa:git-core/ppa

# Install packages I normally use
sudo apt-get update

pkgs=(
  'stow'
  'vim-gtk'
  'tmux'
  'git'
  'htop'
  'curl'
  'ripgrep'
  'silversearcher-ag'
  'unzip'
  'libclang-dev'
  'clang-format'
  'clang-tidy'
  'cmake'
  'cmake-curses-gui'
  'build-essential'
  'gdb'
  'ncdu'
  'rsyncexuberant-ctags'
  'software-properties-common'
  'bat'
  'python3-venv'
  'npm'
  'autojump'
)

pids=()
for pkg in "${pkgs[@]}"; do
  sudo apt-get install -y "$pkg"
  pids+=($!)
done

for pid in "${pids[@]}"; do
  wait "$pid"
done

# To install gnome-tweaks
# sudo apt-get install -y gnome-tweaks

# Current ubuntu version
ubuntu_version=$(lsb_release -sr)

# Add exa apt packages. Installation depends on the system version
if _is_version_larger "$ubuntu_version" "20.04" \
      && ! command dpkg -s exa > /dev/null 2>&1 \
      && [ -f /etc/os-release ]; then
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

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Install nvim proper version
source $SCRIPT_DIR/install_nvim.sh

# Install node proper version
source $SCRIPT_DIR/install_node.sh

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
