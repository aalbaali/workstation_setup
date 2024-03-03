#!/bin/bash
# Add ppa repo to install latest version of git
sudo add-apt-repository ppa:git-core/ppa

# Install packages I normally use
sudo apt-get update

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
  'rsync'
  'xclip'
  'libpython3-dev'
  'gawk'
  'nodejs'
  'tree'
  'fd-find'
  'zsh'
  'brightnessctl'
  'tilix'
  'kitty'
  'exuberant-ctags'
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

# Add exa apt packages. Installation depends on the system version
if ! command dpkg -s eza > /dev/null 2>&1 && [ -f /etc/os-release ]; then
  . /etc/os-release
  if [ "$NAME" == "Ubuntu" ]; then
    VERSION=$(echo $VERSION_ID | cut -d '.' -f 1)
    if [ $VERSION -gt 20 ]; then
      sudo mkdir -p /etc/apt/keyrings
      wget -qO- https://raw.githubusercontent.com/eza-community/eza/main/deb.asc | sudo gpg --dearmor -o /etc/apt/keyrings/gierens.gpg
      echo "deb [signed-by=/etc/apt/keyrings/gierens.gpg] http://deb.gierens.de stable main" | sudo tee /etc/apt/sources.list.d/gierens.list
      sudo chmod 644 /etc/apt/keyrings/gierens.gpg /etc/apt/sources.list.d/gierens.list
      sudo apt update
      sudo apt install -y eza
    fi
  fi
fi

function install_deb() {
  local url="$1"
  wget "$url" -O /tmp/deb_to_install.deb
  sudo dpkg -i /tmp/deb_to_install.debi
  rm /tmp/deb_to_install.debim
}
# Install dust if it doesn't already exist
if [ ! command -v dust &> /dev/null ]; then
  install_deb https://github.com/bootandy/dust/releases/download/v0.9.0/du-dust_0.9.0-1_amd64.deb
fi

# Install zoxide
if [ ! command -v dust &> /dev/null ]; then
  install_deb https://github.com/ajeetdsouza/zoxide/releases/download/v0.9.4/zoxide_0.9.4-1_amd64.deb
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
