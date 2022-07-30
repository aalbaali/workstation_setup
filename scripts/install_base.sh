#/bin/sh
set -e

# Minimum requireements for development. This script is usually used to install scripts in a
# minimal temporary container

# Sometime `sudo` is not available and instead the `sudo` is the default.
# Thus, check if `sudo` exists, and if not, then ignore it.
if command -v sudo &> /dev/null;
then
  export SU=sudo
else
  export SU=''
fi
$SU apt-get update

# setup sources
$SU apt-get install -y \
  bash-completion \
  build-essential \
  cmake \
  curl \
  git \
  git-lfs \
  make \
  pass \
  snapd \
  software-properties-common \
  ssh \
  wget

# Fzf
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
      echo "$HOME/.fzf/install does not exist"
    fi
  ;;
esac

