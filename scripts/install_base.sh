#/bin/sh
set -e

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

# python3-pip \
# snapd \

# $SU apt-get install -y python-is-python3
# $SU update-alternatives --install /usr/bin/pip pip /usr/bin/pip3 1
# pip install -U pip
