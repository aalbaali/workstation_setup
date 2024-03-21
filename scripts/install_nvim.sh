#!/bin/bash

# Install latest stable nvim release, if nvim is older than a specific version

if ! command -v nvim > /dev/null ; then
   echo -e "\033[96;mWill install nvim\033[0m"
  _SHOULD_INSTALL_NVIM=true
else
  # install latest nvim release, if there's no nvim or if it's older than the target version
  _nvim_version="$(nvim --version | grep -oP 'NVIM v\K\d+\.\d+')"
  _nvim_version_major=$(echo $_nvim_version | awk -F'.' '{print $1}')
  _nvim_version_minor=$(echo $_nvim_version | awk -F'.' '{print $2}')
  _nvim_target_version_major=0
  _nvim_target_version_minor=9
  if [[ $_nvim_version_major -le $_nvim_target_version_major ]] \
          && [[ $_nvim_version_minor -le _nvim_target_version_minor ]]; then
    echo -e "\033[96;mWill update nvim version (currently at $_nvim_version)\033[0m"
    _SHOULD_INSTALL_NVIM=true
  fi
fi
if [[ -n "$_SHOULD_INSTALL_NVIM" ]]; then
  cd /tmp
  curl -LO https://github.com/neovim/neovim/releases/download/stable/nvim-linux64.tar.gz
  tar -xvf nvim-linux64.tar.gz
  sudo mv -f /tmp/nvim-linux64 /opt
  sudo ln -s /opt/nvim-linux64/bin/nvim /usr/bin/nvim -f
  cd -
fi


