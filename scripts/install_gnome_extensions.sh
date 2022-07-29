#!/bin/bash

apt-get install -y \
  gnome-tweaks \
  gnome-shell-extensions \
  gnome-shell-extension-ubuntu-dock

# Activate the ubuntu dock extension
gnome-extensions enable ubuntu-dock@ubuntu.com

echo -e "\033[93;1mA restart may be required"
