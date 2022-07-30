#!/bin/bash

# Install gnome extensions such as the `tweaks` app. These extensions allow customization to Ubuntu
# such as customizing the auto dock

apt-get install -y \
  gnome-tweaks \
  gnome-shell-extensions \
  gnome-shell-extension-ubuntu-dock \
  dconf-editor

# Activate the ubuntu dock extension
gnome-extensions enable ubuntu-dock@ubuntu.com

echo -e "\033[93;1mA restart may be required"
