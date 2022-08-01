# Workstation setup
This repo contains script to setup a Linux workstation.
Some configs (e.g., the [gdb config](stow/gdb/.gdbinit)) assume this repo is cloned in `~/.dot`.
Thus, the repo can be cloned using
To do so, run
```bash
git clone git@github.com:aalbaali/workstation_setup.git ~/.dot
```

# Running the installation
## Default installations
The install scripts can be activated using a Makefile.
The default scripts are
- [`post_install_setup.sh`](scripts/post_install_setup.sh)
- [`post_install_setup.sh`](scripts/post_install_setup.sh)

To run the default scripts, run
```bash
make
```

## Installing specific Make objects
The other installations can be activated by running
```bash
make vscode
make latex_classes
```

## Bypassing prompts
There are some prompts for some installations. To bypass these prompts, run the [command](https://serverfault.com/questions/116299/automatically-answer-defaults-when-doing-make-oldconfig-on-a-kernel-tree)
```bash
yes "" | make
```


# Tilix
Install tilix by running
```bash
sudo apt-get install tilix
```

Ensure that the *Windowing system* is set to *X11*; *Wayland* option will have issues with Quake.
- To check the windowing system, check the *About* section in Settings.
- To set X11, [uncomment](https://trendoceans.com/how-to-enable-x11-and-disable-wayland-window-system/) the `WaylandEnable=false` line in `/etc/gdm3/custom.conf`.

## Configuring tilix
Open tilix (by searching for the app) and then open the user preferences
- Appearance
  - Window style : set to *Borderless*
  - Uncheck the *Show the terminal title even if it's the only terminal*
- Quake
  - Adjusut height percentage (I have mine set at 70%)
  - Adjust width percentage (I have mine set at 75%)

To run quake from anywhere in Linux, add a keyboard shortcut in the shortcuts settings.
I have mine set to `Super + -`, which calls the command `tilix --quake`

To run tilix using the `ctrl + alt + T` shortcut, disable the terminal's shortcut and enable a tilix shortcut.

# Ubuntu
## Dock settings
The auto-dock can be customized so that it doesn't pop up when switching applications using shortcuts.
To configure the autodock, an [editor is to be installed](https://linuxconfig.org/how-to-customize-dock-panel-on-ubuntu-22-04-jammy-jellyfish-linux).
Once installed, launch the *dconf Editor* application and go to `/org/gnome/shell/extensions/dash-to-dock/hot-keys` and turn off the *hotkeys-overlay* and *hotkeys-show-dock* options.

# FAQs and common issues
## Neovim's [coc-ccls](https://github.com/Maxattax97/coc-ccls) may give an error of *unable to load global extension*.
The error can be [resolved](https://github.com/Maxattax97/coc-ccls/issues/5) by running
```bash
cd ~/.config/coc/extensions/node_modules/coc-ccls
ln -s node_modules/ws/lib lib
```

# Resources
- Allison Thackston's [workstation setup](https://github.com/athackst/workstation_setup/)
- Hiding terminal titlebar: [AskUbuntu Question](https://askubuntu.com/questions/1230157/how-to-remove-title-bar-from-terminal-on-the-new-ubuntu-20-04)
- Marc Gallant's dotfiles [kam3k/dot_files](https://github.com/kam3k/dot_files)
