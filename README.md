# Workstation setup
This repo contains script to setup a Linux workstation.

# Running the installation
## Default installations
The install scripts can be activated using a Makefile.
The default scripts are
- [`install_base.sh`](scripts/install_base.sh)
- [`install_vscode_conf.sh`](scripts/install_vscode_config.sh)
- [`install_user_config.sh`](scripts/install_user_config.sh)

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

# Resources
- Allison Thackston's [workstation setup](https://github.com/athackst/workstation_setup/)
- Hiding terminal titlebar: [AskUbuntu Question](https://askubuntu.com/questions/1230157/how-to-remove-title-bar-from-terminal-on-the-new-ubuntu-20-04)

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
