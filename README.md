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
- [`install_packages.sh`](scripts/install_packages.sh), and
- [`post_install_setup.sh`](scripts/post_install_setup.sh) using the arguments `--zsh --functions --git --nvim --clang_format --gdb --nvim-setup`

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

## Post-processing scripts
Package-specific config files are stored under the `stow` directory.
These config files can be linked to the home directory using [`stow`](https://www.gnu.org/software/stow/manual/stow.html).
The config files can be set up using the [`post_install_setup.sh`](scripts/post_install_setup.sh).
The takes multiple arguments based on the config files to link (using `stow`).
Furthermore, the script can set up some packages (e.g., neovim plugins installations).

For example, running (from the root of this repo)
```bash
./scripts/post_install_setup.sh --zsh --functions --git --nvim --clang_format --gdb --nvim-setup
```
will install the config files for zsh, git, neovim, clang-format, gdb debugginer, the custom [`.functions.sh`](`stow/functions/.functions.sh`) script, and install the packages for neovim.



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
  - Set font to a nerd font (make sure to install the font first by running the [installation script](./scripts/install_nerd_fonts.sh))
- Quake
  - Adjusut height percentage (I have mine set at 70%)
  - Adjust width percentage (I have mine set at 75%)

To run quake from anywhere in Linux, add a keyboard shortcut in the shortcuts settings.
I have mine set to `Super + -`, which calls the command `tilix --quake`

To run tilix using the `ctrl + alt + T` shortcut, disable the terminal's shortcut and enable a tilix shortcut.

# Sioyek
To make Sioyek the default PDF viewer, the `sioyek.desktop` needs to be installed.
This is done as follows:
1. Link `sioyek.desktop` from `stow/sioyek/.local/share/applications/sioyek.desktop` into `~/.local/share/application`
2. Link `sioyek-icon-linux.png` from `stow/sioyek/.local/share/Sioyek/sioyek-icon-linux.png` into `~/.local/share/Sioyek`
3. Install the application by running `sudo desktop-file-install sioyek.desktop` from `~/.local/share/application`

# Ubuntu
## Dock settings
The auto-dock can be customized so that it doesn't pop up when switching applications using shortcuts.
To configure the autodock, an [editor is to be installed](https://linuxconfig.org/how-to-customize-dock-panel-on-ubuntu-22-04-jammy-jellyfish-linux).
```bash
sudo apt install dconf-editor
```
Once installed, launch the *dconf Editor* application and go to `/org/gnome/shell/extensions/dash-to-dock/hot-keys` and turn off the *hotkeys-overlay* and *hotkeys-show-dock* options.
Also, set `/org/gnome/shell/extensions/dash-to-dock/shortcut-timeout` timeout value to `0` (you need to turn off `Use default value`, and set the `Custom value` to 0).

To load dconf settings, run
```bash
dconf load / < dconf-settings.ini
```

## Borderless windows
```
sudo apt install gnome-tweaks
```

# Node
To install different versions of NodeJS, you can use the [`n` npm package](https://blog.hubspot.com/website/update-node-js).
```bash
# Make sure npm is installed
sudo apt-get install npm

# Install n manager
sudo npm install -g n

# Install latest node version
sudo n latest

# Or, install specific version
# sudo n 16.20
```
Alternatively, use the node version manager `nvm`:
```bash
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.38.0/install.sh | bash
source ~/.zshrc # Or source ~/.bashrc
nvm install 16.20
```

# FAQs and common issues
## Neovim's [coc-ccls](https://github.com/Maxattax97/coc-ccls) may give an error of *unable to load global extension*.
The error can be [resolved](https://github.com/Maxattax97/coc-ccls/issues/5) by running
```bash
cd ~/.config/coc/extensions/node_modules/coc-ccls
ln -s node_modules/ws/lib lib
```
This snippet is executed in the [`post_install_setup.sh`](`scripts/post_install_setup.sh`) script when passing the `--nvim-setup` flag.
Note that this script will only work after installing the `coc-ccls` plugin first (i.e., it sufficies to launch `nvim` before launching the script).


## ZSH fonts
Additional packages may be required for the ZSH font to be displayed correctly.
Check the [spaceship-prompt](https://github.com/spaceship-prompt/spaceship-prompt) and [this answer](https://askubuntu.com/questions/271566/how-to-get-ubuntu-to-display-unicode-supplementary-characters).

## Docker detach bindings
Docker has detach binding set to `ctrl+p` by default.
This causes issues while inside a container such as accessing the previous command.
To fix this issue, [change the default Docker detach binding](https://stackoverflow.com/a/20863838/15749309) by modifying `~/.docker/config.json` to be
```json
{
    "detachKeys": "ctrl-z,z"
}
```

# Resources
- Allison Thackston's [workstation setup](https://github.com/athackst/workstation_setup/)
- Hiding terminal titlebar: [AskUbuntu Question](https://askubuntu.com/questions/1230157/how-to-remove-title-bar-from-terminal-on-the-new-ubuntu-20-04)
- Marc Gallant's dotfiles [kam3k/dot_files](https://github.com/kam3k/dot_files)
