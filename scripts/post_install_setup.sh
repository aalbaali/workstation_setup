#!/bin/bash

# Install dotfiles and setup some applications (e.g., neovim, tmux, etc.)
# Amro Al-Baali

# Check for required dependencies before continuing:
if [[ ! -a $(which git) ]]; then
  echo "Error: git is not installed. Please install git first."
  exit 1
fi

if [[ ! -a $(which stow) ]]; then
  echo "Error: stow is not installed. Please install stow first."
  exit 1
fi

if [[ ! -a $(which tmux) ]]; then
  echo "Error: tmux is not installed. Please install tmux first."
  exit 1
fi

# Symlink everything in stow directory to home directory
cd stow
echo "pwd: $PWD"
if [ ! $? ]; then
  echo "Error encountered... aborting"
  exit 1
fi

for app in */; do
  read -p "Install $app? (y/N): " yn
  case $yn in
      [Yy]* )
          echo "installing \"$app\""
	  stow -t ${HOME} $app
      ;;
      [Nn]* ) ;;
      * )     ;;
  esac
done;

# Set up vim
read -p "Install vim plugins and setup YCM? (y/N): " yn
case $yn in
    [Yy]* )
	if [[ ! -a $(which vim) ]]; then
	  echo "Error: vim is not installed. Please install vim first."
	  exit 1
	fi

	# Install vim plugins
	vim +PlugInstall +qall

	# Compile YouCompleteMe
	~/.vim/plugged/YouCompleteMe/install.py --clangd-completer
    ;;
    [Nn]* ) ;;
    * )     ;;
esac



# Set up nvim
read -p "Install neovim plugin manager and plugins? (y/N): " yn
case $yn in
    [Yy]* )
    # Ensure nodejs is installed (for COC plugin)
    if [[ ! -a $(which node) ]]; then
      echo "Error: nodejs is not installed. Please install nodejs first by running."
      echo -e "\033[93;1m"
      echo "curl -sL install-node.vercel.app/lts | sudo -E bash -"
      echo -e "\033[0m"
      exit 1
    fi

    # Install vim-plug plugin manager
    curl -fLo ~/.local/share/nvim/site/autoload/plug.vim --create-dirs \
        https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
    nvim +'PlugInstall --sync' +'PlugUpdate' +qa

#	# Install vim plugins
#	vim +PlugInstall +qall
    ;;
    [Nn]* ) ;;
    * )     ;;
esac


# Set up nvim
read -p "Set up tmux? (y/N): " yn
case $yn in
    [Yy]* )
	# Set up tmux plugin manager
	mkdir -p ~/.tmux/plugins
	if [ ! -d ~/.tmux/plugins/tpm ]; then
	  git clone https://github.com/tmux-plugins/tpm ~/.tmux/plugins/tpm
	fi

	# Install tmux plugins by starting a server (but not attaching to it),
	# creating a new session (but not attaching to it), installing the
	# plugins, then killing the server
	tmux start-server
	tmux new-session -d
	~/.tmux/plugins/tpm/scripts/install_plugins.sh
	tmux kill-server
    ;;
    [Nn]* ) ;;
    * )     ;;
esac

