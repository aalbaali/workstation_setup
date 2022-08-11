#!/bin/bash

# Install dotfiles and setup some applications (e.g., neovim, tmux, etc.)
# Amro Al-Baali

# Exit if package is not installed
EXIT_IF_ERROR=true

# Packages to check if installed
PACKAGES_TO_CHECK=(
  git
  stow
  tmux
)

# Stow packages to install. If set to true, then will install without a prompt
declare -A STOW_PACKAGES=(
  [bash]=false
  [zsh]=false
  [functions]=false
  [git]=false
  [nvim]=false
  [nvim-setup]=false
  [tmux]=false
  [tmux-setup]=false
  [vim]=false
  [vim-setup]=false
  [clang_format]=false
  [gdb]=false
  [latex]=false
  [vscode]=false
)

while true; do
  case "$1" in
    -a | --all ) INSTALL_ALL=true; shift ;;
    --all-stow ) ALL_STOW=true; shift ;;
    --no-prompt ) EXIT_IF_ERROR=false; shift ;;
    --bash ) STOW_PACKAGES[bash]=true; shift ;;
    --no-bash ) STOW_PACKAGES[bash]=false; shift ;;
    --clang-format ) STOW_PACKAGES[clang_format]=true; shift ;;
    --no-clang-format ) STOW_PACKAGES[clang_format]=false; shift ;;
    --gdb ) STOW_PACKAGES[gdb]=true; shift ;;
    --no-gdb ) STOW_PACKAGES[gdb]=false; shift ;;
    --git ) STOW_PACKAGES[git]=true; shift ;;
    --no-git ) STOW_PACKAGES[git]=false; shift ;;
    --latex ) STOW_PACKAGES[latex]=true; shift ;;
    --no-latex ) STOW_PACKAGES[latex]=false; shift ;;
    --nvim ) STOW_PACKAGES[nvim]=true; shift ;;
    --no-nvim ) STOW_PACKAGES[nvim]=false; shift ;;
    --nvim-setup ) STOW_PACKAGES[nvim-setup]=true; shift ;;
    --no-nvim-setup ) STOW_PACKAGES[nvim-setup]=false; shift ;;
    --vim ) STOW_PACKAGES[vim]=true; shift ;;
    --no-vim ) STOW_PACKAGES[vim]=false; shift ;;
    --vim-setup ) STOW_PACKAGES[vim-setup]=true; shift ;;
    --no-vim-setup ) STOW_PACKAGES[vim-setup]=false; shift ;;
    --tmux ) STOW_PACKAGES[tmux]=true; shift ;;
    --no-tmux ) STOW_PACKAGES[tmux]=false; shift ;;
    --tmux-setup) STOW_PACKAGES[tmux-setup]=true; shift ;;
    --no-tmux-setup ) STOW_PACKAGES[tmux-setup]=false; shift ;;
    --vscode ) STOW_PACKAGES[vscode]=true; shift ;;
    --no-vscode ) STOW_PACKAGES[vscode]=false; shift ;;
    --zsh ) STOW_PACKAGES[zsh]=true; shift ;;
    --no-zsh ) STOW_PACKAGES[zsh]=false; shift ;;
    --functions ) STOW_PACKAGES[functions]=true; shift ;;
    --no-functions ) STOW_PACKAGES[no-functions]=false; shift ;;
    -- ) shift; break ;;
    * ) shift; break ;;
  esac
done

# Check if packages are installed
for p in ${PACKAGES_TO_CHECK[@]}; do
  if [[ ! -a $(which $p) ]]; then
    echo "Error: $p is not installed. Please install"
    ( $EXIT_IF_ERROR ) && exit 1
  fi
done

# Symlink everything in stow directory to home directory
cd stow
if [ ! $? ]; then
  echo "Error encountered... aborting"
  exit 1
fi

# Install stow packages
# Install without prompt if insallation config is set to true
for app_full in */; do
  app=${app_full%/}
  if [[ $ALL_STOW  ]] || [[ $INSTALL_ALL ]] || [[ "${STOW_PACKAGES[$app]}" = true ]]; then
    yn=y
  elif [ "${STOW_PACKAGES[$app]}" = false ]; then
    yn=n
  else
    # Prompt user if stow directory is not found in the dictionary
    read -p "Install $app? (y/N): " yn
  fi
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
if [ $INSTALL_ALL ] || [ "${STOW_PACKAGES[vim-setup]}" = true ]; then
  yn=y
elif [ "${STOW_PACKAGES[vim-setup]}" = false ]; then
  yn=n
else
  read -p "Install vim plugins and setup YCM? (y/N): " yn
fi
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
if [ $INSTALL_ALL ] || [ "${STOW_PACKAGES[nvim-setup]}" = true ]; then
  yn=y
elif [ "${STOW_PACKAGES[nvim-setup]}" = false ]; then
  yn=n
else
  read -p "Install neovim plugin manager and plugins? (y/N): " yn
fi
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

    ;;
    [Nn]* ) ;;
    * )     ;;
esac


# Set up tmux
if [ $INSTALL_ALL ] || [ "${STOW_PACKAGES[tmux-setup]}" = true ]; then
  yn=y
elif [ "${STOW_PACKAGES[tmux-setup]}" = false ]; then
  yn=n
else
  read -p "Install neovim plugin manager and plugins? (y/N): " yn
fi
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

