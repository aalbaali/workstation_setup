#!/bin/sh

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
