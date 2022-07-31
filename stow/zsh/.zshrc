# 'source' autocompletion from bash
autoload bashcompinit
bashcompinit

export ZPLUG_HOME=~/.zplug
export ZSH=~/.oh-my-zsh

# Get zplug if it doesn't exist
if [[ ! -d $ZPLUG_HOME ]];then
    git clone https://github.com/b4b4r07/zplug $ZPLUG_HOME
fi

# Source zplug
source $ZPLUG_HOME/init.zsh

# Plugins
zplug "zsh-users/zsh-autosuggestions"
zplug "zsh-users/zsh-syntax-highlighting"
zplug "lib/history", from:oh-my-zsh
zplug "denysdovhan/spaceship-prompt", use:spaceship.zsh, from:github, as:theme
zplug "arzzen/calc.plugin.zsh"

# Install plugins if there are plugins that have not been installed
if ! zplug check --verbose; then
  printf "Install? [y/N]: "
  if read -q; then
    echo; zplug install
  fi
fi

# Source plugins
zplug load

# Options
# setopt autopushd pushdignoredups

# configure spaceship prompt
SPACESHIP_PROMPT_ORDER=(
  dir           # Current directory section
#  user          # Username section
#  host          # Hostname section
#  git           # Git section (git_branch + git_status)
#  line_sep      # Line break
  char          # Prompt character
)
SPACESHIP_DIR_TRUNC=0
SPACESHIP_DIR_TRUNC_REPO=false
#SPACESHIP_PROMPT_ADD_NEWLINE=true
#SPACESHIP_PROMPT_SEPARATE_LINE=true
#SPACESHIP_PROMPT_FIRST_PREFIX_SHOW=true
SPACESHIP_USER_SHOW=never # When to show user name
SPACESHIP_USER_PREFIX='no '

# Stop prompt from setting tmux title
DISABLE_AUTO_TITLE=true

# Autosuggestion colour
ZSH_AUTOSUGGEST_HIGHLIGHT_STYLE="fg=#707880"
# Autosuggest completion
bindkey '^ ' autosuggest-accept
# Autosuggestion completion and execution ('^M' is the return key)
bindkey '^;' autosuggest-execute

# Source fzf
[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh
export FZF_DEFAULT_OPS="--extended"
export FZF_DEFAULT_COMMAND="fdfind --type f"
export FZF_CTRL_T_COMMAND="$FZF_DEFAULT_COMMAND"

# Enable Ctrl-x to edit command line in vim
autoload -U edit-command-line
zle -N edit-command-line
bindkey '^x' edit-command-line

# Add local to path
export PATH="$HOME/.local/bin:$PATH"



#setopt autocd 
#autoload -Uz compinit
#compinit

source ~/Dev/robot-dev/docker_init.sh &> /dev/null
# Networking alias
# Check https://avidbots.atlassian.net/wiki/spaces/ROBOTSYS/pages/24838213/D
if [ -f ~/avidbots_networking/aliases ]; then
    . ~/avidbots_networking/aliases
fi

# Source localrc
[ -f ~/.localrc ] && source ~/.localrc


unsetopt menu_complete
#setopt list_ambiguous
unsetopt list_ambiguous
setopt bash_auto_list

fpath=(~/.zsh $fpath)

########################
# workspaces
########################
if [ -d $HOME/.workspace ]; then
  for d in $HOME/.workspace/*/; do
    ws=$(basename $d)
    input='$1'
    num_input='$#'
    source /dev/stdin << EOF
function create_${ws}() {
  if [ ${num_input} -eq 0 ]; then
    echo "Usage: create_${ws} <name>";
    return 1;
  fi
  cp -r $d/. ${input};
};
EOF
  done
fi

########################
# Github
########################
if command -v gh &> /dev/null; then
  eval "$(gh completion -s bash)"
fi

########################
# ROS
########################
function noetic_gazebo() {
  docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -u ros athackst/ros:noetic-gazebo gazebo
}

export PATH="$HOME/.yarn/bin:$HOME/.config/yarn/global/node_modules/.bin:$PATH"

# Load custom functions and aliases
[ -f ~/.functions.sh ] && source ~/.functions.sh

# Add local bin to path
export PATH="$PATH:~/.local/bin"
