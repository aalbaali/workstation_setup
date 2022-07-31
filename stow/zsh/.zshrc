# 'source' autocompletion from bash
autoload bashcompinit
bashcompinit

export ZPLUG_HOME=~/.zplug
export ZSH=/home/amro.al-baali/.oh-my-zsh

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

# Aliases
alias ls='ls --color=auto'
alias ll='ls -alF'
alias l='ls -CF'
alias ta='tmux a -t'
alias now='watch -x -t -n 0.01 date +%s.%N' 
alias o=xdg-open
alias k='k -h'
alias cdg='cd "$(git rev-parse --show-cdup)"'
alias cds='cd "$(git rev-parse --show-superproject-working-tree)"'
alias ja='ninja'
alias ctest='ctest --output-on-failure'
alias cm='cmake -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=RelWithDebInfo'
alias cmd='cmake -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=Debug'
alias fd='fdfind'
alias a="apt-cache search '' | sort | cut --delimiter ' ' --fields 1 | fzf --multi --cycle --reverse --preview 'apt-cache show {1}' | xargs -r sudo apt install -y"

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

# Make and change into a directory
mkcd()
{
  mkdir -p -- "$1" &&
  cd -P -- "$1"
}

# Fuzzy checkout git branch with fzf
gz() 
{
  local branches branch
  branches=$(git branch --all | grep -v HEAD) &&
  branch=$(echo "$branches" |
           fzf-tmux -d $(( 2 + $(wc -l <<< "$branches") )) +m) &&
  git checkout $(echo "$branch" | sed "s/.* //" | sed "s#remotes/[^/]*/##")
}

# Log CPU and memory usage of a process
logpid() { while sleep 1; do  ps -p $1 -o pcpu= -o pmem= ; done; }

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

# Python default version
#alias python=python3
alias gd="git diff"
alias gdc="git diff --cached"
alias gll="git log --graph --oneline --all --decorate"
alias gs="git status"
alias gm="git commit -s"
# alias gm="git commit"
alias ga="git add"
alias gb="git branch"
alias gc="git checkout"

alias dps="docker ps"
alias dpsa="docker ps -a"
alias di="docker image"
alias dils="docker image ls"
alias drm="docker rm"
alias drmi="docker rmi"

alias vsc="code -n ."

# Open tmux after a restart. It creates a new session then deletes it.
alias mux='pgrep -vx tmux > /dev/null && \
		tmux new -d -s delete-me && \
		tmux run-shell ~/.tmux/plugins/tmux-resurrect/scripts/restore.sh && \
		tmux kill-session -t delete-me && \
		tmux attach || tmux attach'

#source /home/amro.al-baali/.bash_aliases

fpath=(~/.zsh $fpath)
########################
# git shortcus
########################
#if [ -f $HOME/.aliases/git_aliases.sh ]; then
#  source $HOME/.aliases/git_aliases.sh
#
#  alias gll="git log --graph --oneline --all --decorate"
#  alias gs="git status"
#  alias gm="git commit"
#  alias ga="git add"
#  alias gb="git branch"
#  alias gc="git checkout"
#fi

alias vsc="code -n ."

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


# Open tmux after a restart. It creates a new session then deletes it.
alias mux='pgrep -vx tmux > /dev/null && \
		tmux new -d -s delete-me && \
		tmux run-shell ~/.tmux/plugins/tmux-resurrect/scripts/restore.sh && \
		tmux kill-session -t delete-me && \
		tmux attach || tmux attach'


################################################################################
# Docker
################################################################################
alias dps="docker ps"
alias dpsa="docker ps -a"
alias di="docker image"
alias dils="docker image ls"
alias drm="docker rm"
alias drmi="docker rmi"
alias ds="docker start"

# List running container names
function dn() {
  docker ps $@ | awk 'FNR > 1 {print $(NF)}'
}

alias dna="dn -a"

# Execute already running containers
function dex() {
  # Start container if not started
  if [ $(docker container inspect -f '{{.State.Status}}' $1) != "running" ]; then
    echo -e "\033[93mContainer \033[36;1m$1\033[0;93m not running. Will start it\033[0m"
    ds $1;
  fi
  docker exec -it $1 bash
}

# Complete `dex` with a list of running containers
#   1. Get all docker *running* containers (i.e., witout the `-a` flag)
#   2. Get all rows after the first row (FNR > 1)
#   3. Print last column ($(NF))
complete -C "dn" dex

# Julia aliases
alias jj=julia
alias jfranklin="julia -e 'using Franklin; serve()'"
alias jpluto="julia -e 'using Pluto; Pluto.run()'"

# File sizes
alias sz=ncdu

alias nv=nvim

alias nodejs=node

export PATH="$HOME/.yarn/bin:$HOME/.config/yarn/global/node_modules/.bin:$PATH"
