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
zplug "plugins/web-search", from:oh-my-zsh
zplug "denysdovhan/spaceship-prompt", use:spaceship.zsh, from:github, as:theme
zplug "arzzen/calc.plugin.zsh"

# Install plugins if there are plugins that have not been installed
# Toggle prompt by setting/unsetting the env variable `ZSH_NONINTERACTIVE`
if [ ! -v ZSH_NONINTERACTIVE ] && ! zplug check --verbose; then
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
  user          # Username section
  host          # Hostname (e.g., computer name) section
  dir           # Current directory section
  git           # Git section (git_branch + git_status)
  char          # Prompt character
)

SPACESHIP_PROMPT_ADD_NEWLINE=false
SPACESHIP_PROMPT_SEPARATE_LINE=false
SPACESHIP_PROMPT_FIRST_PREFIX_SHOW=true
SPACESHIP_PROMPT_PREFIXES_SHOW=true
SPACESHIP_PROMPT_SUFFIXES_SHOW=true
SPACESHIP_PROMPT_DEFAULT_PREFIX=true
SPACESHIP_PROMPT_DEFAULT_SUFFIX=true

SPACESHIP_CHAR_PREFIX=''
SPACESHIP_CHAR_SUFFIX=' '
SPACESHIP_CHAR_SYMBOL='$'
SPACESHIP_CHAR_SYMBOL_ROOT=$SPACESHIP_CHAR_SYMBOL
SPACESHIP_CHAR_SYMBOL_SECONDARY='-'
SPACESHIP_CHAR_COLOR_SUCCESS='green'
SPACESHIP_CHAR_COLOR_FAILURE='red'
SPACESHIP_CHAR_COLOR_SECONDARY='yellow'

SPACESHIP_USER_PREFIX=''
SPACESHIP_USER_SUFFIX=''
SPACESHIP_USER_COLOR='green'
SPACESHIP_USER_COLOR_ROOT='red'

SPACESHIP_HOST_SHOW=always
SPACESHIP_HOST_SHOW_FULL=true
SPACESHIP_HOST_PREFIX='@'
SPACESHIP_HOST_SUFFIX=': '
SPACESHIP_HOST_COLOR='blue'
SPACESHIP_HOST_COLOR_SSH='red'

SPACESHIP_DIR_SHOW=true
SPACESHIP_DIR_PREFIX=''
SPACESHIP_DIR_SUFFIX=' '
SPACESHIP_DIR_TRUNC=0
SPACESHIP_DIR_TRUNC_PREFIX='.../'
SPACESHIP_DIR_TRUNC_REPO=false
SPACESHIP_DIR_COLOR='cyan'

SPACESHIP_GIT_SHOW=true
SPACESHIP_GIT_PREFIX='('
SPACESHIP_GIT_SUFFIX=') '
SPACESHIP_GIT_BRANCH_PREFIX='➜ '
SPACESHIP_GIT_BRANCH_COLOR='blue'

SPACESHIP_USER_SHOW=always # When to show user name
SPACESHIP_GIT_STATUS_SHOW=true
SPACESHIP_GIT_STATUS_COLOR='magenta'

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
export FZF_DEFAULT_COMMAND='ag --hidden --ignore .git -g ""'
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

export PATH="$HOME/.yarn/bin:$HOME/.config/yarn/global/node_modules/.bin:$PATH"

# Load custom functions and aliases
[ -f ~/.functions.sh ] && source ~/.functions.sh

# Append PATHs (function imported from .functions)
export PATH="PATH:/home/$USERNAME/.local/bin"
export PATH="PATH:/home/$USERNAME/go/bin"
