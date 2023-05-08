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

# Source starship
eval "$(starship init zsh)"

# Plugins
zplug "zsh-users/zsh-autosuggestions"
zplug "zsh-users/zsh-syntax-highlighting"
zplug "lib/history", from:oh-my-zsh

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
setopt autopushd pushdignoredups

# Autosuggestion colour
ZSH_AUTOSUGGEST_HIGHLIGHT_STYLE="fg=#707880"
# Autosuggest completion
bindkey '^@' autosuggest-accept
# Autosuggestion completion and execution ('^M' is the return key)
bindkey '^[;' autosuggest-execute
bindkey '^[l' clear-screen

# Add local to path
export PATH="$HOME/.local/bin:$PATH"

## The set of commands below slow zsh
#setopt autocd 
#autoload -Uz compinit
#compinit

## Networking alias
#if [ -f ~/avidbots_networking/aliases ]; then
#    . ~/avidbots_networking/aliases
#fi

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

###############################
## fzf
###############################
[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh

# Autojump
[[ -s ~/.autojump/etc/profile.d/autojump.sh ]] && source ~/.autojump/etc/profile.d/autojump.sh

# NOTE: The command below sometimes slows loading zsh. It could be that the commands are configured for bash and not for zsh
#autoload -U compinit && compinit -u

export KEYTIMEOUT=0
# Enable Ctrl-x to edit command line in vim
autoload -U edit-command-line
zle -N edit-command-line
bindkey '^x' edit-command-line

export PATH="$HOME/.yarn/bin:$HOME/.config/yarn/global/node_modules/.bin:$PATH"

# Load custom functions and aliases
[ -f ~/.functions.sh ] && source ~/.functions.sh
[ -f ~/.rosfunctions.sh ] && source ~/.rosfunctions.sh
[[ $(docker --help 2>/dev/null) ]] && [ -f ~/.dockerfunctions.sh ] && source ~/.dockerfunctions.sh

if [[ -f  ~/.zsh/git-completion.bash ]] then
  zstyle ':completion:*:*:git:*' script ~/.zsh/git-completion.bash
  fpath=(~/.zsh $fpath)

  autoload -Uz compinit && compinit
fi

# Append PATHs (function imported from .functions)
export PATH="$PATH:/home/$USERNAME/.local/bin"
export PATH="$PATH:/home/$USERNAME/go/bin"

# Solve a tilix issue
# https://gnunn1.github.io/tilix-web/manual/vteconfig/
if [ $TILIX_ID ] || [ $VTE_VERSION ]; then
        source /etc/profile.d/vte.sh
fi
