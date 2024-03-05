# 'source' autocompletion from bash
autoload bashcompinit
bashcompinit
autoload -U compinit && compinit -u

export ZPLUG_HOME=~/.zplug
export ZSH=~/.oh-my-zsh

# Get zplug if it doesn't exist
if [[ ! -d $ZPLUG_HOME ]];then
    git clone https://github.com/zplug/zplug $ZPLUG_HOME
fi

# Source zplug
source $ZPLUG_HOME/init.zsh

# Source starship
eval "$(starship init zsh)"

# Plugins
zplug "zsh-users/zsh-autosuggestions", as:plugin
zplug "zsh-users/zsh-syntax-highlighting", as:plugin
zplug "lib/history", as:plugin, from:oh-my-zsh

# Install plugins if there are plugins that have not been installed
# Toggle prompt by setting/unsetting the env variable `ZSH_NONINTERACTIVE`
if ! zplug check --verbose; then
  zplug install
fi

# Source plugins
zplug load

# Options
setopt autopushd pushdignoredups

# Retrieve and run last command
function run-again {
    # get previous history item
    zle up-history
    # confirm command
    zle accept-line
}
# define run-again widget from function of the same name
zle -N run-again
# Set 'alt+:' to run last command
bindkey '^[:' run-again

# Autosuggestion colour
ZSH_AUTOSUGGEST_HIGHLIGHT_STYLE="fg=#707880"
# Autosuggest completion
bindkey '^@' autosuggest-accept
# Autosuggestion completion and execution ('^M' is the return key)
bindkey '^[;' autosuggest-execute
bindkey '^[l' clear-screen
bindkey -s '^[p' 'git pull ^M'
bindkey -s '^[P' 'git push ^M'
bindkey -s '^[f' 'git fetch --prune ^M'
bindkey -s '^[k' 'cd .. ^M'
bindkey -s '^[j' 'cd - ^M'
bindkey -s '^[r' '!!^M^M'

# Add local to path
export PATH="$HOME/.local/bin:$PATH"
export PATH="$(which fzf):$PATH"

## The set of commands below slow zsh
#setopt autocd 
autoload -Uz compinit
compinit

# Source localrc
[[ -f ~/.localrc ]] && source ~/.localrc

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
# if command -v gh &> /dev/null; then
#   eval "$(gh completion -s bash)"
# fi

###############################
## fzf
###############################
[ -f ~/.fzf.zsh ] && source ~/.fzf.zsh

# Zoxide
eval "$(zoxide init --cmd cd zsh)"

# Zoxide
eval "$(zoxide init --cmd cd zsh)"

# Enable Ctrl-x to edit command line in vim
autoload -U edit-command-line
zle -N edit-command-line
bindkey "^x^e" edit-command-line

# Load custom functions and aliases
[ -f ~/.logger.sh ] && source ~/.logger.sh
[ -f ~/.functions.sh ] && source ~/.functions.sh
[ -f ~/.rosfunctions.sh ] && source ~/.rosfunctions.sh
[ -f ~/.tmuxfunctions.sh ] && source ~/.tmuxfunctions.sh
[ -f ~/.pickle.sh ] && source ~/.pickle.sh
[[ $(docker --help 2>/dev/null) ]] && [ -f ~/.dockerfunctions.sh ] && source ~/.dockerfunctions.sh

if [[ -f  ~/.zsh/git-completion.bash ]] then
  zstyle ':completion:*:*:git:*' script ~/.zsh/git-completion.bash
  fpath=(~/.zsh $fpath)
  autoload -Uz compinit && compinit
fi

## Append PATHs (function imported from .functions)
#export PATH="$PATH:/home/$USERNAME/.local/bin"
#export PATH="$PATH:/home/$USERNAME/go/bin"
#export PATH="$HOME/.yarn/bin:$HOME/.config/yarn/global/node_modules/.bin:$PATH"

# Solve a tilix issue
# https://gnunn1.github.io/tilix-web/manual/vteconfig/
if [ $TILIX_ID ] || [ $VTE_VERSION ] && [ -f /etc/profile.d/vte.sh ]; then
    source /etc/profile.d/vte.sh
fi

# Import OPENAI_API_KEY
[ -f ~/.chat_gpt_key.zsh ] && source ~/.chat_gpt_key.zsh

export NVM_DIR="$HOME/.nvm"
[ -s "$NVM_DIR/nvm.sh" ] && \. "$NVM_DIR/nvm.sh"  # This loads nvm (it slows down loading zsh)
[ -s "$NVM_DIR/bash_completion" ] && \. "$NVM_DIR/bash_completion"  # This loads nvm bash_completion

#[ -f ~/.resh/shellrc ] && source ~/.resh/shellrc # this line was added by RESH

#source "$HOME/.cargo/env"
