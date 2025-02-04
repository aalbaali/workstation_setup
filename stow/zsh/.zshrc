# 'source' autocompletion from bash
autoload bashcompinit
bashcompinit
autoload -U compinit && compinit -u

# Ensure locale settings are valid
export LANG=en_US.UTF-8
export LANGUAGE=en_US:en
export LC_ALL=en_US.UTF-8

# Source starship
eval "$(starship init zsh)"

# Plugins
source $HOME/.config/zsh/zsh-autosuggestions/zsh-autosuggestions.zsh
source $HOME/.config/zsh/zsh-syntax-highlighting/zsh-syntax-highlighting.zsh
source $HOME/.config/zsh/oh-my-zsh/lib/history.zsh

export USER="${USER:=${USERNAME}}"

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

# Add vcpkg
export VCPKG_ROOT=${HOME}/Dev/vcpkg
if [[ -d $VCPKG_ROOT ]] && [[ ! ":$PATH:" == *":/vcpkg:"* ]]; then
  export PATH=$VCPKG_ROOT:$PATH
fi

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
export PATH="$HOME/.fzf/bin:$PATH"
[ -f ~/.fzf/bin/fzf ] && source <(~/.fzf/bin/fzf --zsh)

# Enable Ctrl-x to edit command line in vim
autoload -U edit-command-line
zle -N edit-command-line
bindkey "^x^e" edit-command-line

# Load custom functions and aliases
[ -f ~/.config/.pickle.sh ] && source ~/.config/.pickle.sh
[ -f ~/.config/.logger.sh ] && source ~/.config/.logger.sh
[ -f ~/.config/.functions.sh ] && source ~/.config/.functions.sh
[ -f ~/.config/.rosfunctions.sh ] && source ~/.config/.rosfunctions.sh
[ -f ~/.config/.tmuxfunctions.sh ] && source ~/.config/.tmuxfunctions.sh
[[ $(docker --help 2>/dev/null) ]] && [ -f ~/.config/.dockerfunctions.sh ] && source ~/.config/.dockerfunctions.sh

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

[[ -f "${HOME}/.cargo/env" ]] && source "$HOME/.cargo/env"

# Add julia, if it exists
if [ -f "${HOME}/.juliaup/bin/julia" ]; then
  path=('/home/amro/.juliaup/bin' $path)
  export PATH
fi
