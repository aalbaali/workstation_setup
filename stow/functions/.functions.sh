# @brief Generic functions and aliases used by bash and zsh (i.e., they're sourced)
# @author Amro Al-Baali

####################
# ROS
####################
[[ -n $ROS_DISTRO ]] && [ -f ~/.rosfunctions.sh ] && source ~/.rosfunctions.sh

################################################
# Application aliases
################################################
alias nv=nvim
alias fd="fdfind"
alias fdh="fdfind -H -I"
alias agh="ag --hidden"
alias rgh="rg --hidden"
alias vsc="code -n ."
alias jj=julia
alias jfranklin="julia --project=Project.toml -e 'using Franklin; serve()'"
alias jpluto="julia -e 'using Pluto; Pluto.run()'"
#alias vpdf=sioyek
alias vpdf=evince            # Pdf viewer
alias sz=ncdu                # view file usage
alias h=htop                 # View system performance
alias e=exa                  # Alternative to `ls`
alias plot=gnuplot           # Data plotter
alias tk=tokei               # Info about code
alias c=sgpt                 # Shell GPT
alias mex="chmod +x "        # Make executable
alias pwdt="echo ${PWD##*/}" # Print truncated directory (i.e., without the full path)

################################################
# System-related aliases and functions
################################################
# Set ls as exa, if the command is installed
if command -v exa >/dev/null 2>&1; then
  alias ls='exa'
  alias ll='exa -alF'
  alias l='exa -1'
else
  alias ls='ls --color=auto'
  alias ll='ls -alF'
  alias l='ls -1'
fi

# Set bat as cat, if the command is installed
if command -v bat >/dev/null 2>&1; then
  alias cat='bat'
fi

alias ta='tmux a -t'
alias now='watch -x -t -n 0.01 date +%s.%N' 
alias o=xdg-open
alias k='k -h'
alias cdg='cd "$(git rev-parse --show-cdup)"'
alias cdr='cd "$(git rev-parse --show-superproject-working-tree)"' # Change to git root director
alias cdd='cd ~/shared/data'
alias cdsh='cd ~/shared'
alias ja='ninja'
alias ctest='ctest --output-on-failure'
alias cm='cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=RelWithDebInfo -S . -B build'
alias cmb='cmake --build build'
alias cmd='cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=Debug'
alias cmn='cmake -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=RelWithDebInfo'
alias cmnd='cmake -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=Debug'
alias a="apt-cache search '' | sort | cut --delimiter ' ' --fields 1 | fzf --multi --cycle --reverse --preview 'apt-cache show {1}' | xargs -r sudo apt install -y"
alias nn="nnn-static"
alias zz="source ~/.zshrc"

# Open current path in file explorer (fe) for a given directory and current directory
alias fe="nautilus --browser"

# Copy current working directory
cpwd() { pwd | tr -d '\n' | xclip -selection clipboard; }

# Make and change into a directory
mkcd()
{
  mkdir -p -- "$1" &&
  cd -P -- "$1"
}

# Log CPU and memory usage of a process
logpid() { while sleep 1; do  ps -p $1 -o pcpu= -o pmem= ; done; }

# Add to path
# https://superuser.com/a/39995
pathadd() {
  if [ -d "$1" ] && [[ ":$PATH:" != *":$1:"* ]]; then
    export PATH="${PATH:+"$PATH:"}$1"
  fi
}

# Go up multiple directories
function cd_up() {
  cd $(printf "%0.s../" $(seq 1 $1 ));
}
alias 'cd..'='cd_up'

# Symbolic link using full path
lnn()
{
  CMD="ln -s "$(pwd)/$1" ${@:2}"
  echo "$CMD"
  eval "$CMD"
}

# Link binaries to local
lnb() {
  CMD="sudo ln -s "$(pwd)/$1" /usr/local/bin ${@:2}"
  echo "$CMD"
  eval "$CMD"
}

###############################
## fzf
###############################
export FZF_DEFAULT_OPS="--extended"
export FZF_DEFAULT_COMMAND='ag --hidden --ignore .git -g ""'
#export FZF_DEFAULT_COMMAND='fdfind -H -I'
#export FZF_CTRL_T_COMMAND="$FZF_DEFAULT_COMMAND"
# Preview file content using bat (https://github.com/sharkdp/bat)
export FZF_CTRL_T_OPTS="
  --preview 'bat -n --color=always {}'
  --bind 'ctrl-e:become(vim {+})'
  --bind 'ctrl-/:change-preview-window(down|hidden|)'
  --bind 'ctrl-y:execute-silent(echo -n {} | xclip -selection clipboard)+abort'"
# CTRL-/ to toggle small preview window to see the full command
# CTRL-Y to copy the command into clipboard using pbcopy
export FZF_CTRL_R_OPTS="
  --preview 'echo {}' --preview-window up:3:hidden:wrap
  --bind 'ctrl-/:toggle-preview'
  --bind 'ctrl-e:become(vim)'
  --bind 'ctrl-y:execute-silent(echo -n {2..} | xclip -selection clipboard)+abort'
  --color header:italic
  --header 'Press CTRL-Y to copy command into clipboard'"
# Print tree structure in the preview window
export FZF_ALT_C_OPTS="--preview 'tree -C {}'"

# Find files that contain regex expressions and preview them using fzf previewer
agp () {
  local str files file
  str="$1"
  file=$(ag -l "$str" ${@:2} | fzf-tmux --preview 'bat -n --color=always {}')
  
  # Abort if no file is selected
  [ -z "$file" ] && return

  # Find line number
  local line_num
  line_nums=$(ag "$str" "$file" --column | ag "^[0-9]" | head -n 1)
  line_num=$(echo "$line_nums" | awk -F':' '{print $1}')
  col_num=$(echo "$line_nums" | awk -F':' '{print $2}')
  nv +$line_num "$file" +"normal $col_num|"
}

# Takes a list of files as arguments and opens the selection in neovim
fnv() {
  files=$(cat - | tr ' ' '\n')
  file=$( echo $files | fzf-tmux --preview 'bat -n --color=always {}')

  # Abort if no file is selected
  [ -z "$file" ] && return

  # Find line number
  nv "$file"
}

# Search for strings in git-changed files
agd() {
  local str file
  str="$1"
  file=$(git diff --name-only -G "$str" | fzf-tmux --preview 'bat -n --color=always {}')

  # Abort if no file is selected
  [ -z "$file" ] && return

  # Find line number
  local line_num
  line_nums=$(ag "$str" "$file" --column | ag "^[0-9]" | head -n 1)
  line_num=$(echo "$line_nums" | awk -F':' '{print $1}')
  col_num=$(echo "$line_nums" | awk -F':' '{print $2}')
  nv +$line_num "$file" +"normal $col_num|"
}

####################
# Git
####################
# Git fuzzy
alias gf="git fuzzy"

alias gll="git log --graph --oneline --decorate"
alias gla="git log --graph --oneline --all --decorate"
alias gss="git status -s"
alias gs="git status"
alias gm="git commit -s"
alias ga="git add"
alias gb="git branch"
alias gc="git checkout"
alias gd="git diff"
alias gdn="git diff --name-only"
alias gdc="git diff --cached"

alias g="git"
complete -o default -o nospace -F _git g


# Fuzzy checkout git branch with fzf
gz() 
{
  local branches branch
  branches=$(git branch --all | grep -v HEAD) &&
  branch=$(echo "$branches" |
           fzf-tmux -d $(( 2 + $(wc -l <<< "$branches") )) +m) &&
  git checkout $(echo "$branch" | sed "s/.* //" | sed "s#remotes/[^/]*/##")
}

# lazygit
alias lg=lazygit

ghttptossh() {
  # First argument is the repo location. Default value is "."
  local repo_dir="."
  if [[ -n "$1" ]]; then
    repo_dir="$1"
  fi

  # Replace git https with ssh links
  local remote_http=$(git remote -v | awk '{print $2}' | head -n 1 )

  # Replace links if doesn't have git@ (i.e., it's an ssh link)
  if [[ "$remote_http" != *"git@"* ]]; then
    # Get the link without the https
    remote_bare=$(echo "$remote_http" | awk -F'https://' '{print $NF}')

    # Remove the 'github' part
    remote_site=$(echo $remote_bare | awk -F'/' '{print $1}')
    remote_username=$(echo $remote_bare | awk -F'/' '{print $2}')
    remote_repo=$(echo $remote_bare | awk -F'/' '{print $3}')


    # Add ssh to it
    remote_ssh="git@$remote_site:$remote_username/$remote_repo"

    # Remove remote
    git remote remove origin

    # Add ssh
    git remote add origin $remote_ssh

    git fetch --prune
    # Track current branch to master
    local local_branch=$(git branch | awk '{print $NF}')
    git branch -u origin/$local_branch
  fi

  echo "git remote -v"
  git remote -v
}

####################
# Zip/tar
####################
zipall() {
  for dir in *
  do
    if [[ -d $dir ]]
    then
      zip -r $dir.zip $dir
    fi
  done
}

# Expand tar.gz files
untar() {
  tar -xvf $@
}

####################
# Tmux
####################
# Open tmux after a restart. It creates a new session then deletes it.
alias mux='pgrep -vx tmux > /dev/null && \
		tmux new -d -s delete-me && \
		tmux run-shell ~/.tmux/plugins/tmux-resurrect/scripts/restore.sh && \
		tmux kill-session -t delete-me && \
		tmux attach || tmux attach'

####################
# CMake
####################
# Run CMake using default arguments stored in `~/.cmake/default_flags`
cmdef() {
    local args
    if [ -f ~/.cmake/default_flags ]; then
        args=$(cat ~/.cmake/default_flags | sed '/^#/d' | awk '{print "-D"$0}' | tr '\n' ' ')
    else
        echo "~/.cmake/default_flags doesn't exist. Not passing any args"
        args=""
    fi
    echo -e "CMake args:\n\033[96;1m${args}\033[0m"
    cmake $@ $args
}

# Temporary: read latest screenshot and copy to clipboard
alias ri="ls /home/aa/Pictures/Screenshots/Screen* -t | head -n 1 | xargs -I{} tesseract "{}" ddd && cat ddd.txt | xclip -selection clipboard"

####################
# Python venv
####################
# Create a Python virtual environment
_default_venv="venv"

# Comment out this line if you don't want to see warnings if a venv doesn't exist
#SHOW_WARNING=1

# Checks if given virtual env exist, and updates variable if it doesn't
_check_venv() {
  if [[ -z "$venv_name" ]]; then
    [[ -n $SHOW_WARNING ]] && echo -e "\033[95mVenv not provided. Will use the default \033[93;1m$_default_venv\033[0m"
    venv_name=$_default_venv
  fi
}

penv() {
  local venv_name
  venv_name="$1"
  _check_venv

  # Create the virtual environment
  python3 -m venv "$venv_name"
}

# Source a Python virtual environment
psrc() {
  local venv_name
  venv_name="$1"
  _check_venv

  # Check if the virtual environment exists
  if [[ ! -d "$venv_name" ]]; then
    echo -e "\033[93mVirtual environment \033[1m$_default_venv\033[0;93m doesn't exist\033[0m"
    return
  fi

  # Source the virtual environment
  source "$venv_name/bin/activate"
}

# Install requirements
_default_requirements="requirements.txt"
pinst() {
  local requirements_file
  requirements_file="$1"

  if [[ -z "$requirements_file" ]]; then
    [[ -n $SHOW_WARNING ]] && echo -e "\033[95mRequirements not provided. Will use the default \033[93;1m$_default_requirements\033[0m"
    requirements_file=$_default_requirements
  fi

  # Install requirements
  pip install -r "$requirements_file"
}

# Deactivate
alias pdeact="deactivate"

