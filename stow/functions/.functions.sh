# @brief Generic functions and aliases used by bash and zsh (i.e., they're sourced)
# @author Amro Al-Baali

####################
# Docker
####################
[[ $(docker --help 2>/dev/null) ]] && [ -f ~/.dockerfunctions.sh ] && source ~/.dockerfunctions.sh

####################
# ROS
####################
[[ -n $ROS_DISTRO ]] && [ -f ~/.rosfunctions.sh ] && source ~/.rosfunctions.sh

################################################
# Application aliases
################################################
alias fd="fdfind"
alias vsc="code -n ."
alias jj=julia
alias jfranklin="julia -e 'using Franklin; serve()'"
alias jpluto="julia -e 'using Pluto; Pluto.run()'"
alias vpdf="sioyek"
alias sz=ncdu
alias nv=nvim

################################################
# System-related aliases and functions
################################################
alias ls='ls --color=auto'
alias ll='ls -alF'
alias l='ls -CF'
alias ta='tmux a -t'
alias now='watch -x -t -n 0.01 date +%s.%N' 
alias o=xdg-open
alias k='k -h'
alias cdg='cd "$(git rev-parse --show-cdup)"'
alias cds='cd "$(git rev-parse --show-superproject-working-tree)"'
alias cdd='cd ~/case/common/shared/data'
alias cdsh='cd ~/case/common/shared'
alias ja='ninja'
alias ctest='ctest --output-on-failure'
alias cm='cmake -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=RelWithDebInfo'
alias cmd='cmake -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=Debug'
alias a="apt-cache search '' | sort | cut --delimiter ' ' --fields 1 | fzf --multi --cycle --reverse --preview 'apt-cache show {1}' | xargs -r sudo apt install -y"

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

####################
# Git
####################
alias gll="git log --graph --oneline --decorate"
alias gla="git log --graph --oneline --all --decorate"
alias gs="git status -s"
alias gm="git commit -s"
alias ga="git add"
alias gb="git branch"
alias gc="git checkout"
alias gd="git diff"
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
# Zip
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

####################
# Tmux
####################
# Open tmux after a restart. It creates a new session then deletes it.
alias mux='pgrep -vx tmux > /dev/null && \
		tmux new -d -s delete-me && \
		tmux run-shell ~/.tmux/plugins/tmux-resurrect/scripts/restore.sh && \
		tmux kill-session -t delete-me && \
		tmux attach || tmux attach'
