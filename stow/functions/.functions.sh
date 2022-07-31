# @brief Generic functions and aliases used by bash and zsh (i.e., they're sourced)
# @author Amro Al-Baali

################################################
# Software abbreviations
################################################
alias fd="fdfind"
alias vsc="code -n ."
alias jj=julia
alias jfranklin="julia -e 'using Franklin; serve()'"
alias jpluto="julia -e 'using Pluto; Pluto.run()'"

# File sizes
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
alias ja='ninja'
alias ctest='ctest --output-on-failure'
alias cm='cmake -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=RelWithDebInfo'
alias cmd='cmake -GNinja -DCMAKE_EXPORT_COMPILE_COMMANDS=On -DCMAKE_BUILD_TYPE=Debug'
alias a="apt-cache search '' | sort | cut --delimiter ' ' --fields 1 | fzf --multi --cycle --reverse --preview 'apt-cache show {1}' | xargs -r sudo apt install -y"

# Make and change into a directory
mkcd()
{
  mkdir -p -- "$1" &&
  cd -P -- "$1"
}

# Log CPU and memory usage of a process
logpid() { while sleep 1; do  ps -p $1 -o pcpu= -o pmem= ; done; }

####################
# Git
####################
alias gll="git log --graph --oneline --all --decorate"
alias gs="git status"
alias gm="git commit -s"
alias ga="git add"
alias gb="git branch"
alias gc="git checkout"
alias gd="git diff"
alias gdc="git diff --cached"


# Fuzzy checkout git branch with fzf
gz() 
{
  local branches branch
  branches=$(git branch --all | grep -v HEAD) &&
  branch=$(echo "$branches" |
           fzf-tmux -d $(( 2 + $(wc -l <<< "$branches") )) +m) &&
  git checkout $(echo "$branch" | sed "s/.* //" | sed "s#remotes/[^/]*/##")
}

####################
# Docker
####################
alias dps="docker ps"
alias dpsa="docker ps -a"
alias di="docker image"
alias dils="docker image ls"
alias drm="docker rm"
alias drmi="docker rmi"

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
