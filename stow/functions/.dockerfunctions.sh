# Docker-related functions for starting containers

alias dps="docker ps"
alias dpsa="docker ps -a"
alias di="docker image"
alias ds="docker start"
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
  # $1: Container name
  # $2-*: Arguments passed to the container
  # Start container if not started
  if [ $(docker container inspect -f '{{.State.Status}}' $1) != "running" ]; then
    echo -e "\033[93mContainer \033[36;1m$1\033[0;93m not running. Will start it\033[0m"
    ds $1;
  fi
  docker exec -it $1 ${@:2}
}

# Complete `dex` with a list of running containers
#   1. Get all docker *running* containers (i.e., witout the `-a` flag)
#   2. Get all rows after the first row (FNR > 1)
#   3. Print last column ($(NF))
complete -C "dn" dex


drun() {
  # First argument: image repo
  # Second argument: image tag
  # Third argument: container name

  if [ $# -lt 3 ]; then
    echo -e "\033[91mOnly two arguments passed. Should pass at least 3 arguments:\033[0m"
    echo "   \$1  : image repo"
    echo "   \$2  : image tag"
    echo "   \$3  : container name"
    echo "   \$4-*: arguments passed to \`docker run\`"
    return 1
  fi

  # First argument is the image repo:tag. Will use the tag as the container's hostname.
  # tag=$(echo "$1" | awk '{split($0,a,":"); print(a[2])}')
  tag=$2

  docker run -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.ssh:/home/cpp/.ssh \
    -v ~/.zsh_history:/home/cpp/.zsh_history  \
    --network host \
    --hostname $tag \
    --name "$3" \
    ${@:4} \
    "$1:$2"
}


__add_complete_commands_drun() {
  # Completion type (https://unix.stackexchange.com/questions/166908/is-there-anyway-to-get-compreply-to-be-output-as-a-vertical-list-of-words-instea)
  COMP_TYPE=9 # 63 for <tab><tab>, or 9 for <tab>
  COMP_KEY=9 # 63 for <tab><tab>, or 9 for <tab>


  # First suggestion is the list of repos
  if [ $COMP_CWORD -eq 1 ]; then
    # First column of `docker image ls`. Display only unique non-dangling images
    repos=$(docker image ls --filter "dangling=false" | awk '(NR>1)' | awk '{print $1}' | sort -u)
    # Print suggestions line by line

    suggestions=( $(compgen -W "$repos" -- ${COMP_WORDS[1]}) )
    COMPREPLY=("${suggestions[@]}")

  elif [ $COMP_CWORD -eq 2 ]; then
    # List the tags for the given image repo
    tags=$(docker image ls --filter=reference="${COMP_WORDS[1]}" | awk '(NR>1)' | \
      awk '{print $2}' | sed '/<none>/d' | sort)

    suggestions=( $(compgen -W "$tags" -- ${COMP_WORDS[2]}) )
    COMPREPLY=("${suggestions[@]}")  
  else
    COMPREPLY=()
  fi
}

# Add autocomplete commands
complete -F __add_complete_commands_drun drun

