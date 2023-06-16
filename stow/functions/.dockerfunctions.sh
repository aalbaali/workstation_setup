# Docker-related functions for starting containers

# Do not source this file if docker is not installed
if ! command -v docker &> /dev/null; then
  return
fi

# Aliases
alias dps="docker ps"
alias dpsa="docker ps -a"
alias di="docker image"
alias ds="docker start"
alias dstop="docker stop"
alias dils="docker image ls"
alias drm="docker rm"
alias drmi="docker rmi"

# List running container names
dn() {
  docker ps $@ | awk 'FNR > 1 {print $(NF)}'
}

alias dna="dn -a"

# Stopping containers using the dn(a) aliases
alias dns="dn | xargs docker stop"
alias dnarm="dna | xargs docker rm"

# Execute already running containers
dex() {
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
#   1. Get all docker *running* containers
#   2. Get all rows after the first row (FNR > 1)
#   3. Print last column ($(NF))
complete -C "dna" dex


# Stop running containers
complete -C "dn" dstop


# Docker run with fully customizeable args
drun_full() {
  local image_repo="$1"
  local image_tag="$2"
  if [ $# -lt 5 ]; then
    echo -e "\033[93mLess than three arguments passed. Should pass at least three args:\033[0m"
    echo "   \$1  : image repo"
    echo "   \$2  : image tag"
    echo "   \$3  : container name"
    echo "   \$4  : container username"
    echo "   \$5  : container hostname"
    echo "   \$6-*: arguments passed to \`docker run\`"
    return 1
  fi
  local cont_name="$3"
  local cont_username="$4"
  local cont_hostname="$5"

  docker run -it \
    -e DISPLAY=$DISPLAY \
    -e DOCKER_REPO_TAG="$image_repo:$image_tag" \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.ssh:/home/$cont_username/.ssh \
    -v ~/.zsh_history:/home/$cont_username/.zsh_history  \
    -v ~/shared:/home/$cont_username/shared  \
    -v ~/.config/github-copilot:/home/$cont_username/.config/github-copilot/  \
    --network host \
    --hostname $cont_hostname \
    --add-host $cont_hostname:127.0.0.1 \
    --name $cont_name \
    ${@:6} \
    "$1:$2"
}

drun() {
  # First argument: image repo
  # Second argument: image tag
  # Third argument: container name

  local image_repo="$1"
  local image_tag="$2"
  if [ $# -lt 3 ]; then
    echo -e "\033[93mLess than three arguments passed. Should pass at least three args:\033[0m"
    echo "   \$1  : image repo"
    echo "   \$2  : image tag"
    echo "   \$3  : container name"
    echo "   \$4-*: arguments passed to \`docker run\`"
    return 1
  fi
  local cont_name="$3"

  # Container username is the image repo without the remote name (i.e., ignore string before '/')
  local cont_username=$(echo $image_repo | awk -F/ '{print $NF}')
  local cont_hostname=$(hostname)

  drun_full $1 $2 $cont_name $cont_username $cont_hostname ${@:4}
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

