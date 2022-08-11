# Docker-related functions for starting containers

drun() {
  # First argument: image repo
  # Second argument: image tag
  # Third argument: container name

  # First argument is the image repo:tag. Will use the tag as the container's hostname.
  # tag=$(echo "$1" | awk '{split($0,a,":"); print(a[2])}')
  tag=$2

  docker run -it \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v ~/.ssh/id_ed25519:/home/cpp/.ssh/id_ed25519 \
    -v ~/.zsh_history:/home/cpp/.zsh_history  \
    --network host \
    --hostname $tag \
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

