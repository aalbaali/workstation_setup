# Export vention environment variables
export TOKEN_PATH="$HOME/vention/TOKEN.txt"
export GITHUB_TOKEN=$(cat ~/vention/TOKEN.txt)

# Copy token to clipboard
alias cptoken="cat ~/vention/TOKEN.txt | xclip -selection clipboard"

# Always use local docker env vars
#export DOCKER_MULTI_INSTANCE_RUN_SERVER_ENVIRONMENT=LOCAL

# Find ROS services
fdsrv() {
  rosservice list | grep -i "$1"
}

srvtype() {
  local srvname
  srvname="$(cat -)"

  # Get the service type
  rosservice info "$srvname" | grep "^Type" | awk -F':' '{print $2}'
}

# Source asdf (required for vention Rails)
source "$HOME/.asdf/asdf.sh"

alias jmrbs="cd ~/vention/multirepo-build-system"
