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

function _create_new_tmux_window_and_send_keys() {
  # Create a new tmux window, change working directory,
  # and send keys (without executing them)
  session="$1"
  window="$2"
  wd="$3"      # Working directory
  cmd="$4"     # Command to send

  _create_new_tmux_window $session "$window"
  [[ -n "$wd" ]] && tmux send-keys -t "$session:$window" "cd $wd" Enter
  [[ -n "$cmd" ]] && tmux send-keys -t "$session:$window" "$cmd"
}

# Default session name
DEFAULT_VENTION_SESSION_NAME="ven-run"

function vention_ses_kill() {
  session="$1"
  if [[ -z "$session" ]]; then
    session="$DEFAULT_VENTION_SESSION_NAME"
    log_info "No session name provided. Using default '$session'"
  fi
  if ! _does_tmux_session_exist "$session"; then
    log_info "Attempting to delete a non-existing session. Ignoring."
    return 1
  fi
  tmux kill-ses -t "$session"
}

# Run Vention packages in tmux
function vention_ses_start() {
  session="$1"
  should_kill="$2"
  if [[ -z "$session" ]]; then
    session="$DEFAULT_VENTION_SESSION_NAME"
    log_info "No session name provided. Using default '$session'"
  fi

  if _does_tmux_session_exist "$session"; then
    if [[ -n "$should_kill" ]]; then
      vention_ses_kill "$session"
    else
      log_error "Sesssion '$session' already exists. Aborting"
      return 1
    fi
  fi
  log_info "Session '$session' doesn't exist. It'll be created"
  first_window=1

  # Rails
  _create_new_tmux_window_and_send_keys "$session" \
    "Rails" \
    "~/vention/multirepo-build-system/vention_rails" \
    "npm run start:vention-rails-run"

  # Assembler
  _create_new_tmux_window_and_send_keys "$session" \
    "Asmblr" \
    "~/vention/multirepo-build-system/vention_assembler/client" \
    "npm run dev"

  # ML
  _create_new_tmux_window_and_send_keys "$session" \
     "ML" \
    "~/vention/multirepo-build-system/vention_assembler/client/vse" \
    "git submodule init && git submodule update"

  # EE
  _create_new_tmux_window_and_send_keys "$session" \
     "EE" \
    "~/vention/multirepo-build-system/mm-execution-engine" \
    "npm run simulate-cad"

  # ML-robot
  _create_new_tmux_window_and_send_keys "$session" \
     "ML-R" \
    "~/vention/multirepo-build-system/mm-machine-logic-robot" \
    "npm run dev"

  # RMSA
  _create_new_tmux_window_and_send_keys "$session" \
     "RMSA" \
    "~/vention/multirepo-build-system/robot-motion-simulation-apis" \
    "npm run build:watch"

  # DMIRS
  _create_new_tmux_window_and_send_keys "$session" \
     "DMIRS" \
    "~/vention/multirepo-build-system/docker-multi-instance-run-server" \
    "npm run start-simulate"

  # Vention-ROS
  _create_new_tmux_window_and_send_keys "$session" \
     "ROS" \
    "~/vention/multirepo-build-system/vention_ros" \
    "npm run simulate-cad"
}

function vention_ses_restart() {
  session="$1"
  if [[ -z "$session" ]]; then
    session="$DEFAULT_VENTION_SESSION_NAME"
    log_info "No session name provided. Using default '$session'"
  fi
  vention_ses_kill $session
  vention_ses_start "$session"
}
