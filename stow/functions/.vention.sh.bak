# Export vention environment variables
export TOKEN_PATH="$HOME/vention/TOKEN.txt"
export GITHUB_TOKEN=$(cat ~/vention/TOKEN.txt)

# Copy token to clipboard
alias cptoken="cat ~/vention/TOKEN.txt | xclip -selection clipboard"
bindkey -s '^[t' 'cptoken^M'


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

function tmux_new_window_and_send_keys() {
  # Create a new tmux window, change working directory,
  # and send keys (without executing them)
  session="$1"
  window="$2"
  wd="$3"      # Working directory
  cmd="$4"     # Command to send
  msg="$5"     # Message to display (optional)

  tmux_new_window $session "$window"
  [[ -n "$wd" ]]  && tmux send-keys -t "$session:$window" "cd $wd" Enter
  tmux send-keys -t "$session:$window" "cd $wd" Enter
  [[ -n "$msg" ]] && tmux send-keys -t "$session:$window" "echo -e \"$msg\"" Enter
  [[ -n "$cmd" ]] && tmux send-keys -t "$session:$window" "$cmd"
}

# Default session name
DEFAULT_VENTION_SESSION_NAME="vention-run-all"

function vention_ses_kill() {
  session="$1"
  if [[ -z "$session" ]]; then
    session="$DEFAULT_VENTION_SESSION_NAME"
    log_info "No session name provided. Using default '$session'"
  fi
  if ! tmux_does_session_exist "$session"; then
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

  if tmux_does_session_exist "$session"; then
    if [[ -n "$should_kill" ]]; then
      vention_ses_kill "$session"
    else
      log_error "Sesssion '$session' already exists. Aborting"
      return 1
    fi
  fi
  log_info "Session '$session' doesn't exist. It'll be created"
  first_window=1

  # MRBS
  tmux_new_window_and_send_keys "$session" \
    "MRBS" \
    "~/vention/multirepo-build-system" \
    "npm run start"

  # Rails
  tmux_new_window_and_send_keys "$session" \
    "Rails" \
    "~/vention/multirepo-build-system/vention_rails" \
    "npm run start:vention-rails-run" \
    "To kill the docker containers, run
      \033[93;1mdocker ps -a --format '{{.Names}}' | grep 'docker-' | xargs docker stop\033[0m
    "

  # TODO: split commands into two splits
  # Pendant
  tmux_new_window_and_send_keys "$session" \
    "Pendant" \
    "~/vention/multirepo-build-system/mm-pendant" \
    "npm run start && npm run dev:watch"

  # Assembler
  tmux_new_window_and_send_keys "$session" \
    "Asmblr" \
    "~/vention/multirepo-build-system/vention_assembler/client" \
    "npm run dev"

  # ML
  tmux_new_window_and_send_keys "$session" \
     "ML" \
    "~/vention/multirepo-build-system/vention_assembler/client/vse" \
    "git submodule init && git submodule update"

  # EE
  tmux_new_window_and_send_keys "$session" \
     "EE" \
    "~/vention/multirepo-build-system/mm-execution-engine" \
    "npm run simulate-cad"

  # ML-robot
  tmux_new_window_and_send_keys "$session" \
     "ML-R" \
    "~/vention/multirepo-build-system/mm-machine-logic-robot" \
    "npm run dev"

  # RMSA
  tmux_new_window_and_send_keys "$session" \
     "RMSA" \
    "~/vention/multirepo-build-system/robot-motion-simulation-apis" \
    "npm run build:watch"

  # DMIRS
  tmux_new_window_and_send_keys "$session" \
     "DMIRS" \
    "~/vention/multirepo-build-system/docker-multi-instance-run-server" \
    "npm run start-simulate"

  # Vention-ROS
  tmux_new_window_and_send_keys "$session" \
     "ROS" \
    "~/vention/multirepo-build-system/vention_ros"
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


# Function combining that 'vention_ses_*' functions, and is controlled using flags
function vention_ses() {
  case "$1" in
    --start|-s)
      log_info "Starting tmux sessions"
      vention_ses_start ${@:2}
      ;;
    --kill|-k)
      log_info "Killing tmux session"
      vention_ses_kill ${@:2}
      ;;
    --restart|-r)
      log_info "Restarting tmux session"
      vention_ses_restart ${@:2}
      ;;
    --help|-h)
      echo "Valid flags:"
      echo "  --start, -s   Start tmux vention session"
      echo "  --kill, -k    Kill tmux vention session"
      echo "  --restart, -r (default) restart tmux vention session"
      ;;
  esac
}

alias vs="vention_ses"
alias vss="vention_ses -s"
alias vsk="vention_ses -k"
alias vsr="vention_ses -r"

#alias mmqa="ssh machine-motion -t /usr/local/bin/mmqa"
