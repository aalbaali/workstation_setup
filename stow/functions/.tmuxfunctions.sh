#!/bin/env bash

function _does_tmux_session_exist() {
  session_name="$1"

  if tmux has-session -t "$session_name" 2>/dev/null; then
    return 0 # true
  fi
  return 1 # false
}

function _create_new_tmux_window() {
  session="$1"
  window="$2"
  command="$3"
  log_debug "Adding window '$window' to session '$session' with command '$command'"

  if [[ "$first_window" -eq 1 ]]; then
    tmux new -s "$session" -n "$window" -d
    first_window=0
  else
    tmux new-window -d -t "$session" -n "$window"
  fi
  tmux send-keys -t "$session:$window" "$command"
}
