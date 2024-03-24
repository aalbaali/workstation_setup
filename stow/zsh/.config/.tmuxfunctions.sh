#!/bin/env bash

function tmux_does_session_exist() {
  # Check if a tmux session exists
  session_name="$1"

  if tmux has-session -t "$session_name" 2>/dev/null; then
    return 0 # true
  fi
  return 1 # false
}

function tmux_new_window() {
  # Create a new tmux window in a given session.
  # If the session doesn't exist, then it'll be created
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

tmux_kill_using_fzf () {
    # Fzf popup to kill tmux sessions
    # https://github.com/junegunn/fzf/wiki/Examples#tmux
    local sessions
    sessions="$(tmux ls|fzf --exit-0 --multi)"  || return $?
    local i
    for i in "${(f@)sessions}"
    do
        [[ $i =~ '([^:]*):.*' ]] && {
            echo "Killing $match[1]"
            tmux kill-session -t "$match[1]"
        }
    done
}

# Tmux kill ses
alias tks='tmux_kill_using_fzf'


# tm - create new tmux session, or switch to existing one. Works from within tmux too. (@bag-man)
# `tm` will allow you to select your tmux session via fzf.
# `tm irc` will attach to the irc session (if it exists), else it will create it.
tmux_switch_ses() {
  [[ -n "$TMUX" ]] && change="switch-client" || change="attach-session"
  if [ $1 ]; then
    tmux $change -t "$1" 2>/dev/null || (tmux new-session -d -s $1 && tmux $change -t "$1"); return
  fi
  session=$(tmux list-sessions -F "#{session_name}" 2>/dev/null | fzf --exit-0) &&  tmux $change -t "$session" || echo "No sessions found."
}
alias ts=tmux_switch_ses

tmux_switch_pane() {
  # Switch pane (@george-b)
  local panes current_window current_pane target target_window target_pane
  panes=$(tmux list-panes -s -F '#I:#P - #{pane_current_path} #{pane_current_command}')
  current_pane=$(tmux display-message -p '#I:#P')
  current_window=$(tmux display-message -p '#I')

  target=$(echo "$panes" | grep -v "$current_pane" | fzf +m --reverse) || return

  target_window=$(echo $target | awk 'BEGIN{FS=":|-"} {print$1}')
  target_pane=$(echo $target | awk 'BEGIN{FS=":|-"} {print$2}' | cut -c 1)

  if [[ $current_window -eq $target_window ]]; then
    tmux select-pane -t ${target_window}.${target_pane}
  else
    tmux select-pane -t ${target_window}.${target_pane} &&
    tmux select-window -t $target_window
  fi
}
