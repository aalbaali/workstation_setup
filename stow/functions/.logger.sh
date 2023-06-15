#!/bin/env bash

# Bash logger functions
#
# Amro Al-Baali

function _log() {
  name="$1"  # E.g., DEBUG
  color="$2" # E.g., 93
  [[ -z "$(echo $name)" ]] && return 0
  echo -n -e "[\033[$color;1m$name\033[0m]\033[${color}m "
  [[ -n "$(echo $name)" ]] && echo ${@:3}
}

function log_error() {
  # Logs if env variable 'ERROR' is set to a non-zero value
  _log "ERROR" "91" $@
}

function log_debug() {
  # Logs if env variable 'DEBUG' is set to a non-zero value
  _log "DEBUG" "93" $@
}

function log_info() {
  # Logs if env variable 'INFO' is set to a non-zero value
  _log "INFO" "96" $@
}

