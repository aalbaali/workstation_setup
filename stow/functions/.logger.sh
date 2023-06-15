#!/bin/env bash

# Bash logger functions
#
# Amro Al-Baali

# Set default log level
[[ -z $LOG_LEVEL ]] && LOG_LEVEL=1

function _log() {
  name="$1"  # E.g., DEBUG
  color="$2" # E.g., 93
  echo -n -e "[\033[$color;1m$name\033[0m]\033[${color}m "
  echo ${@:3}
}

function log_error() {
  # Logs if env variable 'ERROR' is set to a non-zero value
  if [[ -z "$LOG_INFO" ]]; then
    [[ -z $LOG_LEVEL ]] && return 0
    [[ $LOG_LEVEL -lt 1 ]] && return 0
  fi
  _log "ERROR" "91" $@
}

function log_info() {
  # Logs if env variable 'INFO' is set to a non-zero value
  if [[ -z "$LOG_INFO" ]]; then
    [[ -z $LOG_LEVEL ]] && return 0
    [[ $LOG_LEVEL -lt 2 ]] && return 0
  fi
  _log "INFO" "96" $@
}

function log_debug() {
  # Logs if env variable 'DEBUG' is set to a non-zero value
  if [[ -z "$LOG_INFO" ]]; then
    [[ -z $LOG_LEVEL ]] && return 0
    [[ $LOG_LEVEL -lt 3 ]] && return 0
  fi
  _log "DEBUG" "93" $@
}

