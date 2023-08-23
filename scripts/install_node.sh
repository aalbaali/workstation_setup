#!/bin/bash

# Update nodejs if it's older than version 16

# Check if NodeJS installed
! command -v node &> /dev/null && _should_update_node=1

# Check node version
if [[ -n "$_should_update_node" ]]; then
  echo "Installing nodejs version 16"
  curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -
  sudo apt-get update -y
  sudo apt-get install -y nodejs
else
  target_version=16
  # Check if node version is under 16
  if [[ $(node -v | cut -d "." -f 1 | cut -d "v" -f 2) -lt $target_version ]]; then
    echo "Current node version: $(node -v) is less than $target_version. Will update"
    _should_update_node=1
  fi
fi

if [[ -n "$_should_update_node" ]]; then
  echo "Installing nodejs version 16"
  curl -fsSL https://deb.nodesource.com/setup_16.x | sudo -E bash -
  sudo apt-get update -y
  sudo apt-get install -y nodejs
fi
