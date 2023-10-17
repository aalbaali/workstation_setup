#!/bin/bash

# Update nodejs if it's older than version 16

# Check if NodeJS installed
! command -v node &> /dev/null && _should_update_node=1

# Node major version to target
NODE_MAJOR=16

# Check node version
if [[ -z "$_should_update_node" ]]; then
  target_version=$NODE_MAJOR
  # Check if node version is under target version
  if [[ $(node -v | cut -d "." -f 1 | cut -d "v" -f 2) -lt $target_version ]]; then
    echo "Current node version: $(node -v) is less than $target_version. Will update"
    _should_remove_older_node=1
    _should_update_node=1
  fi
fi

if [[ -n "_should_remove_older_node" ]]; then
  echo "Removing older version"
  sudo apt-get remove -y nodejs
  sudo apt-get remove -y $(dpkg -l | grep libnode | awk '{print $2}')
fi

if [[ -n "$_should_update_node" ]]; then
  echo "Installing nodejs version 16"
  sudo apt-get update -y
  sudo apt-get install -y ca-certificates curl gnupg

  sudo mkdir -p /etc/apt/keyrings
  sudo rm -rf /etc/apt/keyrings/nodesource.gpg
  curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | sudo gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg

  echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | sudo tee /etc/apt/sources.list.d/nodesource.list

  sudo apt-get update -y
  sudo apt-get install nodejs -y
fi
