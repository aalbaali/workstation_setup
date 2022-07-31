#!/bin/bash

# @brief A script for installing docker
# @details This scrip is based off Docker's official documentation page
#           https://docs.docker.com/engine/install/ubuntu/
# 			   Requires sudo privilages
# @author Amro Al-Baali

apt-get update

apt-get install -y \
  ca-certificates \
  curl \
  gnupg \
  lsb-release

# Add docker’s official GPG key:
mkdir -p /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg

# Set up the repo
echo \
 "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
 $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install docker engine
apt-get update
apt-get install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin

apt-cache madison docker-ce

echo -e "\033[92mUsing one of the above options run
		\033[93;1msudo apt-get install docker-ce=<VERSION_STRING> docker-ce-cli=<VERSION_STRING> containerd.io docker-compose-plugin\033[0;92m
    where <VERSOIN_STRING> is replaced with the version string from the second column, for example, 5:20.10.16~3-0~ubuntu-jammy.
    \033[0m"

echo -e "\033[92mVerify docker is working by running
		\033[93;1msudo docker run hello-world\033[0m"

echo -e "\033[92mTo resolve docker permission errors, check the following post
          https://stackoverflow.com/questions/48957195/how-to-fix-docker-got-permission-denied-issue\033[0m"
