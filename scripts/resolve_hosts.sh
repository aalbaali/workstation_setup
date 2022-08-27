#!/bin/sh

# Check if local host exists in hosts
if [ -z "$(cat /etc/hosts | grep -i "$HOSTNAME")" ]; then
  # Prepend hostname
  echo "127.0.0.1       $HOSTNAME\n$(cat /etc/hosts)" > /etc/hosts
fi
