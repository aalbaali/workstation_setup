#!/bin/sh
#
# Set up ROS network to use the robot's roscore or use a local roscore.
# Notes about setting up roscore using the robot's roscore:
#   1. This only works through an ethernet cable;
#   2. This only works after disabling the robot's firewall through `sudo ufw disable`;
#   3. This script needs to be sourced for every bash session that needs to access the robot's
#      roscore
#
# Flags
#  -l  Use local network
#  -r  Use robot network
#
# Example
#  source use_robot_ros.sh -r     # Uses robot network
#  source use_robot_ros.sh -l     # Uses local network

# Set starting `getopts` indices so that the flags work with `source`
OPTIND=1

while getopts "lrip" opt; do
  case $opt in
    l) # Local network
      echo "Using local network"
      export ROS_MASTER_URI=http://localhost:11311
      export ROS_HOSTNAME=localhost
      #export ROS_IP=192.168.3.101
      unset ROS_IP
      ;;
    i) # Local IP network
      echo "Using local IP network"

      # Local IP address stripped off whitespace
      IP=$(hostname -I | awk '{$1=$1;print}')
      export ROS_MASTER_URI="http://$IP:11311"
      export ROS_IP=$IP
      export ROS_HOSTNAME=$(hostname -s)
      ;;
    p) # Raspberry pi network
      echo "Using raspberry pi network"
      export ROS_MASTER_URI="http://172.20.2.29:11311"
      export ROS_HOSTNAME=avidbots
      export ROS_IP=172.20.2.29
      ;;
    r) # Robot network
      echo "Using robot network"
      export ROS_MASTER_URI=http://192.168.3.101:11311
      export ROS_HOSTNAME=172.20.0.96
      export ROS_IP=192.168.3.101
      ;;
    \?)
      echo "Invalid option: -$OPTARG" >&2
      exit 1
      ;;
  esac
done
