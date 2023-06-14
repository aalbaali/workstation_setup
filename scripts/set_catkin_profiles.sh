#!/bin/bash

# Script written by Michael Smart from Avidbots
# Script obtained from https://avidbots.atlassian.net/l/c/pEvBF1ev

# Store current working directory
cwd=$PWD

#set -eu

# Script to set catkin profiles based on application
cdi

catkin config --profile slam-debug \
 --extend $ROS_ETC_DIR/../.. \
 --jobs 8 \
 --no-install \
 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DCMAKE_BUILD_TYPE=Debug \
              -DUnencryptedCleaningPlans=ON \
 --buildlist avidbots_msgs avidbots_slam*

catkin config --profile debug \
 --extend $ROS_ETC_DIR/../.. \
 --jobs 8 \
 --no-install \
 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DCMAKE_BUILD_TYPE=Debug \
              -DUnencryptedCleaningPlans=ON

catkin config --profile slam-relwdeb \
 --extend $ROS_ETC_DIR/../.. \
 --jobs 8 \
 --no-install \
 --buildlist avidbots_msgs avidbots_slam* \
 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DCMAKE_BUILD_TYPE=RelWithDebInfo \
              -DUnencryptedCleaningPlans=ON

catkin config --profile relwdeb \
 --extend $ROS_ETC_DIR/../.. \
 --jobs 8 \
 --no-install \
 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DCMAKE_BUILD_TYPE=RelWithDebInfo \
              -DUnencryptedCleaningPlans=ON

catkin config --profile release \
 --extend $ROS_ETC_DIR/../.. \
 --jobs 8 \
 --no-install \
 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DCMAKE_BUILD_TYPE=Release \
              -DUnencryptedCleaningPlans=ON

# Run a local install, and blacklist robot-only packages since they'll fail
catkin config --profile install_local_dev \
 --extend $ROS_ETC_DIR/../.. \
 --jobs 8 \
 --install \
 --blacklist avidbots_3d_sensor_systems \
             avidbots_systems_neo_1_0 \
             avidbots_systems_neo_2_0 \
 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DCMAKE_BUILD_TYPE=Release \
              -DUnencryptedCleaningPlans=ON

catkin config --profile install \
 --extend $ROS_ETC_DIR/../.. \
 --install \
 --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
              -DCMAKE_BUILD_TYPE=Release \
              -DUnencryptedCleaningPlans=ON

# Set profile to relwdebug
echo -e "\033[96mSetting catkin profile to \033[95;1mrelwdeb\033[0m\n"
catkin profile set slam-relwdeb


cd $cwd
