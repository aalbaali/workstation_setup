# Do not continue sourcing this file if ROS is not installed
if [[ -z $ROS_VERSION && -z $ROS_DISTRO ]] then
  return
fi

# ROS workspace
export ROS_WORKSPACE="${ROS_WORKSPACE:=${HOME}/ros_ws}"
if [ ! -e ${ROS_WORKSPACE} ]; then
  echo "WARNING: ROS_WS '${ROS_WORKSPACE}' does not exist"
fi
export ROS_UNDERLAY_WS="/opt/ros/$ROS_DISTRO"

# Change to ros workspace
alias cdr="cd ${ROS_WORKSPACE}"
alias cds="cd ${ROS_WORKSPACE}/src"
alias source_underlay="source $ROS_UNDERLAY_WS/setup.zsh"

# Rosdeup UPDATE existing packages
alias rupdate="rosdep update"

# Rosdep INSTALL necessary packages
alias rinstall="rosdep install -i --from-path $ROS_WORKSPACE/src --rosdistro $ROS_DISTRO -y"

# (Ros) Colcon BUILD with merge install
rcbuild() {
  cd $ROS_WORKSPACE
  colcon build --merge-install --symlink-install --cmake-args '-DCMAKE_BUILD_TYPE=RelWithDebInfo' '-DCMAKE_EXPORT_COMPILE_COMMANDS=On' -Wall -Wextra -Wpedantic
  cd -
}

# Ros Update and Colcon BUILD packages
rucbuild() {
  rupdate
  rcbuild
}


# Source overlay
if [[ -n "${ROS_INSTALLATION_DIRNAME}" ]]; then
  alias source_overlay="source ${ROS_WORKSPACE}/${ROS_INSTALLATION_DIRNAME}/setup.zsh"
elif [[ $ROS_VERSION -eq 1 ]]; then
  alias source_overlay="source $ROS_WORKSPACE/devel/setup.zsh"
else
  alias source_overlay="source $ROS_WORKSPACE/install/setup.zsh"
fi

if [[ $ROS_VERSION -eq 2 ]]; then
  # https://github.com/ros2/ros2cli/issues/534#issuecomment-957516107
  # argcomplete for ros2 & colcon
  eval "$(register-python-argcomplete3 ros2)"
  eval "$(register-python-argcomplete3 colcon)"
fi


# Ros local
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP

# Source overlay and set domain ID to communicate with Turtlebot4
source_underlay
source_overlay
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
