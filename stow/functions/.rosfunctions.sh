# Ros workspace
export ROS_WORKSPACE="/home/ros/ros_ws"

# Change to ros workspace
alias cdr="cd $ROS_WORKSPACE"

# Rosdeup UPDATE existing packages
alias rupdate="rosdep update"

# Rosdep INSTALL necessary packages
alias rinstall="rosdep install -i --from-path $ROS_WORKSPACE/src --rosdistro $ROS_DISTRO -y"

rinstallpackage() {
  rosdep install -i --from-path $1 --rosdistro $ROS_DISTRO -y ${@:2}
}

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
alias sro="source $ROS_WORKSPACE/install/setup.zsh"

# https://github.com/ros2/ros2cli/issues/534#issuecomment-957516107
# argcomplete for ros2 & colcon
#[ -x $(command -v register-python-argcomplete3) 2>/dev/null ] && register-python-argcomplete3 ros2
#[ -x $(command -v register-python-argcomplete3) 2>/dev/null ] && register-python-argcomplete3 colcon
