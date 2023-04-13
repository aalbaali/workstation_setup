# Get rosbag topics
# $1: bag name
gettopics() { rosbag info -k topics "$1" -y | ag topic: | awk -F': ' '{print $2}'}


# Ros workspace
export ROS_WORKSPACE="/home/ros/ros_ws"

# Change to ros workspace
alias cdr="cd $ROS_WORKSPACE"
alias cdi="cd $ROS_WORKSPACE"
alias cds="cd $ROS_WORKSPACE/src"

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
if [[ $ROS_VERSION -eq 1 ]] then
  alias sro="source $ROS_WORKSPACE/devel/setup.zsh"
else
  alias sro="source $ROS_WORKSPACE/install/setup.zsh"
fi


# https://github.com/ros2/ros2cli/issues/534#issuecomment-957516107
# argcomplete for ros2 & colcon
eval "$(register-python-argcomplete3 ros2)"
eval "$(register-python-argcomplete3 colcon)"


# Ros local
export ROS_MASTER_URI=http://localhost:11311
export ROS_HOSTNAME=localhost
unset ROS_IP

