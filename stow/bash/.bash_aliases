# bash-specific functions and aliases

title() {
  ORIG=$PS1
  TITLE="\e]2;$@\a"
  PS1=${ORIG}${TITLE}
}

__bash_prompt() {
    local terminalpart='${debian_chroot:+($debian_chroot)}\[\033[01;32m\]\u@\h\[\033[00m\]:'
    local gitbranch='`export BRANCH=$(git describe --contains --all HEAD 2>/dev/null); \
        if [ "${BRANCH}" != "" ]; then \
            echo -n "\[\033[0;36m\](\[\033[1;31m\]${BRANCH}" \
            && if git ls-files --error-unmatch -m --directory --no-empty-directory -o --exclude-standard ":/*" > /dev/null 2>&1; then \
                    echo -n " \[\033[1;33m\]✗"; \
            fi \
            && echo -n "\[\033[0;36m\]) "; \
        fi`'
    local lightblue='\[\033[1;34m\]'
    local removecolor='\[\033[0m\]'
    PS1="${terminalpart}${lightblue}\w ${gitbranch}${removecolor}\$ "
    unset -f __bash_prompt
}
__bash_prompt

# Source bashrc
alias srb="source ~/.bashrc"

# workspaces
########################
if [ -d $HOME/.workspace ]; then
  for d in $HOME/.workspace/*/; do
    ws=$(basename $d)
    input='$1'
    num_input='$#'
    source /dev/stdin << EOF
function create_${ws}() {
  if [ ${num_input} -eq 0 ]; then
    echo "Usage: create_${ws} <name>";
    return 1;
  fi
  cp -r $d/. ${input};
};
EOF
  done
fi

########################
# Github
########################
if command -v gh &> /dev/null; then
  eval "$(gh completion -s bash)"
fi

########################
# ROS
########################
function noetic_gazebo() {
  docker run -it --rm -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix -u ros athackst/ros:noetic-gazebo gazebo
}

