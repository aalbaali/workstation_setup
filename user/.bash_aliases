title() {
  ORIG=$PS1
  TITLE="\e]2;$@\a"
  PS1=${ORIG}${TITLE}
}

zipall() {
  for dir in *
  do
    if [[ -d $dir ]]
    then
      zip -r $dir.zip $dir
    fi
  done
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

########################
# git shortcus
########################
if [ -f $HOME/.aliases/git_aliases.sh ]; then
  source $HOME/.aliases/git_aliases.sh
fi

alias gll="git log --graph --oneline --all --decorate"
alias gs="git status"
alias gm="git commit"
alias ga="git add"
alias gb="git branch"
alias gc="git checkout"

alias vsc="code -n ."

########################
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


# Open tmux after a restart. It creates a new session then deletes it.
alias mux='pgrep -vx tmux > /dev/null && \
		tmux new -d -s delete-me && \
		tmux run-shell ~/.tmux/plugins/tmux-resurrect/scripts/restore.sh && \
		tmux kill-session -t delete-me && \
		tmux attach || tmux attach'


################################################################################
# Docker
################################################################################
alias dps="docker ps"
alias dpsa="docker ps -a"
alias di="docker image"
alias dils="docker image ls"
alias drm="docker rm"
alias drmi="docker rmi"
