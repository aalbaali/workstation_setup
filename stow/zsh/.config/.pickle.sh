export TOKENS_PATH="/home/amro/src/pickle/TOKENS.sh"
[[ -f "${TOKENS_PATH}" ]] && source "${TOKENS_PATH}"

alias j=jira

export LOGDNA_KEY=b43d4937f6e8055a2fb21e4758a3ea0d

# ROS workspace in rosbridge
export ROS_WORKSPACE="${HOME}/catkin_ws"
export ROS_INSTALLATION_DIRNAME="devel_isolated"

#######################################
# Files to source
#######################################
[[ -f ~/.picklerc ]] && source ~/.picklerc
[[ -f ~/dev_profile.sh ]] && zsh ~/dev_profile.sh


#######################################
# Functions
#######################################
function open-jira-issue() {
  """
  Open Pickle Jira issue in Google Chrome browser.

  Usage:
    open_jira_issue <issue_num>

  Args:
    issue_num: Jira issue number
    url: Jira issue URL (e.g., https://picklerobot.atlassian.net/browse)
  """

  local issue_num="$1"
  local website="${url}/${issue_num}"
  google-chrome "${website}" &>/dev/null &
}

function open-pickle-jira-issue-from-clipboard() {
  """
  Open Pickle Jira issue in Google Chrome browser, where the issue number is obtained from the
  clipboard.

  Usage:
    # Copy Jira issue number
    open_pickle_jira_issue
  """

  local issue_num="$(xclip -selection clipboard -o)"
  local url="https://picklerobot.atlassian.net/browse"
  open-jira-issue "${issue_num}" "${url}"
}

function open-dill-file() {
  """
  Open a file in the Dill github repository.

  Usage:
    open-dill-file <filepath>

  Args:
    filepath: File path relative to the dill repository
  """

  local filepath="$1"
  local url="https://github.com/Pickle-Robot/dill/blob/main/${filepath}"
  google-chrome "${url}" &>/dev/null &
}

function open-dill-file-from-clipboard() {
  local filepath="$(xclip -selection clipboard -o)"
  open-dill-file "${filepath}"
}

#######################################
# Key bindings
#######################################
# Bind opening Pickle Jira issue
bindkey -s '^[i' 'open-pickle-jira-issue-from-clipboard^M'
bindkey -s '^[g' 'open-dill-file-from-clipboard^M'


#######################################
# Dev container setup
#######################################
[[ ! "${DILL_DEVCONTAINER}" = "true" ]] && return

# Add scripts to binary dir
export PATH=${PROJDIR}/scripts:${HOME}/bin:${PATH}
