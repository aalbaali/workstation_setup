export TOKENS_PATH="/home/amro/src/pickle/TOKENS.sh"
[[ -f "${TOKENS_PATH}" ]] && source "${TOKENS_PATH}"

alias j=jira
alias mytasks="jira issue list -a amro -s~Done"

alias dev="dex dill_devcontainer zsh"


export LOGDNA_KEY=b43d4937f6e8055a2fb21e4758a3ea0d

# ROS workspace in rosbridge
if [ -e "${HOME}/catkin_ws" ]; then
  export ROS_WORKSPACE="${HOME}/catkin_ws"
  export ROS_INSTALLATION_DIRNAME="devel_isolated"
fi

#######################################
# Files to source
#######################################
safe_source "${HOME}/.picklerc"
safe_source "${HOME}/pickle_data_v1/amro/pickle-sandbox/pickle_configs/robot_info.bash"


#######################################
# Functions
#######################################
function open-jira-issue() {
  # Open Pickle Jira issue in Google Chrome browser.
  #
  # Usage:
  #   open_jira_issue <issue_num>
  #
  # Args:
  #   issue_num: Jira issue number
  #   url: Jira issue URL (e.g., https://picklerobot.atlassian.net/browse)

  # Match the issue number from any string
  #local issue_num=`echo ${1} | ag -o '\w*-\d+' -m 1 | tr '[:lower:]' '[:upper:]'`
  local issue_num="${1}"
  local website="${url}/${issue_num}"
  google-chrome "${website}" &>/dev/null &
}

function open-pickle-jira-issue-from-clipboard() {
  # Open Pickle Jira issue in Google Chrome browser, where the issue number is obtained from the
  # clipboard.
  #
  # Usage:
  #   # Copy Jira issue number
  #   open_pickle_jira_issue

  local url="https://picklerobot.atlassian.net/browse"

  # Parse multiple issues from clipboard
  for issue in $(xclip -selection clipboard -o | ag -o '\w*-\d+' | tr '[:lower:]' '[:upper:]'); do
    open-jira-issue "${issue}" "${url}"
  done
  #open-jira-issue "${issue_num}" "${url}"
}

function gsutil-download() {
  """
  Take a glob pattern of a bag in GCloud, prompt the user to select the bag to download, and
  download it to the current directory.

  Args:
    pattern: Glob pattern of the bag in GCloud
    download_dir: Directory to download the bag to (default: current directory)
  """
  local pattern="$1"
  local download_dir="${2:-${PWD}}"
  #local path="${2:-${PWD}}"
  #local path="."
  gcloud_bag=$(gsutil ls "${pattern}" | fzf)
  bag_name=$(basename "${gcloud_bag}")
  if [[ -z "${gcloud_bag}" ]]; then
    echo "No bag selected. Exiting."
    return
  fi
  echo "Downloading bag to ${download_dir}/${bag_name}"
  gsutil cp "${gcloud_bag}" "${download_dir}/${bag_name}"
}

#######################################
# Key bindings
#######################################
# Bind opening Pickle Jira issue
bindkey -s '^[i' 'open-pickle-jira-issue-from-clipboard^M'

#######################################
# Parse navigation inputs/output
#######################################
function parse_nav_plotter_params() {
  text="$1"
  should_yank="$2"
  should_debug="$3"
  # Bounds
  bounds=$(echo "$text" | rg "MobileBaseBoundsKey\.(\w+)" -o -r '$1' | tr '[:upper:]' '[:lower:]')

  # Planner
  planner_txt=$(echo "$text" | rg "(\w*)+ line plan initial conditions" -o -r '$1')
  if [ "$planner_txt" = "Straight" ]; then
      planner="StraightLinePlanner"
  elif [ "$planner_txt" = "Global" ]; then
      planner="GlobalPlanner"
  fi

  # Initial conditions
  initial_conditions_txt=$(echo "$text" | rg "Straight line plan initial conditions: x0=\[([^\]]*)\]" -o -r '$1')
  initial_conditions=$(echo "$initial_conditions_txt" | rg '(-?\d+\.\d+)' -o -r '$1' | head -n 3 | tr '\n' ' ')
  width=$(echo "$text" | rg 'initial.*width: (-?\d+\.\d+)' -o -r '$1')

  # Final conditions
  final_conditions_txt=$(echo "$text" | rg "Straight line plan final conditions: xf=\[([^\]]*)\]" -o -r '$1')
  final_conditions=$(echo "$final_conditions_txt" | rg '(-?\d+\.\d+)' -o -r '$1' | head -n 3 | tr '\n' ' ')

  if [[ "$should_debug" ]]; then
    echo "Bounds: $bounds"
    echo "Planner: $planner"
    echo "Initial conditions: $initial_conditions"
    echo "Final conditions: $final_conditions"
    echo "Width: $width\n"
  fi

  # Output
  output="--width $width -t$planner -b$bounds $initial_conditions $final_conditions"
  if [[ "$should_yank" ]]; then
    echo $output | xclip -selection clipboard
  fi
  echo $output
}
#######################################
# Dev container setup
#######################################
[[ ! "${DILL_DEVCONTAINER}" = "true" ]] && return

# Add scripts to binary dir
export PATH=${PROJDIR}/scripts:${HOME}/bin:${PATH}

# Start MB apps in sim
alias start-sim="start --apps motor_controller navigation scan_perception amr_localization"
alias start-safety="start --apps scan_perception safety_interface"
