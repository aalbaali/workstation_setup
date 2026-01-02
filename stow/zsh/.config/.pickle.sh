export TOKENS_PATH="/home/amro/src/pickle/TOKENS.sh"
[[ -f "${TOKENS_PATH}" ]] && source "${TOKENS_PATH}"

alias j=jira
alias mytasks="jira issue list -a amro -s~Done"

alias dev="dex dill_devcontainer zsh"
alias pl=pickle-tui
bindkey -s '^[o' 'pickle-tui^M'

function get_rot() {
  r=${1}
  p=${2}
  y=${3}
  should_copy=${4:-0}
  echo "RPY: ${r} ${p} ${y} [deg]."
  echo "Should copy: ${should_copy}"
  output=$(euler -e -s xyz -p --ros -- ${1} ${2} ${3})
  if [[ ${should_copy} -eq 1 ]]; then
    echo "${output}" | tail -n 6 | xclip -selection clipboard
    echo "Copied quaternion to clipboard."
  fi
}


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

function open-dill-pr() {
  txt="${1}"
  # Filter for the number directly after # (e.g., #1234)
  pr_num=$(echo "$txt" | rg -o '#\d+' | head -n 1 | tr -d '#')
  repo="${2:-dill}"
  # Set url based on the repo, which could be roots or dill
  if [[ "${repo}" = "dill" ]]; then
    url="https://github.com/Pickle-Robot/dill/pull/"
  elif [[ "${repo}" = "roots" ]]; then
    url="https://github.com/Pickle-Robot/roots/pull/"
  fi

  echo "Opening PR: ${url}${pr_num}"
  google-chrome "${url}${pr_num}" &>/dev/null &
}


function open-dill-pr-from-clipboard() {
  # Open Dill PR in Google Chrome browser, where the PR number is obtained from the clipboard.
  #
  # Usage:
  #   open_dill_pr_from_clipboard
  for issue in $(xclip -selection clipboard -o | ag -o '#\d+'); do
    open-dill-pr "${issue}" "${2:-dill}"
  done

}
#######################################
# Key bindings
#######################################
# Bind opening Pickle Jira issue
bindkey -s '^[i' 'open-pickle-jira-issue-from-clipboard^M'
bindkey -s '^[d' 'open-dill-pr-from-clipboard^M'

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

function robot_fk() {
  # Run forward kinematics on the KR70 robot
  docker exec -it dill_devcontainer python -c \
"""
import numpy as np
from app.arm.kuka.kinematics.kinematics import make_kr70_kin
kin = make_kr70_kin(None)
tf=kin.forward($@)
print(tf)

np.set_printoptions(precision=3, suppress=True)
print(f'RPY: {np.rad2deg(tf.rpy)} [deg]')
print(f'Pos: {tf.position_np} [m]')
"""
}

function robot_ik() {
  # Run forward kinematics on the KR70 robot
  q=$(xclip -selection clipboard -o)
  docker exec -it dill_devcontainer python -c \
    """
import numpy as np
from app.shared.measurements import Transform
from app.arm.kuka.kinematics.kinematics import make_kr70_kin
kin = make_kr70_kin(None);
iks = kin.inverse(Transform.from_rpy_and_position(rpy=np.deg2rad([${1}, ${2}, ${3}]), position=np.array([${4}, ${5}, ${6}])))

np.set_printoptions(precision=3, suppress=True)
for idx, ik in enumerate(iks):
    print(f'q{idx}:', np.array2string(np.rad2deg(ik), separator=','), '[deg]');
"""
}



function robot_neutral_q() {
  echo '[92.4, -128.23, 129.17, 0.0, 89.07, 60.0]'
}

function robot_view_frames() {
  docker exec -it dill_devcontainer python /home/dill/pickle_data_v1/amro/pickle-sandbox/py_playground/view_robot_frames.py $@
}


# Function to delete git branches
delete-branch() {
    # Check for help option first
    if [[ "$1" == "-h" || "$1" == "--help" ]]; then
        echo "Usage: delete-branch [-f] [-l | -r] [--fzf] <branch-name> [<branch-name> ...]"
        echo "Delete git branches locally and/or remotely."
        echo ""
        echo "Options:"
        echo "  -f        Force delete (use -D instead of -d for local branches)"
        echo "  -l        Delete only local branches"
        echo "  -r        Delete only remote branches"
        echo "  --fzf     Use fzf to interactively select branches to delete"
        echo "  -h, --help    Show this help message"
        echo ""
        echo "Arguments:"
        echo "  <branch-name>    Name(s) of branch(es) to delete (ignored when --fzf is used)"
        echo ""
        echo "If neither -l nor -r is specified, both local and remote branches will be deleted."
        return 0
    fi

    local force_delete=false
    local delete_local=false
    local delete_remote=false
    local use_fzf=false

    # Parse options to handle combined flags like -lf or -rf
    while [[ "$1" == -* ]]; do
        case "$1" in
            --fzf)
                use_fzf=true
                shift
                ;;
            -*)
                # Handle combined options like -lf, -rf, etc.
                local opt="${1#-}"
                while [[ -n "$opt" ]]; do
                    case "${opt:0:1}" in
                        f)
                            force_delete=true
                            ;;
                        l)
                            delete_local=true
                            ;;
                        r)
                            delete_remote=true
                            ;;
                        *)
                            echo "Invalid option: -${opt:0:1}" >&2
                            return 1
                            ;;
                    esac
                    opt="${opt:1}"
                done
                shift
                ;;
        esac
    done

    # Default to deleting both local and remote if neither -l nor -r is specified
    if [ "$delete_local" = false ] && [ "$delete_remote" = false ]; then
        delete_local=true
        delete_remote=true
    fi

    local branches_to_delete=()

    if [ "$use_fzf" = true ]; then
        # Use fzf to select branches
        local branches

        # Get the list of branches based on the specified options
        if [ "$delete_local" = true ]; then
            branches+=$(git branch --format='%(refname:short)' 2>/dev/null)$'\n'
        fi
        if [ "$delete_remote" = true ]; then
            branches+=$(git branch -r --format='%(refname:short)' 2>/dev/null)$'\n'
        fi

        # Use fzf to select branches to delete
        branches_to_delete=($(echo "$branches" | grep -v HEAD | fzf -m --prompt="Select branches to delete: "))

        if [ ${#branches_to_delete[@]} -eq 0 ]; then
            echo "No branches selected for deletion."
            return 0
        fi

        # Show selected branches and ask for confirmation
        echo "The following branches will be deleted:"
        for branch in "${branches_to_delete[@]}"; do
            echo "  - $branch"
        done
        echo ""

        local delete_type=""
        if [ "$delete_local" = true ] && [ "$delete_remote" = true ]; then
            delete_type="locally and remotely"
        elif [ "$delete_local" = true ]; then
            delete_type="locally"
        elif [ "$delete_remote" = true ]; then
            delete_type="remotely"
        fi

        if [ "$force_delete" = true ]; then
            echo "Branches will be force deleted $delete_type."
        else
            echo "Branches will be deleted $delete_type."
        fi

        read -p "Are you sure you want to proceed? (y/N): " -n 1 -r
        echo ""
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Deletion cancelled."
            return 0
        fi
    else
        # Use command line arguments
        if [ $# -eq 0 ]; then
            echo "Usage: delete-branch [-f] [-l | -r] [--fzf] <branch-name> [<branch-name> ...]"
            return 1
        fi
        branches_to_delete=("$@")
    fi

    for branch in "${branches_to_delete[@]}"; do
        # Strip origin/ prefix from remote branch names if present
        local clean_branch_name="${branch#origin/}"

        if [ "$delete_local" = true ]; then
            if [ "$force_delete" = true ]; then
                # Force delete local branch
                git branch -D "$clean_branch_name"
            else
                # Delete local branch
                git branch -d "$clean_branch_name"
            fi
        fi

        if [ "$delete_remote" = true ]; then
            # Check if the branch exists on the remote before attempting to delete it
            if git show-ref --verify --quiet refs/remotes/origin/"$clean_branch_name"; then
                # Delete remote branch
                git push origin --delete "$clean_branch_name" --no-verify
            else
                echo "Branch '$clean_branch_name' does not exist on the remote."
            fi
        fi
    done
}

# Bash completion for delete-branch function
_delete_branch_completions() {
    local cur prev opts branches
    local delete_local=false delete_remote=false

    # Get the current word being typed
    cur="${COMP_WORDS[COMP_CWORD]}"
    prev="${COMP_WORDS[COMP_CWORD-1]}"

    # Define available options
    opts="-f -l -r --fzf -h --help"

    # If current word starts with -, provide option completions
    if [[ "$cur" == -* ]]; then
        COMPREPLY=($(compgen -W "$opts" -- "$cur"))
        return 0
    fi

    # Parse existing arguments to determine what branches to show
    local i
    for (( i=1; i < COMP_CWORD; i++ )); do
        case "${COMP_WORDS[i]}" in
            -l)
                delete_local=true
                ;;
            -r)
                delete_remote=true
                ;;
            --fzf)
                # When --fzf is used, no branch name arguments are needed
                return 0
                ;;
        esac
    done

    # Default to both local and remote if neither -l nor -r is specified
    if [ "$delete_local" = false ] && [ "$delete_remote" = false ]; then
        delete_local=true
        delete_remote=true
    fi

    # Build branch list based on flags
    branches=""
    if [ "$delete_local" = true ]; then
        branches+=$(git branch --format='%(refname:short)' 2>/dev/null)$'\n'
    fi
    if [ "$delete_remote" = true ]; then
        # Get remote branches and strip the origin/ prefix
        branches+=$(git branch -r --format='%(refname:short)' 2>/dev/null | grep -v HEAD | sed 's#^origin/##')$'\n'
    fi

    # Provide completions for the current word
    COMPREPLY=($(compgen -W "$branches" -- "$cur"))
}

# Register the completion function for delete-branch
complete -F _delete_branch_completions delete-branch

# `rosbag` function run through rosbridge
function rbag() {
  bag_path=""
  if [ "$#" -ge 2 ]; then
    host_path=$(realpath "${2}" --relative-to="${HOME}/pickle_data_v1")
    bag_path="/app/${host_path}"
  fi

  docker run -it --rm --network=host -v ${HOME}/pickle_data_v1:/app rosbridge:latest bash -c "rosbag $1 ${bag_path} ${@:3}"
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
