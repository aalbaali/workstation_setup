export TOKENS_PATH="/home/amro/src/pickle/TOKENS.sh"
[[ -f "${TOKENS_PATH}" ]] && source "${TOKENS_PATH}"

alias j=jira

export LOGDNA_KEY=b43d4937f6e8055a2fb21e4758a3ea0d

#######################################
# Files to source
#######################################
[[ -f ~/.picklerc ]] && source ~/.picklerc
[[ -f ~/dev_profile.sh ]] && source ~/dev_profile.sh

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

#######################################
# Key bindings
#######################################
# Bind opening Pickle Jira issue
bindkey -s '^[i' 'open-pickle-jira-issue-from-clipboard^M'

[ "$USERNAME" != "dill" ] && return
export HOME=/home/dill
export PATH=${PROJDIR}/scripts:${HOME}/bin:${PATH}

export PROTO_HOME="$HOME/.proto"
export PATH="$PROTO_HOME/shims:$PROTO_HOME/bin:$PATH"

eval "$(proto completions --shell bash)"

eval "$(activate-global-python-argcomplete --dest=-)"

eval "$(register-python-argcomplete spice -e ${PROJDIR}/scripts/spice)"

# Register the entrypoint tab completer with scripts/start.
# The parser gets registered separately, because then when we try to do tab-completion, the parser
# runs on its own before the rest of the script does. This allows tab-completion to happen quickly
# even if, for example, scripts/start does nothing after arg parsing other than sleep forever.
eval "$(\
register-python-argcomplete \
--external-argcomplete-script "${PROJDIR}/process_manager/argcomplete_entrypoints/start_dill_entrypoint.py" \
start
)"
