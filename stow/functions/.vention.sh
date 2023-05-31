# Export vention environment variables
export TOKEN_FILE="~/vention/TOKEN.txt"
export GITHUB_TOKEN=$(cat ~/vention/TOKEN.txt)

# Copy token to clipboard
alias cptoken="cat ~/vention/TOKEN.txt | xclip -selection clipboard"

# Always use local docker env vars
export DOCKER_MULTI_INSTANCE_RUN_SERVER_ENVIRONMENT=LOCAL
