-- If set to true, then uses the commit hash instead of the branch name.
vim.g.gh_use_canonical = 1

vim.g.gh_line_map_default = 0
vim.g.gh_line_blame_map_default = 1
vim.g.gh_open_command = 'fn() { echo "$@" | xclip -selection clipboard; }; fn '
