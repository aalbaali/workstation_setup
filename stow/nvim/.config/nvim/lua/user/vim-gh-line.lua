-- If set to true, then uses the commit hash instead of the branch name.
vim.g.gh_use_canonical = 1

vim.g.gh_line_map_default = 0
vim.g.gh_line_blame_map_default = 1
vim.g.gh_open_command = 'fn() { echo "$@" | xclip -selection clipboard; }; fn '

--- Yank the Github remote URL to the clipboard.
---@param use_commit If true, then use the commit hash in the URL. Otherwise, use the branch name.
--            If nil, then use the value of `g:gh_use_canonical`.
function yank_gh_line(use_commit)
  -- Update the global variable only if an argument is passed.
  if use_commit ~= nil then
    -- Cast to int
    vim.g.gh_use_canonical = use_commit and 1 or 0
  end

  local keys = vim.api.nvim_replace_termcodes('<Plug>(gh-line)', true, false, true)
  vim.api.nvim_feedkeys(keys, 'n', false)
end

return {
  yank_gh_line = yank_gh_line
}
