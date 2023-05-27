-- Catkin builds
vim.keymap.set("n", vim.g.altleader .. "kb", ":AsyncRun -cwd=<root> catkin build<CR>")
vim.keymap.set("n", vim.g.altleader .. "kt", ":AsyncRun -cwd=<root> catkin build --make-args tests<CR>")
vim.keymap.set("n", vim.g.altleader .. "kn", ":AsyncStop<CR>")
vim.keymap.set("n", vim.g.altleader .. "<space>", ":call asyncrun#quickfix_toggle(20)<CR>")
vim.g.asyncrun_open = 4

local function OnAsyncRunFinished()
  if vim.g.asyncrun_status == 'success' then
    vim.defer_fn(function() vim.cmd('cclose') end, 1000)
  else
    vim.cmd('copen 20')
  end
end

vim.g.asyncrun_exit = "call OnAsyncRunFinished()"

