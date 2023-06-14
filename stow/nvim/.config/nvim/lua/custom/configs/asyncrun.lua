-- Catkin builds
vim.g.asyncrun_open = 4

local function OnAsyncRunFinished()
  if vim.g.asyncrun_status == 'success' then
    vim.defer_fn(function() vim.cmd('cclose') end, 1000)
  else
    vim.cmd('copen 20')
  end
end

vim.g.asyncrun_exit = "call OnAsyncRunFinished()"

