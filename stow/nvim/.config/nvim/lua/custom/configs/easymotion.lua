-- Explicitly use double-backslash operator for the plugin leader.
-- Don't use the <leader> key, as it may be set to be the <space> key
vim.keymap.set("n", vim.g.altleader .. vim.g.altleader , "<Plug>(easymotion-prefix)")
