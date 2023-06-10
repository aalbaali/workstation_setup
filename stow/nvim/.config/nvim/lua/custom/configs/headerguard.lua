-- The headerguard function needs to be visible from vimscript `call` command. As such,
-- defining the `HeaderguardName()` function as a Lua function is not enough (the 
-- function won't be visible to vimscript's `:call` function). Instead, we define a
-- vimscript function and source it from Lua.
vim.cmd("source ~/.config/nvim/vim/headerguard.vim")

-- Mapping to generate guards
vim.keymap.set("n", vim.g.altleader .. "hg", ":HeaderguardAdd<cr>")

-- Use `//` style comments
vim.g.headerguard_use_cpp_comments = 1
