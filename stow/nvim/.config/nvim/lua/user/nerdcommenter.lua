-- Create default mappings
vim.g.NERDCreateDefaultMappings = 0
-- Align line-wise comment delimiters flush left instead of following code indentation
vim.g.NERDDefaultAlign = 'left'
vim.keymap.set("i", "<C-_>", "<esc><Plug>NERDCommenterToggle<cr>")

-- Default mappings. Check NERDCommenter Github page for more info
vim.keymap.set({"n", "v"}, vim.g.altleader .. "cc", "<Plug>NERDCommenterComment")
vim.keymap.set({"n", "v"}, vim.g.altleader .. "cu", "<Plug>NERDCommenterUncomment")
vim.keymap.set({"n", "v"}, vim.g.altleader .. "ci", "<Plug>NERDCommenterInvert")
vim.keymap.set({"n", "v"}, vim.g.altleader .. "cs", "<Plug>NERDCommenterSexy")
vim.keymap.set({"n", "v"}, vim.g.altleader .. "cy", "<Plug>NERDCommenterYank")
vim.keymap.set({"n", "v"}, vim.g.altleader .. "c$", "<Plug>NERDCommenterToEOL")

