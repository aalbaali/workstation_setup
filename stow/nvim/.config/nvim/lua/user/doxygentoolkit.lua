vim.g.DoxygenToolkit_paramTag_pre = "@param[in] "
-- Doxygen comment
vim.keymap.set("n", vim.g.altleader .. "dd", ":Dox<CR>")

-- Inline class member comment
vim.keymap.set("n", vim.g.altleader .. "ds", "O/**<space><space>*/<Esc>F<space>i")

-- Inline comment
vim.keymap.set("n", vim.g.altleader .. "dc", "O//<space>")
