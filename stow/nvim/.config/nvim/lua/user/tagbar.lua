local status_ok, tagbar = pcall(require, "tagbar")
if not status_ok then
	return
end

vim.keymap.set("n", vim.g.altleader .. "tt", "<cmd>TagbarToggle f<CR>")
vim.keymap.set("n", vim.g.altleader .. "to", "<cmd>TagbarOpen f<CR>")
vim.keymap.set("n", vim.g.altleader .. "tc", "<cmd>TagbarClose<CR>")
vim.keymap.set("n", vim.g.altleader .. "tn", "<cmd>TagbarJumpNext<CR>")
vim.keymap.set("n", vim.g.altleader .. "tN", "<cmd>TagbarJumpPrev<CR>")
vim.g.tagbar_autoclose = 1

