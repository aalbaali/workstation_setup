-- Load session if `(n)vim` is invoked in a directory that contains a `Session.vim` file
vim.g.startify_session_autoload = 0
-- Automatically update session before leaving (i.e., closing) vim
vim.g.startify_session_persistence = 0
-- Bookmarked directories/files
vim.g.startify_bookmarks = {
		'~/Dev/workstation_setup/' }

-- Open startify buffer
vim.keymap.set("n", vim.g.altleader .. "ss", ":Startify<cr>")
