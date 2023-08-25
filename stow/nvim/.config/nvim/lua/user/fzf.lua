vim.keymap.set("n", vim.g.altleader .. "o", ":Files<CR>")
vim.keymap.set("n", vim.g.altleader .. "i", ":Buffers<CR>")
vim.keymap.set("n", vim.g.altleader .. "l", ":BLines<CR>")
vim.keymap.set("n", vim.g.altleader .. "L", ":Lines<CR>")
vim.keymap.set("n", vim.g.altleader .. "a", ":Ag<space>")
vim.keymap.set("n", vim.g.altleader .. "p", ":History<CR>")
vim.keymap.set("n", vim.g.altleader .. ":", ":History:<CR>")
vim.keymap.set("n", vim.g.altleader .. "/", ":History/<CR>")
vim.keymap.set("n", vim.g.altleader .. "w", ":Windows<CR>")
vim.keymap.set("n", vim.g.altleader .. "ts", ":BTags<CR>")

-- Call commands for words under the curser
vim.keymap.set("n", vim.g.altleader .. "eo", ":call fzf#vim#files('.', {'options':'--query '.expand('<cword>')})<CR>")
vim.keymap.set("n", vim.g.altleader .. "eO", ":call fzf#vim#files('.', {'options':'--query '.expand('<cWORD>')})<CR>")
vim.keymap.set("n", vim.g.altleader .. "ei", ":call fzf#vim#buffers('.', {'options':'--query '.expand('<cword>')})<CR>")
vim.keymap.set("n", vim.g.altleader .. "eI", ":call fzf#vim#buffers('.', {'options':'--query '.expand('<cWORD>')})<CR>")
-- vim.keymap.set("n", vim.g.altleader .. "*", ":call fzf#vim#ag({'options':'--query '.expand('<cword>')})<CR>")
vim.keymap.set("n", vim.g.altleader .. "*", ":execute ':Ag ' . expand('<cword>')<CR>")

vim.cmd [[
	let $FZF_DEFAULT_COMMAND = 'ag -g ""' " ignore files in .gitignore
	let g:fzf_layout = { 'down': '~40%' }

	function! FZFSameName(sink, pre_command, post_command)
			let current_file_no_extension = expand("%:t:r")
			let current_file_with_extension = expand("%:t")
			execute a:pre_command
			call fzf#run(fzf#wrap({
						\ 'source': 'find -name "' . current_file_no_extension . '.*" | grep -Ev "*' . current_file_with_extension . '$"',
						\ 'options': -1, 'sink': a:sink}))
			execute a:post_command
	endfunction
]]

vim.keymap.set("n", vim.g.altleader .. "ff", ":call FZFSameName('e', '', '')<CR>")
vim.keymap.set("n", vim.g.altleader .. "fh", ":call FZFSameName('e', 'wincmd h', '')<CR>")
vim.keymap.set("n", vim.g.altleader .. "fl", ":call FZFSameName('e', 'wincmd l', '')<CR>")
vim.keymap.set("n", vim.g.altleader .. "fk", ":call FZFSameName('e', 'wincmd k', '')<CR>")
vim.keymap.set("n", vim.g.altleader .. "fj", ":call FZFSameName('e', 'wincmd j', '')<CR>")
vim.keymap.set("n", vim.g.altleader .. "fH", ":call FZFSameName('leftabove vsplit', '', 'wincmd h')<CR>")
vim.keymap.set("n", vim.g.altleader .. "fL", ":call FZFSameName('rightbelow vsplit', '', 'wincmd l')<CR>")
vim.keymap.set("n", vim.g.altleader .. "fK", ":call FZFSameName('leftabove split', '', 'wincmd k')<CR>")
vim.keymap.set("n", vim.g.altleader .. "fJ", ":call FZFSameName('rightbelow split', '', 'wincmd j')<CR>")

-- Search for name of current file ([s]earch [f]ile)
vim.keymap.set("n", vim.g.altleader .. "sf", ":Ag <C-R>=expand('%:t')<CR><CR>")

-- allows rg to always to detect root and use .gitignore for faster searching
if vim.fn.executable('rg') then
  vim.g.rg_derive_root = 'true'
end

