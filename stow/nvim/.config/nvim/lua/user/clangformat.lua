vim.cmd [[
  autocmd FileType c,cpp,objc nnoremap <buffer> vim.g.altleader .. cf :<C-u>ClangFormat<CR>
  autocmd FileType c,cpp,objc vnoremap <buffer> vim.g.altleader .. cf :ClangFormat<CR>
  autocmd FileType c,cpp,objc nnoremap <silent> == V:ClangFormat<CR>
]]
