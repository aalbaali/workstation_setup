" Note that this file is sourced from within `lua/user/headerguard.lua`

" Get the tail of a path `path`, where it's split of at the string `chopstr`. For example,
"  PathTail('a/b/c/d/e/f', 'd') returns 'e/f'
function! g:PathTail(path, chopstr)
  return substitute(a:path, '.*\/' . a:chopstr . '\/', '', '')
endfunction

" Strip off `include`, and everything before it from a path. This is used to generate the path
" for a header guard
function! g:ChopInclude(string)
  return PathTail(a:string, 'include')
endfunction

" G++ Google standard compliant header guard.
" The function `HeaderGuardName` is used by the `drmikehenry/vim-headerguard` package to generate
" the header guard names.
" This solution is inspired by the issue: https://github.com/drmikehenry/vim-headerguard/issues/2
"
" Explanation of the `substitute` command:
"  Arg 1: Get current file path `%:p` and chop the `include` subpath and everything before it
"  Arg 2: Replace all chars that satisfy the regex expression `[^0-9a-zA-Z_]` (i.e., anything that
"         is NOT (`^`) a number (`0-9`), a lower case character (`a-z`), an upper case character
"         (`A-Z`), or a subscript (`_`))
"  Arg 3: The string to replace the chars with
"  Arg 3: `g` flag passed to `substitute` to replace ALL occurrences. Without this flag, only the
"         first occurrence will be replaced
function! g:HeaderguardName()
  return toupper(substitute(ChopInclude(expand('%:p')), '[^0-9a-zA-Z_]', '_', 'g')) . '_'
endfunction
