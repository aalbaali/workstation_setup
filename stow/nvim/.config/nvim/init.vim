" ======================================
" install vim-plug if it isn't installed
" ======================================
if empty(glob('~/.vim/autoload/plug.vim'))
  silent !curl -fLo ~/.vim/autoload/plug.vim --create-dirs
     \ https://raw.githubusercontent.com/junegunn/vim-plug/master/plug.vim
  autocmd VimEnter * PlugInstall --sync | source $MYVIMRC
endif

" ======================================
" Install plugins
" ======================================
call plug#begin('~/.config/nvim/plugged')

Plug 'morhetz/gruvbox'
Plug 'jremmen/vim-ripgrep'
Plug 'tpope/vim-fugitive'             " Git comments
Plug 'leafgarland/typescript-vim'
Plug 'vim-utils/vim-man'              " View `man` pages in vim
Plug 'mrtazz/DoxygenToolkit.vim'      " Auto-insert Doxygen comments
Plug 'tpope/vim-commentary'           " Easily comment / uncomment blocks
Plug 'skywind3000/asyncrun.vim'       " Run commands / builds in background 
Plug 'christoomey/vim-tmux-navigator' " Seamless navigation between vim and tmux
Plug 'sheerun/vim-polyglot'           " Better syntax highlighting
Plug 'w0ng/vim-hybrid'                " Colorscheme
Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }  " Install fzf (fast fuzzy searcher)
Plug 'junegunn/fzf.vim'               " fzf vim extension
Plug 'airblade/vim-gitgutter'         " Git status on side bar and git operations
Plug 'rhysd/vim-clang-format'         " Commands for applying clang-formatting
Plug 'preservim/nerdtree'             " Navigate files using a tree structure

Plug 'arcticicestudio/nord-vim'       " Build for vim's terminal and GUI mode with true colors
Plug 'tmsvg/pear-tree'                " Pair brackets, braces, etc.
Plug 'jamestthompson3/nvim-remote-containers'  " similar to vscode remote container
Plug 'neoclide/coc.nvim', {'branch': 'v0.0.81'} " Autocompletion


" CMake support
if has('nvim-0.5+' )
  Plug 'cdelledonne/vim-cmake'
  Plug 'lyuts/vim-rtags'
else
  Plug 'cdelledonne/vim-cmake', {'branch': 'v0.7.1'}
endif

call plug#end()

" ======================================
" Settings
" ======================================
syntax on                   " Sets syntax highlighting to on
set noerrorbells            " Turns off error sounds/bells after end of line
set tabstop=2               " Tabstop: 2 characters long
set softtabstop=2           " Tabstop: 2 spaces long
set shiftwidth=2            " Indent next line (after hitting 'enter') by 2 spaces
set expandtab               " Converts tabs to white space
set smartindent             " Attempts smart indentation (auto indent) for different languages
set nu                      " Set line numbering
set nowrap                  " Do not wrap
set noswapfile              " Do not swap files when writing in vim
set nobackup                " No backup file. Instead, use undodir/undofile.
set undodir=~/.nvim/undodir " Undo directory for 'backup'
set incsearch               " Incremental search
set colorcolumn=100         " Color column at this limit
highlight ColorColumn ctermbg=0 guibg=lightblue
set ruler                   " Show row and column
set splitright              " New verticalsplits go to the right
set splitbelow              " New horizontal splits go below
set ignorecase              " Ignore case when using a search pattern
set smartcase               " Override 'ignorecase' when pattern has upper case character
set number                  " Show line numbers
set wildmode=longest,list   " Get bash-like tab completions
set showmatch               " Show matching brackets.
set nohlsearch              " Highlight search results
set autoindent              " Indent a new line the same amount as the line just typed
set cc=100                  " Set an 100 column border for good coding style

" colorscheme
colorscheme gruvbox
set background=dark

filetype plugin indent on   " allows auto-indenting depending on file type

" allows rg to always to detect root and use .gitignore for faster searching
if executable('rg')
  let g:rg_derive_root='true'
endif

" Never automatically continue comment when starting next line and 
" delete comment character when joining commented lines
au FileType * set fo-=c fo-=r fo-=o fo+=j

" Set C++ comment strings to `//` instead of `/* */`
autocmd FileType c,cpp setlocal commentstring=//\ %s
autocmd FileType json syntax match Comment +\/\/.\+$+

" ======================================
" Plugin settings and shortcuts
" ======================================
" Copy to clipboard. Note: requires `vim-gtk` (install using `sudo apt-get install vim-gtk`)
vnoremap <leader>y "+y
nnoremap <leader>yy V"+y

" Variables
let g:cmake_link_complie_commands = 1
let g:cmake_default_config = 'build'

" CMake shortcuts
nmap <leader>cg : CMakeGenerate<cr>
nmap <leader>cb : CMakeBuild<cr>

" -- fzf
nnoremap <leader>o :Files<CR>
nnoremap <leader>i :Buffers<CR>
nnoremap <leader>/ :Lines<CR>
nnoremap <leader>a :Ag<space>
nnoremap <leader>p :History<CR>
nnoremap <leader>: :History:<CR>
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

nnoremap <leader>ff :call FZFSameName('e', '', '')<CR>
nnoremap <leader>fh :call FZFSameName('e', 'wincmd h', '')<CR>
nnoremap <leader>fl :call FZFSameName('e', 'wincmd l', '')<CR>
nnoremap <leader>fk :call FZFSameName('e', 'wincmd k', '')<CR>
nnoremap <leader>fj :call FZFSameName('e', 'wincmd j', '')<CR>
nnoremap <leader>fH :call FZFSameName('leftabove vsplit', '', 'wincmd h')<CR>
nnoremap <leader>fL :call FZFSameName('rightbelow vsplit', '', 'wincmd l')<CR>
nnoremap <leader>fK :call FZFSameName('leftabove split', '', 'wincmd k')<CR>
nnoremap <leader>fJ :call FZFSameName('rightbelow split', '', 'wincmd j')<CR>

" vim-gutter
function! GitStatus()
  let [a,m,r] = GitGutterGetHunkSummary()
  return printf('+%d ~%d -%d', a, m, r)
endfunction
set statusline+=%{GitStatus()}

" vim-clang-format
autocmd FileType c,cpp,objc nnoremap <buffer><Leader>cf :<C-u>ClangFormat<CR>
autocmd FileType c,cpp,objc vnoremap <buffer><Leader>cf :ClangFormat<CR>

" nerdtree mappings
nnoremap <leader>n :NERDTreeFocus<CR>
nnoremap <C-n> :NERDTree<CR>
nnoremap <C-t> :NERDTreeToggle<CR>
nnoremap <C-f> :NERDTreeFind<CR><C-w>w
let NERDTreeMapActivateNode='<space>'

" -- vim-fugitive
nnoremap <leader>gb :Git blame<CR>

" coc
" Set internal encoding of vim, not needed on neovim, since coc.nvim using some
" unicode characters in the file autoload/float.vim
set encoding=utf-8

" TextEdit might fail if hidden is not set.
set hidden

" Some servers have issues with backup files, see #649.
set nobackup
set nowritebackup

" Give more space for displaying messages.
set cmdheight=2

" Having longer updatetime (default is 4000 ms = 4 s) leads to noticeable
" delays and poor user experience.
set updatetime=300

" Don't pass messages to |ins-completion-menu|.
set shortmess+=c

" Always show the signcolumn, otherwise it would shift the text each time
" diagnostics appear/become resolved.
if has("nvim-0.5.0") || has("patch-8.1.1564")
  " Recently vim can merge signcolumn and number column into one
  set signcolumn=number
else
  set signcolumn=yes
endif

" Use tab for trigger completion with characters ahead and navigate.
" NOTE: Use command ':verbose imap <tab>' to make sure tab is not mapped by
" other plugin before putting this into your config.
inoremap <silent><expr> <Tab>
      \ pumvisible() ? "\<C-n>" :
      \ <SID>check_back_space() ? "\<Tab>" :
      \ coc#refresh()
inoremap <expr><S-TAB> pumvisible() ? "\<C-p>" : "\<C-h>"

function! s:check_back_space() abort
  let col = col('.') - 1
  return !col || getline('.')[col - 1]  =~ '\s'
endfunction

" Use <c-space> to trigger completion.
if has('nvim')
  inoremap <silent><expr> <c-space> coc#refresh()
else
  inoremap <silent><expr> <c-@> coc#refresh()
endif

" Make <CR> auto-select the first completion item and notify coc.nvim to
" format on enter, <cr> could be remapped by other vim plugin
" inoremap <silent><expr> <cr> pumvisible() ? coc#_select_confirm()
"                               \: "\<C-g>u\<CR>\<c-r>=coc#on_enter()\<CR>"
inoremap <expr> <cr> pumvisible() ? "\<C-y>" : "\<C-g>u\<CR>"

" Use `[g` and `]g` to navigate diagnostics
" Use `:CocDiagnostics` to get all diagnostics of current buffer in location list.
nmap <silent> [g <Plug>(coc-diagnostic-prev)
nmap <silent> ]g <Plug>(coc-diagnostic-next)

" GoTo code navigation.
nmap <silent> gd <Plug>(coc-definition)
nmap <silent> gD :vsp<CR><Plug>(coc-definition)
nmap <silent> gy <Plug>(coc-type-definition)
nmap <silent> gi <Plug>(coc-implementation)
nmap <silent> gr <Plug>(coc-references)
nmap <silent> gh <C-u>:CocCommand clangd.switchSourceHeader<cr>
nmap <silent> gH :vsp<CR><C-u>:CocCommand clangd.switchSourceHeader<cr>

" Use K to show documentation in preview window.
nnoremap <silent> K :call ShowDocumentation()<CR>

function! ShowDocumentation()
  if CocAction('hasProvider', 'hover')
    call CocActionAsync('doHover')
  else
    call feedkeys('K', 'in')
  endif
endfunction

" Highlight the symbol and its references when holding the cursor.
autocmd CursorHold * silent call CocActionAsync('highlight')

" Symbol renaming.
nmap <leader>rn <Plug>(coc-rename)

" Formatting selected code.
xmap <leader>f  <Plug>(coc-format-selected)
nmap <leader>f  <Plug>(coc-format-selected)

augroup mygroup
  autocmd!
  " Setup formatexpr specified filetype(s).
  autocmd FileType typescript,json setl formatexpr=CocAction('formatSelected')
  " Update signature help on jump placeholder.
  autocmd User CocJumpPlaceholder call CocActionAsync('showSignatureHelp')
augroup end

" Applying codeAction to the selected region.
" Example: `<leader>aap` for current paragraph
" xmap <leader>a  <Plug>(coc-codeaction-selected)
" nmap <leader>a  <Plug>(coc-codeaction-selected)

" " Remap keys for applying codeAction to the current buffer.
" nmap <leader>ac  <Plug>(coc-codeaction)
" Apply AutoFix to problem on the current line.
nmap <leader>qf  <Plug>(coc-fix-current)

" Run the Code Lens action on the current line.
nmap <leader>cl  <Plug>(coc-codelens-action)

" Map function and class text objects
" NOTE: Requires 'textDocument.documentSymbol' support from the language server.
xmap if <Plug>(coc-funcobj-i)
omap if <Plug>(coc-funcobj-i)
xmap af <Plug>(coc-funcobj-a)
omap af <Plug>(coc-funcobj-a)
xmap ic <Plug>(coc-classobj-i)
omap ic <Plug>(coc-classobj-i)
xmap ac <Plug>(coc-classobj-a)
omap ac <Plug>(coc-classobj-a)

" Remap <C-f> and <C-b> for scroll float windows/popups.
if has('nvim-0.4.0') || has('patch-8.2.0750')
  " nnoremap <silent><nowait><expr> <C-f> coc#float#has_scroll() ? coc#float#scroll(1) : "\<C-f>"
  nnoremap <silent><noait><expr> <C-b> coc#float#has_scroll() ? coc#float#scroll(0) : "\<C-b>"
  inoremap <silent><nowait><expr> <C-f> coc#float#has_scroll() ? "\<c-r>=coc#float#scroll(1)\<cr>" : "\<Right>"
  inoremap <silent><nowait><expr> <C-b> coc#float#has_scroll() ? "\<c-r>=coc#float#scroll(0)\<cr>" : "\<Left>"
  " vnoremap <silent><nowait><expr> <C-f> coc#float#has_scroll() ? coc#float#scroll(1) : "\<C-f>"
  vnoremap <silent><nowait><expr> <C-b> coc#float#has_scroll() ? coc#float#scroll(0) : "\<C-b>"
endif

" Use CTRL-S for selections ranges.
" Requires 'textDocument/selectionRange' support of language server.
nmap <silent> <C-s> <Plug>(coc-range-select)
xmap <silent> <C-s> <Plug>(coc-range-select)

" Add `:Format` command to format current buffer.
command! -nargs=0 Format :call CocActionAsync('format')

" Add `:Fold` command to fold current buffer.
command! -nargs=? Fold :call     CocAction('fold', <f-args>)

" Add `:OR` command for organize imports of the current buffer.
command! -nargs=0 OR   :call     CocActionAsync('runCommand', 'editor.action.organizeImport')

" Add (Neo)Vim's native statusline support.
" NOTE: Please see `:h coc-status` for integrations with external plugins that
" provide custom statusline: lightline.vim, vim-airline.
set statusline^=%{coc#status()}%{get(b:,'coc_current_function','')}

" Mappings for CoCList
" Show all diagnostics.
nnoremap <silent><nowait> <space>a  :<C-u>CocList diagnostics<cr>
" Manage extensions.
nnoremap <silent><nowait> <space>e  :<C-u>CocList extensions<cr>
" Show commands.
nnoremap <silent><nowait> <space>c  :<C-u>CocList commands<cr>
" Find symbol of current document.
nnoremap <silent><nowait> <space>o  :<C-u>CocList outline<cr>
" Search workspace symbols.
nnoremap <silent><nowait> <space>s  :<C-u>CocList -I symbols<cr>
" Do default action for next item.
nnoremap <silent><nowait> <space>j  :<C-u>CocNext<CR>
" Do default action for previous item.
nnoremap <silent><nowait> <space>k  :<C-u>CocPrev<CR>
" Resume latest coc list.
nnoremap <silent><nowait> <space>p  :<C-u>CocListResume<CR>w

