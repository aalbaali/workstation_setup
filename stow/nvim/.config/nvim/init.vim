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
Plug 'skywind3000/asyncrun.vim'       " Run commands / builds in background
Plug 'w0rp/ale'                       " Asynchronous linting
Plug 'christoomey/vim-tmux-navigator' " Seamless navigation between vim and tmux
Plug 'sheerun/vim-polyglot'           " Better syntax highlighting
Plug 'w0ng/vim-hybrid'                " Colorscheme
Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }  " Install fzf (fast fuzzy searcher)
Plug 'junegunn/fzf.vim'               " fzf vim extension
Plug 'airblade/vim-gitgutter'         " Git status on side bar and git operations
Plug 'rhysd/vim-clang-format'         " Commands for applying clang-formatting
Plug 'preservim/nerdtree'             " Navigate files using a tree structure
Plug 'mhinz/vim-startify'             " Manage vim sessions
Plug 'vim-airline/vim-airline'        " Custom status bar
Plug 'vim-airline/vim-airline-themes' " Themes
Plug 'lervag/vimtex'                  " Latex support
Plug 'justinmk/vim-sneak'             " Fast navigation
Plug 'tpope/vim-surround'             " Operations for surrounding words with paranthesis
Plug 'cdelledonne/vim-cmake'          " CMake shortcuts
Plug 'chrisbra/csv.vim'               " Handle CSV files
Plug 'kkoomen/vim-doge'               " (Do)cument (Ge)nerator for various file systems
Plug 'preservim/tagbar'               " Browse tags of current file
Plug 'preservim/nerdcommenter'        " Commenting plugin
Plug 'nvim-lua/plenary.nvim'          " Asynchronous programming using coroutines used with diffview.nvim
Plug 'kyazdani42/nvim-web-devicons'   " Icons used for nvim diffview
Plug 'sindrets/diffview.nvim'         " Neovim diffview
Plug 'vim-scripts/AnsiEsc.vim'        " Ansi escape colors


Plug 'arcticicestudio/nord-vim'       " Build for vim's terminal and GUI mode with true colors
Plug 'tmsvg/pear-tree'                " Pair brackets, braces, etc.
Plug 'jamestthompson3/nvim-remote-containers'  " similar to vscode remote container
Plug 'neoclide/coc.nvim', {'branch': 'v0.0.81'} " Autocompletion

" CMake support
if has('nvim-0.5+' )
  Plug 'cdelledonne/vim-cmake'
  " Plug 'lyuts/vim-rtags'
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
set cursorline              " Highlight cursor line
set splitright
set diffopt+=vertical       " Set vertical split as the default split
set autoread                " Automatically read latest changes on a file
set spell spelllang=en_ca    " Set spelling correction language

" colorscheme
colorscheme gruvbox
set background=dark

filetype plugin indent on   " allows auto-indenting depending on file type

" allows rg to always to detect root and use .gitignore for faster searching
if executable('rg')
  let g:rg_derive_root='true'
endif

" Never automatically continue comment when starting next line and delete comment character
" when joining commented lines
au FileType * set fo-=c fo-=r fo-=o fo+=j

" Set C++ comment strings to `//` instead of `/* */`
autocmd FileType c,cpp setlocal commentstring=//\ %s
autocmd FileType json syntax match Comment +\/\/.\+$+

" ======================================
" Plugin settings and shortcuts
" ======================================
" Go to next buffer
map gn :bn<cr>
" Go to previous buffer
map gp :bp<cr>

" Go to file in vertical split
nnoremap <C-W><C-F> <C-W>vgf

" Close tab
nnoremap <C-W>C :tabclose<CR>

" Copy to clipboard. Note: requires `vim-gtk` (install using `sudo apt-get install vim-gtk`)
vnoremap <leader>y "+y
nnoremap <leader>yy V"+y
" Copy current file to clipboard
nmap <leader>Y :let @+ = expand("%:p")<cr>

" Variables
let g:cmake_link_complie_commands = 1
let g:cmake_default_config = 'build'

" ======================================
" vimdiff commands
" ======================================
nmap <buffer> dg :diffget
nmap <buffer> dp :diffput

" ======================================
" CMake shortcuts
" ======================================
nmap <leader>cg : CMakeGenerate<cr>
nmap <leader>cb : CMakeBuild<cr>

" ======================================
" Fzf
" ======================================
nnoremap <leader>o :Files<CR>
nnoremap <leader>i :Buffers<CR>
nnoremap <leader>l :BLines<CR>
nnoremap <leader>L :Lines<CR>
nnoremap <leader>a :Ag<space>
nnoremap <leader>p :History<CR>
nnoremap <leader>: :History:<CR>
nnoremap <leader>/ :History/<CR>
nnoremap <leader>w :Windows<CR>

" Call commands for words under the curser
nnoremap <leader>eo :call fzf#vim#files('.', {'options':'--query '.expand('<cword>')})<CR>
nnoremap <leader>eO :call fzf#vim#files('.', {'options':'--query '.expand('<cWORD>')})<CR>
nnoremap <leader>ei :call fzf#vim#buffers('.', {'options':'--query '.expand('<cword>')})<CR>
nnoremap <leader>eI :call fzf#vim#buffers('.', {'options':'--query '.expand('<cWORD>')})<CR>
" nnoremap <leader>* :call fzf#vim#ag({'options':'--query '.expand('<cword>')})<CR>
nnoremap <leader>* :execute ':Ag ' . expand('<cword>')<CR>

let $FZF_DEFAULT_COMMAND = 'ag --hidden --ignore .git -g ""' " ignore files in .gitignore
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

" ======================================
" Vim-gutter
" ======================================
function! GitStatus()
  let [a,m,r] = GitGutterGetHunkSummary()
  return printf('+%d ~%d -%d', a, m, r)
endfunction
set statusline+=%{GitStatus()}

" Git commands that are not necessarily part of vim-gutter
nmap <leader>gc :G commit -s<cr>
nmap <leader>gp :G push<cr>

" ======================================
" Vim-clang-format
" ======================================
autocmd FileType c,cpp,objc nnoremap <buffer><Leader>cf :<C-u>ClangFormat<CR>
autocmd FileType c,cpp,objc vnoremap <buffer><Leader>cf :ClangFormat<CR>

" ======================================
" Nerdtree mappings
" ======================================
nnoremap <leader>n :NERDTreeFocus<CR>
nnoremap <C-n> :NERDTree<CR>
nnoremap <C-t> :NERDTreeToggle<CR>
nnoremap <C-f> :NERDTreeFind<CR><C-w>w
let NERDTreeMapActivateNode='<space>'

" ======================================
" Vim-fugitive
" ======================================
nnoremap <leader>gb :Git blame<CR>
xnoremap <leader>gb :Git blame<CR>

" ======================================
" Coc (conquer of completion)
" ======================================
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
set signcolumn=yes

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
nmap <silent> [G <Plug>(coc-diagnostic-prev-error)
nmap <silent> ]G <Plug>(coc-diagnostic-next-error)

" GoTo code navigation.
nmap <silent> gd <Plug>(coc-definition)
nmap <silent> gD :vsp<CR><Plug>(coc-definition)
nmap <silent> gsD :sp<CR><Plug>(coc-definition)
nmap <silent> gy <Plug>(coc-type-definition)
nmap <silent> gi <Plug>(coc-implementation)
nmap <silent> gr <Plug>(coc-references)
nmap <silent> gh <C-u>:CocCommand clangd.switchSourceHeader<cr>
nmap <silent> gH :vsp<CR><C-u>:CocCommand clangd.switchSourceHeader<cr>
nmap <silent> gsH :sp<CR><C-u>:CocCommand clangd.switchSourceHeader<cr>

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
nmap <F2> <Plug>(coc-rename)

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
" Show function signature while in isert mode
inoremap <C-P> <C-\><C-O>:call CocActionAsync('showSignatureHelp')<cr>

" Global extensions to install
let g:coc_global_extensions = ['coc-json'    ,
                               \ 'coc-git'   ,
                               \ 'coc-pyright',
                               \ 'coc-cmake' ,
                               \ 'coc-clangd',
                               \ 'coc-ccls']

" ======================================
" Vim-startify
" ======================================
" Load session if `(n)vim` is invoked in a directory that contains a `Session.vim` file
let g:startify_session_autoload = 0
" Automaticall update session before leaving (i.e., closing) vim
let g:startify_session_persistence = 0
" Bookmarked directories/files
let g:startify_bookmarks = [ '~/.config/nvim/init.vim' ,
                          \ '~/Dev/workstation_setup/' ,
                          \ '~/Dev/repos/perception_tools/localization_rework_scripts/' ,
                          \ '~/Dev/repos/autonomy/' ,
                          \ '~/Dev/repos/localization-validation/']

" Open startify buffer
nnoremap <leader>s :Startify<cr>

" ======================================
" Vim-airline
" ======================================
" Display all buffers if a single tab is used
let g:airline#extensions#tabline#enabled = 1
" Customize sepraters
let g:airline#extensions#tabline#left_sep = ' '
let g:airline#extensions#tabline#left_alt_sep = '|'
" Set theme
let g:airline_theme='papercolor'
let g:airline#extensions#tagbar#enabled = 1

" ======================================
" Vimtex
" ======================================
" This is necessary for VimTeX to load properly. The "indent" is optional.
" Note that most plugin managers will do this automatically.
filetype plugin indent on

" This enables Vim's and neovim's syntax-related features. Without this, some
" VimTeX features will not work (see ":help vimtex-requirements" for more
" info).
syntax enable

" Viewer options: One may configure the viewer either by specifying a built-in
" viewer method:
let g:vimtex_view_method = 'general'

" Or with a generic interface:
let g:vimtex_view_general_viewer = 'okular'
let g:vimtex_view_general_options = '--unique file:@pdf\#src:@line@tex'

" VimTeX uses latexmk as the default compiler backend. If you use it, which is
" strongly recommended, you probably don't need to configure anything. If you
" want another compiler backend, you can change it as follows. The list of
" supported backends and further explanation is provided in the documentation,
" see ":help vimtex-compiler".
let g:vimtex_compiler_method = 'latexmk'

" Most VimTeX mappings rely on localleader and this can be changed with the
" following line. The default is usually fine and is the symbol "\".
let maplocalleader = ","

" ======================================
" CMake
" ======================================
g:cmake_link_compile_commands=1
" Show cmake version in status line
set statusline=%{cmake#GetInfo().cmake_version.string}
map <F7> <Plug>(CMakeOpen)

" CMake build
map <F5> <Plug>(CMakeBuild)

" Run make
nmap <leader>m :make<cr>

" ======================================
" Tagbar
" ======================================
nmap <leader>tt :TagbarToggle f<CR>
nmap <leader>to :TagbarOpen f<CR>
nmap <leader>tc :TagbarClose<CR>
nmap <leader>tn :TagbarJumpNext<CR>
nmap <leader>tN :TagbarJumpPrev<CR>
let g:tagbar_autoclose=1

" ======================================
" Doge
" ======================================
" Set Python standard to Numpy
let g:doge_doc_standard_python = 'numpy'

" ======================================
" Nerdcommenter
" ======================================
" Align line-wise comment delimiters flush left instead of following code indentation
let g:NERDDefaultAlign = 'left'
imap <C-_> <esc><Plug>NERDCommenterToggle 

" ======================================
" Diffview
" ======================================
" Open vimdiff
nmap <leader>gd :DiffviewOpen :input()<CR>

" Ruler has column and AsyncRun status
set rulerformat=%60(%=%t\ %c\ %{g:asyncrun_status}%)


" ======================================
" Asyncrun
" ======================================
" C++ builds/run
" Catkin builds
noremap <leader>b :AsyncRun -cwd=<root> catkin build<CR>
noremap <leader>t :AsyncRun -cwd=<root> catkin build --make-args tests<CR>
noremap <leader>n :AsyncStop<CR>
noremap <leader><leader> :call asyncrun#quickfix_toggle(20)<CR>
let g:asyncrun_open = 4
fun! OnAsyncRunFinished()
    if g:asyncrun_status == 'success'
      sleep 1
      cclose
    else
      copen 20
    endif
endf
let g:asyncrun_exit = "call OnAsyncRunFinished()"
