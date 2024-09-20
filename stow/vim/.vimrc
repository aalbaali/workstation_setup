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
call plug#begin('~/.vim/plugged')
Plug 'morhetz/gruvbox'
Plug 'jremmen/vim-ripgrep'
Plug 'tpope/vim-fugitive'             " Git comments
Plug 'leafgarland/typescript-vim'
Plug 'tpope/vim-surround'
Plug 'junegunn/fzf', { 'do': { -> fzf#install() } }  " Install fzf (fast fuzzy searcher)

call plug#end()

" ======================================
" Settings
" ======================================
syntax on " Sets syntax highlighting to on
set noerrorbells " no error sounds/bells after end of line
set tabstop=2 softtabstop=2 " tabstop: 2 characters long. tabstop: 2 spaces long.
set shiftwidth=2 " indent next line (after hitting 'enter') by 2 spaces
set expandtab " convert tabs to spaces
set smartindent " attempts smart indentation (auto indent) for different languages
set nu " set line numbering
set nowrap " do not wrap numbers
set noswapfile " no swap files when writing in vim (I think they're usually there for backup
set nobackup " no backup file. We're doing undodir/undofile instead. 
set undodir=~/.vim/undodir "undo directory for 'backup'
set incsearch " incremental search
set paste " Well formatted text when pasting and no comments when inserting new lines

set colorcolumn=100 " colors column at this limit
highlight ColorColumn ctermbg=0 guibg=lightgrey
set ruler " show row and column
set splitright " new verticalsplits go to the right
set splitbelow " new horizontal splits go below
set ignorecase " ignore case when using a search patter
set smartcase " override 'ignorecase' when pattern has upper case character
set number " show line numbers
set softtabstop=2 " always use spaces, never tabs
"set updatetime=250 " 250 ms between screen updates
set wildmode=list:longest,full " list completions on command line, cycle through with tab
"set wrap " automatically wrap text that extends beyond the screen length

" Copy to clipboard. Note: requires `vim-gtk` (install using `sudo apt-get install vim-gtk`)
vnoremap <leader>y "+y
nnoremap <leader>yy V"+y


nnoremap j gj
nnoremap k gk

" Ruler has column and AsyncRun status
" set rulerformat=%60(%=%t\ %c\ %{g:asyncrun_status}%)

" colorscheme
silent! colorscheme gruvbox
set background=dark

" allows rg to always to detect root and use .gitignore for faster searching
if executable('rg')
  let g:rg_derive_root='true'
endif


" Quitting vim
nnoremap <silent> <Space>q :wqa<CR>
nnoremap <silent> <Space>x :qa!<CR>

