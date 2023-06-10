local fn = vim.fn

-- Automatically install packer
local install_path = fn.stdpath("data") .. "/site/pack/packer/start/packer.nvim"
if fn.empty(fn.glob(install_path)) > 0 then
  PACKER_BOOTSTRAP = fn.system({
    "git",
    "clone",
    "--depth",
    "1",
    "https://github.com/wbthomason/packer.nvim",
    install_path,
  })
  print("Installing packer close and reopen Neovim...")
  vim.cmd([[packadd packer.nvim]])
end

-- Autocommand that reloads neovim whenever you save the plugins.lua file
vim.cmd([[
  augroup packer_user_config
    autocmd!
    autocmd BufWritePost plugins.lua source <afile> | PackerSync
  augroup end
]])

-- Use a protected call so we don't error out on first use
local status_ok, packer = pcall(require, "packer")
if not status_ok then
  return
end

-- Have packer use a popup window
packer.init({
  display = {
    open_fn = function()
      return require("packer.util").float({ border = "rounded" })
    end,
  },
})

-- Install your plugins here
return packer.startup(function(use)
  use { "wbthomason/packer.nvim" } -- Have packer manage itself
  use { "nvim-lua/plenary.nvim" }  -- Useful lua functions used by lots of plugins
  use { "windwp/nvim-autopairs" }  -- Autopairs, integrates with both cmp and treesitter
  use { "numToStr/Comment.nvim" }
  use { "JoosepAlviste/nvim-ts-context-commentstring" }
  use { "kyazdani42/nvim-web-devicons" }
  use { "kyazdani42/nvim-tree.lua", commit = '9c97e6449b0b0269bd44e1fd4857184dfa57bb4c' }
  use { "akinsho/bufferline.nvim" }
  use { "moll/vim-bbye" }
  use { "nvim-lualine/lualine.nvim" }
  use { "akinsho/toggleterm.nvim" }
  --use { "ahmedkhalf/project.nvim" }
  use { "lewis6991/impatient.nvim" }
  use { "lukas-reineke/indent-blankline.nvim" }
  -- use { "goolord/alpha-nvim" }
  use { "folke/which-key.nvim" }

  -- Colorschemes
  use { "folke/tokyonight.nvim" }
  use { "lunarvim/darkplus.nvim" }
  use { "morhetz/gruvbox" }
  use { "bluz71/vim-moonfly-colors", as = "moonfly" }
  use { "savq/melange-nvim" }
  use { "mcchrish/zenbones.nvim", requires = "rktjmp/lush.nvim" }
  use { "nyoom-engineering/oxocarbon.nvim" }
  use { "patstockwell/vim-monokai-tasty" }
  use { "rmehri01/onenord.nvim" }
  use { "luisiacc/gruvbox-baby", branch = 'main' }
  use { "Shatur/neovim-ayu" }
  use { "AlexvZyl/nordic.nvim" }
  use { "kartikp10/noctis.nvim" }
  use { "doums/darcula" }
  use { "briones-gabriel/darcula-solid.nvim", requires = "rktjmp/lush.nvim" }
  use { "cocopon/iceberg.vim" }
  use { "sonph/onehalf" }
  use { "tomasr/molokai" }
  use { "w0ng/vim-hybrid" }

  use('jremmen/vim-ripgrep') --  Ripgrep fuzzy searcher
  use {
    'VonHeikemen/lsp-zero.nvim',
    branch = 'v2.x',
    requires = {
      -- LSP Support
      { 'neovim/nvim-lspconfig' },             -- Required
      { 'williamboman/mason.nvim' },           -- Optional
      { 'williamboman/mason-lspconfig.nvim' }, -- Optional

      -- Autocompletion
      { 'hrsh7th/nvim-cmp' },         -- Required
      { 'hrsh7th/cmp-nvim-lsp' },     -- Required
      { 'hrsh7th/cmp-buffer' },       -- Optional
      { 'hrsh7th/cmp-path' },         -- Optional
      { 'saadparwaiz1/cmp_luasnip' }, -- Optional
      { 'hrsh7th/cmp-nvim-lua' },     -- Optional

      -- Snippets
      { 'L3MON4D3/LuaSnip' },             -- Required
      { 'rafamadriz/friendly-snippets' }, -- Optional
    }
  }
  use 'simrat39/rust-tools.nvim'

  use { "jose-elias-alvarez/null-ls.nvim" } -- for formatters and linters
  use { "RRethy/vim-illuminate" }
  use { "nvim-telescope/telescope.nvim" }
  use { "nvim-treesitter/nvim-treesitter" }
  use { "nvim-treesitter/nvim-treesitter-context" }   -- View context at the current cursor functions/classes
  use { "lewis6991/gitsigns.nvim" }

  use('tpope/vim-fugitive')             --  Git comments
  use('vim-utils/vim-man')              --  View `man` pages in vim
  use('mrtazz/DoxygenToolkit.vim')      --  Auto-insert Doxygen comments
  use('skywind3000/asyncrun.vim')       --  Run commands / builds in background
  use('christoomey/vim-tmux-navigator') --  Seamless navigation between vim and tmux
  --use('dense-analysis/ale')              --  Asynchronous linting
  --use('sheerun/vim-polyglot')            --  Better syntax highlighting
  use { 'junegunn/fzf', run = function() vim.cmd("silent! call fzf#install()") end } --  Install fzf (fast fuzzy searcher)
  use('junegunn/fzf.vim')                                                            --  fzf vim extension
  -- use('airblade/vim-gitgutter')          --  Git status on side bar and git operations
  use('rhysd/vim-clang-format')                                                      --  Commands for applying clang-formatting
  use('preservim/nerdtree')                                                          --  Navigate files using a tree structure
  use('mhinz/vim-startify')                                                          --  Manage vim sessions
  -- use('vim-airline/vim-airline')         --  Custom status bar
  -- use('vim-airline/vim-airline-themes')  --  Themes
  use('lervag/vimtex')                                                                    --  latex support
  use('justinmk/vim-sneak')                                                               --  fast navigation
  use('tpope/vim-surround')                                                               --  operations for surrounding words with paranthesis
  use { 'kkoomen/vim-doge', run = function() vim.cmd("silent! call doge#install()") end } --  (Do)cument (Ge)nerator for various file systems
  use('preservim/tagbar')                                                                 --  Browse tags of current file
  use('preservim/nerdcommenter')                                                          --  Commenting plugin
  use('sindrets/diffview.nvim')                                                           --  Neovim diffview
  use('tmsvg/pear-tree')                                                                  --  Pair brackets, braces, etc.
  use('kshenoy/vim-signature')                                                            --  Place, toggle, and display marks
  use('drmikehenry/vim-headerguard')                                                      --  C++ header guards
  use('easymotion/vim-easymotion')                                                        --  Quick jumping around documents
  use('mbbill/undotree')
  use('mhinz/vim-sayonara')                                                               --  Kill buffers well
  use('asheq/close-buffers.vim')                                                          --  Kill buffers well
  use('mindriot101/vim-yapf')                                                             --  Python formatting using Yapf
  use('cdelledonne/vim-cmake')                                                            --  Construct and build CMake projects
  use({
    "iamcco/markdown-preview.nvim",
    run = function() vim.fn["mkdp#util#install"]() end,
  })

  -- The configs are needed to be here to make sure the plugin is loaded before its used
  use { --  Github copilot
    "zbirenbaum/copilot.lua",
    cmd = "Copilot",
    event = "InsertEnter",
    config = function()
      require("user.copilot")
    end,
  }
  -- Copilot autocompletion
  use {
    "zbirenbaum/copilot-cmp",
    after = { "copilot.lua" },
    config = function()
      require("copilot_cmp").setup()
    end
  }

  -- ChatGPT
  use({
    "jackMort/ChatGPT.nvim",
    config = function()
      require("chatgpt").setup()
    end,
    requires = {
      "MunifTanjim/nui.nvim",
      "nvim-lua/plenary.nvim",
      "nvim-telescope/telescope.nvim"
    }
  })

  -- Markdown table-of-contents generation
  use { "mzlogin/vim-markdown-toc" }

  -- View PlantUML diagrams
  use { "tyru/open-browser.vim" }
  use { "weirongxu/plantuml-previewer.vim" }

  -- Highlight and color todo comments
  use {
    "folke/todo-comments.nvim",
    requires = "nvim-lua/plenary.nvim",
    config = function()
      require("todo-comments").setup {}
    end
  }

  -- Automatically set up your configuration after cloning packer.nvim
  -- Put this at the end after all plugins
  if PACKER_BOOTSTRAP then
    require("packer").sync()
  end
end)
