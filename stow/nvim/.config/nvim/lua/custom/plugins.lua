local overrides = require "custom.configs.overrides"

---@type NvPluginSpec[]
local plugins = {

  -- Override plugin definition options
  {
    "neovim/nvim-lspconfig",
    dependencies = {
      -- format & linting
      {
        "jose-elias-alvarez/null-ls.nvim",
        config = function()
          require "custom.configs.null-ls"
        end,
      },
    },
    config = function()
      require "plugins.configs.lspconfig"
      require "custom.configs.lspconfig"
    end, -- Override to setup mason-lspconfig
  },

  -- override plugin configs
  { "williamboman/mason.nvim", opts = overrides.mason },
  { "nvim-treesitter/nvim-treesitter", opts = overrides.treesitter },
  { "nvim-tree/nvim-tree.lua", opts = overrides.nvimtree },
  -- Override nvim-cmp
  {
    "hrsh7th/nvim-cmp",
    event = "InsertEnter",
    opts = function()
      return require "custom.configs.cmp"
    end,
    config = function(_, opts)
      require("cmp").setup(opts)
    end,
  },
  -- Better escape using `jk`
  {
    "max397574/better-escape.nvim",
    event = "InsertEnter",
    config = function()
      require("better_escape").setup()
    end,
  },
  --  Quick jumping around documents
  {
    "easymotion/vim-easymotion",
    lazy = false,
    config = function()
      require "custom.configs.easymotion"
    end,
  },
  -- Quick two-letter searches
  { "justinmk/vim-sneak", keys = { "s", "S" } },

  -- Navigate thorugh file tags/functions
  {
    "preservim/tagbar",
    cmd = { "TagbarToggle", "TagbarOpen" },
    config = function()
      require "custom.configs.tagbar"
    end,
  },
  -- Commenting
  {
    "preservim/nerdcommenter",
    lazy = false,
    config = function()
      require "custom.configs.nerdcommenter"
    end,
  },
  -- Mappings to surround words with chars
  { "tpope/vim-surround", lazy = false },
  -- Git commands
  {
    "tpope/vim-fugitive",
    cmd = "G",
  },
  -- Git diff
  {
    "sindrets/diffview.nvim",
    cmd = "DiffviewOpen",
    lazy = false,
    config = function()
      require "custom.configs.diffview"
    end,
  },
  -- Toggle terminals from within vim
  {
    "akinsho/toggleterm.nvim",
    cmd = "ToggleTerm",
    lazy = false,
    config = function()
      require "custom.configs.toggleterm"
    end,
  },

  -- ---------------------------------------------------------------
  -- C++
  -- ---------------------------------------------------------------
  -- Doxygen comments
  {
    "MRTAZZ/DOXYGENTOOLKIT.VIM",
    ft = "cpp",
    event = "bufenter",
    lazy = false,
    config = function()
      require "custom.configs.doxygentoolkit"
    end,
  },
  -- Clang format. Note that it may not be used if LSP formatter is already setup
  { "rhysd/vim-clang-format", ft = "cpp", enabled = false },
  -- Header guard
  {
    "drmikehenry/vim-headerguard",
    ft = "cpp",
    config = function()
      require "custom.configs.headerguard"
    end,
  },
  -- CMake management
  {
    "cdelledonne/vim-cmake",
    ft = { "cpp", "cmake" },
    lazy = false,
    config = function()
      require "custom.configs.cmake"
    end,
  },
  -- Preview markdown files
  {
    "iamcco/markdown-preview.nvim",
    init = function()
      vim.fn["mkdp#util#install"]()
    end,
    enabled = false,
  },
  -- Markdown table-of-contents generation
  { "mzlogin/vim-markdown-toc", ft = "md" },

  -- Seamless navigation between tmux and vim
  { "christoomey/vim-tmux-navigator", lazy = false },

  -- Fuzzy search
  {
    "junegunn/fzf",
    init = function()
      vim.cmd "silent! call fzf#install()"
    end,
    lazy = false,
    config = function()
      require "custom.configs.fzf"
    end,
  },

  -- Fzf
  { "junegunn/fzf.vim", lazy = false },

  -- Startup menu
  {
    "mhinz/vim-startify",
    lazy = false,
    config = function()
      require "custom.configs.startify"
    end,
  },

  -- Latex support
  { "lervag/vimtex", ft = "tex" },

  -- Document generation
  {
    "kkoomen/vim-doge",
    init = function()
      vim.cmd "silent! call doge#install()"
    end,
    ft = { "py", "cpp", "js" },
  },

  -- Match pairs (e.g., paranthesis). There's another installed plugin doing that
  { "tmsvg/pear-tree", enabled = false },

  -- Make marks visible
  { "kshenoy/vim-signature", lazy = false },

  -- Save undo actions
  { "mbbill/undotree", cmd = "Undo" },

  -- Buffer closure/deletion
  { "mhinz/vim-sayonara", lazy = false },
  { "asheq/close-buffers.vim", lazy = false },

  -- GPT
  -- The configs are needed to be here to make sure the plugin is loaded before its used
  { --  Github copilot
    "zbirenbaum/copilot.lua",
    cmd = "Copilot",
    event = "InsertEnter",
    config = function(_, _)
      require "custom.configs.copilot"
    end,
  },
  -- Copilot autocompletion
  {
    "zbirenbaum/copilot-cmp",
    after = { "copilot.lua" },
    config = function(_, _)
      require("copilot_cmp").setup()
    end,
    lazy = false,
  },

  -- ChatGPT
  {
    "jackMort/ChatGPT.nvim",
    config = function()
      require("chatgpt").setup()
    end,
    requires = {
      "MunifTanjim/nui.nvim",
      "nvim-lua/plenary.nvim",
      "nvim-telescope/telescope.nvim",
    },
  },

  -- To make a plugin not be loaded
  -- {
  --   "NvChad/nvim-colorizer.lua",
  --   enabled = false
  -- },

  -- All NvChad plugins are lazy-loaded by default
  -- For a plugin to be loaded, you will need to set either `ft`, `cmd`, `keys`, `event`, or set `lazy = false`
  -- If you want a plugin to load on startup, add `lazy = false` to a plugin spec, for example
  -- {
  --   "mg979/vim-visual-multi",
  --   lazy = false,
  -- }
}

return plugins
