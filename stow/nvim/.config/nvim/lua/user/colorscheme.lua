------ Tokyo (dark and high contrast)
--local colorscheme = "tokyonight-night"

---- Darkplus (autocompletion not working properly on this color scheme)
--local colorscheme = "darkplus"

-- catppuccin-mocha
--local colorscheme = "catppuccin-mocha"

---- Gruvbox
local colorscheme = "gruvbox"
vim.g.gruvbox_contrast_dark = 'hard'

---- Iceberg
--local colorscheme = "iceberg"

---- Onehalf light
--local colorscheme = "onehalflight"

---- Molokai (Very colorful)
--local colorscheme = "molokai"

---- Vim hybrid (similar to gruvbox but with higher contrast)
--local colorscheme = "hybrid"
--vim.g.gruvbox_contrast_dark = 'hard'

---- Gruvbox baby (similar to gruvbox but with higher contrast)
----vim.g.gruvbox_baby_function_style = "NONE"
----vim.g.gruvbox_baby_keyword_style = "italic"
----vim.g.gruvbox_baby_highlights = {Normal = {fg = "#123123", bg = "NONE", style="underline"}}
--vim.g.background_color = "dark"
--vim.g.transparent_mode = false
----vim.g.gruvbox_baby_telescope_theme = 1
----vim.g.gruvbox_baby_transparent_mode = 1
--vim.g.use_original_palette = false
--local colorscheme = "gruvbox-baby"


-- -- Monokai (somewhat dark)
-- vim.g.vim_monokai_tasty_machine_tint = 1
--local colorscheme = "vim-monokai-tasty"

---- Onenord (Nice, but underlines the text under cursor)
--require('onenord').setup({
--  theme = "dark", -- "dark" or "light". Alternatively, remove the option and set vim.o.background instead
--  borders = true, -- Split window borders
--  fade_nc = false, -- Fade non-current windows, making them more distinguishable
--  -- Style that is applied to various groups: see `highlight-args` for options
--  styles = {
--    comments = "NONE",
--    strings = "NONE",
--    keywords = "NONE",
--    functions = "NONE",
--    variables = "NONE",
--    diagnostics = "underline",
--  },
--  disable = {
--    background = true, -- Disable setting the background color
--    cursorline = false, -- Disable the cursorline
--    eob_lines = false, -- Hide the end-of-buffer lines
--  },
--  -- Inverse highlight for different groups
--  inverse = {
--    match_paren = true,
--  },
--  custom_highlights = {}, -- Overwrite default highlight groups
--  custom_colors = {}, -- Overwrite default colors
--})
--local colorscheme = "onenord"

---- Ayu (similar to gruvbox when 'mirage' is enabled, otherwise, it's high-contrast dark color scheme)
--require('ayu').setup({
--    mirage = true, -- Set to `true` to use `mirage` variant instead of `dark` for dark background.
--    overrides = {}, -- A dictionary of group names, each associated with a dictionary of parameters (`bg`, `fg`, `sp` and `style`) and colors in hex.
--})
--require('lualine').setup({
--  options = {
--    theme = 'ayu',
--  },
--})
--local colorscheme = "ayu"

---- Nordic (auto completion is not quite visible)
--local colorscheme = "nordic"

---- Noctis (similar to vscode. A little 'too' blue)
--local colorscheme = "noctis"

---- Darcula (CLion's theme. background is a little light)
--local colorscheme = "darcula"

---- Darcula solid (background a little darker than darcula, The colors are not very rich)
--local colorscheme = "darcula-solid"

-- Background options
vim.opt.background = 'dark'

--require("catppuccin").setup({
--  flavour = "mocha",   -- latte, frappe, macchiato, mocha
--  background = {       -- :h background
--    light = "latte",
--    dark = "mocha",
--  },
--  transparent_background = false,   -- disables setting the background color.
--  show_end_of_buffer = false,       -- shows the '~' characters after the end of buffers
--  term_colors = false,              -- sets terminal colors (e.g. `g:terminal_color_0`)
--  dim_inactive = {
--    enabled = false,                -- dims the background color of inactive window
--    shade = "dark",
--    percentage = 0.15,              -- percentage of the shade to apply to the inactive window
--  },
--  no_italic = false,                -- Force no italic
--  no_bold = false,                  -- Force no bold
--  no_underline = false,             -- Force no underline
--  styles = {                        -- Handles the styles of general hi groups (see `:h highlight-args`):
--    comments = { "italic" },        -- Change the style of comments
--    conditionals = { "italic" },
--    loops = {},
--    functions = {},
--    keywords = {},
--    strings = {},
--    variables = {},
--    numbers = {},
--    booleans = {},
--    properties = {},
--    types = {},
--    operators = {},
--    -- miscs = {}, -- Uncomment to turn off hard-coded styles
--  },
--  color_overrides = {
--    mocha = {
--      base = "#001435",
--      mantle = "#000000",
--      crust = "#808080",
--    },
--  },
--  custom_highlights = {},
--  integrations = {
--    cmp = true,
--    gitsigns = true,
--    nvimtree = true,
--    treesitter = true,
--    notify = false,
--    mini = {
--      enabled = true,
--      indentscope_color = "",
--    },
--    -- For more plugins integrations please scroll down (https://github.com/catppuccin/nvim#integrations)
--  },
--})

-- --setup must be called before loading
--vim.cmd.colorscheme "catppuccin"

vim.cmd.colorscheme(colorscheme)
