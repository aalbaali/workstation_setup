------ Tokyo (dark and high contrast)
--local colorscheme = "tokyonight-night"

---- Darkplus (autocompletion not working properly on this color scheme)
--local colorscheme = "darkplus"

-- Gruvbox
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

local status_ok, _ = pcall(vim.cmd, "colorscheme " .. colorscheme)
if not status_ok then
  return
end
