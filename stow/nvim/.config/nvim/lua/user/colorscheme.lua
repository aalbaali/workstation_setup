--local colorscheme = "tokyonight-night"
--local colorscheme = "darkplus"
local colorscheme = "gruvbox"
vim.g.gruvbox_contrast_dark = 'hard'
vim.opt.background = 'dark'


local status_ok, _ = pcall(vim.cmd, "colorscheme " .. colorscheme)
if not status_ok then
  return
end
