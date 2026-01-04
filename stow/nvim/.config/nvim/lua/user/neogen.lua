local status_ok, pkg = pcall(require, 'neogen')
if not status_ok then
  return
end

pkg.setup({
  snippet_engine = 'luasnip',
  languages = {
    python = {
      template = {
        annotation_convention = 'google_docstrings',
      },
    },
  },
})

-- Clear lua snip when exiting to normal mode
--  https://github.com/L3MON4D3/LuaSnip/issues/258#issuecomment-1429989436
vim.api.nvim_create_autocmd('ModeChanged', {
  pattern = '*',
  callback = function()
    if
      ((vim.v.event.old_mode == 's' and vim.v.event.new_mode == 'n') or vim.v.event.old_mode == 'i')
      and require('luasnip').session.current_nodes[vim.api.nvim_get_current_buf()]
      and not require('luasnip').session.jump_active
    then
      require('luasnip').unlink_current()
    end
  end,
})
