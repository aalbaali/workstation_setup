local lsp = require('lsp-zero')
lsp.preset({
  name = 'minimal',
  set_lsp_keymaps = false,
  manage_nvim_cmp = true,
  suggest_lsp_servers = true,
})

-- Fix Undefined global 'vim'
lsp.configure('lua_ls', {
  settings = {
    Lua = {
      diagnostics = {
        globals = { 'vim' }
      }
    }
  }
})


-- Might no longer be necessary
lsp.on_attach = function(client, _)
  if client.name == "ts_ls" then
    client.server_capabilities.documentFormattingProvider = true
  end

  if client.name == "lua_ls" then
    client.server_capabilities.documentFormattingProvider = true
  end

  --if client.name == "autopep8" then
  --  client.server_capabilities.documentFormattingProvider = true
  --end

  -- Add flake8 formatter
  if client.name == "flake8" then
    client.server_capabilities.documentFormattingProvider = true
  end

  -- Disable some keymappings. Custom mappings are defined in the handlers.lua file.
  --lsp.default_keymaps({buffer = bufnr})
end

lsp.setup()

vim.diagnostic.config({
  virtual_text = false
})
