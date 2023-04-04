local lsp = require('lsp-zero')

require "user.lsp.mason"
require("user.lsp.handlers").setup()
require "user.lsp.null-ls"

lsp.preset({
  name = 'minimal',
  set_lsp_keymaps = true,
  manage_nvim_cmp = true,
  suggest_lsp_servers = true,
})

local servers = {
	-- "xml",
	-- "html",
	-- "tsserver",
	"pyright",
	"jsonls",
  "clangd",
	"lua_ls",
}
lsp.ensure_installed(servers)

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


lsp.on_attach = function(client, _)
	if client.name == "tsserver" then
		client.server_capabilities.documentFormattingProvider = false
	end

	if client.name == "lua_ls" then
		client.server_capabilities.documentFormattingProvider = false
	end
end

lsp.setup()

vim.diagnostic.config({
    virtual_text = false
})

