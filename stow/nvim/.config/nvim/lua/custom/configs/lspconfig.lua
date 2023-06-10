local lspconfig = require "plugins.configs.lspconfig"
local capabilities = lspconfig.capabilities
capabilities.offsetEncoding = { "utf-16" }

local config = {
  virtual_text = true, -- disable virtual text
  update_in_insert = true,
  underline = false,
  severity_sort = true,
  float = {
    focusable = true,
    style = "minimal",
    border = "rounded",
    source = "always",
    header = "",
    prefix = "",
  },
}
vim.diagnostic.config(config)

local lspconfig = require "lspconfig"

-- if you just want default config for the servers then put them in a table
local servers = { "html", "cssls", "tsserver", "clangd", "pyright" }

-- Keymaps for LSP
local function lsp_keymaps(bufnr)
  local opts = { noremap = true, silent = true }
  local keymap = vim.api.nvim_buf_set_keymap
  keymap(bufnr, "n", "gD", "<cmd>lua vim.lsp.buf.declaration()<CR>", opts)
  keymap(bufnr, "n", "gd", "<cmd>lua vim.lsp.buf.definition()<CR>", opts)
  keymap(bufnr, "n", "gt", "<cmd>lua vim.lsp.buf.type_definition()<CR>", opts)
  keymap(bufnr, "n", "gsD", "<cmd>vsplit<cr> <cmd>lua vim.lsp.buf.declaration()<CR>", opts)
  keymap(bufnr, "n", "gsd", "<cmd>vsplit<cr> <cmd>lua vim.lsp.buf.definition()<CR>", opts)
  keymap(bufnr, "n", "gst", "<cmd>vsplit<cr> <cmd>lua vim.lsp.buf.type_definition()<CR>", opts)
  keymap(bufnr, "n", "gh", "<cmd>ClangdSwitchSourceHeader<cr>", opts)
  keymap(bufnr, "n", "gsh", "<cmd>vsplit<cr> <cmd>ClangdSwitchSourceHeader<cr>", opts)
  keymap(bufnr, "n", "gsH", "<cmd>split<cr> <cmd>ClangdSwitchSourceHeader<cr>", opts)
  keymap(bufnr, "n", "K", "<cmd>lua vim.lsp.buf.hover()<CR>", opts)
  keymap(bufnr, "n", vim.g.altleader .. "k", "<cmd>lua vim.diagnostic.open_float()<CR>", opts)
  keymap(bufnr, "n", "gI", "<cmd>lua vim.lsp.buf.implementation()<CR>", opts)
  keymap(bufnr, "n", "gr", "<cmd>lua vim.lsp.buf.references()<CR>", opts)
  keymap(bufnr, "n", "gl", "<cmd>lua vim.diagnostic.open_float()<CR>", opts)
  keymap(bufnr, "n", "<leader>lf", "<cmd>lua vim.lsp.buf.format{ async = true }<cr>", opts)
  keymap(bufnr, "n", "<leader>li", "<cmd>LspInfo<cr>", opts)
  keymap(bufnr, "n", "<leader>lI", "<cmd>Mason<cr>", opts)
  keymap(bufnr, "n", "<leader>la", "<cmd>lua vim.lsp.buf.code_action()<cr>", opts)
  keymap(bufnr, "n", "]d", "<cmd>lua vim.diagnostic.goto_next({buffer=0})<cr>", opts)
  keymap(bufnr, "n", "[d", "<cmd>lua vim.diagnostic.goto_prev({buffer=0})<cr>", opts)
  keymap(bufnr, "n", "<leader>lr", "<cmd>lua vim.lsp.buf.rename()<cr>", opts)
  keymap(bufnr, "i", "<c-o>", "<cmd>lua vim.lsp.buf.signature_help()<CR>", opts)
  keymap(bufnr, "n", "<leader>lq", "<cmd>lua vim.diagnostic.setloclist()<CR>", opts)
end

-- Augment on_attach function with custom behaviour
local on_attach = function(client, bufnr)
  lsp_keymaps(bufnr)
end
for _, lsp in ipairs(servers) do
  lspconfig[lsp].setup {
    on_attach = on_attach,
    capabilities = capabilities,
  }
end

--
-- lspconfig.pyright.setup { blabla}
