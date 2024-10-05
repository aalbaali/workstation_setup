local M = {}

local status_cmp_ok, cmp_nvim_lsp = pcall(require, "cmp_nvim_lsp")
if not status_cmp_ok then
  return
end

M.capabilities = vim.lsp.protocol.make_client_capabilities()
M.capabilities.textDocument.completion.completionItem.snippetSupport = true
M.capabilities = cmp_nvim_lsp.default_capabilities(M.capabilities)
M.capabilities.offsetEncoding = { "utf-16" }

M.setup = function()
  local signs = {

    { name = "DiagnosticSignError", text = "" },
    { name = "DiagnosticSignWarn", text = "" },
    { name = "DiagnosticSignHint", text = "" },
    { name = "DiagnosticSignInfo", text = "" },
  }

  for _, sign in ipairs(signs) do
    vim.fn.sign_define(sign.name, { texthl = sign.name, text = sign.text, numhl = "" })
  end

  local config = {
    virtual_text = false, -- disable virtual text
    signs = {
      active = signs,   -- show signs
    },
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

  --vim.lsp.handlers["textDocument/hover"] = vim.lsp.with(vim.lsp.handlers.hover, {
  --  border = "rounded",
  --})

  --vim.lsp.handlers["textDocument/signatureHelp"] = vim.lsp.with(vim.lsp.handlers.signature_help, {
  --  border = "rounded",
  --})
end

local function lsp_keymaps(bufnr)
  local opts = { noremap = true, silent = true }
  local keymap = vim.api.nvim_buf_set_keymap
  keymap(bufnr, "n", "gD", "<cmd>lua vim.lsp.buf.declaration()<CR>", opts)
  keymap(bufnr, "n", "gd", "<cmd>Lspsaga goto_definition<CR>", opts)
  keymap(bufnr, "n", "gt", "<cmd>Lspsaga goto_type_definition<CR>", opts)
  keymap(bufnr, "n", "gsD", "<cmd>vsplit<cr> <cmd>lua vim.lsp.buf.declaration()<CR>", opts)
  keymap(bufnr, "n", "gsd", "<cmd>vsplit<cr> <cmd>lua vim.lsp.buf.definition()<CR>", opts)
  keymap(bufnr, "n", "gst", "<cmd>vsplit<cr> <cmd>lua vim.lsp.buf.type_definition()<CR>", opts)
  keymap(bufnr, "n", "gh", "<cmd>ClangdSwitchSourceHeader<cr>", opts)
  keymap(bufnr, "n", "gsh", "<cmd>vsplit<cr> <cmd>ClangdSwitchSourceHeader<cr>", opts)
  keymap(bufnr, "n", "gsH", "<cmd>split<cr> <cmd>ClangdSwitchSourceHeader<cr>", opts)
  keymap(bufnr, "n", "K", "<cmd>Lspsaga hover_doc<cr>", opts)
  keymap(bufnr, "n", "L", "<cmd>Lspsaga hover_doc ++keep<cr>", opts)
  keymap(bufnr, "n", vim.g.altleader .. "k", "<cmd>Lspsaga show_line_diagnostics<CR>", opts)
  keymap(bufnr, "n", "gI", "<cmd>lua vim.lsp.buf.implementation()<CR>", opts)
  keymap(bufnr, "n", "gR", "<cmd>Lspsaga finder<CR>", opts)
  keymap(bufnr, "n", "gr", "<cmd>lua vim.lsp.buf.references()<CR>", opts)
  keymap(bufnr, "n", "gl", "<cmd>lua vim.diagnostic.open_float()<CR>", opts)
  --keymap(bufnr, "n", "<leader>lf", "<cmd>lua vim.lsp.buf.format{ async = true }<cr>", opts)
  keymap(bufnr, "n", "<leader>li", "<cmd>LspInfo<cr>", opts)
  keymap(bufnr, "n", "<leader>lI", "<cmd>Mason<cr>", opts)
  keymap(bufnr, "n", "<leader>la", "<cmd>Lspsaga code_action<cr>", opts)
  keymap(bufnr, "n", "]d", "<cmd>Lspsaga diagnostic_jump_next<cr>", opts)
  keymap(bufnr, "n", "[d", "<cmd>Lspsaga diagnostic_jump_prev<cr>", opts)
  keymap(bufnr, "n", "<leader>lr", "<cmd>Lspsaga rename<cr>", opts)
  keymap(bufnr, "i", "<c-o>", "<cmd>lua vim.lsp.buf.signature_help()<CR>", opts)
  keymap(bufnr, "n", "<leader>lq", "<cmd>lua vim.diagnostic.setloclist()<CR>", opts)
  keymap(bufnr, "n", "<leader>tt", "<cmd>Lspsaga outline<CR>", opts)
end

M.on_attach = function(client, bufnr)
  if client.name == "ts_ls" then
    client.server_capabilities.documentFormattingProvider = true
  end

  if client.name == "sumneko_lua" then
    client.server_capabilities.documentFormattingProvider = true
  end

  lsp_keymaps(bufnr)
end

return M
