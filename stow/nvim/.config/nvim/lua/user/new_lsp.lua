local lsp_zero = require('lsp-zero')

require('user.lsp.handlers').setup()

-- Lsp-specific keymaps
local function lsp_keymaps(bufnr)
  local opts = { noremap = true, silent = false }
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
  --keymap(bufnr, "n", "<leader>lf", "<cmd>LspZeroFormat<cr>", opts)
  keymap(bufnr, "n", "<leader>li", "<cmd>LspInfo<cr>", opts)
  keymap(bufnr, "n", "<leader>lI", "<cmd>Mason<cr>", opts)
  keymap(bufnr, "n", "<leader>la", "<cmd>lua vim.lsp.buf.code_action()<cr>", opts)
  keymap(bufnr, "n", "]d", "<cmd>lua vim.diagnostic.goto_next({buffer=0})<cr>", opts)
  keymap(bufnr, "n", "[d", "<cmd>lua vim.diagnostic.goto_prev({buffer=0})<cr>", opts)
  keymap(bufnr, "n", "<leader>lr", "<cmd>lua vim.lsp.buf.rename()<cr>", opts)
  keymap(bufnr, "i", "<c-o>", "<cmd>lua vim.lsp.buf.signature_help()<CR>", opts)
  keymap(bufnr, "n", "<leader>lq", "<cmd>lua vim.diagnostic.setloclist()<CR>", opts)
end

lsp_zero.on_attach(function(client, bufnr)
  -- see :help lsp-zero-keybindings
  if client.name == "tsserver" then
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

  lsp_keymaps(bufnr)

  --TODO: I'm not certain what this does and if it's necessary
  local status_ok, illuminate = pcall(require, "illuminate")
  if not status_ok then
    return
  end
  illuminate.on_attach(client)
end)

lsp_zero.preset({
  name = 'minimal',
  set_lsp_keymaps = true,
  manage_nvim_cmp = true,
  suggest_lsp_servers = true,
})

-- Fix Undefined global 'vim'
require('lspconfig').lua_ls.setup {
  settings = {
    Lua = {
      diagnostics = {
        globals = { 'vim' }
      }
    }
  }
}
require('lspconfig').pylsp.setup {
  settings = {
    pylsp = {
      plugins = {
        pycodestyle = {
          ignore = { 'W391' },
          maxLineLength = 100
        }
      }
    }
  }
}


lsp_zero.setup()

-- here you can setup the language servers
local servers = {
  "bashls",
  "pyright",
  "clangd",
  "cmake",
  "rust_analyzer",
  "lua_ls",
  "tsserver",
  "jsonls",
}

-- Handlers setup

-- to learn how to use mason.nvim with lsp-zero
-- read this: https://github.com/VonHeikemen/lsp-zero.nvim/blob/v3.x/doc/md/guides/integrate-with-mason-nvim.md
require('mason').setup({
  ui = { border = "none", },
  log_level = vim.log.levels.INFO,
  max_concurrent_installers = 4,
})
require('mason-lspconfig').setup({
  automatic_installation = true,
  ensure_installed = servers,
  handlers = {
    lsp_zero.default_setup,
  },
})

local opts = {}
for _, server in pairs(servers) do
  opts = {
    capabilities = vim.lsp.protocol.make_client_capabilities(),
  }

  server = vim.split(server, "@")[1]

  local require_ok, conf_opts = pcall(require, "user.lsp.settings." .. server)
  if require_ok then
    opts = vim.tbl_deep_extend("force", conf_opts, opts)
  end

  require('lspconfig')[server].setup(opts)
end
