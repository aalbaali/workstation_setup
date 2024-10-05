local null_ls_status_ok, null_ls = pcall(require, "null-ls")
if not null_ls_status_ok then
  return
end

-- https://github.com/jose-elias-alvarez/null-ls.nvim/tree/main/lua/null-ls/builtins/formatting
local formatting = null_ls.builtins.formatting
-- https://github.com/jose-elias-alvarez/null-ls.nvim/tree/main/lua/null-ls/builtins/diagnostics
local diagnostics = null_ls.builtins.diagnostics

null_ls.setup({
  debug = false,
  sources = {
    formatting.ruff,
    formatting.stylua,
    formatting.clang_format,
    formatting.cmake_format,
    formatting.rustfmt,
    formatting.beautysh,
    diagnostics.mypy,
    diagnostics.clang_check,
    diagnostics.cmake_lint,
    diagnostics.shellcheck,
    diagnostics.zsh,
    diagnostics.ansiblelint,
  },
})
