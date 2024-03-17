conform = require("conform")

local M = {}
M.setup = function(_)
  conform.setup({
    formatters_by_ft = {
      lua = { "stylua" },
      python = { "isort", "black" },
      javascript = { { "prettierd", "prettier" } },
      cpp = { "clang-format" },
    },
  })
end

return M
