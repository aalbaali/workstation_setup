return {
  settings = {
    pylsp = {
      plugins = {
        -- formatter options
        black = { enabled = false },
        ruff = { enabled = false }, -- Enabled separately
        autopep8 = { enabled = false },
        yapf = { enabled = false },

        -- linter options
        pylint = { enabled = false, executable = 'pylint' },
        pyflakes = { enabled = false },
        pycodestyle = { enabled = false },

        -- type checker
        pylsp_mypy = { enabled = false },

        -- auto-completion options
        jedi_completion = { fuzzy = false },

        -- import sorting
        pyls_isort = { enabled = true },
      },
    },
  },
}
