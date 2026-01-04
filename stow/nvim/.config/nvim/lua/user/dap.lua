local dap = require('dap')
local dapui = require('dapui')
dapui.setup()

-- Setup DAP adapters
require('dap-python').setup(result)
--local python_path = 'python3'
--dap.adapters.debugpy = function(cb, config)
--  if config.request == 'attach' then
--    error('Attach request not supported')
--  end
--  cb({
--    type = 'executable',
--    command = python_path,
--    args = { '-m', 'debugpy.adapter' },
--    options = {
--      source_filetype = 'python',
--    },
--  })
--end

-- Setup UI
dap.listeners.before.attach.dapui_config = function()
  dapui.open()
end
dap.listeners.before.launch.dapui_config = function()
  dapui.open()
end
dap.listeners.before.event_terminated.dapui_config = function()
  dapui.close()
end
dap.listeners.before.event_exited.dapui_config = function()
  dapui.close()
end

-- Icons
local codicons = require('codicons')
codicons.setup({})

vim.fn.sign_define('DapBreakpoint', {
  text = '🔴', -- 🛑🔴
  linehl = 'DapBreakpoint',
  numhl = 'DapBreakpoint',
})

vim.fn.sign_define('DapBreakpointCondition', {
  text = '🟡',
  linehl = 'DapBreakpointCondition',
  numhl = 'DapBreakpointCondition',
})

-- Trigger auto-completion automatically
vim.cmd([[ au FileType dap-repl lua require('dap.ext.autocompl').attach() ]])
