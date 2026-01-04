local ok, harpoon = pcall(require, 'harpoon')
if not ok then
  return
end

--harpoon.setup()
harpoon:setup()
--require("telescope").load_extension('harpoon')

local status_ok, which_key = pcall(require, 'which-key')
if not status_ok then
  return
end

-- Whichkey mappings
local opts = {
  mode = 'n', -- NORMAL mode
  prefix = '<leader>',
  buffer = nil, -- Global mappings. Specify a buffer number for buffer local mappings
  silent = true, -- use `silent` when creating keymaps
  noremap = true, -- use `noremap` when creating keymaps
  nowait = true, -- use `nowait` when creating keymaps
}
local mappings = {
  h = {
    name = 'Harpoon',
    a = {
      function()
        harpoon:list():add()
      end,
      'Add file',
    },
    o = {
      function()
        harpoon.ui:toggle_quick_menu(harpoon:list())
      end,
      'Open Harpoon',
    },
    ['1'] = {
      function()
        harpoon:list():select(1)
      end,
      'Navigate to file 1',
    },
    ['2'] = {
      function()
        harpoon:list():select(2)
      end,
      'Navigate to file 2',
    },
    ['3'] = {
      function()
        harpoon:list():select(3)
      end,
      'Navigate to file 3',
    },
    ['4'] = {
      function()
        harpoon:list():select(4)
      end,
      'Navigate to file 4',
    },
    ['5'] = {
      function()
        harpoon:list():select(5)
      end,
      'Navigate to file 5',
    },
    ['6'] = {
      function()
        harpoon:list():select(6)
      end,
      'Navigate to file 6',
    },
    ['7'] = {
      function()
        harpoon:list():select(7)
      end,
      'Navigate to file 7',
    },
    ['8'] = {
      function()
        harpoon:list():select(8)
      end,
      'Navigate to file 8',
    },
    ['9'] = {
      function()
        harpoon:list():select(9)
      end,
      'Navigate to file 9',
    },
  },
}
which_key.register(mappings, opts)

local opts = {
  mode = 'n', -- NORMAL mode
  prefix = '',
  buffer = nil, -- Global mappings. Specify a buffer number for buffer local mappings
  silent = true, -- use `silent` when creating keymaps
  noremap = true, -- use `noremap` when creating keymaps
  nowait = true, -- use `nowait` when creating keymaps
}

vim.keymap.set('n', '<C-1>', function()
  harpoon:list():select(1)
end)

local mappings = {
  ['<A-n>'] = {
    function()
      harpoon:list():next()
    end,
    'Next harpoon file',
  },
  ['<A-p>'] = {
    function()
      harpoon:list():prev()
    end,
    'Previous harpoon file',
  },
}
which_key.register(mappings, opts)
