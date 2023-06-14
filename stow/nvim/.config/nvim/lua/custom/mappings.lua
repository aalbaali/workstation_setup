---@type MappingsTable
local M = {}

-- Alternate custom leader
vim.g.altleader = "\\"
local altleader = vim.g.altleader

M.disabled = {
  n = {
    [";"] = "",
    ["<leader>wk"] = "",
    ["<C-h>"] = "",
    ["<C-j>"] = "",
    ["<C-k>"] = "",
    ["<C-l>"] = "",
    ["<C-c>"] = "",
  },
  t = {
    ["<C-x>"] = "",
  },
}

local opts = {
  silent = true, -- use `silent` when creating keymaps
  noremap = true, -- use `noremap` when creating keymaps
  nowait = true, -- use `nowait` when creating keymaps
}

M.general = {
  n = {
    ["]t"] = { "<cmd>TagbarJumpNext<CR>", "Next tag" },
    ["[t"] = { "<cmd>TagbarJumpPrev<CR>", "Prev tag" },
    ["[q"] = { "<cmd>cnext<cr>", "Next quickfix" },
    ["]q"] = { "<cmd>cnext<cr>", "Prev quickfix" },

    ["<leader>b"] = {
      "<cmd>lua require('telescope.builtin').buffers(require('telescope.themes').get_dropdown{previewer = false})<cr>",
      "Buffers",
      opts,
    },
    ["<leader>e"] = { "<cmd>NvimTreeToggle<cr>", "Toggle explorer" },
    ["<leader>w"] = { "<cmd>w!<CR>", "Save" },
    ["<leader>X"] = { "<cmd>q!<CR>", "Quit without save" },
    ["<leader>q"] = { "<cmd>wqa<CR>", "Save all and quit" },
    ["<leader>c"] = { "<cmd>Bdelete!<CR>", "Close Buffer" },
    ["<leader>h"] = { "<cmd>nohlsearch<CR>", "No Highlight" },
    ["<leader>f"] = {
      "<cmd>lua require('telescope.builtin').find_files(require('telescope.themes').get_dropdown{previewer = false})<cr>",
      "Find files",
    },
    ["<leader>F"] = { "<cmd>Telescope live_grep theme=ivy<cr>", "Find Text" },
    ["<leader>P"] = { "<cmd>lua require('telescope').extensions.projects.projects()<cr>", "Projects" },
    ["<leader>rp"] = { "<cmd>split<CR><cmd>term python %<CR>", "Run" },

    ["gn"] = { "<cmd>bnext<cr>", "Next buffer" },
    ["gN"] = { "<cmd>bprev<cr>", "Previous buffer" },
    ["gL"] = { "<cmd>blast<cr>", "Last buffer" },

    -- Tagbar
    [altleader .. "tt"] = { "<cmd>TagbarToggle f<CR>", "Toggle tagbar" },
    [altleader .. "to"] = { "<cmd>TagbarOpen f<CR>", "Open tagbar" },
    [altleader .. "tc"] = { "<cmd>TagbarClose<CR>", "Close tagbar" },

    -- Quickfix
    [altleader .. "qq"] = { "<cmd>copen<CR>", "Open quick fix" },
    [altleader .. "qc"] = { "<cmd>cclose<CR>", "Close quick fix" },
    [altleader .. "u"] = { "<cmd>lua vim.cmd.UndotreeToggle()<CR>", "Undo tree" },
  },
}

M.git = {
  n = {
    ["<leader>gg"] = { "<cmd>lua _LAZYGIT_TOGGLE()<CR>", "Lazygit" },
    ["<leader>gj"] = { "<cmd>lua require 'gitsigns'.next_hunk()<cr>", "Next Hunk" },
    ["<leader>gk"] = { "<cmd>lua require 'gitsigns'.prev_hunk()<cr>", "Prev Hunk" },
    ["<leader>gl"] = { "<cmd>lua require 'gitsigns'.blame_line()<cr>", "Blame" },
    ["<leader>gp"] = { "<cmd>lua require 'gitsigns'.preview_hunk()<cr>", "Preview Hunk" },
    ["<leader>gr"] = { "<cmd>lua require 'gitsigns'.reset_hunk()<cr>", "Reset Hunk" },
    ["<leader>gR"] = { "<cmd>lua require 'gitsigns'.reset_buffer()<cr>", "Reset Buffer" },
    ["<leader>gs"] = { "<cmd>lua require 'gitsigns'.stage_hunk()<cr>", "Stage Hunk" },
    ["<leader>gu"] = {
      "<cmd>lua require 'gitsigns'.undo_stage_hunk()<cr>",
      "Undo Stage Hunk",
    },
    ["<leader>go"] = { "<cmd>Telescope git_status<cr>", "Open changed file" },
    ["<leader>gb"] = { "<cmd>Telescope git_branches<cr>", "Checkout branch" },
    ["<leader>gc"] = { "<cmd>Telescope git_commits<cr>", "Checkout commit" },
    ["<leader>gd"] = {
      "<cmd>Gitsigns diffthis HEAD<cr>",
      "Diff",
    },

    -- altleader
    [altleader .. "gb"] = { "<cmd>G blame<CR>", "Git blame" },
    [altleader .. "gc"] = { "<cmd>DiffviewClose<CR>", "Close diffview" },
    [altleader .. "gd"] = { "<cmd>DiffviewOpen<CR>", "Diff with current tree" },
    [altleader .. "gf"] = { "<cmd>DiffviewToggleFiles<CR>", "Toggle files sidebar" },
    [altleader .. "gh"] = { "<cmd>DiffviewFileHistory<CR>", "File history" },
    [altleader .. "gl"] = { "<cmd>DiffviewLog<CR>", "Git log" },
    [altleader .. "gm"] = { "<cmd>Git commit -s<CR>", "Git commit" },
    [altleader .. "gp"] = { "<cmd>Git push<CR>", "Git push" },
    [altleader .. "gs"] = { "<cmd>lua vim.cmd.Git()<CR>", "Git status" },
    [altleader .. "hs"] = { "<cmd>lua require 'gitsigns'.stage_hunk()<cr>", "Hunk stage" },
  },
}

M.lsp = {
  n = {
    ["<leader>la"] = { "<cmd>lua vim.lsp.buf.code_action()<cr>", "Code Action" },
    ["<leader>ld"] = {
      "<cmd>Telescope diagnostics bufnr=0<cr>",
      "Document Diagnostics",
    },
    ["<leader>lw"] = {
      "<cmd>Telescope diagnostics<cr>",
      "Workspace Diagnostics",
    },
    ["<leader>lf"] = { "<cmd>lua vim.lsp.buf.format{async=true}<cr>", "Format" },
    ["<leader>li"] = { "<cmd>LspInfo<cr>", "Info" },
    ["<leader>lI"] = { "<cmd>Mason<cr>", "Installer Info" },
    ["<leader>lj"] = {
      "<cmd>lua vim.diagnostic.goto_next()<CR>",
      "Next Diagnostic",
    },
    ["<leader>lk"] = {
      "<cmd>lua vim.diagnostic.goto_prev()<cr>",
      "Prev Diagnostic",
    },
    ["<leader>ll"] = { "<cmd>lua vim.lsp.codelens.run()<cr>", "CodeLens Action" },
    ["<leader>lq"] = { "<cmd>lua vim.diagnostic.setloclist()<cr>", "Quickfix" },
    ["<leader>lr"] = { "<cmd>lua vim.lsp.buf.rename()<cr>", "Rename" },
    ["<leader>ls"] = { "<cmd>Telescope lsp_document_symbols<cr>", "Document Symbols" },
    ["<leader>lS"] = {
      "<cmd>Telescope lsp_dynamic_workspace_symbols<cr>",
      "Workspace Symbols",
    },
  },
}

M.markdow = {
  n = {
    ["<leader>mp"] = { "<cmd>MarkdownPreview<cr>", "Preview" },
    ["<leader>mP"] = { "<cmd>MarkdownPreviewStop<cr>", "Stop Preview" },
    ["<leader>mt"] = { "<cmd>MarkdownPreviewToggle<cr>", "Toggle Markdown preview" },
  },
}

M.search = {
  n = {
    ["]c"] = { "<cmd>lua require 'gitsigns'.next_hunk()<cr>", "Next Hunk" },
    ["[c"] = { "<cmd>lua require 'gitsigns'.prev_hunk()<cr>", "Prev Hunk" },

    ["<leader>sa"] = { "<cmd>Ag<cr>", "Ag search" },
    ["<leader>sb"] = { "<cmd>Telescope git_branches<cr>", "Checkout branch" },
    ["<leader>sc"] = { "<cmd>Telescope colorscheme<cr>", "Colorscheme" },
    ["<leader>sh"] = { "<cmd>Telescope help_tags<cr>", "Find Help" },
    ["<leader>sM"] = { "<cmd>Telescope man_pages<cr>", "Man Pages" },
    ["<leader>sr"] = { "<cmd>Telescope oldfiles<cr>", "Open Recent File" },
    ["<leader>sR"] = { "<cmd>Telescope registers<cr>", "Registers" },
    ["<leader>sk"] = { "<cmd>Telescope keymaps<cr>", "Keymaps" },
    ["<leader>sC"] = { "<cmd>Telescope commands<cr>", "Commands" },
  },
}

M.terminal = {
  n = {
    ["<leader>tn"] = { "<cmd>lua _NODE_TOGGLE()<cr>", "Node" },
    ["<leader>tu"] = { "<cmd>lua _NCDU_TOGGLE()<cr>", "NCDU" },
    ["<leader>tt"] = { "<cmd>lua _HTOP_TOGGLE()<cr>", "Htop" },
    ["<leader>tp"] = { "<cmd>lua _PYTHON_TOGGLE()<cr>", "Python" },
    ["<leader>tj"] = { "<cmd>lua _JULIA_TOGGLE()<cr>", "Julia" },
    ["<leader>tf"] = { "<cmd>ToggleTerm direction=float<cr>", "Float" },
    ["<leader>th"] = { "<cmd>ToggleTerm size=10 direction=horizontal<cr>", "Horizontal" },
    ["<leader>tv"] = { "<cmd>ToggleTerm size=80 direction=vertical<cr>", "Vertical" },
  },
}

-- Apply to both normal and visual modes
M.documentation = {
  n = {
    [altleader .. "c"] = { "<cmd>Telescope commands<CR>", "Find commands", opts },
    [altleader .. "da"] = { ":DoxAuth<CR>", "Doxygen author (documents)" },
    [altleader .. "dc"] = { "O//<space>", "C++ inline comment" },
    [altleader .. "dd"] = { ":Dox<CR>", "Doxygen documentation" },
    [altleader .. "dg"] = { ":DogeGenerate<CR>", "DoGe documentation" },
    [altleader .. "ds"] = { "O/**<space><space>*/<Esc>F<space>i", "C++ inline member comment" },
  },
  v = {
    [altleader .. "c"] = { "<cmd>Telescope commands<CR>", "Find commands", opts },
    [altleader .. "da"] = { ":DoxAuth<CR>", "Doxygen author (documents)" },
    [altleader .. "dc"] = { "O//<space>", "C++ inline comment" },
    [altleader .. "dd"] = { ":Dox<CR>", "Doxygen documentation" },
    [altleader .. "dg"] = { ":DogeGenerate<CR>", "DoGe documentation" },
    [altleader .. "ds"] = { "O/**<space><space>*/<Esc>F<space>i", "C++ inline member comment" },
  },
}

M.tab = {
  n = {
    [altleader .. "tn"] = { "<cmd>tabnext<cr>", "Next tab" },
    [altleader .. "tc"] = { "<cmd>tabprev<cr>", "Prev tab" },
    [altleader .. "tC"] = { "<cmd>tabclose<cr>", "Close tab" },
    [altleader .. "td"] = { "<cmd>TodoQuickFix<cr>", "Populate quickfix with TODOs" },
  },
}

M.fzf = {
  n = {
    [vim.g.altleader .. "o"] = { ":Files<CR>", "Files" },
    [vim.g.altleader .. "i"] = { ":Buffers<CR>", "Buffers" },
    [vim.g.altleader .. "l"] = { ":BLines<CR>", "Search buffer lines" },
    [vim.g.altleader .. "L"] = { ":Lines<CR>", "Search lines" },
    [vim.g.altleader .. "a"] = { ":Ag<space>", "Ag search" },
    [vim.g.altleader .. "p"] = { ":History<CR>", "Search history" },
    [vim.g.altleader .. ":"] = { ":History:<CR>", "Command history" },
    [vim.g.altleader .. "/"] = { ":History/<CR>", "Search history" },
    [vim.g.altleader .. "w"] = { ":Windows<CR>", "Windows" },
    [vim.g.altleader .. "ts"] = { ":BTags<CR>", "Search buffer tags" },
    [vim.g.altleader .. "eo"] = {
      ":call fzf#vim#files('.', {'options':'--query '.expand('<cword>')})<CR>",
      "Search files",
    },
    [vim.g.altleader .. "eO"] = {
      ":call fzf#vim#files('.', {'options':'--query '.expand('<cWORD>')})<CR>",
      "Search files",
    },
    [vim.g.altleader .. "ei"] = {
      ":call fzf#vim#buffers('.', {'options':'--query '.expand('<cword>')})<CR>",
      "Search buffers",
    },
    [vim.g.altleader .. "*"] = { ":execute ':Ag ' . expand('<cword>')<CR>", "Search word" },
    [vim.g.altleader .. "ff"] = { ":call FZFSameName('e', '', '')<CR>", "Find file" },
    [vim.g.altleader .. "fh"] = { ":call FZFSameName('e', 'wincmd h', '')<CR>", "Find file" },
    [vim.g.altleader .. "fl"] = { ":call FZFSameName('e', 'wincmd l', '')<CR>", "Find file" },
    [vim.g.altleader .. "fk"] = { ":call FZFSameName('e', 'wincmd k', '')<CR>", "Find file" },
    [vim.g.altleader .. "fj"] = { ":call FZFSameName('e', 'wincmd j', '')<CR>", "Find file" },
    [vim.g.altleader .. "fH"] = { ":call FZFSameName('leftabove vsplit', '', 'wincmd h')<CR>", "Find file" },
    [vim.g.altleader .. "fL"] = { ":call FZFSameName('rightbelow vsplit', '', 'wincmd l')<CR>", "Find file" },
    [vim.g.altleader .. "fK"] = { ":call FZFSameName('leftabove split', '', 'wincmd k')<CR>", "Find file" },
    [vim.g.altleader .. "fJ"] = { ":call FZFSameName('rightbelow split', '', 'wincmd j')<CR>", "Find file" },
    [vim.g.altleader .. "sf"] = { ":Ag <C-R>=expand('%:t')<CR><CR>", "Search file" },
  },
}

M.build = {
  ["n"] = {
    [vim.g.altleader .. "kb"] = { ":AsyncRun -cwd=<root> catkin build<CR>", "Build catkin workspace" },
    [vim.g.altleader .. "kt"] = {
      ":AsyncRun -cwd=<root> catkin build --make-args tests<CR>",
      "Build catkin workspace tests",
    },
    [vim.g.altleader .. "kn"] = { ":AsyncStop<CR>", "Stop asyncrun" },
    [vim.g.altleader .. "<space>"] = { ":call asyncrun#quickfix_toggle(20},<CR>", "Toggle async quickfix" },
  },
}

-- more keybinds!

return M
