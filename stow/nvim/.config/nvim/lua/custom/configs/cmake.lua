-- Variables
vim.g.cmake_link_complie_commands = 1
vim.g.cmake_default_config = 'build'
vim.g.cmake_jump=1
vim.g.cmake_jump_on_completion=1
-- Open CMake window

-- CMake build
vim.cmd [[
  autocmd BufRead *.cpp nmap \cg : CMakeGenerate<cr>
  autocmd BufRead *.cpp,CMakeLists.txt nmap \cb : CMakeBuild<cr>
]]

-- Show CMake version in status line
vim.cmd [[set statusline=%{cmake#GetInfo().cmake_version.string}]]
