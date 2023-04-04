-- Get the tail of a path `path`, where it's split of at the string `chopstr`. For example,
--  PathTail('a/b/c/d/e/f', 'd') returns 'e/f'
function PathTail(path, chopstr)
  return string.gsub(path, ".*/" .. chopstr .. "/", "")
end

-- Strip off `include`, and everything before it from a path. This is used to generate the path
-- for a header guard
function ChopInclude(str)
  return PathTail(str, "include")
end

-- G++ Google standard compliant header guard.
-- The function `HeaderGuardName` is used by the `drmikehenry/vim-headerguard` package to generate
-- the header guard names.
-- This solution is inspired by the issue: https://github.com/drmikehenry/vim-headerguard/issues/2
--
-- Explanation of the `substitute` command:
--  Arg 1: Get current file path `%:p` and chop the `include` subpath and everything before it
--  Arg 2: Replace all chars that satisfy the regex expression `[^0-9a-zA-Z_]` (i.e., anything that
--         is NOT (`^`) a number (`0-9`), a lower case character (`a-z`), an upper case character
--         (`A-Z`), or a subscript (`_`))
--  Arg 3: The string to replace the chars with
--  Arg 3: `g` flag passed to `substitute` to replace ALL occurrences. Without this flag, only the
--         first occurrence will be replaced
function HeaderguardName()
  local filename = ChopInclude(vim.fn.expand('%:p'))
  local pattern = "[^0-9a-zA-Z_]"
  local replacement = "_"
  return string.upper(string.gsub(filename, pattern, replacement)) .. "_"
end

-- Mapping to generate guards
vim.keymap.set("n", vim.g.altleader .. "hg", ":HeaderguardAdd<cr>")

-- Use `//` style comments
vim.g.headerguard_use_cpp_comments = 1


