local status_ok, configs = pcall(require, "nvim-treesitter.configs")
if not status_ok then
	return
end

configs.setup({
	ensure_installed = {
		"lua",
		"bash",
		"c",
		"cpp",
		"json",
		"lua",
		"python",
		"css",
		"yaml",
		"markdown",
		"markdown_inline",
		"vimdoc",
		"luadoc",
		"vim",
	}, -- one of "all" or a list of languages
	ignore_install = { "phpdoc" }, -- List of parsers to ignore installing
	highlight = {
		enable = true, -- false will disable the whole extension
		disable = { "css" }, -- list of language that will be disabled
	},
	autopairs = {
		enable = true,
	},
	indent = { enable = true, disable = { "css" } },
})
