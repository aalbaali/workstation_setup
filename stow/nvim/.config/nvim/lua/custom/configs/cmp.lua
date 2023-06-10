local cmp = require "cmp"
local setup = require "plugins.configs.cmp"

local snip_status_ok, luasnip = pcall(require, "luasnip")
if not snip_status_ok then
  print "Couldn't load 'luasnip'"
  return
end

setup.mapping = {
  ["<C-p>"] = cmp.mapping.select_prev_item(),
  ["<C-n>"] = cmp.mapping.select_next_item(),
  ["<C-u>"] = cmp.mapping.scroll_docs(4),
  ["<C-d>"] = cmp.mapping.scroll_docs(-4),
  ["<C-b>"] = cmp.mapping(cmp.mapping.scroll_docs(-1), { "i", "c" }),
  ["<C-f>"] = cmp.mapping(cmp.mapping.scroll_docs(1), { "i", "c" }),
  ["<C-Space>"] = cmp.mapping(cmp.mapping.complete(), { "i", "c" }),
  ["<C-y>"] = cmp.config.disable, -- Specify `cmp.config.disable` if you want to remove the default `<C-y>` mapping.
  ["<C-e>"] = cmp.mapping {
    i = cmp.mapping.abort(),
    c = cmp.mapping.close(),
  },
  -- Accept currently selected item. If none selected, `select` first item.
  -- Set `select` to `false` to only confirm explicitly selected items.
  ["<Tab>"] = cmp.mapping(function(fallback)
    if cmp.get_selected_entry() then
      cmp.mapping.confirm { select = true }
      cmp.confirm()
      --elseif luasnip.expandable() then
      --  luasnip.expand()
    elseif luasnip.expand_or_jumpable() then
      luasnip.expand_or_jump()
    else
      fallback()
    end
  end, {
    "i",
    "s",
  }),
  ["<S-Tab>"] = cmp.mapping(function(fallback)
    if luasnip.jumpable(-1) then
      luasnip.jump(-1)
    else
      fallback()
    end
  end, {
    "i",
    "s",
  }),
}
return setup
