local status_ok, pkg = pcall(require, "neogen")
if not status_ok then
  return
end

pkg.setup {
  snippet_engine = "luasnip",
  languages = {
    python = {
      template = {
        annotation_convention = "google_docstrings",
      }
    },
  }
}
