local status_ok, gpt = pcall(require, "chatgpt")
if not status_ok then
  print("Couldn't load 'cmp'")
  return
end

gpt.setup({
  --api_key_cmd = "gpg --decrypt ~/.chat_gpt_secret.txt 2>/dev/null"
  api_key_cmd = "cat ~/.chat_gpt_secret.txt"
})
