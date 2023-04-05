-- Check if git version is satisfied
local git_target_version = "2.31.0"
local git_version = io.popen("git --version | awk '{print $3}'"):read('*all')
git_version = string.gsub(git_version, '\n', '')
local major, minor, patch = string.gmatch(git_version, "(%d+).(%d+).(%d+)")()

-- Convert the version strings to arrays of integers
local function version_string_to_int_array(version_string)
    local version_parts = {}
    for part in string.gmatch(version_string, "%d+") do
        table.insert(version_parts, tonumber(part))
    end
    return version_parts
end

local version_parts = version_string_to_int_array(git_version)
local target_version_parts = version_string_to_int_array(git_target_version)

-- Compare the version parts
local function is_version_less_than(version_parts, target_version_parts)
    for i = 1, math.max(#version_parts, #target_version_parts) do
        local version_part = version_parts[i] or 0
        local target_version_part = target_version_parts[i] or 0
        if version_part < target_version_part then
            return true
        elseif version_part > target_version_part then
            return false
        end
    end
    return false
end


-- Function for calling diffview open if the git version is satisfied
local function diffviewopen()
    if is_version_less_than(version_parts, target_version_parts) then
      print("The git version " .. git_version .. " is less than the required version", git_target_version)
      return
    end

    vim.cmd("DiffviewOpen")
end

-- Open vimdiff
--vim.keymap.set("n", vim.g.altleader .. "gd", diffviewopen)

-- Ruler has column and AsyncRun status
--vim.o.rulerformat = '%60(%=%t\\ %c\\ %{g:asyncrun_status}%)'

