#pragma once
#include <windows.h>
#include <optional>
#include <string>

namespace WindowsNameDialog
{
    std::wstring Wide(const std::string& text);
    std::optional<std::string> Prompt(HWND owner, const wchar_t* title, const std::string& initial);
}
