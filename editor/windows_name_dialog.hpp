#pragma once
#include <windows.h>
#include <optional>
#include <string>

namespace WindowsNameDialog
{
    std::string Utf8(const std::wstring& text);
    std::wstring Wide(const std::string& text);
    std::optional<std::string> Prompt(HWND owner, const wchar_t* title, const std::string& initial);
}
