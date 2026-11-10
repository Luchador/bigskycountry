#include "windows_name_dialog.hpp"
#include "windows_theme.hpp"
#include "panel_ids.h"

namespace
{
    struct NameEdit
    {
        std::wstring title, value;
    };
    INT_PTR CALLBACK NameDialog(HWND dlg, UINT msg, WPARAM w, LPARAM l)
    {
        if (msg == WM_INITDIALOG)
        {
            auto* edit = reinterpret_cast<NameEdit*>(l);
            SetWindowLongPtrW(dlg, DWLP_USER, l);
            WindowsTheme::ApplyDialog(dlg);
            SetWindowTextW(dlg, edit->title.c_str());
            SetDlgItemTextW(dlg, IDC_NAME, edit->value.c_str());
            SendDlgItemMessageW(dlg, IDC_NAME, EM_SETLIMITTEXT, 255, 0);
            SendDlgItemMessageW(dlg, IDC_NAME, EM_SETSEL, 0, -1);
            return TRUE;
        }
        if (msg == WM_COMMAND && LOWORD(w) == IDOK)
        {
            wchar_t name[256]{};
            GetDlgItemTextW(dlg, IDC_NAME, name, 256);
            if (!*name)
            {
                return TRUE;
            }
            reinterpret_cast<NameEdit*>(GetWindowLongPtrW(dlg, DWLP_USER))->value = name;
            EndDialog(dlg, IDOK);
            return TRUE;
        }
        if (msg == WM_CLOSE || (msg == WM_COMMAND && LOWORD(w) == IDCANCEL))
        {
            EndDialog(dlg, IDCANCEL);
            return TRUE;
        }
        return FALSE;
    }
} // namespace

namespace WindowsNameDialog
{
    std::wstring Wide(const std::string& s)
    {
        int n = MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), nullptr, 0);
        std::wstring w(n, 0);
        if (n)
        {
            MultiByteToWideChar(CP_UTF8, 0, s.data(), static_cast<int>(s.size()), w.data(), n);
        }
        return w;
    }
    std::optional<std::string> Prompt(HWND owner, const wchar_t* title, const std::string& initial)
    {
        NameEdit edit{title, Wide(initial)};
        if (DialogBoxParamW(GetModuleHandleW(nullptr), MAKEINTRESOURCEW(IDD_NAME), owner, NameDialog,
                            reinterpret_cast<LPARAM>(&edit)) != IDOK) return {};
        const int count = WideCharToMultiByte(CP_UTF8, 0, edit.value.data(), static_cast<int>(edit.value.size()),
                                              nullptr, 0, nullptr, nullptr);
        std::string text(count, 0);
        WideCharToMultiByte(CP_UTF8, 0, edit.value.data(), static_cast<int>(edit.value.size()), text.data(), count,
                            nullptr, nullptr);
        return text;
    }
}
