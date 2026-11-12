#include "windows_outliner.hpp"
#include "windows_theme.hpp"
#include "windows_name_dialog.hpp"
#include "panel_ids.h"
#include <commctrl.h>
#include <windowsx.h>
#include <algorithm>

using WindowsNameDialog::Wide;

HWND WindowsOutliner::Create(HWND frame, HINSTANCE module)
{
    eventType = SDL_RegisterEvents(1);
    if (!eventType)
    {
        return nullptr;
    }
    panel = CreateDialogParamW(module, MAKEINTRESOURCEW(IDD_OUTLINER), frame, Dialog, reinterpret_cast<LPARAM>(this));
    if (!panel)
    {
        return nullptr;
    }
    list = GetDlgItem(panel, IDC_OUTLINE_LIST);
    ListView_SetExtendedListViewStyle(list, LVS_EX_FULLROWSELECT | LVS_EX_DOUBLEBUFFER);
    LVCOLUMNW col{};
    col.mask = LVCF_TEXT | LVCF_WIDTH;
    col.cx = 170;
    col.pszText = const_cast<wchar_t*>(L"Item label");
    ListView_InsertColumn(list, 0, &col);
    col.cx = 75;
    col.pszText = const_cast<wchar_t*>(L"Type");
    ListView_InsertColumn(list, 1, &col);
    checkIcon.reset(LoadOutlineMoveIcon(true));
    blockIcon.reset(LoadOutlineMoveIcon(false));
    badge = CreateWindowExW(WS_EX_TOOLWINDOW | WS_EX_NOACTIVATE | WS_EX_TRANSPARENT, L"STATIC", L"",
                            WS_POPUP | WS_DISABLED, 0, 0, 0, 0, frame, nullptr, module, nullptr);
    if (badge && !SetWindowSubclass(badge, BadgeProc, 1, reinterpret_cast<DWORD_PTR>(this)))
    {
        DestroyWindow(badge);
        badge = nullptr;
    }
    return panel;
}

void WindowsOutliner::Layout(int x, int y, int width, int height)
{
    MoveWindow(panel, x, y, width, height, TRUE);
    RECT inset{0, 0, 5, 22};
    MapDialogRect(panel, &inset);
    const int w = std::max(1, width - 2 * static_cast<int>(inset.right));
    MoveWindow(list, inset.right, inset.bottom, w, std::max(1, height - static_cast<int>(inset.bottom) - 2), TRUE);
    ListView_SetColumnWidth(list, 0, w * 2 / 3);
    ListView_SetColumnWidth(list, 1, std::max(36, w / 3 - GetSystemMetrics(SM_CXVSCROLL)));
}

std::set<SceneKey> WindowsOutliner::Selected() const
{
    std::set<SceneKey> result;
    for (int i = -1; (i = ListView_GetNextItem(list, i, LVNI_SELECTED)) != -1;)
    {
        if (i < static_cast<int>(rows.size()))
        {
            result.insert(rows[i].key);
        }
    }
    return result;
}

void WindowsOutliner::Sync(const SceneOutliner& data)
{
    model = &data;
    const bool rebuild = version != data.Version();
    if (!rebuild && displayed == data.Selection())
    {
        return;
    }
    syncing = true;
    SendMessageW(list, WM_SETREDRAW, FALSE, 0);
    if (rebuild)
    {
        if (GetCapture() == panel)
        {
            ReleaseCapture();
        }
        version = data.Version();
        rows = data.Rows();
        ListView_DeleteAllItems(list);
        for (int i = 0; i < static_cast<int>(rows.size()); ++i)
        {
            const auto& row = rows[i];
            std::wstring label(std::min(row.depth, 32) * 3, L' ');
            label += row.branch ? (row.expanded ? L"\u25BE " : L"\u25B8 ") : L"   ";
            label += Wide(row.label);
            LVITEMW item{};
            item.mask = LVIF_TEXT;
            item.iItem = i;
            item.pszText = label.data();
            ListView_InsertItem(list, &item);
            auto type = Wide(row.type);
            ListView_SetItemText(list, i, 1, type.data());
        }
    }
    displayed = data.Selection();
    // Native Ctrl/Shift selection already has a range anchor and a possible drag in progress.
    if (rebuild || Selected() != displayed)
    {
        ListView_SetItemState(list, -1, 0, LVIS_SELECTED);
        int first = -1;
        for (int i = 0; i < static_cast<int>(rows.size()); ++i)
        {
            if (!displayed.contains(rows[i].key))
            {
                continue;
            }
            ListView_SetItemState(list, i, LVIS_SELECTED, LVIS_SELECTED);
            if (first < 0)
            {
                first = i;
            }
        }
        if (first >= 0)
        {
            ListView_EnsureVisible(list, first, FALSE);
        }
    }
    const bool folder = displayed.size() == 1 && displayed.begin()->kind == SceneKind::Folder;
    EnableWindow(GetDlgItem(panel, IDC_FOLDER_RENAME), folder);
    EnableWindow(GetDlgItem(panel, IDC_FOLDER_DELETE), folder);
    SendMessageW(list, WM_SETREDRAW, TRUE, 0);
    InvalidateRect(list, nullptr, TRUE);
    syncing = false;
}

bool WindowsOutliner::QueueTerrainFill(bool repeat)
{
    auto keys = Selected();
    if (!IsRoadGroup(keys)) return false;
    if (!repeat) Queue({OutlineOp::FillTerrain, {}, std::move(keys)});
    return true;
}

void WindowsOutliner::Queue(OutlineCommand cmd)
{
    SDL_Event e{};
    e.type = eventType;
    pending.push_back(std::move(cmd));
    if (!SDL_PushEvent(&e))
    {
        pending.pop_back();
    }
}

std::optional<OutlineCommand> WindowsOutliner::Take(const SDL_Event& event)
{
    if (event.type != eventType || pending.empty())
    {
        return {};
    }
    auto result = std::move(pending.front());
    pending.pop_front();
    return result;
}

void WindowsOutliner::Name(bool create)
{
    if (!model)
    {
        return;
    }
    const auto sel = Selected();
    SceneKey target{};
    std::string initial;
    if (create)
    {
        if (sel.size() == 1)
        {
            const auto key = *sel.begin();
            if (key.kind == SceneKind::Folder)
            {
                target = key;
            }
            else
            {
                auto row = std::find_if(rows.begin(), rows.end(), [&](const auto& r) { return r.key == key; });
                if (row != rows.end())
                {
                    const int depth = row->depth;
                    while (row != rows.begin() && row->depth >= depth)
                    {
                        --row;
                    }
                    target = row->key;
                }
            }
        }
        initial = model->NewFolderName();
    }
    else
    {
        if (sel.size() != 1 || sel.begin()->kind != SceneKind::Folder)
        {
            return;
        }
        target = *sel.begin();
        for (const auto& r : rows)
        {
            if (r.key == target)
            {
                initial = r.label;
            }
        }
    }
    if (auto text = WindowsNameDialog::Prompt(GetParent(panel), create ? L"New folder" : L"Rename folder", initial))
        Queue({create ? OutlineOp::Create : OutlineOp::Rename, target, {}, *text});
    SetFocus(list);
}

LRESULT CALLBACK WindowsOutliner::BadgeProc(HWND hwnd, UINT msg, WPARAM w, LPARAM l, UINT_PTR, DWORD_PTR data)
{
    if (msg == WM_NCHITTEST) return HTTRANSPARENT;
    if (msg == WM_ERASEBKGND) return 1;
    if (msg != WM_PAINT) return DefSubclassProc(hwnd, msg, w, l);

    const auto& s = *reinterpret_cast<WindowsOutliner*>(data);
    PAINTSTRUCT paint{};
    HDC dc = BeginPaint(hwnd, &paint);
    RECT box{};
    GetClientRect(hwnd, &box);
    FillRect(dc, &box, WindowsTheme::Brush(EditorTheme::panel));
    FrameRect(dc, &box, WindowsTheme::Brush(EditorTheme::border));
    auto font = SelectObject(dc, reinterpret_cast<HFONT>(SendMessageW(s.list, WM_GETFONT, 0, 0)));
    SetBkMode(dc, TRANSPARENT);
    SetTextColor(dc, WindowsTheme::Color(EditorTheme::text));
    const int top = (box.bottom - s.iconSize) / 2;
    RECT icon{s.padding, top, s.padding + s.iconSize, top + s.iconSize};
    auto* surface = s.drop >= 0 ? s.checkIcon.get() : s.blockIcon.get();
    if (surface) WindowsTheme::DrawPreview(dc, icon, surface, EditorTheme::panel);
    else DrawTextW(dc, s.drop >= 0 ? L"\u2713" : L"\u00D7", 1, &icon, DT_SINGLELINE | DT_VCENTER | DT_CENTER);
    box.left = icon.right + s.padding;
    box.right -= s.padding;
    DrawTextW(dc, s.dragLabel.c_str(), -1, &box, DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX);
    SelectObject(dc, font);
    EndPaint(hwnd, &paint);
    return 0;
}

void WindowsOutliner::BeginDrag(int row)
{
    if (!model || row < 0 || row >= static_cast<int>(rows.size())) return;
    auto items = Selected();
    if (!items.contains(rows[row].key) || items.contains({})) return;
    dragged = std::move(items);
    dragLabel = Wide(rows[row].DragLabel(dragged.size()));
    const int dpi = GetDpiForWindow(list);
    iconSize = MulDiv(16, dpi, 96);
    padding = MulDiv(3, dpi, 96);
    HDC dc = GetDC(list);
    auto font = SelectObject(dc, reinterpret_cast<HFONT>(SendMessageW(list, WM_GETFONT, 0, 0)));
    SIZE text{};
    GetTextExtentPoint32W(dc, dragLabel.c_str(), static_cast<int>(dragLabel.size()), &text);
    SelectObject(dc, font);
    ReleaseDC(list, dc);
    badgeSize = {std::min<LONG>(text.cx, MulDiv(320, dpi, 96)) + iconSize + 3 * padding,
                 std::max<LONG>(text.cy, iconSize) + 2 * padding};
    SetCapture(panel);
    if (GetCapture() != panel) { EndDrag(); return; }
    POINT cursor{};
    GetCursorPos(&cursor);
    ScreenToClient(panel, &cursor);
    Hover(cursor);
    if (badge) InvalidateRect(badge, nullptr, FALSE);
}

void WindowsOutliner::EndDrag()
{
    dragged.clear();
    if (badge) ShowWindow(badge, SW_HIDE);
    if (drop >= 0) ListView_SetItemState(list, drop, 0, LVIS_DROPHILITED);
    drop = -1;
    if (GetCapture() == panel) ReleaseCapture();
}

void WindowsOutliner::Hover(POINT p)
{
    POINT cursor = p;
    ClientToScreen(panel, &cursor);
    MapWindowPoints(panel, list, &p, 1);
    RECT area{};
    GetClientRect(list, &area);
    const bool inside = PtInRect(&area, p) && WindowFromPoint(cursor) == list;
    if (inside)
    {
        if (p.y < 28)
        {
            ListView_Scroll(list, 0, -16);
        }
        if (p.y > area.bottom - 12)
        {
            ListView_Scroll(list, 0, 16);
        }
    }
    LVHITTESTINFO hit{};
    hit.pt = p;
    int next = inside ? ListView_SubItemHitTest(list, &hit) : -1;
    if (next >= 0 && (next >= static_cast<int>(rows.size()) || !model->CanDrop(dragged, rows[next].key)))
    {
        next = -1;
    }
    if (drop != next)
    {
        if (drop >= 0)
        {
            ListView_SetItemState(list, drop, 0, LVIS_DROPHILITED);
        }
        if (badge && (drop >= 0) != (next >= 0)) InvalidateRect(badge, nullptr, FALSE);
        drop = next;
        if (drop >= 0)
        {
            ListView_SetItemState(list, drop, LVIS_DROPHILITED, LVIS_DROPHILITED);
        }
    }
    if (badge)
    {
        const int gap = 4 * padding;
        int x = cursor.x + gap, y = cursor.y + gap;
        MONITORINFO monitor{sizeof(monitor)};
        if (GetMonitorInfoW(MonitorFromPoint(cursor, MONITOR_DEFAULTTONEAREST), &monitor))
        {
            if (x + badgeSize.cx > monitor.rcWork.right) x = std::max(monitor.rcWork.left, cursor.x - gap - badgeSize.cx);
            if (y + badgeSize.cy > monitor.rcWork.bottom) y = std::max(monitor.rcWork.top, cursor.y - gap - badgeSize.cy);
        }
        SetWindowPos(badge, HWND_TOP, x, y, badgeSize.cx, badgeSize.cy, SWP_NOACTIVATE | SWP_SHOWWINDOW);
    }
}

INT_PTR CALLBACK WindowsOutliner::Dialog(HWND hwnd, UINT msg, WPARAM w, LPARAM l)
{
    if (msg == WM_INITDIALOG)
    {
        SetWindowLongPtrW(hwnd, DWLP_USER, l);
        WindowsTheme::ApplyDialog(hwnd);
        return FALSE;
    }
    auto* self = reinterpret_cast<WindowsOutliner*>(GetWindowLongPtrW(hwnd, DWLP_USER));
    if (!self || !self->list)
    {
        return FALSE;
    }
    auto& s = *self;
    const auto remove = [&]()
    {
        const auto sel = s.Selected();
        if (sel.size() == 1 && sel.begin()->kind == SceneKind::Folder)
        {
            s.Queue({OutlineOp::Remove, *sel.begin()});
        }
    };
    if (msg == WM_COMMAND && HIWORD(w) == BN_CLICKED)
    {
        switch (LOWORD(w))
        {
        case IDC_FOLDER_NEW:
            s.Name(true);
            return TRUE;
        case IDC_FOLDER_RENAME:
            s.Name(false);
            return TRUE;
        case IDC_FOLDER_DELETE:
            remove();
            SetFocus(s.list);
            return TRUE;
        }
    }
    if (msg == WM_NOTIFY && !s.syncing && reinterpret_cast<NMHDR*>(l)->hwndFrom == s.list)
    {
        const auto code = reinterpret_cast<NMHDR*>(l)->code;
        if (code == LVN_ITEMCHANGED)
        {
            const auto* n = reinterpret_cast<NMLISTVIEW*>(l);
            if ((n->uChanged & LVIF_STATE) && ((n->uNewState ^ n->uOldState) & LVIS_SELECTED))
            {
                s.Queue({OutlineOp::Select, {}, s.Selected()});
            }
        }
        else if (code == LVN_BEGINDRAG)
        {
            s.BeginDrag(reinterpret_cast<NMLISTVIEW*>(l)->iItem);
        }
        else if (code == NM_CLICK || code == NM_DBLCLK)
        {
            const auto* hit = reinterpret_cast<NMITEMACTIVATE*>(l);
            const int i = hit->iItem;
            if (code == NM_CLICK && i >= 0 && s.rows[i].key.kind == SceneKind::Road)
            {
                // Clicking an already highlighted road switches from its point or segment to the whole road.
                s.Queue({OutlineOp::Select, {}, s.Selected()});
            }
            if (i >= 0 && s.rows[i].branch && hit->iSubItem == 0)
            {
                RECT label{};
                ListView_GetItemRect(s.list, i, &label, LVIR_LABEL);
                HDC dc = GetDC(s.list);
                auto oldFont = SelectObject(dc, reinterpret_cast<HFONT>(SendMessageW(s.list, WM_GETFONT, 0, 0)));
                SIZE space{}, glyph{};
                GetTextExtentPoint32W(dc, L" ", 1, &space);
                GetTextExtentPoint32W(dc, L"\u25BE ", 2, &glyph);
                SelectObject(dc, oldFont);
                ReleaseDC(s.list, dc);
                const int end = label.left + 4 + std::min(s.rows[i].depth, 32) * 3 * space.cx + glyph.cx;
                const bool onArrow = hit->ptAction.x < end;
                if ((code == NM_CLICK && onArrow) || (code == NM_DBLCLK && !onArrow))
                {
                    s.Queue({OutlineOp::Expand, s.rows[i].key});
                }
            }
        }
        else if (code == LVN_KEYDOWN)
        {
            const auto vk = reinterpret_cast<NMLVKEYDOWN*>(l)->wVKey;
            if (vk == VK_ESCAPE && GetCapture() == hwnd)
            {
                ReleaseCapture();
            }
            if (vk == VK_DELETE)
            {
                const auto selected = s.Selected();
                if (selected.size() == 1 && (selected.begin()->kind == SceneKind::Object ||
                    selected.begin()->kind == SceneKind::Intersection || selected.begin()->kind == SceneKind::Terrain))
                    s.Queue({OutlineOp::Remove, *selected.begin()});
                else
                    remove();
            }
            if (vk == VK_F2)
            {
                s.Name(false);
            }
            const int i = ListView_GetNextItem(s.list, -1, LVNI_FOCUSED);
            if (i >= 0 && s.rows[i].branch &&
                ((vk == VK_LEFT && s.rows[i].expanded) || (vk == VK_RIGHT && !s.rows[i].expanded)))
            {
                s.Queue({OutlineOp::Expand, s.rows[i].key});
            }
        }
    }
    if (!s.dragged.empty())
    {
        if (msg == WM_MOUSEMOVE || msg == WM_LBUTTONUP)
        {
            if (msg == WM_MOUSEMOVE && !(w & MK_LBUTTON)) { s.EndDrag(); return TRUE; }
            s.Hover({GET_X_LPARAM(l), GET_Y_LPARAM(l)});
            if (msg == WM_LBUTTONUP)
            {
                if (s.drop >= 0)
                {
                    s.Queue({OutlineOp::Move, s.rows[s.drop].key, s.dragged});
                }
                s.EndDrag();
            }
            return TRUE;
        }
        if (msg == WM_CAPTURECHANGED || msg == WM_CANCELMODE || (msg == WM_COMMAND && LOWORD(w) == IDCANCEL))
        {
            s.EndDrag();
            return TRUE;
        }
    }
    return FALSE;
}
