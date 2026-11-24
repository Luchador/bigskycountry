#include "windows_mesh_controls.hpp"
#include "mesh_editor.hpp"
#include "mesh_materials.hpp"
#include "windows_menu.hpp"
#include "windows_name_dialog.hpp"
#include "windows_theme.hpp"
#include "panel_ids.h"
#include <commctrl.h>
#include <windowsx.h>
#include <algorithm>
#include <cmath>

namespace
{
    constexpr UINT lit = 47001, colors = 47002, alpha = 47003, frameMesh = 47004, bakeAO = 47005;
    constexpr WindowsMenu::MenuItem viewTitle{0, L"&View", 0, true};
    constexpr WindowsMenu::MenuItem viewItems[] = {
        {lit, L"&Lit", MF_CHECKED, false, true},
        {colors, L"&Vertex Colors", MF_ENABLED, false, true},
        {alpha, L"Vertex &Alpha", MF_ENABLED, false, true},
        {frameMesh, L"&Frame Mesh\tF"}
    };
    constexpr WindowsMenu::MenuItem toolsTitle{0, L"&Tools", 0, true};
    constexpr WindowsMenu::MenuItem toolsItems[] = {{bakeAO, L"&Bake AO..."}};
    void Caption(HWND window, const std::string& text)
    {
        const auto wide = WindowsNameDialog::Wide(text);
        std::wstring old(GetWindowTextLengthW(window) + 1, L'\0');
        old.resize(GetWindowTextW(window, old.data(), static_cast<int>(old.size())));
        if (wide != old) SetWindowTextW(window, wide.c_str());
    }
}

WindowsMeshControls::~WindowsMeshControls()
{
    CancelResize(false);
    if (frame) RemoveWindowSubclass(frame, FrameProc, 31);
    // SDL only wraps view; release its input/swapchain state before DestroyWindow.
    SDL_DestroyWindow(viewport);
    if (view) DestroyWindow(view);
    if (panel) DestroyWindow(panel);
    if (splitter) DestroyWindow(splitter);
    if (hint) DestroyWindow(hint);
    if (font) DeleteObject(font);
}

int WindowsMeshControls::Scale(int value) const { return MulDiv(value, GetDpiForWindow(frame), 96); }

SDL_Window* WindowsMeshControls::Create(SDL_Window* parent)
{
    frame = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(parent), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
    if (!frame) { SDL_SetError("Cannot find the native mesh editor window"); return nullptr; }
    SetWindowLongPtrW(frame, GWL_STYLE, GetWindowLongPtrW(frame, GWL_STYLE) | WS_CLIPCHILDREN);
    SetWindowLongPtrW(frame, GWL_EXSTYLE, GetWindowLongPtrW(frame, GWL_EXSTYLE) | WS_EX_CONTROLPARENT);
    const auto module = GetModuleHandleW(nullptr);
    WNDCLASSW wc{};
    wc.hInstance = module; wc.lpfnWndProc = ViewProc;
    wc.lpszClassName = L"BigSkyMeshViewport";
    wc.hCursor = LoadCursorW(nullptr, IDC_ARROW);
    if (!RegisterClassW(&wc) && GetLastError() != ERROR_CLASS_ALREADY_EXISTS)
    { SDL_SetError("Cannot register mesh viewport"); return nullptr; }
    view = CreateWindowExW(WS_EX_NOPARENTNOTIFY, wc.lpszClassName, L"Mesh viewport",
        WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | WS_TABSTOP, 0, 0, 1, 1, frame, nullptr, module, nullptr);
    panel = CreateDialogParamW(module, MAKEINTRESOURCEW(IDD_MESH), frame, PanelProc, reinterpret_cast<LPARAM>(this));
    splitter = CreateWindowExW(WS_EX_NOPARENTNOTIFY, L"STATIC", L"Resize material panel",
        WS_CHILD | WS_VISIBLE | WS_CLIPSIBLINGS | SS_NOTIFY | WS_TABSTOP, 0, 0, 1, 1, frame, nullptr, module, nullptr);
    hint = CreateWindowExW(0, L"STATIC", L"LMB: orbit   Wheel: zoom   F: frame",
        WS_CHILD | WS_VISIBLE | SS_CENTERIMAGE | SS_ENDELLIPSIS, 0, 0, 1, 1, frame, nullptr, module, nullptr);
    if (!view || !panel || !splitter || !hint ||
        !SetWindowSubclass(frame, FrameProc, 31, reinterpret_cast<DWORD_PTR>(this)) ||
        !SetWindowSubclass(splitter, SplitterProc, 1, reinterpret_cast<DWORD_PTR>(this)))
    { SDL_SetError("Cannot create mesh editor controls"); return nullptr; }
    HMENU menu = CreateMenu();
    if (!menu) { SDL_SetError("Cannot create mesh editor menu"); return nullptr; }
    if (!WindowsMenu::AddSubmenu(menu, viewTitle, viewItems) ||
        !WindowsMenu::AddSubmenu(menu, toolsTitle, toolsItems) || !SetMenu(frame, menu))
    {
        DestroyMenu(menu);
        SDL_SetError("Cannot attach mesh editor menu");
        return nullptr;
    }
    // The frame owns the menu, just as in the main editor.
    SDL_SetHint(SDL_HINT_WINDOWS_ENABLE_MENU_MNEMONICS, "1");
    WindowsTheme::ApplyDialog(frame);
    UpdateFont();
    DrawMenuBar(frame);
    Layout();
    const auto props = SDL_CreateProperties();
    SDL_SetPointerProperty(props, SDL_PROP_WINDOW_CREATE_WIN32_HWND_POINTER, view);
    viewport = SDL_CreateWindowWithProperties(props);
    SDL_DestroyProperties(props);
    return viewport;
}

void WindowsMeshControls::UpdateFont()
{
    const auto next = CreateFontW(-MulDiv(9, GetDpiForWindow(frame), 72), 0, 0, 0, FW_NORMAL, FALSE, FALSE, FALSE,
        DEFAULT_CHARSET, OUT_DEFAULT_PRECIS, CLIP_DEFAULT_PRECIS, CLEARTYPE_QUALITY, DEFAULT_PITCH, L"Segoe UI");
    if (!next) return;
    SDL_SetWindowMinimumSize(editor.window, Scale(560), Scale(320));
    SendMessageW(panel, WM_SETFONT, reinterpret_cast<WPARAM>(next), TRUE);
    EnumChildWindows(panel, [](HWND child, LPARAM data) -> BOOL {
        SendMessageW(child, WM_SETFONT, data, TRUE); return TRUE;
    }, reinterpret_cast<LPARAM>(next));
    SendMessageW(hint, WM_SETFONT, reinterpret_cast<WPARAM>(next), TRUE);
    if (font) DeleteObject(font);
    font = next;
    SendDlgItemMessageW(panel, IDC_MESH_SLOTS, LB_SETITEMHEIGHT, 0, Scale(76));
}

void WindowsMeshControls::Layout()
{
    if (!panel || !view || !splitter || !hint || IsIconic(frame)) return;
    RECT area{}; GetClientRect(frame, &area);
    const int divider = Scale(6), pad = Scale(10), footer = Scale(24);
    const int available = std::max(1, int(area.right) - divider);
    const int width = std::clamp(Scale(int(std::lround(editor.panelWidth))), std::min(Scale(200), available / 2),
                                std::max(available - Scale(160), available / 2));
    const int split = available - width;
    MoveWindow(view, 0, 0, std::max(1, split), std::max(1, int(area.bottom) - footer), TRUE);
    MoveWindow(hint, pad, std::max(0, int(area.bottom) - footer), std::max(1, split - 2 * pad), footer, TRUE);
    MoveWindow(splitter, split, 0, divider, area.bottom, TRUE);
    MoveWindow(panel, split + divider, 0, width, area.bottom, TRUE);
    const auto move = [&](int id, int y, int h) {
        MoveWindow(GetDlgItem(panel, id), pad, Scale(y), std::max(1, width - 2 * pad), Scale(h), TRUE);
    };
    move(IDC_MESH_TITLE, 10, 20); move(IDC_MESH_HELP, 32, 20); move(IDC_MESH_STATS, 54, 20);
    move(IDC_MESH_LOD_LABEL, 79, 18); move(IDC_MESH_LOD, 99, 240);
    const int top = Scale(132), bottom = std::max(top + Scale(24), int(area.bottom) - Scale(68));
    MoveWindow(GetDlgItem(panel, IDC_MESH_SLOTS), pad, top, std::max(1, width - 2 * pad), bottom - top, TRUE);
    MoveWindow(GetDlgItem(panel, IDC_MESH_REMOVE), pad, bottom + Scale(8), std::max(1, width - 2 * pad), Scale(26), TRUE);
    MoveWindow(GetDlgItem(panel, IDC_MESH_STATUS), pad, bottom + Scale(39), std::max(1, width - 2 * pad), Scale(22), TRUE);
    editor.dirty = true;
    RedrawWindow(frame, nullptr, nullptr, RDW_INVALIDATE | RDW_ERASE | RDW_ALLCHILDREN);
}

void WindowsMeshControls::Reload()
{
    SendDlgItemMessageW(panel, IDC_MESH_LOD, CB_RESETCONTENT, 0, 0);
    SendDlgItemMessageW(panel, IDC_MESH_LOD, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(L"Automatic"));
    for (std::size_t i = 0; i <= editor.instance.mesh->lods.size(); ++i)
    {
        const auto label = L"LOD " + std::to_wstring(i);
        SendDlgItemMessageW(panel, IDC_MESH_LOD, CB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
    }
    SendDlgItemMessageW(panel, IDC_MESH_LOD, CB_SETCURSEL, editor.previewLOD + 1, 0);
    SendDlgItemMessageW(panel, IDC_MESH_SLOTS, LB_RESETCONTENT, 0, 0);
    RefreshSlots();
    Sync(0);
}

void WindowsMeshControls::RefreshSlots()
{
    if (!panel || !editor.instance.mesh || !editor.instance.mesh->materials) return;
    const HWND list = GetDlgItem(panel, IDC_MESH_SLOTS);
    const auto selected = SendMessageW(list, LB_GETCURSEL, 0, 0), top = SendMessageW(list, LB_GETTOPINDEX, 0, 0);
    SendMessageW(list, WM_SETREDRAW, FALSE, 0);
    SendMessageW(list, LB_RESETCONTENT, 0, 0);
    for (const auto& slot : editor.instance.mesh->materials->slots)
    {
        // Native list text also provides the accessible name and keyboard search.
        const auto label = WindowsNameDialog::Wide(slot.name + " - " + (slot.material.empty() ? "Unassigned" : slot.material));
        SendMessageW(list, LB_ADDSTRING, 0, reinterpret_cast<LPARAM>(label.c_str()));
    }
    SendMessageW(list, LB_SETCURSEL, selected, 0);
    SendMessageW(list, LB_SETTOPINDEX, std::max<LRESULT>(0, top), 0);
    SendMessageW(list, WM_SETREDRAW, TRUE, 0);
    InvalidateSlots();
    UpdateRemove();
}

void WindowsMeshControls::InvalidateSlots()
{ if (panel) InvalidateRect(GetDlgItem(panel, IDC_MESH_SLOTS), nullptr, FALSE); }

void WindowsMeshControls::Sync(std::size_t level)
{
    const auto& mesh = *editor.instance.mesh;
    Caption(GetDlgItem(panel, IDC_MESH_STATS), std::to_string(mesh.materials->slots.size()) + " slots | " +
        std::to_string(mesh.Level(level).indices.size() / 3) + " tris | LOD " + std::to_string(level));
    Caption(GetDlgItem(panel, IDC_MESH_STATUS), editor.status);
    CheckMenuRadioItem(GetSubMenu(GetMenu(frame), 0), lit, alpha,
        editor.viewMode == MeshEditorWindow::ViewMode::Lit ? lit :
        editor.viewMode == MeshEditorWindow::ViewMode::VertexColors ? colors : alpha, MF_BYCOMMAND);
    UpdateRemove();
}

void WindowsMeshControls::UpdateRemove()
{
    const auto selected = SendDlgItemMessageW(panel, IDC_MESH_SLOTS, LB_GETCURSEL, 0, 0);
    const bool assigned = editor.instance.mesh && selected >= 0 &&
        static_cast<std::size_t>(selected) < editor.instance.mesh->materials->slots.size() &&
        !editor.instance.mesh->materials->slots[selected].material.empty();
    EnableWindow(GetDlgItem(panel, IDC_MESH_REMOVE), assigned);
}

void WindowsMeshControls::RemoveMaterial()
{
    const auto selected = SendDlgItemMessageW(panel, IDC_MESH_SLOTS, LB_GETCURSEL, 0, 0);
    if (selected >= 0 && editor.instance.mesh && static_cast<std::size_t>(selected) < editor.instance.mesh->materials->slots.size())
        editor.Drop(static_cast<std::size_t>(selected), {});
    UpdateRemove();
    if (GetFocus() == GetDlgItem(panel, IDC_MESH_REMOVE)) SetFocus(GetDlgItem(panel, IDC_MESH_SLOTS));
}

bool WindowsMeshControls::Focused() const
{ const HWND focus = GetFocus(); return frame && (focus == frame || IsChild(frame, focus)); }

bool WindowsMeshControls::ContainsScreen(float x, float y) const
{
    if (!frame || !IsWindowVisible(frame) || IsIconic(frame)) return false;
    const HWND hit = WindowFromPoint({LONG(std::lround(x)), LONG(std::lround(y))});
    return hit == frame || IsChild(frame, hit);
}

std::optional<std::size_t> WindowsMeshControls::SlotAtScreen(float x, float y) const
{
    if (!panel || resizing || menuActive || !IsWindowVisible(frame) || IsIconic(frame)) return {};
    POINT point{LONG(std::lround(x)), LONG(std::lround(y))};
    const HWND list = GetDlgItem(panel, IDC_MESH_SLOTS);
    if (WindowFromPoint(point) != list) return {};
    ScreenToClient(list, &point);
    RECT client{}; GetClientRect(list, &client);
    if (!PtInRect(&client, point)) return {}; // Exclude the scrollbar and borders.
    const auto hit = SendMessageW(list, LB_ITEMFROMPOINT, 0, MAKELPARAM(point.x, point.y));
    const auto index = LOWORD(hit);
    RECT row{};
    if (HIWORD(hit) || !editor.instance.mesh || index >= editor.instance.mesh->materials->slots.size() ||
        SendMessageW(list, LB_GETITEMRECT, index, reinterpret_cast<LPARAM>(&row)) == LB_ERR || !PtInRect(&row, point)) return {};
    return index;
}

bool WindowsMeshControls::HandleMessage(MSG& message)
{
    if (!frame || (message.hwnd != frame && !IsChild(frame, message.hwnd))) return false;
    if (message.message < WM_KEYFIRST || message.message > WM_KEYLAST) return false;
    if (message.message == WM_KEYDOWN && !menuActive)
    {
        const bool dropdown = SendDlgItemMessageW(panel, IDC_MESH_LOD, CB_GETDROPPEDSTATE, 0, 0) != 0;
        if (!dropdown && message.wParam == VK_ESCAPE)
        {
            CancelResize(true); editor.CancelPointer();
            SendDlgItemMessageW(panel, IDC_MESH_SLOTS, LB_SETCURSEL, -1, 0);
            UpdateRemove();
            return true;
        }
        if (!dropdown && message.wParam == 'F' && !(GetKeyState(VK_CONTROL) & 0x8000) && !(GetKeyState(VK_MENU) & 0x8000))
        { editor.Frame(); return true; }
        if (message.wParam == VK_DELETE && GetFocus() == GetDlgItem(panel, IDC_MESH_SLOTS))
        { RemoveMaterial(); return true; }
    }
    return IsDialogMessageW(frame, &message) != FALSE;
}

void WindowsMeshControls::PaintSlot(const DRAWITEMSTRUCT& draw)
{
    if (!editor.instance.mesh || draw.itemID >= editor.instance.mesh->materials->slots.size()) return;
    const bool selected = (draw.itemState & ODS_SELECTED) || editor.hover == draw.itemID;
    const auto& slot = editor.instance.mesh->materials->slots[draw.itemID];
    FillRect(draw.hDC, &draw.rcItem, WindowsTheme::Brush(selected ? EditorTheme::selected : EditorTheme::list));
    const int pad = Scale(6);
    RECT image{draw.rcItem.left + pad, draw.rcItem.top + pad, draw.rcItem.left + Scale(70), draw.rcItem.top + Scale(70)};
    SDL_Surface* thumbnail = nullptr;
    if (!editor.imagesDirty && draw.itemID < editor.materialPreviews.size() && !editor.materialPreviews[draw.itemID].Path().empty())
        thumbnail = editor.materialPreviews[draw.itemID].Preview(64, 64);
    WindowsTheme::DrawPreview(draw.hDC, image, thumbnail, EditorTheme::field);
    SetBkMode(draw.hDC, TRANSPARENT);
    SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::text));
    RECT line{image.right + pad, image.top, draw.rcItem.right - pad, image.top + Scale(25)};
    constexpr UINT flags = DT_SINGLELINE | DT_VCENTER | DT_END_ELLIPSIS | DT_NOPREFIX;
    DrawTextW(draw.hDC, WindowsNameDialog::Wide(slot.name).c_str(), -1, &line, flags);
    line.top = line.bottom; line.bottom = image.bottom;
    SetTextColor(draw.hDC, WindowsTheme::Color(EditorTheme::secondary));
    const auto name = slot.material.empty() ? "Unassigned" : slot.material.substr(slot.material.find_last_of('/') + 1);
    DrawTextW(draw.hDC, WindowsNameDialog::Wide(name).c_str(), -1, &line, flags);
    if ((draw.itemState & ODS_FOCUS) && !(draw.itemState & ODS_NOFOCUSRECT)) DrawFocusRect(draw.hDC, &draw.rcItem);
}

INT_PTR CALLBACK WindowsMeshControls::PanelProc(HWND window, UINT message, WPARAM w, LPARAM l)
{
    if (message == WM_INITDIALOG)
    {
        auto* self = reinterpret_cast<WindowsMeshControls*>(l);
        self->panel = window;
        SetWindowLongPtrW(window, DWLP_USER, l);
        WindowsTheme::ApplyDialog(window);
        return TRUE;
    }
    auto* self = reinterpret_cast<WindowsMeshControls*>(GetWindowLongPtrW(window, DWLP_USER));
    if (!self) return FALSE;
    if (message == WM_MEASUREITEM)
    {
        auto* measure = reinterpret_cast<MEASUREITEMSTRUCT*>(l);
        if (measure->CtlID == IDC_MESH_SLOTS) { measure->itemHeight = self->Scale(76); return TRUE; }
    }
    if (message == WM_DRAWITEM && reinterpret_cast<DRAWITEMSTRUCT*>(l)->CtlID == IDC_MESH_SLOTS)
    { self->PaintSlot(*reinterpret_cast<DRAWITEMSTRUCT*>(l)); return TRUE; }
    if (message == WM_COMMAND)
    {
        if (LOWORD(w) == IDC_MESH_REMOVE && HIWORD(w) == BN_CLICKED) self->RemoveMaterial();
        else if (LOWORD(w) == IDC_MESH_SLOTS && HIWORD(w) == LBN_SELCHANGE) self->UpdateRemove();
        else if (LOWORD(w) == IDC_MESH_LOD && HIWORD(w) == CBN_SELCHANGE)
        {
            self->editor.previewLOD = int(SendDlgItemMessageW(window, IDC_MESH_LOD, CB_GETCURSEL, 0, 0)) - 1;
            self->editor.dirty = true;
        }
        return TRUE;
    }
    return FALSE;
}

LRESULT CALLBACK WindowsMeshControls::ViewProc(HWND window, UINT message, WPARAM w, LPARAM l)
{
    if (message == WM_ERASEBKGND) return 1;
    if (message == WM_LBUTTONDOWN) SetFocus(window);
    if (message == WM_GETDLGCODE) return DLGC_WANTARROWS | DLGC_WANTCHARS;
    return DefWindowProcW(window, message, w, l);
}

LRESULT CALLBACK WindowsMeshControls::FrameProc(HWND window, UINT message, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR data)
{
    auto& self = *reinterpret_cast<WindowsMeshControls*>(data);
    LRESULT result = 0;
    if (WindowsMenu::HandleMessage(window, message, w, l, result)) return result;
    if (message == WM_NCPAINT || message == WM_NCACTIVATE || message == WM_EXITMENULOOP)
    {
        if (message == WM_EXITMENULOOP) self.menuActive = false;
        result = DefSubclassProc(window, message, w, l);
        WindowsMenu::PaintMenuBarFrame(window);
        return result;
    }
    if (message == WM_ENTERMENULOOP)
    { self.menuActive = true; self.CancelResize(false); self.editor.CancelPointer(); }
    if (message == WM_SIZE && w != SIZE_MINIMIZED) self.Layout();
    if (message == WM_DPICHANGED)
    {
        result = DefSubclassProc(window, message, w, l);
        self.UpdateFont(); self.Layout(); return result;
    }
    if (message == WM_COMMAND && l == 0)
    {
        if (LOWORD(w) == lit || LOWORD(w) == colors || LOWORD(w) == alpha)
        {
            self.editor.SetViewMode(LOWORD(w) == lit ? MeshEditorWindow::ViewMode::Lit :
                LOWORD(w) == colors ? MeshEditorWindow::ViewMode::VertexColors : MeshEditorWindow::ViewMode::VertexAlpha);
            CheckMenuRadioItem(GetSubMenu(GetMenu(window), 0), lit, alpha, LOWORD(w), MF_BYCOMMAND);
            return 0;
        }
        if (LOWORD(w) == frameMesh) { self.editor.Frame(); return 0; }
        if (LOWORD(w) == bakeAO) { self.editor.OpenAO(); return 0; }
    }
    if (message == WM_SETFOCUS) SetFocus(self.view);
    if (message == WM_NCDESTROY) { RemoveWindowSubclass(window, FrameProc, id); self.frame = nullptr; }
    return DefSubclassProc(window, message, w, l);
}

void WindowsMeshControls::CancelResize(bool restore)
{
    if (!resizing) return;
    resizing = false;
    if (restore) { editor.panelWidth = oldWidth; Layout(); }
    if (GetCapture() == splitter) ReleaseCapture();
}

LRESULT CALLBACK WindowsMeshControls::SplitterProc(HWND window, UINT message, WPARAM w, LPARAM l, UINT_PTR id, DWORD_PTR data)
{
    auto& self = *reinterpret_cast<WindowsMeshControls*>(data);
    if (message == WM_SETCURSOR) { SetCursor(LoadCursorW(nullptr, IDC_SIZEWE)); return TRUE; }
    if (message == WM_GETDLGCODE) return DLGC_WANTARROWS;
    if (message == WM_LBUTTONDOWN)
    {
        self.editor.CancelPointer();
        SetFocus(window); SetCapture(window);
        self.resizing = true; self.oldWidth = self.editor.panelWidth;
        self.grabOffset = GET_X_LPARAM(l);
        return 0;
    }
    if (message == WM_MOUSEMOVE && self.resizing)
    {
        if (!(w & MK_LBUTTON)) { self.CancelResize(false); return 0; }
        POINT point{GET_X_LPARAM(l), GET_Y_LPARAM(l)};
        MapWindowPoints(window, self.frame, &point, 1);
        RECT area{}; GetClientRect(self.frame, &area);
        const int available = std::max(1, int(area.right) - self.Scale(6));
        const int width = std::clamp(available - int(point.x) + self.grabOffset, std::min(self.Scale(200), available / 2),
                                     std::max(available - self.Scale(160), available / 2));
        self.editor.panelWidth = width * 96.f / GetDpiForWindow(self.frame);
        self.Layout(); return 0;
    }
    if (message == WM_KEYDOWN && (w == VK_LEFT || w == VK_RIGHT))
    {
        RECT area{}; GetClientRect(self.frame, &area);
        const float available = std::max(1.f, area.right * 96.f / GetDpiForWindow(self.frame) - 6.f);
        self.editor.panelWidth = std::clamp(self.editor.panelWidth + (w == VK_LEFT ? 16.f : -16.f),
            std::min(200.f, available / 2), std::max(available - 160.f, available / 2));
        self.Layout(); return 0;
    }
    if (message == WM_LBUTTONUP || message == WM_CANCELMODE || message == WM_CAPTURECHANGED) self.CancelResize(false);
    if (message == WM_PAINT)
    {
        PAINTSTRUCT paint{}; HDC dc = BeginPaint(window, &paint);
        RECT r{}; GetClientRect(window, &r);
        FillRect(dc, &r, WindowsTheme::Brush(EditorTheme::chrome));
        RECT grip{(r.right - self.Scale(2)) / 2, (r.bottom - self.Scale(32)) / 2,
                  (r.right + self.Scale(2)) / 2, (r.bottom + self.Scale(32)) / 2};
        FillRect(dc, &grip, WindowsTheme::Brush(GetFocus() == window ? EditorTheme::accent : EditorTheme::disabled));
        EndPaint(window, &paint); return 0;
    }
    if (message == WM_NCDESTROY) RemoveWindowSubclass(window, SplitterProc, id);
    return DefSubclassProc(window, message, w, l);
}
