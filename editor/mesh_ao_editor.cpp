#include "mesh_ao_editor.hpp"
#include "mesh_ao_baker.hpp"
#include "material.hpp"
#include "editor_theme.hpp"
#include <algorithm>
#include <array>
#include <charconv>
#include <chrono>
#include <future>
#ifdef _WIN32
#include "windows_name_dialog.hpp"
#include "windows_theme.hpp"
#include "panel_ids.h"
#include <commctrl.h>
#endif

struct MeshAOEditor::Impl
{
    std::shared_ptr<const Mesh> mesh;
    std::array<std::string, 4> fields{"64", "3", "1", "0.01"};
    bool twoSided = true;
    std::string status = "Bake all LODs into vertex alpha. Changes are saved automatically.";
    std::future<MeshAOBakeResult> job;
    std::atomic<bool> cancel{false};
    std::atomic<std::size_t> completed{0}, total{0};
    std::uint64_t assignments = 0;
    unsigned materials = 0, images = 0;
#ifdef _WIN32
    HWND window = nullptr;
    static constexpr std::array ids{IDC_AO_SAMPLES, IDC_AO_DISTANCE, IDC_AO_STRENGTH, IDC_AO_BIAS};
#else
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    int selected = -1;
    bool replaceField = false;
    static constexpr SDL_FRect Field(int i) { return {260, 54.f + i * 38, 160, 28}; }
    static constexpr SDL_FRect Button(int i) { return {16.f + i * 140, 265, 130, 30}; }
    static constexpr SDL_FRect twoSidedBox{16, 212, 440, 28};
#endif
    bool Busy() const { return job.valid(); }
    int Progress() const
    { const auto n = total.load(); return n ? int(100 * completed.load() / n) : 0; }
    void Start()
    {
        if (Busy() || !mesh) return;
        MeshAOBakeSettings settings;
        const auto parse = [](const std::string& text, auto& value) {
            const auto result = std::from_chars(text.data(), text.data() + text.size(), value);
            return result.ec == std::errc{} && result.ptr == text.data() + text.size();
        };
        if (!parse(fields[0], settings.samples) || !parse(fields[1], settings.distance) ||
            !parse(fields[2], settings.strength) || !parse(fields[3], settings.bias) || !settings.Valid())
        {
            status = "Use 16-1024 samples, radius 0.001-10000 m, strength 0-4, and bias >=0.000001 m below radius.";
            Refresh(); return;
        }
        settings.twoSidedFoliage = twoSided;
        cancel = false; completed = 0; total = 0;
        status = "Preparing geometry and cutout textures...";
        try
        {
            const auto source = mesh;
            const auto slots = mesh->materials ? mesh->materials->slots : std::vector<MeshMaterialSlot>{};
            assignments = mesh->materials ? mesh->materials->revision : 0;
            materials = Material::Revision(); images = Material::ImageRevision();
            job = std::async(std::launch::async, [this, source, slots, settings] {
                return BakeMeshAO(*source, slots, settings, cancel, completed, total);
            });
        }
        catch (const std::exception& e) { status = e.what(); }
        Refresh();
    }
    void Clear()
    {
        if (Busy() || !mesh) return;
        status = MeshAO::Clear(*mesh) ? "AO cleared and saved. Vertex alpha is white on every LOD." : SDL_GetError();
        Refresh();
    }
    void Cancel()
    { if (Busy()) { cancel = true; status = "Cancelling..."; Refresh(); } }
    void Poll()
    {
        if (!Busy()) return;
        if (job.wait_for(std::chrono::seconds(0)) == std::future_status::ready)
        {
            try
            {
                auto result = job.get();
                if (result.cancelled || cancel) status = "Bake cancelled. Previous AO is unchanged.";
                else if (!result.data) status = result.error.empty() ? "AO bake failed." : result.error;
                else if ((mesh->materials && mesh->materials->revision != assignments) ||
                    Material::Revision() != materials || Material::ImageRevision() != images)
                    status = "Materials changed during the bake. Bake again to use the updated cutouts.";
                else if (!MeshAO::Save(*mesh, std::move(result.data))) status = SDL_GetError();
                else status = "AO baked and saved for all LODs. Inspect it with View > Vertex Alpha.";
            }
            catch (const std::exception& e) { status = e.what(); }
        }
        else if (!cancel && completed.load()) status = "Baking all LODs: " + std::to_string(Progress()) + "%";
        Refresh();
    }
    void Stop()
    {
        cancel = true;
        if (job.valid()) { job.wait(); job = {}; }
    }
    void Close()
    {
        Stop();
#ifdef _WIN32
        if (window) DestroyWindow(window);
#else
        if (window) SDL_StopTextInput(window);
        SDL_DestroyRenderer(renderer); renderer = nullptr;
        SDL_DestroyWindow(window); window = nullptr;
#endif
        mesh.reset();
    }
    void Refresh()
    {
#ifdef _WIN32
        if (!window) return;
        const auto wide = WindowsNameDialog::Wide(status);
        // Timer and render loop can both poll; avoid invalidating unchanged text.
        std::wstring previous(GetWindowTextLengthW(GetDlgItem(window, IDC_AO_STATUS)) + 1, L'\0');
        previous.resize(GetDlgItemTextW(window, IDC_AO_STATUS, previous.data(), int(previous.size())));
        if (previous != wide) SetDlgItemTextW(window, IDC_AO_STATUS, wide.c_str());
        for (int id : ids) EnableWindow(GetDlgItem(window, id), !Busy());
        for (int id : {IDC_AO_TWOSIDED, IDC_AO_BAKE, IDC_AO_CLEAR}) EnableWindow(GetDlgItem(window, id), !Busy());
        EnableWindow(GetDlgItem(window, IDC_AO_CANCEL), Busy() && !cancel);
        SendDlgItemMessageW(window, IDC_AO_PROGRESS, PBM_SETPOS, Progress(), 0);
#endif
    }
#ifdef _WIN32
    static INT_PTR CALLBACK Dialog(HWND window, UINT message, WPARAM w, LPARAM l)
    {
        if (message == WM_INITDIALOG)
        {
            auto* self = reinterpret_cast<Impl*>(l); self->window = window;
            SetWindowLongPtrW(window, DWLP_USER, l); WindowsTheme::ApplyDialog(window);
            SetWindowTextW(window, WindowsNameDialog::Wide("Bake AO - " + self->mesh->assetPath).c_str());
            for (std::size_t i = 0; i < ids.size(); ++i)
                SetDlgItemTextW(window, ids[i], WindowsNameDialog::Wide(self->fields[i]).c_str());
            CheckDlgButton(window, IDC_AO_TWOSIDED, self->twoSided ? BST_CHECKED : BST_UNCHECKED);
            SendDlgItemMessageW(window, IDC_AO_PROGRESS, PBM_SETRANGE32, 0, 100);
            SetTimer(window, 1, 100, nullptr); self->Refresh(); return TRUE;
        }
        auto* self = reinterpret_cast<Impl*>(GetWindowLongPtrW(window, DWLP_USER));
        if (!self) return FALSE;
        if (message == WM_TIMER) { self->Poll(); return TRUE; }
        if (message == WM_CLOSE || (message == WM_COMMAND && LOWORD(w) == IDCANCEL))
        { self->Close(); return TRUE; }
        if (message == WM_NCDESTROY) { KillTimer(window, 1); self->window = nullptr; return FALSE; }
        if (message == WM_COMMAND)
        {
            if (LOWORD(w) == IDC_AO_BAKE && !self->Busy())
            {
                for (std::size_t i = 0; i < ids.size(); ++i)
                {
                    wchar_t value[128]{}; GetDlgItemTextW(window, ids[i], value, 128);
                    self->fields[i] = WindowsNameDialog::Utf8(value);
                }
                self->twoSided = IsDlgButtonChecked(window, IDC_AO_TWOSIDED) == BST_CHECKED;
                self->Start(); return TRUE;
            }
            if (LOWORD(w) == IDC_AO_CLEAR) { self->Clear(); return TRUE; }
            if (LOWORD(w) == IDC_AO_CANCEL) { self->Cancel(); return TRUE; }
        }
        return FALSE;
    }
#endif
};

MeshAOEditor::MeshAOEditor() : impl(std::make_unique<Impl>()) {}
MeshAOEditor::~MeshAOEditor() { Close(); }
void MeshAOEditor::Close() { impl->Close(); }

bool MeshAOEditor::Open(SDL_Window* owner, std::shared_ptr<const Mesh> mesh)
{
    if (impl->mesh != mesh)
    {
        Close(); impl->completed = 0; impl->total = 0;
        impl->status = "Bake all LODs into vertex alpha. Changes are saved automatically.";
    }
    impl->mesh = std::move(mesh);
#ifdef _WIN32
    if (!impl->window)
    {
        const auto parent = static_cast<HWND>(SDL_GetPointerProperty(SDL_GetWindowProperties(owner), SDL_PROP_WINDOW_WIN32_HWND_POINTER, nullptr));
        if (!CreateDialogParamW(GetModuleHandleW(nullptr), MAKEINTRESOURCEW(IDD_MESH_AO), parent, Impl::Dialog, reinterpret_cast<LPARAM>(impl.get())))
            return SDL_SetError("Cannot create Bake AO window");
    }
    ShowWindow(impl->window, SW_SHOWNORMAL); SetForegroundWindow(impl->window);
#else
    if (!impl->window)
    {
        const auto properties = SDL_CreateProperties();
        SDL_SetPointerProperty(properties, SDL_PROP_WINDOW_CREATE_PARENT_POINTER, owner);
        SDL_SetStringProperty(properties, SDL_PROP_WINDOW_CREATE_TITLE_STRING, ("Bake AO - " + impl->mesh->assetPath).c_str());
        SDL_SetNumberProperty(properties, SDL_PROP_WINDOW_CREATE_WIDTH_NUMBER, 580);
        SDL_SetNumberProperty(properties, SDL_PROP_WINDOW_CREATE_HEIGHT_NUMBER, 400);
        impl->window = SDL_CreateWindowWithProperties(properties); SDL_DestroyProperties(properties);
        if (!impl->window) return false;
        impl->renderer = SDL_CreateRenderer(impl->window, nullptr);
        if (!impl->renderer) { Close(); return false; }
    }
    SDL_ShowWindow(impl->window); SDL_RaiseWindow(impl->window);
#endif
    return true;
}

bool MeshAOEditor::Focused() const
{
#ifdef _WIN32
    const auto focus = GetFocus(); return impl->window && (focus == impl->window || IsChild(impl->window, focus));
#else
    return impl->window && SDL_GetKeyboardFocus() == impl->window;
#endif
}
bool MeshAOEditor::ContainsScreen(float x, float y) const
{
    if (!impl->window) return false;
#ifdef _WIN32
    if (!IsWindowVisible(impl->window)) return false;
    const HWND hit = WindowFromPoint({LONG(x), LONG(y)});
    return hit == impl->window || IsChild(impl->window, hit);
#else
    if (SDL_GetWindowFlags(impl->window) & SDL_WINDOW_HIDDEN) return false;
    int wx, wy, w, h; SDL_GetWindowPosition(impl->window, &wx, &wy); SDL_GetWindowSize(impl->window, &w, &h);
    return x >= wx && y >= wy && x < wx + w && y < wy + h;
#endif
}
#ifdef _WIN32
bool MeshAOEditor::HandleNativeMessage(void* value)
{
    auto& msg = *static_cast<MSG*>(value);
    return impl->window && (msg.hwnd == impl->window || IsChild(impl->window, msg.hwnd)) &&
        msg.message >= WM_KEYFIRST && msg.message <= WM_KEYLAST && IsDialogMessageW(impl->window, &msg);
}
#endif

bool MeshAOEditor::Handle(const SDL_Event& event)
{
#ifdef _WIN32
    (void)event; return false;
#else
    auto& self = *impl;
    if (!self.window || SDL_GetWindowFromEvent(&event) != self.window) return false;
    if (event.type == SDL_EVENT_WINDOW_CLOSE_REQUESTED) { Close(); return true; }
    if (event.type == SDL_EVENT_KEY_DOWN)
    {
        if (event.key.key == SDLK_ESCAPE) { if (self.Busy()) self.Cancel(); else Close(); return true; }
        if (!self.Busy() && event.key.key == SDLK_RETURN) self.Start();
        if (!self.Busy() && event.key.key == SDLK_TAB)
        { self.selected = (self.selected + 1) % 4; self.replaceField = true; SDL_StartTextInput(self.window); }
        if (!self.Busy() && self.selected >= 0)
        {
            auto& value = self.fields[self.selected];
            if (event.key.key == SDLK_A && (event.key.mod & SDL_KMOD_CTRL)) self.replaceField = true;
            if (event.key.key == SDLK_BACKSPACE)
            { if (self.replaceField) value.clear(); else if (!value.empty()) value.pop_back(); self.replaceField = false; }
        }
    }
    if (event.type == SDL_EVENT_TEXT_INPUT && !self.Busy() && self.selected >= 0)
    {
        auto& value = self.fields[self.selected];
        if (self.replaceField) value.clear();
        self.replaceField = false;
        if (value.size() < 32) value += event.text.text;
    }
    if (event.type == SDL_EVENT_MOUSE_BUTTON_DOWN && event.button.button == SDL_BUTTON_LEFT)
    {
        const SDL_FPoint point{event.button.x, event.button.y};
        self.selected = -1; SDL_StopTextInput(self.window);
        for (int i = 0; i < 4; ++i)
        {
            const auto box = Impl::Field(i);
            if (!self.Busy() && SDL_PointInRectFloat(&point, &box))
            { self.selected = i; self.replaceField = true; SDL_StartTextInput(self.window); }
        }
        if (!self.Busy() && SDL_PointInRectFloat(&point, &Impl::twoSidedBox)) self.twoSided = !self.twoSided;
        for (int i = 0; i < 4; ++i)
        {
            const auto box = Impl::Button(i);
            if (!SDL_PointInRectFloat(&point, &box)) continue;
            if (i == 0) self.Start(); else if (i == 1) self.Clear(); else if (i == 2) self.Cancel(); else Close();
        }
    }
    return true;
#endif
}

bool MeshAOEditor::Draw()
{
    impl->Poll();
#ifndef _WIN32
    auto& self = *impl;
    if (!self.window) return true;
    auto* r = self.renderer;
    EditorTheme::SetColor(r, EditorTheme::panel); SDL_RenderClear(r);
    const auto text = [&](float x, float y, const std::string& value) {
        EditorTheme::SetColor(r, EditorTheme::text); SDL_RenderDebugText(r, x, y, value.c_str());
    };
    text(16, 18, "Bake ambient occlusion into vertex alpha (all LODs)");
    constexpr std::array labels{"Samples per vertex (16-1024)", "Occlusion radius (m)", "Strength (0-4)", "Surface bias (m)"};
    for (int i = 0; i < 4; ++i)
    {
        const auto box = Impl::Field(i);
        text(16, box.y + 10, labels[i]);
        EditorTheme::SetColor(r, i == self.selected ? EditorTheme::selected : EditorTheme::field); SDL_RenderFillRect(r, &box);
        text(box.x + 6, box.y + 10, self.fields[i]);
    }
    text(16, 221, std::string(self.twoSided ? "[x] " : "[ ] ") + "Sample both sides of foliage");
    text(16, 245, "Radius and bias use model meters, before instance scaling.");
    constexpr std::array labels2{"Bake AO", "Clear AO", "Cancel Bake", "Close"};
    for (int i = 0; i < 4; ++i)
    {
        const auto box = Impl::Button(i);
        EditorTheme::SetColor(r, EditorTheme::button); SDL_RenderFillRect(r, &box); text(box.x + 8, box.y + 10, labels2[i]);
    }
    if (self.Busy()) text(16, 310, std::to_string(self.Progress()) + "%");
    // Wrap status/error messages rather than hiding the reason a bake failed.
    for (std::size_t i = 0; i < self.status.size(); i += 68) text(16, 332 + float(i / 68) * 14, self.status.substr(i, 68));
    return SDL_RenderPresent(r);
#else
    return true;
#endif
}
