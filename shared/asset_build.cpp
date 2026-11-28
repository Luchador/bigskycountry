#include "asset_build.hpp"
#include "asset_build_settings.hpp"
#include "asset_paths.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <string>
#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

namespace
{
    std::string Utf8(const char8_t* text) { return reinterpret_cast<const char*>(text); }
    std::filesystem::path Path(const std::string& text)
    { return std::filesystem::path(std::u8string(text.begin(), text.end())); }

    bool Build(const std::string& cmake, const std::string& script, SDL_IOStream* log, int& exitCode)
    {
#ifdef _WIN32
        // SDL's background-process mode hides the console but discards the exit
        // code. Keep the real result and hide the console with CreateProcess.
        const auto executable = Path(cmake).wstring();
        // Both arguments are file paths (Windows filenames cannot contain quotes
        // or end in a slash); no shell interprets spaces or metacharacters.
        auto command = L"\"" + executable + L"\" -P \"" + Path(script).wstring() + L"\"";
        const auto file = SDL_GetPointerProperty(SDL_GetIOProperties(log), SDL_PROP_IOSTREAM_WINDOWS_HANDLE_POINTER, nullptr);
        HANDLE output = nullptr;
        if (!file || !DuplicateHandle(GetCurrentProcess(), file, GetCurrentProcess(), &output, 0, TRUE, DUPLICATE_SAME_ACCESS))
            return SDL_SetError("Cannot redirect asset build output (Windows error %lu)", GetLastError());
        SECURITY_ATTRIBUTES attributes{sizeof(SECURITY_ATTRIBUTES), nullptr, TRUE};
        const HANDLE input = CreateFileW(L"NUL", GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE,
                                         &attributes, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (input == INVALID_HANDLE_VALUE)
        {
            const auto error = GetLastError();
            CloseHandle(output);
            return SDL_SetError("Cannot initialize asset build input (Windows error %lu)", error);
        }
        STARTUPINFOW startup{};
        startup.cb = sizeof(startup);
        startup.dwFlags = STARTF_USESTDHANDLES;
        startup.hStdInput = input;
        startup.hStdOutput = startup.hStdError = output;
        PROCESS_INFORMATION process{};
        const bool started = CreateProcessW(executable.c_str(), command.data(), nullptr, nullptr, TRUE,
                                            CREATE_NO_WINDOW, nullptr, nullptr, &startup, &process);
        const auto startError = GetLastError();
        CloseHandle(input);
        CloseHandle(output);
        if (!started) return SDL_SetError("Cannot start asset build (Windows error %lu)", startError);
        CloseHandle(process.hThread);
        DWORD result = 0;
        const bool finished = WaitForSingleObject(process.hProcess, INFINITE) == WAIT_OBJECT_0 &&
                              GetExitCodeProcess(process.hProcess, &result);
        const auto waitError = GetLastError();
        CloseHandle(process.hProcess);
        if (!finished) return SDL_SetError("Cannot wait for asset build (Windows error %lu)", waitError);
        exitCode = static_cast<int>(result);
        return true;
#else
        const char* args[]{cmake.c_str(), "-P", script.c_str(), nullptr};
        const auto properties = SDL_CreateProperties();
        if (!properties) return false;
        const bool configured = SDL_SetPointerProperty(properties, SDL_PROP_PROCESS_CREATE_ARGS_POINTER, args) &&
            SDL_SetNumberProperty(properties, SDL_PROP_PROCESS_CREATE_STDOUT_NUMBER, SDL_PROCESS_STDIO_REDIRECT) &&
            SDL_SetPointerProperty(properties, SDL_PROP_PROCESS_CREATE_STDOUT_POINTER, log) &&
            SDL_SetBooleanProperty(properties, SDL_PROP_PROCESS_CREATE_STDERR_TO_STDOUT_BOOLEAN, true);
        std::unique_ptr<SDL_Process, decltype(&SDL_DestroyProcess)> process(
            configured ? SDL_CreateProcessWithProperties(properties) : nullptr, SDL_DestroyProcess);
        SDL_DestroyProperties(properties);
        return process && SDL_WaitProcess(process.get(), true, &exitCode);
#endif
    }

    std::string LogTail(const std::string& file)
    {
        std::ifstream input(Path(file), std::ios::binary | std::ios::ate);
        if (!input) return {};
        const auto size = input.tellg();
        if (size <= 0) return {};
        const auto count = std::min<std::streamoff>(size, 4000);
        input.seekg(size - count);
        std::string text(static_cast<std::size_t>(count), '\0');
        input.read(text.data(), count);
        text.resize(static_cast<std::size_t>(input.gcount()));
        return text;
    }
}

bool AssetBuild::Run()
{
    const auto cmake = Utf8(AssetBuildSettings::cmake);
    const auto script = Utf8(AssetBuildSettings::script);
    const auto logPath = Utf8(AssetBuildSettings::log);
    if (std::filesystem::weakly_canonical(Path(AssetPaths::Root(AssetRoot::Cooked))) !=
        std::filesystem::weakly_canonical(Path(Utf8(AssetBuildSettings::cooked))))
        return SDL_SetError("Cook World requires the editor/cooker in its configured build directory. Reconfigure and rebuild after moving the project.");
    SDL_PathInfo info{};
    if (!SDL_GetPathInfo(script.c_str(), &info) || info.type != SDL_PATHTYPE_FILE)
        return SDL_SetError("Asset build configuration is missing. Reconfigure and rebuild the editor/cooker. Expected: %s", script.c_str());
    std::unique_ptr<SDL_IOStream, decltype(&SDL_CloseIO)> log(SDL_IOFromFile(logPath.c_str(), "wb"), SDL_CloseIO);
    if (!log) return false;
    SDL_Log("Rebuilding runtime assets. Log: %s", logPath.c_str());
    int exitCode = -1;
    const bool completed = Build(cmake, script, log.get(), exitCode);
    const std::string error = completed ? std::string{} : SDL_GetError();
    log.reset();
    if (!completed || exitCode != 0)
        return SDL_SetError("Asset build failed%s%s. The world package was not replaced.\nLog: %s\n\n%s",
                            error.empty() ? "" : ": ", error.c_str(), logPath.c_str(), LogTail(logPath).c_str());
    return true;
}
