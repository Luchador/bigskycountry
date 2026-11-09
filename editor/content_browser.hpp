#pragma once

#include <SDL3/SDL.h>
#include <string>
#include <vector>

// Lists model files without loading the whole asset collection.
class ContentBrowser
{
public:
    void Refresh();
    void HandlePointer(const SDL_Event& event, SDL_FPoint local, int width, int height);
    bool Draw(SDL_Renderer* renderer, int width, int height);

private:
    struct Row
    {
        std::string label;
        std::string modelPath; // Empty for a folder heading.
    };

    void UpdatePreview(SDL_Renderer* renderer);
    std::vector<Row> rows;
    std::string selected;
    std::string scanStatus;
    std::string previewStatus;
    std::size_t triangleCount = 0;
    float scroll = 0;
    bool previewDirty = true;
    SDL_Texture* preview = nullptr; // Owned by the editor renderer. Replaced while it is live.
};
