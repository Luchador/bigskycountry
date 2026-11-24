#pragma once
#include "mesh.hpp"

// Owned, modeless utility window. Bakes run on a cancellable CPU worker.
class MeshAOEditor
{
public:
    MeshAOEditor();
    ~MeshAOEditor();
    bool Open(SDL_Window* owner, std::shared_ptr<const Mesh> mesh);
    void Close();
    bool Draw();
    bool Handle(const SDL_Event& event);
    bool Focused() const;
    bool ContainsScreen(float x, float y) const;
#ifdef _WIN32
    bool HandleNativeMessage(void* message);
#endif
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
