#pragma once
#include "bscobject.hpp"

// Editor-only placement; the cook stores its pose as startup metadata, never as
// visible geometry or collision. The model's +Z direction is the car's heading.
class SpawnPoint final : public BSCObject
{
public:
    static constexpr char modelPath[] = "editor/start.glb";
    SpawnPoint() { invincible = true; }
    ObjectType Type() const override { return ObjectType::SpawnPoint; }
    bool EditorOnly() const override { return true; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<SpawnPoint>(*this); }
};
