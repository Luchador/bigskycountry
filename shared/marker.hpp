#pragma once

#include "bscobject.hpp"
#include <array>
#include <optional>

class FreeCamera;
class WorldRenderer;

class Marker final : public BSCObject
{
public:
    static constexpr char modelPath[] = "objects/marker/marker.glb";
    static constexpr double labelGap = 50.0;   // Game meters above the transformed model.
    static constexpr double capHeight = 100.0; // Game meters for a capital H.
    static constexpr int maxCharacters = 120;

    ObjectType Type() const override { return ObjectType::Marker; }
    std::unique_ptr<BSCObject> Clone() const override { return std::make_unique<Marker>(*this); }
    const std::string& Text() const { return text; }
    bool SetText(std::string value);
    void SubmitLabel(WorldRenderer& renderer, const FreeCamera& camera) const;
    std::string material; // Per-instance .bmat path. Empty uses the GLB's mesh colors.
    using TextColor = std::array<Uint8, 3>;
    TextColor textColor{255, 255, 255}; // Display RGB for the unlit lettering.

private:
    MeshInstance Label(const FreeCamera& camera) const;
    mutable double labelRadius = 0;
    std::string text = "Marker";
    mutable std::shared_ptr<const Mesh> lettering;
    mutable std::optional<ObjectTransform> measuredTransform;
    mutable std::weak_ptr<const Mesh> measuredMesh;
    mutable Vector3 labelOrigin{};
};
