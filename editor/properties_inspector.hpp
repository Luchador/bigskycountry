#pragma once

#include "marker.hpp"
#include <string_view>

class WorldDocument;
struct RoadEditorState;
struct SceneKey;

enum class PropertiesPage { Road, Intersection, Anchor, Terrain, Object, Marker, Other };

struct PropertiesInspector
{
    PropertiesPage page = PropertiesPage::Road;
    std::string title, detail, material, markerText;
    std::optional<std::string> roadNames; // Also present for an unnamed road or draft.
    bool paintMarkings = true;
    Marker::TextColor textColor{255, 255, 255};
    const char* MaterialCaption() const;
    bool materialSlot = false, invincible = false;
    bool stopLine = false;
    double radius = 0;
    bool operator==(const PropertiesInspector&) const = default;
};

PropertiesInspector InspectProperties(const WorldDocument& world, const RoadEditorState& state, bool editingDraft);
std::string* FindPlacedMaterial(WorldDocument& world, SceneKey key);
std::string FormatTextColor(Marker::TextColor color);
std::optional<Marker::TextColor> ParseTextColor(std::string_view text);
