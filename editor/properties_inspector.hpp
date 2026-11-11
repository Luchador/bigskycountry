#pragma once

#include <string>

class WorldDocument;
struct RoadEditorState;

enum class PropertiesPage { Road, Intersection, Anchor, Terrain, Object, Marker, Other };

struct PropertiesInspector
{
    PropertiesPage page = PropertiesPage::Road;
    std::string title, detail, material, markerText;
    bool materialSlot = false, invincible = false;
    bool stopLine = false;
    double radius = 0;
    bool operator==(const PropertiesInspector&) const = default;
};

PropertiesInspector InspectProperties(const WorldDocument& world, const RoadEditorState& state, bool editingDraft);
