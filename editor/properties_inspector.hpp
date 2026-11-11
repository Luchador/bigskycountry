#pragma once

#include <string>

class WorldDocument;
struct RoadEditorState;

enum class PropertiesPage { Road, Intersection, Terrain, Object, Other };

struct PropertiesInspector
{
    PropertiesPage page = PropertiesPage::Road;
    std::string title, detail, material;
    bool materialSlot = false, invincible = false;
    double radius = 0;
    bool operator==(const PropertiesInspector&) const = default;
};

PropertiesInspector InspectProperties(const WorldDocument& world, const RoadEditorState& state, bool editingDraft);
