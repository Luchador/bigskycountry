#pragma once

#include "marker.hpp"
#include "road.hpp"
#include "road_action.hpp"
#include <bitset>
#include <string_view>

class WorldDocument;
struct RoadEditorState;
struct SceneKey;

enum class PropertiesPage { Road, Intersection, Anchor, Terrain, Object, Marker, Other };

enum class RoadProperty { Names, Material, TransitionMaterial, Markings, Alignment, Layout, Left, Right, Edge, Divided, Median, Width, Count };

struct PropertiesInspector
{
    PropertiesPage page = PropertiesPage::Road;
    std::string title, detail, material, markerText;
    std::optional<std::string> roadNames; // Also present for an unnamed road or draft.
    bool paintMarkings = true;
    RoadStyle style;
    std::bitset<static_cast<size_t>(RoadProperty::Count)> mixed;
    bool Mixed(RoadProperty property) const { return mixed.test(static_cast<size_t>(property)); }
    int minLanes = 2, maxLanes = 2;
    double minExtraWidth = 0.0, maxExtraWidth = 0.0;
    double minGap = 4, maxGap = 4;
    bool canDivide = true, anyDivided = false;
    bool AllDivided() const { return anyDivided && !Mixed(RoadProperty::Divided); }
    bool HasTransitionSlot() const
    {
        return page == PropertiesPage::Road && materialSlot && style.edge == RoadEdge::Transition && !Mixed(RoadProperty::Edge);
    }
    bool OneLane() const { return maxLanes == 1; }
    bool ThreeLanes() const { return minLanes == 3 && maxLanes == 3; }
    bool HasMedian() const { return AllDivided() && minGap > RoadStyle::minSeparation; }
    int StepLanes(bool increase) const;
    bool CanStepLanes(bool increase) const { return minLanes != maxLanes || minLanes != StepLanes(increase); }
    void ApplyRoadStyle(RoadStyle& destination, RoadAction action) const;
    double StepGap(bool increase) const;
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
