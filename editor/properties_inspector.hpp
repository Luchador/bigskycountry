#pragma once

#include "marker.hpp"
#include "road.hpp"
#include "intersection.hpp"
#include "road_action.hpp"
#include <bitset>
#include <string_view>

class WorldDocument;
struct RoadEditorState;
struct SceneKey;

enum class PropertiesPage { Road, Railroad, Intersection, Anchor, Terrain, Object, Marker, Other };

enum class RoadProperty { Names, Material, TransitionMaterial, Markings, Alignment, Layout, Left, Right, Edge, Divided, Median, Width, Count };

struct PropertiesInspector
{
    PropertiesPage page = PropertiesPage::Road;
    std::string title, detail, material, markerText;
    std::optional<std::string> roadNames; // Also present for an unnamed road or draft.
    bool paintMarkings = true;
    bool hasDeadEnd = false;
    std::optional<double> deadEndLength; // Editable only for the selected free endpoint.
    std::optional<double> bridgeDepth; // All selected segments must belong to a bridge.
    bool mixedBridgeDepth = false;
    bool HasRoadDetail() const { return deadEndLength.has_value() || bridgeDepth.has_value(); }
    const char* RoadDetailLabel() const { return bridgeDepth ? "Deck thickness (m)" : "End transition (m)"; }
    const char* RoadDetailHint() const { return bridgeDepth ? "Overpass" : "0 = no dead end"; }
    std::string RoadDetailText() const;
    RoadStyle style;
    std::bitset<static_cast<size_t>(RoadProperty::Count)> mixed;
    bool Mixed(RoadProperty property) const { return mixed.test(static_cast<size_t>(property)); }
    int minLanes = 2, maxLanes = 2;
    bool segmentSelection = false; // Lane, shoulder and division controls use this scope.
    double minExtraWidth = 0.0, maxExtraWidth = 0.0;
    double minGap = 4, maxGap = 4;
    bool canDivide = true, anyDivided = false;
    bool AllDivided() const { return anyDivided && !Mixed(RoadProperty::Divided); }
    bool HasTransitionSlot() const
    {
        return (page == PropertiesPage::Road || page == PropertiesPage::Railroad) && materialSlot &&
            (hasDeadEnd || (style.edge == RoadEdge::Transition && !Mixed(RoadProperty::Edge)));
    }
    bool OneLane() const { return maxLanes == 1; }
    bool ThreeLanes() const { return !style.oneWay && minLanes == 3 && maxLanes == 3; }
    bool HasMedian() const { return AllDivided() && minGap > RoadStyle::minSeparation; }
    int StepLanes(bool increase) const;
    bool CanStepLanes(bool increase) const { return minLanes != maxLanes || minLanes != StepLanes(increase); }
    void ApplyRoadStyle(RoadStyle& destination, RoadAction action) const;
    double StepGap(bool increase) const;
    Marker::TextColor textColor{255, 255, 255};
    const char* MaterialCaption() const;
    bool materialSlot = false, invincible = false;
    bool lake = false, editingLake = false;
    bool stopLine = false;
    double radius = 0;
    bool yIntersection = false, island = false;
    double intersectionSize = 60;
    std::string IntersectionSizeText() const;
    IntersectionPaint intersectionPaint = IntersectionPaint::Standard;
    bool operator==(const PropertiesInspector&) const = default;
};

PropertiesInspector InspectProperties(const WorldDocument& world, const RoadEditorState& state, bool editingDraft);
std::string* FindPlacedMaterial(WorldDocument& world, SceneKey key);
std::string FormatTextColor(Marker::TextColor color);
std::optional<Marker::TextColor> ParseTextColor(std::string_view text);
