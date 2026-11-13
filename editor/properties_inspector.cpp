#include "properties_inspector.hpp"
#include "edit_history.hpp"
#include "world_document.hpp"
#include <algorithm>
#include <charconv>

int PropertiesInspector::StepLanes(bool increase) const
{
    int count = increase ? maxLanes + 1 : minLanes - 1;
    if (anyDivided && count % 2 != 0) count += increase ? 1 : -1;
    return std::clamp(count, anyDivided ? 2 : 1, RoadStyle::maxLanes);
}

double PropertiesInspector::StepGap(bool increase) const
{
    return increase ? maxGap + 0.5 : std::max(RoadStyle::minSeparation, minGap - 0.5);
}

void PropertiesInspector::ApplyRoadStyle(RoadStyle& destination, RoadAction action) const
{
    switch (action)
    {
    case RoadAction::Straight:
        destination.curved = false;
        break;
    case RoadAction::Curve:
        destination.curved = true;
        break;
    case RoadAction::Fewer:
    case RoadAction::More:
        destination.SetLaneCount(StepLanes(action == RoadAction::More));
        break;
    case RoadAction::LessExtraWidth:
    case RoadAction::MoreExtraWidth:
        if (OneLane())
        {
            const double width = action == RoadAction::MoreExtraWidth ? maxExtraWidth + 0.5 : std::max(0.0, minExtraWidth - 0.5);
            destination.SetSingleLaneExtraWidth(width);
        }
        break;
    case RoadAction::TwoNorthbound:
        if (ThreeLanes()) destination.threeLaneLayout = ThreeLaneLayout::TwoNorthbound;
        break;
    case RoadAction::TwoSouthbound:
        if (ThreeLanes()) destination.threeLaneLayout = ThreeLaneLayout::TwoSouthbound;
        break;
    case RoadAction::CenterTurn:
        if (ThreeLanes()) destination.threeLaneLayout = ThreeLaneLayout::CenterTurn;
        break;
    case RoadAction::Left:
        destination.leftShoulder = Mixed(RoadProperty::Left) || !style.leftShoulder;
        break;
    case RoadAction::Right:
        destination.rightShoulder = Mixed(RoadProperty::Right) || !style.rightShoulder;
        break;
    case RoadAction::NoEdge:
        destination.edge = RoadEdge::None;
        break;
    case RoadAction::Transition:
        destination.edge = RoadEdge::Transition;
        break;
    case RoadAction::Curb:
        destination.edge = RoadEdge::Curb;
        break;
    case RoadAction::Divide:
        destination.SetDivided(canDivide && !AllDivided());
        break;
    case RoadAction::NarrowerGap:
    case RoadAction::WiderGap:
        if (AllDivided()) destination.SetDividedGap(StepGap(action == RoadAction::WiderGap));
        break;
    case RoadAction::ConcreteMedian:
    case RoadAction::GrassMedian:
        if (!HasMedian()) return;
        destination.median = action == RoadAction::GrassMedian ? RoadMedian::Grass : RoadMedian::Concrete;
        break;
    default:
        return;
    }
}

const char* PropertiesInspector::MaterialCaption() const
{
    if (page == PropertiesPage::Marker) return "Mesh material";
    if (page == PropertiesPage::Terrain) return "Terrain material";
    return "Pavement material";
}

std::string* FindPlacedMaterial(WorldDocument& world, SceneKey key)
{
    if (!world.Contains(key)) return nullptr;
    if (key.kind == SceneKind::Intersection) return &world.intersections.at(key.index).material;
    if (key.kind == SceneKind::Terrain) return &world.Terrain()[key.index]->material;
    if (key.kind == SceneKind::Object)
    {
        if (auto* marker = dynamic_cast<Marker*>(world.Objects()[key.index].get())) return &marker->material;
    }
    return nullptr;
}

std::string FormatTextColor(Marker::TextColor color)
{
    char hex[8];
    SDL_snprintf(hex, sizeof(hex), "#%02X%02X%02X", unsigned(color[0]), unsigned(color[1]), unsigned(color[2]));
    return hex;
}

std::optional<Marker::TextColor> ParseTextColor(std::string_view text)
{
    if (text.starts_with('#')) text.remove_prefix(1);
    if (text.size() != 6) return {};
    unsigned rgb;
    const auto end = text.data() + text.size();
    const auto [ptr, error] = std::from_chars(text.data(), end, rgb, 16);
    if (error != std::errc{} || ptr != end) return {};
    return Marker::TextColor{Uint8(rgb >> 16), Uint8(rgb >> 8), Uint8(rgb)};
}

PropertiesInspector InspectProperties(const WorldDocument& world, const RoadEditorState& state, bool editingDraft)
{
    PropertiesInspector result;
    const auto target = state.Target();
    const bool group = !editingDraft && state.MultipleRoadsSelected();
    if (editingDraft || group || target.kind == SelectionKind::Road || target.kind == SelectionKind::Point ||
        (target.kind == SelectionKind::None && state.outlineSelection.empty()))
    {
        const Road* road = editingDraft ? &*state.draft : group ? &world.roads.at(state.SelectedRoadRows().front().index) :
                           state.selected >= 0 ? &world.roads.at(state.selected) : nullptr;
        const auto& first = road ? road->style : state.defaults;
        result.style = first;
        result.minLanes = result.maxLanes = first.GetLaneCount();
        result.minExtraWidth = result.maxExtraWidth = first.SingleLaneExtraWidth();
        result.minGap = result.maxGap = first.DividedGap();
        result.anyDivided = first.IsDivided();
        result.canDivide = first.CanDivide();
        result.materialSlot = road != nullptr;
        if (road)
        {
            result.material = first.material;
            result.roadNames = road->NameList();
            result.paintMarkings = road->paintMarkings;
        }

        double meters = road ? road->SplineLengthMeters() : 0;
        int roadCount = 0;
        if (group)
        {
            const auto compare = [&](RoadProperty field, bool differs)
            {
                if (differs) result.mixed.set(static_cast<size_t>(field));
            };
            const auto width = [](const RoadStyle& s) { return s.LeftWidth() + s.RightWidth() + 2 * s.EdgeWidth(); };
            meters = 0;
            for (const auto key : state.SelectedRoadRows())
            {
                ++roadCount;
                const auto& other = world.roads.at(key.index);
                const auto& s = other.style;
                meters += other.SplineLengthMeters();
                result.minLanes = std::min(result.minLanes, s.GetLaneCount());
                result.maxLanes = std::max(result.maxLanes, s.GetLaneCount());
                result.minExtraWidth = std::min(result.minExtraWidth, s.SingleLaneExtraWidth());
                result.maxExtraWidth = std::max(result.maxExtraWidth, s.SingleLaneExtraWidth());
                result.minGap = std::min(result.minGap, s.DividedGap());
                result.maxGap = std::max(result.maxGap, s.DividedGap());
                result.anyDivided |= s.IsDivided();
                result.canDivide &= s.CanDivide();
                compare(RoadProperty::Names, *result.roadNames != other.NameList());
                compare(RoadProperty::Material, first.material != s.material);
                compare(RoadProperty::Markings, result.paintMarkings != other.paintMarkings);
                compare(RoadProperty::Alignment, first.curved != s.curved);
                compare(RoadProperty::Layout, first.threeLaneLayout != s.threeLaneLayout);
                compare(RoadProperty::Left, first.leftShoulder != s.leftShoulder);
                compare(RoadProperty::Right, first.rightShoulder != s.rightShoulder);
                compare(RoadProperty::Edge, first.edge != s.edge);
                compare(RoadProperty::Divided, first.IsDivided() != s.IsDivided());
                compare(RoadProperty::Median, first.median != s.median);
                compare(RoadProperty::Width, width(first) != width(s));
            }
            if (result.Mixed(RoadProperty::Names)) result.roadNames->clear();
            if (result.Mixed(RoadProperty::Material)) result.material.clear();
        }
        if (road && !editingDraft)
        {
            constexpr double metersPerMile = 1609.344;
            char length[96];
            SDL_snprintf(length, sizeof(length), "%.2f m (%.3f mi)", meters, meters / metersPerMile);
            result.title = group ? std::to_string(roadCount) + " roads | " : "Length: ";
            result.title += length;
        }
        else result.title = editingDraft ? "Road editor | Draft" : "Road defaults";
    }
    else if (target.kind == SelectionKind::Intersection)
    {
        const auto& item = world.intersections.at(target.index);
        result.page = PropertiesPage::Intersection;
        result.title = "Intersection " + std::to_string(target.index);
        result.radius = item.cornerRadius;
        result.material = item.material;
        result.materialSlot = true;
    }
    else if (target.kind == SelectionKind::Anchor)
    {
        static constexpr std::array names{"Local +X anchor", "Local +Z anchor", "Local -X anchor", "Local -Z anchor"};
        result.page = PropertiesPage::Anchor;
        result.title = "Intersection " + std::to_string(target.index);
        result.detail = names.at(target.point);
        result.stopLine = world.intersections.at(target.index).stopLines.test(target.point);
    }
    else if (target.kind == SelectionKind::Terrain)
    {
        const auto& chunk = *world.Terrain().at(target.index);
        result.page = PropertiesPage::Terrain;
        result.title = "Terrain " + std::to_string(target.index + 1);
        if (chunk.boundary)
            result.detail = chunk.Instance().mesh ? "Follows road boundary" : "Road boundary is open";
        else result.detail = chunk.Instance().mesh->assetPath;
        result.material = chunk.material;
        result.materialSlot = true;
    }
    else if (target.kind == SelectionKind::Object)
    {
        const auto& object = *world.Objects().at(target.index);
        result.page = PropertiesPage::Object;
        result.title = "Object " + std::to_string(target.index + 1);
        result.detail = object.Instance().mesh->assetPath;
        result.invincible = object.invincible;
        if (const auto* marker = dynamic_cast<const Marker*>(&object))
        {
            result.page = PropertiesPage::Marker;
            result.title = "Marker " + std::to_string(target.index + 1);
            result.markerText = marker->Text();
            result.textColor = marker->textColor;
            result.material = marker->material;
            result.materialSlot = true;
        }
    }
    else
    {
        result.page = PropertiesPage::Other;
        result.title = "Properties";
    }
    return result;
}
