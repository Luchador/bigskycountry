#include "properties_inspector.hpp"
#include "edit_history.hpp"
#include "world_document.hpp"
#include <charconv>

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
    if (editingDraft || target.kind == SelectionKind::Road || target.kind == SelectionKind::Point ||
        (target.kind == SelectionKind::None && state.outlineSelection.empty()))
    {
        const Road* road = editingDraft ? &*state.draft : state.selected >= 0 ? &world.roads.at(state.selected) : nullptr;
        if (road && !editingDraft)
        {
            constexpr double metersPerMile = 1609.344;
            const double meters = road->SplineLengthMeters();
            char label[96];
            SDL_snprintf(label, sizeof(label), "Length: %.2f m (%.3f mi)", meters, meters / metersPerMile);
            result.title = label;
        }
        else
        {
            result.title = editingDraft ? "Road editor | Draft" : "Road defaults";
        }
        result.materialSlot = road != nullptr;
        if (road)
        {
            result.material = road->style.material;
            result.roadNames = road->NameList();
            result.paintMarkings = road->paintMarkings;
        }
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
