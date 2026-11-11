#include "properties_inspector.hpp"
#include "edit_history.hpp"
#include "world_document.hpp"

PropertiesInspector InspectProperties(const WorldDocument& world, const RoadEditorState& state, bool editingDraft)
{
    PropertiesInspector result;
    const auto target = state.Target();
    if (editingDraft || target.kind == SelectionKind::Road || target.kind == SelectionKind::Point ||
        (target.kind == SelectionKind::None && state.outlineSelection.empty()))
    {
        const Road* road = editingDraft ? &*state.draft : state.selected >= 0 ? &world.roads.at(state.selected) : nullptr;
        result.title = editingDraft ? "Road editor | Draft" : road ? "Road editor | Road " + std::to_string(state.selected + 1) : "Road defaults";
        result.materialSlot = road != nullptr;
        if (road) result.material = road->style.material;
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
        result.detail = chunk.Instance().mesh->assetPath;
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
    }
    else
    {
        result.page = PropertiesPage::Other;
        result.title = "Properties";
    }
    return result;
}
