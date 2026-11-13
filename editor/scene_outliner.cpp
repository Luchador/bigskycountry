#include "scene_outliner.hpp"
#include "asset_paths.hpp"
#include "marker.hpp"
#include "world_document.hpp"

#include <SDL3/SDL.h>
#include <algorithm>

namespace
{
    std::string ObjectLabel(ObjectType type)
    {
        switch (type)
        {
        case ObjectType::Building:
            return "Building";
        case ObjectType::Vehicle:
            return "Vehicle";
        case ObjectType::Character:
            return "Character";
        case ObjectType::Marker:
            return "Marker";
        case ObjectType::Prop:
            return "Prop";
        default:
            return "Object";
        }
    }

    std::string Stem(std::string_view path)
    {
        const auto slash = path.find_last_of("/\\");
        const auto file = path.substr(slash == path.npos ? 0 : slash + 1);
        const auto dot = file.find_last_of('.');
        return std::string(file.substr(0, dot == 0 ? file.npos : dot));
    }

    std::string ModelLabel(const MeshInstance& instance, int i)
    {
        const std::string path = instance.mesh ? instance.mesh->assetPath : "";
        return Stem(path) + " #" + std::to_string(i + 1);
    }
} // namespace

std::string OutlineRow::DragLabel(std::size_t count) const
{
    return count > 1 ? label + " (+" + std::to_string(count - 1) + ")" : label;
}

SDL_Surface* LoadOutlineMoveIcon(bool allowed)
{
    const auto path = AssetPaths::Root(AssetRoot::Project) +
                      (allowed ? "textures/editor/move_check.png" : "textures/editor/move_block.png");
    SDL_Surface* image = SDL_LoadPNG(path.c_str());
    if (!image) return nullptr;
    SDL_Surface* icon = SDL_ConvertSurface(image, SDL_PIXELFORMAT_BGRA32);
    SDL_DestroySurface(image);
    return icon;
}

bool SceneOutliner::CanDrop(const std::set<SceneKey>& items, SceneKey target, FolderPlacement placement) const
{
    return document && (target.kind == SceneKind::Folder || target.kind == SceneKind::World) &&
           document->Contains(target) &&
           std::all_of(items.begin(), items.end(), [&](SceneKey key) { return document->Contains(key); }) &&
           document->hierarchy.CanMove(items, target.index, placement);
}

std::optional<OutlineDrop> SceneOutliner::DropAt(const std::set<SceneKey>& items, int row, int yInRow, int height, int edge) const
{
    const int count = static_cast<int>(rows.size());
    if (!document || items.empty() || row < 0 || row > count || yInRow < 0 || yInRow >= height) return {};
    std::optional<OutlineDrop> drop;
    if (row < count) drop = OutlineDrop{rows[row].key, FolderPlacement::Into, row, rows[row].depth};

    const bool nearTop = yInRow < edge;
    if ((nearTop || yInRow >= height - edge) &&
        std::all_of(items.begin(), items.end(), [](SceneKey key) { return key.kind == SceneKind::Folder; }))
    {
        const int gap = nearTop ? row : row + 1;
        if (gap > count) return {};
        const auto& hierarchy = document->hierarchy;
        const int sourceParent = hierarchy.Parent(*items.begin());
        std::optional<OutlineDrop> insertion;
        if (gap < count && rows[gap].key.kind == SceneKind::Folder)
            insertion = OutlineDrop{rows[gap].key, FolderPlacement::Before, gap, rows[gap].depth};

        // At the end of an expanded subtree several sibling lists can share one visible gap.
        // Prefer the dragged folder's level so dropping its sibling here preserves the nesting.
        if (gap > 0)
        {
            SceneKey above = rows[gap - 1].key;
            const int nextDepth = gap < count ? rows[gap].depth : 1;
            for (int depth = rows[gap - 1].depth; depth >= nextDepth && depth > 0; --depth)
            {
                const int parent = hierarchy.Parent(above);
                if (above.kind == SceneKind::Folder &&
                    (!insertion || (parent == sourceParent && hierarchy.Parent(insertion->target) != sourceParent)))
                    insertion = OutlineDrop{above, FolderPlacement::After, gap, depth};
                above = {SceneKind::Folder, parent};
            }
        }
        if (insertion) drop = insertion;
    }
    if (drop && CanDrop(items, drop->target, drop->placement)) return drop;
    return {};
}

void SceneOutliner::Update(const WorldDocument& world, const RoadEditorState& state)
{
    document = &world;
    const auto primary = state.outlineSelection.empty() ? state.SelectedRows() : std::set<SceneKey>{};
    const auto& selected = state.outlineSelection.empty() ? primary : state.outlineSelection;
    const std::array sizes{world.roads.size(), world.Terrain().size(), world.Objects().size(), world.meshes.size(), world.intersections.size()};
    if (dirty || selected != selection)
    {
        parentForNew = 0;
        if (selected.size() == 1)
        {
            const auto key = *selected.begin();
            parentForNew = key.kind == SceneKind::Folder ? key.index : world.hierarchy.Parent(key);
        }
        siblingNames.clear();
        for (const auto& [id, folder] : world.hierarchy.Folders())
        {
            if (folder.parent == parentForNew)
            {
                siblingNames.insert(folder.name);
            }
        }
        if (selected != selection)
        {
            for (auto key : selected)
            {
                for (int parent = world.hierarchy.Parent(key); parent;
                     parent = world.hierarchy.Parent({SceneKind::Folder, parent}))
                {
                    if (collapsed.erase(parent))
                    {
                        dirty = true;
                    }
                }
            }
            if (!selected.empty() && !selected.contains({}) && collapsed.erase(0))
            {
                dirty = true;
            }
        }
        selection = selected;
    }
    if (!dirty && counts == sizes)
    {
        return;
    }
    counts = sizes;
    dirty = false;
    ++version;

    std::map<int, std::vector<OutlineRow>> children;
    for (int id : world.hierarchy.OrderedFolderIds())
    {
        const auto& f = world.hierarchy.Folders().at(id);
        children[f.parent].push_back({{SceneKind::Folder, id}, f.name, "Folder", 0, false, false});
    }
    const auto add = [&](SceneKind kind, int index, std::string name, std::string type)
    {
        const SceneKey key{kind, index};
        children[world.hierarchy.Parent(key)].push_back({key, std::move(name), std::move(type), 0, false, false});
    };
    for (int i = 0; i < static_cast<int>(sizes[0]); ++i)
    {
        auto label = world.roads[i].NameList();
        const std::string type = world.roads[i].style.railroad ? "Railroad" : "Road";
        if (label.empty()) label = type + " " + std::to_string(i + 1);
        add(SceneKind::Road, i, std::move(label), type);
    }
    for (int i = 0; i < static_cast<int>(sizes[1]); ++i)
    {
        const auto& chunk = *world.Terrain()[i];
        const auto label = chunk.boundary ? "Road-bounded terrain #" + std::to_string(i + 1) : ModelLabel(chunk.Instance(), i);
        add(SceneKind::Terrain, i, label, "Terrain");
    }
    for (int i = 0; i < static_cast<int>(sizes[2]); ++i)
    {
        const auto& object = *world.Objects()[i];
        const auto* marker = dynamic_cast<const Marker*>(&object);
        add(SceneKind::Object, i, marker && !marker->Text().empty() ? marker->Text() : ModelLabel(object.Instance(), i),
            ObjectLabel(object.Type()));
    }
    for (int i = 0; i < static_cast<int>(sizes[3]); ++i)
    {
        add(SceneKind::Mesh, i, ModelLabel(world.meshes[i], i), "Static mesh");
    }

    for (const auto& [id, item] : world.intersections)
        add(SceneKind::Intersection, id, "Intersection " + std::to_string(id), "Intersection");

    rows.clear();
    std::vector<OutlineRow> stack{{{}, Stem(world.Path()), "World", 0, true, true}};
    while (!stack.empty())
    {
        auto row = std::move(stack.back());
        stack.pop_back();
        row.hidden = world.hierarchy.EditorHidden(row.key);
        if (row.key.kind == SceneKind::World || row.key.kind == SceneKind::Folder)
        {
            const auto it = children.find(row.key.index);
            row.branch = it != children.end();
            row.expanded = !collapsed.contains(row.key.index);
            if (row.branch && row.expanded)
            {
                for (auto child = it->second.rbegin(); child != it->second.rend(); ++child)
                {
                    child->depth = row.depth + 1;
                    stack.push_back(std::move(*child));
                }
            }
        }
        rows.push_back(std::move(row));
    }
}

std::string SceneOutliner::NewFolderName() const
{
    std::string name = "New Folder";
    for (int i = 2; siblingNames.contains(name); ++i)
    {
        name = "New Folder " + std::to_string(i);
    }
    return name;
}

bool SceneOutliner::Apply(const OutlineCommand& cmd, WorldDocument& world, RoadEditorState& state, EditHistory& history)
{
    if (!world.Contains(cmd.target))
    {
        return false;
    }
    for (auto key : cmd.items)
    {
        if (!world.Contains(key))
        {
            return false;
        }
    }
    if (cmd.op == OutlineOp::Select)
    {
        state.SelectRows(cmd.items);
        return false;
    }
    if (cmd.target.kind != SceneKind::World && cmd.target.kind != SceneKind::Folder)
    {
        return false;
    }
    const int id = cmd.target.index;
    if (cmd.op == OutlineOp::Expand)
    {
        if (!collapsed.erase(id))
        {
            collapsed.insert(id);
        }
        dirty = true;
        return false;
    }
    EditSnapshot before(state);
    before.hierarchy = world.hierarchy;
    switch (cmd.op)
    {
    case OutlineOp::Create:
    {
        const int child = world.hierarchy.Create(id, cmd.name);
        if (!child)
        {
            return false;
        }
        collapsed.erase(id);
        collapsed.erase(child);
        state.SelectRows({{SceneKind::Folder, child}});
        break;
    }
    case OutlineOp::Rename:
        if (!world.hierarchy.Rename(id, cmd.name))
        {
            return false;
        }
        break;
    case OutlineOp::HideChildren:
    case OutlineOp::UnhideChildren:
    {
        if (!world.hierarchy.SetChildrenHidden(id, cmd.op == OutlineOp::HideChildren)) return false;
        auto selected = state.SelectedRows();
        if (std::erase_if(selected, [&](SceneKey key) {
                return key.kind != SceneKind::Folder && world.hierarchy.EditorHidden(key);
            }))
            state.SelectRows(selected);
        if (state.anchorSelection && world.hierarchy.EditorHidden({SceneKind::Intersection, state.anchorSelection->intersectionId}))
            state.anchorSelection.reset();
        break;
    }
    case OutlineOp::Remove:
        if (!world.hierarchy.Remove(id))
        {
            return false;
        }
        collapsed.erase(id);
        state.SelectRows({});
        break;
    case OutlineOp::Move:
        if (!world.hierarchy.Move(cmd.items, id, cmd.placement))
        {
            return false;
        }
        if (cmd.placement == FolderPlacement::Into) collapsed.erase(id);
        state.SelectRows(cmd.items);
        break;
    default:
        return false;
    }
    EditSnapshot after(state);
    after.hierarchy = world.hierarchy;
    dirty = true;
    history.Record(-1, std::move(before), std::move(after));
    return true;
}
