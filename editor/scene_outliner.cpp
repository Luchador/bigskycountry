#include "scene_outliner.hpp"
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

void SceneOutliner::Update(const WorldDocument& world, const RoadEditorState& state)
{
    const auto primary = state.outlineSelection.empty() ? state.SelectedRows() : std::set<SceneKey>{};
    const auto& selected = state.outlineSelection.empty() ? primary : state.outlineSelection;
    const std::array sizes{world.roads.size(), world.Terrain().size(), world.Objects().size(), world.meshes.size()};
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
    for (const auto& [id, f] : world.hierarchy.Folders())
    {
        children[f.parent].push_back({{SceneKind::Folder, id}, f.name, "Folder", 0, false, false});
    }
    const auto add = [&](SceneKind kind, int index, std::string name, std::string type)
    {
        const SceneKey key{kind, index};
        children[world.hierarchy.Parent(key)].push_back({key, std::move(name), std::move(type), 0, false, false});
    };
    for (int i = 0; i < static_cast<int>(sizes[0]); ++i)
    {
        add(SceneKind::Road, i, "Road " + std::to_string(i + 1), "Road");
    }
    for (int i = 0; i < static_cast<int>(sizes[1]); ++i)
    {
        add(SceneKind::Terrain, i, ModelLabel(world.Terrain()[i]->Instance(), i), "Terrain");
    }
    for (int i = 0; i < static_cast<int>(sizes[2]); ++i)
    {
        add(SceneKind::Object, i, ModelLabel(world.Objects()[i]->Instance(), i),
            ObjectLabel(world.Objects()[i]->Type()));
    }
    for (int i = 0; i < static_cast<int>(sizes[3]); ++i)
    {
        add(SceneKind::Mesh, i, ModelLabel(world.meshes[i], i), "Static mesh");
    }

    rows.clear();
    std::vector<OutlineRow> stack{{{}, Stem(world.Path()), "World", 0, true, true}};
    while (!stack.empty())
    {
        auto row = std::move(stack.back());
        stack.pop_back();
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
    case OutlineOp::Remove:
        if (!world.hierarchy.Remove(id))
        {
            return false;
        }
        collapsed.erase(id);
        state.SelectRows({});
        break;
    case OutlineOp::Move:
        if (!world.hierarchy.Move(cmd.items, id))
        {
            return false;
        }
        collapsed.erase(id);
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
