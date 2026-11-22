#include "world_document.hpp"
#include "river_junction.hpp"
#include <algorithm>
#include <functional>

bool WorldDocument::ConnectRiver(int tributary, int end, int receiver, Vector3 position)
{
    if (tributary<0 || receiver<0 || tributary==receiver || tributary>=static_cast<int>(terrain.size()) ||
        receiver>=static_cast<int>(terrain.size()) || !terrain[tributary]->River() || !terrain[receiver]->HasWater())
        return SDL_SetError("Choose a river end and a different river or lake");
    auto& child=*terrain[tributary]; auto& parent=*terrain[receiver];
    if (child.EditingWater() || parent.EditingWater()) return SDL_SetError("Finish water editing before connecting a river");
    auto definition=*child.River(); const auto original=definition;
    definition.connection=RiverJunction::Locate({definition,child.Instance().transform},end,
        {*parent.WaterSource(),parent.Instance().transform},parent.persistentId,position);
    if (!definition.connection) return false;
    if (!child.SetRiver(definition)) return false;
    if (RefreshRiverConnections()) return true;
    const std::string error=SDL_GetError();
    child.SetRiver(original); RefreshRiverConnections();
    return SDL_SetError("%s",error.c_str());
}

bool WorldDocument::RefreshRiverConnections()
{
    if (riverRevision==TerrainChunk::LatestWaterRevision()) return true;
    std::map<std::uint64_t,TerrainChunk*> waters;
    for (auto& chunk:terrain) if (chunk->HasWater()) waters.emplace(chunk->persistentId,chunk.get());
    // Authoring previews keep the last complete network until Finish Editing.
    for (const auto& [id,water]:waters) if (water->EditingWater()) return true;
    std::map<std::uint64_t,std::uint64_t> roots;
    std::set<std::uint64_t> visiting;
    std::function<bool(std::uint64_t)> visit=[&](auto id) {
        if (roots.contains(id)) return true;
        if (!visiting.insert(id).second) return SDL_SetError("River connections cannot form a cycle");
        auto root=id; const auto& river=waters.at(id)->River();
        const auto link=river ? river->connection : std::nullopt;
        // Missing receivers remain dormant so deleting/undoing a water body restores
        // its connections without relying on unstable outliner indices.
        if (link && waters.contains(link->waterId))
        {
            if (!visit(link->waterId)) return false;
            root=roots.at(link->waterId);
        }
        roots[id]=root; visiting.erase(id); return true;
    };
    for (const auto& [id,water]:waters) if (!visit(id)) return false;
    std::map<std::uint64_t,std::vector<std::uint64_t>> groups;
    for (const auto& [id,root]:roots) groups[root].push_back(id);
    struct Change { TerrainChunk* chunk; WaterDefinition definition; LakeGeometry geometry; bool joined; };
    std::vector<Change> changes;
    std::set<std::uint64_t> rebuilt;
    for (const auto& [root,ids]:groups)
    {
        std::vector<std::pair<std::uint64_t,std::uint64_t>> signature;
        for (auto id:ids) signature.push_back({id,waters.at(id)->WaterRevision()});
        if (riverNetworks.contains(root) && riverNetworks.at(root)==signature) continue;
        rebuilt.insert(root);
        if (ids.size()==1)
        {
            auto* chunk=waters.at(root);
            if (chunk->JoinedWater())
            {
                auto geometry=std::visit([&](const auto& source) {
                    if constexpr (std::is_same_v<std::decay_t<decltype(source)>,LakeDefinition>)
                        return geometryCache ? geometryCache->Lake(source) : LakeGeometry::Build(source);
                    else return geometryCache ? geometryCache->River(source) : RiverGeometry::Build(source);
                },*chunk->WaterSource());
                if (!geometry) return false;
                changes.push_back({chunk,*chunk->WaterSource(),std::move(*geometry),false});
            }
            continue;
        }
        std::vector<std::uint64_t> order;
        std::map<std::uint64_t,std::size_t> indices;
        std::vector<RiverJunction::Source> sources;
        std::function<bool(std::uint64_t)> resolve=[&](auto id) {
            if (indices.contains(id)) return true;
            auto* chunk=waters.at(id); RiverJunction::Source source{*chunk->WaterSource(),chunk->Instance().transform};
            if (id!=root)
            {
                auto parent=std::get<RiverDefinition>(source.definition).connection->waterId;
                if (!resolve(parent) || !RiverJunction::Attach(source,sources[indices.at(parent)])) return false;
            }
            indices[id]=sources.size(); order.push_back(id); sources.push_back(std::move(source)); return true;
        };
        for (auto id:ids) if (!resolve(id)) return false;
        auto geometry=RiverJunction::Build(sources); if (!geometry) return false;
        for (std::size_t i=0;i<order.size();++i)
            changes.push_back({waters.at(order[i]),std::move(sources[i].definition),std::move((*geometry)[i]),true});
    }
    // Publish only after every affected component validates successfully.
    for (auto& change:changes) change.chunk->ApplyWaterGeometry(change.definition,std::move(change.geometry),change.joined);
    std::erase_if(riverNetworks,[&](const auto& item) { return !groups.contains(item.first); });
    for (auto root:rebuilt)
    {
        auto& signature=riverNetworks[root]; signature.clear();
        for (auto id:groups.at(root)) signature.push_back({id,waters.at(id)->WaterRevision()});
    }
    riverRevision=TerrainChunk::LatestWaterRevision();
    return true;
}
