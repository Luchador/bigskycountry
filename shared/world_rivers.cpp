#include "world_document.hpp"
#include "river_junction.hpp"
#include <algorithm>
#include <functional>

bool WorldDocument::ConnectRiver(int tributary, int end, int receiver, Vector3 position)
{
    if (tributary<0 || receiver<0 || tributary==receiver || tributary>=static_cast<int>(terrain.size()) ||
        receiver>=static_cast<int>(terrain.size()) || !terrain[tributary]->River() || !terrain[receiver]->River())
        return SDL_SetError("Choose two different rivers");
    auto& child=*terrain[tributary]; auto& parent=*terrain[receiver];
    if (child.EditingWater() || parent.EditingWater()) return SDL_SetError("Finish river editing before connecting rivers");
    auto definition=*child.River(); const auto original=definition;
    definition.connection=RiverJunction::Locate({definition,child.Instance().transform},end,
        {*parent.River(),parent.Instance().transform},parent.persistentId,position);
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
    std::map<std::uint64_t,TerrainChunk*> rivers;
    for (auto& chunk:terrain) if (chunk->River()) rivers.emplace(chunk->persistentId,chunk.get());
    // Authoring previews keep the last complete network until Finish Editing.
    for (const auto& [id,river]:rivers) if (river->EditingWater()) return true;
    std::map<std::uint64_t,std::uint64_t> roots;
    std::set<std::uint64_t> visiting;
    std::function<bool(std::uint64_t)> visit=[&](auto id) {
        if (roots.contains(id)) return true;
        if (!visiting.insert(id).second) return SDL_SetError("River connections cannot form a cycle");
        auto root=id; const auto& link=rivers.at(id)->River()->connection;
        // Missing receivers remain dormant so deleting/undoing a river restores
        // its connections without relying on unstable outliner indices.
        if (link && rivers.contains(link->riverId))
        {
            if (!visit(link->riverId)) return false;
            root=roots.at(link->riverId);
        }
        roots[id]=root; visiting.erase(id); return true;
    };
    for (const auto& [id,river]:rivers) if (!visit(id)) return false;
    std::map<std::uint64_t,std::vector<std::uint64_t>> groups;
    for (const auto& [id,root]:roots) groups[root].push_back(id);
    struct Change { TerrainChunk* chunk; RiverDefinition definition; LakeGeometry geometry; bool joined; };
    std::vector<Change> changes;
    std::set<std::uint64_t> rebuilt;
    for (const auto& [root,ids]:groups)
    {
        std::vector<std::pair<std::uint64_t,std::uint64_t>> signature;
        for (auto id:ids) signature.push_back({id,rivers.at(id)->WaterRevision()});
        if (riverNetworks.contains(root) && riverNetworks.at(root)==signature) continue;
        rebuilt.insert(root);
        if (ids.size()==1)
        {
            auto* chunk=rivers.at(root);
            if (chunk->JoinedRiver())
            {
                auto geometry=geometryCache ? geometryCache->River(*chunk->River()) : RiverGeometry::Build(*chunk->River());
                if (!geometry) return false;
                changes.push_back({chunk,*chunk->River(),std::move(*geometry),false});
            }
            continue;
        }
        std::vector<std::uint64_t> order;
        std::map<std::uint64_t,std::size_t> indices;
        std::vector<RiverJunction::Source> sources;
        std::function<bool(std::uint64_t)> resolve=[&](auto id) {
            if (indices.contains(id)) return true;
            auto* chunk=rivers.at(id); RiverJunction::Source source{*chunk->River(),chunk->Instance().transform};
            if (id!=root)
            {
                auto parent=source.definition.connection->riverId;
                if (!resolve(parent) || !RiverJunction::Attach(source,sources[indices.at(parent)])) return false;
            }
            indices[id]=sources.size(); order.push_back(id); sources.push_back(std::move(source)); return true;
        };
        for (auto id:ids) if (!resolve(id)) return false;
        auto geometry=RiverJunction::Build(sources); if (!geometry) return false;
        for (std::size_t i=0;i<order.size();++i)
            changes.push_back({rivers.at(order[i]),std::move(sources[i].definition),std::move((*geometry)[i]),true});
    }
    // Publish only after every affected component validates successfully.
    for (auto& change:changes) change.chunk->ApplyRiverGeometry(change.definition,std::move(change.geometry),change.joined);
    std::erase_if(riverNetworks,[&](const auto& item) { return !groups.contains(item.first); });
    for (auto root:rebuilt)
    {
        auto& signature=riverNetworks[root]; signature.clear();
        for (auto id:groups.at(root)) signature.push_back({id,rivers.at(id)->WaterRevision()});
    }
    riverRevision=TerrainChunk::LatestWaterRevision();
    return true;
}
