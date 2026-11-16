#include "terrain_chunk.hpp"

#include <algorithm>
#include <map>

bool TerrainChunk::SetLake(const LakeDefinition& definition)
{
    if (lakeDraft)
    {
        if (*lakeDraft == definition) return true;
        auto samples = LakeGeometry::Sample(definition);
        if (!samples) return false;
        lakeDraft = definition;
        draftShoreline = std::move(*samples);
        return true;
    }
    if (lake == definition) return true;
    auto geometry = LakeGeometry::Build(definition);
    if (!geometry) return false;
    lake = definition;
    SetMesh(std::move(geometry->bed));
    water = {std::move(geometry->water), Instance().transform};
    shoreline = std::move(geometry->shoreline);
    return true;
}

bool TerrainChunk::BeginLakeEdit()
{
    if (!lake) return SDL_SetError("Select a lake to edit its shoreline");
    if (lakeDraft) return true;
    lakeDraft = lake;
    draftShoreline = shoreline;
    return true;
}

bool TerrainChunk::FinishLakeEdit()
{
    if (!lakeDraft) return true;
    if (lakeDraft != lake)
    {
        auto geometry = LakeGeometry::Build(*lakeDraft);
        if (!geometry) return false;
        lake = std::move(lakeDraft);
        SetMesh(std::move(geometry->bed));
        water = {std::move(geometry->water), Instance().transform};
        shoreline = std::move(geometry->shoreline);
    }
    CancelLakeEdit();
    return true;
}

void TerrainChunk::CancelLakeEdit()
{
    lakeDraft.reset();
    draftShoreline.clear();
}

const std::vector<TerrainChunk::OutlineEdge>& TerrainChunk::Outline() const
{
    const auto& mesh = Instance().mesh;
    if (!mesh)
    {
        outline.clear();
        outlinedMesh.reset();
        return outline;
    }
    if (outlinedMesh.lock() == mesh) return outline;
    outlinedMesh = mesh;
    outline.clear();

    // Weld coincident positions so imported normal/UV seams do not become
    // selection lines. Only edges used by one triangle belong to the rim.
    std::map<std::array<double, 3>, std::size_t> positions;
    std::vector<std::size_t> welded;
    welded.reserve(mesh->vertices.size());
    for (std::size_t i = 0; i < mesh->vertices.size(); ++i)
    {
        const auto p = mesh->vertices[i].position;
        welded.push_back(positions.try_emplace({p.x, p.y, p.z}, i).first->second);
    }
    struct EdgeUse { unsigned count = 0; std::size_t opposite = 0; };
    std::map<std::pair<std::size_t, std::size_t>, EdgeUse> edges;
    for (std::size_t i = 0; i + 2 < mesh->indices.size(); i += 3)
    {
        const std::array triangle{welded[mesh->indices[i]], welded[mesh->indices[i + 1]], welded[mesh->indices[i + 2]]};
        if (triangle[0] == triangle[1] || triangle[1] == triangle[2] || triangle[2] == triangle[0]) continue;
        for (int j = 0; j < 3; ++j)
        {
            auto& edge = edges[std::minmax(triangle[j], triangle[(j + 1) % 3])];
            ++edge.count;
            edge.opposite = triangle[(j + 2) % 3];
        }
    }
    for (const auto& [edge, use] : edges)
    {
        if (use.count != 1) continue;
        const auto a = mesh->vertices[edge.first].position, b = mesh->vertices[edge.second].position;
        const auto c = mesh->vertices[use.opposite].position;
        outline.push_back({a, b, Math::Normalize(Math::Cross(b - a, c - a))});
    }
    return outline;
}
