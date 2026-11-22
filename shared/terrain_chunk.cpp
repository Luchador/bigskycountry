#include "terrain_chunk.hpp"
#include "editor_geometry_cache.hpp"

#include <algorithm>
#include <map>

bool TerrainChunk::SetLake(const LakeDefinition& definition)
{
    if (river) return SDL_SetError("Cannot turn a river into a lake");
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
    auto geometry = geometryCache ? geometryCache->Lake(definition) : LakeGeometry::Build(definition);
    if (!geometry) return false;
    lake = definition;
    SetMesh(std::move(geometry->bed));
    water = {std::move(geometry->water), Instance().transform};
    shoreline = std::move(geometry->shoreline);
    WaterChanged();
    return true;
}

bool TerrainChunk::BeginLakeEdit()
{
    if (!lake) return SDL_SetError("Select a lake to edit its shoreline");
    if (lakeDraft) return true;
    lakeDraft = lake;
    // The committed rim may contain extra junction vertices. Draft picking uses
    // the original spline's segment/t values, just as river editing does.
    draftShoreline = *LakeGeometry::Sample(*lake);
    return true;
}

bool TerrainChunk::FinishLakeEdit()
{
    if (!lakeDraft) return true;
    if (lakeDraft != lake)
    {
        auto geometry = geometryCache ? geometryCache->Lake(*lakeDraft) : LakeGeometry::Build(*lakeDraft);
        if (!geometry) return false;
        lake = std::move(lakeDraft);
        SetMesh(std::move(geometry->bed));
        water = {std::move(geometry->water), Instance().transform};
        shoreline = std::move(geometry->shoreline);
        WaterChanged();
    }
    CancelLakeEdit();
    return true;
}

void TerrainChunk::CancelLakeEdit()
{
    lakeDraft.reset();
    draftShoreline.clear();
}

bool TerrainChunk::SetWater(const WaterDefinition& definition)
{
    return std::visit([&](const auto& source) {
        using T = std::decay_t<decltype(source)>;
        if constexpr (std::is_same_v<T, LakeDefinition>) return SetLake(source);
        else return SetRiver(source);
    }, definition);
}

bool TerrainChunk::SetRiver(const RiverDefinition& definition)
{
    if (lake) return SDL_SetError("Cannot turn a lake into a river");
    if (riverDraft)
    {
        if (*riverDraft == definition) return true;
        auto samples = RiverGeometry::Sample(definition);
        if (!samples) return false;
        riverDraft = definition;
        draftShoreline = std::move(*samples);
        return true;
    }
    if (river == definition) return true;
    auto geometry = geometryCache ? geometryCache->River(definition) : RiverGeometry::Build(definition);
    if (!geometry) return false;
    river = definition;
    SetMesh(std::move(geometry->bed));
    water = {std::move(geometry->water), Instance().transform};
    shoreline = std::move(geometry->shoreline);
    WaterChanged();
    return true;
}

bool TerrainChunk::BeginWaterEdit()
{
    if (!river) return BeginLakeEdit();
    if (riverDraft) return true;
    riverDraft = river;
    draftShoreline = *RiverGeometry::Sample(*river);
    return true;
}

bool TerrainChunk::FinishWaterEdit()
{
    if (!riverDraft) return FinishLakeEdit();
    if (riverDraft != river)
    {
        auto geometry = geometryCache ? geometryCache->River(*riverDraft) : RiverGeometry::Build(*riverDraft);
        if (!geometry) return false;
        river = std::move(riverDraft);
        SetMesh(std::move(geometry->bed));
        water = {std::move(geometry->water), Instance().transform};
        shoreline = std::move(geometry->shoreline);
        WaterChanged();
    }
    CancelWaterEdit();
    return true;
}

void TerrainChunk::CancelWaterEdit()
{
    riverDraft.reset();
    CancelLakeEdit();
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

void TerrainChunk::ApplyWaterGeometry(const WaterDefinition& definition, LakeGeometry geometry, bool joined)
{
    if (const auto* source = std::get_if<RiverDefinition>(&definition)) river = *source;
    else lake = std::get<LakeDefinition>(definition);
    SetMesh(std::move(geometry.bed));
    water = {std::move(geometry.water), Instance().transform};
    shoreline = std::move(geometry.shoreline);
    joinedWater = joined;
    WaterChanged();
}
