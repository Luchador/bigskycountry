#include "spatial_index.hpp"
#include <algorithm>
#include <cmath>

Bounds3 WorldBounds(const Bounds3& local, const MeshTransform& transform)
{
    Bounds3 bounds;
    if (local.Empty()) return bounds;
    for (int i = 0; i < 8; ++i)
        bounds.Include(transform.Point({(i & 1) ? local.max.x : local.min.x,
                                        (i & 2) ? local.max.y : local.min.y,
                                        (i & 4) ? local.max.z : local.min.z}));
    return bounds;
}

double BoundsDistanceSquared(const Bounds3& bounds, Vector3 point)
{
    if (bounds.Empty()) return 0; // Unknown bounds must never hide geometry.
    const Vector3 offset{std::max({bounds.min.x - point.x, 0.0, point.x - bounds.max.x}),
                         std::max({bounds.min.y - point.y, 0.0, point.y - bounds.max.y}),
                         std::max({bounds.min.z - point.z, 0.0, point.z - bounds.max.z})};
    return Math::Dot(offset, offset);
}

bool SpatialIndex::CellRange(const Bounds3& bounds, Cell& low, Cell& high)
{
    if (bounds.Empty()) return false;
    const auto cell = [](double value, std::int64_t& out) {
        const double n = std::floor(value / cellMeters);
        // Leave room for inclusive iteration and range subtraction, even for
        // malformed imported bounds. Such records use the conservative bucket.
        constexpr double limit = 1e12;
        if (!std::isfinite(n) || std::abs(n) > limit) return false;
        out = static_cast<std::int64_t>(n);
        return true;
    };
    return cell(bounds.min.x, low.x) && cell(bounds.min.z, low.z) &&
           cell(bounds.max.x, high.x) && cell(bounds.max.z, high.z) &&
           low.x <= high.x && low.z <= high.z;
}

void SpatialIndex::BeginSync()
{
    ++sync;
    stats.updated = 0;
}

void SpatialIndex::Unlink(SceneKey key, const Entry& entry)
{
    for (const auto cell : entry.cells)
    {
        auto it = cells.find(cell);
        it->second.erase(key);
        if (it->second.empty()) cells.erase(it);
    }
    oversized.erase(key);
}

void SpatialIndex::Upsert(SceneKey key, const Bounds3& bounds)
{
    auto [it, added] = entries.try_emplace(key);
    auto& entry = it->second;
    entry.touched = sync;
    if (!added && entry.bounds == bounds) return;
    if (!added) Unlink(key, entry);
    ++stats.updated;
    entry.bounds = bounds;
    entry.cells.clear();
    Cell low{}, high{};
    // Until long roads are chunked, avoid duplicating a continental AABB into
    // millions of cells. Oversized objects are tested once per query instead.
    if (!CellRange(bounds, low, high) ||
        double(high.x - low.x + 1) * double(high.z - low.z + 1) > 256)
    {
        oversized.insert(key);
        return;
    }
    for (auto z = low.z; z <= high.z; ++z)
        for (auto x = low.x; x <= high.x; ++x)
        {
            const Cell cell{x, z};
            cells[cell].insert(key);
            entry.cells.push_back(cell);
        }
}

void SpatialIndex::EndSync()
{
    std::erase_if(entries, [&](const auto& item) {
        if (item.second.touched == sync) return false;
        Unlink(item.first, item.second);
        ++stats.updated;
        return true;
    });
    stats.entries = entries.size();
    stats.cells = cells.size();
    stats.oversized = oversized.size();
}

std::span<const SceneKey> SpatialIndex::Query(Vector3 eye, double radius)
{
    ++query;
    result.clear();
    stats.visitedCells = stats.candidates = stats.inRange = 0;
    const bool limited = std::isfinite(radius) && radius > 0;
    const auto visit = [&](SceneKey key) {
        auto& entry = entries.at(key);
        if (entry.queried == query) return;
        entry.queried = query;
        ++stats.candidates;
        if (!limited || BoundsDistanceSquared(entry.bounds, eye) <= radius * radius)
            result.push_back(key);
    };
    Cell low{}, high{};
    const Vector3 extent{radius, radius, radius};
    if (!limited || !CellRange({eye - extent, eye + extent}, low, high))
    {
        for (const auto& [key, entry] : entries) visit(key);
    }
    else
    {
        const auto visitCell = [&](Cell cell, const Keys& keys) {
            // Distance to the infinite vertical cell prism; altitude is handled
            // by the per-entry 3D test. Include cells behind the camera too.
            const Bounds3 box{{double(cell.x) * cellMeters, eye.y, double(cell.z) * cellMeters},
                              {double(cell.x + 1) * cellMeters, eye.y, double(cell.z + 1) * cellMeters}};
            if (BoundsDistanceSquared(box, eye) > radius * radius) return;
            ++stats.visitedCells;
            for (const auto key : keys) visit(key);
        };
        if (double(high.x - low.x + 1) * double(high.z - low.z + 1) > double(cells.size()))
        {
            // A zoomed-out sparse world is cheaper to query by occupied cells.
            for (const auto& [cell, keys] : cells)
                if (cell.x >= low.x && cell.x <= high.x && cell.z >= low.z && cell.z <= high.z)
                    visitCell(cell, keys);
        }
        else
        {
            for (auto z = low.z; z <= high.z; ++z)
                for (auto x = low.x; x <= high.x; ++x)
                    if (auto it = cells.find({x, z}); it != cells.end()) visitCell(it->first, it->second);
        }
        for (const auto key : oversized) visit(key);
    }
    // Preserve original submission order for coplanar paint and transparent
    // surfaces. Grid hash iteration must not make the image flicker.
    std::sort(result.begin(), result.end());
    stats.inRange = result.size();
    return result;
}
