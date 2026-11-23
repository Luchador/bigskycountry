#include "pawn_index.hpp"
#include <algorithm>
#include <cmath>

bool PawnIndex::Overlap(const Bounds3& a, const Bounds3& b)
{
    return !a.Empty() && !b.Empty() && a.min.x <= b.max.x && a.max.x >= b.min.x &&
        a.min.y <= b.max.y && a.max.y >= b.min.y && a.min.z <= b.max.z && a.max.z >= b.min.z;
}
Bounds3 PawnIndex::Around(Vector3 center, double radius)
{
    return {center - Vector3{radius,radius,radius}, center + Vector3{radius,radius,radius}};
}
std::vector<std::uint64_t> PawnIndex::Keys(const Bounds3& bounds)
{
    const auto coordinate = [](double x) { return static_cast<std::int32_t>(std::clamp(std::floor(x / 32.0), -1e9, 1e9)); };
    const auto x0 = coordinate(bounds.min.x), x1 = coordinate(bounds.max.x);
    const auto z0 = coordinate(bounds.min.z), z1 = coordinate(bounds.max.z);
    // An exceptional large body/query takes a bounded fallback, never billions of cells.
    if (std::int64_t(x1 - x0 + 1) * (z1 - z0 + 1) > 256) return {};
    std::vector<std::uint64_t> result;
    for (auto x = x0; x <= x1; ++x)
        for (auto z = z0; z <= z1; ++z)
            result.push_back((std::uint64_t(static_cast<std::uint32_t>(x)) << 32) | static_cast<std::uint32_t>(z));
    return result;
}
void PawnIndex::Unlink(PawnHandle handle, const Entry& entry)
{
    if (entry.cells.empty()) std::erase(oversized, handle);
    for (auto key : entry.cells)
        if (auto it = cells.find(key); it != cells.end())
        {
            std::erase(it->second, handle);
            if (it->second.empty()) cells.erase(it);
        }
}
void PawnIndex::Remove(PawnHandle handle)
{
    if (auto it = entries.find(handle); it != entries.end()) { Unlink(handle, it->second); entries.erase(it); }
}
void PawnIndex::Update(Pawn& pawn)
{
    Bounds3 current = pawn.CollisionBounds();
    if (!pawn.hasCollision || current.Empty() || !std::isfinite(Math::Length(current.min)) || !std::isfinite(Math::Length(current.max)))
    { Remove(pawn.Handle()); return; }
    // Margin covers the bounded movement during a physics substep and small contact corrections.
    current.min = current.min - Vector3{0.3,0.3,0.3}; current.max = current.max + Vector3{0.3,0.3,0.3};
    Bounds3 predicted = current;
    Vector3 travel = pawn.Velocity() * predictionSeconds;
    if (!std::isfinite(Math::Length(travel))) travel = {};
    predicted.Include(current.min + travel); predicted.Include(current.max + travel);
    predicted.min = predicted.min - Vector3{2.5,0,2.5}; predicted.max = predicted.max + Vector3{2.5,0,2.5};
    auto keys = Keys(predicted);
    auto [it, inserted] = entries.try_emplace(pawn.Handle());
    auto& entry = it->second;
    if (inserted || entry.cells != keys)
    {
        if (!inserted) Unlink(pawn.Handle(), entry);
        for (auto key : keys) cells[key].push_back(pawn.Handle());
        if (keys.empty()) oversized.push_back(pawn.Handle());
        entry.cells = std::move(keys);
    }
    entry.pawn = &pawn; entry.current = current; entry.predicted = predicted;
}
std::vector<Pawn*> PawnIndex::Query(const Bounds3& bounds, bool predicted) const
{
    ++statistics.queries;
    if (bounds.Empty() || !std::isfinite(Math::Length(bounds.min)) || !std::isfinite(Math::Length(bounds.max))) return {};
    std::vector<PawnHandle> handles;
    const auto keys = Keys(bounds);
    if (keys.empty())
        for (const auto& [handle, entry] : entries) handles.push_back(handle);
    else
    {
        handles = oversized;
        for (auto key : keys)
            if (auto it = cells.find(key); it != cells.end()) handles.insert(handles.end(), it->second.begin(), it->second.end());
    }
    std::sort(handles.begin(), handles.end());
    handles.erase(std::unique(handles.begin(), handles.end()), handles.end());
    statistics.candidates += handles.size();
    std::vector<Pawn*> result;
    for (auto handle : handles)
    {
        const auto& entry = entries.at(handle);
        if (Overlap(bounds, predicted ? entry.predicted : entry.current)) result.push_back(entry.pawn);
    }
    statistics.results += result.size();
    return result;
}
