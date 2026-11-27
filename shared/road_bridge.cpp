#include "road_bridge.hpp"
#include "road.hpp"
#include "terrain_fill.hpp"
#include "terrain_chunk.hpp"
#include "world_document.hpp"
#include "world_lighting.hpp"
#include <SDL3/SDL.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <tuple>

namespace
{
    using Rim = std::vector<Vector3>;
    double ProfileRadius(const Road& road)
    {
        if (road.style.railroad)
        {
            const auto& bed = *road.style.railroad;
            const double outer = bed.ToeHalfWidth(road.MaximumLaneCount()) +
                (road.style.edge == RoadEdge::Transition ? RoadStyle::stripWidth : 0);
            // End-cap tests must reach the raised crest as well as wide/banked toes.
            return std::hypot(outer, bed.height) + 1;
        }
        return road.MaximumHalfTravelWidth() + road.MaximumHalfSeparation() +
               std::max(road.MaximumShoulderWidth(true), road.MaximumShoulderWidth(false)) + road.style.EdgeWidth() + 1;
    }
    double DeckBaseHeight(const Road& road, Vector3 position, Vector3 direction)
    {
        if (!road.style.railroad) return position.y;
        const auto& bed = *road.style.railroad;
        const auto frame = ObjectTransform{road.points.front().position, road.rotation}.ToMeshTransform();
        const auto forward = Math::Normalize(direction);
        auto side = Math::Cross(forward, frame.y);
        if (Math::Length(side) < 1e-8) side = frame.z - forward * Math::Dot(frame.z, forward);
        side = Math::Normalize(side);
        const auto up = Math::Cross(side, forward);
        const double width = bed.ToeHalfWidth(road.MaximumLaneCount()) +
            (road.style.edge == RoadEdge::Transition ? RoadStyle::stripWidth : 0);
        // A banked railway's lower toe can be below its center spline. Use a
        // conservative envelope of the full bed, never the higher rail heads.
        return position.y - width * std::abs(side.y) + std::min(0.0, bed.height * up.y);
    }
    double Cross(Vector3 a, Vector3 b)
    {
        return a.x * b.z - a.z * b.x;
    }
    bool Inside(Vector3 p, const Rim& rim)
    {
        bool inside = false;
        for (std::size_t i = 0, j = rim.size() - 1; i < rim.size(); j = i++)
        {
            const auto a = rim[i], b = rim[j];
            if ((a.z > p.z) != (b.z > p.z) && p.x < (b.x - a.x) * (p.z - a.z) / (b.z - a.z) + a.x) inside = !inside;
        }
        return inside;
    }
    std::optional<std::pair<double, double>> Crossing(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        const auto u = b - a, v = d - c;
        const double denominator = Cross(u, v);
        if (std::abs(denominator) < 1e-10) return {};
        const double t = Cross(c - a, v) / denominator, s = Cross(c - a, u) / denominator;
        if (t < -1e-9 || t > 1 + 1e-9 || s < -1e-9 || s > 1 + 1e-9) return {};
        return std::pair{std::clamp(t, 0.0, 1.0), std::clamp(s, 0.0, 1.0)};
    }
    std::optional<std::size_t> PathEnd(std::size_t edge, std::size_t count, double t)
    {
        if (edge == 1 && t <= 1e-9) return 0;
        if (edge + 1 == count && t >= 1 - 1e-9) return 1;
        return {};
    }
    bool Overlaps(const Rim& a, const Rim& b)
    {
        if (a.size() < 3 || b.size() < 3) return false;
        Bounds3 boundsA, boundsB;
        for (auto p : a) boundsA.Include(p);
        for (auto p : b) boundsB.Include(p);
        if (boundsA.max.x < boundsB.min.x || boundsB.max.x < boundsA.min.x ||
            boundsA.max.z < boundsB.min.z || boundsB.max.z < boundsA.min.z) return false;
        if (Inside(a.front(), b) || Inside(b.front(), a)) return true;
        for (std::size_t i = 0; i < a.size(); ++i)
            for (std::size_t j = 0; j < b.size(); ++j)
                if (Crossing(a[i], a[(i + 1) % a.size()], b[j], b[(j + 1) % b.size()])) return true;
        return false;
    }
    std::optional<std::pair<double, double>> CoveredRange(Vector3 a, Vector3 b, Vector3 side, const Rim& rim)
    {
        const Rim strip{a - side, b - side, b + side, a + side};
        if (!Overlaps(strip, rim)) return {};
        const auto direction = b - a;
        const double length = direction.x * direction.x + direction.z * direction.z;
        if (length < 1e-16) return {};
        double first = 1, last = 0;
        const auto include = [&](Vector3 p) {
            const double t = std::clamp(((p.x - a.x) * direction.x + (p.z - a.z) * direction.z) / length, 0.0, 1.0);
            first = std::min(first, t);
            last = std::max(last, t);
        };
        for (auto p : strip) if (Inside(p, rim)) include(p);
        for (auto p : rim) if (Inside(p, strip)) include(p);
        for (std::size_t i = 0; i < strip.size(); ++i)
            for (std::size_t j = 0; j < rim.size(); ++j)
                if (auto at = Crossing(strip[i], strip[(i + 1) % strip.size()], rim[j], rim[(j + 1) % rim.size()]))
                    include(strip[i] + (strip[(i + 1) % strip.size()] - strip[i]) * at->first);
        if (first > last) return {};
        return std::pair{first, last};
    }
    Rim SpanRim(const Road& road, std::size_t first, std::size_t last)
    {
        Rim rim;
        if (road.bridgeKnots.size() != road.points.size()) return rim;
        const double start = road.bridgeKnots[first], end = road.bridgeKnots[last];
        for (int side : {0, 1})
        {
            Rim edge;
            for (const auto& p : road.bridgeRims[side])
                if (p.distance >= start - 1e-8 && p.distance <= end + 1e-8)
                    if (edge.empty() || edge.back() != p.position) edge.push_back(p.position);
            if (side == 0) rim.insert(rim.end(), edge.begin(), edge.end());
            else rim.insert(rim.end(), edge.rbegin(), edge.rend());
        }
        return rim;
    }
    Rim Shoreline(const TerrainChunk& water)
    {
        Rim rim;
        rim.reserve(water.BuiltShoreline().size());
        for (const auto& sample : water.BuiltShoreline())
            rim.push_back(water.Water().transform.Point(sample.position));
        if (rim.size() > 1 && rim.front() == rim.back()) rim.pop_back();
        return rim;
    }
    Rim ClipWaterToSpan(Rim polygon, Vector3 a, Vector3 b, Vector3 side)
    {
        // Clip each water triangle to the swept road rectangle in XZ. Keep Y
        // interpolated on its triangle so clearance follows sloping rivers.
        const Vector3 along{b.x-a.x,0,b.z-a.z};
        for (const auto [axis, low, high] : {std::tuple{along,0.0,Math::Dot(along,along)},
                                          std::tuple{side,-Math::Dot(side,side),Math::Dot(side,side)}})
            for (bool lower : {true,false})
            {
                if (polygon.empty()) return {};
                Rim clipped;
                auto previous = polygon.back();
                const auto distance = [&](Vector3 p) { const double d = Math::Dot(p-a,axis); return lower ? d-low : high-d; };
                double before = distance(previous);
                for (auto point : polygon)
                {
                    const double next = distance(point);
                    if ((before >= 0) != (next >= 0)) clipped.push_back(previous+(point-previous)*(before/(before-next)));
                    if (next >= 0) clipped.push_back(point);
                    previous = point; before = next;
                }
                polygon = std::move(clipped);
            }
        return polygon;
    }
    void Face(Road::ColoredSurface& out, Vector3 a, Vector3 b, Vector3 c, SDL_FColor color)
    {
        if (Math::Length(Math::Cross(b - a, c - a)) > 1e-10)
            out.push_back({Road::EdgeVertex{a, color}, {b, color}, {c, color}});
    }
    void Wall(Road::ColoredSurface& out, const std::array<Vector3, 4>& q, bool both = false)
    {
        constexpr SDL_FColor concrete{.52f, .52f, .49f, 1};
        Face(out, q[0], q[1], q[2], concrete);
        Face(out, q[0], q[2], q[3], concrete);
        if (both)
        {
            Face(out, q[2], q[1], q[0], concrete);
            Face(out, q[3], q[2], q[0], concrete);
        }
    }
    void SampleGroundRim(Vector3 p, std::span<const Vector3> rim, double& best, double& height)
    {
        for (std::size_t i = 0; i < rim.size(); ++i)
        {
            const auto a = rim[i], b = rim[(i + 1) % rim.size()], d = b - a;
            const double length = d.x * d.x + d.z * d.z;
            const double t =
                length > 1e-16 ? std::clamp(((p.x - a.x) * d.x + (p.z - a.z) * d.z) / length, 0.0, 1.0) : 0;
            const auto q = a + d * t;
            const double distance = std::hypot(p.x - q.x, p.z - q.z);
            if (distance < best)
            {
                best = distance;
                height = q.y;
            }
        }
    }
    double GroundHeight(Vector3 p, std::span<const Rim> sources, double fallback)
    {
        double best = std::numeric_limits<double>::infinity(), height = fallback;
        for (const auto& rim : sources)
            SampleGroundRim(p, rim, best, height);
        return height;
    }
    double GroundHeight(Vector3 p, std::span<const Vector3> rim, double fallback)
    {
        double best = std::numeric_limits<double>::infinity(), height = fallback;
        SampleGroundRim(p, rim, best, height);
        return height;
    }
    std::vector<Rim> ClearancePaths(const Road& lower)
    {
        if (lower.style.railroad)
        {
            // Use the generated rail-head sweep rather than the authored spline
            // at the embankment toe. Both edges of both heads account for banking,
            // and the center guide also detects a crossing near a track endpoint.
            constexpr double inner = RailroadSettings::gauge / 2;
            constexpr double outer = inner + RailroadSettings::headWidth;
            std::vector<Rim> paths;
            for (const auto& span : lower.trackSpans)
                for (double offset : {0.0, -inner, inner, -outer, outer})
                    paths.push_back({span[0].top + span[0].side * offset,
                                     span[1].top + span[1].side * offset});
            return paths;
        }
        std::vector<Rim> paths(1);
        for (std::size_t i = 0; i + 1 < lower.points.size(); ++i)
        {
            const auto samples = lower.SegmentSamples(i);
            paths.front().insert(paths.front().end(), samples.begin(), samples.end());
        }
        return paths;
    }
    Rim GroundRim(const Rim& top, std::span<const Rim> sources, double fallback)
    {
        Rim result;
        for (std::size_t i = 0; i < top.size(); ++i)
        {
            const auto a = top[i], b = top[(i + 1) % top.size()];
            std::map<double, double> cuts;
            const int steps = std::max(1, int(std::ceil(std::hypot(b.x - a.x, b.z - a.z) / 8)));
            for (int j = 0; j < steps; ++j)
            {
                double t = double(j) / steps;
                cuts[t] = GroundHeight(a + (b - a) * t, sources, fallback);
            }
            // Ground guides and lower-road constraints share an exact height at every crossing.
            for (const auto& rim : sources)
                for (std::size_t j = 0; j < rim.size(); ++j)
                    if (auto at = Crossing(a, b, rim[j], rim[(j + 1) % rim.size()]))
                        cuts[at->first] = std::lerp(rim[j].y, rim[(j + 1) % rim.size()].y, at->second);
            for (const auto& [t, y] : cuts)
            {
                auto p = a + (b - a) * t;
                p.y = y;
                if (result.empty() || Math::Length(p - result.back()) > 1e-8) result.push_back(p);
            }
        }
        if (result.size() > 1 && Math::Length(result.front() - result.back()) < 1e-8) result.pop_back();
        return result;
    }
} // namespace

bool BridgeSettings::Valid() const
{
    return std::isfinite(depth) && depth >= .1 && depth <= 10 && std::isfinite(groundHeight) &&
           lowerRoads.size() <= 10000 && (lowerRoads.empty() || *lowerRoads.begin() > 0) &&
           lowerWaterBodies.size() <= 10000 && (lowerWaterBodies.empty() || *lowerWaterBodies.begin() > 0);
}
bool Road::HasOverpasses() const
{
    for (std::size_t i = 0; i + 1 < points.size(); ++i)
        if (points[i].bridge) return true;
    return false;
}

std::optional<BridgeSettings> PlanOverpass(const Road& upper, const std::set<int>& segments,
                                           std::span<const Road> roads,
                                           std::span<const std::unique_ptr<TerrainChunk>> terrain)
{
    const auto fail = [](const char* message) -> std::optional<BridgeSettings> {
        SDL_SetError("%s", message);
        return {};
    };
    if (segments.empty() || *segments.begin() < 0 ||
        std::size_t(*segments.rbegin() + 1) >= upper.points.size() ||
        int(segments.size()) != *segments.rbegin() - *segments.begin() + 1)
        return fail("Select adjoining road or railroad segments for the overpass");
    if (!upper.rampLanes.empty() || !upper.crossings.empty() || upper.HasDeadEnd() || upper.HasCulDeSac())
        return fail("Split the overpass section from ramps, grade crossings, or dead ends first");
    if (upper.style.railroad)
        for (const auto& road : roads)
            for (const auto& crossing : road.crossings)
                if (crossing.railroadId == upper.id)
                    return fail("Split the railroad overpass section from at-grade crossings first");
    for (int i : segments)
        if (upper.points[i].Joined() || upper.points[i + 1].Joined())
            return fail("Place the overpass ends away from road and intersection connections");
    BridgeSettings settings;
    double ground = 0;
    int hits = 0;
    const auto capsClear = [&](const Rim& rim) {
        for (std::size_t p : {std::size_t(*segments.begin()), std::size_t(*segments.rbegin() + 1)})
        {
            const auto axis = upper.PointAxes(p)[0];
            const auto left = Math::Normalize(Vector3{-axis.z, 0, axis.x});
            const double width = ProfileRadius(upper);
            const auto a = upper.points[p].position - left * width, b = upper.points[p].position + left * width;
            if (Inside(a, rim) || Inside(b, rim) || Inside((a + b) * .5, rim)) return false;
            for (std::size_t j = 0; j < rim.size(); ++j)
                if (Crossing(a, b, rim[j], rim[(j + 1) % rim.size()])) return false;
        }
        return true;
    };
    for (const auto& lower : roads)
    {
        if (lower.id == upper.id || lower.id <= 0 || lower.HasOverpasses() || lower.outline.size() < 3) continue;
        const auto lowerPaths = ClearancePaths(lower);
        bool crosses = false;
        for (int segment : segments)
        {
            const auto path = upper.SegmentSamples(segment);
            for (const auto& lowerPath : lowerPaths)
                for (std::size_t i = 1; i < path.size(); ++i)
                    for (std::size_t j = 1; j < lowerPath.size(); ++j)
                        if (auto at = Crossing(path[i - 1], path[i], lowerPath[j - 1], lowerPath[j]))
                        {
                            // Split roads can share an endpoint without an explicit
                            // connector. That contact is an approach to the bridge,
                            // not a lower crossing. Only ignore the endpoint hit:
                            // the same road may curve back under the span elsewhere.
                            const auto upperEnd = PathEnd(i, path.size(), at->first);
                            const auto lowerEnd = PathEnd(j, lowerPath.size(), at->second);
                            if (upperEnd && lowerEnd &&
                                ((segment == 0 && *upperEnd == 0) ||
                                 (std::size_t(segment + 2) == upper.points.size() && *upperEnd == 1)) &&
                                upper.style.railroad.has_value() == lower.style.railroad.has_value() &&
                                Math::Length(upper.points[segment + *upperEnd].position -
                                    lower.points[*lowerEnd ? lower.points.size() - 1 : 0].position) <= 1e-6)
                                continue;
                            const auto position = path[i - 1] + (path[i] - path[i - 1]) * at->first;
                            const double bottom = std::lerp(lowerPath[j - 1].y, lowerPath[j].y, at->second);
                            if (DeckBaseHeight(upper, position, path[i] - path[i - 1]) - bottom < settings.depth + 1)
                                return fail(lower.style.railroad ? "Raise the upper road or railroad before creating an overpass "
                                                                   "(deck plus at least 1 m above the rail heads)"
                                                                 : "Raise the upper road or railroad before creating an overpass "
                                                                   "(deck plus at least 1 m of clear space)");
                            // Clearance is above the rails; terrain must still meet
                            // the embankment's unchanged toe/transition-strip rim.
                            ground += lower.style.railroad ? GroundHeight(position, lower.outline, bottom) : bottom;
                            ++hits;
                            crosses = true;
                        }
        }
        if (!crosses) continue;
        if (!capsClear(lower.outline))
            return fail("Extend the overpass span so its abutments clear every lower road and railroad embankment");
        settings.lowerRoads.insert(lower.id);
    }
    std::vector<std::span<const Vector3>> upperPaths;
    for (int segment : segments) upperPaths.push_back(upper.SegmentSamples(segment));
    for (const auto& water : terrain)
    {
        if (!water->HasWater() || !water->persistentId || !water->Water().mesh) continue;
        const auto rim = Shoreline(*water);
        if (rim.size() < 3) continue;
        bool crosses = false;
        double height = 0;
        std::size_t samples = 0;
        for (const auto& path : upperPaths)
        {
            for (std::size_t i = 1; i < path.size(); ++i)
            {
                const auto a = path[i - 1], b = path[i], direction = b - a;
                const auto side = Math::Normalize(Vector3{-direction.z, 0, direction.x}) * ProfileRadius(upper);
                // Check the full swept width, but only the part over water. A
                // long straight segment can have lower ends over dry ground.
                if (CoveredRange(a, b, side, rim))
                {
                    const auto& mesh = *water->Water().mesh;
                    for (std::size_t j = 0; j < mesh.indices.size(); j += 3)
                    {
                        Rim triangle;
                        for (int k = 0; k < 3; ++k) triangle.push_back(water->Water().transform.Point(mesh.vertices[mesh.indices[j+k]].position));
                        for (auto p : ClipWaterToSpan(std::move(triangle),a,b,side))
                        {
                            const double t = std::clamp(((p.x-a.x)*direction.x+(p.z-a.z)*direction.z) /
                                                        (direction.x*direction.x+direction.z*direction.z),0.0,1.0);
                            if (DeckBaseHeight(upper, a + direction * t, direction) - p.y < settings.depth + 1)
                                return fail("Raise the upper road or railroad before creating an overpass "
                                            "(deck plus at least 1 m above the water surface)");
                            height += p.y; ++samples;
                            crosses = true;
                        }
                    }
                }
            }
        }
        if (!crosses) continue;
        if (water->EditingWater()) return fail("Finish editing the river or lake before creating its overpass");
        if (!capsClear(rim)) return fail("Extend the overpass span so both abutments are beyond the river or lake banks");
        settings.lowerWaterBodies.insert(water->persistentId);
        ground += height / samples;
        ++hits;
    }
    if (!hits) return fail("The selected span must cross above a road, railroad, river or lake");
    settings.groundHeight = ground / hits;
    return settings;
}

void Road::BuildOverpasses()
{
    if (!HasOverpasses() || bridgeKnots.size() != points.size() || bridgeRims[0].empty() || bridgeRims[1].empty())
        return;
    const auto& topEdges = edgeTriangles;
    for (std::size_t first = 0; first + 1 < points.size();)
    {
        std::size_t last = first + 1;
        while (last + 1 < points.size() && points[last].bridge == points[first].bridge)
            ++last;
        auto rim = SpanRim(*this, first, last);
        if (rim.size() < 3)
        {
            first = last;
            continue;
        }
        auto& contact = groundFootprints.emplace_back();
        if (!points[first].bridge)
        {
            contact.outline = std::move(rim);
            first = last;
            continue;
        }
        const auto& bridge = *points[first].bridge;
        std::vector<Rim> sources, shorelines;
        for (int id : bridge.lowerRoads)
            if (auto it = bridgeGroundSources.find(id); it != bridgeGroundSources.end())
                sources.push_back(it->second);
        const auto roadSources = sources.size();
        for (auto id : bridge.lowerWaterBodies)
            if (auto it = bridgeWaterSources.find(id); it != bridgeWaterSources.end())
            {
                sources.push_back(it->second);
                shorelines.push_back(it->second);
            }
        contact.outline = GroundRim(rim, sources, bridge.groundHeight);
        const Vector3 down{0, -bridge.depth, 0};
        const auto bottom = [&](const auto& triangles) {
            for (const auto& t : triangles)
            {
                const auto a = t[0].position, b = t[1].position, c = t[2].position;
                if (Math::Cross(b - a, c - a).y >= -1e-10 && Inside((a + b + c) * (1.0 / 3), rim))
                    Face(bridgeTriangles, c + down, b + down, a + down, {.48f, .48f, .46f, 1});
            }
        };
        bottom(pavementTriangles);
        bottom(topEdges);
        for (const auto& [material, strip] : transitionStrips)
            bottom(strip);
        const std::array<std::size_t, 2> ends{first, last};
        const double capRadius = ProfileRadius(*this);
        const auto onCap = [&](Vector3 p, std::size_t knot) {
            const auto delta = p - points[knot].position;
            return Math::Length(delta) <= capRadius && std::abs(Math::Dot(delta, Tangent(knot))) < 1e-6;
        };
        for (std::size_t i = 0; i < rim.size(); ++i)
        {
            const auto a = rim[i], b = rim[(i + 1) % rim.size()];
            if ((onCap(a, first) && onCap(b, first)) || (onCap(a, last) && onCap(b, last))) continue;
            Wall(bridgeTriangles, {a, b, b + down, a + down}, true);
        }
        for (int end : {0, 1})
        {
            const bool continuation =
                end ? (last + 1 < points.size() ? points[last].bridge.has_value() : bridgeNeighbors[1])
                    : (first ? points[first - 1].bridge.has_value() : bridgeNeighbors[0]);
            // Close against actual pavement/curb cross sections, rather than a line
            // joining the curb tops (which would put a concrete lip across the lanes).
            std::vector<std::array<Vector3, 2>> edges;
            const auto collect = [&](const auto& triangles) {
                for (const auto& t : triangles)
                    for (int i = 0; i < 3; ++i)
                    {
                        const auto a = t[i].position, b = t[(i + 1) % 3].position;
                        if (!onCap(a, ends[end]) || !onCap(b, ends[end]) || std::hypot(b.x - a.x, b.z - a.z) < 1e-8)
                            continue;
                        if (std::ranges::none_of(edges, [&](const auto& e) {
                                return (e[0] == a && e[1] == b) || (e[0] == b && e[1] == a);
                            }))
                            edges.push_back({a, b});
                    }
            };
            collect(pavementTriangles);
            collect(topEdges);
            for (const auto& [material, strip] : transitionStrips)
                collect(strip);
            for (const auto& edge : edges)
            {
                const auto a = edge[0], b = edge[1], delta = b - a;
                if (continuation)
                {
                    Wall(bridgeTriangles, {a, b, b + down, a + down}, true);
                    continue;
                }
                std::set<double> cuts{0, 1};
                const double length = delta.x * delta.x + delta.z * delta.z;
                for (auto p : contact.outline)
                {
                    const double t = ((p.x - a.x) * delta.x + (p.z - a.z) * delta.z) / length;
                    const auto q = a + delta * t;
                    if (t > 0 && t < 1 && std::hypot(q.x - p.x, q.z - p.z) < 1e-6) cuts.insert(t);
                }
                for (auto it = std::next(cuts.begin()); it != cuts.end(); ++it)
                {
                    const auto topA = a + delta * *std::prev(it), topB = a + delta * *it;
                    auto lowA = topA, lowB = topB;
                    lowA.y = GroundHeight(topA, contact.outline, bridge.groundHeight);
                    lowB.y = GroundHeight(topB, contact.outline, bridge.groundHeight);
                    contact.walls.push_back({topA, topB, lowB, lowA});
                    Wall(bridgeTriangles, contact.walls.back(), true);
                }
            }
        }
        std::vector<RoadGeometry> lowerGeometry;
        for (std::size_t i = 0; i < roadSources; ++i)
        {
            lowerGeometry.emplace_back().outline = sources[i];
        }
        std::vector<const RoadGeometry*> footprints;
        for (const auto& g : lowerGeometry)
            footprints.push_back(&g);
        std::vector<std::array<Vector3, 2>> seams;
        for (std::size_t i = 0; i < contact.outline.size(); ++i)
            seams.push_back({contact.outline[i], contact.outline[(i + 1) % contact.outline.size()]});
        // This ground patch has one owner (the bridge). Adjacent fills exclude it,
        // so filling neighboring blocks cannot duplicate the terrain under the deck.
        // The river/lake already owns its basin. Land ends exactly at the banks,
        // leaving the channel open rather than laying a ground sheet over it.
        if (auto mesh = BuildTerrainFill(footprints, seams, shorelines))
            for (std::size_t i = 0; i < mesh->indices.size(); i += 3)
            {
                const auto a = mesh->vertices[mesh->indices[i]].position;
                const auto b = mesh->vertices[mesh->indices[i + 1]].position;
                const auto c = mesh->vertices[mesh->indices[i + 2]].position;
                if (Inside((a + b + c) * (1.0 / 3), contact.outline))
                    Face(bridgeTriangles, a, b, c,
                         {Srgb::ToDisplay(TerrainChunk::groundColor.r), Srgb::ToDisplay(TerrainChunk::groundColor.g),
                          Srgb::ToDisplay(TerrainChunk::groundColor.b), 1});
            }
        first = last;
    }
}

void WorldDocument::RefreshOverpasses()
{
    if (bridgeRevision == RoadGeometry::latestRevision && bridgeRoadCount == roads.size() &&
        bridgeWaterRevision == TerrainChunk::LatestWaterRevision() && bridgeTerrainCount == terrain.size()) return;
    std::map<int, const Road*> byId;
    std::map<std::uint64_t, const TerrainChunk*> waterById;
    std::vector<const Road*> bridges;
    for (const auto& road : roads)
    {
        byId.emplace(road.id, &road);
        if (road.HasOverpasses()) bridges.push_back(&road);
    }
    for (const auto& water : terrain)
        if (water->HasWater() && water->Water().mesh) waterById.emplace(water->persistentId, water.get());
    for (auto& road : roads)
    {
        std::map<int, std::uint64_t> revisions;
        std::map<std::uint64_t, std::uint64_t> waterRevisions;
        for (const auto& point : road.points)
            if (point.bridge)
            {
                for (int id : point.bridge->lowerRoads)
                    if (auto it = byId.find(id);
                        it != byId.end() && it->second != &road && !it->second->HasOverpasses())
                        revisions.try_emplace(id, it->second->revision);
                for (auto id : point.bridge->lowerWaterBodies)
                    if (auto it = waterById.find(id); it != waterById.end())
                        waterRevisions.try_emplace(id, it->second->WaterRevision());
            }
        // A split is still one span while the two exposed ends meet. Re-evaluate
        // this after moves/deletion so an abandoned continuation cannot omit an abutment.
        std::array<bool, 2> continues{};
        if (road.HasOverpasses())
            for (int end : {0, 1})
            {
                const std::size_t segment = end ? road.points.size() - 2 : 0;
                if (!road.points[segment].bridge) continue;
                const auto p = end ? road.points.back().position : road.points.front().position;
                for (const Road* other : bridges)
                {
                    if (other == &road) continue;
                    for (int neighbor : {0, 1})
                    {
                        const auto index = neighbor ? other->points.size() - 2 : 0;
                        const auto q = neighbor ? other->points.back().position : other->points.front().position;
                        if (other->points[index].bridge && Math::Length(p - q) < 1e-6) continues[end] = true;
                    }
                }
            }
        if (revisions != road.bridgeSourceRevisions || waterRevisions != road.bridgeWaterRevisions ||
            continues != road.bridgeNeighbors)
        {
            // Unrelated edits do not copy large lower-road outlines or retriangulate ground.
            std::map<int, Rim> sources;
            for (const auto& [id, revision] : revisions)
                sources.emplace(id, byId.at(id)->outline);
            std::map<std::uint64_t, Rim> waterSources;
            for (const auto& [id, revision] : waterRevisions)
                waterSources.emplace(id, Shoreline(*waterById.at(id)));
            road.bridgeSourceRevisions = std::move(revisions);
            road.bridgeWaterRevisions = std::move(waterRevisions);
            if (sources != road.bridgeGroundSources || waterSources != road.bridgeWaterSources ||
                continues != road.bridgeNeighbors)
            {
                road.bridgeGroundSources = std::move(sources);
                road.bridgeWaterSources = std::move(waterSources);
                road.bridgeNeighbors = continues;
                road.Rebuild();
            }
        }
    }
    bridgeRevision = RoadGeometry::latestRevision;
    bridgeRoadCount = roads.size();
    bridgeWaterRevision = TerrainChunk::LatestWaterRevision();
    bridgeTerrainCount = terrain.size();
}
