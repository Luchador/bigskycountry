#include "railroad_crossing.hpp"
#include "road.hpp"
#include "world_document.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    using V = RoadGeometry::TransitionVertex; // Common clipping vertex; preserves UVs and vertex masks.
    using Polygon = std::vector<V>;
    constexpr double epsilon = 1e-8;
    double Cross(Vector3 a, Vector3 b)
    {
        return a.x * b.z - a.z * b.x;
    }
    double LengthXZ(Vector3 a)
    {
        return std::hypot(a.x, a.z);
    }
    double Ease(double t)
    {
        t = std::clamp(t, 0.0, 1.0);
        return t * t * t * (t * (6 * t - 15) + 10);
    }
    V Mix(const V& a, const V& b, double t)
    {
        const auto f = [t](float x, float y) { return static_cast<float>(std::lerp(x, y, t)); };
        return {a.position + (b.position - a.position) * t,
                {f(a.color.r, b.color.r), f(a.color.g, b.color.g), f(a.color.b, b.color.b), f(a.color.a, b.color.a)},
                {f(a.uv.x, b.uv.x), f(a.uv.y, b.uv.y)}};
    }
    // Keep the left half-plane. Shared cut vertices are interpolated from source
    // attributes, so an asphalt UV or transition mask never restarts at a crossing.
    Polygon Clip(const Polygon& poly, Vector3 a, Vector3 b)
    {
        Polygon out;
        if (poly.empty())
            return out;
        V previous = poly.back();
        double dp = Cross(b - a, previous.position - a);
        for (const V& current : poly)
        {
            const double dc = Cross(b - a, current.position - a);
            if ((dp < 0) != (dc < 0))
                out.push_back(Mix(previous, current, dp / (dp - dc)));
            if (dc >= 0)
                out.push_back(current);
            previous = current;
            dp = dc;
        }
        return out;
    }
    double Area(const Polygon& p)
    {
        double area = 0;
        for (size_t i = 1; i + 1 < p.size(); ++i)
            area += Math::Length(Math::Cross(p[i].position - p[0].position, p[i + 1].position - p[0].position));
        return std::abs(area);
    }
    template <class Emit> void Triangulate(const Polygon& p, Emit emit)
    {
        for (size_t i = 1; i + 1 < p.size(); ++i)
            if (Math::Length(Math::Cross(p[i].position - p[0].position, p[i + 1].position - p[0].position)) > 1e-10)
                emit(std::array<V, 3>{p[0], p[i], p[i + 1]});
    }
    bool Overlap(const Polygon& a, const Polygon& b)
    {
        const auto bounds = [](const Polygon& p)
        {
            Bounds3 box;
            for (const V& v : p)
                box.Include(v.position);
            return box;
        };
        const auto x = bounds(a), y = bounds(b);
        return x.min.x <= y.max.x && x.max.x >= y.min.x && x.min.z <= y.max.z && x.max.z >= y.min.z;
    }
    // Clip out a convex footprint, returning the interior and disjoint exterior pieces.
    Polygon Partition(Polygon p, Polygon footprint, std::vector<Polygon>& outside)
    {
        if (!Overlap(p, footprint))
        {
            outside.push_back(std::move(p));
            return {};
        }
        if (Cross(footprint[1].position - footprint[0].position, footprint[2].position - footprint[0].position) < 0)
            std::reverse(footprint.begin(), footprint.end());
        for (size_t i = 0; i < footprint.size() && !p.empty(); ++i)
        {
            const auto a = footprint[i].position, b = footprint[(i + 1) % footprint.size()].position;
            auto part = Clip(p, b, a);
            if (Area(part) > 1e-10)
                outside.push_back(std::move(part));
            p = Clip(p, a, b);
        }
        return p;
    }
    V Vertex(Vector3 p)
    {
        return {p, {1, 1, 1, 1}, {}};
    }
    Polygon Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d)
    {
        return {Vertex(a), Vertex(b), Vertex(c), Vertex(d)};
    }
    struct Band
    {
        TrackSection a, b;
        double low, high, weightLow, weightHigh;
        bool groove;
        Vector3 Point(const TrackSection& row, double v) const { return row.top + row.side * v; }
        Polygon Footprint() const { return Quad(Point(a, low), Point(a, high), Point(b, high), Point(b, low)); }
        double Lateral(Vector3 p) const
        {
            // Parameter in the straight cross-track cut lines. It remains exact
            // on both rail faces even when the two sweep sections turn or bank.
            const auto l0 = Point(a, low), l1 = Point(b, low), h0 = Point(a, high), h1 = Point(b, high);
            const double dl = Cross(l1 - l0, p - l0), dh = Cross(h1 - h0, p - h0);
            const double denominator = dl - dh;
            return std::abs(denominator) > 1e-12 ? std::clamp(dl / denominator, 0.0, 1.0) : 0;
        }
    };
    std::vector<double> Bands(const CrossingProfile& profile)
    {
        std::vector<double> values{-profile.levelWidth, profile.levelWidth};
        for (int s : {-1, 1})
        {
            const int count = std::max(1, static_cast<int>(std::ceil(profile.approach)));
            for (int i = 1; i <= count; ++i)
                values.push_back(s * (profile.levelWidth + profile.approach * i / count));
        }
        std::sort(values.begin(), values.end());
        return values;
    }
    bool Groove(double lateral)
    {
        return std::abs(lateral) > RailroadSettings::gauge / 2 - RailroadSettings::flangeClearance &&
               std::abs(lateral) <
                   RailroadSettings::gauge / 2 + RailroadSettings::headWidth + RailroadSettings::outerClearance;
    }
    std::vector<Band> MakeBands(const CrossingProfile& profile)
    {
        std::vector<Band> bands;
        const auto offsets = Bands(profile);
        for (size_t i = 1; i < profile.rows.size(); ++i)
            for (size_t j = 1; j < offsets.size(); ++j)
            {
                const auto weight = [&](double v)
                { return 1 - Ease((std::abs(v) - profile.levelWidth) / profile.approach); };
                bands.push_back({profile.rows[i - 1], profile.rows[i], offsets[j - 1], offsets[j],
                                 weight(offsets[j - 1]), weight(offsets[j]), false});
            }
        return bands;
    }
    double PlaneY(Vector3 p, Vector3 a, Vector3 b, Vector3 c)
    {
        const auto n = Math::Cross(b - a, c - a);
        return std::abs(n.y) > 1e-10 ? a.y - (n.x * (p.x - a.x) + n.z * (p.z - a.z)) / n.y : a.y;
    }
    template <class Emit> void Stamp(const Polygon& source, const Band& band, Emit emit)
    {
        const auto q = band.Footprint();
        for (int half = 0; half < 2; ++half)
        {
            const auto a = q[0].position, b = q[half ? 2 : 1].position, c = q[half ? 3 : 2].position;
            std::vector<Polygon> ignored;
            auto p = Partition(source, {Vertex(a), Vertex(b), Vertex(c)}, ignored);
            for (auto& vertex : p)
            {
                const double w = std::lerp(band.weightLow, band.weightHigh, band.Lateral(vertex.position));
                vertex.position.y = std::lerp(vertex.position.y, PlaneY(vertex.position, a, b, c), w);
            }
            Triangulate(p, emit);
        }
    }
    template <class Surface, class Transform> void Modify(Surface& surface, Transform transform)
    {
        Surface result;
        for (const auto& triangle : surface)
        {
            Polygon polygon;
            for (const auto& vertex : triangle)
            {
                V v = Vertex(vertex.position);
                if constexpr (requires { vertex.uv; })
                    v.uv = vertex.uv;
                if constexpr (requires { vertex.color; })
                    v.color = vertex.color;
                polygon.push_back(v);
            }
            transform(polygon,
                      [&](const std::array<V, 3>& t)
                      {
                          auto out = triangle;
                          for (int i = 0; i < 3; ++i)
                          {
                              out[i].position = t[i].position;
                              if constexpr (requires { out[i].uv; })
                                  out[i].uv = t[i].uv;
                              if constexpr (requires { out[i].color; })
                                  out[i].color = t[i].color;
                          }
                          result.push_back(out);
                      });
        }
        surface = std::move(result);
    }
    template <class Emit> void Process(Polygon polygon, const CrossingProfile& profile, Emit emit)
    {
        const auto offsets = Bands(profile);
        const double reach = profile.levelWidth + profile.approach;
        std::vector<Polygon> pending{std::move(polygon)};
        for (size_t row = 1; row < profile.rows.size() && !pending.empty(); ++row)
        {
            Band strip{profile.rows[row - 1], profile.rows[row], -reach, reach, 0, 0, false};
            std::vector<Polygon> remaining;
            for (auto& source : pending)
            {
                auto inside = Partition(std::move(source), strip.Footprint(), remaining);
                if (Area(inside) < 1e-10)
                    continue;
                for (size_t i = 1; i < offsets.size(); ++i)
                {
                    const auto weight = [&](double v)
                    { return 1 - Ease((std::abs(v) - profile.levelWidth) / profile.approach); };
                    Band band{strip.a, strip.b, offsets[i - 1], offsets[i], weight(offsets[i - 1]), weight(offsets[i]),
                              false};
                    std::vector<Polygon> ignored;
                    auto part = Partition(inside, band.Footprint(), ignored);
                    if (Area(part) < 1e-10)
                        continue;
                    // The complete pavement/curb/median/paint cross-section ends
                    // at the two slab edges. Only concrete contains flangeways.
                    if (row > profile.slabFirst && row <= profile.slabLast && band.weightLow == 1 &&
                        band.weightHigh == 1)
                        continue;
                    Stamp(part, band, emit);
                }
            }
            pending = std::move(remaining);
        }
        for (const auto& p : pending)
            Triangulate(p, emit);
    }
    bool Inside(Vector3 p, const Polygon& polygon)
    {
        bool inside = false;
        for (size_t i = 0, j = polygon.size() - 1; i < polygon.size(); j = i++)
        {
            const auto a = polygon[i].position, b = polygon[j].position;
            if ((a.z > p.z) != (b.z > p.z) && p.x < (b.x - a.x) * (p.z - a.z) / (b.z - a.z) + a.x)
                inside = !inside;
        }
        return inside;
    }
    void Intersections(Vector3 a, Vector3 b, Vector3 c, Vector3 d, std::vector<double>& cuts)
    {
        const double determinant = Cross(b - a, d - c);
        if (std::abs(determinant) < 1e-12)
            return;
        const double t = Cross(c - a, d - c) / determinant, u = Cross(c - a, b - a) / determinant;
        if (t > epsilon && t < 1 - epsilon && u >= -epsilon && u <= 1 + epsilon)
            cuts.push_back(t);
    }
    template <class Height>
    void CutOutline(std::vector<Vector3>& outline, const std::vector<Polygon>& cells, Height height)
    {
        std::vector<Vector3> out;
        for (size_t i = 0; i < outline.size(); ++i)
        {
            const auto a = outline[i], b = outline[(i + 1) % outline.size()];
            std::vector<double> cuts{0, 1};
            for (const auto& cell : cells)
                for (size_t j = 0; j < cell.size(); ++j)
                    Intersections(a, b, cell[j].position, cell[(j + 1) % cell.size()].position, cuts);
            std::sort(cuts.begin(), cuts.end());
            cuts.erase(std::unique(cuts.begin(), cuts.end(), [](double a, double b) { return std::abs(a - b) < 1e-9; }),
                       cuts.end());
            for (size_t j = 1; j < cuts.size(); ++j)
            {
                const auto middle = a + (b - a) * ((cuts[j - 1] + cuts[j]) / 2);
                for (double t : {cuts[j - 1], cuts[j]})
                {
                    const auto p = height(a + (b - a) * t, middle);
                    if (out.empty() || Math::Length(out.back() - p) > 1e-8)
                        out.push_back(p);
                }
            }
        }
        if (out.size() > 1 && Math::Length(out.front() - out.back()) < 1e-8)
            out.pop_back();
        outline = std::move(out);
    }
    std::vector<Vector3> Path(const Road& road)
    {
        std::vector<Vector3> result;
        for (size_t i = 0; i + 1 < road.points.size(); ++i)
            for (auto p : road.SegmentSamples(i))
                if (result.empty() || LengthXZ(p - result.back()) > 1e-8)
                    result.push_back(p);
        return result;
    }
    std::vector<double> Stations(const std::vector<Vector3>& path)
    {
        std::vector<double> result{0};
        for (size_t i = 1; i < path.size(); ++i)
            result.push_back(result.back() + LengthXZ(path[i] - path[i - 1]));
        return result;
    }
    TrackSection Interpolate(const TrackSection& a, const TrackSection& b, double t)
    {
        return {a.top + (b.top - a.top) * t, a.side + (b.side - a.side) * t};
    }
    void SolidQuad(RoadGeometry::ColoredSurface& surface, Vector3 a, Vector3 b, Vector3 c, Vector3 d, SDL_FColor color)
    {
        const auto emit = [&](const std::array<V, 3>& t)
        {
            surface.push_back({RoadGeometry::EdgeVertex{t[0].position, color},
                               RoadGeometry::EdgeVertex{t[1].position, color},
                               RoadGeometry::EdgeVertex{t[2].position, color}});
        };
        Triangulate(Quad(a, b, c, d), emit);
    }

    void BuildSlab(RoadGeometry& road, const CrossingProfile& profile)
    {
        constexpr SDL_FColor concrete{.56f, .55f, .52f, 1}, recess{.14f, .14f, .14f, 1};
        const double inner = RailroadSettings::gauge / 2 - RailroadSettings::flangeClearance;
        const double outer =
            RailroadSettings::gauge / 2 + RailroadSettings::headWidth + RailroadSettings::outerClearance;
        const std::array offsets{-profile.levelWidth, -outer, -inner, inner, outer, profile.levelWidth};
        const Vector3 depth{0, profile.slabDepth, 0}, groove{0, RailroadSettings::grooveDepth, 0};
        auto& surface = road.edgeTriangles;
        for (size_t i = profile.slabFirst + 1; i <= profile.slabLast; ++i)
        {
            for (size_t j = 1; j < offsets.size(); ++j)
            {
                const Band band{profile.rows[i - 1],
                                profile.rows[i],
                                offsets[j - 1],
                                offsets[j],
                                1,
                                1,
                                Groove((offsets[j - 1] + offsets[j]) / 2)};
                const auto q = band.Footprint();
                const auto a = q[0].position, b = q[1].position, c = q[2].position, d = q[3].position;
                const Vector3 drop = band.groove ? groove : Vector3{};
                SolidQuad(surface, a - drop, b - drop, c - drop, d - drop, band.groove ? recess : concrete);
                SolidQuad(surface, d - depth, c - depth, b - depth, a - depth, concrete);
                if (band.groove)
                {
                    SolidQuad(surface, a, a - groove, d - groove, d, recess);
                    SolidQuad(surface, b - groove, b, c, c - groove, recess);
                }
            }
        }
        auto& slab = road.crossingSlabs.emplace_back();
        std::vector<Vector3> bottomRim;
        const auto point = [&](size_t i, double offset, double drop = 0.0)
        { return profile.rows[i].top + profile.rows[i].side * offset - Vector3{0, drop, 0}; };
        const auto end = [&](size_t i, bool reverse)
        {
            std::vector<Vector3> cap, bottom;
            for (size_t j = 1; j < offsets.size(); ++j)
            {
                const double drop = Groove((offsets[j - 1] + offsets[j]) / 2) ? RailroadSettings::grooveDepth : 0;
                cap.push_back(point(i, offsets[j - 1], drop));
                cap.push_back(point(i, offsets[j], drop));
                bottom.push_back(point(i, offsets[j - 1]) - depth);
                bottom.push_back(point(i, offsets[j]) - depth);
            }
            if (reverse)
            {
                std::reverse(cap.begin(), cap.end());
                std::reverse(bottom.begin(), bottom.end());
            }
            slab.outline.insert(slab.outline.end(), cap.begin(), cap.end());
            bottomRim.insert(bottomRim.end(), bottom.begin(), bottom.end());
        };
        end(profile.slabFirst, false);
        for (size_t i = profile.slabFirst + 1; i < profile.slabLast; ++i)
        {
            slab.outline.push_back(point(i, profile.levelWidth));
            bottomRim.push_back(point(i, profile.levelWidth) - depth);
        }
        end(profile.slabLast, true);
        for (size_t i = profile.slabLast; i > profile.slabFirst + 1; --i)
        {
            slab.outline.push_back(point(i - 1, -profile.levelWidth));
            bottomRim.push_back(point(i - 1, -profile.levelWidth) - depth);
        }
        for (size_t i = 0; i < slab.outline.size(); ++i)
        {
            const auto a = slab.outline[i], b = slab.outline[(i + 1) % slab.outline.size()];
            if (LengthXZ(b - a) < 1e-8)
                continue;
            // The skirt extends below ground even at the flangeway floor.
            const auto bottomA = bottomRim[i], bottomB = bottomRim[(i + 1) % bottomRim.size()];
            SolidQuad(surface, a, bottomA, bottomB, b, concrete);
            slab.walls.push_back({a, b, bottomB, bottomA});
        }
    }
} // namespace

bool RailroadCrossingLink::Valid() const
{
    return railroadId > 0 && std::isfinite(roadFraction) && roadFraction >= 0 && roadFraction <= 1 &&
           std::isfinite(railFraction) && railFraction >= 0 && railFraction <= 1;
}

std::vector<CrossingSite> FindRailroadCrossings(const Road& road, const Road& railway)
{
    std::vector<CrossingSite> result;
    if (road.style.railroad || !railway.style.railroad || railway.id <= 0)
        return result;
    const auto a = Path(road), b = Path(railway);
    if (a.size() < 2 || b.size() < 2)
        return result;
    const auto sa = Stations(a), sb = Stations(b);
    for (size_t i = 1; i < a.size(); ++i)
        for (size_t j = 1; j < b.size(); ++j)
        {
            const auto u = a[i] - a[i - 1], v = b[j] - b[j - 1];
            const double cross = Cross(u, v);
            if (std::abs(cross) < 1e-8)
                continue;
            const auto delta = b[j - 1] - a[i - 1];
            const double t = Cross(delta, v) / cross, s = Cross(delta, u) / cross;
            if (t < 0 || t > 1 || s < 0 || s > 1)
                continue;
            CrossingSite site{{railway.id, (sa[i - 1] + (sa[i] - sa[i - 1]) * t) / sa.back(),
                               (sb[j - 1] + (sb[j] - sb[j - 1]) * s) / sb.back()},
                              a[i - 1] + u * t};
            if (std::ranges::none_of(result,
                                     [&](const auto& other) { return LengthXZ(other.position - site.position) < .01; }))
                result.push_back(site);
        }
    return result;
}

std::optional<CrossingProfile> PlanRailroadCrossing(const Road& road, const Road& railway, const CrossingSite& site)
{
    if (railway.trackSections.size() < 2)
        return {};
    const auto path = Path(road), rail = Path(railway);
    const auto s = Stations(path), rs = Stations(rail);
    const double station = site.link.roadFraction * s.back(), railStation = site.link.railFraction * rs.back();
    const auto index =
        std::clamp<size_t>(std::upper_bound(s.begin(), s.end(), station) - s.begin(), 1, path.size() - 1);
    const auto direction =
        Math::Normalize(Vector3{path[index].x - path[index - 1].x, 0, path[index].z - path[index - 1].z});
    std::vector<Vector3> heads;
    for (const auto& row : railway.trackSections)
        heads.push_back(row.top);
    const auto distances = Stations(heads);
    // The rail heads can be offset horizontally by banking; use the authored
    // rail station as a fraction of the same swept path.
    const double center = site.link.railFraction * distances.back();
    const size_t ri = std::clamp<size_t>(
        std::upper_bound(distances.begin(), distances.end(), center) - distances.begin(), 1, heads.size() - 1);
    const auto forward = Math::Normalize(Vector3{heads[ri].x - heads[ri - 1].x, 0, heads[ri].z - heads[ri - 1].z});
    const double sine = std::abs(Cross(direction, forward));
    if (sine < 0.25)
        return {}; // Nearly parallel tracks need a different junction layout.
    const auto& settings = *railway.style.railroad;
    const double toe = settings.width / 2 + settings.height * 2 +
                       (railway.style.edge == RoadEdge::Transition ? RoadStyle::stripWidth : 0);
    const double level = toe + 0.5;
    const double width = std::max(road.style.LeftWidth(), road.style.RightWidth()) + road.style.EdgeWidth();
    const double skew = std::abs(Math::Dot(direction, forward));
    const double deckRun = (level + width * skew) / sine;
    const double available = std::min(station, s.back() - station) - deckRun;
    const double t = (center - distances[ri - 1]) / (distances[ri] - distances[ri - 1]);
    const double elevation = std::lerp(heads[ri - 1].y, heads[ri].y, t);
    const double approach = std::min(std::max(15.0, std::abs(elevation - site.position.y) * 12), available - 1);
    if (approach < 5 || std::abs(elevation - site.position.y) > approach * .20)
        return {};
    // Reserve room for the whole road, not just the centerline intersection.
    constexpr double slabMargin = 3.0; // Pad projects past both sides of the full road footprint.
    const double core = (width + level * skew) / sine + slabMargin;
    if (railStation < core || rs.back() - railStation < core)
        return {};
    const double reach = level + approach * sine;
    double half = std::max(core, (width + reach * skew) / sine + slabMargin);
    if (half - core < 1e-6)
        half = core; // Avoid a near-zero strip at almost perpendicular crossings.
    CrossingProfile result;
    result.levelWidth = level;
    result.approach = approach * sine;
    result.slabDepth = settings.height + RailroadSettings::tieHeight + RailroadSettings::railHeight + 0.25;
    const auto sample = [&](double at)
    {
        const size_t i = std::clamp<size_t>(
            std::upper_bound(distances.begin(), distances.end(), at) - distances.begin(), 1, heads.size() - 1);
        return Interpolate(railway.trackSections[i - 1], railway.trackSections[i],
                           (at - distances[i - 1]) / (distances[i] - distances[i - 1]));
    };
    std::vector<double> cuts{center - half, center - core, center + core, center + half};
    for (double d : distances)
        if (d > center - half && d < center + half &&
            std::ranges::none_of(cuts, [&](double cut) { return std::abs(cut - d) < 1e-7; }))
            cuts.push_back(d);
    std::sort(cuts.begin(), cuts.end());
    cuts.erase(std::unique(cuts.begin(), cuts.end()), cuts.end());
    result.rows.push_back(sample(cuts.front()));
    for (size_t i = 1; i < cuts.size(); ++i)
    {
        const int steps = std::max(1, static_cast<int>(std::ceil(cuts[i] - cuts[i - 1])));
        for (int j = 1; j <= steps; ++j)
            result.rows.push_back(sample(std::lerp(cuts[i - 1], cuts[i], double(j) / steps)));
        if (cuts[i] == center - core)
            result.slabFirst = result.rows.size() - 1;
        if (cuts[i] == center + core)
            result.slabLast = result.rows.size() - 1;
    }
    // A tight curve can fold a wide approach ribbon back across itself. Leave
    // that layout unmodified instead of emitting overlapping crossing cells.
    for (size_t i = 1; i < result.rows.size(); ++i)
    {
        const Band band{result.rows[i - 1], result.rows[i], -reach, reach, 0, 0, false};
        const auto quad = band.Footprint();
        double sign = 0;
        for (size_t j = 0; j < 4; ++j)
        {
            const double turn = Cross(quad[(j + 1) % 4].position - quad[j].position,
                                      quad[(j + 2) % 4].position - quad[(j + 1) % 4].position);
            if (std::abs(turn) < 1e-8 || sign * turn < 0)
                return {};
            sign = turn;
        }
    }
    return result;
}

void ApplyRailroadCrossings(RoadGeometry& road, const std::vector<CrossingProfile>& profiles)
{
    for (const auto& profile : profiles)
    {
        const auto stamp = [&](auto& surface)
        { Modify(surface, [&](Polygon p, auto emit) { Process(std::move(p), profile, emit); }); };
        stamp(road.pavementTriangles);
        stamp(road.edgeTriangles);
        for (auto& [material, surface] : road.transitionStrips)
            stamp(surface);
        for (auto* stripes : {&road.whiteStripes, &road.yellowStripes})
        {
            std::vector<RoadGeometry::Triangle> result;
            for (const auto& tri : *stripes)
            {
                const Vector3 lift{0, RoadStyle::stripeHeight, 0};
                Process({Vertex(tri[0] - lift), Vertex(tri[1] - lift), Vertex(tri[2] - lift)}, profile,
                        [&](const auto& t)
                        { result.push_back({t[0].position + lift, t[1].position + lift, t[2].position + lift}); });
            }
            *stripes = std::move(result);
        }
        const auto bands = MakeBands(profile);
        std::vector<Polygon> cells;
        for (const auto& band : bands)
        {
            const auto q = band.Footprint();
            cells.push_back({q[0], q[1], q[2]});
            cells.push_back({q[0], q[2], q[3]});
        }
        CutOutline(road.outline, cells,
                   [&](Vector3 p, Vector3 middle)
                   {
                       for (size_t i = 0; i < cells.size(); ++i)
                           if (Inside(middle, cells[i]))
                           {
                               const auto& band = bands[i / 2];
                               const auto& cell = cells[i];
                               const double w = std::lerp(band.weightLow, band.weightHigh, band.Lateral(p));
                               p.y = std::lerp(p.y, PlaneY(p, cell[0].position, cell[1].position, cell[2].position), w);
                               break;
                           }
                       return p;
                   });
    }
    for (const auto& profile : profiles)
        BuildSlab(road, profile);
}

void WorldDocument::RefreshRailroadCrossings(bool force)
{
    if (!force && crossingRevision == RoadGeometry::latestRevision)
        return;
    std::map<int, size_t> byId;
    for (size_t i = 0; i < roads.size(); ++i)
        byId.emplace(roads[i].id, i);
    for (auto& road : roads)
    {
        std::vector<CrossingProfile> profiles;
        for (const auto& link : road.crossings)
        {
            const auto it = byId.find(link.railroadId);
            if (it == byId.end())
                continue; // Deleting a source disables its crossing; undo can restore it.
            const auto& railway = roads[it->second];
            auto candidates = FindRailroadCrossings(road, railway);
            if (candidates.empty())
                continue;
            const auto score = [&](const CrossingSite& site)
            {
                return std::abs(site.link.roadFraction - link.roadFraction) +
                       std::abs(site.link.railFraction - link.railFraction);
            };
            const auto best = std::min_element(candidates.begin(), candidates.end(),
                                               [&](const auto& a, const auto& b) { return score(a) < score(b); });
            if (auto profile = PlanRailroadCrossing(road, railway, *best))
            {
                profiles.push_back(std::move(*profile));
            }
        }
        if (profiles != road.crossingProfiles)
        {
            road.crossingProfiles = std::move(profiles);
            road.Rebuild();
        }
    }
    crossingRevision = RoadGeometry::latestRevision;
}
