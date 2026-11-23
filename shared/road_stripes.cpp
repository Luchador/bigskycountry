#include "road.hpp"
#include "road_paint.hpp"
#include "mesh.hpp"

#include <algorithm>
#include <cmath>
#include <optional>
#include <limits>

namespace
{
    constexpr double stripeWidth = 0.16;
    constexpr double yellowGap = 0.16; // Clear space between the two yellow stripes.
    constexpr double yellowPairOffset = (yellowGap + stripeWidth) * 0.5;
    constexpr double goreWidth = 1.0;
    constexpr double fullWidthFraction = 0.5; // Begin the marked auxiliary lane halfway through the taper.
    constexpr double goreStartFraction = 0.7;
    constexpr double dashLength = 3.0;
    constexpr double gapLength = RoadStyle::dashPeriod - dashLength;

    struct Span
    {
        Vector3 start;
        Vector3 end;
        Vector3 left;
        double length;
    };

    constexpr Vector3 defaultLeft{0, 0, 1}; // A wholly vertical road has no horizontal heading.

    template<typename Visit>
    void VisitSpans(const std::vector<Vector3>& path, std::optional<Vector3> initialLeft, Visit visit)
    {
        // Use the preceding heading for vertical spans. If the whole path
        // begins vertically, look ahead for its first horizontal heading.
        Vector3 left = initialLeft.value_or(defaultLeft);
        if (!initialLeft)
        {
            for (std::size_t i = 1; i < path.size(); ++i)
            {
                const Vector3 delta = path[i] - path[i - 1];
                const double run = std::hypot(delta.x, delta.z);
                if (run > 0.0)
                {
                    left = {-delta.z / run, 0, delta.x / run};
                    break;
                }
            }
        }

        for (std::size_t i = 1; i < path.size(); ++i)
        {
            const Vector3 delta = path[i] - path[i - 1];
            const double run = std::hypot(delta.x, delta.z);
            if (run > 0.0)
            {
                left = {-delta.z / run, 0, delta.x / run};
            }
            else if (delta.y == 0.0)
            {
                continue;
            }
            visit(Span{path[i - 1], path[i], left, run}, i);
        }
    }

    struct RoadProfile
    {
        const Road* road = nullptr;
        std::vector<double> knots;
        std::vector<RoadStyle> styles;

        std::pair<std::size_t, double> Location(double distance) const
        {
            const auto next = std::upper_bound(knots.begin(), knots.end(), distance);
            const auto segment = std::clamp<std::size_t>(next - knots.begin(), 1, knots.size() - 1) - 1;
            const double length = knots[segment + 1] - knots[segment];
            return {segment, length > 0 ? (distance - knots[segment]) / length : 0};
        }

        Road::LaneBlend At(double distance) const
        {
            const auto [segment, fraction] = Location(distance);
            return road->SegmentSurfaceBlend(segment, fraction);
        }

        double Shoulder(double distance, bool left) const
        {
            const auto [segment, fraction] = Location(distance);
            const auto blend = road->SegmentShoulderBlend(segment, fraction, left);
            return std::lerp(double(blend.from), double(blend.to), blend.mix) * RoadStyle::shoulderWidth;
        }

        double PavementWidth(double distance, bool left) const
        {
            return Mix(distance, [](const RoadStyle& s) { return s.HalfSeparation() + s.GetLaneCount() * s.TravelLaneWidth() * 0.5; }) +
                   Shoulder(distance, left);
        }

        template<typename Value> double Mix(double distance, Value value) const
        {
            const auto blend = At(distance);
            return std::lerp(double(value(styles[blend.from])), double(value(styles[blend.to])), blend.mix);
        }
    };

    struct RoadPath
    {
        std::array<std::optional<RoadMouthFrame>, 2> caps;
        std::vector<Span> spans;
        std::vector<double> distances{0.0}; // Cumulative 3D centerline length at each station.
        std::vector<Vector3> curveLeft; // One cross-section axis per Bézier station.
        std::vector<Vector3> curveUp;   // Per-station up for a bank override or an attached ramp.
        std::optional<RoadProfile> profile;
    };

    Vector3 CurveLeft(Vector3 tangent, Vector3 fallback, Vector3 up = {0, 1, 0})
    {
        const Vector3 across = Math::Cross(tangent, up);
        const double run = Math::Length(across);
        if (run <= 1e-8 * std::abs(Math::Dot(tangent, up))) return fallback;
        return across * (1.0 / run);
    }

    struct CrossSection
    {
        Vector3 center, lateral;
        double distance;
        double bevelSlope = 0.0; // Signed advance along the join per meter from the centerline.
        Vector3 up{0, 1, 0};
        Vector3 At(double offset) const { return center + lateral * offset; }
    };

    // A positive offset is left of the road in the direction its points were placed.
    template<typename Visit>
    void VisitOffsetStations(const RoadPath& path, double offset, Visit visit)
    {
        const auto& spans = path.spans;
        const auto up = [&](std::size_t i) { return path.curveUp.empty() ? Vector3{0, 1, 0} : path.curveUp[i]; };
        const auto endSection = [&](int end, CrossSection section)
        {
            if (const auto& cap = path.caps[end])
            {
                section.lateral = cap->left;
                section.up = cap->up;
            }
            visit(section);
        };
        if (!path.curveLeft.empty())
        {
            // A sampled spline is smooth here. Sweeping both sides from its tangent
            // gives one cross section at this elevation for every part of the road.
            endSection(0, {spans.front().start, path.curveLeft.front(), 0.0, 0.0, up(0)});
            for (std::size_t i = 0; i + 1 < spans.size(); ++i)
                visit(CrossSection{spans[i].end, path.curveLeft[i + 1], path.distances[i + 1], 0.0,
                                   up(i + 1)});
            endSection(1, {spans.back().end, path.curveLeft.back(), path.distances.back(), 0.0, up(spans.size())});
            return;
        }

        // On an inside bend the offset lines intersect before/after the corner.
        // Record that trim distance; a negative entry requests a pavement-style bevel.
        const auto leftAt = [&](const Span& span, Vector3 normal)
        {
            if (normal == Vector3{0, 1, 0}) return span.left;
            return CurveLeft(span.end - span.start, span.left, normal);
        };
        std::vector<double> trim(spans.size() + 1, 0.0);

        for (std::size_t i = 1; i < spans.size(); ++i)
        {
            const Vector3 a = leftAt(spans[i - 1], up(i));
            const Vector3 b = leftAt(spans[i], up(i));
            const double turn = Math::Dot(Math::Cross(b, a), up(i));
            const double divisor = 1.0 + Math::Dot(a, b);
            trim[i] = divisor > 0.0 ? offset * turn / divisor : -1.0;
        }

        endSection(0, {spans.front().start, leftAt(spans.front(), up(0)), 0.0, 0.0, up(0)});

        for (std::size_t i = 1; i < spans.size(); ++i)
        {
            const Span& a = spans[i - 1];
            const Span& b = spans[i];
            const Vector3 leftA = leftAt(a, up(i)), leftB = leftAt(b, up(i));

            // Keep a miter only if the trims from both ends fit on each span.
            if (trim[i] >= 0.0 && trim[i] + std::max(0.0, trim[i - 1]) <= a.length && trim[i] + std::max(0.0, trim[i + 1]) <= b.length)
            {
                const double divisor = 1.0 + Math::Dot(leftA, leftB);
                visit(CrossSection{b.start, (leftA + leftB) * (1.0 / divisor), path.distances[i], 0.0, up(i)});
            }
            else
            {
                // Both sections of a bevel share a center. Give the outer edge its
                // chord length in V and leave room on short spans between two bevels.
                const double room = 0.25 * std::min(path.distances[i] - path.distances[i - 1],
                                                    path.distances[i + 1] - path.distances[i]);
                const double slope = offset == 0.0 ? 0.0 :
                    std::min(0.5 * Math::Length(leftA - leftB), room / std::abs(offset));
                visit(CrossSection{a.end, leftA, path.distances[i], -slope, up(i)});
                visit(CrossSection{b.start, leftB, path.distances[i], slope, up(i)});
            }
        }

        endSection(1, {spans.back().end, leftAt(spans.back(), up(spans.size())), path.distances.back(), 0.0, up(spans.size())});
    }

    struct MarkingPath
    {
        std::array<std::optional<RoadMouthFrame>, 2> caps;
        std::vector<Vector3> points;
        std::vector<Vector3> curveLeft, curveUp;
        std::vector<double> widths; // Optional paint-width multiplier while a lane appears/disappears.
        std::vector<bool> hiddenSpans; // Optional mask for the span ending at each point; the first entry is unused.

        void Append(Vector3 center, Vector3 left, double offset, bool curved, const Vector3* up)
        {
            points.push_back(center + left * offset);
            if (curved || up) curveLeft.push_back(left);
            if (up) curveUp.push_back(*up);
        }
    };

    MarkingPath OffsetPath(const RoadPath& path, double offset)
    {
        MarkingPath marking;
        marking.caps = path.caps;
        VisitOffsetStations(path, offset, [&](const CrossSection& station)
        {
            marking.Append(station.center, station.lateral, offset, !path.curveLeft.empty(),
                           path.curveUp.empty() ? nullptr : &station.up);
        });
        return marking;
    }

    void AppendBoundary(Road& road, int side, const std::vector<Vector3>& boundary)
    {
        if (side < 0)
        {
            road.outline.insert(road.outline.end(), boundary.begin(), boundary.end());
        }
        else
        {
            road.outline.insert(road.outline.end(), boundary.rbegin(), boundary.rend());
        }
    }

    using EdgeProfile = std::array<Vector3, 4>;

    void AddEdgeFace(Road& road, int side, const EdgeProfile& previous, const EdgeProfile& current,
                     int face, SDL_FColor inner, SDL_FColor outer)
    {
        const Road::EdgeVertex a{previous[face], inner}, b{previous[face + 1], outer};
        const Road::EdgeVertex c{current[face + 1], outer}, d{current[face], inner};
        if (side > 0)
        {
            road.edgeTriangles.push_back({a, b, c});
            road.edgeTriangles.push_back({a, c, d});
        }
        else
        {
            road.edgeTriangles.push_back({a, c, b});
            road.edgeTriangles.push_back({a, d, c});
        }
    }

    // On a bevel the sections meet at the center, leaving one triangle in the innermost band.
    template<typename MakeVertex, typename Emit>
    void VisitBand(int side, const CrossSection& a, const CrossSection& b, double ai, double ao, double bi, double bo,
                   MakeVertex vertex, Emit emit)
    {
        const auto p = vertex(a, ai), q = vertex(a, ao), r = vertex(b, bi), s = vertex(b, bo);
        if (ai != ao && a.At(ai) != b.At(bi)) emit(side > 0 ? std::array{q, r, p} : std::array{p, r, q});
        if (bi != bo) emit(side > 0 ? std::array{q, s, r} : std::array{q, r, s});
    }

    template<typename MakeVertex, typename Emit>
    void VisitBand(int side, const CrossSection& a, const CrossSection& b, double inner, double outer,
                   MakeVertex vertex, Emit emit)
    {
        const auto ai = vertex(a, inner), bi = vertex(b, inner), ao = vertex(a, outer), bo = vertex(b, outer);
        if (a.At(inner) != b.At(inner))
        {
            emit(side > 0 ? std::array{ao, bi, ai} : std::array{ai, bi, ao});
        }
        emit(side > 0 ? std::array{ao, bo, bi} : std::array{ao, bi, bo});
    }

    Vector3 BandPosition(const CrossSection& section, double offset) { return section.At(offset); }

    // A taper occupies only one carriageway. Station distances follow the rendered
    // centerline, including grade and any trimming at intersection mouths.
    struct TaperProfile
    {
        double clearance, width, endSlope;
        static double Ease(double t) { return t * t * (3.0 - 2.0 * t); }

        double OutwardShift(double f) const
        {
            // Hermite offset, tangent to the highway at the tip and to the ramp at the seam.
            return f * f * (clearance * (3.0 - 2.0 * f) - endSlope * (1.0 - f));
        }

        double Opening(double f) const { return Ease(std::min(f / fullWidthFraction, 1.0)); }
        double Outer(double f) const { return width * Opening(f) + OutwardShift(f); }
        double Inner(double f, double hostLaneWidth) const
        {
            // Start inside the outer highway lane. As the pavement widens, release the
            // shared width at the same rate. Both lanes fit before the divider begins.
            if (f < fullWidthFraction) return -std::min(width, hostLaneWidth) * (1.0 - Opening(f));
            const double nose = std::clamp((f - goreStartFraction) / (1.0 - goreStartFraction), 0.0, 1.0);
            return OutwardShift(f) * Ease(nose);
        }
    };

    struct Taper
    {
        RampLane lane;
        double tip, gore, stationDistance;
        size_t firstGuide = 0;
        double Fraction(double d) const { return std::clamp((d - tip) / (gore - tip), 0.0, 1.0); }

        TaperProfile Profile(double clearance) const
        {
            const double inner = clearance + lane.innerPavement - lane.halfTravel;
            const double width = 2.0 * lane.halfTravel;
            // A monotone outward shift keeps both edges advancing after the lane has opened.
            return {inner, width, std::min(inner + width, 2.0 * inner)};
        }
    };

    struct ShoulderOpening
    {
        const Taper* taper;
        std::vector<Vector3> outline; // In the host road's mesh frame.
        Vector3 lo, hi;

        bool Reaches(Vector3 a, Vector3 b, double width) const
        {
            return hi.x >= std::min(a.x, b.x) - width && lo.x <= std::max(a.x, b.x) + width &&
                   hi.z >= std::min(a.z, b.z) - width && lo.z <= std::max(a.z, b.z) + width;
        }
    };

    double CrossXZ(Vector3 a, Vector3 b) { return a.x * b.z - a.z * b.x; }

    // Add samples only where a branch can reach the shoulder. Long, straight
    // highways otherwise have no stations between their authored control points.
    std::vector<double> SampleShoulderOpenings(const std::vector<CrossSection>& stations,
                                const std::vector<ShoulderOpening>& openings, double width)
    {
        std::vector<double> sampled;
        if (openings.empty() || stations.empty()) return sampled;
        for (size_t i = 1; i < stations.size(); ++i)
        {
            const auto& a = stations[i - 1];
            const auto& b = stations[i];
            const auto delta = b.center - a.center;
            const double run2 = delta.x * delta.x + delta.z * delta.z;
            std::vector<double> cuts;
            if (run2 > 1e-12)
            {
                const double run = std::sqrt(run2);
                for (const auto& opening : openings)
                {
                    if (!opening.Reaches(a.center, b.center, width * std::max(Math::Length(a.lateral), Math::Length(b.lateral)))) continue;
                    double lo = 1, hi = 0, acrossLo = std::numeric_limits<double>::max(), acrossHi = -acrossLo;
                    for (const auto& point : opening.outline)
                    {
                        const auto v = point - a.center;
                        const double t = (v.x * delta.x + v.z * delta.z) / run2;
                        lo = std::min(lo, t); hi = std::max(hi, t);
                        const double across = CrossXZ(delta, v) / run;
                        acrossLo = std::min(acrossLo, across); acrossHi = std::max(acrossHi, across);
                    }
                    if (acrossLo > width || acrossHi < -width) continue;
                    lo = std::clamp(lo, 0.0, 1.0); hi = std::clamp(hi, 0.0, 1.0);
                    if (hi <= lo) continue;
                    const int steps = std::max(1, static_cast<int>(std::ceil((hi - lo) * run / 0.5)));
                    for (int j = 0; j <= steps; ++j)
                    {
                        const double t = std::lerp(lo, hi, double(j) / steps);
                        if (t > 0 && t < 1) cuts.push_back(t);
                    }
                }
            }
            std::ranges::sort(cuts);
            double previous = 0;
            for (double t : cuts)
            {
                if (t - previous < 1e-9) continue;
                sampled.push_back(std::lerp(a.distance, b.distance, t));
                previous = t;
            }
        }
        return sampled;
    }

    template<typename Visit>
    void VisitTaperSpans(const std::vector<CrossSection>& stations, const std::vector<Taper>& tapers, const std::vector<double>& shoulderSamples, Visit visit)
    {
        for (size_t i = 1; i < stations.size(); ++i)
        {
            const auto& a = stations[i - 1];
            const auto& b = stations[i];
            if (tapers.empty()) { visit(a, b, nullptr, a, b); continue; }
            std::vector<double> cuts;
            for (double d : shoulderSamples)
                if (d > a.distance && d < b.distance) cuts.push_back(d);
            for (const auto& taper : tapers)
            {
                // Resolve the curved width independently of host spline knots and paint dashes.
                // Include the exact start of each painted region, in either travel direction.
                const double nose = std::lerp(taper.tip, taper.gore, goreStartFraction);
                const auto cutCurve = [&](double start, double end, int subdivisions)
                {
                    if (std::max(start, end) <= a.distance || std::min(start, end) >= b.distance) return;
                    for (int j = 0; j <= subdivisions; ++j)
                    {
                        const double d = std::lerp(start, end, static_cast<double>(j) / subdivisions);
                        if (d > a.distance && d < b.distance) cuts.push_back(d);
                    }
                };
                const double full = std::lerp(taper.tip, taper.gore, fullWidthFraction);
                cutCurve(taper.tip, full, 64);
                cutCurve(full, nose, 24);
                cutCurve(nose, taper.gore, 48);
                // Break the mainline divider at the same stations as its dash pattern.
                const double lo = std::max(a.distance, std::min(taper.tip, taper.gore));
                const double hi = std::min(b.distance, std::max(taper.tip, taper.gore));
                for (double d = std::floor(lo / RoadStyle::dashPeriod) * RoadStyle::dashPeriod;
                     d < hi; d += RoadStyle::dashPeriod)
                    for (double edge : {d, d + dashLength})
                        if (edge > lo && edge < hi) cuts.push_back(edge);
            }
            std::ranges::sort(cuts);
            CrossSection last = a;
            const auto emit = [&](const CrossSection& next)
            {
                const double mid = (last.distance + next.distance) * 0.5;
                const auto active = std::ranges::find_if(tapers, [&](const Taper& t)
                {
                    return mid > std::min(t.tip, t.gore) && mid < std::max(t.tip, t.gore);
                });
                visit(last, next, active == tapers.end() ? nullptr : &*active, a, b);
                last = next;
            };
            for (double d : cuts)
            {
                if (d <= last.distance) continue;
                const double t = (d - a.distance) / (b.distance - a.distance);
                emit({a.center + (b.center - a.center) * t, a.lateral + (b.lateral - a.lateral) * t, d,
                      std::lerp(a.bevelSlope, b.bevelSlope, t), a.up + (b.up - a.up) * t});
            }
            emit(b);
        }
    }

    void PaintDashes(const MarkingPath& path, std::vector<Road::Triangle>& mesh);
    void PaintStrip(const MarkingPath& path, double lo, double hi, std::vector<Road::Triangle>& mesh,
                    std::optional<Vector3> initialLeft = {});

    void VariableLaneMarkings(Road& road, const RoadPath& path, int side, const std::vector<CrossSection>& stations)
    {
        const auto& profile = *path.profile;
        const int maximum = road.MaximumLaneCount();
        const auto halfTravel = [](const RoadStyle& s) { return s.GetLaneCount() * s.TravelLaneWidth() * 0.5; };
        const auto inside = [&](const RoadStyle& s)
        {
            if (s.IsDivided()) return s.HalfSeparation();
            if (s.HasCenterTurnLane()) return RoadStyle::laneWidth * 0.5;
            if (s.GetLaneCount() == 3) return side * road.ThreeLaneDividerOffset();
            return 0.0;
        };
        const auto throughLanes = [&](const RoadStyle& s)
        {
            if (s.GetLaneCount() < 2) return 0;
            return static_cast<int>(std::lround((s.HalfSeparation() + halfTravel(s) - inside(s)) / RoadStyle::laneWidth));
        };
        // Paint and guides share the exact cross sections of the pavement.
        const auto marking = [&](auto offset, auto visible, bool yellow, bool dashed)
        {
            if (!road.paintMarkings) return;
            MarkingPath line;
            for (std::size_t i = 0; i < stations.size(); ++i)
            {
                const auto& s = stations[i];
                // PaintStrip expects the authored left axis for upward-facing triangles
                // on both carriageways; these pavement stations point outward instead.
                line.Append(s.center, s.lateral * side, side * profile.Mix(s.distance, offset), true, &s.up);
                line.widths.push_back(profile.Mix(s.distance, visible));
                if (dashed && !yellow)
                {
                    // Keep white lane dividers off the taper, including spans that
                    // straddle the end of a transition on a curved segment.
                    const double start = stations[i ? i - 1 : i].distance;
                    const auto middle = profile.At((start + s.distance) * 0.5);
                    const auto losesLane = [&](Road::LaneBlend b)
                    {
                        const auto& from = profile.styles[b.from];
                        const auto& to = profile.styles[b.to];
                        if (from.GetLaneCount() == to.GetLaneCount()) return false;
                        // Replacing a center turn lane with a median does not add
                        // or remove through lanes. Keep their dash phase continuous.
                        const bool centerConversion = (from.HasCenterTurnLane() && to.IsDivided()) ||
                                                      (to.HasCenterTurnLane() && from.IsDivided());
                        return !centerConversion || throughLanes(from) != throughLanes(to);
                    };
                    const auto blending = [&](Road::LaneBlend b)
                    { return losesLane(b) && b.mix > 0 && b.mix < 1; };
                    line.hiddenSpans.push_back(i && (losesLane(middle) ||
                        blending(profile.At(start)) || blending(profile.At(s.distance))));
                }
            }
            auto& triangles = yellow ? road.yellowStripes : road.whiteStripes;
            if (dashed) PaintDashes(line, triangles);
            else PaintStrip(line, -stripeWidth * 0.5, stripeWidth * 0.5, triangles);
        };
        const auto guide = [&](auto offset, auto visible)
        {
            std::vector<Vector3> run;
            const auto flush = [&]
            {
                if (run.size() > 1) road.laneCenterlines.push_back(std::move(run));
                run.clear();
            };
            for (const auto& s : stations)
            {
                if (profile.Mix(s.distance, visible) <= 1e-8) { flush(); continue; }
                const auto p = s.At(profile.Mix(s.distance, offset));
                if (run.empty() || run.back() != p) run.push_back(p);
            }
            flush();
        };
        if (road.style.oneWay)
        {
            if (side > 0) return;
            for (int lane = 0; lane < maximum; ++lane)
            {
                guide([&](const RoadStyle& s) { return std::max(-halfTravel(s), halfTravel(s) - (lane + 0.5) * s.TravelLaneWidth()); },
                      [&](const RoadStyle& s) { return s.GetLaneCount() >= 2 && lane < s.GetLaneCount(); });
                if (lane == 0) continue;
                marking([&](const RoadStyle& s) { return std::max(-halfTravel(s), halfTravel(s) - lane * s.TravelLaneWidth()); },
                        [&](const RoadStyle& s) { return lane < s.GetLaneCount(); }, false, true);
            }
            return;
        }
        for (int lane = 0; lane < (maximum + 1) / 2; ++lane)
        {
            guide([&](const RoadStyle& s) { return std::min(s.HalfSeparation() + halfTravel(s), inside(s) + (lane + 0.5) * RoadStyle::laneWidth); },
                  [&](const RoadStyle& s) { return lane < throughLanes(s); });
            if (lane == 0) continue;
            marking([&](const RoadStyle& s) { return std::min(s.HalfSeparation() + halfTravel(s), inside(s) + lane * RoadStyle::laneWidth); },
                    [&](const RoadStyle& s) { return lane < throughLanes(s); }, false, true);
        }
        marking([&](const RoadStyle& s) { return inside(s) + (s.IsDivided() ? stripeWidth * 0.5 : yellowPairOffset); },
                [](const RoadStyle& s) { return s.GetLaneCount() >= 2; }, true, false);
        marking([&](const RoadStyle& s) { return inside(s) - yellowPairOffset; },
                [](const RoadStyle& s) { return s.HasCenterTurnLane(); }, true, true);
        if (side > 0)
            guide([](const RoadStyle&) { return 0.0; }, [](const RoadStyle& s) { return s.HasCenterTurnLane(); });
    }

    // Traffic shares the pavement stations, including banks, skewed mouths and
    // profile blends. Narrowing/new lanes stop at the full-width boundary until
    // the later lane-change milestone supplies merge/split movements.
    void TrafficPaths(Road& road, const RoadPath& path, int side, const std::vector<CrossSection>& stations)
    {
        if (road.style.railroad) return;
        const auto add = [&](int ordinal, int direction, auto offset, auto usable)
        {
            RoadTrafficPath run{direction, ordinal, RoadStyle::laneWidth, {}};
            const auto flush = [&] {
                if (run.points.size() > 1) road.trafficPaths.push_back(run);
                run.points.clear();
            };
            for (const auto& station : stations)
            {
                const double available = path.profile ? path.profile->Mix(station.distance, usable) : usable(road.style);
                if (available < 1.0 - 1e-8) { flush(); continue; }
                const double lateral = path.profile ? path.profile->Mix(station.distance, offset) : offset(road.style);
                const Vector3 p = station.At(lateral);
                if (run.points.empty() || Math::Length(p - run.points.back()) > 1e-8) run.points.push_back(p);
            }
            flush();
        };
        const int maximum = road.MaximumLaneCount();
        if (road.style.oneWay)
        {
            if (side > 0) return;
            for (int lane = 0; lane < maximum; ++lane)
                add(lane, 1, [=](const RoadStyle& s) { return (s.GetLaneCount() * 0.5 - lane - 0.5) * s.TravelLaneWidth(); },
                    [=](const RoadStyle& s) { return lane < s.GetLaneCount() ? 1.0 : 0.0; });
            return;
        }
        const auto inside = [&](const RoadStyle& s) {
            if (s.IsDivided()) return s.HalfSeparation();
            if (s.HasCenterTurnLane()) return RoadStyle::laneWidth * 0.5;
            if (s.GetLaneCount() == 3) return side * road.ThreeLaneDividerOffset();
            return 0.0;
        };
        for (int lane = 0; lane < (maximum + 1) / 2; ++lane)
            add(lane, -side, [&](const RoadStyle& s) { return inside(s) + (lane + 0.5) * RoadStyle::laneWidth; },
                [&](const RoadStyle& s) {
                    const double lanes = (s.HalfSeparation() + s.GetLaneCount() * s.TravelLaneWidth() * 0.5 - inside(s)) / RoadStyle::laneWidth;
                    return s.GetLaneCount() >= 2 && lane + 1 <= lanes + 1e-8 ? 1.0 : 0.0;
                });
    }

    void AppendLanePoint(std::vector<Vector3>& line, Vector3 point)
    {
        if (line.empty() || line.back() != point) line.push_back(point);
    }

    // Continue the terminal cross section, sharing its exact edge vertices.
    // The red channel fades along the cap and, for side strips, across its corners.
    std::optional<std::array<Vector3, 2>> BuildDeadEnd(Road& road, const RoadPath& path, int end, Vector3 tangent)
    {
        const auto& point = end ? road.points.back() : road.points.front();
        if (point.deadEndLength <= 0) return {};
        CrossSection section{};
        bool first = true;
        VisitOffsetStations(path, 0, [&](const CrossSection& station)
        {
            if (end || first) section = station;
            first = false;
        });
        const Vector3 outward = Math::Normalize(tangent) * (end ? 1.0 : -1.0);
        if (outward == Vector3{}) return {};
        const auto style = road.EndpointStyle(end == 0);
        const double gap = path.profile ? path.profile->Mix(section.distance,
            [](const RoadStyle& s) { return s.HalfSeparation(); }) : style.HalfSeparation();
        const double median = path.profile ? path.profile->Mix(section.distance,
            [](const RoadStyle& s) { return s.HalfMedianWidth(); }) : style.HalfMedianWidth();
        struct Rim { double offset, height, red; };
        const auto side = [&](int sign)
        {
            const auto widthOf = [&](const RoadStyle& s) { return sign < 0 ? s.RightWidth() : s.LeftWidth(); };
            const double width = path.profile ? path.profile->PavementWidth(section.distance, sign > 0) : widthOf(style);
            std::vector<Rim> rim{{0, 0, 1}};
            if (median > 0) rim.push_back({sign * median, 0, 1});
            if (gap > 0) rim.push_back({sign * gap, 0, 1});
            rim.push_back({sign * width, 0, 1});
            if (style.edge != RoadEdge::None)
                rim.push_back({sign * (width + RoadStyle::stripWidth), 0, style.edge == RoadEdge::Transition ? 0.0 : 1.0});
            if (style.edge == RoadEdge::Curb)
            {
                rim.push_back({sign * (width + RoadStyle::stripWidth), RoadStyle::curbHeight, 1});
                rim.push_back({sign * (width + style.EdgeWidth()), RoadStyle::curbHeight, 1});
            }
            return rim;
        };
        auto rim = side(-1);
        std::reverse(rim.begin(), rim.end());
        const auto left = side(1);
        rim.insert(rim.end(), left.begin() + 1, left.end());
        const auto vertex = [&](const Rim& at, double fraction)
        {
            const double remaining = 1.0 - fraction;
            const Vector3 p = section.At(at.offset) + outward * (point.deadEndLength * fraction)
                            + section.up * (at.height * remaining);
            const bool positive = at.offset >= 0;
            const double travelTile = path.profile ? path.profile->Mix(section.distance, [](const RoadStyle& s) { return s.TravelLaneWidth(); }) : style.TravelLaneWidth();
            const double tileWidth = std::abs(at.offset) < gap ? 2 * RoadStyle::innerShoulderWidth : travelTile;
            const auto originU = [&](const RoadStyle& s) { return s.GetLaneCount() * 0.5 + (s.IsDivided() && !positive ? 1 : 0); };
            const double centerU = path.profile ? path.profile->Mix(section.distance, originU) : originU(style);
            const double u = centerU - (positive ? 1 : -1) * (std::abs(at.offset) - gap) / tileWidth;
            const double v = (section.distance + (end ? 1 : -1) * point.deadEndLength * fraction) / RoadStyle::laneWidth;
            return Road::TransitionVertex{p, {static_cast<float>(at.red * remaining), 0, 0, 1},
                                          {static_cast<float>(u), static_cast<float>(v)}};
        };
        auto& strip = road.transitionStrips[style.transitionMaterial];
        const auto emit = [&](std::array<Road::TransitionVertex, 3> t)
        {
            if (Math::Length(Math::Cross(t[1].position - t[0].position, t[2].position - t[0].position)) < 1e-12) return;
            if (!end) std::swap(t[1], t[2]);
            strip.push_back(t);
        };
        for (std::size_t i = 1; i < rim.size(); ++i)
        {
            // A center row lets corner weights fade in both directions instead
            // of depending on which diagonal happens to triangulate the quad.
            for (int row = 0; row < 2; ++row)
            {
                const auto a = vertex(rim[i-1], row * 0.5), b = vertex(rim[i], row * 0.5);
                const auto c = vertex(rim[i], (row+1) * 0.5), d = vertex(rim[i-1], (row+1) * 0.5);
                emit({a, b, c}); emit({a, c, d});
            }
        }
        return std::array{vertex(rim.front(), 1).position, vertex(rim.back(), 1).position};
    }

    // Use the same cross sections for pavement, its outline and any edge treatment.
    // Adjacent spans then meet at the same elevation along the entire join.
    void BuildRoadSide(Road& road, const RoadPath& path, int side, bool hasGradedBend, const std::vector<Taper>& tapers)
    {
        const RoadEdge edge = road.style.edge;
        const int faces = edge == RoadEdge::Curb ? 3 : (edge == RoadEdge::Transition ? 1 : 0);
        const bool variable = path.profile.has_value();
        const bool bridge = road.HasOverpasses();
        const auto pavementAt = [&](const CrossSection& section)
        {
            const auto width = [&](const RoadStyle& s) { return side < 0 ? s.RightWidth() : s.LeftWidth(); };
            return variable ? path.profile->PavementWidth(section.distance, side > 0) : width(road.style);
        };
        const auto travelAt = [&](const CrossSection& section)
        {
            const auto width = [](const RoadStyle& s) { return s.HalfSeparation() + s.GetLaneCount() * s.TravelLaneWidth() * 0.5; };
            return variable ? path.profile->Mix(section.distance, width) : width(road.style);
        };
        const int lanes = variable ? road.MaximumLaneCount() : road.style.GetLaneCount();
        auto widest = road.style;
        widest.SetLaneCount(lanes);
        const double pavement = road.MaximumHalfSeparation() + road.MaximumHalfTravelWidth() +
            road.MaximumShoulderWidth(side > 0);
        const double halfGap = road.style.HalfSeparation();
        const double halfMedian = road.style.HalfMedianWidth();
        const double laneWidth = widest.TravelLaneWidth();
        const auto gapAt = [&](const CrossSection& s) { return variable ? path.profile->Mix(s.distance,
            [](const RoadStyle& style) { return style.HalfSeparation(); }) : halfGap; };
        const auto medianAt = [&](const CrossSection& s) { return variable ? path.profile->Mix(s.distance,
            [](const RoadStyle& style) { return style.HalfMedianWidth(); }) : halfMedian; };

        std::vector<ShoulderOpening> openings;
        const auto frame = ObjectTransform{road.points.front().position, road.rotation}.ToMeshTransform();
        for (const auto& cut : road.rampShoulderCuts)
        {
            if (cut.side != side || cut.outline.empty() || road.MaximumShoulderWidth(side > 0) == 0) continue;
            const auto taper = std::ranges::find_if(tapers, [&](const Taper& t) { return t.lane.station == cut.station; });
            if (taper == tapers.end()) continue;
            auto& opening = openings.emplace_back(ShoulderOpening{&*taper, cut.outline, {}, {}});
            if (road.rotation != Vector3{})
                for (auto& p : opening.outline) p = frame.Unrotate(p - frame.origin);
            opening.lo = opening.hi = opening.outline.front();
            for (const auto& p : opening.outline)
            {
                opening.lo.x = std::min(opening.lo.x, p.x); opening.lo.z = std::min(opening.lo.z, p.z);
                opening.hi.x = std::max(opening.hi.x, p.x); opening.hi.z = std::max(opening.hi.z, p.z);
            }
        }
        const auto shoulderEdge = [&](const CrossSection& s)
        {
            const double travel = travelAt(s);
            double edge = pavementAt(s);
            for (const auto& opening : openings)
            {
                const auto& taper = *opening.taper;
                // The taper itself supplies the shared pavement. Beyond its seam,
                // reserve the same clear gore between the branch and mainline rims.
                if ((s.distance - taper.gore) * (taper.gore - taper.tip) < -1e-8) continue;
                if (!opening.Reaches(s.center, s.center, edge * Math::Length(s.lateral))) continue;
                for (size_t i = 0; i < opening.outline.size(); ++i)
                {
                    const auto p = opening.outline[i];
                    const auto segment = opening.outline[(i + 1) % opening.outline.size()] - p;
                    const double denominator = CrossXZ(s.lateral, segment);
                    if (std::abs(denominator) < 1e-10) continue;
                    const double u = CrossXZ(p - s.center, s.lateral) / denominator;
                    if (u < -1e-8 || u > 1 + 1e-8) continue;
                    const double offset = CrossXZ(p - s.center, segment) / denominator;
                    if (offset < travel - 1e-6) continue;
                    edge = std::min(edge, std::max(travel, offset - goreWidth - road.style.EdgeWidth()));
                }
            }
            return edge;
        };

        // Split the asphalt at the yellow line and, on graded bends, along the other lane boundaries.
        std::vector<double> pavementCuts;
        if (!variable && road.style.IsDivided()) pavementCuts.push_back(halfGap);
        if (!variable && hasGradedBend)
        {
            const double firstLane = lanes % 2 ? laneWidth * 0.5 : laneWidth;
            for (int i = 0; i < (lanes + 1) / 2; ++i)
            {
                const double offset = halfGap + firstLane + i * laneWidth;
                if (offset < pavement) pavementCuts.push_back(offset);
            }
        }
        // Reserve one U tile for the pair of inner shoulders. Each carriageway
        // then starts at an integer U, regardless of the width of the median.
        const double centerU = lanes * 0.5 + (road.style.IsDivided() && side < 0 ? 1.0 : 0.0);
        const auto pavementVertex = [&](const CrossSection& section, double offset)
        {
            const double gap = gapAt(section);
            const double tileWidth = offset < gap ? 2.0 * RoadStyle::innerShoulderWidth :
                variable ? path.profile->Mix(section.distance, [](const RoadStyle& s) { return s.TravelLaneWidth(); }) : laneWidth;
            const double uOrigin = variable ? path.profile->Mix(section.distance, [&](const RoadStyle& s)
                { return s.GetLaneCount() * 0.5 + (s.IsDivided() && side < 0 ? 1.0 : 0.0); }) : centerU;
            const double u = uOrigin - side * (offset - gap) / tileWidth;
            const double v = (section.distance + offset * section.bevelSlope) / RoadStyle::laneWidth;
            return Road::PavementVertex{section.At(offset), {static_cast<float>(u), static_cast<float>(v)}};
        };

        auto* strip = edge == RoadEdge::Transition ? &road.transitionStrips[road.style.transitionMaterial] : nullptr;
        const bool paintDivided = road.paintMarkings && road.style.IsDivided();
        std::vector<MarkingPath> dividers(paintDivided && !variable ? lanes / 2 - 1 : 0);
        std::vector<CrossSection> stations;
        VisitOffsetStations(path, side * (pavement + road.style.EdgeWidth()), [&](CrossSection station)
        {
            for (std::size_t i = 0; i < dividers.size(); ++i)
                dividers[i].Append(station.center, station.lateral, side * (halfGap + (i + 1) * laneWidth),
                                   !path.curveLeft.empty(), path.curveUp.empty() ? nullptr : &station.up);
            station.lateral = station.lateral * side;
            stations.push_back(station);
        });
        TrafficPaths(road, path, side, stations);
        const auto shoulderSamples = SampleShoulderOpenings(stations, openings, pavement + road.style.EdgeWidth());

        if (lanes >= 2 && !variable)
        {
            for (int lane = 0; lane < lanes; ++lane)
            {
                const double fromCenter = (lane + 0.5 - lanes * 0.5) * laneWidth;
                if ((fromCenter < 0 ? -1 : 1) != side) continue;
                const double offset = halfGap + std::abs(fromCenter);
                auto& line = road.laneCenterlines[lane];
                line.reserve(stations.size());
                const bool outer = lane == 0 || lane == lanes - 1;
                for (const auto& station : stations)
                {
                    const Vector3 point = station.At(offset);
                    if (!line.empty() && point == line.back()) continue;
                    line.push_back(point);
                    if (outer) road.outerLaneDistances[side > 0].push_back(station.distance);
                }
            }
        }

        const auto profile = [&](const CrossSection& s, double outer)
        {
            EdgeProfile rim{};
            rim[0] = s.At(outer);
            rim[1] = s.At(outer + RoadStyle::stripWidth);
            rim[2] = rim[1] + s.up * RoadStyle::curbHeight;
            rim[3] = rim[2] + s.lateral * RoadStyle::curbTopWidth;
            return rim;
        };
        std::vector<Vector3> boundary;
        EdgeProfile lastRim{};
        VisitTaperSpans(stations, tapers, shoulderSamples, [&](const CrossSection& a, const CrossSection& b, const Taper* taper,
                                              const CrossSection& spanStart, const CrossSection& spanEnd)
        {
            const double fa = taper ? taper->Fraction(a.distance) : 0;
            const double fb = taper ? taper->Fraction(b.distance) : 0;
            // Leave a painted nose between the two edge lines even on highways without shoulders.
            const double clearance = taper ? goreWidth + road.style.EdgeWidth() + taper->lane.edgeWidth : 0;
            const double ta = travelAt(a), tb = travelAt(b);
            const double shoulderA = pavementAt(a) - ta, shoulderB = pavementAt(b) - tb;
            const double pa = taper ? ta : shoulderEdge(a), pb = taper ? tb : shoulderEdge(b);
            const TaperProfile shapeA = taper ? taper->Profile(clearance) : TaperProfile{};
            const TaperProfile shapeB = taper ? taper->Profile(clearance) : TaperProfile{};
            const double outerA = ta + shapeA.Outer(fa), outerB = tb + shapeB.Outer(fb);
            const double innerA = ta + shapeA.Inner(fa, laneWidth), innerB = tb + shapeB.Inner(fb, laneWidth);
            const auto edgeWidthAt = [&](double f, double outer, double shoulder)
            {
                // The auxiliary lane consumes the highway shoulder as it opens.
                const double blend = shapeA.Opening(f);
                const double rampShoulder = taper ? taper->lane.outerPavement - taper->lane.halfTravel : shoulder;
                return outer + std::lerp(shoulder, rampShoulder, blend);
            };
            const double wa = taper ? edgeWidthAt(fa, outerA, shoulderA) : pa;
            const double wb = taper ? edgeWidthAt(fb, outerB, shoulderB) : pb;
            if (taper)
            {
                for (int lane = 0; lane < taper->lane.laneCount; ++lane)
                {
                    const double across = (lane + 0.5) / taper->lane.laneCount;
                    auto& line = road.laneCenterlines[taper->firstGuide + lane];
                    AppendLanePoint(line, a.At(std::lerp(innerA, outerA, across)));
                    AppendLanePoint(line, b.At(std::lerp(innerB, outerB, across)));
                }
            }
            if (taper && (fa == 1 || fb == 1) && b.distance > a.distance)
            {
                const auto& end = fa == 1 ? a : b;
                const auto& lane = taper->lane;
                const double extra = clearance + lane.innerPavement;
                const double offset = travelAt(end) + extra;
                // Differentiate the swept center of the departing lane, including grade,
                // the turn of the highway's cross section, and the taper's lateral slope.
                const double spanLength = spanEnd.distance - spanStart.distance;
                const Vector3 along = (spanEnd.At(travelAt(spanEnd) + extra) - spanStart.At(travelAt(spanStart) + extra)) * (1.0 / spanLength)
                                    + end.lateral * ((fa == 1 ? shapeA : shapeB).endSlope / (taper->gore - taper->tip));
                const int direction = road.style.TravelDirection(side);
                const double speed = Math::Length(along);
                const double perMeter = speed > 0.0 ? 1.0 / (spanLength * speed) : 0.0;
                const RoadMouthFrame cap{end.lateral * (side * direction), end.up,
                                         (spanEnd.lateral - spanStart.lateral) * (side * perMeter),
                                         (spanEnd.up - spanStart.up) * (direction * perMeter)};
                road.rampSeams[{lane.station, side}] =
                    {end.At(offset), Math::Normalize(along * direction), cap, std::abs(taper->gore - taper->tip),
                     taper->gore, taper->tip, taper->stationDistance, side * direction, lane.rampIndex};
            }
            const auto ra = profile(a, wa), rb = profile(b, wb);
            if (bridge)
            {
                auto& rim = road.bridgeRims[side > 0];
                if (rim.empty()) for (int i = 0; i < faces; ++i) rim.push_back({a.distance, ra[i]});
                if (rim.empty() || rim.back().position != ra[faces]) rim.push_back({a.distance, ra[faces]});
                rim.push_back({b.distance, rb[faces]});
            }
            if (boundary.empty() && road.points.front().deadEndLength == 0)
                boundary.insert(boundary.end(), ra.begin(), ra.begin() + faces);
            if (boundary.empty() || boundary.back() != ra[faces]) boundary.push_back(ra[faces]);
            boundary.push_back(rb[faces]);
            lastRim = rb;

            const double ma = medianAt(a), mb = medianAt(b);
            if (ma > 0.0 || mb > 0.0)
            {
                const auto vertex = [&](const CrossSection& s, double offset)
                {
                    const double grass = variable ? path.profile->Mix(s.distance,
                        [](const RoadStyle& style) { return style.median == RoadMedian::Grass; }) :
                        road.style.median == RoadMedian::Grass;
                    const SDL_FColor color{float(std::lerp(0.69, 0.29, grass)), float(std::lerp(0.69, 0.49, grass)),
                                           float(std::lerp(0.67, 0.24, grass)), 1};
                    return Road::EdgeVertex{s.At(offset), color};
                };
                VisitBand(side, a, b, 0.0, ma, 0.0, mb, vertex,
                    [&](const auto& t) { road.edgeTriangles.push_back(t); });
            }
            const auto asphalt = [&](const auto& t) { road.pavementTriangles.push_back(t); };
            double ia = ma, ib = mb;
            const auto cut = [&](double ca, double cb)
            {
                ca = std::clamp(ca, ia, pa);
                cb = std::clamp(cb, ib, pb);
                if (ca == ia && cb == ib) return;
                VisitBand(side, a, b, ia, ca, ib, cb, pavementVertex, asphalt);
                ia = ca;
                ib = cb;
            };
            if (variable)
            {
                cut(gapAt(a), gapAt(b));
                for (int lane = 0; lane <= (lanes + 1) / 2; ++lane)
                {
                    const auto offset = [&](const RoadStyle& s)
                    {
                        const double halfTravel = s.GetLaneCount() * s.TravelLaneWidth() * 0.5;
                        const double first = s.GetLaneCount() % 2 ? s.TravelLaneWidth() * 0.5 : 0.0;
                        return s.HalfSeparation() + std::min(halfTravel, first + lane * s.TravelLaneWidth());
                    };
                    cut(path.profile->Mix(a.distance, offset), path.profile->Mix(b.distance, offset));
                }
            }
            else for (double offset : pavementCuts) cut(offset, offset);
            VisitBand(side, a, b, ia, pa, ib, pb, pavementVertex, asphalt);

            // The narrow end has zero added width. Use the same diagonal/winding
            // convention as the neighboring bands and avoid degenerate triangles.
            const auto band = [&](double ai, double ao, double bi, double bo, auto vertex, auto emit)
            {
                VisitBand(side, a, b, ai, ao, bi, bo, vertex, emit);
            };
            if (taper)
            {
                // Split the cross section at both sides and the center of the ramp.
                // Its first pavement row can reuse all three positions exactly.
                double ai = pa, bi = pb;
                const double extra = clearance + taper->lane.innerPavement + taper->lane.outerPavement;
                for (double cut : {clearance, clearance + taper->lane.innerPavement, extra})
                {
                    const double ao = std::lerp(pa, wa, cut / extra);
                    const double bo = std::lerp(pb, wb, cut / extra);
                    band(ai, ao, bi, bo, pavementVertex, asphalt);
                    ai = ao;
                    bi = bo;
                }
            }

            if (paintDivided || (road.paintMarkings && (!tapers.empty() || variable)))
            {
                const Vector3 delta = b.center - a.center;
                const bool vertical = delta.x == 0 && delta.z == 0 && delta.y != 0;
                const auto paintUp = vertical ? Math::Cross(Math::Normalize(b.lateral) * side,
                                                             {0, delta.y > 0 ? 1.0 : -1.0, 0}) : Vector3{};
                const auto raised = [&](const CrossSection& s, double d) {
                    return s.At(d) + (vertical ? paintUp : s.up) * RoadStyle::stripeHeight;
                };
                const auto white = [&](const Road::Triangle& t) { road.whiteStripes.push_back(t); };
                if (paintDivided && !variable)
                    VisitBand(side, a, b, halfGap, halfGap + stripeWidth, raised,
                              [&](const Road::Triangle& t) { road.yellowStripes.push_back(t); });
                const auto edgePaint = [&](const Road::Triangle& t) {
                    (road.style.oneWay && side > 0 ? road.yellowStripes : road.whiteStripes).push_back(t);
                };
                const double mid = (fa + fb) * 0.5;
                const bool separate = mid >= fullWidthFraction;
                const bool solid = mid >= goreStartFraction;
                const bool dash = std::fmod((a.distance + b.distance) * 0.5, RoadStyle::dashPeriod) < dashLength;
                if (!taper || (separate && (solid || dash)))
                    band(ta - stripeWidth, ta, tb - stripeWidth, tb, raised, edgePaint);
                if (taper)
                {
                    band(outerA - stripeWidth, outerA,
                         outerB - stripeWidth, outerB, raised, edgePaint);
                    if (solid)
                    {
                        band(innerA, innerA + stripeWidth,
                             innerB, innerB + stripeWidth, raised, white);
                    }
                }
            }
            if (strip)
            {
                const auto stripVertex = [&](const CrossSection& s, double offset)
                {
                    const auto v = pavementVertex(s, offset);
                    const bool inside = offset == (&s == &a ? wa : wb);
                    return Road::TransitionVertex{v.position, inside ? RoadEdgeColor::transitionInner : RoadEdgeColor::transitionOuter, v.uv};
                };
                band(wa, wa + RoadStyle::stripWidth, wb, wb + RoadStyle::stripWidth, stripVertex,
                     [&](const auto& triangle) { strip->push_back(triangle); });
            }
            else if (edge == RoadEdge::Curb)
            {
                AddEdgeFace(road, side, ra, rb, 0, RoadEdgeColor::apron, RoadEdgeColor::apron);
                AddEdgeFace(road, side, ra, rb, 1, RoadEdgeColor::riser, RoadEdgeColor::riser);
                AddEdgeFace(road, side, ra, rb, 2, RoadEdgeColor::top, RoadEdgeColor::top);
            }
        });
        if (road.points.back().deadEndLength == 0)
            for (int i = faces - 1; i >= 0; --i) boundary.push_back(lastRim[i]);

        if (bridge && !stations.empty())
            for (int i = faces - 1; i >= 0; --i) road.bridgeRims[side > 0].push_back({stations.back().distance, lastRim[i]});
        AppendBoundary(road, side, boundary);
        if (variable)
        {
            auto& outer = road.outerLaneCenterlines[side > 0];
            auto& distances = road.outerLaneDistances[side > 0];
            for (const auto& station : stations)
            {
                const double offset = path.profile->Mix(station.distance, [](const RoadStyle& s)
                { return s.HalfSeparation() + (s.GetLaneCount() - 1) * s.TravelLaneWidth() * 0.5; });
                const auto point = station.At(offset);
                if (!outer.empty() && outer.back() == point) continue;
                outer.push_back(point);
                distances.push_back(station.distance);
            }
            VariableLaneMarkings(road, path, side, stations);
        }
        for (auto& divider : dividers)
        {
            divider.caps = path.caps;
            PaintDashes(divider, road.whiteStripes);
        }
    }

    // lo and hi are signed distances perpendicular to each span, in meters.
    void PaintStrip(const MarkingPath& path, double lo, double hi, std::vector<Road::Triangle>& mesh,
                    std::optional<Vector3> initialLeft)
    {
        Vector3 previousLo{}, previousHi{};
        bool havePrevious = false;

        VisitSpans(path.points, initialLeft, [&](const Span& span, std::size_t i)
        {
            if (!path.hiddenSpans.empty() && path.hiddenSpans[i])
            {
                havePrevious = false;
                return;
            }
            const auto& startCap = path.caps.front();
            const auto& endCap = path.caps.back();
            const bool atStart = i == 1 && startCap.has_value();
            const bool atEnd = i + 1 == path.points.size() && endCap.has_value();
            const Vector3 leftA = atStart ? startCap->left : (path.curveLeft.empty() ? span.left : path.curveLeft[i - 1]);
            const Vector3 leftB = atEnd ? endCap->left : (path.curveLeft.empty() ? span.left : path.curveLeft[i]);
            // A vertical stripe needs a horizontal depth offset from its pavement.
            const Vector3 up = span.length > 0.0 ? Vector3{0, 1, 0}
                : Math::Cross(leftA, {0, span.end.y > span.start.y ? 1.0 : -1.0, 0});
            const auto upA = atStart ? startCap->up : (path.curveUp.empty() ? up : path.curveUp[i - 1]);
            const auto upB = atEnd ? endCap->up : (path.curveUp.empty() ? up : path.curveUp[i]);
            const Vector3 a = span.start + upA * RoadStyle::stripeHeight;
            const Vector3 b = span.end + upB * RoadStyle::stripeHeight;
            const double wa = path.widths.empty() ? 1.0 : path.widths[i - 1];
            const double wb = path.widths.empty() ? 1.0 : path.widths[i];
            const Vector3 aLo = a + leftA * (lo * wa), aHi = a + leftA * (hi * wa);
            const Vector3 bLo = b + leftB * (lo * wb), bHi = b + leftB * (hi * wb);

            if (havePrevious && path.curveLeft.empty())
            {
                if (lo != 0.0)
                {
                    mesh.push_back({a, previousLo, aLo});
                }

                if (hi != 0.0)
                {
                    mesh.push_back({a, previousHi, aHi});
                }
            }

            if (wa > 0) mesh.push_back({aLo, aHi, bHi});
            if (wb > 0) mesh.push_back({aLo, bHi, bLo});
            previousLo = bLo;
            previousHi = bHi;
            havePrevious = true;
        });
    }

    void PaintDashes(const MarkingPath& path, std::vector<Road::Triangle>& mesh)
    {
        if (path.points.empty())
        {
            return;
        }

        // Carry the unfinished dash or gap into the next span of this divider.
        bool painting = true;
        double remaining = dashLength;
        MarkingPath dash;
        dash.points.push_back(path.points.front());
        if (!path.curveLeft.empty()) dash.curveLeft.push_back(path.curveLeft.front());
        if (!path.curveUp.empty()) dash.curveUp.push_back(path.curveUp.front());
        if (!path.widths.empty()) dash.widths.push_back(path.widths.front());
        if (!path.hiddenSpans.empty()) dash.hiddenSpans.push_back(false);

        Vector3 dashLeft = defaultLeft;
        const auto emitDash = [&]
        {
            if (dash.points.size() < 2) return;
            dash.caps[0] = dash.points.front() == path.points.front() ? path.caps[0] : std::nullopt;
            dash.caps[1] = dash.points.back() == path.points.back() ? path.caps[1] : std::nullopt;
            PaintStrip(dash, -stripeWidth * 0.5, stripeWidth * 0.5, mesh, dashLeft);
        };
        VisitSpans(path.points, {}, [&](const Span& span, std::size_t i)
        {
            const Vector3 start = span.start;
            const Vector3 delta = span.end - start;
            if (dash.points.size() == 1)
            {
                dashLeft = span.left;
            }
            const double length = Math::Length(delta);
            const auto addPoint = [&](Vector3 point, double fraction)
            {
                dash.points.push_back(point);
                if (!path.hiddenSpans.empty()) dash.hiddenSpans.push_back(path.hiddenSpans[i]);
                if (!path.widths.empty()) dash.widths.push_back(std::lerp(path.widths[i - 1], path.widths[i], fraction));
                if (!path.curveLeft.empty())
                {
                    // Interpolate both stripe edges when a dash ends between stations.
                    const Vector3 a = path.curveLeft[i - 1], b = path.curveLeft[i];
                    dash.curveLeft.push_back(fraction == 1.0 ? b : a + (b - a) * fraction);
                }
                if (!path.curveUp.empty())
                {
                    const Vector3 a = path.curveUp[i - 1], b = path.curveUp[i];
                    dash.curveUp.push_back(a + (b - a) * fraction);
                }
            };
            double used = 0.0;

            while (used < length)
            {
                const double available = length - used;

                if (remaining > available)
                {
                    if (painting)
                    {
                        addPoint(span.end, 1.0);
                    }
                    remaining -= available;
                    break;
                }

                used += remaining;
                const double fraction = used / length;
                const Vector3 boundary = used == length ? span.end : start + delta * fraction;

                if (painting)
                {
                    addPoint(boundary, fraction);
                    emitDash();
                    dash.points.clear();
                    dash.curveLeft.clear();
                    dash.curveUp.clear();
                    dash.widths.clear();
                    dash.hiddenSpans.clear();
                }
                else
                {
                    dashLeft = span.left;
                    addPoint(boundary, fraction);
                }

                painting = !painting;
                remaining = painting ? dashLength : gapLength;
            }
        });
        emitDash();
    }

    void PaintDoubleYellow(const RoadPath& path, double boundary, std::vector<Road::Triangle>& mesh)
    {
        for (int side : {-1, 1})
        {
            const auto stripe = OffsetPath(path, boundary + side * yellowPairOffset);
            PaintStrip(stripe, -stripeWidth * 0.5, stripeWidth * 0.5, mesh);
        }
    }
}

void Road::BuildSurface(const std::vector<Vector3>& samples, std::span<const Vector3> tangents,
                        const std::array<std::optional<RoadMouthFrame>, 2>& caps, std::size_t firstSample,
                        std::span<const Vector3> surfaceUps)
{
    RoadPath path;
    path.caps = caps;
    VisitSpans(samples, {}, [&](const Span& span, std::size_t end)
    {
        if (!surfaceUps.empty())
        {
            const auto station = [&](std::size_t i)
            {
                if (!tangents.empty())
                {
                    const auto fallback = path.curveLeft.empty() ? span.left : path.curveLeft.back();
                    path.curveLeft.push_back(CurveLeft(tangents[i], fallback, surfaceUps[i]));
                }
                path.curveUp.push_back(surfaceUps[i]);
            };
            if (path.spans.empty()) station(end - 1);
            station(end);
        }
        else if (!tangents.empty())
        {
            if (path.spans.empty()) path.curveLeft.push_back(CurveLeft(tangents[end - 1], span.left));
            path.curveLeft.push_back(CurveLeft(tangents[end], path.curveLeft.back()));
        }
        path.spans.push_back(span);
        path.distances.push_back(path.distances.back() + Math::Length(span.end - span.start));
    });
    if (!path.curveLeft.empty())
    {
        for (int end = 0; end < 2; ++end)
        {
            if (!rampCaps[end] || !caps[end]) continue;
            const auto& cap = *caps[end];
            if (path.curveUp.empty()) path.curveUp.resize(path.curveLeft.size(), {0, 1, 0});
            const double reach = std::min(path.distances.back() * 0.5, RampFrameBlendLength());
            // Continue the highway's sweep at the cut so each offset edge keeps its own
            // tangent on a bend. Ease into the branch's natural cross section farther out.
            for (size_t i = 0; i < path.curveLeft.size(); ++i)
            {
                const size_t j = end ? path.curveLeft.size() - 1 - i : i;
                const double dist = end ? path.distances.back() - path.distances[j] : path.distances[j];
                if (dist >= reach) break;
                const double t = dist / reach;
                const double blend = t * t * (3.0 - 2.0 * t);
                const double advance = end ? -dist : dist;
                const Vector3 left = cap.left + cap.leftPerMeter * advance;
                const Vector3 up = cap.up + cap.upPerMeter * advance;
                const Vector3 blendedLeft = left + (path.curveLeft[j] - left) * blend;
                // Interpolating differently oriented unit axes would pinch the ramp.
                // Interpolate orientation and cross-section scale separately.
                const double scale = std::lerp(Math::Length(left), 1.0, blend);
                path.curveLeft[j] = blendedLeft == Vector3{} ? path.curveLeft[j]
                    : Math::Normalize(blendedLeft) * std::max(1.0, scale);
                path.curveUp[j] = up + (path.curveUp[j] - up) * blend;
            }
        }
    }
    const auto& spans = path.spans;
    if (spans.empty())
    {
        return;
    }

    if (HasOverpasses())
        for (auto sample : pointSamples)
            bridgeKnots.push_back(path.distances[std::min(sample > firstSample ? sample - firstSample : 0, path.distances.size() - 1)]);
    const bool variable = HasProfileOverrides();
    if (variable)
    {
        auto& profile = path.profile.emplace();
        profile.road = this;
        profile.styles.reserve(points.size() + 1);
        for (std::size_t i = 0; i <= points.size(); ++i)
            profile.styles.push_back(SurfaceStyle(static_cast<int>(i)));
        for (auto sample : pointSamples)
            profile.knots.push_back(path.distances[std::min(sample > firstSample ? sample - firstSample : 0, path.distances.size() - 1)]);
        profile.knots.front() = 0;
        profile.knots.back() = path.distances.back();
        profileLengths.reserve(points.size() - 1);
        for (std::size_t i = 1; i < profile.knots.size(); ++i)
            profileLengths.push_back(profile.knots[i] - profile.knots[i - 1]);
    }
    const int laneCount = variable ? MaximumLaneCount() : style.GetLaneCount();
    if (laneCount >= 2 && !variable) laneCenterlines.resize(laneCount);
    const bool hasGradedBend = std::adjacent_find(spans.begin(), spans.end(), [](const Span& a, const Span& b)
    {
        return a.left != b.left && (a.start.y != a.end.y || b.start.y != b.end.y);
    }) != spans.end();
    const double halfTravelWidth = laneCount * style.TravelLaneWidth() * 0.5;
    std::size_t rightBoundaryCount = 0;

    for (int side : {-1, 1})
    {
        std::vector<Taper> tapers;
        for (const auto& lane : rampLanes)
        {
            if (laneCount < 2 || lane.side != side) continue;
            const auto knot = std::ranges::find(points, lane.station, &RoadPoint::rampStation);
            if (knot == points.end()) continue;
            const auto sample = pointSamples[knot - points.begin()];
            if (sample < firstSample || sample - firstSample >= path.distances.size()) continue;
            const double gore = std::clamp(path.distances[sample - firstSample] + lane.offset, 0.0, path.distances.back());
            const int direction = style.TravelDirection(side);
            const double tip = std::clamp(gore + (lane.exit ? -direction : direction) * lane.length, 0.0, path.distances.back());
            if (gore != tip) tapers.push_back({lane, tip, gore, path.distances[sample - firstSample]});
        }
        // If a later highway edit shortens the available run, keep both joins at
        // their knots. Only shorten the conflicting tapers, restoring them when space returns.
        std::ranges::sort(tapers, {}, &Taper::gore);
        for (size_t i = 1; i < tapers.size(); ++i)
        {
            auto& a = tapers[i - 1];
            auto& b = tapers[i];
            a.tip = std::min(a.tip, b.gore);
            b.tip = std::max(b.tip, a.gore);
            if (a.tip > a.gore && b.tip < b.gore && a.tip > b.tip)
            {
                const double fraction = a.lane.length / (a.lane.length + b.lane.length);
                a.tip = b.tip = std::lerp(a.gore, b.gore, fraction);
            }
        }
        for (auto& taper : tapers)
        {
            taper.firstGuide = laneCenterlines.size();
            laneCenterlines.resize(taper.firstGuide + taper.lane.laneCount);
        }
        BuildRoadSide(*this, path, side, hasGradedBend, tapers);
        if (side < 0) rightBoundaryCount = outline.size();
        if (paintMarkings && !style.IsDivided() && tapers.empty() && !variable)
        {
            // Place the white stripe wholly inside the travel edge, beside any shoulder.
            const auto edge = OffsetPath(path, side * halfTravelWidth);
            PaintStrip(edge, side > 0 ? -stripeWidth : 0.0, side > 0 ? 0.0 : stripeWidth,
                       style.oneWay && side > 0 ? yellowStripes : whiteStripes);
        }
    }
    const Vector3 firstTangent = tangents.empty() ? spans.front().end - spans.front().start : tangents.front();
    const Vector3 lastTangent = tangents.empty() ? spans.back().end - spans.back().start : tangents.back();
    if (const auto cap = BuildDeadEnd(*this, path, 1, lastTangent))
        outline.insert(outline.begin() + rightBoundaryCount, cap->begin(), cap->end());
    if (const auto cap = BuildDeadEnd(*this, path, 0, firstTangent))
        outline.insert(outline.end(), cap->rbegin(), cap->rend());
    if (!paintMarkings || style.IsDivided() || variable) return;

    if (style.oneWay)
    {
        for (int i = 1; i < laneCount; ++i)
            PaintDashes(OffsetPath(path, -halfTravelWidth + i * style.TravelLaneWidth()), whiteStripes);
        return;
    }
    constexpr double halfLane = RoadStyle::laneWidth * 0.5;
    const bool centerTurn = style.HasCenterTurnLane();

    if (centerTurn)
    {
        for (int side : {-1, 1})
        {
            const auto solid = OffsetPath(path, side * (halfLane + yellowPairOffset));
            PaintStrip(solid, -stripeWidth * 0.5, stripeWidth * 0.5, yellowStripes);
            const auto dashed = OffsetPath(path, side * (halfLane - yellowPairOffset));
            PaintDashes(dashed, yellowStripes);
        }
    }
    else if (laneCount == 3)
    {
        const double boundary = ThreeLaneDividerOffset();
        PaintDoubleYellow(path, boundary, yellowStripes);
        PaintDashes(OffsetPath(path, -boundary), whiteStripes);
        return;
    }
    else
    {
        if(laneCount != 1)
        {
            PaintDoubleYellow(path, 0.0, yellowStripes);
        }
    }

    // Start counting through lanes at the edge of the center turn lane, if present.
    const double throughLaneStart = centerTurn ? halfLane : 0.0;
    const int lanesPerDirection = laneCount / 2;

    for (int side : {-1, 1})
    {
        for (int lane = 1; lane < lanesPerDirection; ++lane)
        {
            const auto divider = OffsetPath(path, side * (throughLaneStart + lane * RoadStyle::laneWidth));
            PaintDashes(divider, whiteStripes);
        }
    }
}

void BuildConnectingRoadPaint(RoadGeometry& geometry, const RoadMouth& a, const RoadMouth& b,
                              RoadPaintProfile profileA, RoadPaintProfile profileB)
{
    if (!profileA.enabled || !profileB.enabled) return;
    // Enter A against its outward frame, leave B along its outward frame.
    profileA.divider = -profileA.divider;
    profileA.oneWay = -profileA.oneWay;
    struct Line { double offset; bool yellow, dashed; };
    const auto pattern = [](const RoadPaintProfile& p)
    {
        std::map<int, Line> lines;
        const double half = p.lanes * p.laneWidth * 0.5;
        for (int side : {-1, 1})
        {
            const int index = side > 0 ? 1 : 0;
            const bool yellowEdge = p.oneWay == side;
            lines[index + (yellowEdge ? 2 : 0)] =
                {side * (p.separation + half - stripeWidth * 0.5), yellowEdge, false};
            if (p.oneWay) continue;
            if (p.lanes > 1)
            {
                double offset = p.divider + side * yellowPairOffset;
                if (p.separation > 0) offset = side * (p.separation + stripeWidth * 0.5);
                else if (p.centerTurn) offset = side * (RoadStyle::laneWidth * 0.5 + yellowPairOffset);
                lines[10 + index] = {offset, true, false};
            }
            if (p.centerTurn)
                lines[20 + index] = {side * (RoadStyle::laneWidth * 0.5 - yellowPairOffset), true, true};
            if (p.lanes == 3 && !p.centerTurn)
            {
                if (side * p.divider < 0) lines[100 + index * 16] = {-p.divider, false, true};
            }
            else
            {
                const double inner = p.separation + (p.centerTurn ? RoadStyle::laneWidth * 0.5 : 0);
                for (int lane = 1; lane < p.lanes / 2; ++lane)
                    lines[100 + index * 16 + lane] = {side * (inner + lane * RoadStyle::laneWidth), false, true};
            }
        }
        if (p.oneWay)
            for (int lane = 1; lane < p.lanes; ++lane)
                lines[200 + lane] = {-half + lane * p.laneWidth, false, true};
        return lines;
    };
    const auto first = pattern(profileA), last = pattern(profileB);
    const double length = Math::Length(b.center - a.center);
    if (length < 1e-6) return;
    const Vector3 p0 = a.center, p1 = p0 - a.outward * (length / 3.0);
    const Vector3 p3 = b.center, p2 = p3 - b.outward * (length / 3.0);
    struct Section { Vector3 center, left; double blend; };
    std::vector<Section> sections;
    const int intervals = static_cast<int>(std::clamp(std::ceil(length * 2), 16.0, 4096.0));
    sections.reserve(intervals + 1);
    for (int i = 0; i <= intervals; ++i)
    {
        const double t = double(i) / intervals, u = 1 - t;
        const Vector3 center = p0 * (u*u*u) + p1 * (3*u*u*t) + p2 * (3*u*t*t) + p3 * (t*t*t);
        const Vector3 tangent = (p1 - p0) * (u*u) + (p2 - p1) * (2*u*t) + (p3 - p2) * (t*t);
        sections.push_back({center, Math::Normalize(Math::Cross(tangent, {0, 1, 0})), t*t*(3 - 2*t)});
    }
    auto combined = first;
    combined.insert(last.begin(), last.end());
    for (const auto& [key, line] : combined)
    {
        const auto begin = first.find(key), end = last.find(key);
        // Like road lane transitions, omit dashed white dividers while lane
        // configurations differ. Edge/center lines still meet both mouths exactly.
        if (line.dashed && !line.yellow && (profileA.lanes != profileB.lanes ||
            profileA.oneWay != profileB.oneWay || profileA.centerTurn != profileB.centerTurn ||
            profileA.divider != profileB.divider || profileA.separation != profileB.separation)) continue;
        const double offsetA = begin == first.end() ? end->second.offset : begin->second.offset;
        const double offsetB = end == last.end() ? begin->second.offset : end->second.offset;
        MarkingPath path;
        for (const auto& section : sections)
        {
            path.points.push_back(section.center + section.left * std::lerp(offsetA, offsetB, section.blend));
            path.curveLeft.push_back(section.left);
            path.widths.push_back(std::lerp(begin == first.end() ? 0.0 : 1.0, end == last.end() ? 0.0 : 1.0, section.blend));
        }
        auto& mesh = line.yellow ? geometry.yellowStripes : geometry.whiteStripes;
        if (line.dashed) PaintDashes(path, mesh);
        else PaintStrip(path, -stripeWidth * 0.5, stripeWidth * 0.5, mesh);
    }
}
