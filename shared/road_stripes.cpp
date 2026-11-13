#include "road.hpp"

#include <algorithm>
#include <cmath>
#include <optional>

namespace
{
    constexpr double stripeWidth = 0.16;
    constexpr double yellowGap = 0.16; // Clear space between the two yellow stripes.
    constexpr double yellowPairOffset = (yellowGap + stripeWidth) * 0.5;
    constexpr double goreWidth = 1.0;
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

    struct RoadPath
    {
        std::array<std::optional<RoadMouthFrame>, 2> caps;
        std::vector<Span> spans;
        std::vector<double> distances{0.0}; // Cumulative 3D centerline length at each station.
        std::vector<Vector3> curveLeft; // One cross-section axis per Bézier station.
        std::vector<Vector3> curveUp;   // Present when easing out of an attached ramp's cut plane.
    };

    Vector3 CurveLeft(Vector3 tangent, Vector3 fallback)
    {
        const double run = std::hypot(tangent.x, tangent.z);
        if (run <= 1e-8 * std::abs(tangent.y)) return fallback;
        return {-tangent.z / run, 0, tangent.x / run};
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
            endSection(0, {spans.front().start, path.curveLeft.front(), 0.0});
            for (std::size_t i = 0; i + 1 < spans.size(); ++i)
                visit(CrossSection{spans[i].end, path.curveLeft[i + 1], path.distances[i + 1], 0.0,
                                   path.curveUp.empty() ? Vector3{0, 1, 0} : path.curveUp[i + 1]});
            endSection(1, {spans.back().end, path.curveLeft.back(), path.distances.back()});
            return;
        }

        // On an inside bend the offset lines intersect before/after the corner.
        // Record that trim distance; a negative entry requests a pavement-style bevel.
        std::vector<double> trim(spans.size() + 1, 0.0);

        for (std::size_t i = 1; i < spans.size(); ++i)
        {
            const Vector3 a = spans[i - 1].left;
            const Vector3 b = spans[i].left;
            const double turn = a.x * b.z - a.z * b.x;
            const double divisor = 1.0 + Math::Dot(a, b);
            trim[i] = divisor > 0.0 ? offset * turn / divisor : -1.0;
        }

        endSection(0, {spans.front().start, spans.front().left, 0.0});

        for (std::size_t i = 1; i < spans.size(); ++i)
        {
            const Span& a = spans[i - 1];
            const Span& b = spans[i];

            // Keep a miter only if the trims from both ends fit on each span.
            if (trim[i] >= 0.0 && trim[i] + std::max(0.0, trim[i - 1]) <= a.length && trim[i] + std::max(0.0, trim[i + 1]) <= b.length)
            {
                const double divisor = 1.0 + Math::Dot(a.left, b.left);
                visit(CrossSection{b.start, (a.left + b.left) * (1.0 / divisor), path.distances[i]});
            }
            else
            {
                // Both sections of a bevel share a center. Give the outer edge its
                // chord length in V and leave room on short spans between two bevels.
                const double room = 0.25 * std::min(path.distances[i] - path.distances[i - 1],
                                                    path.distances[i + 1] - path.distances[i]);
                const double slope = offset == 0.0 ? 0.0 :
                    std::min(0.5 * Math::Length(a.left - b.left), room / std::abs(offset));
                visit(CrossSection{a.end, a.left, path.distances[i], -slope});
                visit(CrossSection{b.start, b.left, path.distances[i], slope});
            }
        }

        endSection(1, {spans.back().end, spans.back().left, path.distances.back()});
    }

    struct MarkingPath
    {
        std::array<std::optional<RoadMouthFrame>, 2> caps;
        std::vector<Vector3> points;
        std::vector<Vector3> curveLeft, curveUp;

        void Append(Vector3 center, Vector3 left, double offset, bool curved, const Vector3* up)
        {
            points.push_back(center + left * offset);
            if (curved) curveLeft.push_back(left);
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
        double inner, halfTravel;
        // Cubic Hermite offset: zero slope where the stripe branches off, and a
        // shared outward slope for both edges where the full-width ramp takes over.
        double endSlope; // Lateral meters per unit of normalized taper distance.
        static double Offset(double width, double t, double slope)
        {
            return t * t * (width * (3.0 - 2.0 * t) - slope * (1.0 - t));
        }
        double Outer(double f) const { return Offset(inner + 2.0 * halfTravel, f, endSlope); }
        double Inner(double f) const
        {
            constexpr double noseLength = 1.0 - goreStartFraction;
            const double t = std::clamp((f - goreStartFraction) / noseLength, 0.0, 1.0);
            return Offset(inner, t, endSlope * noseLength);
        }
    };

    struct Taper
    {
        RampLane lane;
        double tip, gore, stationDistance;
        size_t firstGuide = 0;
        double Fraction(double d) const { return std::clamp((d - tip) / (gore - tip), 0.0, 1.0); }

        TaperProfile profile{};

        void SetProfile(double shoulder, double clearance)
        {
            const double inner = shoulder + clearance + lane.innerPavement - lane.halfTravel;
            // Limit the Hermite end slope to keep even a short, wide gore monotonic.
            const double slope = std::min(inner + 2.0 * lane.halfTravel,
                                          2.0 * inner / (1.0 - goreStartFraction));
            profile = {inner, lane.halfTravel, slope};
        }
    };

    template<typename Visit>
    void VisitTaperSpans(const std::vector<CrossSection>& stations, const std::vector<Taper>& tapers, Visit visit)
    {
        for (size_t i = 1; i < stations.size(); ++i)
        {
            const auto& a = stations[i - 1];
            const auto& b = stations[i];
            if (tapers.empty()) { visit(a, b, nullptr, a, b); continue; }
            std::vector<double> cuts;
            for (const auto& taper : tapers)
            {
                // Resolve the curved width independently of host spline knots and paint dashes.
                // Each piece is a cubic and spans at most 1/32 of its parameter interval per facet.
                const double nose = std::lerp(taper.tip, taper.gore, goreStartFraction);
                const auto cutCurve = [&](double start, double end)
                {
                    if (std::max(start, end) <= a.distance || std::min(start, end) >= b.distance) return;
                    for (int j = 0; j <= 32; ++j)
                    {
                        const double d = std::lerp(start, end, j / 32.0);
                        if (d > a.distance && d < b.distance) cuts.push_back(d);
                    }
                };
                cutCurve(taper.tip, nose);
                cutCurve(nose, taper.gore);
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

    void AppendLanePoint(std::vector<Vector3>& line, Vector3 point)
    {
        if (line.empty() || line.back() != point) line.push_back(point);
    }

    // Use the same cross sections for pavement, its outline and any edge treatment.
    // Adjacent spans then meet at the same elevation along the entire join.
    void BuildRoadSide(Road& road, const RoadPath& path, int side, bool hasGradedBend, const std::vector<Taper>& tapers)
    {
        const RoadEdge edge = road.style.edge;
        const int faces = edge == RoadEdge::Curb ? 3 : (edge == RoadEdge::Transition ? 1 : 0);
        const double pavement = side < 0 ? road.style.RightWidth() : road.style.LeftWidth();
        const double halfGap = road.style.HalfSeparation();
        const double halfMedian = road.style.HalfMedianWidth();
        const int lanes = road.style.GetLaneCount();
        const double laneWidth = road.style.TravelLaneWidth();
        const double travelEdge = halfGap + lanes * laneWidth * 0.5;
        const SDL_FColor medianColor = road.style.median == RoadMedian::Grass
            ? SDL_FColor{0.29f, 0.49f, 0.24f, 1} : SDL_FColor{0.69f, 0.69f, 0.67f, 1};

        // Split the asphalt at the yellow line and, on graded bends, along the other lane boundaries.
        std::vector<double> pavementCuts;
        if (road.style.IsDivided()) pavementCuts.push_back(halfGap);
        if (hasGradedBend)
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
            const double tileWidth = offset < halfGap ? 2.0 * RoadStyle::innerShoulderWidth : laneWidth;
            const double u = centerU - side * (offset - halfGap) / tileWidth;
            const double v = (section.distance + offset * section.bevelSlope) / RoadStyle::laneWidth;
            return Road::PavementVertex{section.At(offset), {static_cast<float>(u), static_cast<float>(v)}};
        };

        auto* strip = edge == RoadEdge::Transition ? &road.transitionStrips[road.style.transitionMaterial] : nullptr;
        const bool paintDivided = road.paintMarkings && road.style.IsDivided();
        std::vector<MarkingPath> dividers(paintDivided ? lanes / 2 - 1 : 0);
        std::vector<CrossSection> stations;
        VisitOffsetStations(path, side * (pavement + road.style.EdgeWidth()), [&](CrossSection station)
        {
            for (std::size_t i = 0; i < dividers.size(); ++i)
                dividers[i].Append(station.center, station.lateral, side * (halfGap + (i + 1) * laneWidth),
                                   !path.curveLeft.empty(), path.curveUp.empty() ? nullptr : &station.up);
            station.lateral = station.lateral * side;
            stations.push_back(station);
        });

        if (lanes >= 2)
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
        VisitTaperSpans(stations, tapers, [&](const CrossSection& a, const CrossSection& b, const Taper* taper,
                                              const CrossSection& spanStart, const CrossSection& spanEnd)
        {
            const double fa = taper ? taper->Fraction(a.distance) : 0;
            const double fb = taper ? taper->Fraction(b.distance) : 0;
            // Leave a painted nose between the two edge lines even on highways without shoulders.
            const double clearance = taper ? goreWidth + road.style.EdgeWidth() + taper->lane.edgeWidth : 0;
            const double shoulder = pavement - travelEdge;
            const TaperProfile shape = taper ? taper->profile : TaperProfile{};
            const double outerA = travelEdge + shape.Outer(fa), outerB = travelEdge + shape.Outer(fb);
            const double innerA = travelEdge + shape.Inner(fa), innerB = travelEdge + shape.Inner(fb);
            const auto edgeWidthAt = [&](double f, double outer)
            {
                const double blend = f * f * (3.0 - 2.0 * f);
                const double rampShoulder = taper ? taper->lane.outerPavement - taper->lane.halfTravel : shoulder;
                return outer + std::lerp(shoulder, rampShoulder, blend);
            };
            const double wa = taper ? edgeWidthAt(fa, outerA) : pavement;
            const double wb = taper ? edgeWidthAt(fb, outerB) : pavement;
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
                const double offset = pavement + clearance + lane.innerPavement;
                // Differentiate the swept center of the departing lane, including grade,
                // the turn of the highway's cross section, and the taper's lateral slope.
                const double spanLength = spanEnd.distance - spanStart.distance;
                const Vector3 along = (spanEnd.At(offset) - spanStart.At(offset)) * (1.0 / spanLength)
                                    + end.lateral * (shape.endSlope / (taper->gore - taper->tip));
                const int direction = road.style.TravelDirection(side);
                const double speed = Math::Length(along);
                const double perMeter = speed > 0.0 ? 1.0 / (spanLength * speed) : 0.0;
                const RoadMouthFrame cap{end.lateral * (side * direction), end.up,
                                         (spanEnd.lateral - spanStart.lateral) * (side * perMeter),
                                         (spanEnd.up - spanStart.up) * (direction * perMeter)};
                road.rampSeams[{lane.station, side}] =
                    {end.At(offset), Math::Normalize(along * direction), cap, std::abs(taper->gore - taper->tip),
                     taper->gore, taper->tip, taper->stationDistance, side * direction};
            }
            const auto ra = profile(a, wa), rb = profile(b, wb);
            if (boundary.empty()) boundary.insert(boundary.end(), ra.begin(), ra.begin() + faces);
            if (boundary.empty() || boundary.back() != ra[faces]) boundary.push_back(ra[faces]);
            boundary.push_back(rb[faces]);
            lastRim = rb;

            if (halfMedian > 0.0)
            {
                VisitBand(side, a, b, 0.0, halfMedian, BandPosition, [&](const Road::Triangle& t)
                {
                    road.edgeTriangles.push_back({Road::EdgeVertex{t[0], medianColor},
                                                  Road::EdgeVertex{t[1], medianColor},
                                                  Road::EdgeVertex{t[2], medianColor}});
                });
            }
            const auto asphalt = [&](const auto& t) { road.pavementTriangles.push_back(t); };
            double inner = halfMedian;
            for (double cut : pavementCuts)
            {
                VisitBand(side, a, b, inner, cut, pavementVertex, asphalt);
                inner = cut;
            }
            VisitBand(side, a, b, inner, pavement, pavementVertex, asphalt);

            // The narrow end has zero added width. Use the same diagonal/winding
            // convention as the neighboring bands and avoid degenerate triangles.
            const auto band = [&](double ai, double ao, double bi, double bo, auto vertex, auto emit)
            {
                const auto p = vertex(a, ai), q = vertex(a, ao), r = vertex(b, bi), s = vertex(b, bo);
                if (ai != ao) emit(side > 0 ? std::array{q, r, p} : std::array{p, r, q});
                if (bi != bo) emit(side > 0 ? std::array{q, s, r} : std::array{q, r, s});
            };
            if (taper)
            {
                // Split the cross section at both sides and the center of the ramp.
                // Its first pavement row can reuse all three positions exactly.
                double ai = pavement, bi = pavement;
                const double extra = clearance + taper->lane.innerPavement + taper->lane.outerPavement;
                for (double cut : {clearance, clearance + taper->lane.innerPavement, extra})
                {
                    const double ao = std::lerp(pavement, wa, cut / extra);
                    const double bo = std::lerp(pavement, wb, cut / extra);
                    band(ai, ao, bi, bo, pavementVertex, asphalt);
                    ai = ao;
                    bi = bo;
                }
            }

            if (paintDivided || (road.paintMarkings && !tapers.empty()))
            {
                const Vector3 delta = b.center - a.center;
                const bool vertical = delta.x == 0 && delta.z == 0 && delta.y != 0;
                const auto paintUp = vertical ? Math::Cross(Math::Normalize(b.lateral) * side,
                                                             {0, delta.y > 0 ? 1.0 : -1.0, 0}) : Vector3{};
                const auto raised = [&](const CrossSection& s, double d) {
                    return s.At(d) + (vertical ? paintUp : s.up) * RoadStyle::stripeHeight;
                };
                const auto white = [&](const Road::Triangle& t) { road.whiteStripes.push_back(t); };
                if (paintDivided)
                    VisitBand(side, a, b, halfGap, halfGap + stripeWidth, raised,
                              [&](const Road::Triangle& t) { road.yellowStripes.push_back(t); });
                const auto edgePaint = [&](const Road::Triangle& t) {
                    (road.style.oneWay && side > 0 ? road.yellowStripes : road.whiteStripes).push_back(t);
                };
                const double mid = (fa + fb) * 0.5;
                if (!taper || mid >= goreStartFraction || std::fmod((a.distance + b.distance) * 0.5, RoadStyle::dashPeriod) < dashLength)
                    VisitBand(side, a, b, travelEdge - stripeWidth, travelEdge, raised, edgePaint);
                if (taper)
                {
                    band(outerA - stripeWidth, outerA,
                         outerB - stripeWidth, outerB, raised, edgePaint);
                    if (mid >= goreStartFraction)
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
        for (int i = faces - 1; i >= 0; --i) boundary.push_back(lastRim[i]);

        AppendBoundary(road, side, boundary);
        for (auto& divider : dividers)
        {
            divider.caps = path.caps;
            PaintDashes(divider, road.whiteStripes);
        }
    }

    // lo and hi are signed distances perpendicular to each span, in meters.
    void PaintStrip(const MarkingPath& path, double lo, double hi, std::vector<Road::Triangle>& mesh,
                    std::optional<Vector3> initialLeft = {})
    {
        Vector3 previousLo{}, previousHi{};
        bool havePrevious = false;

        VisitSpans(path.points, initialLeft, [&](const Span& span, std::size_t i)
        {
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
            const Vector3 aLo = a + leftA * lo, aHi = a + leftA * hi;
            const Vector3 bLo = b + leftB * lo, bHi = b + leftB * hi;

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

            mesh.push_back({aLo, aHi, bHi});
            mesh.push_back({aLo, bHi, bLo});
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
                        const std::array<std::optional<RoadMouthFrame>, 2>& caps, std::size_t firstSample)
{
    RoadPath path;
    path.caps = caps;
    VisitSpans(samples, {}, [&](const Span& span, std::size_t end)
    {
        if (!tangents.empty())
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
                path.curveLeft[j] = left + (path.curveLeft[j] - left) * blend;
                path.curveUp[j] = up + (Vector3{0, 1, 0} - up) * blend;
            }
        }
    }
    const auto& spans = path.spans;
    if (spans.empty())
    {
        return;
    }

    const int laneCount = style.GetLaneCount();
    if (laneCount >= 2) laneCenterlines.resize(laneCount);
    const bool hasGradedBend = std::adjacent_find(spans.begin(), spans.end(), [](const Span& a, const Span& b)
    {
        return a.left != b.left && (a.start.y != a.end.y || b.start.y != b.end.y);
    }) != spans.end();
    const double halfTravelWidth = laneCount * style.TravelLaneWidth() * 0.5;

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
            const double pavement = side < 0 ? style.RightWidth() : style.LeftWidth();
            taper.SetProfile(pavement - style.HalfSeparation() - halfTravelWidth,
                             goreWidth + style.EdgeWidth() + taper.lane.edgeWidth);
            taper.firstGuide = laneCenterlines.size();
            laneCenterlines.resize(taper.firstGuide + taper.lane.laneCount);
        }
        BuildRoadSide(*this, path, side, hasGradedBend, tapers);
        if (paintMarkings && !style.IsDivided() && tapers.empty())
        {
            // Place the white stripe wholly inside the travel edge, beside any shoulder.
            const auto edge = OffsetPath(path, side * halfTravelWidth);
            PaintStrip(edge, side > 0 ? -stripeWidth : 0.0, side > 0 ? 0.0 : stripeWidth,
                       style.oneWay && side > 0 ? yellowStripes : whiteStripes);
        }
    }
    if (!paintMarkings || style.IsDivided()) return;

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
