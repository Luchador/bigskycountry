#include "railroad.hpp"
#include "road.hpp"

#include <algorithm>
#include <cmath>

namespace
{
    constexpr double headWidth = RailroadSettings::headWidth, footWidth = 0.145, webWidth = 0.018;
    constexpr double footHeight = 0.02, headBottom = 0.12;
    constexpr double tieWidth = 0.23;
    constexpr SDL_FColor steel{0.51f, 0.48f, 0.45f, 1}, polished{0.76f, 0.77f, 0.78f, 1};
    constexpr SDL_FColor ballast{0.43f, 0.42f, 0.39f, 1};

    struct Section
    {
        MeshTransform axes;
        double distance;
        std::size_t sampleIndex;

        Vector3 Point(double lateral, double height) const
        {
            return axes.origin + axes.z * lateral + axes.y * height;
        }
    };

    MeshTransform TrackFrame(Vector3 center, Vector3 forward, const MeshTransform& reference)
    {
        auto side = Math::Cross(forward, reference.y);
        if (Math::Length(side) < 1e-8)
            side = reference.z - forward * Math::Dot(reference.z, forward);
        side = Math::Normalize(side);
        return {center, forward, Math::Cross(side, forward), side};
    }

    void SolidQuad(RoadGeometry::ColoredSurface& surface, Vector3 a, Vector3 b, Vector3 c, Vector3 d, SDL_FColor color)
    {
        const RoadGeometry::EdgeVertex va{a, color}, vb{b, color}, vc{c, color}, vd{d, color};
        if (a != b && b != c && a != c) surface.push_back({va, vb, vc});
        if (a != c && c != d && a != d) surface.push_back({va, vc, vd});
    }

    // Follow the outside contour of the foot, web and head. The gauge is set by
    // the head's inner face on each side of the track.
    struct ProfilePoint { double lateral, height; };
    constexpr ProfilePoint railProfile[] = {
        {-footWidth / 2, 0}, {-footWidth / 2, footHeight}, {-webWidth / 2, footHeight},
        {-webWidth / 2, headBottom}, {-headWidth / 2, headBottom}, {-headWidth / 2, RailroadSettings::railHeight},
        {headWidth / 2, RailroadSettings::railHeight}, {headWidth / 2, headBottom}, {webWidth / 2, headBottom},
        {webWidth / 2, footHeight}, {footWidth / 2, footHeight}, {footWidth / 2, 0}
    };

    void Rail(RoadGeometry::ColoredSurface& surface, const Section& a, const Section& b, double center, double base)
    {
        for (std::size_t i = 0; i < std::size(railProfile); ++i)
        {
            const auto p = railProfile[i], q = railProfile[(i + 1) % std::size(railProfile)];
            SolidQuad(surface, a.Point(center + p.lateral, base + p.height), a.Point(center + q.lateral, base + q.height),
                      b.Point(center + q.lateral, base + q.height), b.Point(center + p.lateral, base + p.height),
                      p.height == RailroadSettings::railHeight && q.height == p.height ? polished : steel);
        }
    }

    void EndFace(RoadGeometry::ColoredSurface& surface, const Section& section, ProfilePoint left, ProfilePoint right,
                 double bottom, SDL_FColor color, bool first)
    {
        auto a = section.Point(left.lateral, bottom), b = section.Point(right.lateral, bottom);
        auto c = section.Point(right.lateral, right.height), d = section.Point(left.lateral, left.height);
        if (first) SolidQuad(surface, a, b, c, d, color);
        else SolidQuad(surface, d, c, b, a, color);
    }

    void BedBand(RoadGeometry& mesh, const Section& a, const Section& b, ProfilePoint left, ProfilePoint right, double expandA, double expandB)
    {
        const auto vertex = [](const Section& s, ProfilePoint p)
        {
            return RoadGeometry::PavementVertex{s.Point(p.lateral, p.height),
                    {static_cast<float>(p.lateral / 4), static_cast<float>(s.distance / 4)}};
        };
        const auto expand = [](ProfilePoint p, double width) { p.lateral += p.lateral < 0 ? -width : width; return p; };
        const auto v0 = vertex(a, expand(left, expandA)), v1 = vertex(a, expand(right, expandA)),
                   v2 = vertex(b, expand(right, expandB)), v3 = vertex(b, expand(left, expandB));
        mesh.pavementTriangles.push_back({v0, v1, v2});
        mesh.pavementTriangles.push_back({v0, v2, v3});
    }

    void Transition(RoadGeometry& mesh, const Section& a, const Section& b, double toe, double rise, int side,
                    const std::string& material, double expandA, double expandB)
    {
        const auto vertex = [&](const Section& s, bool outer, double expand)
        {
            const double offset = side * (toe + expand + (outer ? RoadStyle::stripWidth : 0.0));
            return RoadGeometry::TransitionVertex{s.Point(offset, outer ? 0.0 : rise),
                       outer ? RoadEdgeColor::transitionOuter : RoadEdgeColor::transitionInner,
                       {static_cast<float>(offset / 4), static_cast<float>(s.distance / 4)}};
        };
        const auto v0 = vertex(a, side < 0, expandA), v1 = vertex(a, side > 0, expandA),
                   v2 = vertex(b, side > 0, expandB), v3 = vertex(b, side < 0, expandB);
        auto& triangles = mesh.transitionStrips[material];
        triangles.push_back({v0, v1, v2});
        triangles.push_back({v0, v2, v3});
    }
}

void RoadGeometry::RailroadSegment::AddTrackSample(Vector3 p)
{
    if (track.empty()) lower = upper = p;
    else
    {
        lower = {std::min(lower.x, p.x), std::min(lower.y, p.y), std::min(lower.z, p.z)};
        upper = {std::max(upper.x, p.x), std::max(upper.y, p.y), std::max(upper.z, p.z)};
    }
    track.push_back(p);
}

bool RoadGeometry::RailroadSegment::WithinDistance(Vector3 eye, double meters) const
{
    if (meters <= 0 || track.empty()) return false;
    const double limit = meters * meters;
    const Vector3 nearest{std::clamp(eye.x, lower.x, upper.x), std::clamp(eye.y, lower.y, upper.y),
                          std::clamp(eye.z, lower.z, upper.z)};
    const Vector3 outside = eye - nearest;
    if (Math::Dot(outside, outside) > limit) return false;

    for (std::size_t i = 1; i < track.size(); ++i)
    {
        const Vector3 along = track[i] - track[i - 1], offset = eye - track[i - 1];
        const double lengthSquared = Math::Dot(along, along);
        const double t = lengthSquared > 0 ? std::clamp(Math::Dot(offset, along) / lengthSquared, 0.0, 1.0) : 0.0;
        const Vector3 delta = offset - along * t;
        if (Math::Dot(delta, delta) <= limit) return true;
    }
    return false;
}

bool RailroadSettings::Valid() const
{
    return tracks >= 1 && tracks <= maxTracks && std::isfinite(width) && width >= tieLength && std::isfinite(height) && height >= 0;
}

double RailroadTieRun::Radius() const
{
    return Math::Length(last.origin - first.origin) * 0.5 + RailroadSettings::tieLength *
        std::max({1.0, Math::Length(first.z), Math::Length(last.z)});
}

MeshTransform RailroadTieRun::At(int index) const
{
    if (index == 0) return first;
    if (index + 1 == count) return last;
    const double t = static_cast<double>(index) / (count - 1);
    auto forward = Math::Normalize(first.x * (1 - t) + last.x * t);
    if (forward == Vector3{}) forward = first.x;
    const auto up = first.y * (1 - t) + last.y * t;
    MeshTransform reference = first;
    reference.y = up;
    auto result = TrackFrame(first.origin + (last.origin - first.origin) * t, forward, reference);
    result.z = result.z * std::lerp(Math::Length(first.z), Math::Length(last.z), t);
    return result;
}

const std::shared_ptr<const Mesh>& RailroadTieMesh()
{
    static const std::shared_ptr<const Mesh> tie = []
    {
        auto mesh = std::make_shared<Mesh>();
        const auto face = [&](Vector3 a, Vector3 b, Vector3 c, Vector3 d)
        {
            const auto normal = Math::Normalize(Math::Cross(b - a, c - a));
            const auto first = static_cast<std::uint32_t>(mesh->vertices.size());
            for (auto p : {a, b, c, d}) mesh->vertices.push_back({p, normal, {}, {0.105f, 0.058f, 0.033f, 1}});
            for (unsigned i : {0u, 1u, 2u, 0u, 2u, 3u}) mesh->indices.push_back(first + i);
        };
        const double x = tieWidth / 2, y = RailroadSettings::tieHeight, z = RailroadSettings::tieLength / 2;
        face({-x, y, -z}, {-x, y, z}, {x, y, z}, {x, y, -z});
        face({-x, 0, z}, {-x, 0, -z}, {x, 0, -z}, {x, 0, z});
        face({-x, 0, -z}, {-x, 0, z}, {-x, y, z}, {-x, y, -z});
        face({x, 0, z}, {x, 0, -z}, {x, y, -z}, {x, y, z});
        face({x, 0, -z}, {-x, 0, -z}, {-x, y, -z}, {x, y, -z});
        face({-x, 0, z}, {x, 0, z}, {x, y, z}, {-x, y, z});
        return mesh;
    }();
    return tie;
}

void BuildRailroad(Road& mesh, const RailroadSettings& settings, std::span<const Vector3> samples,
                   std::span<const Vector3> tangents, std::span<const std::size_t> controlSamples,
                   const MeshTransform& frame, bool transition,
                   const std::string& material)
{
    if (samples.size() < 2 || controlSamples.size() < 2 || !settings.Valid()) return;
    std::vector<Section> sections;
    for (std::size_t i = 0; i < samples.size(); ++i)
    {
        const double step = sections.empty() ? 0 : Math::Length(samples[i] - sections.back().axes.origin);
        if (!sections.empty() && step < 1e-8) continue;
        Vector3 direction = tangents.empty() ? Vector3{} : Math::Normalize(tangents[i]);
        if (direction == Vector3{})
        {
            const auto next = i + 1 < samples.size() ? Math::Normalize(samples[i + 1] - samples[i]) : Vector3{};
            const auto previous = sections.empty() ? Vector3{} : Math::Normalize(samples[i] - sections.back().axes.origin);
            direction = Math::Normalize(next + previous);
            if (direction == Vector3{}) direction = next == Vector3{} ? frame.x : next;
        }
        sections.push_back({TrackFrame(samples[i], direction, frame), sections.empty() ? 0 : sections.back().distance + step, i});
    }
    if (sections.size() < 2) return;

    // Insert exact profile midpoints so a change of track correspondence cannot
    // occur inside one rail span. Keep tie stationing on the original spline.
    std::vector<double> knots;
    std::size_t cursor = 0;
    for (auto sample : controlSamples)
    {
        while (cursor + 1 < sections.size() && sections[cursor + 1].sampleIndex <= sample) ++cursor;
        knots.push_back(sections[cursor].distance);
    }
    const auto location = [&](double distance)
    {
        const auto next = std::upper_bound(knots.begin(), knots.end(), distance);
        const auto segment = std::clamp<std::size_t>(next - knots.begin(), 1, knots.size() - 1) - 1;
        const double length = knots[segment + 1] - knots[segment];
        return std::pair{segment, length > 0 ? (distance - knots[segment]) / length : 0.0};
    };
    const auto blendAt = [&](double distance)
    {
        const auto [segment, t] = location(distance);
        return mesh.SegmentLaneBlend(segment, t);
    };
    const auto countAt = [&](double distance)
    {
        const auto blend = blendAt(distance);
        return std::lerp(double(blend.from), double(blend.to), blend.mix);
    };
    const auto expansion = [&](double distance) { return (countAt(distance) - 1) * RailroadSettings::trackSpacing * 0.5; };
    if (mesh.HasLaneOverrides())
    {
        const auto original = sections;
        for (std::size_t k = 1; k < knots.size(); ++k)
        {
            const double at = (knots[k - 1] + knots[k]) * 0.5;
            const auto found = std::lower_bound(original.begin(), original.end(), at,
                [](const Section& s, double d) { return s.distance < d; });
            if (found == original.begin() || found == original.end() || std::abs(found->distance - at) < 1e-8) continue;
            const auto& a = *(found - 1); const auto& b = *found;
            const double t = (at - a.distance) / (b.distance - a.distance);
            sections.push_back({TrackFrame(a.axes.origin * (1 - t) + b.axes.origin * t,
                Math::Normalize(a.axes.x * (1 - t) + b.axes.x * t), frame), at, a.sampleIndex});
        }
        std::ranges::sort(sections, {}, &Section::distance);
    }

    const double crown = settings.width / 2, height = settings.height, run = height * 2;
    const double toe = settings.ToeHalfWidth(1), toeHeight = transition ? std::min(0.035, height * 0.1) : 0;
    std::vector<ProfilePoint> bed;
    // A smoothstep slope meets the crest and the toe with level tangents.
    constexpr int intervals = 6;
    const auto slope = [&](int i, int side)
    {
        const double t = static_cast<double>(i) / intervals;
        return ProfilePoint{side * (crown + run * t), std::lerp(height, toeHeight, t * t * (3 - 2 * t))};
    };
    if (height > 0)
        for (int i = intervals; i > 0; --i) bed.push_back(slope(i, -1));
    bed.push_back({-crown, height});
    bed.push_back({crown, height});
    if (height > 0)
        for (int i = 1; i <= intervals; ++i) bed.push_back(slope(i, 1));

    const double railBase = height + RailroadSettings::tieHeight;
    const double railCenter = (RailroadSettings::gauge + headWidth) / 2;
    const double outerWidth = toe + (transition ? RoadStyle::stripWidth : 0);
    if (mesh.HasOverpasses())
    {
        // Reuse the road bridge builder with the railway's actual toe/transition
        // rim. Track geometry and sleeper stationing are independent of bridges.
        mesh.bridgeKnots = knots;
        for (const auto& s : sections)
            for (int side : {-1, 1})
                mesh.bridgeRims[side > 0].push_back({s.distance, s.Point(side * (outerWidth + expansion(s.distance)), 0)});
    }
    for (const auto& s : sections)
        mesh.trackSections.push_back({s.Point(0, railBase + RailroadSettings::railHeight), s.axes.z});
    for (const auto& s : sections) mesh.outline.push_back(s.Point(-outerWidth - expansion(s.distance), 0));
    for (auto it = sections.rbegin(); it != sections.rend(); ++it) mesh.outline.push_back(it->Point(outerWidth + expansion(it->distance), 0));

    for (bool first : {true, false})
    {
        const auto& end = first ? sections.front() : sections.back();
        // The bridge builder closes these ends against the complete bed profile
        // and ground seam. Do not leave a coincident ballast cap over its abutment.
        const bool bridgeEnd = mesh.points[first ? 0 : mesh.points.size() - 2].bridge.has_value();
        if (height > 0 && !bridgeEnd)
        {
            auto expanded = bed;
            const double extra = expansion(end.distance);
            for (auto& p : expanded) p.lateral += p.lateral < 0 ? -extra : extra;
            for (std::size_t i = 1; i < expanded.size(); ++i) EndFace(mesh.edgeTriangles, end, expanded[i - 1], expanded[i], 0, ballast, first);
            if (transition)
            {
                EndFace(mesh.edgeTriangles, end, {-outerWidth - extra, 0}, expanded.front(), 0, ballast, first);
                EndFace(mesh.edgeTriangles, end, expanded.back(), {outerWidth + extra, 0}, 0, ballast, first);
            }
        }
    }

    const auto railCaps = [&](RoadGeometry::ColoredSurface& rails, const Section& end, bool first)
    {
        for (const double center : {-railCenter, railCenter})
        {
            const auto cap = [&](double width, double low, double high)
            {
                EndFace(rails, end, {center - width / 2, RailroadSettings::tieHeight + high}, {center + width / 2, RailroadSettings::tieHeight + high}, RailroadSettings::tieHeight + low, steel, first);
            };
            cap(footWidth, 0, footHeight);
            cap(webWidth, footHeight, headBottom);
            cap(headWidth, headBottom, RailroadSettings::railHeight);
        }
    };

    // Offset each track, then orient its cross section along that track's own
    // tangent. This preserves standard gauge through a fan, rather than shearing
    // two rails sideways in the common embankment frame.
    std::vector<Vector3> sideRates(sections.size()), upRates(sections.size());
    for (std::size_t i = 0; i < sections.size(); ++i)
    {
        const auto lo = i ? i - 1 : i, hi = std::min(i + 1, sections.size() - 1);
        const double length = sections[hi].distance - sections[lo].distance;
        if (length > 0)
        {
            sideRates[i] = (sections[hi].axes.z - sections[lo].axes.z) * (1 / length);
            upRates[i] = (sections[hi].axes.y - sections[lo].axes.y) * (1 / length);
        }
    }
    const auto offset = [&](double at, Road::LaneBlend blend, int track)
    {
        const int count = std::max(blend.from, blend.to);
        const auto position = [&](int n)
        {
            const int index = count > 1 ? static_cast<int>(std::lround(double(track) * (n - 1) / (count - 1))) : 0;
            return (index - (n - 1) * 0.5) * RailroadSettings::trackSpacing;
        };
        const double t = blend.from != blend.to ? std::clamp((countAt(at) - blend.from) / (blend.to - blend.from), 0.0, 1.0) : 0;
        return std::lerp(position(blend.from), position(blend.to), t);
    };
    const auto trackSection = [&](std::size_t i, Road::LaneBlend blend, int track)
    {
        const auto& s = sections[i];
        const double lateral = offset(s.distance, blend, track);
        const double lo = std::max(0.0, s.distance - 0.001), hi = std::min(sections.back().distance, s.distance + 0.001);
        const double slope = hi > lo ? (offset(hi, blend, track) - offset(lo, blend, track)) / (hi - lo) : 0;
        const auto forward = Math::Normalize(s.axes.x + sideRates[i] * lateral + s.axes.z * slope + upRates[i] * height);
        return Section{TrackFrame(s.Point(lateral, height), forward, frame), s.distance, s.sampleIndex};
    };
    const auto interpolate = [&](const Section& a, const Section& b, double at)
    {
        const double t = (at - a.distance) / (b.distance - a.distance);
        return TrackFrame(a.axes.origin * (1 - t) + b.axes.origin * t,
            Math::Normalize(a.axes.x * (1 - t) + b.axes.x * t), frame);
    };
    std::size_t previous = knots.size();
    for (std::size_t i = 1; i < sections.size(); ++i)
    {
        const auto& a = sections[i - 1]; const auto& b = sections[i];
        const auto [segment, fraction] = location((a.distance + b.distance) * 0.5);
        if (previous != segment) { mesh.railroadSegments.emplace_back(); previous = segment; }
        auto& detail = mesh.railroadSegments.back();
        const double ea = expansion(a.distance), eb = expansion(b.distance);
        for (std::size_t j = 1; j < bed.size(); ++j) BedBand(mesh, a, b, bed[j - 1], bed[j], ea, eb);
        if (transition) for (int side : {-1, 1}) Transition(mesh, a, b, toe, toeHeight, side, material, ea, eb);
        const auto blend = mesh.SegmentLaneBlend(segment, fraction);
        const int count = std::max(blend.from, blend.to);
        std::vector<std::pair<Section, Section>> tracks;
        for (int track = 0; track < count; ++track)
        {
            const auto ta = trackSection(i - 1, blend, track), tb = trackSection(i, blend, track);
            if (!tracks.empty() && Math::Length(ta.axes.origin - tracks.back().first.axes.origin) < 1e-8 &&
                Math::Length(tb.axes.origin - tracks.back().second.axes.origin) < 1e-8) continue;
            tracks.emplace_back(ta, tb);
            detail.AddTrackSample(ta.axes.origin); detail.AddTrackSample(tb.axes.origin);
            for (int side : {-1, 1}) Rail(detail.rails, ta, tb, side * railCenter, RailroadSettings::tieHeight);
            if (i == 1) railCaps(detail.rails, ta, true);
            if (i + 1 == sections.size()) railCaps(detail.rails, tb, false);
            const double top = RailroadSettings::tieHeight + RailroadSettings::railHeight;
            mesh.trackSpans.push_back({TrackSection{ta.Point(0, top), ta.axes.z}, TrackSection{tb.Point(0, top), tb.axes.z}});
        }
        // At a merge, overlapping sleepers become a shared timber. Farther out
        // each track has its own ties. Batch consecutive equal groups as before.
        std::map<std::pair<int, int>, std::size_t> runs;
        double at = RailroadSettings::tieSpacing * (std::ceil((a.distance - RailroadSettings::tieSpacing * 0.5) /
            RailroadSettings::tieSpacing - 1e-9) + 0.5);
        const double limit = std::min(b.distance, sections.back().distance - tieWidth / 2);
        if (blend.from == blend.to)
        {
            // Long, constant-count spans remain O(instanced runs), not O(ties).
            const int countTies = std::max(0, static_cast<int>(std::ceil((limit - at - 1e-9) / RailroadSettings::tieSpacing)));
            for (const auto& track : tracks)
                for (int first = 0; first < countTies; first += RailroadTieRun::capacity)
                {
                    const int count = std::min(RailroadTieRun::capacity, countTies - first);
                    detail.ties.push_back({interpolate(track.first, track.second, at + first * RailroadSettings::tieSpacing),
                        interpolate(track.first, track.second, at + (first + count - 1) * RailroadSettings::tieSpacing), count});
                }
            continue;
        }
        for (; at < limit - 1e-9; at += RailroadSettings::tieSpacing)
        {
            for (int first = 0; first < static_cast<int>(tracks.size());)
            {
                int last = first;
                auto tf = interpolate(tracks[first].first, tracks[first].second, at);
                auto end = tf;
                while (last + 1 < static_cast<int>(tracks.size()))
                {
                    const auto next = interpolate(tracks[last + 1].first, tracks[last + 1].second, at);
                    if (Math::Length(next.origin - end.origin) >= RailroadSettings::tieLength + 0.1) break;
                    end = next; ++last;
                }
                if (last != first)
                {
                    const double distance = Math::Length(end.origin - tf.origin);
                    const auto forward = Math::Normalize(tf.x + end.x);
                    tf = TrackFrame((tf.origin + end.origin) * 0.5, forward, frame);
                    tf.z = tf.z * (1 + distance / RailroadSettings::tieLength);
                }
                const auto key = std::pair{first, last};
                const auto run = runs.find(key);
                if (run != runs.end() && detail.ties[run->second].count < RailroadTieRun::capacity &&
                    std::abs(Math::Length(detail.ties[run->second].last.z) - Math::Length(tf.z)) < 1e-8)
                { auto& r = detail.ties[run->second]; r.last = tf; ++r.count; }
                else { runs[key] = detail.ties.size(); detail.ties.push_back({tf, tf, 1}); }
                first = last + 1;
            }
        }
    }
}
