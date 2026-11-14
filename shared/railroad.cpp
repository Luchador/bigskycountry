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

    void BedBand(RoadGeometry& mesh, const Section& a, const Section& b, ProfilePoint left, ProfilePoint right)
    {
        const auto vertex = [](const Section& s, ProfilePoint p)
        {
            return RoadGeometry::PavementVertex{s.Point(p.lateral, p.height),
                    {static_cast<float>(p.lateral / 4), static_cast<float>(s.distance / 4)}};
        };
        const auto v0 = vertex(a, left), v1 = vertex(a, right), v2 = vertex(b, right), v3 = vertex(b, left);
        mesh.pavementTriangles.push_back({v0, v1, v2});
        mesh.pavementTriangles.push_back({v0, v2, v3});
    }

    void Transition(RoadGeometry& mesh, const Section& a, const Section& b, double toe, double rise, int side,
                    const std::string& material)
    {
        const auto vertex = [&](const Section& s, bool outer)
        {
            const double offset = side * (toe + (outer ? RoadStyle::stripWidth : 0.0));
            return RoadGeometry::TransitionVertex{s.Point(offset, outer ? 0.0 : rise),
                       outer ? RoadEdgeColor::transitionOuter : RoadEdgeColor::transitionInner,
                       {static_cast<float>(offset / 4), static_cast<float>(s.distance / 4)}};
        };
        const auto v0 = vertex(a, side < 0), v1 = vertex(a, side > 0), v2 = vertex(b, side > 0), v3 = vertex(b, side < 0);
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
    return std::isfinite(width) && width >= tieLength && std::isfinite(height) && height >= 0;
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
    return TrackFrame(first.origin + (last.origin - first.origin) * t, forward, reference);
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

void BuildRailroad(RoadGeometry& mesh, const RailroadSettings& settings, std::span<const Vector3> samples,
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

    const double crown = settings.width / 2, height = settings.height, run = height * 2;
    const double toe = crown + run, toeHeight = transition ? std::min(0.035, height * 0.1) : 0;
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
    for (const auto& s : sections)
        mesh.trackSections.push_back({s.Point(0, railBase + RailroadSettings::railHeight), s.axes.z});
    for (const auto& s : sections) mesh.outline.push_back(s.Point(-outerWidth, 0));
    for (auto it = sections.rbegin(); it != sections.rend(); ++it) mesh.outline.push_back(it->Point(outerWidth, 0));

    for (bool first : {true, false})
    {
        const auto& end = first ? sections.front() : sections.back();
        if (height > 0)
        {
            for (std::size_t i = 1; i < bed.size(); ++i) EndFace(mesh.edgeTriangles, end, bed[i - 1], bed[i], 0, ballast, first);
            if (transition)
            {
                EndFace(mesh.edgeTriangles, end, {-outerWidth, 0}, bed.front(), 0, ballast, first);
                EndFace(mesh.edgeTriangles, end, bed.back(), {outerWidth, 0}, 0, ballast, first);
            }
        }
    }

    const auto railCaps = [&](RoadGeometry::ColoredSurface& rails, const Section& end, bool first)
    {
        for (const double center : {-railCenter, railCenter})
        {
            const auto cap = [&](double width, double low, double high)
            {
                EndFace(rails, end, {center - width / 2, railBase + high}, {center + width / 2, railBase + high}, railBase + low, steel, first);
            };
            cap(footWidth, 0, footHeight);
            cap(webWidth, footHeight, headBottom);
            cap(headWidth, headBottom, RailroadSettings::railHeight);
        }
    };

    // The first center sits half a spacing from the start. Carry stationing
    // across all spline spans so adding a control point cannot restart the ties.
    std::size_t segment = 0;
    double tieStation = RailroadSettings::tieSpacing / 2;
    for (std::size_t i = 1; i < sections.size(); ++i)
    {
        const auto& a = sections[i - 1];
        const auto& b = sections[i];
        const auto previous = segment;
        while (segment + 2 < controlSamples.size() && b.sampleIndex > controlSamples[segment + 1]) ++segment;
        if (i == 1 || previous != segment)
        {
            auto& detail = mesh.railroadSegments.emplace_back();
            detail.AddTrackSample(a.Point(0, railBase));
        }
        auto& detail = mesh.railroadSegments.back();
        detail.AddTrackSample(b.Point(0, railBase));
        if (i == 1) railCaps(detail.rails, a, true);
        if (i + 1 == sections.size()) railCaps(detail.rails, b, false);

        for (std::size_t j = 1; j < bed.size(); ++j) BedBand(mesh, a, b, bed[j - 1], bed[j]);
        for (int side : {-1, 1})
        {
            Rail(detail.rails, a, b, side * railCenter, railBase);
            if (transition) Transition(mesh, a, b, toe, toeHeight, side, material);
        }
        const auto tieAt = [&](double distance)
        {
            const double t = (distance - a.distance) / (b.distance - a.distance);
            auto forward = Math::Normalize(a.axes.x * (1 - t) + b.axes.x * t);
            if (forward == Vector3{}) forward = Math::Normalize(b.axes.origin - a.axes.origin);
            const auto center = a.Point(0, height) * (1 - t) + b.Point(0, height) * t;
            return TrackFrame(center, forward, frame);
        };
        const double limit = std::min(b.distance, sections.back().distance - tieWidth / 2);
        while (tieStation <= limit)
        {
            const double remaining = std::floor((limit - tieStation) / RailroadSettings::tieSpacing);
            const int count = 1 + static_cast<int>(std::min<double>(remaining, RailroadTieRun::capacity - 1));
            detail.ties.push_back({tieAt(tieStation), tieAt(tieStation + (count - 1) * RailroadSettings::tieSpacing), count});
            tieStation += count * RailroadSettings::tieSpacing;
        }
    }
}
