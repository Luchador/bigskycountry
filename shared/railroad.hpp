#pragma once

#include "mesh.hpp"
#include <span>

struct RoadGeometry;

struct RailroadSettings
{
    static constexpr double gauge = 1.435; // Between the inner faces of the rail heads.
    static constexpr double headWidth = 0.075;
    static constexpr double flangeClearance = 0.06; // Game crossing groove, on the gauge side.
    static constexpr double outerClearance = 0.01;
    static constexpr double grooveDepth = 0.06;
    static constexpr double tieLength = 2.6;
    static constexpr double tieSpacing = 0.60;
    static constexpr double tieHeight = 0.18;
    static constexpr double railHeight = 0.17;
    double width = 4.0;   // Flat top of the embankment, in meters.
    double height = 0.75; // Above the authored spline at the toe.
    bool Valid() const;
    bool operator==(const RailroadSettings&) const = default;
};

// Bound groups of sleepers for rendering and picking; keep only their end frames in memory.
struct RailroadTieRun
{
    static constexpr int capacity = 64;
    MeshTransform first, last;
    int count;
    Vector3 Center() const { return (first.origin + last.origin) * 0.5; }
    double Radius() const { return Math::Length(last.origin - first.origin) * 0.5 + RailroadSettings::tieLength; }
    MeshTransform At(int index) const;
};

void BuildRailroad(RoadGeometry& geometry, const RailroadSettings& settings, std::span<const Vector3> samples,
                   std::span<const Vector3> tangents, std::span<const std::size_t> controlSamples,
                   const MeshTransform& frame, bool transition,
                   const std::string& transitionMaterial);

// All ties share a single solid mesh, in meters, with X along the track.
const std::shared_ptr<const Mesh>& RailroadTieMesh();
