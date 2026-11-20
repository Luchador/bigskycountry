#pragma once

#include "railroad.hpp"
#include "road_bridge.hpp"
#include "railroad_crossing.hpp"
#include <SDL3/SDL_pixels.h>
#include <SDL3/SDL_rect.h>

#include <array>
#include <cstddef>
#include <compare>
#include <cstdint>
#include <map>
#include <optional>
#include <set>
#include <span>
#include <string>
#include <string_view>
#include <vector>

enum class ThreeLaneLayout
{
    TwoNorthbound = 0,
    TwoSouthbound = 1,
    CenterTurn = 2
};

enum class RoadEdge 
{ 
    None = 0, 
    Transition = 1, 
    Curb = 2 
};

namespace RoadEdgeColor
{
    inline constexpr SDL_FColor transitionInner{1, 0, 0, 1}, transitionOuter{0, 0, 0, 1};
    inline constexpr SDL_FColor apron{0.69f, 0.69f, 0.69f, 1};
    inline constexpr SDL_FColor riser{0.56f, 0.56f, 0.56f, 1};
    inline constexpr SDL_FColor top{0.78f, 0.78f, 0.78f, 1};
}

enum class RoadMedian { Concrete, Grass };

struct RoadDivision
{
    bool enabled = false;
    double separation = 4.0;
    RoadMedian median = RoadMedian::Concrete;
    bool operator==(const RoadDivision&) const = default;
    bool Valid() const;
};

struct RoadStyle
{
    static constexpr double dashPeriod = 12.0; // One painted dash plus its gap, in meters.
    static constexpr double laneWidth = 5.0;
    static constexpr double shoulderWidth = 5.0;
    static constexpr double innerShoulderWidth = 1.0;
    static constexpr double minSeparation = 2.0 * innerShoulderWidth;
    static constexpr double stripeHeight = 0.02; // Road paint offset above the pavement.
    static constexpr double stripWidth = 0.40;
    static constexpr double curbHeight = 0.15;
    static constexpr double curbTopWidth = 0.15;
    static constexpr int maxLanes = 10;
    static constexpr const char* defaultMaterial = "surfaces/roads/M_Asphalt01.bmat";
    std::string material = defaultMaterial;
    std::string transitionMaterial;
    std::optional<RailroadSettings> railroad;
    bool oneWay = false; // All lanes travel from the first point to the last.
    bool leftShoulder = false;
    bool rightShoulder = false;
    bool curved = true;
    RoadMedian median = RoadMedian::Concrete;
    ThreeLaneLayout threeLaneLayout = ThreeLaneLayout::CenterTurn;
    RoadEdge edge = RoadEdge::None;

    bool operator==(const RoadStyle&) const = default;
    int GetLaneCount() const;
    void SetLaneCount(int count);
    double SingleLaneExtraWidth() const { return singleLaneExtraWidth; }
    bool SetSingleLaneExtraWidth(double meters);
    double TravelLaneWidth() const { return laneWidth + (lanes == 1 ? singleLaneExtraWidth : 0.0); }
    int TravelDirection(int side) const { return oneWay ? 1 : -side; } // Along (+1) or against the authored spline.
    bool CanDivide() const { return !oneWay && lanes % 2 == 0; }
    bool IsDivided() const { return divided; }
    void SetDivided(bool value) { divided = value && CanDivide(); }
    double DividedGap() const { return dividedGap; }
    void SetDividedGap(double meters);
    RoadDivision Division() const { return {divided, dividedGap, median}; }
    void SetDivision(const RoadDivision& value)
    {
        SetDivided(value.enabled);
        SetDividedGap(value.separation);
        median = value.median;
    }
    double HalfSeparation() const { return divided ? dividedGap * 0.5 : 0.0; }
    double HalfMedianWidth() const { return divided ? HalfSeparation() - innerShoulderWidth : 0.0; }
    bool HasCenterTurnLane() const;
    double LeftWidth() const;
    double RightWidth() const;
    double EdgeWidth() const;

private:
    double singleLaneExtraWidth = 0.0; // Total added travel width, split evenly about the spline.
    bool divided = false;
    double dividedGap = 4.0; // Distance between opposing travel lanes, including both inner shoulders.
    int lanes = 2; // Total across both directions, including any center turn lane.
};

// Persistent reference to one of a junction's four approaches.
struct IntersectionAnchor
{
    int intersectionId;
    int side; // Index into directions, stored in ROAD records.
    static constexpr int sideCount = 4;
    static constexpr std::array<Vector3, sideCount> directions{{{1, 0, 0}, {0, 0, 1}, {-1, 0, 0}, {0, 0, -1}}};
    bool Valid() const { return intersectionId > 0 && side >= 0 && side < sideCount; }
    auto operator<=>(const IntersectionAnchor&) const = default;
};

// Positive station IDs identify authored highway knots, even after subdivision or a road split.
struct RampLink
{
    int station = 0, side = 0; // Side in the host spline frame, +1 left and -1 right.
    double length = 60.0;      // Game meters from the narrow end of the taper to the gore.
    double offset = 0.0; // Signed meters along the host from the stable knot to the ramp attachment.
    bool operator==(const RampLink&) const = default;
    bool Valid() const;
};

struct RoadPoint
{
    bool operator==(const RoadPoint&) const = default;
    Vector3 position;
    std::optional<double> heading; // Radians from +X toward +Z; empty follows the neighboring points.
    double pitch = 0.0; // Radians above the XZ plane for a manual tangent.
    double roll = 0.0; // Radians about the tangent; orients the control point's local Y/Z axes.
    std::optional<IntersectionAnchor> anchor; // Only endpoints can be attached.
    // Bézier handle lengths as fractions of the adjacent chords.
    double incomingHandle = 1.0 / 3.0;
    double outgoingHandle = 1.0 / 3.0;
    int rampStation = 0; // Stable attachment knot shared by highway ramps and railroad switches.
    std::optional<RampLink> ramp; // First endpoint exits the highway, last endpoint enters it.
    std::optional<RailroadSwitchLink> railSwitch;
    int connectorSocket = 0; // Identifies the source endpoint of a road/ramp join.
    int connectorTarget = 0; // Connected endpoint follows the matching source socket.
    Vector3 surfaceUp{0, 1, 0}; // Cross-section up in the road's local frame.
    double deadEndLength = 0.0; // Meters beyond a free endpoint; zero disables the end transition.
    int segmentLanes = 0; // Outgoing lane/railroad track count. Zero inherits and tapers to edited neighbors.
    std::array<int, 2> segmentShoulders{}; // Left/right outgoing overrides: 0 inherits, 1 off, 2 on.
    std::optional<RoadDivision> segmentDivision; // Outgoing segment; empty inherits the road default.
    std::optional<BridgeSettings> bridge; // Outgoing segment, independent of pavement/lane settings.
    bool Joined() const { return anchor || ramp || railSwitch || connectorSocket || connectorTarget; }
};

struct RoadGeometry
{
    // Monotonic token shared by all generated road and junction footprints.
    inline static std::uint64_t latestRevision = 0;
    std::uint64_t revision = 0;
    void MarkRebuilt() { revision = ++latestRevision; bounds = {}; }
    Bounds3 bounds; // World space, including paint, rails and selection outlines.
    void UpdateBounds();

    using Triangle = std::array<Vector3, 3>;
    struct PavementVertex
    {
        Vector3 position;
        SDL_FPoint uv;
    };
    struct EdgeVertex
    {
        Vector3 position;
        SDL_FColor color;
    };

    struct TransitionVertex
    {
        Vector3 position;
        SDL_FColor color;
        SDL_FPoint uv;
    };
    // Junction corners can inherit different strip materials from their adjoining roads.
    std::map<std::string, std::vector<std::array<TransitionVertex, 3>>> transitionStrips;
    std::vector<std::array<PavementVertex, 3>> pavementTriangles;
    using ColoredSurface = std::vector<std::array<EdgeVertex, 3>>;
    ColoredSurface edgeTriangles; // Vertex-colored outer edges and median surfaces.
    ColoredSurface bridgeTriangles; // Underside, abutments and ground under a span; excluded from the 2D overlay.
    std::vector<Vector3> outline; // Closed perimeter including shoulders, edge treatment and road ends.
    std::vector<std::vector<Vector3>> holes; // Interior terrain boundaries (e.g. a Y-intersection island).
    struct CrossingSlab
    {
        std::vector<Vector3> outline; // Additional footprint, wider than the road at the crossing.
        // Top A, top B, bottom B, bottom A: actual solid side faces which can
        // seal terrain to a lower railway toe without changing the embankment.
        std::vector<std::array<Vector3, 4>> walls;
    };
    std::vector<CrossingSlab> crossingSlabs;
    // Explicit ground contact replaces the elevated road outline when overpasses are present.
    std::vector<CrossingSlab> groundFootprints;
    struct RailroadSegment
    {
        ColoredSurface rails;
        std::vector<RailroadTieRun> ties;
        void AddTrackSample(Vector3 position);
        bool WithinDistance(Vector3 eye, double meters) const;

    private:
        std::vector<Vector3> track;
        Vector3 lower, upper; // Bounds of the sampled track centerline.
    };
    std::vector<RailroadSegment> railroadSegments;
    std::vector<TrackSection> trackSections; // Center guide at rail-head elevation for crossing slabs.
    // Individual track spans, including merging/diverging paths. Used for bridge clearance.
    std::vector<std::array<TrackSection, 2>> trackSpans;
    std::vector<Triangle> whiteStripes;
    std::vector<Triangle> yellowStripes;
};

struct RoadMouth
{
    Vector3 anchor, center, outward; // outward is the normal to the cut plane.
    double lateralScale = 1.0; // Oblique cuts span more than the perpendicular road width.
    bool operator==(const RoadMouth&) const = default;
};

struct RoadMouthFrame
{
    bool operator==(const RoadMouthFrame&) const = default;
    Vector3 left, up; // Junction cross-section axes in the road mesh frame.
    Vector3 leftPerMeter{}, upPerMeter{}; // Axis derivatives along an attached ramp's travel direction.
};

struct RampLane
{
    int station, side;
    bool exit;
    double length, halfTravel, innerPavement, outerPavement, edgeWidth;
    int laneCount = 1;
    double offset = 0.0;
    int rampIndex = -1; // Source road row for the generated taper's editor handles.
    bool operator==(const RampLane&) const = default;
};

// Runtime footprint of an attached ramp; shoulders yield to it after the taper.
struct RampShoulderCut
{
    int station, side;
    std::vector<Vector3> outline;
    bool operator==(const RampShoulderCut&) const = default;
};

struct RampSeam
{
    Vector3 center, forward;
    RoadMouthFrame frame;
    double taperLength; // Effective length after fitting the current highway.
    double goreDistance, tipDistance, stationDistance; // Along the surfaced centerline, in authored order.
    int outwardSide; // Side of the branch in its own travel frame.
    int rampIndex = -1;
};

struct Road : RoadGeometry
{
    static constexpr double minPointSpacing = 0.05; // Meters between neighboring control points.
    static constexpr double defaultDeadEndLength = 5.0;
    bool CanDeadEnd(int point) const;
    bool HasDeadEnd() const;
    int id = 0; // Persistent terrain neighbor ID. Drafts have no ID.
    bool paintMarkings = true;
    RoadStyle style;
    std::array<int, 2> laneNeighbors{}; // Lane context retained at free ends when a taper is split into separate roads.
    std::array<std::array<int, 2>, 2> shoulderNeighbors{}; // First/last end, then left/right, using the same override codes.
    std::array<std::optional<RoadDivision>, 2> divisionNeighbors{};
    // Length of the removed neighbor at a split, so both halves keep the same taper.
    // Zero means legacy/unspecified context; use the surviving segment's length.
    std::array<double, 2> profileNeighborLengths{};
    int SegmentLaneCount(std::size_t segment) const;
    int MaximumLaneCount() const;
    double MaximumHalfTravelWidth() const;
    bool HasLaneOverrides() const;
    bool SegmentShoulder(std::size_t segment, bool left) const;
    bool HasShoulderOverrides() const;
    bool HasOverpasses() const;
    void BuildOverpasses();
    std::array<bool, 2> bridgeNeighbors{}; // Derived continuation at a matching bridge endpoint; never saved.
    struct RimPoint { double distance; Vector3 position; };
    std::array<std::vector<RimPoint>, 2> bridgeRims;
    std::vector<double> bridgeKnots;
    std::map<int, std::vector<Vector3>> bridgeGroundSources; // Derived lower-road rims, never saved.
    std::map<int, std::uint64_t> bridgeSourceRevisions;
    std::map<std::uint64_t, std::vector<Vector3>> bridgeWaterSources; // Committed world-space shorelines.
    std::map<std::uint64_t, std::uint64_t> bridgeWaterRevisions;
    bool HasProfileOverrides() const { return HasLaneOverrides() || HasShoulderOverrides() || HasDivisionOverrides(); }
    bool HasDivisionOverrides() const;
    double MaximumHalfSeparation() const;
    RoadStyle SegmentStyle(std::size_t segment) const;
    // Profile IDs: 0 = preceding split context, 1..N = segments, N+1 = following context.
    RoadStyle SurfaceStyle(int profile) const;
    bool SetSegmentDivision(const std::set<int>& segments, const RoadDivision& division);
    void SetDivision(const RoadDivision& division);
    double MaximumShoulderWidth(bool left) const;
    bool SetSegmentShoulders(const std::set<int>& segments, bool left, bool enabled);
    void SetShoulder(bool left, bool enabled); // Whole road: clear overrides for this side only.
    bool CanDivideAllSegments() const;
    RoadStyle EndpointStyle(bool first) const;
    bool SetSegmentLaneCounts(const std::set<int>& segments, int count);
    void SetLaneCount(int count); // Whole road: replace the default and clear segment overrides.
    struct ProfileBlend { int from, to; double mix; };
    using LaneBlend = ProfileBlend;
    ProfileBlend SegmentSurfaceBlend(std::size_t segment, double fraction) const;
    LaneBlend SegmentLaneBlend(std::size_t segment, double fraction) const;
    ProfileBlend SegmentShoulderBlend(std::size_t segment, double fraction, bool left) const;
    double ProfileTransitionMix(std::size_t segment, double fraction, bool before) const;
    Vector3 rotation{}; // Radians about X, Y, Z. The first control point is the road pivot.
    std::vector<RoadPoint> points;
    std::vector<RailroadCrossingLink> crossings;
    // Generated constraints; authored knots and the railway alignment are preserved.
    std::vector<CrossingProfile> crossingProfiles;
    // Generated cut lines at active intersections. The authored spline stays editable all the way to the anchor.
    void SetMouths(const std::array<std::optional<RoadMouth>, 2>& mouths, bool rebuild = false);

    const std::optional<RoadMouth>& EndpointMouth(bool first) const { return mouths[first ? 0 : 1]; }

    // Includes both endpoints; the span is valid until the road is rebuilt.
    std::span<const Vector3> SegmentSamples(std::size_t segment) const;
    // World-space centers of the surfaced lanes, sampled in spline order. On multi-lane
    // roads the configured lanes run right to left, followed by any ramp taper lanes.
    std::vector<std::vector<Vector3>> laneCenterlines;
    // Same indices as the outer lane centerlines, including equal-distance bevel stations.
    std::array<std::vector<double>, 2> outerLaneDistances;
    std::array<std::vector<Vector3>, 2> outerLaneCenterlines; // Independent of lane guide runs through tapers.
    std::span<const Vector3> OuterLane(int side) const;
    Vector3 OuterLanePosition(int side, double distance) const;
    Vector3 SurfaceUp(std::size_t segment, double t) const;
    Vector3 GuideOffset() const; // Lift railroad handles and splines above their raised track.
    double SplineLengthMeters() const { return splineLengthMeters; }
    Vector3 Tangent(std::size_t index) const;
    bool InsertStation(int segment, double t, int stationId);
    // Derived from attached ramp ends. Each seam uses the actual highway surface frame.
    std::vector<RampLane> rampLanes;
    std::vector<RampShoulderCut> rampShoulderCuts; // Derived, never serialized.
    std::map<std::pair<int, int>, RampSeam> rampSeams; // (station ID, side)
    std::array<std::optional<RoadMouthFrame>, 2> rampCaps{};
    std::array<bool, 2> skewEnds{}; // Derived from the linked intersection, including unpaved junctions.

    double ThreeLaneDividerOffset() const; // Signed distance left of the authored path for a 2+1 layout.
    std::array<Vector3, 3> PointAxes(std::size_t index, bool includeRoll = true) const;
    void RotatePoint(std::size_t index, double heading);
    void RotatePoint(std::size_t index, Vector3 unitAxis, double angle);
    bool SetPose(Vector3 position, Vector3 angles);
    bool TransformControlPoints(Vector3 position, Vector3 angles); // Authored data only, for batch edits.
    std::string NameList() const;
    bool SetNameList(std::string_view text);
    Road NewSection() const; // Inherit road properties before supplying the section's points.
    // An independent run of surviving segments, including both endpoint control points.
    Road Section(std::size_t first, std::size_t last) const;
    bool SubdivideSegments(const std::set<int>& segments);
    void Rebuild();

private:
    double ProfileSegmentLength(int segment) const;
    std::vector<double> profileLengths; // Rebuilt once; physical distance along each surfaced segment.
    double RampFrameBlendLength() const { return 2.0 * (style.LeftWidth() + style.RightWidth()); }
    std::vector<std::string> names;
    std::array<std::optional<RoadMouth>, 2> mouths;
    void BuildSurface(const std::vector<Vector3>& samples, std::span<const Vector3> tangents,
                      const std::array<std::optional<RoadMouthFrame>, 2>& caps, std::size_t firstSample,
                      std::span<const Vector3> surfaceUps);
    std::vector<Vector3> centerline;
    double splineLengthMeters = 0.0;
    std::vector<std::size_t> pointSamples; // Centerline index of each control point.
};
