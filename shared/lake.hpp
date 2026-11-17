#pragma once

#include "mesh.hpp"
#include <iosfwd>
#include <optional>
#include <set>

// Shared bank/shoreline control point. Lakes keep these in local XZ;
// rivers also store elevation. Handles preserve the curve on insertion.
struct LakePoint
{
    Vector3 position, incoming, outgoing; // Handles are offsets from position.
    bool operator==(const LakePoint&) const = default;
};

struct LakeDefinition
{
    static constexpr const char* waterMaterial = "world/M_LakeWater.bmat";
    static constexpr std::size_t maxPoints = 256;
    std::vector<LakePoint> points;
    double depth = 5;
    static LakeDefinition Circle(double diameter = 200);
    Vector3 Evaluate(int segment, double t) const;
    bool MovePoint(int index, Vector3 position);
    bool TranslatePoints(const std::set<int>& indices, Vector3 delta);
    bool InsertPoint(int segment, double t);
    bool RemovePoint(int index);
    bool Read(std::istream& in);
    bool Write(std::ostream& out) const;
    bool operator==(const LakeDefinition&) const = default;

private:
    void SmoothHandles();
};

struct LakeSample
{
    Vector3 position;
    int segment;
    double t;
    int bank = -1; // Rivers: 0/1 for banks, -1 for the closing end cap.
};

struct LakeGeometry
{
    // Cheap authoring preview: finite/spacing checks and spline samples only.
    // Polygon validation, triangulation and basin heights wait until Build.
    static std::optional<std::vector<LakeSample>> Sample(const LakeDefinition& lake);
    std::shared_ptr<const Mesh> bed, water;
    std::vector<LakeSample> shoreline; // Closed, including the repeated first sample.
    // Invalid/self-crossing edits leave the previous meshes intact.
    static std::optional<LakeGeometry> Build(const LakeDefinition& lake);
    // Shared polygon validation and subdivided water/basin builder. River rim
    // elevations define the surface; the bed is lowered relative to that surface.
    static std::optional<LakeGeometry> BuildShoreline(std::vector<LakeSample> rim, double depth);
};
