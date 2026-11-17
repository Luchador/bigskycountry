#pragma once

#include "lake.hpp"
#include <array>
#include <variant>

// Two open cubic splines running from start to end, including local elevation.
// Banks can have different numbers of control points. End handles move both banks.
struct RiverDefinition
{
    std::array<std::vector<LakePoint>, 2> banks;
    double depth = 5;
    static RiverDefinition Straight(double width = 25, double length = 200);
    Vector3 Evaluate(int bank, int segment, double t) const;
    int PointCount() const;
    int End(int point) const; // 0 = start handle, 1 = end handle, -1 = bank point.
    Vector3 Point(int point) const;
    bool MovePoint(int point, Vector3 position);
    bool Extend(int end, Vector3 position);
    int InsertPoint(int bank, int segment, double t); // Returns the new flattened control index.
    bool RemovePoint(int point);
    bool Read(std::istream& in, bool elevations = true);
    bool Write(std::ostream& out) const;
    bool operator==(const RiverDefinition&) const = default;
private:
    void SmoothHandles(int bank);
};

struct RiverGeometry
{
    static std::optional<std::vector<LakeSample>> Sample(const RiverDefinition& river);
    static std::optional<LakeGeometry> Build(const RiverDefinition& river);
};

// Common editor operations keep lake and river edit mode/undo on the same path.
using WaterDefinition = std::variant<LakeDefinition, RiverDefinition>;
int WaterPointCount(const WaterDefinition& water);
Vector3 WaterPoint(const WaterDefinition& water, int point);
bool MoveWaterPoint(WaterDefinition& water, int point, Vector3 position);
int InsertWaterPoint(WaterDefinition& water, int bank, int segment, double t);
bool RemoveWaterPoint(WaterDefinition& water, int point);
