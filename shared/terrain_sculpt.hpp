#pragma once

#include "mesh.hpp"
#include <optional>
#include <iosfwd>

enum class TerrainTool { Raise, Lower, Flatten };
struct TerrainBrushSettings
{
    bool enabled = false;
    TerrainTool tool = TerrainTool::Raise;
    double radius = 10;
    double strength = 2; // Meters/second; flatten convergence rate/second.
    double seamFalloff = 10;
};
struct TerrainStamp
{
    TerrainTool tool = TerrainTool::Raise;
    Vector3 center{}; // Generated terrain uses world coordinates. Y is the flatten target.
    double radius = 10, amount = 0, seamFalloff = 10;
    bool Valid() const;
};
// Persistent immutable history: a stroke snapshot shares its unchanged prefix.
struct TerrainSculptLayer
{
    TerrainStamp stamp;
    mutable std::shared_ptr<const TerrainSculptLayer> previous; // Mutable only during iterative destruction.
    std::size_t count = 1;
    ~TerrainSculptLayer();
};
using TerrainSculptEdits = std::shared_ptr<const TerrainSculptLayer>;
bool ReadTerrainSculpt(std::istream& in, TerrainSculptEdits& edits);
void WriteTerrainSculpt(std::ostream& out, TerrainSculptEdits edits);

// Private editable copy; cached source meshes and neighboring infrastructure stay immutable.
class TerrainSculptSurface
{
public:
    explicit TerrainSculptSurface(std::shared_ptr<const Mesh> source);
    ~TerrainSculptSurface();
    bool Apply(const TerrainStamp& stamp); // False for no change, including protected rims.
    const std::shared_ptr<const Mesh>& MeshData() const;
    std::optional<double> Height(double x, double z) const;
private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
