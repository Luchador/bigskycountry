#include "cooked_world.hpp"
#include "world_renderer.hpp"
#include "camera_view.hpp"
#include "project_settings.hpp"

bool CookedWorld::Load(const WorldDocument& source)
{
    ready = false; cells.clear();
    WorldPackage::Reader reader;
    if (!reader.Open(WorldPackage::DefaultPath(source)) || !reader.Matches(source.Path(), AssetRoot::Cooked)) return false;
    std::vector<WorldPackage::Cell> loaded;
    loaded.reserve(reader.Index().cells.size());
    for (std::size_t i = 0; i < reader.Index().cells.size(); ++i)
    {
        WorldPackage::Cell cell;
        if (!reader.ReadCell(i, cell)) return false;
        loaded.push_back(std::move(cell));
    }
    index.BeginSync();
    for (std::size_t i = 0; i < loaded.size(); ++i)
        index.Upsert({SceneKind::Terrain, static_cast<int>(i)}, reader.Index().cells[i].bounds);
    index.EndSync();
    cells = std::move(loaded); ready = true; return true;
}
void CookedWorld::Submit(WorldRenderer& renderer, const CameraView& camera, double radius)
{
    using namespace WorldPackage;
    const auto eye = camera.GetPosition();
    const double railDistance = g_ProjectSettings.RailroadDetailDistance();
    for (auto key : index.Query(eye, radius))
    {
        const auto& bounds = index.Bounds(key);
        if (!renderer.BoundsVisible(bounds)) continue;
        const bool far = BoundsDistanceSquared(bounds, eye) > distantMeters * distantMeters;
        for (const auto& batch : cells[key.index].batches)
        {
            const auto& mesh = batch.layer == Layer::Rail || !far ? batch.nearMesh : batch.distantMesh;
            if (!mesh.mesh || mesh.mesh->indices.empty()) continue;
            if (batch.layer == Layer::Rail && BoundsDistanceSquared(WorldBounds(mesh.mesh->bounds, mesh.transform), eye) > railDistance * railDistance) continue;
            renderer.SubmitWorldMesh(mesh, batch.material);
        }
        for (const auto& ties : cells[key.index].ties) renderer.SubmitRailroadTies(ties.run, 1);
    }
}
