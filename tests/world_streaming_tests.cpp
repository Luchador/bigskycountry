#include "cooked_world.hpp"
#include "world_collision.hpp"
#include "world_renderer.hpp"
#include "surface_material.hpp"
#include "project_settings.hpp"
#include "spatial_index.hpp"
#include "camera_view.hpp"
#include <algorithm>
#include <SDL3/SDL.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
using namespace WorldPackage;
void Check(bool ok, const char *what)
{
    if (!ok) throw std::runtime_error(std::string(what) + ": " + SDL_GetError());
}
struct Fixture
{
    std::filesystem::path directory;
    std::string source, package;
    Manifest manifest;
    std::vector<Cell> cells;
    Fixture()
    {
        SDL_Time now;
        SDL_GetCurrentTime(&now);
        directory = std::filesystem::temp_directory_path() / ("bigsky-streaming-" + std::to_string(now));
        std::filesystem::create_directories(directory);
        source = (directory / "world.wld").string();
        WorldDocument world(AssetRoot::Project);
        Check(world.Open(source.c_str()) && world.Save(), "Create source fixture");
        // A unique package basename avoids touching the user's cooked world.
        source = (directory / ("streaming-test-" + std::to_string(now) + ".wld")).string();
        std::filesystem::copy_file(directory / "world.wld", source);
        WorldDocument runtime;
        Check(runtime.OpenPath(source.c_str()), "Resolve runtime path");
        Check(!runtime.Save(), "Runtime source document must be read-only");
        package = DefaultPath(runtime);
        manifest.sourceHash = FileHash(source);
        manifest.spawns.push_back({{20, 0, 100}, 0});
        Entity owner;
        owner.id = {SceneKind::Road, 1};
        owner.bounds.Include({-512, 0, 0});
        owner.bounds.Include({42000, 0, 512});
        manifest.entities.push_back(owner);
        for (int x : {-1, 0, 1, 3, 5, 10, 20, 40, 80})
        {
            auto mesh = std::make_shared<Mesh>();
            mesh->vertices = {
                {{0, 0, 0}, {0, 1, 0}}, {{512, 0, 0}, {0, 1, 0}}, {{512, 0, 512}, {0, 1, 0}}, {{0, 0, 512}, {0, 1, 0}}};
            mesh->indices = {0, 2, 1, 0, 3, 2};
            mesh->UpdateBounds();
            Cell cell;
            cell.key = {x, 0};
            Batch batch;
            batch.owner = owner.id;
            batch.nearMesh = {mesh, MeshTransform{cell.key.Origin()}};
            batch.distantMesh = batch.nearMesh;
            cell.batches.push_back(batch);
            cells.push_back(std::move(cell));
        }
        Check(Write(package, manifest, cells), "Write independent streaming fixture");
    }
    ~Fixture()
    {
        std::filesystem::remove(package);
        std::filesystem::remove_all(directory);
    }
};
const StreamingSettings defaults;
template <typename Predicate>
void Pump(CookedWorld &stream, WorldCollision &collision, Vector3 eye, const std::vector<Vector3> &sources,
          double radius, const StreamingSettings &settings, Predicate finished)
{
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(10);
    do
    {
        stream.Update(eye, sources, collision, nullptr, radius, settings);
        if (finished()) return;
        SDL_Delay(1);
    } while (std::chrono::steady_clock::now() < deadline);
    throw std::runtime_error("Timed out waiting for asynchronous streaming: " + stream.Error());
}
void Residency(Fixture &f)
{
    CookedWorld stream;
    WorldCollision collision;
    Check(stream.LoadPath(f.source), "Open package without authored geometry");
    Check(stream.Stats().geometryBytes == 0 && collision.ChunkCount() == 0, "Catalog eagerly loaded the world");
    Check(stream.MayHaveGroundBelow({256, 3000, 100}), "Unloaded landing terrain was treated as empty space");
    Check(stream.MayHaveGroundBelow({-2, 3000, -2}), "Landing check missed pawn footprint at cell edge");
    Check(!stream.MayHaveGroundBelow({256, -1, 100}), "Ground above pawn blocked out-of-world recovery");
    Check(!stream.MayHaveGroundBelow({100000, 3000, 100}), "Empty space reported a landing surface");
    const Vector3 home{256, 0, 100}, away{10500, 3, 100};
    Check(!stream.CollisionReady(home), "Missing collision reported ready");
    Pump(stream, collision, home, {home}, 600, defaults,
         [&] { return stream.Stats().nearCells >= 3 && !stream.Stats().queued; });
    Check(stream.CollisionReady(home) && stream.Stats().collisionChunks > 0, "Collision not published");
    Check(stream.Stats().nearCells < stream.Stats().totalCells, "Loaded cells outside visibility radius");
    for (double x : {-0.001, 0.0, 0.001, 511.999, 512.0, 512.001})
    {
        const auto hit = collision.Ground({x, 2, 100}, 1, 10);
        Check(hit && std::abs(hit->position.y) < 1e-10, "Collision gap at chunk seam");
    }
    const auto completed = stream.Stats().completed;
    for (int i = 0; i < 50; ++i) stream.Update(home, std::vector{home}, collision, nullptr, 600, defaults);
    Check(stream.Stats().completed == completed && !stream.Stats().queued, "Idle camera repeated work");
    Pump(stream, collision, away, {home}, 600, defaults,
         [&] { return stream.Stats().nearCells == 1 && !stream.Stats().queued; });
    Check(stream.Stats().evicted > 0 && collision.Ground(home, 1, 10).has_value(),
          "Debug camera unloaded parked pawn support");
    Pump(stream, collision, away, {away}, 600, defaults,
         [&] { return stream.CollisionReady(away) && !stream.Stats().queued; });
    Check(!collision.Ground(home, 1, 10), "Old collision not released after pawns moved");
    Check(collision.Ground(away, 1, 10).has_value(), "New collision missing");
    Pump(stream, collision, home, {home}, 12000, defaults,
         [&] { return stream.Stats().distantCells >= 3 && !stream.Stats().queued; });
    Check(stream.Stats().nearCells > 0 && stream.Stats().distantCells > 0,
          "Near/far representations not selected independently");
    StreamingSettings tiny = defaults;
    tiny.geometryBytes = 1;
    tiny.collisionBytes = 1;
    Pump(stream, collision, home, {home}, 12000, tiny, [&] { return !stream.Stats().queued; });
    Check(stream.Stats().pressure && stream.CollisionReady(home), "Memory pressure evicted required collision");
    const auto stable = stream.Stats().completed;
    for (int i = 0; i < 50; ++i)
    {
        stream.Update(home, std::vector{home}, collision, nullptr, 12000, tiny);
        SDL_Delay(1);
    }
    Check(stream.Stats().completed == stable && !stream.Stats().queued, "Budget pressure caused load/evict churn");
}
void CancellationAndFailure(Fixture &f)
{
    CookedWorld stream;
    WorldCollision collision;
    const Vector3 home{20, 3, 100}, away{41000, 3, 100};
    Check(stream.LoadPath(f.source), "Open cancellation fixture");
    stream.Update(home, std::vector{home}, collision, nullptr, 600, defaults);
    Pump(stream, collision, away, {away}, 600, defaults,
         [&] { return stream.CollisionReady(away) && stream.Stats().nearCells == 1 && !stream.Stats().queued; });
    Check(stream.Stats().discarded >= 1 && !collision.Ground(home, 1, 10), "Stale camera completion published");
    Check(stream.LoadPath(f.source), "Reload stream catalog");
    Pump(stream, collision, home, {home}, 600, defaults,
         [&] { return stream.CollisionReady(home) && !stream.Stats().queued; });
    Check(!collision.Ground(away, 1, 10), "Old catalog collision survived reload");
    Reader reader;
    Check(reader.Open(f.package), "Inspect failure fixture");
    std::fstream file(f.package, std::ios::in | std::ios::out | std::ios::binary);
    file.seekg(-1, std::ios::end);
    char byte;
    file.get(byte);
    byte ^= 1;
    file.seekp(-1, std::ios::end);
    file.put(byte);
    file.close();
    Check(stream.LoadPath(f.source), "Manifest should open without eager payload reads");
    Pump(stream, collision, away, {away}, 600, defaults, [&] { return stream.Stats().failed > 0; });
    Check(!stream.CollisionReady(away) && !collision.Ground(away, 1, 10),
          "Corrupt collision allowed physics to proceed");
    const auto failures = stream.Stats().failed;
    for (int i = 0; i < 25; ++i)
    {
        stream.Update(away, std::vector{away}, collision, nullptr, 600, defaults);
        SDL_Delay(1);
    }
    Check(stream.Stats().failed == failures, "Failed chunk was retried every frame");
    Check(Write(f.package, f.manifest, f.cells), "Restore fixture");
    // Destruction while a job is in flight joins the worker before releasing its catalog.
    {
        CookedWorld exiting;
        Check(exiting.LoadPath(f.source), "Open shutdown fixture");
        exiting.Update(home, std::vector{home}, collision, nullptr, 600, defaults);
    }
}
void Models(Fixture &f)
{
    MeshAssets assets(AssetRoot::Cooked);
    const auto model = assets.Load("terrain/test_grid.glb");
    Check(model != nullptr, "Load fixture shared model");
    for (int i = 0; i < 2; ++i)
    {
        Entity object;
        object.id = {SceneKind::Object, std::uint64_t(i + 1)};
        object.model = "terrain/test_grid.glb";
        object.type = ObjectType::Marker;
        object.markerText = "Test city";
        object.markerColor = {12, 34, 56};
        object.pose.position = {20.0 + i * 30, 0, 100};
        object.transform = object.pose.ToMeshTransform();
        object.bounds = WorldBounds(model->bounds, object.transform);
        f.manifest.entities.push_back(object);
    }
    Asset asset;
    asset.kind = AssetKind::Model;
    asset.path = "terrain/test_grid.glb";
    asset.hash = AssetHash(asset.kind, asset.path, AssetRoot::Cooked);
    f.manifest.assets.push_back(asset);
    Check(Write(f.package, f.manifest, f.cells), "Write models fixture");
    Reader reader;
    Check(reader.Open(f.package), "Read marker metadata");
    Check(reader.Index().entities.back().markerText == "Test city" &&
              reader.Index().entities.back().markerColor[1] == 34,
          "Lost marker state");
    CookedWorld stream;
    WorldCollision collision;
    const Vector3 home{20, 3, 100};
    Check(stream.LoadPath(f.source), "Open shared models");
    Pump(stream, collision, home, {home}, 600, defaults,
         [&] { return stream.Stats().modelInstances == 2 && stream.CollisionReady(home) && !stream.Stats().queued; });
    Check(stream.Stats().modelAssets == 1, "Repeated placements decoded duplicate model assets");
    const Vector3 away{41000, 3, 100};
    Pump(stream, collision, away, {away}, 600, defaults,
         [&] { return stream.CollisionReady(away) && !stream.Stats().queued; });
    Check(stream.Stats().modelAssets == 0 && stream.Stats().modelInstances == 0 && stream.Stats().modelBytes == 0,
          "Unused shared models retained");
}
void SharedTextures(SDL_GPUDevice *device = nullptr)
{
    SDL_Time now;
    SDL_GetCurrentTime(&now);
    const auto folder = "stream-test-" + std::to_string(now) + "/";
    const auto mapPath = folder + "color.png";
    const auto file = DdsTexture::CookedPath(mapPath, DdsTexture::Format::BaseColor);
    std::filesystem::create_directories(std::filesystem::path(file).parent_path());
    std::vector<unsigned char> dds(148 + 48);
    auto u32 = [&](int offset, unsigned value) {
        for (int i = 0; i < 4; ++i) dds[offset + i] = static_cast<unsigned char>(value >> (8 * i));
    };
    std::copy_n("DDS ", 4, dds.begin());
    u32(4, 124);
    u32(76, 32);
    std::copy_n("DX10", 4, dds.begin() + 84);
    u32(128, 99);
    u32(132, 3);
    u32(140, 1);
    u32(16, 4);
    u32(12, 4);
    u32(28, 3);
    {
        std::ofstream out(file, std::ios::binary);
        out.write(reinterpret_cast<const char *>(dds.data()), dds.size());
    }
    const auto root = AssetPaths::Root(AssetRoot::Cooked) + "textures/";
    std::filesystem::create_directories(root + folder);
    for (const auto name : {"one.bmat", "two.bmat"})
    {
        std::ofstream out(root + folder + name);
        out << R"({"version":1})";
    }
    Material material;
    material.type = MaterialType::Unlit;
    material.textures[static_cast<int>(MaterialSlot::Emissive)] = mapPath;
    Check(material.Save(root + folder + "one.bmat") && material.Save(root + folder + "two.bmat"),
          "Save shared texture materials");
    {
        SurfaceMaterialCache cache(AssetRoot::Cooked);
        auto use = [&] {
            cache.BeginFrame(device, 1024);
            const auto *a = cache.Get(folder + "one.bmat", device);
            const auto *b = cache.Get(folder + "two.bmat", device);
            Check(a && b, "Streaming material fallback unavailable");
            Check(a->maps[3] == b->maps[3], "Materials uploaded duplicate shared maps");
        };
        for (int i = 0; i < 1000 && cache.TextureCount() != 1; ++i)
        {
            use();
            SDL_Delay(1);
        }
        Check(cache.TextureCount() == 1, "Asynchronous texture was never published");
        const auto bytes = cache.Bytes();
        for (int i = 0; i < 50; ++i) use();
        Check(cache.Bytes() == bytes && cache.TextureCount() == 1, "Repeated materials grew texture residency");
        for (int i = 0; i < 150; ++i) cache.BeginFrame(device, 1024);
        Check(cache.Bytes() == bytes && cache.TextureCount() == 1, "Turning away expired a warm texture below budget");
        use();
        cache.BeginFrame(device, 1);
        cache.BeginFrame(device, 1);
        Check(cache.TextureCount() == 0, "Unused texture ignored budget pressure");
        // Refresh while a read is pending must discard its old generation safely.
        use();
        Material::RefreshAssets();
        cache.BeginFrame(device, 1024);
        for (int i = 0; i < 1000 && cache.TextureCount() != 1; ++i)
        {
            use();
            SDL_Delay(1);
        }
        Check(cache.TextureCount() == 1, "Refresh left texture permanently pending");
    }
    std::filesystem::remove_all(root + folder);
    std::filesystem::remove_all(std::filesystem::path(file).parent_path());
}
void GpuCaches()
{
    Check(SDL_Init(SDL_INIT_VIDEO), "Initialize GPU test");
    SDL_Window *window = SDL_CreateWindow("Streaming GPU regression", 640, 480, SDL_WINDOW_HIDDEN);
    Check(window != nullptr, "Create GPU window");
    SDL_Renderer *target = SDL_CreateRenderer(window, "gpu");
    Check(target != nullptr, "Create GPU renderer");
    auto *device = static_cast<SDL_GPUDevice *>(
        SDL_GetPointerProperty(SDL_GetRendererProperties(target), SDL_PROP_RENDERER_GPU_DEVICE_POINTER, nullptr));
    Check(device != nullptr, "GPU device unavailable");
    SharedTextures(device);
    {
        WorldRenderer renderer(AssetRoot::Project);
        CameraView camera;
        WorldLighting lights;
        lights.ambient = {0.5, 0.5, 0.5};
        camera.LookAt({256, 150, -300}, {256, 0, 256});
        Fixture fixture;
        const auto &mesh = fixture.cells[1].batches.front().nearMesh;
        auto draw = [&](bool show) {
            Check(SDL_SetRenderDrawColor(target, 20, 30, 50, 255) && SDL_RenderClear(target), "Clear GPU target");
            Check(renderer.Begin(target, camera, &lights), "Begin GPU frame");
            if (show) renderer.SubmitWorldMesh(mesh);
            Check(renderer.End() && SDL_RenderPresent(target), "Publish GPU frame");
        };
        draw(true);
        Check(renderer.ResidentBytes() > 0, "Geometry was not retained on GPU");
        for (int i = 0; i < 125; ++i) draw(false);
        Check(renderer.ResidentBytes() > 0, "Turning away expired warm GPU geometry below budget");
        draw(true);
        Check(renderer.PreparedVertices() == 0 && renderer.UploadedBytes() == 0,
              "Turning back rebuilt or uploaded warm geometry");
        renderer.Forget(&mesh);
        draw(false);
        Check(renderer.ResidentBytes() == 0 && mesh.mesh && !mesh.mesh->indices.empty(),
              "GPU eviction removed authored data or retained buffers");

        // Many triangles in a single batch used to bypass the upload limit and
        // all normals/tangents were prepared on its first visible frame.
        auto grid = std::make_shared<Mesh>();
        constexpr unsigned side = 128;
        for (unsigned z = 0; z <= side; ++z)
            for (unsigned x = 0; x <= side; ++x)
                grid->vertices.push_back({{4.0 * x, 0, 4.0 * z}, {0, 1, 0}});
        for (unsigned z = 0; z < side; ++z)
            for (unsigned x = 0; x < side; ++x)
            {
                const auto a = z * (side + 1) + x, b = a + 1, c = a + side + 1, d = c + 1;
                grid->indices.insert(grid->indices.end(), {a, c, b, b, c, d});
            }
        grid->UpdateBounds();
        MeshInstance large{grid};
        std::size_t prepared = 0, uploaded = 0;
        int frames = 0, uploadFrames = 0;
        const auto expected = grid->indices.size() * sizeof(WorldDepthVertex);
        const auto pixel = [&] {
            auto* surface = SDL_RenderReadPixels(target, nullptr);
            Check(surface != nullptr, "Read incremental render");
            std::array<Uint8, 4> color;
            const bool ok = SDL_ReadSurfacePixel(surface, surface->w / 2, surface->h / 2,
                &color[0], &color[1], &color[2], &color[3]);
            SDL_DestroySurface(surface);
            Check(ok, "Read incremental render pixel");
            return color;
        };
        std::array<Uint8, 4> completePixel{};
        while (!renderer.ReadyBuffers() && frames++ < 200)
        {
            Check(SDL_RenderClear(target) && renderer.Begin(target, camera, &lights), "Begin incremental geometry frame");
            renderer.SubmitWorldMesh(large);
            Check(renderer.End(), "Publish incremental geometry frame");
            Check(renderer.PreparedVertices() <= 8192 * 3 && renderer.UploadedBytes() <= 4 * 1024 * 1024,
                  "Large batch bypassed preparation/upload burst limit");
            prepared += renderer.PreparedVertices();
            uploaded += renderer.UploadedBytes();
            if (renderer.UploadedBytes()) ++uploadFrames;
            if (uploaded < expected)
            {
                Check(renderer.ReadyBuffers() == 0, "Partially uploaded mesh became drawable");
                if (renderer.UploadedBytes())
                {
                    const auto color = pixel();
                    Check(color[0] == 20 && color[1] == 30 && color[2] == 50,
                          "Partially uploaded geometry was rendered");
                }
            }
            else completePixel = pixel();
            Check(SDL_RenderPresent(target), "Present incremental geometry frame");
        }
        std::cout << "Large GPU batch: " << prepared / 3 << " tris prepared over " << frames
                  << " frames; " << uploaded << " bytes in " << uploadFrames << " upload frames\n";
        Check(renderer.ReadyBuffers() == 1 && prepared == grid->indices.size() && uploaded == expected &&
              frames >= 4 && uploadFrames >= 2, "Incremental mesh failed to finish exactly once");
        Check(SDL_RenderClear(target) && renderer.Begin(target, camera, &lights), "Begin reference frame");
        renderer.SubmitMesh(large);
        Check(renderer.End(), "Draw reference frame");
        Check(pixel() == completePixel && completePixel[0] > 50,
              "Completed upload differs from the immediate mesh renderer");
        Check(SDL_RenderPresent(target), "Present reference frame");
        renderer.Forget(&large);
        draw(false);
        Check(renderer.ResidentBytes() == 0, "Incremental buffer survived explicit unload");

        // Cancellation during preparation must not resurrect the old mesh.
        Check(renderer.Begin(target, camera, &lights), "Begin cancelled preparation");
        renderer.SubmitWorldMesh(large);
        Check(renderer.End(), "End cancelled preparation");
        renderer.Forget(&large);
        draw(false);
        Check(renderer.ResidentBytes() == 0, "Cancelled preparation retained memory");
        for (int i = 0; i < 200; ++i)
        {
            Check(renderer.Begin(target, camera, &lights), "Begin cancelled upload");
            renderer.SubmitWorldMesh(large);
            Check(renderer.End() && SDL_RenderPresent(target), "End cancelled upload");
            if (renderer.UploadedBytes()) break;
        }
        Check(renderer.UploadedBytes() > 0 && !renderer.ReadyBuffers(), "Expected a partial upload to cancel");
        renderer.Forget(&large);
        draw(false);
        Check(renderer.ResidentBytes() == 0, "Cancelled upload retained memory");
    }
    SDL_DestroyRenderer(target);
    SDL_DestroyWindow(window);
    SDL_Quit();
}

} // namespace
int main(int argc, char **argv)
{
    try
    {
        if (argc > 1 && std::string(argv[1]) == "--gpu")
        {
            GpuCaches();
            std::cout << "GPU cache tests passed\n";
            return 0;
        }
        Fixture fixture;
        Residency(fixture);
        CancellationAndFailure(fixture);
        Models(fixture);
        SharedTextures();
        std::cout << "Streaming tests passed: bounded loading, seams, LOD, budgets, pawn pins, teleport cancellation, "
                     "errors, shutdown, shared models, read-only runtime.\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
