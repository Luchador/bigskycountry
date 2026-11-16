#include "mesh_materials.hpp"
#include "world_package.hpp"
#include "world_renderer.hpp"
#include "camera_view.hpp"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
    void Check(bool value, const char* message)
    { if (!value) throw std::runtime_error(std::string(message) + ": " + SDL_GetError()); }
    void Put(const std::string& path, const std::string& data)
    { std::ofstream out(path, std::ios::binary); out.write(data.data(), data.size()); Check(bool(out), "Write fixture"); }
    std::string Bytes(const std::string& path)
    { std::ifstream in(path, std::ios::binary); return {std::istreambuf_iterator<char>(in), {}}; }
    void U32(std::string& bytes, std::uint32_t value)
    { for (int i = 0; i < 4; ++i) bytes.push_back(char(value >> (i * 8))); }
    void Model(const std::string& path)
    {
        std::string json = R"({"asset":{"version":"2.0"},"buffers":[{"byteLength":96}],
"bufferViews":[{"buffer":0,"byteOffset":0,"byteLength":36},{"buffer":0,"byteOffset":36,"byteLength":36},{"buffer":0,"byteOffset":72,"byteLength":24}],
"accessors":[{"bufferView":0,"componentType":5126,"count":3,"type":"VEC3","min":[-0.8,-1,0],"max":[0.8,1,0]},
{"bufferView":1,"componentType":5126,"count":3,"type":"VEC3"},{"bufferView":2,"componentType":5126,"count":3,"type":"VEC2"}],
"materials":[{"name":"pole","doubleSided":true},{"name":"front"},{"name":"bolts","alphaMode":"BLEND"}],
"meshes":[{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2},"material":0}]},
{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2},"material":1}]},
{"primitives":[{"attributes":{"POSITION":0,"NORMAL":1,"TEXCOORD_0":2},"material":2}]}],
"nodes":[{"mesh":0,"translation":[-2,0,0]},{"mesh":1},{"mesh":2,"translation":[2,0,0]}],
"scenes":[{"nodes":[0,1,2]}],"scene":0})";
        while (json.size() % 4) json += ' ';
        const float data[]{-.8f,-1,0, .8f,-1,0, 0,1,0, 0,0,1, 0,0,1, 0,0,1, 0,0, 1,0, .5f,1};
        std::string bytes;
        U32(bytes, 0x46546c67); U32(bytes, 2); U32(bytes, std::uint32_t(28 + json.size() + sizeof(data)));
        U32(bytes, std::uint32_t(json.size())); U32(bytes, 0x4e4f534a); bytes += json;
        U32(bytes, sizeof(data)); U32(bytes, 0x004e4942); bytes.append(reinterpret_cast<const char*>(data), sizeof(data));
        Put(path, bytes);
    }
    void Render(const std::shared_ptr<const Mesh>& source, const std::string& folder, const char* driver)
    {
        Check(SDL_Init(SDL_INIT_VIDEO), "Initialize rendering test");
        std::unique_ptr<SDL_Window, decltype(&SDL_DestroyWindow)> window(
            SDL_CreateWindow("Mesh material render test", 600, 300, 0), SDL_DestroyWindow);
        Check(bool(window), "Create rendering window");
        std::unique_ptr<SDL_Renderer, decltype(&SDL_DestroyRenderer)> renderer(
            SDL_CreateRenderer(window.get(), driver), SDL_DestroyRenderer);
        Check(bool(renderer), "Create renderer");
        auto mesh = std::make_shared<Mesh>(*source);
        const SDL_FColor colors[]{{1,0,0,1}, {0,1,0,1}, {0,0,1,1}};
        for (const auto& part : mesh->parts)
            for (auto i = part.firstIndex; i < part.firstIndex + part.indexCount; ++i)
                mesh->vertices[mesh->indices[i]].color = colors[part.slot];
        MeshInstance instance{mesh, {}};
        WorldRenderer scene(AssetRoot::Cooked);
        CameraView camera; camera.LookAt({0,0,-5}, {});
        const WorldLighting lights{{{0,0,-1},{}}, {{0,1,0},{}}, {1,1,1}};
        const auto white = folder + "/white.bmat", red = folder + "/red.bmat";
        Put(AssetPaths::Root(AssetRoot::Cooked) + "textures/" + white, R"({"version":1,"type":"standard"})");
        auto draw = [&](const std::string& overrideMaterial = std::string{}) {
            for (int i = 0; i < 8; ++i)
            {
                SDL_SetRenderDrawColor(renderer.get(), 0,0,0,255); SDL_RenderClear(renderer.get());
                Check(scene.Begin(renderer.get(), camera, &lights), "Begin render");
                scene.SubmitWorldMesh(instance, overrideMaterial);
                Check(scene.End(), "Render slot materials");
                Check(SDL_RenderPresent(renderer.get()), "Present frame");
            }
            SDL_SetRenderDrawColor(renderer.get(), 0,0,0,255); SDL_RenderClear(renderer.get());
            Check(scene.Begin(renderer.get(), camera, &lights), "Begin readback");
            scene.SubmitWorldMesh(instance, overrideMaterial); Check(scene.End(), "Finish readback");
            std::unique_ptr<SDL_Surface, decltype(&SDL_DestroySurface)> pixels(
                SDL_RenderReadPixels(renderer.get(), nullptr), SDL_DestroySurface);
            Check(bool(pixels), "Read rendered pixels");
            std::array<SDL_Color, 3> samples{};
            for (int i = 0; i < 3; ++i)
                Check(SDL_ReadSurfacePixel(pixels.get(), 240 + 60 * i + int(instance.transform.origin.x * 30), 150,
                      &samples[i].r, &samples[i].g, &samples[i].b, &samples[i].a), "Read material sample");
            return samples;
        };
        auto redPixel = [](SDL_Color c) { return c.r > 100 && c.g < 20 && c.b < 20; };
        auto bluePixel = [](SDL_Color c) { return c.b > 100 && c.r < 20 && c.g < 20; };
        Check(MeshMaterialIO::Assign(*mesh, 1, white), "Assign opaque render material");
        auto pixels = draw();
        Check(redPixel(pixels[0]) && bluePixel(pixels[2]) && pixels[1].r > 100 && pixels[1].g > 100 && pixels[1].b > 100,
              "Resident material ranges leaked into neighboring primitives");
        if (std::string(SDL_GetRendererName(renderer.get())) == "gpu") Check(scene.ReadyBuffers() == 1, "Opaque slots were not retained on GPU");
        Check(MeshMaterialIO::Assign(*mesh, 1, {}), "Clear resident material"); pixels = draw();
        Check(pixels[1].g > 100 && pixels[1].r < 20 && pixels[1].b < 20, "Clearing slot did not refresh resident geometry");
        Check(MeshMaterialIO::Assign(*mesh, 1, red), "Assign unlit render material"); pixels = draw();
        Check(redPixel(pixels[0]) && redPixel(pixels[1]) && bluePixel(pixels[2]), "Dynamic/unlit material ranges incorrect");
        instance.transform.origin.x = 1; pixels = draw();
        Check(redPixel(pixels[0]) && redPixel(pixels[1]) && bluePixel(pixels[2]), "Moving mesh lost material ranges");
        pixels = draw(red);
        Check(redPixel(pixels[0]) && redPixel(pixels[1]) && redPixel(pixels[2]), "Instance override did not replace slot materials");
        Material translucent;
        const auto redFile = AssetPaths::Root(AssetRoot::Cooked) + "textures/" + red;
        Check(translucent.Load(redFile), "Read translucent material");
        translucent.translucency = .5f;
        Check(translucent.Save(redFile), "Change translucency"); pixels = draw();
        Check(pixels[1].r > 60 && pixels[1].r < 245 && pixels[1].g < 20 && pixels[1].b < 20 &&
              redPixel(pixels[0]) && bluePixel(pixels[2]), "Slot translucency or live material update failed");
    }
    void Run(const std::string& folder, const char* driver)
    {
        const auto root = AssetPaths::Root(AssetRoot::Cooked);
        const auto modelPath = folder + "/sample.glb", materialPath = folder + "/red.bmat";
        const auto file = root + "models/" + modelPath, sidecar = file + ".bmesh";
        Model(file);
        Put(root + "textures/" + materialPath, R"({"version":1,"type":"unlit","emissiveTint":[1,0,0]})");
        const auto original = Bytes(file);
        const auto oldHash = WorldPackage::AssetHash(WorldPackage::AssetKind::Model, modelPath, AssetRoot::Cooked);
        Check(oldHash == WorldPackage::FileHash(file), "Legacy model hash changed");
        {
            MeshAssets a(AssetRoot::Cooked), b(AssetRoot::Cooked);
            const auto mesh = a.Load(modelPath), other = b.Load(modelPath);
            Check(mesh && other, "Import fixture");
            Check(mesh->materials == other->materials, "Imports do not share slot assignments");
            Check(mesh->materials->slots.size() == 3 && mesh->materials->slots[1].name == "front", "GLTF slot names lost");
            Check(mesh->parts.size() == 3 && mesh->parts[0].firstIndex == 0 && mesh->parts[0].indexCount == 6 &&
                  mesh->parts[1].firstIndex == 6 && mesh->parts[1].indexCount == 3 && mesh->parts[2].firstIndex == 9 &&
                  mesh->indices.size() == 12, "Primitive ranges/back faces/transparent primitive lost");
            Check(!mesh->HasMaterials(), "Unassigned mesh has material override");
            const auto revision = mesh->materials->revision;
            Check(MeshMaterialIO::Assign(*mesh, 1, materialPath), "Assign material");
            Check(mesh->materials->revision > revision && other->MaterialAt(6) == materialPath, "Live shared binding not updated");
            Check(mesh->MaterialAt(0).empty() && mesh->MaterialAt(5).empty() && mesh->MaterialAt(8) == materialPath &&
                  mesh->MaterialAt(9).empty() && mesh->MaterialAt(12).empty(), "Material escaped its primitive range");
            const auto saved = Bytes(sidecar);
            Check(!MeshMaterialIO::Assign(*mesh, 4, materialPath) && !MeshMaterialIO::Assign(*mesh, 0, "../bad.bmat") &&
                  !MeshMaterialIO::Assign(*mesh, 0, folder + "/missing.bmat") && Bytes(sidecar) == saved,
                  "Rejected assignment changed saved data");
            Check(WorldPackage::AssetHash(WorldPackage::AssetKind::Model, modelPath, AssetRoot::Cooked) != oldHash,
                  "Cook hash omitted material assignments");
            WorldDocument world(AssetRoot::Cooked);
            const auto source = root + "models/" + folder + "/world.wld", cooked = source + ".bscworld";
            Check(world.Open(source.c_str()), "Open fixture world");
            world.meshes.push_back({mesh, {}});
            Check(world.Save() && WorldPackage::Cook(world, cooked), "Cook mesh world");
            WorldPackage::Reader reader;
            Check(reader.Open(cooked) && reader.Matches(source, AssetRoot::Cooked), "Open cooked mesh world");
            bool foundMaterial = false;
            for (const auto& asset : reader.Index().assets)
                if (asset.kind == WorldPackage::AssetKind::Material && asset.path == materialPath) foundMaterial = true;
            Check(foundMaterial, "Cook manifest omitted slot material dependency");
            Check(MeshMaterialIO::Assign(*mesh, 1, {}), "Clear slot");
            Check(!mesh->HasMaterials() && !reader.Matches(source, AssetRoot::Cooked), "Clearing slot failed to invalidate cooked world");
            Check(MeshMaterialIO::Assign(*mesh, 1, materialPath), "Reassign slot");
        }
        MeshAssets fresh(AssetRoot::Cooked);
        const auto reopened = fresh.Load(modelPath);
        Check(reopened && reopened->MaterialAt(6) == materialPath, "Assignment lost after cache eviction/reopen");
        Check(Bytes(file) == original, "Assignment modified original GLB");
        if (driver) Render(reopened, folder, driver);
        std::vector<MeshMaterialSlot> reordered{{"front", {}}, {"bolts", {}}, {"pole", {}}};
        Check(MeshMaterialIO::Load(sidecar, reordered) && reordered[0].material == materialPath && reordered[1].material.empty(),
              "Named slot reorder lost assignments");
        Put(sidecar, "BSC_MESH_MATERIALS 1\n2\n\"front\" \"\"\ninvalid");
        Check(!MeshMaterialIO::Load(sidecar, reordered) && reordered[0].material == materialPath,
              "Malformed file partially changed assignments");
    }
}
int main(int argc, char** argv)
{
    SDL_Time now; SDL_GetCurrentTime(&now);
    const auto folder = "mesh-material-test-" + std::to_string(now);
    const auto root = AssetPaths::Root(AssetRoot::Cooked);
    int result = 0;
    try
    {
        std::filesystem::create_directories(root + "models/" + folder);
        std::filesystem::create_directories(root + "textures/" + folder);
        const char* driver = argc > 1 ? (std::string(argv[1]) == "--software" ? "software" : "gpu") : nullptr;
        Run(folder, driver);
        std::cout << "Mesh material tests passed\n";
    }
    catch (const std::exception& e) { std::cerr << e.what() << '\n'; result = 1; }
    std::error_code ignored;
    std::filesystem::remove_all(root + "models/" + folder, ignored);
    std::filesystem::remove_all(root + "textures/" + folder, ignored);
    SDL_Quit();
    return result;
}
