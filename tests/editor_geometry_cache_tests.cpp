#include "world_document.hpp"

#include <fstream>
#include <iostream>
#include <stdexcept>

namespace
{
    void Check(bool ok, const char* message)
    { if (!ok) throw std::runtime_error(std::string(message) + ": " + SDL_GetError()); }

    void Equal(const Mesh& a, const Mesh& b)
    {
        Check(a.indices == b.indices && a.vertices.size() == b.vertices.size() && a.bounds == b.bounds, "Mesh topology/bounds changed");
        for (std::size_t i = 0; i < a.vertices.size(); ++i)
        {
            const auto& x = a.vertices[i]; const auto& y = b.vertices[i];
            Check(x.position == y.position && x.normal == y.normal && x.uv.x == y.uv.x && x.uv.y == y.uv.y &&
                  x.color.r == y.color.r && x.color.g == y.color.g && x.color.b == y.color.b && x.color.a == y.color.a,
                  "Cache changed mesh precision or vertex attributes");
        }
    }

    void Equal(const WorldDocument& a, const WorldDocument& b)
    {
        Check(a.Terrain().size() == b.Terrain().size(), "Terrain count changed");
        for (std::size_t i = 0; i < a.Terrain().size(); ++i)
        {
            const auto& x = *a.Terrain()[i]; const auto& y = *b.Terrain()[i];
            Check(x.Instance().mesh && y.Instance().mesh, "Missing terrain mesh");
            Equal(*x.Instance().mesh, *y.Instance().mesh);
            Check(x.material == y.material && x.GetTransform() == y.GetTransform() && x.boundary == y.boundary &&
                  x.persistentId == y.persistentId && x.WaterSource() == y.WaterSource(), "Cache changed authored terrain");
            if (x.HasWater())
            {
                Equal(*x.Water().mesh, *y.Water().mesh);
                const auto& xs = x.BuiltShoreline(); const auto& ys = y.BuiltShoreline();
                Check(xs.size() == ys.size(), "Shoreline count changed");
                for (std::size_t j = 0; j < xs.size(); ++j)
                    Check(xs[j].position == ys[j].position && xs[j].segment == ys[j].segment &&
                          xs[j].t == ys[j].t && xs[j].bank == ys[j].bank, "Shoreline seam/edit metadata changed");
            }
        }
    }

    void Populate(WorldDocument& world)
    {
        TerrainBoundary boundary;
        const Vector3 origin{500000, 37, -300000};
        const std::array<Vector3, 5> corners{{{-60,0,-60},{60,0,-60},{60,0,60},{-60,0,60},{-60,0,-60}}};
        for (int i = 0; i < 4; ++i)
        {
            Road road; road.id = world.AllocateRoadId(); road.style.curved = false;
            road.points = {{origin + corners[i]}, {origin + corners[i+1]}}; road.Rebuild();
            boundary.roads.insert(road.id); world.roads.push_back(std::move(road));
        }
        auto* lake = world.PlaceLake(LakeDefinition::Circle(40), ObjectTransform{origin});
        Check(lake, "Place lake"); lake->material = "test/bed.bmat";
        auto river = RiverDefinition::Straight();
        for (auto& bank : river.banks) bank.back().position.y += 3;
        Check(world.PlaceRiver(river, ObjectTransform{origin + Vector3{300,0,0}}), "Place sloping river");
        Check(world.PlaceTerrainFill(boundary), "Fill around lake");
    }

    std::vector<std::filesystem::path> Files(const std::filesystem::path& directory)
    {
        std::vector<std::filesystem::path> files;
        for (const auto& entry : std::filesystem::directory_iterator(directory))
            if (entry.path().extension() == ".bgeom") files.push_back(entry.path());
        return files;
    }

    void RoundTrip(const std::filesystem::path& root)
    {
        const auto file = (root / "world.wld").string(); const auto directory = root / "cache";
        WorldDocument source; Check(source.Open(file.c_str()), "Open blank world"); Populate(source); Check(source.Save(), "Save source");
        auto cold = std::make_shared<EditorGeometryCache>(directory);
        WorldDocument first(AssetRoot::Cooked, cold); Check(first.Open(file.c_str()), "Cold open"); Equal(source, first);
        Check(cold->Stats().misses == 3 && cold->Stats().writes == 3 && cold->Stats().errors == 0, "Cold population failed");
        auto warm = std::make_shared<EditorGeometryCache>(directory);
        WorldDocument second(AssetRoot::Cooked, warm); Check(second.Open(file.c_str()), "Warm open"); Equal(first, second);
        Check(warm->Stats().hits == 3 && warm->Stats().misses == 0, "Fresh cache instance missed unchanged world");

        const auto unchanged = second.Terrain().back()->Instance().mesh;
        second.RefreshTerrainFills(); Check(second.Terrain().back()->Instance().mesh == unchanged, "Idle terrain rebuilt");
        auto* lake = second.Terrain()[0].get(); const auto old = *lake->Lake(); auto edited = old; edited.depth += 2;
        Check(lake->BeginWaterEdit() && lake->SetLake(edited), "Begin water edit");
        Check(warm->Stats().writes == 0, "Draft wrote geometry");
        Check(lake->FinishWaterEdit(), "Finish water edit");
        Check(warm->Stats().writes == 1, "Water depth did not invalidate cache");
        second.RefreshTerrainFills(); Check(second.Terrain().back()->Instance().mesh != nullptr, "Lost fill after water edit");
        // An unchanged shoreline lets the fill be reused despite a new water revision.
        Check(warm->Stats().hits == 4, "Unchanged rim missed terrain cache");
        Check(lake->SetLake(old) && warm->Stats().hits == 5, "Undo did not reuse prior water geometry");

        auto moved = old; Check(moved.MovePoint(0, moved.points[0].position + Vector3{4,0,0}), "Move lake control");
        Check(lake->SetLake(moved), "Change shoreline"); second.RefreshTerrainFills();
        Check(warm->Stats().writes == 3, "Changed shoreline did not rebuild water and fill");
        WorldDocument uncached; Populate(uncached);
        Check(uncached.Terrain()[0]->SetLake(moved), "Change reference shoreline"); uncached.RefreshTerrainFills();
        Equal(uncached, second);
        const auto hits = warm->Stats().hits;
        for (auto& p : second.roads[0].points) p.position.y += 1;
        second.roads[0].Rebuild(); second.RefreshTerrainFills();
        Check(warm->Stats().hits == hits, "Road elevation change reused stale fill");

        // Every corrupted entry must be regenerated, without touching the world file.
        for (const auto& path : Files(directory)) { std::ofstream out(path, std::ios::binary | std::ios::trunc); out << "broken"; }
        auto damaged = std::make_shared<EditorGeometryCache>(directory);
        WorldDocument repaired(AssetRoot::Cooked, damaged); Check(repaired.Open(file.c_str()), "Recover truncated entries"); Equal(first, repaired);
        Check(damaged->Stats().errors == 3 && damaged->Stats().writes == 3, "Corruption was not repaired");
        // Header versions and payload checksums are independently checked.
        for (const auto& path : Files(directory))
        {
            if (std::filesystem::file_size(path) < 40) continue;
            std::fstream io(path, std::ios::binary | std::ios::in | std::ios::out); io.seekp(8); io.put(99);
        }
        auto stale = std::make_shared<EditorGeometryCache>(directory);
        WorldDocument refreshed(AssetRoot::Cooked, stale); Check(refreshed.Open(file.c_str()), "Recover stale entries"); Equal(first, refreshed);
        Check(stale->Stats().errors == 3 && stale->Stats().writes == 3, "Stale header accepted");
        for (const auto& path : Files(directory))
        {
            if (std::filesystem::file_size(path) < 40) continue;
            std::fstream io(path, std::ios::binary | std::ios::in | std::ios::out); io.seekp(45); io.put(99);
        }
        auto checksum = std::make_shared<EditorGeometryCache>(directory);
        WorldDocument recovered(AssetRoot::Cooked, checksum); Check(recovered.Open(file.c_str()), "Recover checksum failure"); Equal(first, recovered);
        Check(checksum->Stats().errors == 3, "Damaged payload accepted");

        auto blocked = std::make_shared<EditorGeometryCache>(root / "world.wld");
        WorldDocument noDisk(AssetRoot::Cooked, blocked); Check(noDisk.Open(file.c_str()), "Unwritable cache blocked world"); Equal(first, noDisk);
        Check(blocked->Stats().writes == 0 && blocked->Stats().errors == 3, "Unwritable cache not handled");
        auto disabled = std::make_shared<EditorGeometryCache>(root / "disabled", 0);
        WorldDocument noCache(AssetRoot::Cooked, disabled); Check(noCache.Open(file.c_str()), "Disabled cache blocked world"); Equal(first, noCache);
        Check(!std::filesystem::exists(root / "disabled"), "Disabled cache created files");
    }

    void Dependencies(const std::filesystem::path& root)
    {
        RoadGeometry ring;
        ring.outline = {{-60,0,-60},{60,0,-60},{60,0,60},{-60,0,60}};
        ring.holes = {{{-50,0,-50},{50,0,-50},{50,0,50},{-50,0,50}}};
        const std::array<const RoadGeometry*,1> footprints{&ring};
        EditorGeometryCache cache(root / "dependencies");
        std::vector<std::array<Vector3,2>> links, seams;
        std::vector<std::vector<Vector3>> exclusions;
        auto scope = TerrainFillScope::AllPockets;
        std::vector<const RoadGeometry*> supporting;
        const auto fill = [&] { return cache.Fill(footprints, links, exclusions, seams, scope, supporting); };
        Check(fill() != nullptr && fill() != nullptr && cache.Stats().hits == 1, "Fixture did not cache");
        ++ring.revision; Check(fill() != nullptr && cache.Stats().hits == 2, "Transient revision entered persistent key");
        const auto miss = [&]
        {
            const auto before = cache.Stats().misses; fill();
            Check(cache.Stats().misses == before + 1, "Geometry dependency missing from key");
        };
        ring.holes[0][0].y += .5; miss();
        ring.outline[0].y += .5; miss();
        links.push_back({Vector3{0,0,0}, Vector3{0,0,1}}); miss();
        seams = links; miss();
        exclusions.push_back({{-5,0,-5},{5,0,-5},{5,0,5},{-5,0,5}}); miss();
        scope = TerrainFillScope::SelectedBoundary; miss();
        supporting.push_back(&ring); miss();
        ring.crossingSlabs.push_back({{{70,0,70},{80,0,70},{80,0,80},{70,0,80}}, {}}); miss();
        ring.crossingSlabs[0].walls.push_back({Vector3{70,0,70},Vector3{80,0,70},Vector3{80,-1,70},Vector3{70,-1,70}}); miss();
        ring.groundFootprints = ring.crossingSlabs; miss();
        ring.groundFootprints[0].walls[0][0].y += 1; miss();

        // Bounded disk usage evicts disposable entries, and evicted data rebuilds.
        const auto files = Files(root / "dependencies"); Check(files.size() > 1, "No entries for budget test");
        EditorGeometryCache limited(root / "dependencies", 1);
        Check(Files(root / "dependencies").empty(), "Cache disk budget not enforced");
    }

    void InvalidPayload(const std::filesystem::path& root)
    {
        const auto directory = root / "invalid";
        const auto definition = LakeDefinition::Circle(30);
        EditorGeometryCache cache(directory);
        auto expected = cache.Lake(definition); Check(expected.has_value(), "Create validation fixture");
        const auto path = Files(directory).at(0);
        const auto change = [&](int kind)
        {
            std::ifstream in(path, std::ios::binary);
            std::vector<char> bytes((std::istreambuf_iterator<char>(in)), {}); in.close();
            const auto read32 = [&](std::size_t at)
            {
                std::uint32_t v = 0;
                for (int i = 0; i < 4; ++i) v |= std::uint32_t(static_cast<unsigned char>(bytes[at+i])) << (i*8);
                return v;
            };
            const auto write32 = [&](std::size_t at, std::uint32_t v)
            { for (int i = 0; i < 4; ++i) { bytes[at+i] = static_cast<char>(v & 255); v >>= 8; } };
            if (kind == 0) write32(40, UINT32_MAX); // Count exceeds payload; do not allocate it.
            if (kind == 1) write32(48 + std::size_t(read32(40))*72, UINT32_MAX); // Out-of-range triangle index.
            if (kind == 2) write32(52, 0x7ff80000); // Non-finite coordinate.
            if (kind == 3) { bytes.push_back(0); write32(28, static_cast<std::uint32_t>(bytes.size()-40)); }
            write32(36, SDL_crc32(0, bytes.data()+40, bytes.size()-40)); // Deliberately valid checksum.
            std::ofstream out(path, std::ios::binary | std::ios::trunc); out.write(bytes.data(), static_cast<std::streamsize>(bytes.size())); out.close();
            const auto errors = cache.Stats().errors;
            auto actual = cache.Lake(definition);
            Check(actual && cache.Stats().errors == errors + 1, "Malformed mesh with valid checksum accepted");
            Equal(*expected->bed, *actual->bed); Equal(*expected->water, *actual->water);
        };
        for (int i = 0; i < 4; ++i) change(i);
        const auto entryBytes = std::filesystem::file_size(path);
        EditorGeometryCache limited(directory, entryBytes + entryBytes/2);
        auto other = definition; other.depth += 1; Check(limited.Lake(other).has_value(), "Budgeted write failed");
        other.depth += 1; Check(limited.Lake(other).has_value(), "Budgeted eviction failed");
        std::uint64_t bytes = 0; for (const auto& file : Files(directory)) bytes += std::filesystem::file_size(file);
        Check(bytes <= entryBytes + entryBytes/2, "Writes exceeded budget after pruning");
    }
}

int main()
{
    const auto root = std::filesystem::temp_directory_path() / ("bigsky-cache-test-" + std::to_string(SDL_GetPerformanceCounter()));
    try
    {
        std::filesystem::create_directories(root);
        RoundTrip(root); Dependencies(root); InvalidPayload(root);
        std::filesystem::remove_all(root);
        std::cout << "Editor geometry cache tests passed\n";
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::error_code error; std::filesystem::remove_all(root, error); return 1;
    }
}
