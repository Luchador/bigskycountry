#include "world_package.hpp"
#include "cooked_world.hpp"
#include "prop.hpp"
#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <set>
#include <stdexcept>

namespace
{
    void Check(bool value, const char* message)
    {
        if (!value) throw std::runtime_error(std::string(message) + ": " + SDL_GetError());
    }
    std::vector<char> Bytes(const std::filesystem::path& path)
    {
        std::ifstream in(path, std::ios::binary);
        return {std::istreambuf_iterator<char>(in), {}};
    }
    void Put(const std::filesystem::path& path, const std::vector<char>& bytes)
    {
        std::ofstream out(path, std::ios::binary); out.write(bytes.data(), bytes.size());
    }
    double Area(const Mesh& mesh)
    {
        double area = 0;
        for (std::size_t i = 0; i < mesh.indices.size(); i += 3)
        {
            auto a = mesh.vertices[mesh.indices[i]].position, b = mesh.vertices[mesh.indices[i + 1]].position, c = mesh.vertices[mesh.indices[i + 2]].position;
            area += Math::Length(Math::Cross(b - a, c - a)) * .5;
        }
        return area;
    }
    using Point = std::array<double, 3>;
    std::set<Point> Rim(const Mesh& m)
    {
        std::map<std::pair<unsigned, unsigned>, int> edges;
        for (std::size_t i = 0; i < m.indices.size(); i += 3)
            for (int j = 0; j < 3; ++j) ++edges[std::minmax(m.indices[i+j], m.indices[i+(j+1)%3])];
        std::set<Point> points;
        for (auto [edge, count] : edges) if (count == 1)
            for (auto i : {edge.first, edge.second}) { auto p = m.vertices[i].position; points.insert({p.x,p.y,p.z}); }
        return points;
    }
    void Geometry()
    {
        std::map<WorldPackage::CellKey, WorldPackage::Cell> cells;
        const std::array<MeshVertex, 3> triangle{{{{-600,10,-300},{0,1,0},{0,0}}, {{900,30,-100},{0,1,0},{1,0}}, {{200,50,800},{0,1,0},{0,1}}}};
        WorldPackage::ClipTriangle(cells, {SceneKind::Road,1}, WorldPackage::Layer::Surface, {}, triangle);
        const double expected = Math::Length(Math::Cross(triangle[1].position-triangle[0].position,triangle[2].position-triangle[0].position))*.5;
        double area = 0;
        Check(cells.size() > 4, "Triangle must cross positive and negative cells");
        for (const auto& [key, cell] : cells) for (const auto& batch : cell.batches)
        {
            area += Area(*batch.nearMesh.mesh);
            for (const auto& v : batch.nearMesh.mesh->vertices)
            {
                Check(v.position.x >= 0 && v.position.x <= 512 && v.position.z >= 0 && v.position.z <= 512, "Clipped vertex outside cell");
                const auto reconstructed = triangle[0].position + (triangle[1].position-triangle[0].position)*v.uv.x + (triangle[2].position-triangle[0].position)*v.uv.y;
                Check(Math::Length(reconstructed - batch.nearMesh.transform.Point(v.position)) < .001, "Clipping lost interpolated UVs");
            }
        }
        Check(std::abs(area-expected)<1e-7*expected, "Cell clipping lost or duplicated triangle area");
        Mesh grid;
        constexpr int side = 17;
        for (int z=0;z<side;++z) for (int x=0;x<side;++x)
            grid.vertices.push_back({{x*32.0, .02*x*32.0, z*32.0},{0,1,0},{float(x),float(z)}});
        for (int z=0;z<side-1;++z) for (int x=0;x<side-1;++x)
        { auto a=unsigned(z*side+x); grid.indices.insert(grid.indices.end(),{a,a+side,a+1,a+1,a+side,a+side+1}); }
        grid.UpdateBounds();
        const auto low = WorldPackage::Simplify(grid);
        Check(low->indices.size() < grid.indices.size()/2, "Distant LOD did not reduce interior triangles");
        Check(Rim(*low) == Rim(grid), "Distant LOD changed the shared cell rim");
        Check(std::abs(Area(*low)-Area(grid))<1e-8, "Distant LOD lost planar surface area");
        auto hole=grid; hole.indices.clear();
        for(std::size_t i=0;i<grid.indices.size();i+=3)
        {
            Vector3 center{};for(int j=0;j<3;++j) center=center+grid.vertices[grid.indices[i+j]].position*(1.0/3.0);
            if(center.x>192 && center.x<320 && center.z>192 && center.z<320) continue;
            hole.indices.insert(hole.indices.end(),grid.indices.begin()+i,grid.indices.begin()+i+3);
        }
        const auto lowHole=WorldPackage::Simplify(hole);
        Check(Rim(*lowHole)==Rim(hole), "Distant LOD changed an interior hole boundary");
        Check(std::abs(Area(*lowHole)-Area(hole))<1e-8, "Distant LOD filled an intended hole");
        for (auto v: low->vertices) Check(std::abs(v.position.y-.02*v.position.x)<1e-10, "LOD altered the planar slope");
    }
    Road RoadAt(WorldDocument& world, Vector3 a, Vector3 b, bool rail = false)
    {
        Road road; road.id=world.AllocateRoadId();
        road.points = {RoadPoint{a},RoadPoint{b}};
        if (rail) road.style.railroad = RailroadSettings{};
        road.Rebuild(); return road;
    }
    void RoundTrip(const std::filesystem::path& root)
    {
        auto source=(root/"world.wld").string(), package=(root/"world.bscworld").string();
        WorldDocument world(AssetRoot::Project); Check(world.Open(source.c_str()), "Open fresh world");
        world.roads.push_back(RoadAt(world, {-700,8,10},{1200,28,100}));
        world.roads.push_back(RoadAt(world, {400,0,-700},{400,0,800},true));
        const int junction=world.CreateIntersection({50,0,900});
        world.roads[0].points.front().anchor = IntersectionAnchor{junction,0};
        world.roads[0].points.front().position = world.intersections.at(junction).AnchorPosition(0);
        for (int side : {1,2})
        {
            const auto at=world.intersections.at(junction).AnchorPosition(side);
            auto road=RoadAt(world,at,at+IntersectionAnchor::directions[side]*100);
            road.points.front().anchor=IntersectionAnchor{junction,side}; world.roads.push_back(std::move(road));
        }
        world.UpdateIntersections();
        Check(world.PlaceLake(LakeDefinition::Circle(),ObjectTransform{{520,0,520}}), "Place lake");
        const auto lakeId=world.Terrain().back()->persistentId;
        // Exercise persistent IDs independently from model loading.
        auto object=std::make_unique<Prop>();
        object->SetMesh(world.models.Load("terrain/test_grid.glb"));
        Check(object->IsLoaded(), "Load shared test model");
        Check(world.InsertObject(0,std::move(object),0), "Insert object");
        const auto objectId=world.Objects()[0]->persistentId;
        auto duplicate=world.Objects()[0]->Clone(); duplicate->persistentId=0;
        Check(world.InsertObject(1,std::move(duplicate),0), "Duplicate object");
        Check(world.Objects()[1]->persistentId != objectId, "Duplicate reused identity");
        auto removed=world.TakeObject(0); Check(world.InsertObject(1,std::move(removed),0), "Undo object deletion");
        Check(world.Objects()[1]->persistentId==objectId, "Undo changed identity");
        Check(world.Save(), "Save IDs");
        WorldDocument restored(AssetRoot::Project); Check(restored.Open(source.c_str()), "Reload IDs");
        Check(restored.Terrain()[0]->persistentId==lakeId && restored.Objects()[1]->persistentId==objectId, "IDs changed across reload");
        // Simulate a legacy world by removing only the optional EIDS section.
        auto legacy=Bytes(source);
        auto u32=[&](std::size_t p) { unsigned v=0; for(int i=0;i<4;++i) v|=unsigned(static_cast<unsigned char>(legacy[p+i]))<<(i*8); return v; };
        Check(std::string(legacy.data()+12,4)=="EIDS", "Expected first identity section");
        std::uint64_t identityBytes=0; for(int i=0;i<8;++i) identityBytes|=std::uint64_t(static_cast<unsigned char>(legacy[20+i]))<<(i*8);
        const auto sections=u32(8)-1; for(int i=0;i<4;++i) legacy[8+i]=char((sections>>(i*8))&255);
        legacy.erase(legacy.begin()+12,legacy.begin()+28+identityBytes); const auto legacyPath=(root/"legacy.wld").string(); Put(legacyPath,legacy);
        WorldDocument old(AssetRoot::Project); Check(old.Open(legacyPath.c_str()) && old.Save(), "Upgrade legacy identities");
        const auto oldId=old.Terrain()[0]->persistentId;
        WorldDocument upgraded(AssetRoot::Project); Check(upgraded.Open(legacyPath.c_str()) && upgraded.Terrain()[0]->persistentId==oldId, "Legacy upgrade did not persist IDs");
        // Remove attachments before testing deletion of their junction.
        for(auto& road:restored.roads) for(auto& point:road.points) point.anchor.reset();
        const int removedRoad=restored.roads.back().id; restored.roads.pop_back(); restored.intersections.erase(junction);
        const auto removedObject=restored.TakeObject(0)->persistentId;
        Check(restored.Save(), "Save deleted highest IDs");
        WorldDocument again(AssetRoot::Project); Check(again.Open(source.c_str()), "Reload deleted IDs");
        Check(again.AllocateRoadId()>removedRoad && again.CreateIntersection({})>junction && again.AllocatePlacementId()>removedObject, "Deleted ID was reused after restart");
        // Cook twice: deterministic package and persistent source IDs.
        const auto sourceBefore=Bytes(source);
        Check(!WorldPackage::Cook(world,source) && Bytes(source)==sourceBefore, "Cook overwrote its source");
        Check(WorldPackage::Cook(world, package), "Cook fixture"); const auto bytes=Bytes(package);
        Check(WorldPackage::Cook(world, package), "Recook fixture"); Check(Bytes(package)==bytes, "Identical world produced different cooked bytes");
        WorldPackage::Reader reader; Check(reader.Open(package), "Open directory"); Check(reader.Matches(source,AssetRoot::Project), "Valid package rejected");
        Check(reader.Index().entities.size()==8, "Entity catalog omitted records");
        const auto intersection=std::find_if(reader.Index().entities.begin(),reader.Index().entities.end(),[&](const auto& e) {return e.id==EntityId{SceneKind::Intersection,std::uint64_t(junction)};});
        Check(intersection!=reader.Index().entities.end() && intersection->dependencies.size()==3, "Intersection dependency metadata incomplete");
        Check(reader.Index().assets.size()>=2, "Shared model/material dependencies missing");
        Check(reader.Index().cells.size()>4, "Long roads were not chunked");
        std::uint64_t near=0,far=0,ties=0; bool water=false;
        for (std::size_t i=0;i<reader.Index().cells.size();++i)
        {
            WorldPackage::Cell cell; Check(reader.ReadCell(i,cell), "Read independent cell");
            for (const auto& b:cell.batches)
            { near+=b.nearMesh.mesh->indices.size(); far+=b.distantMesh.mesh->indices.size(); water|=b.layer==WorldPackage::Layer::Water; }
            for (const auto& run:cell.ties) ties+=run.run.count;
        }
        Check(water && ties>0 && far<near, "Missing lake/railroad/LOD data");
        const auto bad=(root/"bad.bscworld").string(); auto corrupt=bytes; corrupt.back()^=1; Put(bad,corrupt);
        WorldPackage::Reader damaged; Check(damaged.Open(bad), "Directory should be readable without loading payload");
        WorldPackage::Cell cell; Check(!damaged.ReadCell(damaged.Index().cells.size()-1,cell), "Corrupt cell accepted");
        corrupt=bytes; corrupt[28]^=1; Put(bad,corrupt); Check(!damaged.Open(bad), "Corrupt manifest accepted");
        corrupt=bytes; corrupt.resize(20); Put(bad,corrupt); Check(!damaged.Open(bad), "Truncated header accepted");
        corrupt=bytes; corrupt[8]=99; Put(bad,corrupt); Check(!damaged.Open(bad), "Unknown version accepted");
        Check(world.PlaceLake(LakeDefinition::Circle(20)), "Add source edit"); Check(world.Save(), "Save source edit");
        Check(!reader.Matches(source,AssetRoot::Project), "Stale package accepted");
        // Failed cooking must retain the existing package and reject unfinished shoreline drafts.
        Check(world.Terrain().back()->BeginLakeEdit(), "Begin lake edit");
        Check(!WorldPackage::Cook(world,package) && Bytes(package)==bytes, "Failed cook replaced valid package");
        std::cout << "Cooked fixture: " << reader.Index().cells.size() << " cells, " << near/3 << " near and " << far/3 << " distant triangles, " << ties << " ties\n";
    }
}
int main()
{
    SDL_Time now; SDL_GetCurrentTime(&now);
    const auto temp=std::filesystem::temp_directory_path()/ ("bigsky-package-test-"+std::to_string(now));
    try
    {
        std::filesystem::create_directories(temp); Geometry(); RoundTrip(temp);
        std::filesystem::remove_all(temp); std::cout << "World package tests passed\n"; return 0;
    }
    catch (const std::exception& e) { std::cerr << e.what() << "\nFixture: " << temp << '\n'; return 1; }
}
