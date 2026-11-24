#include "mesh_collision.hpp"
#include "spatial_index.hpp"
#include <cJSON.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <mutex>

namespace
{
    std::mutex mutex;
    std::unordered_map<std::string, std::weak_ptr<MeshCollisionState>> states;
    using Json = std::unique_ptr<cJSON, decltype(&cJSON_Delete)>;
    constexpr const char* modes[]{"detailed", "simple", "none"};
    constexpr const char* types[]{"capsule", "sphere"};
    bool ReadVector(const cJSON* object, const char* key, Vector3& out)
    {
        const auto* a = cJSON_GetObjectItemCaseSensitive(object, key);
        if (!cJSON_IsArray(a) || cJSON_GetArraySize(a) != 3) return false;
        double v[3];
        for (int i = 0; i < 3; ++i)
        {
            const auto* n = cJSON_GetArrayItem(a, i);
            if (!cJSON_IsNumber(n) || !std::isfinite(n->valuedouble)) return false;
            v[i] = n->valuedouble;
        }
        out = {v[0], v[1], v[2]}; return true;
    }
    bool WriteVector(cJSON* object, const char* key, Vector3 v)
    {
        const double values[]{v.x, v.y, v.z};
        Json a(cJSON_CreateDoubleArray(values, 3), cJSON_Delete);
        if (!a || !cJSON_AddItemToObject(object, key, a.get())) return false;
        a.release(); return true;
    }
    int Choice(const cJSON* object, const char* key, const char* const* names, int count)
    {
        const auto* s = cJSON_GetStringValue(cJSON_GetObjectItemCaseSensitive(object, key));
        for (int i = 0; s && i < count; ++i) if (std::strcmp(s, names[i]) == 0) return i;
        return -1;
    }
}

bool MeshCollision::Valid(const MeshCollisionData& data)
{
    if (unsigned(data.mode) > unsigned(MeshCollisionMode::None) || data.shapes.size() > maxShapes) return false;
    for (const auto& s : data.shapes)
    {
        const auto& t = s.transform;
        if (unsigned(s.type) > unsigned(CollisionShapeType::Sphere) || !t.ToMeshTransform().Valid()) return false;
        for (double v : {t.position.x, t.position.y, t.position.z, t.rotation.x, t.rotation.y, t.rotation.z})
            if (!std::isfinite(v) || std::abs(v) > 1e6) return false;
        for (double v : {t.scale.x, t.scale.y, t.scale.z}) if (!(v >= 0.0001 && v <= 1e6)) return false;
        if (t.scale.x != t.scale.z || (s.type == CollisionShapeType::Sphere && t.scale.x != t.scale.y)) return false;
    }
    return true;
}

MeshCollision::Snapshot MeshCollision::Current(const Mesh& mesh)
{
    static const auto defaults = std::make_shared<const MeshCollisionData>();
    return mesh.collision ? mesh.collision->data.load() : defaults;
}

bool MeshCollision::Bind(Mesh& mesh, AssetRoot root)
{
    const auto file = AssetPaths::Root(root) + "models/" + mesh.assetPath + ".bcollision";
    std::lock_guard lock(mutex);
    std::erase_if(states, [](const auto& e) { return e.second.expired(); });
    auto state = states[file].lock();
    if (!state)
    {
        auto data = std::make_shared<MeshCollisionData>();
        std::error_code error;
        const bool exists = std::filesystem::exists(std::filesystem::path(std::u8string(file.begin(), file.end())), error);
        if (error) return SDL_SetError("Cannot inspect mesh collision: %s", error.message().c_str());
        if (exists)
        {
            SDL_PathInfo info{};
            if (!SDL_GetPathInfo(file.c_str(), &info) || info.size > 1024 * 1024) return SDL_SetError("Invalid mesh collision file size");
            size_t size = 0;
            std::unique_ptr<char, decltype(&SDL_free)> bytes(static_cast<char*>(SDL_LoadFile(file.c_str(), &size)), SDL_free);
            if (!bytes) return false;
            Json json(nullptr, cJSON_Delete);
            if (!std::memchr(bytes.get(), 0, size)) json.reset(cJSON_ParseWithLengthOpts(bytes.get(), size + 1, nullptr, true));
            const auto* version = cJSON_GetObjectItemCaseSensitive(json.get(), "version");
            const int mode = Choice(json.get(), "mode", modes, 3);
            const auto* shapes = cJSON_GetObjectItemCaseSensitive(json.get(), "shapes");
            if (!cJSON_IsNumber(version) || version->valuedouble != 1 || mode < 0 || !cJSON_IsArray(shapes) ||
                cJSON_GetArraySize(shapes) > int(maxShapes)) return SDL_SetError("Invalid mesh collision file: %s", file.c_str());
            data->mode = MeshCollisionMode(mode);
            for (int i = 0; i < cJSON_GetArraySize(shapes); ++i)
            {
                const auto* item = cJSON_GetArrayItem(shapes, i);
                const int type = Choice(item, "type", types, 2);
                CollisionShape s;
                if (type < 0 || !ReadVector(item, "position", s.transform.position) ||
                    !ReadVector(item, "rotation", s.transform.rotation) || !ReadVector(item, "scale", s.transform.scale))
                    return SDL_SetError("Invalid collision shape: %s", file.c_str());
                s.type = CollisionShapeType(type); data->shapes.push_back(s);
            }
            if (!Valid(*data)) return SDL_SetError("Invalid collision dimensions: %s", file.c_str());
        }
        state = std::make_shared<MeshCollisionState>(); state->file = file; state->root = root; state->data.store(data);
        states[file] = state;
    }
    mesh.collision = state;
    for (auto& lod : mesh.lods) lod.mesh->collision = state;
    return true;
}

bool MeshCollision::Save(const Mesh& mesh, const MeshCollisionData& data)
{
    if (!mesh.collision || mesh.collision->root != AssetRoot::Project || !Valid(data))
        return SDL_SetError("Invalid editable mesh collision data");
    auto snapshot = std::make_shared<const MeshCollisionData>(data);
    Json json(cJSON_CreateObject(), cJSON_Delete);
    if (!json || !cJSON_AddNumberToObject(json.get(), "version", 1) ||
        !cJSON_AddStringToObject(json.get(), "mode", modes[int(data.mode)])) return SDL_OutOfMemory();
    auto* shapes = cJSON_AddArrayToObject(json.get(), "shapes");
    if (!shapes) return SDL_OutOfMemory();
    for (const auto& s : data.shapes)
    {
        Json item(cJSON_CreateObject(), cJSON_Delete);
        if (!item || !cJSON_AddStringToObject(item.get(), "type", types[int(s.type)]) ||
            !WriteVector(item.get(), "position", s.transform.position) || !WriteVector(item.get(), "rotation", s.transform.rotation) ||
            !WriteVector(item.get(), "scale", s.transform.scale) || !cJSON_AddItemToArray(shapes, item.get())) return SDL_OutOfMemory();
        item.release();
    }
    std::unique_ptr<char, decltype(&cJSON_free)> text(cJSON_Print(json.get()), cJSON_free);
    if (!text) return SDL_OutOfMemory();
    SDL_Time time; if (!SDL_GetCurrentTime(&time)) return false;
    const auto temp = mesh.collision->file + "." + std::to_string(time) + ".tmp";
    auto* out = SDL_IOFromFile(temp.c_str(), "wbx"); if (!out) return false;
    const auto size = std::strlen(text.get());
    bool ok = SDL_WriteIO(out, text.get(), size) == size;
    ok = SDL_CloseIO(out) && ok;
    if (!ok || !SDL_RenamePath(temp.c_str(), mesh.collision->file.c_str()))
    {
        const std::string error = SDL_GetError(); SDL_RemovePath(temp.c_str());
        return SDL_SetError("Cannot save mesh collision: %s", error.c_str());
    }
    mesh.collision->data.store(std::move(snapshot)); return true;
}

CollisionCapsule MeshCollision::Capsule(const CollisionShape& shape, const MeshTransform& placement)
{
    auto pose = shape.transform; pose.scale = {1, 1, 1};
    const auto local = pose.ToMeshTransform();
    const double half = shape.type == CollisionShapeType::Capsule ? shape.transform.scale.y * .5 : 0;
    // Upper bound on the transform's largest singular value; exact for the
    // orthogonal rotation/scale placements used by objects, including mirrors.
    const Vector3 axes[]{placement.x, placement.y, placement.z};
    double scale2 = 0;
    for (auto a : axes)
    {
        double row = 0; for (auto b : axes) row += std::abs(Math::Dot(a, b));
        scale2 = std::max(scale2, row);
    }
    return {placement.Point(local.Point({0, -half, 0})), placement.Point(local.Point({0, half, 0})),
            shape.transform.scale.x * .5 * std::sqrt(scale2)};
}

Bounds3 MeshCollision::Bounds(const MeshInstance& instance)
{
    if (!instance.mesh) return {};
    auto bounds = WorldBounds(instance.mesh->bounds, instance.transform);
    const auto data = Current(*instance.mesh);
    if (data->mode == MeshCollisionMode::Simple)
        for (const auto& shape : data->shapes)
        {
            const auto b = Capsule(shape, instance.transform).Bounds();
            bounds.Include(b.min); bounds.Include(b.max);
        }
    return bounds;
}

Bounds3 CollisionCapsule::Bounds() const
{
    Bounds3 result; const Vector3 r{radius, radius, radius};
    result.Include(a - r); result.Include(a + r); result.Include(b - r); result.Include(b + r); return result;
}
Vector3 CollisionCapsule::ClosestAxisPoint(Vector3 p) const
{
    const auto ab = b - a; const double length2 = Math::Dot(ab, ab);
    return a + ab * (length2 > 1e-20 ? std::clamp(Math::Dot(p - a, ab) / length2, 0.0, 1.0) : 0);
}
std::optional<CollisionCapsule::Hit> CollisionCapsule::Raycast(Vector3 origin, Vector3 d, double distance, bool walkableOnly) const
{
    std::optional<Hit> hit;
    const auto ab = b - a; const double length = Math::Length(ab);
    const auto axis = length > 1e-10 ? ab * (1 / length) : Vector3{0, 1, 0};
    const auto accept = [&](double t) {
        if (t < 0 || t > distance) return;
        const auto p = origin + d * t;
        const auto normal = Math::Normalize(p - ClosestAxisPoint(p));
        if (walkableOnly && normal.y < .65) return;
        distance = t; hit = Hit{t, normal};
    };
    // Cylinder side and hemispherical caps. Filtering the sphere roots avoids
    // false hits on the internal hemisphere where a cap meets the cylinder.
    if (length > 1e-10)
    {
        const auto p = origin - a;
        const double dy = Math::Dot(d, axis), py = Math::Dot(p, axis);
        const auto rd = d - axis * dy, rp = p - axis * py;
        const double qa = Math::Dot(rd, rd), qb = Math::Dot(rd, rp), qc = Math::Dot(rp, rp) - radius * radius;
        const double disc = qb * qb - qa * qc;
        if (qa > 1e-20 && disc >= 0)
            for (double t : {(-qb - std::sqrt(disc)) / qa, (-qb + std::sqrt(disc)) / qa})
                if (const double h = py + t * dy; h >= 0 && h <= length) accept(t);
    }
    for (int cap = 0; cap < (length > 1e-10 ? 2 : 1); ++cap)
    {
        const auto center = cap ? b : a, p = origin - center;
        const double qb = Math::Dot(p, d), disc = qb * qb - Math::Dot(p, p) + radius * radius;
        if (disc < 0) continue;
        for (double t : {-qb - std::sqrt(disc), -qb + std::sqrt(disc)})
        {
            const double h = Math::Dot(origin + d * t - center, axis);
            if (length <= 1e-10 || (cap ? h >= 0 : h <= 0)) accept(t);
        }
    }
    return hit;
}
