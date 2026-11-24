#include "mesh_fbx.hpp"
#include "mesh.hpp"
#include <ufbx.h>
#include <algorithm>
#include <cmath>
#include <new>

namespace
{
    // Bound both triangulation scratch and expanded geometry before allocating it.
    constexpr std::size_t maxCorners = 12'000'000;
    Vector3 Reflect(ufbx_vec3 p) { return {p.x, p.y, -p.z}; }

    bool ReadNode(const ufbx_node& node, Mesh& mesh, std::size_t defaultSlot)
    {
        const auto& source = *node.mesh;
        if (source.skin_deformers.count || source.blend_deformers.count || source.cache_deformers.count)
            return SDL_SetError("FBX import supports static meshes, not skin, morph or cache deformers");
        const auto& m = node.geometry_to_world;
        const MeshTransform transform{Reflect(m.cols[3]), Reflect(m.cols[0]), Reflect(m.cols[1]), Reflect(m.cols[2])};
        if (!transform.Valid()) return SDL_SetError("Invalid FBX node transform");
        const auto normalTransform = transform.NormalTransform();
        const bool mirrored = transform.Determinant() < 0;
        if (source.max_face_triangles > maxCorners / 3 || source.num_triangles > maxCorners / 3)
            return SDL_SetError("FBX mesh is too large");
        std::vector<std::uint32_t> triangles(source.max_face_triangles * 3);
        for (const auto& part : source.material_parts)
        {
            const auto* material = part.index < node.materials.count ? node.materials.data[part.index] : nullptr;
            const bool twoSided = material && material->features.double_sided.enabled;
            if (part.num_triangles > (maxCorners - mesh.vertices.size()) / (twoSided ? 6 : 3))
                return SDL_SetError("FBX LOD is too large");
            const std::size_t first = mesh.vertices.size();
            ufbx_vec3 tint{1, 1, 1};
            if (material && material->fbx.diffuse_color.has_value) tint = material->fbx.diffuse_color.value_vec3;
            for (auto faceIndex : part.face_indices)
            {
                const auto count = ufbx_triangulate_face(triangles.data(), triangles.size(), &source, source.faces.data[faceIndex]);
                for (std::size_t i = 0; i < count * 3u; i += 3)
                {
                    MeshVertex face[3]{};
                    SDL_FColor colors[3]{};
                    for (std::size_t j = 0; j < 3; ++j)
                    {
                        const auto index = triangles[i + (mirrored && j ? 3 - j : j)];
                        auto& v = face[j];
                        const auto p = ufbx_get_vertex_vec3(&source.vertex_position, index);
                        const auto n = ufbx_get_vertex_vec3(&source.vertex_normal, index);
                        v.position = transform.Point({p.x, p.y, p.z});
                        v.normal = Math::Normalize(normalTransform.Direction({n.x, n.y, n.z}));
                        if (source.vertex_uv.exists)
                        {
                            const auto uv = ufbx_get_vertex_vec2(&source.vertex_uv, index);
                            // FBX UVs use a bottom-left origin; engine textures use top-left.
                            v.uv = {float(uv.x), float(1 - uv.y)};
                        }
                        const auto color = source.vertex_color.exists ? ufbx_get_vertex_vec4(&source.vertex_color, index) : ufbx_vec4{1, 1, 1, 1};
                        colors[j] = {float(color.x), float(color.y), float(color.z), float(color.w)};
                        v.color = {float(color.x * tint.x), float(color.y * tint.y), float(color.z * tint.z), 1};
                        if (!std::isfinite(Math::Length(v.position)) || !std::isfinite(Math::Length(v.normal)) ||
                            !std::isfinite(v.uv.x) || !std::isfinite(v.uv.y) || !std::isfinite(v.color.r) ||
                            !std::isfinite(v.color.g) || !std::isfinite(v.color.b) ||
                            !std::isfinite(colors[j].r) || !std::isfinite(colors[j].g) || !std::isfinite(colors[j].b) || !std::isfinite(colors[j].a))
                            return SDL_SetError("Non-finite FBX vertex");
                    }
                    mesh.vertices.insert(mesh.vertices.end(), std::begin(face), std::end(face));
                    mesh.vertexColors.insert(mesh.vertexColors.end(), std::begin(colors), std::end(colors));
                    if (twoSided)
                    {
                        for (auto& v : face) v.normal = v.normal * -1;
                        std::swap(face[1], face[2]);
                        std::swap(colors[1], colors[2]);
                        mesh.vertices.insert(mesh.vertices.end(), std::begin(face), std::end(face));
                        mesh.vertexColors.insert(mesh.vertexColors.end(), std::begin(colors), std::end(colors));
                    }
                }
            }
            if (mesh.vertices.size() != first)
            {
                const auto slot = material ? std::size_t(material->typed_id) : defaultSlot;
                if (slot == mesh.materials->slots.size()) mesh.materials->slots.push_back({"Default", {}});
                mesh.parts.push_back({first, mesh.vertices.size() - first, slot});
            }
        }
        return true;
    }

    bool ReadLevel(const ufbx_scene& scene, std::size_t level, Mesh& mesh)
    {
        std::vector<const ufbx_node*> pending{scene.root_node};
        while (!pending.empty())
        {
            const auto* node = pending.back();
            pending.pop_back();
            if (node->mesh && !ReadNode(*node, mesh, scene.materials.count)) return false;
            if (ufbx_as_lod_group(node->attrib) && node->children.count)
                pending.push_back(node->children.data[std::min(level, node->children.count - 1)]);
            else for (std::size_t i = node->children.count; i > 0; --i) pending.push_back(node->children.data[i - 1]);
        }
        if (mesh.vertices.empty()) return SDL_SetError("No triangles in FBX LOD %zu", level);
        mesh.indices.resize(mesh.vertices.size());
        const ufbx_vertex_stream streams[] = {
            {mesh.vertices.data(), mesh.vertices.size(), sizeof(MeshVertex)},
            {mesh.vertexColors.data(), mesh.vertexColors.size(), sizeof(SDL_FColor)}};
        ufbx_error error{};
        const auto count = ufbx_generate_indices(streams, 2, mesh.indices.data(), mesh.indices.size(), nullptr, &error);
        if (!count) return SDL_SetError("Could not index FBX vertices: %s", error.description.data);
        mesh.vertices.resize(count);
        mesh.vertices.shrink_to_fit();
        mesh.vertexColors.resize(count);
        mesh.vertexColors.shrink_to_fit();
        mesh.UpdateBounds();
        return true;
    }
}

bool ImportFBX(const void* bytes, std::size_t size, Mesh& mesh)
{
    ufbx_load_opts options{};
    options.file_format = UFBX_FILE_FORMAT_FBX;
    options.target_axes = ufbx_axes_right_handed_y_up;
    options.target_unit_meters = 1;
    options.generate_missing_normals = true;
    options.ignore_animation = true;
    options.ignore_embedded = true;
    options.load_external_files = false; // Materials are assigned through .bmesh, as with GLB.
    options.index_error_handling = UFBX_INDEX_ERROR_HANDLING_ABORT_LOADING;
    options.node_depth_limit = 256;
    options.temp_allocator.memory_limit = 256 * 1024 * 1024;
    options.result_allocator.memory_limit = 512 * 1024 * 1024;
    ufbx_error error{};
    std::unique_ptr<ufbx_scene, decltype(&ufbx_free_scene)> scene(ufbx_load_memory(bytes, size, &options, &error), ufbx_free_scene);
    if (!scene) return SDL_SetError("Cannot read FBX: %s", error.description.data);
    try
    {
        mesh.materials = std::make_shared<MeshMaterials>();
        for (const auto* material : scene->materials)
            mesh.materials->slots.push_back({material->name.length ? std::string(material->name.data, material->name.length) :
                "Material " + std::to_string(material->typed_id + 1), {}});
        std::size_t levels = 1;
        for (const auto* node : scene->nodes)
            if (ufbx_as_lod_group(node->attrib)) levels = std::max(levels, node->children.count);
        if (levels > Mesh::maxLODCount) return SDL_SetError("FBX exceeds the limit of %zu LODs", Mesh::maxLODCount);
        if (!ReadLevel(*scene, 0, mesh)) return false;
        // Some exporters (including SpeedTree) leave all distances at zero.
        // Fall back to screen coverage: 35%, 15%, then progressively smaller.
        for (std::size_t i = 1; i < levels; ++i)
        {
            auto lod = std::make_shared<Mesh>();
            lod->materials = mesh.materials;
            if (!ReadLevel(*scene, i, *lod)) return false;
            mesh.lods.push_back({std::move(lod), 0.35 * std::pow(3.0 / 7.0, double(i - 1))});
        }
        // A single group's valid authored thresholds can be used directly.
        // Multiple groups use matching detail indices and the common screen rule.
        if (scene->lod_groups.count == 1)
        {
            const auto& group = *scene->lod_groups.data[0];
            bool valid = group.lod_levels.count >= levels;
            double previous = group.relative_distances ? 100.0 : 0.0;
            for (std::size_t i = 1; valid && i < levels; ++i)
            {
                const double value = group.lod_levels.data[i].distance;
                valid = std::isfinite(value) && value > 0 && (group.relative_distances ? value < previous : value > previous);
                previous = value;
            }
            if (valid)
            {
                mesh.lodDistances = !group.relative_distances;
                for (std::size_t i = 1; i < levels; ++i)
                    mesh.lods[i - 1].threshold = group.lod_levels.data[i].distance *
                        (group.relative_distances ? 0.01 : scene->settings.unit_meters);
            }
        }
        mesh.UpdateBounds();
        return true;
    }
    catch (const std::bad_alloc&)
    {
        return SDL_SetError("Not enough memory to import FBX geometry");
    }
}
