#include "mesh.hpp"

#define CGLTF_IMPLEMENTATION
#include <cgltf.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string_view>

namespace
{
    bool CheckBuffers(const cgltf_data& data)
    {
        // Static GLBs use their own BIN chunk; check ranges before any accessor is read.
        if (data.buffers_count != 1 || data.buffers[0].uri || !data.bin || data.buffers[0].size > data.bin_size)
        {
            return SDL_SetError("Expected one embedded GLB geometry buffer");
        }
        for (size_t i = 0; i < data.buffer_views_count; ++i)
        {
            const auto& view = data.buffer_views[i];
            if (view.buffer != data.buffers || view.offset > view.buffer->size ||
                view.size > view.buffer->size - view.offset || view.has_meshopt_compression)
            {
                return SDL_SetError("Invalid or compressed GLB buffer view");
            }
        }
        for (size_t i = 0; i < data.accessors_count; ++i)
        {
            const auto& a = data.accessors[i];
            const size_t element = cgltf_calc_size(a.type, a.component_type);
            const size_t alignment = cgltf_component_size(a.component_type);
            const auto* view = a.buffer_view;
            if (!view || a.is_sparse || !element || !alignment || !a.count || a.stride < element ||
                a.offset > view->size || element > view->size - a.offset ||
                a.count - 1 > (view->size - a.offset - element) / a.stride ||
                (reinterpret_cast<std::uintptr_t>(data.bin) + view->offset + a.offset) % alignment || a.stride % alignment)
            {
                return SDL_SetError("Expected a dense, in-bounds GLB accessor");
            }
        }
        return true;
    }

    MeshTransform NodeTransform(const cgltf_node& node)
    {
        float m[16];
        cgltf_node_transform_world(&node, m);
        // glTF is right-handed, Y-up. Reflect the Z output into world coordinates.
        return {{m[12], m[13], -m[14]}, {m[0], m[1], -m[2]},
                {m[4], m[5], -m[6]}, {m[8], m[9], -m[10]}};
    }

    bool AppendTriangles(const cgltf_primitive& p, size_t start, size_t vertexCount, bool mirrored, bool generateNormals, Mesh& mesh)
    {
        const size_t count = p.indices ? p.indices->count : vertexCount;
        const bool list = p.type == cgltf_primitive_type_triangles;

        if (count < 3 || (list && count % 3))
        {
            return SDL_SetError("Incomplete triangle primitive");
        }

        for (size_t i = 2; i < count; i += list ? 3 : 1)
        {
            size_t corners[3]{p.type == cgltf_primitive_type_triangle_fan ? 0 : i - 2, i - 1, i};

            if (p.type == cgltf_primitive_type_triangle_strip && i % 2)
            {
                std::swap(corners[0], corners[1]);
            }

            if (mirrored)
            {
                std::swap(corners[1], corners[2]);
            }

            std::uint32_t tri[3];

            for (int j = 0; j < 3; ++j)
            {
                size_t index = p.indices ? cgltf_accessor_read_index(p.indices, corners[j]) : corners[j];
                if (index >= vertexCount)
                {
                    return SDL_SetError("Triangle index is outside its vertex array");
                }
                tri[j] = static_cast<std::uint32_t>(start + index);
            }

            mesh.indices.insert(mesh.indices.end(), {tri[0], tri[1], tri[2]});

            if (generateNormals)
            {
                const auto& v = mesh.vertices;
                const Vector3 face = Math::Cross(v[tri[1]].position - v[tri[0]].position, v[tri[2]].position - v[tri[0]].position);
                for (auto index : tri)
                {
                    mesh.vertices[index].normal = mesh.vertices[index].normal + face;
                }
            }

            if (p.material && p.material->double_sided)
            {
                mesh.indices.insert(mesh.indices.end(), {tri[0], tri[2], tri[1]});
            }
        }

        return true;
    }

    bool ReadPrimitive(const cgltf_primitive& p, const MeshTransform& transform, Mesh& mesh)
    {
        if (p.has_draco_mesh_compression || p.targets_count)
        {
            return SDL_SetError("Compressed and morph-target primitives are unsupported");
        }

        if (p.type != cgltf_primitive_type_triangles && p.type != cgltf_primitive_type_triangle_strip && p.type != cgltf_primitive_type_triangle_fan)
        {
            return SDL_SetError("Expected a triangle mesh");
        }

        const auto* pos = cgltf_find_accessor(&p, cgltf_attribute_type_position, 0);
        const auto* normal = cgltf_find_accessor(&p, cgltf_attribute_type_normal, 0);
        const auto* uv = cgltf_find_accessor(&p, cgltf_attribute_type_texcoord, 0);
        const auto* color = cgltf_find_accessor(&p, cgltf_attribute_type_color, 0);

        if (!pos || pos->type != cgltf_type_vec3 || (normal && normal->type != cgltf_type_vec3) || (uv && uv->type != cgltf_type_vec2) || (color && color->type != cgltf_type_vec3 && color->type != cgltf_type_vec4))
        {
            return SDL_SetError("Invalid mesh vertex attributes");
        }

        if (p.material && p.material->alpha_mode != cgltf_alpha_mode_opaque)
        {
            return SDL_SetError("The world mesh pass requires opaque materials");
        }

        const size_t start = mesh.vertices.size();

        if (pos->count > std::numeric_limits<std::uint32_t>::max() - start)
        {
            return SDL_SetError("Mesh has too many vertices");
        }

        const bool mirrored = transform.Determinant() < 0;
        const Vector3 nx = Math::Cross(transform.y, transform.z);
        const Vector3 ny = Math::Cross(transform.z, transform.x);
        const Vector3 nz = Math::Cross(transform.x, transform.y);

        for (size_t i = 0; i < pos->count; ++i)
        {
            float xyz[3], n[3]{}, st[2]{}, rgba[4]{1, 1, 1, 1};

            if (!cgltf_accessor_read_float(pos, i, xyz, 3) ||
                (normal && !cgltf_accessor_read_float(normal, i, n, 3)) ||
                (uv && !cgltf_accessor_read_float(uv, i, st, 2)) ||
                (color && !cgltf_accessor_read_float(color, i, rgba, cgltf_num_components(color->type))))
            {
                return SDL_SetError("Could not decode mesh vertices");
            }

            MeshVertex v;
            v.position = transform.Point({xyz[0], xyz[1], xyz[2]});
            // Inverse-transpose of the node's linear transform, up to a positive scalar.
            v.normal = Math::Normalize((nx * n[0] + ny * n[1] + nz * n[2]) * (mirrored ? -1.0 : 1.0));
            v.uv = {st[0], st[1]};

            if (p.material && p.material->has_pbr_metallic_roughness)
            {
                const auto& factor = p.material->pbr_metallic_roughness.base_color_factor;
                for (int j = 0; j < 3; ++j)
                {
                    rgba[j] *= factor[j];
                }
            }

            v.color = {rgba[0], rgba[1], rgba[2], 1};

            if (!std::isfinite(Math::Length(v.position)) || !std::isfinite(Math::Length(v.normal)) || !std::isfinite(st[0]) || !std::isfinite(st[1]) || !std::isfinite(rgba[0]) || !std::isfinite(rgba[1]) || !std::isfinite(rgba[2]))
            {
                return SDL_SetError("Non-finite mesh vertex");
            }

            mesh.vertices.push_back(v);
        }

        if (!AppendTriangles(p, start, pos->count, mirrored, !normal, mesh))
        {
            return false;
        }

        if (!normal)
        {
            for (size_t i = start; i < mesh.vertices.size(); ++i)
            {
                mesh.vertices[i].normal = Math::Normalize(mesh.vertices[i].normal);
            }
        }
        return true;
    }

    bool ReadMesh(const cgltf_mesh& source, const MeshTransform& transform, Mesh& mesh)
    {
        if (!transform.Valid())
        {
            return SDL_SetError("Invalid mesh node transform");
        }

        for (size_t i = 0; i < source.primitives_count; ++i)
        {
            if (!ReadPrimitive(source.primitives[i], transform, mesh))
            {
                return false;
            }
        }

        return true;
    }

    bool ImportGLB(const void* bytes, size_t size, Mesh& mesh)
    {
        cgltf_options options{};
        options.type = cgltf_file_type_glb;
        cgltf_data* parsed = nullptr;

        if (cgltf_parse(&options, bytes, size, &parsed) != cgltf_result_success)
        {
            return SDL_SetError("Invalid glTF 2.0 binary");
        }

        std::unique_ptr<cgltf_data, decltype(&cgltf_free)> data(parsed, cgltf_free);

        if (!CheckBuffers(*data))
        {
            return false;
        }

        if (cgltf_load_buffers(&options, data.get(), nullptr) != cgltf_result_success || cgltf_validate(data.get()) != cgltf_result_success)
        {
            return SDL_SetError("Invalid GLB buffers or scene");
        }

        for (size_t i = 0; i < data->extensions_required_count; ++i)
        {
            const std::string_view name = data->extensions_required[i];
            if (name != "KHR_mesh_quantization" && name != "KHR_materials_unlit")
            {
                return SDL_SetError("Unsupported required glTF extension: %s", data->extensions_required[i]);
            }
        }

        std::vector<const cgltf_node*> pending;
        const auto* scene = data->scene ? data->scene : (data->scenes_count ? data->scenes : nullptr);
        if (scene)
        {
            for (size_t i = scene->nodes_count; i > 0; --i)
            {
                pending.push_back(scene->nodes[i - 1]);
            }
        }
        else
        {
            for (size_t i = data->nodes_count; i > 0; --i)
            {
                if (!data->nodes[i - 1].parent)
                {
                    pending.push_back(&data->nodes[i - 1]);
                }
            }
            // A GLB can also be a mesh library with no scene or nodes.
            if (!data->nodes_count)
            {
                MeshTransform transform;
                transform.z.z = -1;
                for (size_t i = 0; i < data->meshes_count; ++i)
                {
                    if (!ReadMesh(data->meshes[i], transform, mesh))
                    {
                        return false;
                    }
                }
            }
        }
        while (!pending.empty())
        {
            const auto* node = pending.back();
            pending.pop_back();
            for (size_t i = node->children_count; i > 0; --i)
            {
                pending.push_back(node->children[i - 1]);
            }
            if (!node->mesh)
            {
                continue;
            }
            if (node->skin || node->has_mesh_gpu_instancing)
            {
                return SDL_SetError("Expected static mesh nodes");
            }
            if (!ReadMesh(*node->mesh, NodeTransform(*node), mesh))
            {
                return false;
            }
        }
        return !mesh.indices.empty() || SDL_SetError("No triangles in the active scene");
    }
}

std::shared_ptr<const Mesh> MeshAssets::Load(const std::string& relativePath)
{
    std::string name = relativePath;
    std::replace(name.begin(), name.end(), '\\', '/');

    if (name.empty() || name.front() == '/' || name.find(':') != std::string::npos || name.find('\0') != std::string::npos || ("/" + name + "/").find("/../") != std::string::npos)
    {
        SDL_SetError("Expected a model path relative to assets/models");
        return nullptr;
    }

    const auto found = loaded.find(name);

    if (found != loaded.end())
    {
        return found->second;
    }

    const std::string path = AssetPaths::Root(location) + "models/" + name;
    size_t size;
    std::unique_ptr<void, decltype(&SDL_free)> bytes(SDL_LoadFile(path.c_str(), &size), SDL_free);
    auto mesh = std::make_shared<Mesh>();

    if (!bytes || !ImportGLB(bytes.get(), size, *mesh))
    {
        const std::string reason = SDL_GetError();
        SDL_SetError("Cannot load model '%s': %s", name.c_str(), reason.c_str());

        return nullptr;
    }

    mesh->assetPath = name;
    loaded.emplace(name, mesh);
    return mesh;
}
