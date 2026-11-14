#include "test_pawn_meshes.hpp"

#include <array>
#include <cmath>
#include <numbers>

namespace
{
    void Quad(Mesh& mesh, Vector3 a, Vector3 b, Vector3 c, Vector3 d, SDL_FColor color)
    {
        const auto first = static_cast<std::uint32_t>(mesh.vertices.size());
        const Vector3 normal = Math::Normalize(Math::Cross(b - a, c - a));
        for (Vector3 point : {a, b, c, d}) mesh.vertices.push_back({point, normal, {}, color});
        for (unsigned offset : {0, 1, 2, 0, 2, 3}) mesh.indices.push_back(first + offset);
    }

    void Box(Mesh& mesh, Vector3 lo, Vector3 hi, SDL_FColor color)
    {
        const Vector3 center = (lo + hi) * 0.5, half = (hi - lo) * 0.5;
        constexpr Vector3 normals[] = {{1,0,0}, {-1,0,0}, {0,1,0}, {0,-1,0}, {0,0,1}, {0,0,-1}};
        for (Vector3 normal : normals)
        {
            const Vector3 u = normal.y != 0 ? Vector3{1,0,0} : Math::Cross({0,1,0}, normal);
            const Vector3 v = Math::Cross(normal, u);
            const auto scaled = [half](Vector3 axis) { return Vector3{axis.x * half.x, axis.y * half.y, axis.z * half.z}; };
            const auto middle = center + scaled(normal), a = scaled(u), b = scaled(v);
            Quad(mesh, middle - a - b, middle + a - b, middle + a + b, middle - a + b, color);
        }
    }

    void Wheel(Mesh& mesh, Vector3 center)
    {
        constexpr int sides = 16;
        constexpr double radius = 0.35, halfWidth = 0.13;
        const SDL_FColor rubber{0.025f,0.03f,0.035f,1}, metal{0.25f,0.27f,0.29f,1};
        for (int i = 0; i < sides; ++i)
        {
            const double a = i * 2 * std::numbers::pi / sides, b = (i + 1) * 2 * std::numbers::pi / sides;
            const Vector3 p{0, radius * std::cos(a), radius * std::sin(a)};
            const Vector3 q{0, radius * std::cos(b), radius * std::sin(b)};
            const Vector3 left = center - Vector3{halfWidth,0,0}, right = center + Vector3{halfWidth,0,0};
            Quad(mesh, left + p, left + q, right + q, right + p, rubber);
            Quad(mesh, right, right + p, right + q, right, metal);
            Quad(mesh, left, left + q, left + p, left, metal);
        }
    }
}

std::shared_ptr<const Mesh> TestPawnMeshes::Car()
{
    static const auto mesh = [] {
        auto result = std::make_shared<Mesh>();
        Box(*result, {-0.9,0.4,-2.15}, {0.9,0.95,2.15}, {0.04f,0.24f,0.65f,1});
        Box(*result, {-0.76,0.95,-0.95}, {0.76,1.55,0.95}, {0.035f,0.06f,0.08f,1});
        Box(*result, {-0.8,1.55,-1.0}, {0.8,1.65,1.0}, {0.04f,0.24f,0.65f,1});
        for (double side : {-1.0,1.0})
        {
            for (double z : {-1.35,1.35}) Wheel(*result, {side * 0.94,0.35,z});
            Box(*result, {side * 0.61 - 0.16,0.65,2.15}, {side * 0.61 + 0.16,0.85,2.17}, {0.95f,0.92f,0.65f,1});
            Box(*result, {side * 0.61 - 0.16,0.65,-2.17}, {side * 0.61 + 0.16,0.85,-2.15}, {0.65f,0.015f,0.01f,1});
        }
        result->UpdateBounds();
        return result;
    }();
    return mesh;
}

std::shared_ptr<const Mesh> TestPawnMeshes::Character()
{
    static const auto mesh = [] {
        auto result = std::make_shared<Mesh>();
        Box(*result, {-0.24,0.75,-0.15}, {0.24,1.45,0.15}, {0.06f,0.15f,0.25f,1});
        Box(*result, {-0.15,1.45,-0.15}, {0.15,1.78,0.15}, {0.55f,0.32f,0.19f,1});
        for (double x : {-0.14,0.14})
            Box(*result, {x-0.1,0,-0.12}, {x+0.1,0.75,0.12}, {0.04f,0.05f,0.07f,1});
        result->UpdateBounds();
        return result;
    }();
    return mesh;
}

std::shared_ptr<const Mesh> TestPawnMeshes::Ground()
{
    static const auto mesh = [] {
        auto result = std::make_shared<Mesh>();
        for (int x = -10; x < 10; ++x)
            for (int z = -10; z < 10; ++z)
            {
                const float shade = (x + z) % 2 ? 0.19f : 0.23f;
                Quad(*result, {x*5.0,0,z*5.0}, {x*5.0,0,(z+1)*5.0},
                     {(x+1)*5.0,0,(z+1)*5.0}, {(x+1)*5.0,0,z*5.0}, {shade,shade,shade,1});
            }
        result->UpdateBounds();
        return result;
    }();
    return mesh;
}
