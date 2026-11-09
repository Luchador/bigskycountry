#include "world_renderer.hpp"
#include "view_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

// SDL renderers without a GPU device still get the same depth and culling rules.
namespace
{
    using Polygon = ViewGeometry::VertexPolygon<WorldDepthVertex>;

    WorldDepthVertex Interpolate(const WorldDepthVertex& a, const WorldDepthVertex& b, double t)
    {
        const float f = static_cast<float>(t);
        return {std::lerp(a.x, b.x, f), std::lerp(a.y, b.y, f), std::lerp(a.z, b.z, f), std::lerp(a.w, b.w, f),
                {std::lerp(a.color.r, b.color.r, f), std::lerp(a.color.g, b.color.g, f),
                 std::lerp(a.color.b, b.color.b, f), std::lerp(a.color.a, b.color.a, f)}};
    }

    template<typename Distance>

    void Clip(Polygon& polygon, Distance distance)
    {
        for (int i = 0; i < polygon.count; ++i)
        {
            if (distance(polygon.points[i]) < 0)
            {
                polygon = ViewGeometry::ClipPolygon(polygon, distance, Interpolate);
                return;
            }
        }
    }

    struct ScreenVertex
    {
        double x, y, depth, invW;
        SDL_FColor color;
    };

    ScreenVertex Project(const WorldDepthVertex& v, int width, int height)
    {
        const double invW = 1.0 / v.w;

        return {(1.0 + v.x * invW) * width * 0.5, (1.0 - v.y * invW) * height * 0.5, v.z * invW, invW, v.color};
    }

    double Edge(const ScreenVertex& a, const ScreenVertex& b, double x, double y)
    {
        return (b.x - a.x) * (y - a.y) - (b.y - a.y) * (x - a.x);
    }

    Uint8 Channel(double value)
    {
        return static_cast<Uint8>(std::clamp(value, 0.0, 1.0) * 255.0 + 0.5);
    }
}

bool WorldRenderer::DrawSoftware()
{
    void* pixels;
    int pitch;

    if (!SDL_LockTexture(colorTarget, nullptr, &pixels, &pitch))
    {
        return false;
    }

    const int width = colorTarget->w, height = colorTarget->h;
    std::fill(softwareDepth.begin(), softwareDepth.end(), Sample{});

    for (int y = 0; y < height; ++y)
    {
        std::memset(static_cast<Uint8*>(pixels) + static_cast<std::size_t>(y) * pitch, 0, static_cast<std::size_t>(width) * 4);
    }

    for (std::size_t first = 0; first < vertices.size(); first += 3)
    {
        const bool outline = first >= outlineStart;
        const bool selected = first >= selectedStart && first < selectedEnd;
        Polygon p{};
        p.count = 3;
        std::copy_n(vertices.data() + first, 3, p.points.begin());
        Clip(p, [](const auto& v) { return v.w - v.z; });
        Clip(p, [](const auto& v) { return v.w + v.x; });
        Clip(p, [](const auto& v) { return v.w - v.x; });
        Clip(p, [](const auto& v) { return v.w + v.y; });
        Clip(p, [](const auto& v) { return v.w - v.y; });

        for (int i = 2; i < p.count; ++i)
        {
            const auto a = Project(p.points[0], width, height);
            auto b = Project(p.points[i - 1], width, height);
            auto c = Project(p.points[i], width, height);
            double area = Edge(a, b, c.x, c.y);

            if (outline && area < 0.0)
            {
                std::swap(b, c);
                area = -area;
            }

            if (area <= 0.0)
            {
                continue;
            }

            const int left = std::max(0, static_cast<int>(std::floor(std::min({a.x, b.x, c.x}))));
            const int right = std::min(width, static_cast<int>(std::ceil(std::max({a.x, b.x, c.x}))));
            const int top = std::max(0, static_cast<int>(std::floor(std::min({a.y, b.y, c.y}))));
            const int bottom = std::min(height, static_cast<int>(std::ceil(std::max({a.y, b.y, c.y}))));

            for (int y = top; y < bottom; ++y)
            {
                auto* row = static_cast<Uint8*>(pixels) + static_cast<std::size_t>(y) * pitch;

                for (int x = left; x < right; ++x)
                {
                    const double wa = Edge(b, c, x + 0.5, y + 0.5);
                    const double wb = Edge(c, a, x + 0.5, y + 0.5);
                    const double wc = Edge(a, b, x + 0.5, y + 0.5);
                    if (wa < 0 || wb < 0 || wc < 0)
                    {
                        continue;
                    }
                    Sample& sample = softwareDepth[static_cast<std::size_t>(y) * width + x];
                    const float depth = static_cast<float>((wa * a.depth + wb * b.depth + wc * c.depth) / area);
                    if (depth < sample.depth || (outline && !sample.selected))
                    {
                        continue;
                    }
                    if (!outline)
                    {
                        sample = {depth, selected};
                    }
                    const double u = wa * a.invW, v = wb * b.invW, w = wc * c.invW;
                    const double sum = u + v + w;
                    Uint8* pixel = row + static_cast<std::size_t>(x) * 4;
                    pixel[0] = Channel((u * a.color.r + v * b.color.r + w * c.color.r) / sum);
                    pixel[1] = Channel((u * a.color.g + v * b.color.g + w * c.color.g) / sum);
                    pixel[2] = Channel((u * a.color.b + v * b.color.b + w * c.color.b) / sum);
                    pixel[3] = 255;
                }
            }
        }
    }

    SDL_UnlockTexture(colorTarget);

    return true;
}
