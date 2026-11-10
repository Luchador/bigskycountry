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
        auto lighting = a.lighting;
        for (size_t i = 0; i < lighting.view.size(); ++i)
        {
            lighting.view[i] = std::lerp(a.lighting.view[i], b.lighting.view[i], f);
        }
        return {std::lerp(a.x, b.x, f), std::lerp(a.y, b.y, f), std::lerp(a.z, b.z, f), std::lerp(a.w, b.w, f),
                {std::lerp(a.color.r, b.color.r, f), std::lerp(a.color.g, b.color.g, f),
                 std::lerp(a.color.b, b.color.b, f), std::lerp(a.color.a, b.color.a, f)},
                {std::lerp(a.uv.x, b.uv.x, f), std::lerp(a.uv.y, b.uv.y, f)}, lighting};
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
        SDL_FPoint uv;
        std::array<float, 3> view;
    };

    ScreenVertex Project(const WorldDepthVertex& v, int width, int height)
    {
        const double invW = 1.0 / v.w;

        return {(1.0 + v.x * invW) * width * 0.5, (1.0 - v.y * invW) * height * 0.5, v.z * invW, invW, v.color, v.uv, v.lighting.view};
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

    size_t range = 0;
    for (std::size_t first = 0; first < vertices.size(); first += 3)
    {
        while (range < pavement.size() && first >= pavement[range].end) ++range;
        const bool textured = drawAsphalt && range < pavement.size() && first >= pavement[range].first;
        const bool wire = first < wireframeEnd;
        if (first == foregroundStart) std::fill(softwareDepth.begin(), softwareDepth.end(), Sample{});
        const bool outline = first >= outlineStart && first < foregroundStart;
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

            // Derivatives of the three perspective weights. They give the UV
            // footprint of a screen pixel for mip selection, including clipped spans.
            const double ax = (b.y - c.y) * a.invW, ay = (c.x - b.x) * a.invW;
            const double bx = (c.y - a.y) * b.invW, by = (a.x - c.x) * b.invW;
            const double cx = (a.y - b.y) * c.invW, cy = (b.x - a.x) * c.invW;
            // The UVs of a wire triangle encode its original edges, including
            // after near-plane clipping has split it into several triangles.
            auto edgeEquations = [&](double u, double v, double w) -> Vector3
            {
                const double x = u*a.uv.x + v*b.uv.x + w*c.uv.x;
                const double y = u*a.uv.y + v*b.uv.y + w*c.uv.y;
                return {x, y, u + v + w - x - y};
            };
            Vector3 threshold{};
            if (wire)
            {
                const Vector3 dx = edgeEquations(ax, bx, cx), dy = edgeEquations(ay, by, cy);
                constexpr double halfWidth = 1.0; // Pixels on either side of a shared edge.
                threshold = Vector3{std::hypot(dx.x, dy.x), std::hypot(dx.y, dy.y), std::hypot(dx.z, dy.z)} * halfWidth;
            }
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
                    if (wire)
                    {
                        const Vector3 edges = edgeEquations(u, v, w);
                        if (edges.x > threshold.x && edges.y > threshold.y && edges.z > threshold.z)
                        {
                            pixel[3] = 0;
                            continue;
                        }
                    }
                    if (textured)
                    {
                        const double tu = (u*a.uv.x + v*b.uv.x + w*c.uv.x) / sum;
                        const double tv = (u*a.uv.y + v*b.uv.y + w*c.uv.y) / sum;
                        const auto gradient = [&](double da, double db, double dc)
                        {
                            return std::hypot(da*(a.uv.x-tu) + db*(b.uv.x-tu) + dc*(c.uv.x-tu),
                                              da*(a.uv.y-tv) + db*(b.uv.y-tv) + dc*(c.uv.y-tv)) / sum;
                        };
                        const float footprint = static_cast<float>(std::max(gradient(ax,bx,cx), gradient(ay,by,cy)));
                        auto lighting = vertices[first].lighting;
                        for (size_t j = 0; j < lighting.view.size(); ++j)
                        {
                            lighting.view[j] = static_cast<float>((u*a.view[j] + v*b.view[j] + w*c.view[j]) / sum);
                        }
                        const auto color = asphalt.Shade(float(tu), float(tv), footprint, lighting);
                        pixel[0] = Channel(color.r);
                        pixel[1] = Channel(color.g);
                        pixel[2] = Channel(color.b);
                        pixel[3] = 255;
                        continue;
                    }
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
