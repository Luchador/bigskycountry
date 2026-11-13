#pragma once

#include "math.hpp"
#include <algorithm>
#include <array>
#include <cmath>

inline std::array<float, 4> BlendMaterialSamples(const std::array<float, 4>& outer,
                                                const std::array<float, 4>& inner, float weight)
{
    std::array<float, 4> result;
    for (std::size_t i = 0; i < result.size(); ++i) result[i] = std::lerp(outer[i], inner[i], weight);
    return result;
}

inline Vector3 DecodeNormalXY(const std::array<float, 4>& sample)
{
    const float x = 2 * sample[0] - 1, y = 2 * sample[1] - 1;
    return {x, y, std::sqrt(std::max(0.f, 1 - x*x - y*y))};
}
