#include "world_vertex.hlsli"

cbuffer DepthLimits : register(b0, space3)
{
    float4 limits; // max radial distance squared (includes the sampling radius), alpha cutoff
};
#ifdef __spirv__
#define BSC_COMBINED [[vk::combinedImageSampler]]
#else
#define BSC_COMBINED
#endif
BSC_COMBINED Texture2D<float4> baseMap : register(t0, space2);
BSC_COMBINED SamplerState baseSampler : register(s0, space2);
BSC_COMBINED Texture2D<float4> opacityMap : register(t1, space2);
BSC_COMBINED SamplerState opacitySampler : register(s1, space2);

void DepthPS(WorldFragment input)
{
    clip(input.normal.w - .5); // No editor overlays.
    clip(limits.x - dot(input.view, input.view));
}

void CutoutPS(WorldFragment input)
{
    DepthPS(input);
    float alpha = baseMap.Sample(baseSampler, input.uv).a * opacityMap.Sample(opacitySampler, input.uv).r;
    clip(alpha - limits.y); // Same cutout test as the shadow maps, including foliage backs.
}
