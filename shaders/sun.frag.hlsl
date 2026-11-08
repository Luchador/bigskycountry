// SDL binds the texture submitted to SDL_RenderGeometry at fragment sampler slot 0.
#ifdef __spirv__
[[vk::combinedImageSampler]]
#endif
Texture2D<float4> sunMask : register(t0, space2);
#ifdef __spirv__
[[vk::combinedImageSampler]]
#endif
SamplerState sunSampler : register(s0, space2);

// Match the outputs of SDL's compiled textured vertex shader: color in slot 0, UV in slot 1.
float4 main(float4 emission : TEXCOORD0, float2 uv : TEXCOORD1) : SV_Target0
{
    float coverage = sunMask.Sample(sunSampler, uv).a * emission.a;
    // Premultiplied additive blending: the framebuffer adds this light to the sky,
    // including at the fully covered center. Transparent padding contributes zero.
    return float4(emission.rgb * coverage, coverage);
}
