struct WorldVertex
{
    float4 position : TEXCOORD0;
    float4 color : TEXCOORD1;
    float2 uv : TEXCOORD2;
    float3 light : TEXCOORD3;
    float3 view : TEXCOORD4;
};

struct WorldFragment
{
    float4 position : SV_Position;
    float4 color : TEXCOORD0;
    float2 uv : TEXCOORD1;
    float3 light : TEXCOORD2;
    float3 view : TEXCOORD3;
};

WorldFragment VSMain(WorldVertex input)
{
    WorldFragment output;
    output.position = input.position;
    output.color = input.color;
    output.uv = input.uv;
    output.light = input.light;
    output.view = input.view;
    return output;
}

float4 PSMain(WorldFragment input) : SV_Target0
{
    return input.color;
}

// Multiplying by reciprocal clip W makes each component a linear screen-space
// edge equation. Its gradient converts the threshold to pixels at any zoom.
float4 WireframePS(WorldFragment input) : SV_Target0
{
    float3 edge = float3(input.uv, 1 - input.uv.x - input.uv.y) * input.position.w;
    float3 dx = ddx(edge), dy = ddy(edge);
    const float halfWidth = 1.0;
    bool onEdge = any(edge * edge <= halfWidth * halfWidth * (dx * dx + dy * dy));
    // Clear interiors to sky while writing depth and selection stencil as usual.
    return float4(input.color.rgb, onEdge ? 1.0 : 0.0);
}

#ifdef __spirv__
#define BSC_COMBINED [[vk::combinedImageSampler]]
#else
#define BSC_COMBINED
#endif
BSC_COMBINED Texture2D<float4> baseColorMap : register(t0, space2);
BSC_COMBINED Texture2D<float2> normalMap : register(t1, space2);
BSC_COMBINED Texture2D<float> roughnessMap : register(t2, space2);
BSC_COMBINED SamplerState colorSampler : register(s0, space2);
BSC_COMBINED SamplerState normalSampler : register(s1, space2);
BSC_COMBINED SamplerState roughnessSampler : register(s2, space2);

float4 AsphaltPS(WorldFragment input) : SV_Target0
{
    float3 albedo = baseColorMap.Sample(colorSampler, input.uv).rgb;
    float2 xy = normalMap.Sample(normalSampler, input.uv) * 2 - 1;
    float3 normal = normalize(float3(xy, sqrt(saturate(1 - dot(xy, xy)))));
    float gloss = 1 - roughnessMap.Sample(roughnessSampler, input.uv);
    float diffuse = saturate(dot(normal, input.light));
    float3 halfway = normalize(input.light + normalize(input.view));
    float specular = 0.08 * gloss * diffuse * pow(saturate(dot(normal, halfway)), 4 + 124 * gloss * gloss);
    float3 c = saturate(albedo * (0.28 + 0.72 * diffuse) + specular);
    // The shared SDL color target stores display-encoded colors.
    return float4(lerp(12.92 * c, 1.055 * pow(c, 1.0 / 2.4) - 0.055, step(0.0031308, c)), 1);
}
