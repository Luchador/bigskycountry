struct WorldVertex
{
    float4 position : TEXCOORD0;
    float4 color : TEXCOORD1;
    float2 uv : TEXCOORD2;
    float4 normal : TEXCOORD3; // .w selects scene lighting or unlit editor colors.
    float4 tangent : TEXCOORD4; // .w is the sign of the bitangent.
    float3 view : TEXCOORD5;
};

struct WorldFragment
{
    float4 position : SV_Position;
    float4 color : TEXCOORD0;
    float2 uv : TEXCOORD1;
    float4 normal : TEXCOORD2;
    float4 tangent : TEXCOORD3;
    float3 view : TEXCOORD4;
};

WorldFragment VSMain(WorldVertex input)
{
    WorldFragment output;
    output.position = input.position;
    output.color = input.color;
    output.uv = input.uv;
    output.normal = input.normal;
    output.tangent = input.tangent;
    output.view = input.view;
    return output;
}

cbuffer WorldLights : register(b0, space3)
{
    float4 sunDirection, sunRadiance;
    float4 moonDirection, moonRadiance;
    float4 skyAmbient;
};

float3 UnitVector(float3 v)
{
    return v * rsqrt(max(dot(v, v), 1e-20));
}

void Illuminate(float3 direction, float3 radiance, float3 normal, float3 eye, float gloss,
                float metal, inout float3 diffuse, inout float3 specular)
{
    float cosine = saturate(dot(normal, direction));
    diffuse += radiance * ((1 - metal) * cosine);
    if (gloss > 0 && cosine > 0 && any(radiance))
    {
        float3 halfway = UnitVector(direction + eye);
        float highlight = pow(saturate(dot(normal, halfway)), 4 + 124 * gloss * gloss);
        specular += radiance * (gloss * cosine * highlight);
    }
}

// Keep the software renderer's WorldLighting::Shade in sync.
float3 ShadeSurface(float3 normal, float3 view, float3 base, float3 orm, float3 emission)
{
    float3 diffuse = skyAmbient.rgb * orm.r;
    float3 specular = 0;
    float3 eye = UnitVector(view);
    Illuminate(sunDirection.xyz, sunRadiance.rgb, normal, eye, 1 - orm.g, orm.b, diffuse, specular);
    Illuminate(moonDirection.xyz, moonRadiance.rgb, normal, eye, 1 - orm.g, orm.b, diffuse, specular);
    float3 c = saturate(emission + base * diffuse + lerp(0.08, base, orm.b) * specular);
    // SDL composites the world into a display-encoded color target.
    return lerp(12.92 * c, 1.055 * pow(c, 1.0 / 2.4) - 0.055, step(0.0031308, c));
}

float4 PSMain(WorldFragment input) : SV_Target0
{
    if (input.normal.w == 0) return input.color;
    return float4(ShadeSurface(UnitVector(input.normal.xyz), 0, input.color.rgb, float3(1, 1, 0), 0), input.color.a);
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
BSC_COMBINED Texture2D<float4> ormMap : register(t2, space2);
BSC_COMBINED Texture2D<float4> emissiveMap : register(t3, space2);
BSC_COMBINED Texture2D<float> opacityMap : register(t4, space2);
BSC_COMBINED SamplerState colorSampler : register(s0, space2);
BSC_COMBINED SamplerState normalSampler : register(s1, space2);
BSC_COMBINED SamplerState ormSampler : register(s2, space2);

BSC_COMBINED SamplerState emissiveSampler : register(s3, space2);
BSC_COMBINED SamplerState opacitySampler : register(s4, space2);
cbuffer MaterialProperties : register(b1, space3)
{
    uint alphaCutout;
};

float4 MaterialPS(WorldFragment input) : SV_Target0
{
    float4 base = baseColorMap.Sample(colorSampler, input.uv);
    if (alphaCutout) clip(base.a * opacityMap.Sample(opacitySampler, input.uv) - 0.5);
    float3 orm = ormMap.Sample(ormSampler, input.uv).rgb;
    float3 glow = emissiveMap.Sample(emissiveSampler, input.uv).rgb;
    float2 xy = normalMap.Sample(normalSampler, input.uv) * 2 - 1;
    float3 n = UnitVector(input.normal.xyz);
    float3 t = UnitVector(input.tangent.xyz - n * dot(n, input.tangent.xyz));
    float3 b = cross(n, t) * input.tangent.w;
    float3 mapped = UnitVector(t * xy.x + b * xy.y + n * sqrt(saturate(1 - dot(xy, xy))));
    return float4(ShadeSurface(mapped, input.view, base.rgb, orm, glow), 1);
}
