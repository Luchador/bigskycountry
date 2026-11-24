// Camera-view-space SSAO. Reversed camera depth is kept separate from the MSAA
// depth/stencil target, so the same path works at all configured sample counts.
struct ScreenFragment
{
    float4 position : SV_Position;
    float2 uv : TEXCOORD0;
};
ScreenFragment FullscreenVS(uint id : SV_VertexID)
{
    ScreenFragment output;
    output.uv = float2((id << 1) & 2, id & 2);
    output.position = float4(output.uv * float2(2,-2) + float2(-1,1), 0, 1);
    return output;
}

#ifdef __spirv__
#define BSC_COMBINED [[vk::combinedImageSampler]]
#else
#define BSC_COMBINED
#endif
BSC_COMBINED Texture2D<float4> sourceMap : register(t0, space2);
BSC_COMBINED SamplerState sourceSampler : register(s0, space2);
cbuffer OcclusionSettings : register(b0, space3)
{
    float4 projection; // inverse projection scales x/y, near depth, world radius
    float4 textureSize; // inverse full-resolution depth width/height, AO width/height
    float4 settings; // samples, bias in meters, fade start, fade end
    float4 blurStep; // blur direction in texture coordinates
};

float Depth(float2 uv)
{
    return sourceMap.SampleLevel(sourceSampler, uv, 0).r;
}
float3 Position(float2 uv, float depth)
{
    return float3((uv*2-1)*float2(1,-1)*projection.xy, 1) * (projection.z / max(depth, 1e-8));
}
float2 OcclusionPS(ScreenFragment input) : SV_Target0
{
    // Snap to the actual full-resolution texel used by the point sample. This
    // matters at odd viewport sizes and avoids introducing slope bias.
    float2 uv = (floor(input.uv/textureSize.xy)+.5)*textureSize.xy;
    float depth = Depth(uv);
    if (depth <= 0) return float2(1,0);
    float3 p = Position(uv,depth);
    float distance = length(p);
    if (distance >= settings.w) return float2(1,depth);

    float2 dx = float2(textureSize.x,0), dy = float2(0,textureSize.y);
    float3 left = Position(uv-dx,Depth(uv-dx)), right = Position(uv+dx,Depth(uv+dx));
    float3 top = Position(uv-dy,Depth(uv-dy)), bottom = Position(uv+dy,Depth(uv+dy));
    float3 horizontal = abs(left.z-p.z) < abs(right.z-p.z) ? p-left : right-p;
    float3 vertical = abs(top.z-p.z) < abs(bottom.z-p.z) ? p-top : bottom-p;
    float3 normal = cross(horizontal,vertical);
    float sizeSquared = dot(normal,normal);
    if (sizeSquared < 1e-16) return float2(1,depth);
    normal *= rsqrt(sizeSquared);
    if (dot(normal,p)>0) normal = -normal;
    float3 tangent = normalize(cross(abs(normal.z)<.9 ? float3(0,0,1) : float3(0,1,0),normal));
    float3 bitangent = cross(normal,tangent);
    // Stable per-pixel rotation; no frame jitter or temporal history/ghosting.
    float rotation = frac(52.9829189 * frac(dot(floor(input.position.xy),float2(.06711056,.00583715)))) * 6.2831853;
    float occluded = 0;
    uint count = (uint)settings.x;
    [loop] for (uint i=0; i<count; ++i)
    {
        float u = (i+.5)/count;
        float angle = i*2.3999632 + rotation;
        float z = .15 + .85*u;
        float ring = sqrt(1-z*z);
        float3 direction = tangent*(cos(angle)*ring) + bitangent*(sin(angle)*ring) + normal*z;
        float radius = projection.w * (.2 + .8*frac((i+1)*.618033989));
        float3 q = p + direction*radius;
        if (q.z <= projection.z) continue;
        float2 sampleUV = q.xy/(q.z*projection.xy)*float2(.5,-.5)+.5;
        if (any(sampleUV<=0) || any(sampleUV>=1)) continue;
        float sampleDepth = Depth(sampleUV);
        if (sampleDepth<=0) continue;
        float surfaceZ = projection.z/sampleDepth;
        float rangeWeight = 1-smoothstep(projection.w*.5,projection.w*2,abs(surfaceZ-p.z));
        occluded += (surfaceZ < q.z-settings.y ? 1 : 0) * rangeWeight;
    }
    float fade = 1-smoothstep(settings.z,settings.w,distance);
    return float2(saturate(1-occluded/count*fade),depth);
}

float2 BlurPS(ScreenFragment input) : SV_Target0
{
    float2 center = sourceMap.SampleLevel(sourceSampler,input.uv,0).rg;
    if (center.y<=0) return float2(1,0);
    float depth = projection.z/center.y;
    float total = 0, result = 0;
    [unroll] for (int i=-3; i<=3; ++i)
    {
        float2 sample = sourceMap.SampleLevel(sourceSampler,input.uv+blurStep.xy*i,0).rg;
        float delta = abs(projection.z/max(sample.y,1e-8)-depth);
        float weight = exp2(-i*i*.25 - delta/max(.08,depth*.01));
        total += weight;
        result += sample.x*weight;
    }
    return float2(result/max(total,1e-6),center.y);
}
