// All inputs/intermediate targets are linear RGBA16F. Only EncodePS writes sRGB.
#ifdef __spirv__
#define COMBINED [[vk::combinedImageSampler]]
#else
#define COMBINED
#endif
COMBINED Texture2D<float4> sourceImage : register(t0, space2);
COMBINED SamplerState sourceSampler : register(s0, space2);
COMBINED Texture2D<float4> secondImage : register(t1, space2);
COMBINED SamplerState secondSampler : register(s1, space2);
cbuffer PostSettings : register(b0, space3)
{
    float4 filter; // source texel xy, upsample radius, first downsample
    float4 settings; // bloom intensity, threshold, tone mapping enabled, sky supplied
    float4 background; // linear preview background
};
struct Fragment { float4 position : SV_Position; float2 uv : TEXCOORD0; };
Fragment FullscreenVS(uint id : SV_VertexID)
{
    Fragment output;
    output.uv = float2((id << 1) & 2, id & 2);
    output.position = float4(output.uv * float2(2,-2) + float2(-1,1), 0, 1);
    return output;
}
float3 SafeColor(float3 color) { return clamp(color, 0, 60000); }
float4 ComposePS(Fragment input) : SV_Target0
{
    float4 scene = sourceImage.SampleLevel(sourceSampler, input.uv, 0);
    float3 sky = settings.w != 0 ? secondImage.SampleLevel(secondSampler, input.uv, 0).rgb : background.rgb;
    return float4(SafeColor(scene.rgb + sky * (1 - saturate(scene.a))), 1);
}
float3 Read(float2 uv) { return SafeColor(sourceImage.SampleLevel(sourceSampler, uv, 0).rgb); }
float4 Weighted(float3 color)
{
    // Normalize the weighted sum afterwards; never compress the HDR sample itself.
    // Only the first reduction uses this suppression of isolated bright pixels.
    float weight = filter.w != 0 ? rcp(1 + dot(color, float3(.2126,.7152,.0722))) : 1;
    return float4(color * weight, weight);
}
float4 DownsamplePS(Fragment input) : SV_Target0
{
    float2 uv = input.uv, t = filter.xy;
    float3 a = Read(uv + t*float2(-2,-2)), b = Read(uv + t*float2(0,-2)), c = Read(uv + t*float2(2,-2));
    float3 d = Read(uv + t*float2(-2,0)), e = Read(uv), f = Read(uv + t*float2(2,0));
    float3 g = Read(uv + t*float2(-2,2)), h = Read(uv + t*float2(0,2)), i = Read(uv + t*float2(2,2));
    float3 j = Read(uv + t*float2(-1,-1)), k = Read(uv + t*float2(1,-1));
    float3 l = Read(uv + t*float2(-1,1)), m = Read(uv + t*float2(1,1));
    // Overlapping 2x2 groups provide a stable 13-tap first reduction during motion.
    float4 sum = Weighted((j+k+l+m)*.25)*.5;
    sum += (Weighted((a+b+d+e)*.25) + Weighted((b+c+e+f)*.25) +
            Weighted((d+e+g+h)*.25) + Weighted((e+f+h+i)*.25))*.125;
    float3 color = sum.rgb / max(sum.a, 1e-6);
    if (filter.w != 0 && settings.y > 0)
    {
        float brightness = max(color.r, max(color.g, color.b));
        float knee = max(settings.y*.5, .0001);
        float soft = clamp(brightness - settings.y + knee, 0, 2*knee);
        soft = soft*soft/(4*knee);
        color *= max(brightness-settings.y, soft) / max(brightness, .0001);
    }
    return float4(color, 1);
}
float4 UpsamplePS(Fragment input) : SV_Target0
{
    float2 uv = input.uv, t = filter.xy * filter.z;
    float3 sum = Read(uv)*4;
    sum += (Read(uv+t*float2(-1,0)) + Read(uv+t*float2(1,0)) +
            Read(uv+t*float2(0,-1)) + Read(uv+t*float2(0,1)))*2;
    sum += Read(uv+t*float2(-1,-1)) + Read(uv+t*float2(1,-1)) +
           Read(uv+t*float2(-1,1)) + Read(uv+t*float2(1,1));
    float3 fine = secondImage.SampleLevel(secondSampler, uv, 0).rgb;
    // Normalized accumulation keeps intensity independent of pyramid depth.
    return float4(lerp(fine, sum/16, .7), 1);
}
float3 WideTent(float2 uv, float2 stepUV)
{
    // Dense separable taps avoid the disconnected 3x3 images produced by
    // spreading the small-radius kernel across bright, tiny sources like stars.
    // Match the small kernel's spread as radius crosses 2, rather than making
    // the halo narrower when switching to dense sampling. Radius 32 reaches
    // 56 texels either side. Keep every tap: combining adjacent taps is not
    // equivalent at the fractional texel phases used during upsampling, and
    // produces cross-shaped dips around very bright points.
    float width = sqrt(3 * filter.z * filter.z + 1);
    int extent = min(56, (int)ceil(width));
    float3 color = 0;
    float weightSum = 0;
    [loop] for (int offset = -extent; offset <= extent; ++offset)
    {
        float weight = max(0, width - abs(offset));
        color += Read(uv + stepUV * offset) * weight;
        weightSum += weight;
    }
    return color / weightSum;
}
float4 UpsampleHorizontalPS(Fragment input) : SV_Target0
{
    return float4(WideTent(input.uv, float2(filter.x, 0)), 1);
}
float4 UpsampleWidePS(Fragment input) : SV_Target0
{
    float3 wide = WideTent(input.uv, float2(0, filter.y));
    float3 fine = secondImage.SampleLevel(secondSampler, input.uv, 0).rgb;
    return float4(lerp(fine, wide, .7), 1);
}
float4 TonePS(Fragment input) : SV_Target0
{
    float3 color = Read(input.uv);
    if (settings.x > 0) color += secondImage.SampleLevel(secondSampler, input.uv, 0).rgb * settings.x;
    if (settings.z != 0)
    {
        // Identity through the midtones, then a smooth highlight shoulder.
        // Scale all channels together to retain saturated brake-light colors.
        float peak = max(color.r, max(color.g, color.b));
        if (peak > .6) color *= (.6 + .4 * (1 - exp(-(peak-.6)/.4))) / peak;
    }
    return float4(color, 1);
}
float4 EncodePS(Fragment input) : SV_Target0
{
    float3 color = saturate(Read(input.uv));
    color = lerp(12.92*color, 1.055*pow(color, 1.0/2.4)-.055, step(.0031308, color));
    return float4(color, 1);
}
