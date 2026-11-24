struct Vertex
{
    float4 position : TEXCOORD0;
    float4 color : TEXCOORD1;
    float2 uv : TEXCOORD2;
    float4 normal : TEXCOORD3;
    float4 tangent : TEXCOORD4;
    float3 residual : TEXCOORD5;
};
struct Fragment
{
    float4 position : SV_Position;
    float2 uv : TEXCOORD0;
    float lit : TEXCOORD1;
};
cbuffer ShadowCamera : register(b0, space1)
{
    float4 lightX, lightY, lightZ;
    float4 originMinusEye, originRemainder;
};
Fragment ShadowVS(Vertex input)
{
    Fragment output;
    precise float3 p = originMinusEye.w != 0 ?
        (input.position.xyz + originMinusEye.xyz) + (input.residual + originRemainder.xyz) : -input.residual;
    output.position = float4(dot(p,lightX.xyz)+lightX.w, dot(p,lightY.xyz)+lightY.w, dot(p,lightZ.xyz)+lightZ.w, 1);
    output.uv = input.uv;
    output.lit = input.normal.w;
    return output;
}
void ShadowPS(Fragment input)
{
    clip(input.lit-.5); // Editor labels, outlines and handles do not cast shadows.
}
#ifdef __spirv__
#define BSC_COMBINED [[vk::combinedImageSampler]]
#else
#define BSC_COMBINED
#endif
BSC_COMBINED Texture2D<float4> colorMap : register(t0, space2);
BSC_COMBINED SamplerState colorSampler : register(s0, space2);
BSC_COMBINED Texture2D<float4> opacityMap : register(t1, space2);
BSC_COMBINED SamplerState opacitySampler : register(s1, space2);
cbuffer CutoutProperties : register(b0, space3)
{
    float4 cutout; // Alpha cutoff in x.
};
void ShadowCutoutPS(Fragment input)
{
    clip(input.lit-.5);
    clip(colorMap.Sample(colorSampler,input.uv).a * opacityMap.Sample(opacitySampler,input.uv).r - cutout.x);
}
