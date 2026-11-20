cbuffer StarView : register(b0, space1)
{
    float4 viewRight;   // equatorial -> view, projection scale X
    float4 viewUp;      // equatorial -> view, projection scale Y
    float4 viewForward;// equatorial -> view, daylight fade
    float4 horizonUp;   // equatorial -> world altitude, sprite scale
    float4 pixelSize;   // 2/viewport width, 2/viewport height
};

struct StarInput
{
    float4 directionMagnitude : TEXCOORD0;
    float4 colorBrightness : TEXCOORD1;
};
struct StarPixel
{
    float4 position : SV_Position;
    float2 disc : TEXCOORD0;
    float3 color : TEXCOORD1;
};

StarPixel VSMain(StarInput star, uint vertex : SV_VertexID)
{
    static const float2 corners[6] = {
        float2(-1,-1), float2(1,-1), float2(1,1),
        float2(-1,-1), float2(1,1), float2(-1,1)
    };
    StarPixel output;
    output.disc = corners[vertex];
    float3 direction = star.directionMagnitude.xyz;
    float depth = dot(viewForward.xyz, direction);
    float altitude = dot(horizonUp.xyz, direction);
    float fade = viewForward.w * smoothstep(0.0, 0.0871557427, altitude);
    output.color = star.colorBrightness.rgb * (star.colorBrightness.a * fade);
    if (depth <= 0.001 || altitude <= 0)
    {
        output.position = float4(2, 2, 0, 1);
        output.color = 0;
        return output;
    }
    float radius = (1.5 + 2.0 * sqrt(star.colorBrightness.a)) * horizonUp.w;
    float2 center = float2(dot(viewRight.xyz, direction) * viewRight.w,
                           dot(viewUp.xyz, direction) * viewUp.w);
    output.position = float4(center + output.disc * radius * pixelSize.xy * depth, 0.5 * depth, depth);
    return output;
}

float4 PSMain(StarPixel input) : SV_Target0
{
    float r2 = dot(input.disc, input.disc);
    float coverage = saturate((exp(-5.0 * r2) - 0.006737947) / 0.993262053);
    return float4(input.color * coverage, 1);
}
