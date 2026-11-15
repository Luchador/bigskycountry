struct PlanVertex
{
    float4 position : TEXCOORD0; // XZ relative to the batch origin, then rounding residual.
    float4 color : TEXCOORD1;
};
struct PlanFragment
{
    float4 position : SV_Position;
    float4 color : TEXCOORD0;
};
cbuffer PlanCamera : register(b0, space1)
{
    float4 offset; // Origin minus camera center: high XZ, low XZ.
    float4 scale;  // World meters to clip coordinates.
};
PlanFragment VSMain(PlanVertex input)
{
    PlanFragment output;
    // Do the large cancellation before adding the residuals. This keeps narrow
    // paint lines stable even when a long road spans large geographic coordinates.
    precise float2 local = (input.position.xy + offset.xy) + (input.position.zw + offset.zw);
    output.position = float4(local * scale.xy, 0.5, 1);
    output.color = input.color;
    return output;
}
float4 PSMain(PlanFragment input) : SV_Target0
{
    return input.color;
}
