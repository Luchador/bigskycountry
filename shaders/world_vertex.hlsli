struct WorldVertex
{
    float4 position : TEXCOORD0;
    float4 color : TEXCOORD1;
    float2 uv : TEXCOORD2;
    float4 normal : TEXCOORD3; // .w: 0 editor colors, 1 scene lighting, 2 emissive surface.
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

// Mode 0 is the dynamic clip-space stream. Resident vertices are relative to
// a nearby origin so large geographic coordinates retain local precision.
cbuffer WorldCamera : register(b0, space1)
{
    float4 cameraRight, cameraUp, cameraForward;
    float4 originMinusEye; // w = 1 for shaded geometry, 2 for wireframe
    float4 originRemainder, wireColor;
};

WorldFragment VSMain(WorldVertex input, uint vertexID : SV_VertexID)
{
    WorldFragment output;
    output.position = input.position;
    output.color = input.color;
    output.uv = input.uv;
    output.normal = input.normal;
    output.tangent = input.tangent;
    output.view = input.view;
    if (originMinusEye.w != 0)
    {
        // Preserve centimeter-sized features when a long road's origin is
        // kilometers from the camera. Keep the two sums in this order.
        precise float3 position = (input.position.xyz + originMinusEye.xyz) + (input.view + originRemainder.xyz);
        output.position = float4(dot(position, cameraRight.xyz), dot(position, cameraUp.xyz),
                                 cameraForward.w, dot(position, cameraForward.xyz));
        output.view = -position;
        if (originMinusEye.w == 2)
        {
            uint corner = vertexID % 3;
            output.uv = float2(corner == 0, corner == 1);
            output.color = wireColor;
        }
    }
    return output;
}
