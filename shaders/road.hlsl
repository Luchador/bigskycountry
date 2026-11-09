struct RoadVertex
{
    float4 position : TEXCOORD0;
    float4 color : TEXCOORD1;
};

struct RoadFragment
{
    float4 position : SV_Position;
    float4 color : TEXCOORD0;
};

RoadFragment VSMain(RoadVertex input)
{
    RoadFragment output;
    output.position = input.position;
    output.color = input.color;
    return output;
}

float4 PSMain(RoadFragment input) : SV_Target0
{
    return input.color;
}
