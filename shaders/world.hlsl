struct WorldVertex
{
    float4 position : TEXCOORD0;
    float4 color : TEXCOORD1;
};

struct WorldFragment
{
    float4 position : SV_Position;
    float4 color : TEXCOORD0;
};

WorldFragment VSMain(WorldVertex input)
{
    WorldFragment output;
    output.position = input.position;
    output.color = input.color;
    return output;
}

float4 PSMain(WorldFragment input) : SV_Target0
{
    return input.color;
}
