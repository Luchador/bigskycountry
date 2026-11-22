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

cbuffer WorldLights : register(b0, space3)
{
    float4 sunDirection, sunRadiance;
    float4 moonDirection, moonRadiance;
    float4 skyAmbient;
    float4 fogDensity; // density, height falloff, eye height minus base, start distance
    float4 fogAir;     // uniform air density in x; zero density disables fog
    float4 fogZenith, fogOpposite, fogSunward, fogSun;
    float4 shadowX[4], shadowY[4], shadowZ[4];
    float4 shadowParams[4]; // far view depth, texel meters, depth bias, blend start
    float4 shadowControl;   // cascade count, inverse map size, moon flag, horizon fade
    float4 shadowForward;
    float4 visibility; // radius squared, inverse focal length, viewport half width/height
};

#ifdef __spirv__
#define BSC_COMBINED [[vk::combinedImageSampler]]
#else
#define BSC_COMBINED
#endif
BSC_COMBINED Texture2DArray<float> shadowMap : register(t0, space2);
BSC_COMBINED SamplerComparisonState shadowSampler : register(s0, space2);

float CascadeShadow(int cascade, float3 p, float3 normal)
{
    // Use the geometric normal, not the normal-map perturbation. Correct each
    // PCF tap for the receiver plane so sloped terrain does not shadow itself.
    float3 zRow = shadowZ[cascade].xyz;
    float depthScale = length(zRow);
    float facing = dot(normal, -zRow/depthScale);
    if (facing <= 0) return 1;
    float2 planeSlope = float2(dot(normal,normalize(shadowX[cascade].xyz)),
                             -dot(normal,normalize(shadowY[cascade].xyz))) / max(facing,.05);
    float2 tapDepth = planeSlope * shadowParams[cascade].y * depthScale;
    float bias = shadowParams[cascade].z + .75*(abs(tapDepth.x)+abs(tapDepth.y));
    p += normal * shadowParams[cascade].y * .25;
    float3 q = float3(dot(p,shadowX[cascade].xyz)+shadowX[cascade].w,
                      dot(p,shadowY[cascade].xyz)+shadowY[cascade].w,
                      dot(p,shadowZ[cascade].xyz)+shadowZ[cascade].w);
    float2 uv = float2(q.x*.5+.5, .5-q.y*.5);
    if (any(uv < shadowControl.y*2) || any(uv > 1-shadowControl.y*2) || q.z <= 0 || q.z >= 1) return 1;
    float result = 0;
    [unroll] for (int y=-1;y<=1;++y)
        [unroll] for (int x=-1;x<=1;++x)
            result += shadowMap.SampleCmpLevelZero(shadowSampler, float3(uv+float2(x,y)*shadowControl.y,cascade),
                                                   q.z+dot(tapDepth,float2(x,y))-bias);
    return result/9;
}

float ShadowVisibility(float3 normal, float3 view)
{
    if (shadowControl.x == 0) return 1;
    float depth = dot(-view,shadowForward.xyz);
    if (depth <= 0 || depth >= shadowParams[3].x) return 1;
    int c = depth > shadowParams[0].x ? 1 : 0;
    if (depth > shadowParams[1].x) c=2;
    if (depth > shadowParams[2].x) c=3;
    float result = CascadeShadow(c,-view,normal);
    float blend = saturate((depth-shadowParams[c].w) / max(shadowParams[c].x-shadowParams[c].w,.001));
    if (blend > 0)
    {
        float next = 1;
        if (c < 3) next = CascadeShadow(c+1,-view,normal);
        result = lerp(result,next,blend);
    }
    return lerp(1,result,shadowControl.w);
}

void ClipVisibility(WorldFragment input)
{
    if (visibility.x > 0)
    {
        // SV_Position.w is reciprocal clip W (view depth). Reconstruct distance
        // for every pass, including wireframe and screen-expanded orange outlines.
        float2 ray = (input.position.xy - visibility.zw) * visibility.y;
        clip(visibility.x * input.position.w * input.position.w - (1 + dot(ray, ray)));
    }
}

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

float3 ToDisplay(float3 c)
{
    c = saturate(c);
    return lerp(12.92 * c, 1.055 * pow(c, 1.0 / 2.4) - 0.055, step(0.0031308, c));
}

// Keep the software renderer's WorldLighting::Shade in sync.
float3 ShadeSurface(float3 normal, float3 geometricNormal, float3 view, float3 base, float3 orm, float3 emission)
{
    float3 diffuse = skyAmbient.rgb * orm.r;
    float3 specular = 0;
    float3 eye = UnitVector(view);
    float shadow = ShadowVisibility(geometricNormal,view);
    Illuminate(sunDirection.xyz, sunRadiance.rgb * (shadowControl.z == 0 ? shadow : 1), normal, eye, 1 - orm.g, orm.b, diffuse, specular);
    Illuminate(moonDirection.xyz, moonRadiance.rgb * (shadowControl.z != 0 ? shadow : 1), normal, eye, 1 - orm.g, orm.b, diffuse, specular);
    float3 c = emission + base * diffuse + lerp(0.08, base, orm.b) * specular;
    return c;
}

float3 ToLinear(float3 c)
{
    return lerp(c / 12.92, pow((c + 0.055) / 1.055, 2.4), step(0.04045, c));
}

// Analytic height-fog integration: no ray marching or extra render target.
// Keep the software equivalent in WorldFog in sync.
float3 ApplyFog(float3 linearColor, float3 view)
{
    if (fogDensity.x == 0 && fogAir.x == 0) return linearColor;
    float distance = length(view);
    if (distance <= fogDensity.w) return linearColor;
    float3 ray = -view / distance;
    float y0 = fogDensity.z + ray.y * fogDensity.w;
    float y1 = fogDensity.z - view.y;
    float span = fogDensity.y * abs(y1 - y0);
    float average = span < 0.001 ? 1 - span * 0.5 + span * span / 6 : (1 - exp(-span)) / span;
    float lowerDensity = fogDensity.x * exp(clamp(-fogDensity.y * min(y0, y1), -80, 8));
    float opticalDepth = (distance - fogDensity.w) * (fogAir.x + lowerDensity * average);
    float transmission = exp(-clamp(opticalDepth, 0, 80));

    // Match the sky's sunward/opposite horizon and its day/night palette.
    float facing = saturate(0.5 + 0.5 * dot(ray, fogSun.xyz));
    float3 haze = lerp(fogOpposite.rgb, fogSunward.rgb, facing * facing);
    if (ray.y >= 0) haze = lerp(haze, fogZenith.rgb, sqrt(ray.y));
    else
    {
        float depth = min(-ray.y * 2, 1);
        haze = lerp(haze, lerp(fogOpposite.rgb, fogZenith.rgb, 0.55), depth * (2 - depth));
    }
    return lerp(ToLinear(haze), linearColor, transmission);
}

float4 PSMain(WorldFragment input) : SV_Target0
{
    ClipVisibility(input);
    if (input.normal.w == 0) return input.color;
    float3 c = ShadeSurface(UnitVector(input.normal.xyz), UnitVector(input.normal.xyz), input.view, input.color.rgb, float3(1, 1, 0), 0);
    return float4(ToDisplay(ApplyFog(c, input.view)), input.color.a);
}

// Multiplying by reciprocal clip W makes each component a linear screen-space
// edge equation. Its gradient converts the threshold to pixels at any zoom.
float4 WireframePS(WorldFragment input) : SV_Target0
{
    ClipVisibility(input);
    float3 edge = float3(input.uv, 1 - input.uv.x - input.uv.y) * input.position.w;
    float3 dx = ddx(edge), dy = ddy(edge);
    const float halfWidth = 1.0;
    bool onEdge = any(edge * edge <= halfWidth * halfWidth * (dx * dx + dy * dy));
    // Clear interiors to sky while writing depth and selection stencil as usual.
    return onEdge ? float4(input.color.rgb, 1.0) : 0;
}

BSC_COMBINED Texture2D<float4> baseColorMap : register(t1, space2);
BSC_COMBINED Texture2D<float2> normalMap : register(t2, space2);
BSC_COMBINED Texture2D<float4> ormMap : register(t3, space2);
BSC_COMBINED Texture2D<float4> emissionOrOuterColor : register(t4, space2);
BSC_COMBINED Texture2D<float4> alphaOrOuterNormal : register(t5, space2);
BSC_COMBINED Texture2D<float4> outerOrmMap : register(t6, space2);
BSC_COMBINED SamplerState outerOrmSampler : register(s6, space2);
BSC_COMBINED SamplerState colorSampler : register(s1, space2);
BSC_COMBINED SamplerState normalSampler : register(s2, space2);
BSC_COMBINED SamplerState ormSampler : register(s3, space2);

BSC_COMBINED SamplerState emissiveSampler : register(s4, space2);
BSC_COMBINED SamplerState opacitySampler : register(s5, space2);
cbuffer MaterialProperties : register(b1, space3)
{
    float4 tint;
    uint alphaCutout, unlit, transition;
};

float3 DecodeNormalXY(float2 channels)
{
    float2 xy = 2 * channels - 1;
    return float3(xy, sqrt(saturate(1 - dot(xy, xy))));
}

float4 MaterialPS(WorldFragment input) : SV_Target0
{
    ClipVisibility(input);
    if (unlit)
    {
        float alpha = alphaOrOuterNormal.Sample(opacitySampler, input.uv).r * tint.a;
        if (alpha <= 0) discard;
        float3 c = emissionOrOuterColor.Sample(emissiveSampler, input.uv).rgb * tint.rgb;
        if (input.normal.w != 0) c = ApplyFog(c, input.view);
        return float4(ToDisplay(c), alpha);
    }
    float4 base = baseColorMap.Sample(colorSampler, input.uv);
    if (alphaCutout) clip(base.a * alphaOrOuterNormal.Sample(opacitySampler, input.uv).r - 0.5);
    float3 orm = ormMap.Sample(ormSampler, input.uv).rgb;
    float3 layerColor = emissionOrOuterColor.Sample(emissiveSampler, input.uv).rgb;
    float3 surfaceNormal = DecodeNormalXY(normalMap.Sample(normalSampler, input.uv));
    float3 glow = 0;
    if (transition)
    {
        float weight = saturate(input.color.r);
        base.rgb = lerp(layerColor, base.rgb, weight);
        orm = lerp(outerOrmMap.Sample(outerOrmSampler, input.uv).rgb, orm, weight);
        float3 outerNormal = DecodeNormalXY(alphaOrOuterNormal.Sample(opacitySampler, input.uv).rg);
        surfaceNormal = lerp(outerNormal, surfaceNormal, weight);
    }
    else glow = layerColor;
    float3 n = UnitVector(input.normal.xyz);
    float3 t = UnitVector(input.tangent.xyz - n * dot(n, input.tangent.xyz));
    float3 b = cross(n, t) * input.tangent.w;
    float3 mapped = UnitVector(t * surfaceNormal.x + b * surfaceNormal.y + n * surfaceNormal.z);
    float3 c = ShadeSurface(mapped, n, input.view, base.rgb, orm, glow);
    if (input.normal.w != 0) c = ApplyFog(c, input.view);
    // SDL composites the world into a display-encoded color target.
    return float4(ToDisplay(c), 1);
}
