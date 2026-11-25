#include "world_vertex.hlsli"

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
    float4 ssaoControl; // enabled, inverse AO width/height, near depth
    float4 spotControl; // tile columns, tile size, light count, uint4 groups per tile
};

#ifdef __spirv__
#define BSC_COMBINED [[vk::combinedImageSampler]]
#else
#define BSC_COMBINED
#endif
BSC_COMBINED Texture2DArray<float> shadowMap : register(t0, space2);
BSC_COMBINED SamplerComparisonState shadowSampler : register(s0, space2);

BSC_COMBINED Texture2D<float2> ambientMap : register(t1, space2);
BSC_COMBINED SamplerState ambientSampler : register(s1, space2);

float AmbientVisibility(float2 pixel, float3 view)
{
    if (ssaoControl.x == 0) return 1;
    float depth = dot(-view, shadowForward.xyz);
    if (depth <= 0 || dot(view,view) >= 150*150) return 1;
    float2 uv = pixel / (2 * visibility.zw);
    float2 p = uv / ssaoControl.yz - .5;
    float2 base = floor(p), fraction = frac(p);
    float result = 0, total = 0;
    // Bilateral upsampling avoids leaking a foreground object's dark silhouette
    // onto unrelated background pixels. A missing matching surface is unoccluded.
    [unroll] for (int y=0; y<2; ++y)
        [unroll] for (int x=0; x<2; ++x)
        {
            float2 sample = ambientMap.SampleLevel(ambientSampler, (base+float2(x,y)+.5)*ssaoControl.yz, 0);
            float delta = abs(ssaoControl.w / max(sample.y, 1e-8) - depth);
            float weight = (x ? fraction.x : 1-fraction.x) * (y ? fraction.y : 1-fraction.y);
            weight *= exp2(-delta / max(.08, depth*.01));
            result += sample.x * weight;
            total += weight;
        }
    return total > .001 ? saturate(result / total) : 1;
}

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

// SDL exposes storage buffers as raw SRVs on Direct3D 12 (zero structure
// stride). Use byte-addressed reads on both backends, with explicit 16-byte
// record offsets. Tile masks stay integer bits, including patterns that
// represent floating-point denormals or NaNs.
// Storage buffers follow sampled textures in each shader's resource set.
#ifdef BSC_MATERIAL
ByteAddressBuffer spotData : register(t8, space2);
#else
ByteAddressBuffer spotData : register(t2, space2);
#endif

void IlluminateSpots(float2 pixel, float3 view, float3 normal, float3 eye, float gloss, float metal,
                     inout float3 diffuse, inout float3 specular)
{
    if (spotControl.z == 0) return;
    uint2 tile = uint2(pixel / spotControl.y);
    uint tileStart = uint(spotControl.z)*3 + (tile.y*uint(spotControl.x)+tile.x)*uint(spotControl.w);
    for (uint group=0; group<uint(spotControl.w); ++group)
    {
        uint4 mask = spotData.Load4((tileStart+group)*16);
        [unroll] for (uint word=0; word<4; ++word)
        {
            uint bits = mask[word];
            while (bits != 0)
            {
                uint bit = firstbitlow(bits);
                bits &= bits-1;
                uint light = group*128+word*32+bit;
                if (light >= uint(spotControl.z)) continue;
                uint byteOffset = light*48; // Three float4 records per light.
                float4 position = asfloat(spotData.Load4(byteOffset));
                float4 direction = asfloat(spotData.Load4(byteOffset+16));
                float4 intensity = asfloat(spotData.Load4(byteOffset+32));
                float3 toLight = position.xyz + view;
                float distanceSquared = dot(toLight,toLight);
                if (distanceSquared >= position.w*position.w || distanceSquared < 1e-8) continue;
                toLight *= rsqrt(distanceSquared);
                float cone = saturate((-dot(toLight,direction.xyz)-direction.w)*intensity.w);
                if (cone == 0) continue;
                cone = cone*cone*(3-2*cone);
                float fade = 1-distanceSquared/(position.w*position.w);
                float3 radiance = intensity.xyz*(cone*fade*fade/(1+distanceSquared));
                Illuminate(toLight,radiance,normal,eye,gloss,metal,diffuse,specular);
            }
        }
    }
}

// Keep the software renderer's WorldLighting::Shade in sync.
float3 ShadeSurface(float2 pixel, float3 normal, float3 geometricNormal, float3 view, float3 base, float3 orm, float3 emission, float3 transmission = 0)
{
    float3 diffuse = skyAmbient.rgb * orm.r * AmbientVisibility(pixel, view);
    float3 specular = 0;
    float3 eye = UnitVector(view);
    float shadow = ShadowVisibility(geometricNormal,view);
    Illuminate(sunDirection.xyz, sunRadiance.rgb * (shadowControl.z == 0 ? shadow : 1), normal, eye, 1 - orm.g, orm.b, diffuse, specular);
    Illuminate(moonDirection.xyz, moonRadiance.rgb * (shadowControl.z != 0 ? shadow : 1), normal, eye, 1 - orm.g, orm.b, diffuse, specular);
    IlluminateSpots(pixel,view,normal,eye,1-orm.g,orm.b,diffuse,specular);
    float3 c = emission + base * diffuse + lerp(0.08, base, orm.b) * specular;
    // Thin-sheet subsurface approximation: tinted light transmitted through the
    // back of a leaf. Occluders still attenuate it through the cascaded shadows.
    c += transmission * (sunRadiance.rgb * (shadowControl.z == 0 ? shadow : 1) * saturate(-dot(normal, sunDirection.xyz)) +
                         moonRadiance.rgb * (shadowControl.z != 0 ? shadow : 1) * saturate(-dot(normal, moonDirection.xyz)));
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
    if (input.normal.w == 2) return float4(ToDisplay(ApplyFog(input.color.rgb,input.view)),input.color.a);
    float3 c = ShadeSurface(input.position.xy, UnitVector(input.normal.xyz), UnitVector(input.normal.xyz), input.view, input.color.rgb, float3(1, 1, 0), 0);
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

#ifdef BSC_MATERIAL
BSC_COMBINED Texture2D<float4> baseColorMap : register(t2, space2);
BSC_COMBINED Texture2D<float2> normalMap : register(t3, space2);
BSC_COMBINED Texture2D<float4> ormMap : register(t4, space2);
BSC_COMBINED Texture2D<float4> emissionOrOuterColor : register(t5, space2);
BSC_COMBINED Texture2D<float4> alphaOrOuterNormal : register(t6, space2);
BSC_COMBINED Texture2D<float4> outerOrmMap : register(t7, space2);
BSC_COMBINED SamplerState outerOrmSampler : register(s7, space2);
BSC_COMBINED SamplerState colorSampler : register(s2, space2);
BSC_COMBINED SamplerState normalSampler : register(s3, space2);
BSC_COMBINED SamplerState ormSampler : register(s4, space2);

BSC_COMBINED SamplerState emissiveSampler : register(s5, space2);
BSC_COMBINED SamplerState opacitySampler : register(s6, space2);
cbuffer MaterialProperties : register(b1, space3)
{
    float4 tint;
    uint alphaCutout, unlit, transition, foliage;
    float4 subsurfaceProperties; // Strength, has subsurface map, use vertex alpha AO, alpha cutoff.
};

float3 DecodeNormalXY(float2 channels)
{
    float2 xy = 2 * channels - 1;
    return float3(xy, sqrt(saturate(1 - dot(xy, xy))));
}

#ifdef BSC_MSAA_SAMPLES
uint AlphaToCoverage(float alpha, float cutoff, float2 position)
{
    // Smooth the material's cutoff over approximately one pixel. At zero the
    // cutout is disabled; at one only fully opaque samples survive.
    float coverage = saturate((alpha - cutoff) / max(fwidth(alpha), 1.0 / 255.0) + 0.5);
    if (cutoff <= 0) coverage = 1;
    else if (cutoff >= 1) coverage = alpha >= 1 ? 1 : 0;
    uint covered = uint(coverage * BSC_MSAA_SAMPLES + 0.5);
    uint mask = (1u << covered) - 1u;
    uint2 pixel = uint2(position);
    uint rotation = (pixel.x + 3u * pixel.y) & (BSC_MSAA_SAMPLES - 1u);
    // Rotate a nested mask spatially, without time-dependent noise. Use explicit
    // coverage rather than writing fractional alpha: SDL later composites the
    // resolved target over the sky, so surviving samples must have alpha=1.
    return ((mask << rotation) | (mask >> (BSC_MSAA_SAMPLES - rotation))) & ((1u << BSC_MSAA_SAMPLES) - 1u);
}
#endif

float4 MaterialPS(WorldFragment input, bool frontFace : SV_IsFrontFace
#ifdef BSC_MSAA_SAMPLES
                  , out uint sampleCoverage : SV_Coverage
#endif
                  ) : SV_Target0
{
#ifdef BSC_MSAA_SAMPLES
    sampleCoverage = (1u << BSC_MSAA_SAMPLES) - 1u;
#endif
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
    if (alphaCutout)
    {
        float opacity = base.a * alphaOrOuterNormal.Sample(opacitySampler, input.uv).r;
#ifdef BSC_MSAA_SAMPLES
        sampleCoverage = AlphaToCoverage(opacity, subsurfaceProperties.w, input.position.xy);
        if (sampleCoverage == 0) discard;
#else
        clip(opacity - subsurfaceProperties.w);
#endif
    }
    float3 orm = ormMap.Sample(ormSampler, input.uv).rgb;
    float3 surfaceNormal = DecodeNormalXY(normalMap.Sample(normalSampler, input.uv));
    float3 n = UnitVector(input.normal.xyz);
    float3 t = UnitVector(input.tangent.xyz - n * dot(n, input.tangent.xyz));
    float3 b = cross(n, t) * input.tangent.w;
    float3 mapped = UnitVector(t * surfaceNormal.x + b * surfaceNormal.y + n * surfaceNormal.z);
    float3 glow = 0;
    if (transition)
    {
        float weight = saturate(input.color.r);
        float3 outerColor = emissionOrOuterColor.Sample(emissiveSampler, input.outerUV).rgb;
        base.rgb = lerp(outerColor, base.rgb, weight);
        orm = lerp(outerOrmMap.Sample(outerOrmSampler, input.outerUV).rgb, orm, weight);
        float3 outerNormal = DecodeNormalXY(alphaOrOuterNormal.Sample(opacitySampler, input.outerUV).rg);
        // The inner layer follows the road; the outer layer follows terrain.
        // Interpret each normal in its own frame before blending in world space.
        float3 outerT = UnitVector(input.outerTangent.xyz - n * dot(n, input.outerTangent.xyz));
        float3 outerB = cross(n, outerT) * input.outerTangent.w;
        float3 outerMapped = UnitVector(outerT * outerNormal.x + outerB * outerNormal.y + n * outerNormal.z);
        mapped = UnitVector(lerp(outerMapped, mapped, weight));
    }
    else glow = emissionOrOuterColor.Sample(emissiveSampler, input.uv).rgb;
    // Vertex alpha is ambient visibility, independent of cutout opacity and
    // alpha-to-coverage. Apply it only when enabled by this material.
    if (subsurfaceProperties.z != 0) orm.r *= saturate(input.color.a);
    float3 transmission = 0;
    float3 shadowNormal = n;
    if (foliage)
    {
        // Flip the complete tangent-space result on backs, not just its Z axis.
        if (!frontFace) { mapped = -mapped; n = -n; }
        float4 subsurface = subsurfaceProperties.y != 0 ? outerOrmMap.Sample(outerOrmSampler, input.uv) : float4(base.rgb, 1);
        transmission = subsurface.rgb * (subsurface.a * subsurfaceProperties.x);
        float3 shadowLight = shadowControl.z == 0 ? sunDirection.xyz : moonDirection.xyz;
        // Bias toward the shadow light on either side of a leaf. The ordinary
        // backface early-out would otherwise let transmitted light ignore shade.
        shadowNormal = dot(n, shadowLight) < 0 ? -n : n;
    }
    float3 c = ShadeSurface(input.position.xy, mapped, shadowNormal, input.view, base.rgb, orm, glow, transmission);
    if (input.normal.w != 0) c = ApplyFog(c, input.view);
    // SDL composites the world into a display-encoded color target.
    return float4(ToDisplay(c), 1);
}

#endif // BSC_MATERIAL
