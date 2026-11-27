// Surface-only inland water. The opaque scene is captured before this pass;
// it must never sample the color/depth attachments being written by water.
#define BSC_WATER 1
#include "world.hlsl"

BSC_COMBINED Texture2D<float2> waterNormal : register(t2, space2);
BSC_COMBINED SamplerState waterNormalSampler : register(s2, space2);
BSC_COMBINED Texture2D<float4> opaqueScene : register(t3, space2);
BSC_COMBINED SamplerState sceneSampler : register(s3, space2);
BSC_COMBINED Texture2D<float> opaqueDepth : register(t4, space2);
BSC_COMBINED SamplerState depthSampler : register(s4, space2);
cbuffer WaterProperties : register(b1, space3)
{
    float4 waterColor; // linear scattering color, roughness
    float4 waves; // slope strength, wavelength scale, speed, clarity depth
    float4 optics; // refraction distance, shoreline width, foam strength, scene ready
    float4 waterRight, waterUp, waterForward; // unit axes, projection scales/near
    float4 wavePhase; // phase at the eye, calculated in double on CPU
    float4 waterScroll; // wrapped detail UV animation
};
static const float2 waveDirections[4] = {float2(1,0), float2(.6,.8), float2(-.8,.6), float2(.28,-.96)};
static const float4 waveLengths = float4(1,.57,.23,.11);
static const float4 waveWeights = float4(.52,.28,.14,.06);

float2 ProjectWater(float3 p)
{
    float z = max(dot(p,waterForward.xyz), .01);
    return float2(.5 + .5*dot(p,waterRight.xyz)*waterRight.w/z,
                  .5 - .5*dot(p,waterUp.xyz)*waterUp.w/z);
}
float3 ReconstructWater(float2 uv, float depth)
{
    float z = waterForward.w / max(depth, 1e-9);
    return (waterForward.xyz + waterRight.xyz*((uv.x*2-1)/waterRight.w) +
            waterUp.xyz*((1-uv.y*2)/waterUp.w))*z;
}
float3 WaterSky(float3 ray)
{
    float facing = saturate(.5 + .5*dot(ray,fogSun.xyz));
    float3 horizon = lerp(fogOpposite.rgb,fogSunward.rgb,facing*facing);
    float3 zenith = fogZenith.rgb;
    // Mesh previews have no world fog palette.
    if (!any(zenith) && !any(horizon))
    {
        horizon = ToDisplay(skyAmbient.rgb*2);
        zenith = ToDisplay(skyAmbient.rgb*float3(.65,.95,1.5));
    }
    return ToLinear(lerp(horizon,zenith,sqrt(saturate(ray.y))));
}
float3 WaterReflection(float3 p, float3 ray, float roughness, float3 sky)
{
    if (optics.w == 0 || roughness >= .8) return sky;
    float last = .08;
    // Bounded screen-space reflection, with sky fallback at screen edges,
    // missing geometry and rays that leave the captured scene.
    [loop] for (int i=1; i<=24; ++i)
    {
        float distance = .08 + i*i*.18;
        float3 q = p + ray*distance;
        float z = dot(q,waterForward.xyz);
        if (z <= waterForward.w) break;
        float2 uv = ProjectWater(q);
        if (any(uv <= 0) || any(uv >= 1)) break;
        float depth = opaqueDepth.SampleLevel(depthSampler,uv,0);
        float delta = z - waterForward.w/max(depth,1e-9);
        if (depth > 0 && delta > 0)
        {
            float lo = last, hi = distance;
            [unroll] for (int j=0; j<4; ++j)
            {
                float mid = (lo+hi)*.5;
                float3 r = p+ray*mid;
                float2 st = ProjectWater(r);
                float d = opaqueDepth.SampleLevel(depthSampler,st,0);
                if (dot(r,waterForward.xyz) > waterForward.w/max(d,1e-9)) hi=mid;
                else lo=mid;
            }
            q = p+ray*hi;
            uv = ProjectWater(q);
            depth = opaqueDepth.SampleLevel(depthSampler,uv,0);
            float3 hit = ReconstructWater(uv,depth);
            float error = abs(dot(q-hit,waterForward.xyz));
            // Reject submerged geometry and thickness/disocclusion false hits.
            if (hit.y > p.y+.03 && error < max(.2,length(q)*.005))
            {
                float edge = saturate(min(min(uv.x,uv.y),min(1-uv.x,1-uv.y))*12);
                float4 scene = opaqueScene.SampleLevel(sceneSampler,uv,0);
                float confidence = edge * scene.a * (1-smoothstep(.35,.8,roughness));
                return lerp(sky,scene.rgb/max(scene.a,.001),confidence);
            }
            break;
        }
        last = distance;
    }
    return sky;
}
float3 WaterGlint(float3 light, float3 radiance, float3 n, float3 eye, float roughness)
{
    float nl = saturate(dot(n,light)), nv = max(saturate(dot(n,eye)),.01);
    float3 h = UnitVector(light+eye);
    float nh = saturate(dot(n,h)), vh = saturate(dot(eye,h));
    float a2 = pow(roughness,4);
    float denominator = nh*nh*(a2-1)+1;
    float distribution = a2/(3.14159265*max(denominator*denominator,1e-8));
    float k = (roughness+1)*(roughness+1)/8;
    float geometry = nl/(nl*(1-k)+k) * nv/(nv*(1-k)+k);
    float fresnel = .02037 + .97963*pow(1-vh,5); // air/water IOR 1.333
    return radiance * min(30, distribution*geometry*fresnel/(4*nv));
}

float4 WaterPS(WorldFragment input) : SV_Target0
{
    ClipVisibility(input);
    float3 p = -input.view, eye = UnitVector(input.view);
    float3 geometric = UnitVector(input.normal.xyz);
    float2 slope = 0;
    [unroll] for (int i=0; i<4; ++i)
    {
        float phase = dot(p.xz,waveDirections[i]) * (6.2831853/(waves.y*waveLengths[i])) + wavePhase[i];
        float filtered = 1-smoothstep(.6,3,length(float2(ddx(phase),ddy(phase))));
        slope += waveDirections[i]*(cos(phase)*waveWeights[i]*filtered);
    }
    float2 detail = waterNormal.Sample(waterNormalSampler,input.uv/waves.y+waterScroll.xy)*2-1;
    detail += waterNormal.Sample(waterNormalSampler,input.uv/waves.y*1.71-waterScroll.yx)*2-1;
    // Optional detail normal follows authored UVs; procedural ripples stay
    // continuous across lake/river and streamed-cell boundaries in world space.
    float3 t = UnitVector(input.tangent.xyz-geometric*dot(geometric,input.tangent.xyz));
    float3 b = cross(geometric,t)*input.tangent.w;
    float3 n = UnitVector(geometric-float3(slope.x,0,slope.y)*waves.x + (t*detail.x+b*detail.y)*(.3*waves.x));
    float variance = dot(ddx(n),ddx(n))+dot(ddy(n),ddy(n));
    float roughness = clamp(sqrt(waterColor.w*waterColor.w+variance*.5),.04,1);
    float2 uv = input.position.xy/(2*visibility.zw);
    float waterZ = dot(p,waterForward.xyz);
    float depth = optics.w != 0 ? opaqueDepth.SampleLevel(depthSampler,uv,0) : 0;
    float3 bottom = depth > 0 ? ReconstructWater(uv,depth) : p-float3(0,waves.w*12,0);
    float thickness = depth > 0 ? max(0,length(bottom)-length(p)) : waves.w*12;
    float shore = depth > 0 ? saturate(max(0,p.y-bottom.y)/optics.y) : 1;
    float3 offset = float3(n.x,0,n.z)*(optics.x*shore);
    float2 refractedUV = uv + float2(dot(offset,waterRight.xyz)*waterRight.w,
                                    -dot(offset,waterUp.xyz)*waterUp.w)*(.5/max(waterZ,.1));
    float2 inset = .5/(2*visibility.zw);
    refractedUV = clamp(refractedUV,inset,1-inset);
    if (optics.w != 0)
    {
        float refractedDepth = opaqueDepth.SampleLevel(depthSampler,refractedUV,0);
        float3 refractedPosition = ReconstructWater(refractedUV,refractedDepth);
        if (refractedDepth == 0 || dot(refractedPosition,waterForward.xyz) <= waterZ+.01 || refractedPosition.y > p.y)
            refractedUV = uv; // Do not pull banks, bridges or cars into the water.
        else thickness = length(refractedPosition-p);
    }
    float4 background = optics.w != 0 ? opaqueScene.SampleLevel(sceneSampler,refractedUV,0) : 0;
    float3 transmission = exp2(-thickness/waves.w*float3(1.4,1,.7))*background.a;
    float shadow = ShadowVisibility(geometric,input.view);
    float3 sun = sunRadiance.rgb*(shadowControl.z == 0 ? shadow : 1);
    float3 moon = moonRadiance.rgb*(shadowControl.z != 0 ? shadow : 1);
    float3 irradiance = skyAmbient.rgb + .5*(sun*saturate(sunDirection.y)+moon*saturate(moonDirection.y));
    float3 scattering = waterColor.rgb*irradiance;
    float3 body = (background.rgb/max(background.a,.001))*transmission + scattering*(1-transmission);
    float3 reflectionRay = reflect(-eye,n);
    float3 reflection = WaterReflection(p+geometric*.03,reflectionRay,roughness,WaterSky(reflectionRay));
    float fresnel = .02037 + .97963*pow(1-saturate(dot(n,eye)),5);
    float3 color = lerp(body,reflection,fresnel);
    color += WaterGlint(sunDirection.xyz,sun,n,eye,roughness) + WaterGlint(moonDirection.xyz,moon,n,eye,roughness);
    float3 diffuse = 0, specular = 0;
    IlluminateSpots(input.position.xy,input.view,n,eye,1-roughness,0,diffuse,specular);
    color += waterColor.rgb*diffuse*.15 + specular*.02037;
    // A narrow, broken contact band, intentionally restrained for lakes/rivers.
    float foamPattern = .5+.5*sin(p.x*(6.2831853/waves.y)+wavePhase.x);
    float foam = optics.z*(1-shore)*smoothstep(.25,.75,foamPattern);
    color = lerp(color,irradiance*.7,foam);
    color = ApplyFog(color,input.view);
    // Blend in shader, retaining opaque alpha for MSAA and SDL sky composition.
    float4 shoreScene = optics.w != 0 ? opaqueScene.SampleLevel(sceneSampler,uv,0) : 0;
    color = lerp(shoreScene.rgb/max(shoreScene.a,.001),color,lerp(1,shore,shoreScene.a));
    return float4(color,1);
}
