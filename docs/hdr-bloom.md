# HDR and bloom

Both executables read the `[Graphics]` settings from `config.ini` beside the
executable, once at startup. The build inserts missing entries while preserving
existing preferences. Rebuild and restart after applying the implementation;
existing worlds and cooked geometry do not need recooking.

| Key | Default | Accepted values |
| --- | --- | --- |
| `Bloom` | `1` | `0` off, `1` on |
| `BloomIntensity` | `0.08` | `0..100`; zero skips bloom work |
| `BloomRadius` | `1.0` | `0.5..32`; larger values spread the halo farther and cost more GPU time |
| `BloomThreshold` | `1.0` | `0..16` linear brightness; zero includes all scene colors |

A positive threshold has a soft knee of half its value. Begin with the defaults;
raise the threshold to restrict glow to brighter highlights, or reduce intensity
for a more restrained effect. No automatic exposure adaptation is applied.
Disabling bloom retains HDR and highlight tone mapping.
Radii above 2 use a denser horizontal/vertical tent filter so small bright sources
produce a continuous halo rather than a grid of separated samples. These settings
cost an extra filter pass and an intermediate image per upsample level; the
default radius retains the original filter and allocation sizes.

Bright stars receive an additional HDR exposure in `shaders/stars.hlsl`:
`zeroMagnitudeRadiance` is 256, scaled by `10^(-0.4 * magnitude)`. The boost
has full weight through magnitude 1 and fades smoothly to zero at magnitude 3.
The original faint-star baseline remains. Both contributions follow the existing
sunrise/sunset and horizon fades, and the boost retains the catalog color and
sprite opacity. The GPU's floating-point star target preserves this light until
scene bloom and tone mapping; sprite sizes are unchanged.

## Frame order

1. Capture the sky in a linear floating-point target. The sun's disc has HDR
   emission; the existing sky palette and atmospheric aureoles are retained.
   Moon textures and star output are converted to linear light before composition.
2. Render the opaque world with shadows and SSAO into RGBA16F, using the configured
   0/2/4 MSAA setting. Cutout alpha-to-coverage continues to preserve opaque samples.
3. Capture resolved opaque HDR color for water refraction/reflection, then render
   water and world translucency. Water samples linear colors without gamma decoding.
4. Resolve the world and composite it over the HDR sky, before extracting bloom.
5. Downsample through up to six levels, starting at half dimensions. The first
   reduction uses a 13-tap filter and normalized brightness weighting to suppress
   isolated bright samples. Upsample with a 3x3 tent (dense separable tent above
   radius 2), combining levels with
   normalized weights so intensity does not grow with the number of levels.
6. Combine scene and bloom. Preserve linear midtones up to 0.6, then apply a smooth
   exponential shoulder to the brightest channel and scale RGB together. This
   retains colored highlights, including red brake lamps.
7. Draw world labels, editor controls, selection outlines and gizmos over the
   tone-mapped scene. Retain scene depth/stencil for occluded overlays; foreground
   gizmos get their existing fresh depth buffer. These passes keep MSAA.
8. Resolve and encode to sRGB once. Draw the SDL HUD, minimap, speedometer, debug
   text, road-name overlays, and editor panels before presenting.

`WorldRenderer::BeginUnlit()` marks the start of world-space overlays; lit-world
emission must be submitted before this boundary. `BeginForeground()` additionally
separates depth-independent handles. Resident geometry tracks the boundary by
submission index, since multiple resident draws may have the same dynamic offset.

The MSAA HDR color attachment is reused for the tone-mapped scene and overlays.
Bloom intermediates are single-sampled, clamped at their edges, rebuilt on resize,
and omitted entirely when bloom is disabled or its intensity is zero. There is
no readback or CPU filtering in the frame loop.

Lit mesh previews use the same pipeline. Vertex RGB/alpha inspection skips tone
mapping and bloom to preserve the inspected values. Wireframe omits bloom, and
plan view and the CPU renderer retain their existing rendering paths. An HDR
monitor is not required: output is still SDR.

## Code

- `shared/world_renderer_post.cpp`: targets, filters, composition and tone mapping.
- `shaders/bloom.hlsl`: linear filtering, soft threshold, highlight shoulder and
  final display encoding.
- `shared/world_renderer_gpu.cpp`: HDR, water, MSAA and overlay pass ordering.
- `shared/sky.cpp`, `shared/sun.cpp`: HDR sky capture and sun-disc emission.
- `shared/car.cpp`: visible headlight/brake-lamp radiance, separate from the
  spotlight intensities that illuminate the environment.

Profile in Release mode at the intended resolution. HDR targets increase render
memory/bandwidth, especially with 4x MSAA; bloom uses progressively smaller images.
The half-resolution bright-pass needs particular attention with moving distant
lights, reflective water and foliage highlights.

## Validation

Release builds of the game and editor were compiled for Windows (MinGW) and Linux;
all changed shaders compiled to both DXIL and SPIR-V. A Vulkan software-GPU render
harness exercised bloom on/off, supported MSAA modes and fallback, sky/day/night,
water, resident meshes, empty scenes, viewport offsets and odd-sized resizes. Pixel
checks confirmed bloom outside bright geometry, unchanged HUD/gizmo colors, retained
overlay depth, and unmodified raw vertex-color values. Config parsing and migration
checks covered defaults, ranges, invalid values, CRLF/BOM and preserved preferences.
These checks do not measure Direct3D 12 performance on the target Windows GPU.

The extended settings and bright-star update were also checked in Release with
DXIL/SPIR-V compilation, config boundary/invalid-value checks, and Vulkan renders
at radii 1, 4 and 8. Bloom-on/off image comparisons confirmed an extended bright-star
halo while HUD pixels stayed unchanged; larger radii were inspected for repeated
images of the star. Both MSAA and non-MSAA paths were exercised across resizes.
