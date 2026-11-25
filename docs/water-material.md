# Water materials

`Water` is a surface material for lakes, rivers, and other mostly horizontal bodies of water viewed from above. It is available in the material editor in both the game asset pipeline and the editor renderer. The built-in `world/M_LakeWater.bmat` now uses this type, so existing lakes and rivers acquire the new appearance without replacing their geometry.

## Using it

1. Open or create a `.bmat` in the texture browser and select **Water** as its type.
2. Choose **Water color** and adjust the sliders below it. No textures are required: the shader generates animated ripples. An optional **Normal** map adds small surface detail and uses the mesh's UVs.
3. Inspect the animated result in the world or mesh editor. The material editor shows a static shallow basin with a checker floor to make clarity, distortion, and shore blending visible; this is an illustrative preview, not the full scene-reflection renderer.
4. For the game, rebuild the Release application/assets and cook the world again. Materials are staged by the asset build, while the world cook fingerprints their contents. Repeat both steps after editing a material used by the cooked world.

Water is single sided with the front face above the surface. It does not require a translucency texture or an alpha cutoff. Existing material types and their stored indices are unchanged.

## Controls

All distances use engine meters; colors in the file and picker are sRGB.

| Control | Default | Range | Effect |
| --- | ---: | ---: | --- |
| Water color | RGB 0.08, 0.26, 0.22 | 0–1 per channel | Color approached as the bottom becomes obscured. Lighting still affects it. |
| Roughness | 0.16 | 0.02–1 | Broadens light highlights and gradually reduces sharp scene reflections. |
| Ripple strength | 0.18 | 0–1 | Surface normal variation; zero produces smooth water. |
| Wave scale (m) | 3 | 0.1–30 | Wavelength of the largest procedural ripple; smaller waves are layered over it. |
| Wave speed (m/s) | 0.6 | 0–5 | Animation speed; zero freezes the waves. |
| Clarity depth (m) | 4 | 0.1–50 | Approximate half-transmission distance along the viewing ray. Lower values make murkier water. Red is absorbed faster than blue. |
| Refraction (m) | 0.15 | 0–1 | Normal-driven distortion of the submerged scene. Zero removes distortion. |
| Shore blend (m) | 0.35 | 0.01–3 | Vertical water depth over which water blends into the bank or bed. |
| Shore foam | 0.2 | 0–1 | Strength of a narrow broken band near shallow edges. Zero is appropriate for still ponds. |

For a muddy river, start with a brown-green color and decrease clarity. For a calm lake, reduce ripple strength, speed, and foam. These are artistic controls rather than a fluid simulation.

The serialized properties live in a `water` object with keys `color`, `roughness`, `waveStrength`, `waveScale`, `waveSpeed`, `clarity`, `refraction`, `shoreWidth`, and `foam`. Missing values use the defaults above. Invalid values are rejected without overwriting the material. Unrelated JSON fields and previously assigned textures survive edits and changes of material type.

## Rendering

The GPU renderer draws opaque geometry first and captures its resolved color. Water then uses this separate snapshot and a separate opaque depth texture, avoiding feedback from the color/depth attachments it is writing. Ordinary unlit translucent geometry and editor overlays follow water. MSAA color and selection stencil are preserved between the opaque and water passes, including when a foreground gizmo is present.

The surface combines:

- Four moving sine waves applied to surface normals, plus an optional scrolling normal map. Wave phases are anchored in world space and reduced in double precision on the CPU so large geographic coordinates and moving cameras do not make ripples jump. Derivative filtering suppresses distant ripple aliasing.
- Air/water Fresnel reflectance, with approximately 2% reflection head-on and stronger reflection toward grazing angles.
- A bounded screen-space reflection trace against opaque scene depth, with the current day/night sky palette as fallback. Sun and moon highlights and the existing tiled vehicle lights illuminate the surface.
- Depth-based transmission and colored absorption, normal-driven refraction, and rejection of foreground samples so distortion does not drag banks or vehicles into the water.
- Shallow-edge blending and restrained contact foam. Directional shadows affect the water's lighting; water itself does not cast opaque shadows or block mesh AO baking.

The depth capture shares the SSAO prepass. When water is visible it covers the full visible scene, even with SSAO disabled; disabling SSAO still disables its occlusion and blur work. The reflection trace is capped at 24 steps plus four refinements. There is one full-viewport color snapshot and no second reflected scene render. Views without visible water skip the water pass and capture. Water remains an opaque tinted fallback in the software rasterizer.

## Current limits

This version keeps the existing water mesh flat. Ripples alter lighting and refraction, not the shoreline silhouette, collision, or buoyancy. It does not add underwater rendering, caustics, wakes, waterfalls, or directional river flow.

Screen-space reflections can only reproduce opaque objects present in the camera's color/depth capture. Off-screen objects, the hidden backs of objects, other water surfaces, and translucent objects cannot be reflected accurately. Reflections can fade or change as objects leave the screen. The fallback reflects the sky palette, not sky sprites or clouds. Roughness fades scene reflections toward the sky rather than providing a fully blurred reflection pyramid.

Shore blending needs a bank or bottom intersecting the water mesh. It cannot invent a missing lake bed or repair mismatched terrain geometry. The color snapshot already contains atmospheric fog, so refraction/reflection plus surface fog is an approximation over long distances.

## Technique references

- NVIDIA, [Effective Water Simulation from Physical Models](https://developer.nvidia.com/gpugems/gpugems/part-i-natural-effects/chapter-1-effective-water-simulation-physical-models): layered waves and detailed surface normals.
- Alex Tardif, [Water Rendering](https://alextardif.com/Water.html): separate scene captures, screen-space reflection, refraction, depth softening, and foam.
- Epic Games, [Single Layer Water Shading Model](https://dev.epicgames.com/documentation/en-us/unreal-engine/single-layer-water-shading-model-in-unreal-engine): a dedicated water pass using scene depth/color and absorption/scattering.
- NVIDIA, [Generic Refraction Simulation](https://developer.nvidia.com/gpugems/gpugems2/part-ii-shading-lighting-and-shadows/chapter-19-generic-refraction-simulation): screen-space scene distortion and foreground leakage considerations.
