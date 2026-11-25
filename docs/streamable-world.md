# Cooked worlds — milestone 3

Build normally, then use **File → Cook World** in the editor. Cooking saves the
world and writes `build/cooked/worlds/<world filename>.bscworld`. It is an explicit,
synchronous operation; ordinary saves do not recook the world. Finish road and
lake drafts first. The completion dialog gives the exact output path.

The game tries this package on its next launch. Its source-world fingerprint and
model/material fingerprints must match. Missing, stale, or unsupported packages
fall back to the authored world. A valid package opens its manifest without
rebuilding the authored world or loading all cell geometry. Payload corruption
found later is reported by the streamer; missing collision blocks physics instead
of allowing a pawn to fall through the world.

**Recook once after updating to milestone 3.** Version 2 stores separate near,
distant, and collision payloads. Version 1 packages cannot be streamed. Authored
`.wld` files remain compatible. Recook after world, model, or material changes;
texture pixel changes only need the usual texture build.

Traffic milestone 1 adds format **v5** with directed lane metadata in the
manifest. Versions 2–4 remain readable, but need recooking to use the test AI
vehicle. See [traffic setup and limitations](traffic.md). The test AI requests
collision along its lane and waits independently if its support is unavailable;
it does not add a global loading pause for the player.

## Loading and unloading

A CPU worker reads cells and shared GLB/FBX models (including authored LODs) and builds collision BVHs. The main
thread publishes completed data and owns all GPU operations. Each streamer has
one in-flight request/result slot: camera teleports cannot accumulate an
unbounded queue. Completions outside the new desired set are discarded. Shutdown
joins the worker before releasing the catalog; an already running read finishes
before shutdown completes. A separate per-renderer worker loads cooked DDS maps;
shared maps appear using neutral fallbacks until ready.

Render cells and model placements are requested inside the view distance plus a
512 m preload ring, and unloaded outside the view distance plus 768 m. Near/far
LOD has 128 m hysteresis. A cell's conservative bounds can extend beyond its grid
address (for example, a tie run); visibility and collision use the bounds.
Newly visible static meshes are packed incrementally, with an 8,192-triangle and
approximately 2 ms preparation budget per frame (checked every 256 triangles).
Railroad ties share that preparation budget; editor road batches check it between
batches. Resident geometry uploads are capped at 4 MiB and eight buffers per
frame, including oversized meshes, which resume at their last byte offset. A
buffer is drawn only after its final upload has been submitted. Already resident
geometry continues rendering while new buffers are prepared. Dynamic geometry,
such as moving pawns, is independent of this resident-upload budget.
One completed texture is published per frame. GPU allocation/upload and some
instance setup still occur on the main thread; this is not texture mip streaming
or a guarantee of zero frame spikes.

Collision is independent of visual LOD. Full-detail solid surfaces are cooked
without paint or water, and chunk BVHs are attached/detached without rebuilding a
world-wide BVH. Model collision uses the same shared decoded mesh as rendering.
Collision is prefetched within 256 m of **every live pawn**, including parked
vehicles. Velocity prediction covers the next half-second of travel. Physics
runs only when collision within 64 m of every predicted sample is resident.
When data is missing, simulation waits and drops catch-up time; input/UI and the
debug camera keep working. Startup uses cooked spawn candidates and waits for
safe collision before placing the player and test car. F1 camera travel does not
unload parked-pawn collision. A failed required payload displays a recook/restart
message; it is not retried every frame.

Dynamic gameplay pawns remain live. Static authored placements stream as shared
mesh instances; marker text, tint, pose, scale and mesh material are preserved.
The manifest also retains object type, health, invincibility and vehicle
acceleration for future gameplay-state integration. There is no persistent
simulation-state eviction for dynamic actors in this milestone.

## Budgets and editor protection

**Settings → Project Settings** exposes three project budgets (64–16,384 MiB):

| Control | Default | Accounting |
| --- | ---: | --- |
| Geometry per CPU/GPU cache | 512 MiB | Separate limits for decoded cell geometry and retained renderer buffers/staging |
| Shared models + textures | 512 MiB | Half for unique decoded model meshes, half for shared material maps per renderer |
| Streamed collision | 256 MiB | Chunk/instance triangle and BVH allocations |

`config.ini` beside the game/editor executable can override these on this machine:

```ini
[Streaming]
GeometryMiB=0
AssetsMiB=0
CollisionMiB=0
```

Each key accepts `0` to inherit its project budget, or an integer from 64 to
16,384 MiB. Missing/invalid entries also inherit; invalid entries log a warning.
The build adds missing keys with `0` so existing project budgets keep working.
Project Settings continues to display and save the project defaults; local
overrides are applied to streaming and renderer/texture caches, including editor
previews. Restart to apply INI changes; no world recook is necessary.

These are working-set targets, not a cap on total process memory. The manifest,
indexes, allocator overhead, dynamic geometry, render targets, sky textures,
fonts, and bounded in-flight decoding/upload data are additional memory. Model
meshes and material maps are charged once regardless of reference count. The
renderer can still hold a separate GPU buffer for each model placement.

Under pressure, farthest unprotected CPU geometry/models and unused render
resources are evicted first. Cells within 128 m, pawn collision dependencies,
and currently active GPU geometry/maps stay pinned and may exceed a target.
Known payload/model sizes are retained for admission checks so over-budget
content does not reload every frame. Unused GPU buffers and textures stay warm
while there is room in their budgets, instead of expiring after 120 unseen frames.
Unloading a streamed cell or placement still explicitly releases its buffers;
deleted mesh owners are also reclaimed. An in-flight texture read is allowed to
finish even if the camera turns away, avoiding repeated reads of the same map.
Lower view distance if the active set exceeds the GPU budget. **F3** shows cell,
model, collision, cache-memory and error counters, plus triangles prepared and
resident geometry bytes uploaded this frame;
pressure is reported rather than silently removing required support surfaces.

The **editor keeps the full authored document, selected objects, open spline
edits, connectivity and undo/redo history resident**. It renders live data and
never swaps in potentially stale cooked chunks. Budgeting reclaims only derived
GPU/material caches and model-cache entries with no owners. Every object remains
editable and selectable from the outliner, including distant/hidden objects.
Runtime path-only documents refuse to save over source worlds. Asynchronous
unloading of authored editor documents is intentionally not part of this step.

## Command line

The normal game/editor build also builds the console tool:

```bat
build\BigSkyWorldCooker.exe
build\BigSkyWorldCooker.exe "C:\path\world.wld" "build\cooked\worlds\world.wld.bscworld"
build\BigSkyWorldCooker.exe --inspect "build\cooked\worlds\world.wld.bscworld"
```

No arguments uses the same default saved world as the game/editor. Cooking
upgrades legacy identity data in the source `.wld`; cooked output cannot overwrite
that source. `--inspect` verifies every cell checksum and reports entity, asset,
cell, and triangle counts. An output filename with a different basename/location
is useful for inspection, but the game automatically looks only at the default
path above.

## Persistent identities

`EntityId` is `(SceneKind, uint64 value)`, scoped to one world. Roads and
intersections retain their existing positive IDs and connectivity references.
Objects, terrain (including lakes), and raw model placements gain persistent IDs.
Scene/outliner vector indices remain transient UI handles.

The new optional `EIDS` version-1 section in `.wld` records road/intersection/
placement allocation high-water marks, followed by placement IDs in MESH, TERR,
and OBJS record order. Deleted IDs are not reused after restarting. Reordering,
undo, and redo preserve identities; new duplicates allocate new identities.
Legacy worlds receive IDs when opened and persist them on save. Continue editing
upgraded worlds with this editor or newer: older editors do not maintain EIDS.

## Package version 2

The current writer uses version 9, adding the subsurface texture dependency to
each asset record. Readers still accept versions 2–8, which have nine texture
slots; the new slot defaults to empty. Recook after changing foliage materials.

All integers are explicitly little-endian; no compiler struct layouts are stored.
Floats use IEEE-754 bit patterns. Strings are length-prefixed UTF-8. Counts are
uint32; offsets, byte sizes, fingerprints, and stable ID values are uint64.

| Record | Contents |
| --- | --- |
| Header (28 bytes) | `BSCCHNK\0`, uint32 version, uint64 manifest size, uint64 manifest checksum |
| Manifest | Source hash; 512 m cell size; 2,000 m initial LOD switch; spawn candidates; entity table; asset table; cell directory |
| Entity | Typed ID, folder ID, name, optional shared model/material paths, affine placement, world bounds, object type/pose/properties, marker text/color, direct dependency IDs |
| Asset | Model/material kind, logical path, content hash; material texture paths indexed by `MaterialSlot` |
| Cell directory | Signed int32 X/Z address, 3D bounds, three payload-relative offset/length/checksum records, near/far triangle counts, owner IDs |
| Near/distant payload | Geometry batches; near payload also has compact railroad tie runs |
| Collision payload | Position/index counts; cell-relative double positions; uint32 solid triangle indices |
| Geometry batch | Owner ID, layer, optional material path, one mesh for the requested LOD |
| Mesh | Vertex/index counts; vertices; uint32 triangle indices |
| Vertex (60 bytes) | Cell-relative double position, float normal, float UV, float linear RGBA |
| Tie run | Owner ID, first/last affine transforms, count; uses the shared built-in tie model |

Fingerprints use FNV-1a 64-bit for accidental corruption/staleness detection, not
authentication. Reader limits manifest size to 128 MiB and individual cells to
512 MiB, validates sizes/offsets/counts/references, and rejects unsupported versions.
`Reader::Open` reads only the directory; `ReadRender` and `ReadCollision` seek to
one independent payload. `ReadCell` is an inspection convenience that reads both
render LODs. The inspect tool validates all three payloads.
Cooking spools payload bytes and publishes a completed archive by atomic rename.
The last successful package survives a failed cook.

Generated roads, intersections, terrain, lake beds/water, embankments, rails, and
bridge structures are clipped to 512 m XZ cells. Normals, UVs, and vertex colors
are interpolated at cuts. Models remain shared asset references and placements,
not duplicated triangles. Their entity bounds are spatially indexed without repeating oversized models
in every crossed cell. Dynamic pawns and marker lettering remain live.

Dependencies cover intersection attachments, ramp stations, endpoint connectors,
road/rail crossings, overpass neighbors, and road-bounded terrain. They are direct,
bidirectional metadata edges; cycles are expected. They must not be interpreted
as a requirement to recursively load the entire connected road network.

## Initial distant geometry

Each surface batch uses meshoptimizer's index-only simplification with a 25%
triangle target and 0.20 m absolute error tolerance. Topological borders, holes,
material/attribute seams, and cell-plane vertices are protected. Simplification
may stop early; preserving a sealed boundary takes priority over the target.
Original retained positions are never moved. There are no skirts masking gaps.

The game chooses the distant surface at 2 km from the nearest point of a cell's
bounds. It omits distant paint. Rails/ties retain the independent Project Settings
detail-distance control; embankments remain visible. Shared imported models do
not yet have generated LODs. The existing view-distance and fog controls continue
to apply. This first LOD switch is fixed in the recipe; screen-space tuning and
fade transitions can follow in a later milestone.

Bump the package version when serialization or geometry recipes change in ways
that invalidate existing payloads.

## Manual validation

These scheduling/cache changes work with existing version-2 packages; they do not
require recooking. For a camera-turn check, press F3 and turn away and back after
several seconds. Below the cache budgets, previously seen geometry should return
with zero preparation/uploads. First visits spread that work over several frames.

For a manual check: recook, launch the game, press F3, then use F1 to fly several
kilometers away and back. Resident render counts should move with the camera;
collision near the parked car/player should remain. In the editor, change a
road/lake, travel out of range, return, and undo/redo: authored edits must remain.
