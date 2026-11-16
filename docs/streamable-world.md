# Cooked worlds — milestone 2

Build normally, then use **File → Cook World** in the editor. Cooking saves the
world and writes `build/cooked/worlds/<world filename>.bscworld`. It is an explicit,
synchronous operation; ordinary saves do not recook the world. Finish road and
lake drafts first. The completion dialog gives the exact output path.

The game tries this package on its next launch. Its source-world fingerprint and
model/material fingerprints must match. Missing, stale, unsupported, or damaged
packages fall back to authored geometry. Recook after world, model, or material
changes. Changing texture pixels alone does not require a geometry recook: the
normal asset build updates the shared cooked textures.

The editor continues to render its live authored geometry. In this milestone the
game still loads the authored world for gameplay/collision and eagerly loads the
cooked CPU payloads. It does **not** yet unload distant entities or reduce startup
world-generation work. Independent cell reads, identities, and metadata are the
foundation for asynchronous streaming in milestone 3.

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

## Package version 1

All integers are explicitly little-endian; no compiler struct layouts are stored.
Floats use IEEE-754 bit patterns. Strings are length-prefixed UTF-8. Counts are
uint32; offsets, byte sizes, fingerprints, and stable ID values are uint64.

| Record | Contents |
| --- | --- |
| Header (28 bytes) | `BSCCHNK\0`, uint32 version, uint64 manifest size, uint64 manifest checksum |
| Manifest | Source hash; 512 m cell size; 2,000 m initial LOD switch; entity table; asset table; cell directory |
| Entity | Typed ID, folder ID, name, optional shared model/material paths, affine placement, world bounds, direct dependency IDs |
| Asset | Model/material kind, logical path, content hash; material texture paths indexed by `MaterialSlot` |
| Cell directory | Signed int32 X/Z address, 3D bounds, payload-relative offset, length, checksum, near/far triangle counts, owner IDs |
| Cell payload | Geometry batches and compact railroad tie runs |
| Geometry batch | Owner ID, layer, optional material path, near mesh, distant mesh |
| Mesh | Vertex/index counts; vertices; uint32 triangle indices |
| Vertex (60 bytes) | Cell-relative double position, float normal, float UV, float linear RGBA |
| Tie run | Owner ID, first/last affine transforms, count; uses the shared built-in tie model |

Fingerprints use FNV-1a 64-bit for accidental corruption/staleness detection, not
authentication. Reader limits manifest size to 128 MiB and individual cells to
512 MiB, validates sizes/offsets/counts/references, and rejects unsupported versions.
`Reader::Open` reads only the directory; `ReadCell` seeks directly to one payload.
Cooking spools payload bytes and publishes a completed archive by atomic rename.
The last successful package survives a failed cook.

Generated roads, intersections, terrain, lake beds/water, embankments, rails, and
bridge structures are clipped to 512 m XZ cells. Normals, UVs, and vertex colors
are interpolated at cuts. Models remain shared asset references and placements,
not duplicated triangles. Their entity bounds allow future cell registration
without repeating oversized models in every crossed cell. Dynamic pawns and
marker lettering remain live; this is a render package, not a replacement for
all authored/gameplay state.

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
fade transitions can follow with residency management.

Bump the package version when serialization or geometry recipes change in ways
that invalidate existing payloads.

## Tests

Configure with `-DBIGSKY_BUILD_TESTS=ON`, then build `BigSkyWorldPackageTests` and
`BigSkyVisibilityTests` and run `ctest --test-dir build --output-on-failure`.
Tests cover identity persistence/deletion/duplication, clipping across positive
and negative cells, UV interpolation, preserved LOD rims/area, deterministic
cooking, lake/rail payloads, stale/corrupt/truncated packages, and failed cooking.
