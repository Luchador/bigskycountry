# Editor geometry cache

The editor automatically stores generated terrain fills, lake beds, river beds,
and water surfaces in `editor-cache/` inside the CMake build directory (normally
`build/editor-cache/`). The first load generates this data; subsequent loads
reuse it. Roads and intersections still rebuild normally, including their exact
terrain boundary footprints. Imported meshes use the existing asset loader.

The cache stores full-precision mesh positions/normals and the exact shoreline
samples used by terrain seams and control-point picking. World files, material
assignments and editing data remain authoritative and unchanged. The game and
world cooker do not use or require the editor cache. Recooking is not required.

Each entry is keyed by its geometry inputs. Terrain keys include road and
intersection footprints, holes, crossing slabs, bridge ground contacts/walls,
anchor links, railroad junction/switch seams, nearby water shorelines, pocket
selection mode and supporting-boundary roles. Water keys include all control
points/handles and depth. Revision counters and pointers are not persistent keys.
Normal in-session revision guards still prevent idle rebuilding. Draft water
edits remain lightweight; geometry is cached only after committing a valid edit.

CMake fingerprints the geometry builders, relevant shared types and CDT headers.
Changing them automatically invalidates old entries on the next build. Entries
also have a format version, bounded lengths and a payload checksum. Invalid or
missing entries are regenerated; disk errors do not prevent opening/editing.
Files are installed by renaming complete temporary files.

The cache has a 1 GiB disk budget, checked on startup and every 64 MiB of writes;
the least recently used entries are discarded first. It does not retain extra
mesh copies in memory. Close the editor and delete `build/editor-cache/` whenever
you want to force a clean rebuild or reclaim space. The next load repopulates it.
Startup logs report the number of reused, regenerated and stored entries.
