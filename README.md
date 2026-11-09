My project Big Sky Country

## World saves

The editor and game open `world.wld` in the SDL per-user data directory by default. On Windows the path is `%APPDATA%\BigSkyCountry\BigSkyCountry\world.wld`. To work on a different world, pass its path as the first argument to both executables.

```powershell
.\BigSkyCountryEditor.exe .\montana.wld
.\BigSkyCountry.exe .\montana.wld
```

A missing world starts empty. The editor writes it on the first save. Finishing a road, editing a committed road, and undoing or redoing a road edit all save the world. File > Save (Ctrl+S) also writes it. The old `.roads` format is retired.

### WLD version 1

All framing integers are unsigned and little-endian. The file begins with the four ASCII bytes `BSCW`, a 32-bit format version, and a 32-bit section count. Each section has a four-byte tag, a 32-bit section version, a 64-bit payload length in bytes, and exactly that many payload bytes. Sections are independent of their physical order.

`ROAD` version 2 stores roads in draw order. Its payload is whitespace-separated text in the classic locale. Each road starts with lane count, curved flag, left and right shoulder flags, three-lane layout, divided flag, divided gap, and edge treatment (0 for none, 1 for a transition strip, 2 for a curb). Version 1 roads load with no edge treatment. Next comes the point count, then one record per point with X, Y, Z, a manual-heading flag, and a heading in radians. Flags are 0 or 1. The reader rebuilds the meshes from these records. An absent or empty `ROAD` section means the world has no roads.

To add terrain or objects, give the new data its own tag and section version. The current editor preserves unrecognized sections byte-for-byte when saving road edits. It rejects unsupported versions of the world container or the known `ROAD` section.

### Static mesh assets

Put a self-contained glTF 2.0 binary under `assets/models/`. Both executables load models relative to that directory. The importer reads the active scene, applies node transforms, and converts glTF coordinates to the engine's Y-up world. It supports indexed and unindexed triangle lists, strips and fans, including interleaved vertex data. It retains normals and the first UV set. The current opaque pass draws material base color multiplied by vertex color. Texture sampling, animation, skinning, sparse accessors, and compressed primitives are outside this import path.

A `MESH` version 1 section in a world file stores one instance per record. Write the quoted model path, world position (X Y Z), then the three basis vectors (local X, Y and Z in world coordinates). The following payload places a model at (10, 2, 30) with its authored size and orientation.

```text
"props/barn.glb" 10 2 30   1 0 0   0 1 0   0 0 1
```

In C++, load the asset with `world.models.Load("props/barn.glb")`. On success, append a `MeshInstance` with that mesh and its transform to `world.meshes`. Instances share the imported geometry and save by asset path. In the 3D editor, `editor/arrow.glb` marks the selected road point (or the first point of a selected road) with red X, green Y and blue Z arrows. These markers are editor-only.
