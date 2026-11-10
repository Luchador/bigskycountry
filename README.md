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

`ROAD` version 4 stores roads in draw order. Its payload is whitespace-separated text in the classic locale. Each road starts with lane count, curved flag, left and right shoulder flags, three-lane layout, divided flag, divided gap, and edge treatment (0 for none, 1 for a transition strip, 2 for a curb). Version 1 roads load with no edge treatment. Version 4 adds the whole-road XYZ rotation (radians) after the style. Next comes the point count, then each point’s world XYZ, manual-tangent flag, heading, pitch, and (in version 4) roll. Older roads start with zero whole-road rotation and point roll. Versions 1 and 2 load with zero pitch for manual tangents. Flags are 0 or 1. The reader rebuilds the meshes from these records. An absent or empty `ROAD` section means the world has no roads.

Future world data, such as terrain, can use its own section tag and version. The current editor preserves unrecognized sections byte-for-byte when saving road edits. It rejects unsupported versions of the container and known sections.

### Static mesh assets

Put a self-contained glTF 2.0 binary under `assets/models/`. Both executables load models relative to that directory. The importer reads the active scene, applies node transforms, and converts glTF coordinates to the engine's Y-up world. It supports indexed and unindexed triangle lists, strips and fans, including interleaved vertex data. It retains normals and the first UV set. The current opaque pass draws material base color multiplied by vertex color. Texture sampling, animation, skinning, sparse accessors, and compressed primitives are outside this import path.

A `MESH` version 1 section in a world file stores one instance per record. Write the quoted model path, world position (X Y Z), then the three basis vectors (local X, Y and Z in world coordinates). The following payload places a model at (10, 2, 30) with its authored size and orientation.

```text
"props/barn.glb" 10 2 30   1 0 0   0 1 0   0 0 1
```

In C++, load the asset with `world.models.Load("props/barn.glb")`. On success, append a `MeshInstance` with that mesh and its transform to `world.meshes`. Instances share the imported geometry and save by asset path.

### Placed objects

`BSCObject` holds a transform, a shared mesh, health, and invincibility. `Building`, `Vehicle`, `Prop`, and `Character` inherit those fields. Signs and trees can use `Prop`. NPCs and animals can use `Character`. Each object gets its own transform and gameplay state. Objects using the same GLB share its geometry.

Place an object from C++ using a model path relative to `assets/models`. This example places a barn from `assets/models/buildings/barn.glb`.

```cpp
ObjectTransform placement;
placement.position = {10.0, 2.0, 30.0};
placement.rotation.y = Math::DegreesToRadians(90.0);
BSCObject* barn = world.PlaceObject(ObjectType::Building, "buildings/barn.glb", placement);
if (!barn)
{
    SDL_Log("Cannot place barn: %s", SDL_GetError());
}
```

The world owns the returned object. Use `SetTransform` to edit its placement. A successful edit updates the matrix used for rendering and picking. Use `world.Save()` to persist the placement. Positions are in meters. Scale defaults to (1, 1, 1) and applies along the model axes. Rotation defaults to zero and applies about X, then Y, then Z in radians. Signed scale supports mirroring. Each game frame calls the object's virtual `Tick` with elapsed seconds.

The optional `OBJS` version 1 section stores one record per object. Each record contains the type, quoted GLB path, position XYZ, rotation XYZ, scale XYZ, health, and an invincibility flag (0 or 1). Type IDs are 0 for a generic object, 1 for a building, 2 for a vehicle, 3 for a prop, and 4 for a character. A vehicle record ends with its acceleration in meters per second squared. Existing `MESH` placements keep their original transform, including any shear.

### Terrain chunks

A terrain chunk is a placed mesh with a world position and rotation. Call `world.PlaceTerrain(modelPath, transform)` with a GLB path relative to `assets/models`. The optional `TERR` version 1 section stores the quoted path, position XYZ in meters, and rotation XYZ in radians for each chunk. Chunks use the mesh’s authored scale.

### Transform panel

The Transform panel sits above the road inspector. Click the pavement to select a whole road, a visible control-point marker to edit just that point, or a placed object or terrain chunk to edit its transform. The title identifies the selection. Locations are world coordinates in meters, shown to two decimal places. Rotations are in degrees. Only objects have editable scale.

Type a value in one axis field and press **Enter** to apply it. **Esc** or leaving the field discards uncommitted text. Each change saves the world and supports **Ctrl+Z** and **Ctrl+Y**. The other axes keep their full precision.

A whole road rotates about its first point. This moves its pavement, markings, edges, and control points together. For a control point, X is roll about the tangent, Y is yaw about world up, and Z is pitch above the horizontal. Point roll sets the orientation of its local editing axes. A straight point’s yaw and pitch pivot its neighboring spans. A Bézier point’s yaw and pitch set its tangent. When a segment is highlighted, the panel shows its owning road.

### Editor controls on Windows

The content browser uses a native tree of the GLB files in `assets/models`. Select a model to preview it, or click **Refresh** after changing the folder. The road inspector uses checkboxes for shoulders and drop-downs for alignment, three-lane layout, and edges. Its changes use the editor’s existing undo and save commands. The toolbar’s camera-speed slider stays in sync with the viewport mouse wheel.

Use **Tab** or **Shift+Tab** to move between the viewport and controls. In the 3D viewport, hold the **right mouse button** to look around and fly. **W/S** move forward and backward, **A/D** move left and right, and **Q/E** move down and up. Releasing the button stops keyboard movement immediately.

Right-click the 3D viewport and choose **Create → Create Road**. The first control point lands on the nearest road triangle under the original click, including elevated roads and curbs. On empty space it lands on the ground plane (Y = 0), or 20 m along the ray if the ground is out of reach. The new road starts with a level 10 m segment along world +X. Its far endpoint is selected for Shift-drag extension. Creation saves the world and supports **Ctrl+Z** and **Ctrl+Y**.

### Editing road control points

Click a road to show its control points, then click a point to select it. Press **W** for the translation arrows or **E** for the rotation rings. Drag a red (X), green (Y), or blue (Z) handle. The green arrow changes elevation. Rotating a Bézier point adjusts its tangent, while rotating a straight-road point pivots the neighboring spans around it. Release the mouse to save the edit, or press **Esc** during the drag to cancel. Each drag can be undone with **Ctrl+Z** and redone with **Ctrl+Y**.

In the 3D viewport, each cyan spline between two consecutive control points represents one road segment. Click the spline to select that segment in amber. **Shift+click** adds a segment to the selection, and **Ctrl+click** removes just the clicked segment. Click a control point to resume point editing. In either view, **Shift+click** adds a control point and **Ctrl+click** removes it from the selection. The active point drives the transform inspector and gizmo.

With the viewport focused, **Delete** removes the selected road, segments, or control points. Deleting segments leaves a separate road for each continuous run of surviving segments. Deleting points reconnects the remaining points in order. A road is removed when it has fewer than two points left. Each deletion is saved immediately and can be undone or redone in one step.

The globe/cube button to the left of the camera speed slider toggles the 3D transform space. The editor starts in world space (globe). In local space (cube), a road point’s red X axis follows its tangent, its green Y axis is perpendicular to the grade, and its blue Z axis runs across the road. Translation arrows and rotation rings both use the chosen axes.

The handles use three instances of `editor/arrow.glb` or `editor/rotation_gizmo.glb`.

To extend either end of a road, select its endpoint, hold **Shift**, and drag a translation arrow. In the top-down road view, hold **Shift** and drag the endpoint itself. The original point stays in place and the new endpoint stays selected, ready for the next segment. Top-down extensions keep the endpoint's elevation. Release to save, or press **Esc** during the drag to cancel.
