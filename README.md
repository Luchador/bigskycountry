My project Big Sky Country

## Material textures

New roads and roads loaded from older worlds use `surfaces/roads/M_Asphalt01.bmat`. Its base color, normal, and ORM textures belong in `assets/textures/surfaces/roads/`. To change one road, drag a material from the Content Browser’s **Textures** tab onto its **Pavement material** thumbnail or directly onto the road in the 3D viewport. The game and editor wrap it along the road spline. U repeats once across each travel lane. V starts at the first control point and repeats every lane-width (5 m) of 3D centerline length, including elevation changes. The pattern continues onto the shoulders. Markings, curbs, transition strips, and medians use their own colors.

The build recursively converts TIFF, PNG, TGA, JPEG, and BMP images under `assets/textures` into DDS files with a full mip chain. Filename matching ignores case and reads the suffix immediately before the extension.

| Filename stem | DDS format | Channels |
| --- | --- | --- |
| Ends in `_basecolor` or `_emissive` | BC7 sRGB | RGB and alpha |
| Ends in `_normal` | BC5 linear | Red and green |
| Ends in `_orm` | BC7 linear | Occlusion (R), roughness (G), metalness (B) |
| Any other name | BC4 linear | Red |

Keep editable images and `.bmat` files in the project's `assets/textures/` and GLBs in `assets/models/`. Click **Refresh** in the Content Browser after adding, changing, or removing a file. The editor reads these directories directly and can preview a new material immediately.

Build either executable, or run `cmake --build build --target BigSkyAssets` for an asset-only update. The game reads `build/cooked/`. It contains runtime-ready GLBs, material definitions, and DDS textures. For example, `road.png` used as a base color cooks to `cooked/textures/BC7_UNORM_SRGB/road.png.dds`. When a material slot requires a different format from the filename default, the build cooks that format once and shares it across materials. The sun and moon use BC7 sRGB to preserve their colors and alpha. A packaged game needs the `cooked/` directory beside its executable.

**Upgrading an existing build.** Close the old editor and build in the existing build directory once. CMake recovers build-only textures, materials, and models into the project, and removes the old `build/assets/`. If both copies of a file differ, the project copy stays active and CMake reports the build copy's location under `assets-backup/` in the build directory. A successful cook also removes the old `build/texture-cache/` when all its files are accounted for.

On Windows, CMake fetches a pinned DirectXTex `texconv` automatically. An existing host executable can be supplied with `-DBIGSKY_TEXCONV=...`. On Linux, install the libtiff development package and run `python3 -m pip install -r tools/texture-requirements.txt` in the Python environment used by CMake. When cross-compiling material maps that need SDL_image decoding, set `BIGSKY_HOST_IMAGE_DECODER` to a native build of `BigSkyDecodeImage` or configure a target emulator.

## World saves

The editor and game open `world.wld` in the SDL per-user data directory by default. On Windows the path is `%APPDATA%\BigSkyCountry\BigSkyCountry\world.wld`. To work on a different world, pass its path as the first argument to both executables.

```powershell
.\BigSkyCountryEditor.exe .\montana.wld
.\BigSkyCountry.exe .\montana.wld
```

A missing world starts empty. The editor writes it on the first save. Finishing a road, editing a committed road, and undoing or redoing a road edit all save the world. File > Save (Ctrl+S) also writes it. The old `.roads` format is retired.

### WLD version 1

All framing integers are unsigned and little-endian. The file begins with the four ASCII bytes `BSCW`, a 32-bit format version, and a 32-bit section count. Each section has a four-byte tag, a 32-bit section version, a 64-bit payload length in bytes, and exactly that many payload bytes. Sections are independent of their physical order.

`ROAD` version 9 stores roads in draw order. Its payload is whitespace-separated text in the classic locale. Each road starts with lane count, curved flag, left and right shoulder flags, three-lane layout, divided flag, divided gap, edge treatment (0 for none, 1 for a transition strip, 2 for a curb), median type (0 for concrete, 1 for grass), and the quoted material path relative to `assets/textures`. Versions 1–5 use the default asphalt material. Versions 1–4 default to a concrete median. Version 1 roads load with no edge treatment. Version 4 adds the whole-road XYZ rotation (radians) after the style. Version 8 adds a persistent road ID after the rotation. Earlier worlds acquire IDs on load. Next comes the point count, then each point’s world XYZ, manual-tangent flag, heading, pitch, and (since version 4) roll. Version 7 adds an intersection ID after each point. Zero denotes a free point. A positive ID is followed by an anchor index (0 for +X, 1 for +Z, 2 for −X, 3 for −Z in the junction’s local frame) and is valid only at a road end. Version 9 appends incoming and outgoing Bézier handle lengths to each point, expressed as fractions of their respective chord lengths. Earlier versions use one third for both. Roads from before version 7 load with free endpoints. Older roads start with zero whole-road rotation and point roll. Versions 1 and 2 load with zero pitch for manual tangents. Flags are 0 or 1. The reader rebuilds the meshes from these records. An absent or empty `ROAD` section means the world has no roads.

Terrain placements use the separate `TERR` section described below. The current editor preserves unrecognized sections byte-for-byte when saving road edits. It rejects unsupported versions of the container and known sections.

### Static mesh assets

Put a self-contained glTF 2.0 binary under `assets/models/`. Model references use paths relative to that directory. The editor reads the project GLB and the game reads its staged copy in `cooked/models/`. The importer reads the active scene, applies node transforms, and converts glTF coordinates to the engine's Y-up world. It supports indexed and unindexed triangle lists, strips and fans, including interleaved vertex data. It retains normals and the first UV set. The current opaque pass draws material base color multiplied by vertex color. Texture sampling, animation, skinning, sparse accessors, and compressed primitives are outside this import path.

A `MESH` version 1 section in a world file stores one instance per record. Write the quoted model path, world position (X Y Z), then the three basis vectors (local X, Y and Z in world coordinates). The following payload places a model at (10, 2, 30) with its authored size and orientation.

```text
"props/barn.glb" 10 2 30   1 0 0   0 1 0   0 0 1
```

In C++, load the asset with `world.models.Load("props/barn.glb")`. On success, append a `MeshInstance` with that mesh and its transform to `world.meshes`. Instances share the imported geometry and save by asset path.

### Placed objects

`BSCObject` holds a transform, a shared mesh, health, and invincibility. `Building`, `Vehicle`, `Prop`, `Character`, and `Marker` inherit those fields. Signs and trees can use `Prop`. NPCs and animals can use `Character`. Each object gets its own transform and gameplay state. Objects using the same GLB share its geometry.

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

The optional `OBJS` version 2 section stores one record per object. Each record contains the type, quoted GLB path, position XYZ, rotation XYZ, scale XYZ, health, and an invincibility flag (0 or 1). Type IDs are 0 for a generic object, 1 for a building, 2 for a vehicle, 3 for a prop, 4 for a character, and 5 for a marker. A marker record ends with its quoted UTF-8 label. Version 1 object sections also load. A vehicle record ends with its acceleration in meters per second squared. Existing `MESH` placements keep their original transform, including any shear.

### Place markers

Right-click in the 3D viewport and choose **Create → Marker**. The editor places `assets/models/objects/marker/marker.glb` at the picked position and selects it. In the inspector, enter a place name in **Marker text** and press **Enter**. An empty name hides the lettering. The Outliner shows the name and the world saves it with the marker. Placement, text edits, and deletion support Undo and Redo.

In the game and the editor, the lettering faces the active camera around the world Y axis. Its lower edge is 50 game meters above the transformed mesh. The default capital letter height is 100 game meters.

### Terrain chunks

Open the single-chunk example from the repository root with `.\build\BigSkyCountryEditor.exe examples\test-terrain.wld`.

The test grid at `assets/models/terrain/test_grid.glb` has 10 × 10 shared vertices on the XZ plane at 5 m spacing. Each placement has its own position and rotation in the Transform panel.

Drag a material from the Content Browser’s **Textures** tab onto a chunk in the 3D viewport. The assignment belongs to that placement, saves with the world, and supports **Ctrl+Z** and **Ctrl+Y**. The texture repeats every 4 m in the chunk’s local coordinates.

To fill a block, Shift-click the roads and intersections surrounding it and press **F**. You can also select the group in the Outliner and press **F** there. The new terrain follows the outer edge of the pavement, shoulder, transition strip, or curb and the rounded corners of generated intersections. At intersections with fewer than three occupied anchors, the boundary follows the attached anchors to the junction center. The interior has shared vertices at roughly 8 m spacing, with wider spacing on very large blocks. **View → Wireframe** shows the triangles. Move the surrounding roads to reshape the chunk, or select the chunk to assign a material or press **Delete**. If the perimeter opens, the chunk keeps its material and boundary links and becomes visible again when the loop closes.

The optional `TERR` version 3 section distinguishes imported meshes and generated fills. A `mesh` record stores the quoted model path, position XYZ, rotation XYZ in radians, and quoted material path. A `fill` record stores the quoted material path, a counted list of road IDs, and a counted list of intersection IDs. The editor and game regenerate the mesh from those neighbors when loading the world. Versions 1 and 2 still load as imported chunks.

### Moving roads and intersections together

Click a road surface or an intersection center, then **Shift-click** other roads or intersections to add them. **Ctrl-click** a group member to remove it. The Outliner shows the same selection. In the 3D viewport, use **W** to translate or **E** to rotate about the average of the selected road pivots and intersection centers. Groups of roads can rotate on all three axes. A group containing an intersection rotates about Y to keep each crossing level. In local space, the lowest-numbered selected intersection supplies the axes, or the lowest-numbered road if the group contains only roads. The Transform panel shows the pivot’s world position and the reference member’s orientation. Editing either moves or rotates the group. Hold **Ctrl** during a rotation to snap to 5°.

In the top-down view, use the same selection gestures on the road surfaces and intersection centers. Drag the group's blue center handle to move it, or its gold handle to rotate it. A moved intersection carries its connected road ends. Moving a selected road away from an unselected intersection releases that connection. **Esc** cancels the drag and **Ctrl+Z** undoes the whole edit.

### Intersection anchors

In the 3D viewport, right-click and choose **Create → Create Intersection**. The editor places a selected cross at the nearest road or terrain surface under the cursor, with the same ground-plane and camera-ray fallback as road creation. Pick the cross or its Outliner row to edit it again. Use **W** to move the center and **E** to rotate the junction around its vertical axis. The Transform fields also accept a location and a Y rotation. The intersection inspector lets you adjust the corner radius and assign a pavement material. The horizontal ring previews the radius. The optional `INTS` section in the world file stores these settings. Version 3 adds yaw in radians. Earlier versions load with zero yaw.

Each intersection has four permanent control-point markers, 10 m from its center along its local X and Z axes at the intersection elevation. In the 3D or top-down view, select a road and its first or last point, **Shift + left-click** a free anchor, then press **F**. Both markers turn amber before you connect. The editor adds one segment to that end of the road and saves the connection. **Esc** cancels the pending pairing. Each anchor accepts one road end. Moving the intersection carries its attached ends. Moving a road end away, extending past it, or deleting its intersection frees the connection. These edits support undo and redo.

To start a new road in either view, click a free anchor, then **Shift + left-drag** from that marker. The road previews at the intersection elevation using the current road defaults. Hold **Ctrl** during the drag to snap the heading in 5° increments from that anchor’s outward direction. Release Ctrl to aim freely again. Release the mouse button to attach the road and select its free endpoint, or press **Esc** to cancel.

Three occupied anchors generate a flat T-junction at the intersection elevation. A loop road can use two of those anchors. Connect its first and last points to separate anchors on the same intersection, then connect an approach road to a third anchor. Connecting the fourth side opens it into a four-way crossing. The surface uses the intersection’s pavement material and corner radius. It matches each approach’s shoulders, curbs and transition strips. Lane markings and divided-road medians end where the road meets the junction. Wide approaches and large corner radii can move that seam beyond the 10 m anchor. Disconnecting roads updates the surface immediately. With fewer than three occupied anchors, the four anchor markers remain available for editing.

Click an intersection anchor in the 3D or top-down view to edit its **Stop line** checkbox. Each anchor defaults to on and remembers its setting when a road is disconnected. On a generated junction the white bar runs across the incoming travel lanes, starting at the road paint and extending 0.60 m onto the intersection. When an attached road is selected, its endpoint takes priority at the shared marker. Select the junction surface or its Outliner row, then click the anchor to inspect it.

### Scene outliner

The **Outliner** above Transform lists the world’s roads, intersections, terrain chunks, placed objects, and static meshes. Click an entry to select it. Selecting a road or a control point in the viewport highlights its road in the list. Selecting a single road, object, or imported terrain chunk in the list makes its transform available for editing.

Use **New folder** to name a folder. Selecting a folder first creates a subfolder. **Rename** or **F2** changes a folder name. **Ctrl-click** toggles individual entries and **Shift-click** selects a range. Drag the selection onto a folder, or drop it on the world row to move it to the root. You can also move a folder with its descendants. Multi-selection is for organizing entries.

With a folder selected, **Delete** moves its contents to its parent and removes the folder. Folder edits save immediately and support **Ctrl+Z** and **Ctrl+Y**. The optional `OUTL` section in each `.wld` stores its hierarchy and folder assignments. Worlds without that section open with all entries at the root.

### Transform panel

The Transform panel sits above a selection-sensitive properties inspector. Roads and control points show the road settings. Terrain chunks show their mesh and material slot, and objects show their mesh and Invincible setting. Click the pavement to select a whole road, a visible control-point marker to edit just that point, or a placed object or terrain chunk to edit its transform. With **Use Geographic Coordinates** unchecked, locations are world coordinates in meters, shown to two decimal places. Rotations are in degrees. Only objects have editable scale.

Check **Use Geographic Coordinates** to display the selection’s latitude and longitude. Paste a Google Maps pair such as `35.0851266266639, -106.65282545913769` into **Latitude, longitude** and press **Enter**. Both horizontal axes update in one undo step. The separate elevation field uses game meters.

The world origin is **40° N, 98.5° W**. The editor projects WGS 84 coordinates using ellipsoidal Mercator with true scale at 40° N, subtracts the projected origin, then divides east and north distances by six. +X points east and +Z points north. This keeps constant-latitude lines horizontal and preserves local angles. Distance scale varies with latitude. The checkbox changes the coordinate display and input. World files continue to store game meters.

Type a value in one axis field and press **Enter** to apply it. **Esc** or leaving the field discards uncommitted text. Each change saves the world and supports **Ctrl+Z** and **Ctrl+Y**. The other axes keep their full precision.

With a whole road selected, press **W** for translation arrows or **E** for rotation rings. Drag a handle to move or rotate the pavement, markings, edges, and control points together. The first control point is the pivot for both the gizmo and the Transform panel. In local space the handles follow the road’s stored rotation.

For a control point, X is roll about the tangent, Y is yaw about world up, and Z is pitch above the horizontal. Point roll sets the orientation of its local editing axes. A straight point’s yaw and pitch pivot its neighboring spans. A Bézier point’s yaw and pitch set its tangent. When a segment is highlighted, the panel shows its owning road.

### Editor controls on Windows

The editor uses a dark charcoal theme. New panels and controls should use the UI color roles in `editor/editor_theme.hpp`. For a native Windows dialog, call `WindowsTheme::ApplyDialog` during `WM_INITDIALOG` to theme its standard controls. Viewport selection colors are defined in `shared/editor_colors.hpp`.

Drag a GLB model from the **Models** tab into the 3D viewport to place a **Prop**. The lowest part of the model sits at the height of the nearest road or terrain triangle under the cursor. On empty ground it sits at Y = 0. If the ray misses the ground, its pivot lands 20 m along the cursor ray. The new prop is selected. Use **W** to move it, **E** to rotate it, or **Delete** to remove it. Placement, transforms and deletion save to the world and support **Ctrl+Z** and **Ctrl+Y**.

In the Content Browser’s **Textures** tab, select a folder to browse its texture thumbnails and materials. Right-click an empty area and choose **Add New Material**. Enter a name to save a versioned JSON `.bmat` asset in that folder. Material creation and editing always write to the project’s `assets/textures` directory.

Double-click a material to open its floating **Material Editor**. Hold the left mouse button and drag in its preview viewport to orbit the sphere. Drag textures from any Content Browser folder onto the **Base color**, **Normal**, **ORM**, **Emissive**, or **Opacity** thumbnail. Use **Clear** to remove a map. The sphere updates and the material saves after each change. ORM uses red for occlusion, green for roughness, and blue for metalness. **Alpha cutout** discards pixels where base-color alpha multiplied by the opacity map’s red channel is below 0.5. **Standard** and **Terrain** both render the whole surface and use the same texture slots. Materials reference the original textures by their paths and share the same decoded images.

Choose **Unlit** for a surface whose color stays constant as the world lighting changes. Its **Emissive** map is multiplied by the **Emissive tint**, which defaults to white. The **Translucency** map’s red channel multiplied by the slider sets opacity. A black texel is invisible, and a white texel at the default slider value of **1.00** is opaque. Both slots use white when empty, so the tint and slider also work on their own.

The content browser uses a native tree of the GLB files in `assets/models`. Select a model to preview it, or click **Refresh** after changing the folder. The road inspector uses checkboxes for shoulders and drop-downs for alignment, three-lane layout, and edges. Its changes use the editor’s existing undo and save commands. The toolbar’s camera-speed slider stays in sync with the viewport mouse wheel.

For an even lane count, enable **Divided road** in the road inspector. The lane buttons then add or remove one lane in each direction. **Separation (m)** measures between the inside edges of the opposing travel lanes and changes in 0.5 m steps. At 2 m, the two 1 m paved inner shoulders meet. A wider separation adds a flush **Concrete** or **Grass** median between them. For example, a 6 m separation gives a 4 m median. Each carriageway has a solid yellow line along its inner lane edge.

Use **Tab** or **Shift+Tab** to move between the viewport and controls. In the 3D viewport, hold the **right mouse button** to look around and fly. **W/S** move forward and backward, **A/D** move left and right, and **Q/E** move down and up. Releasing the button stops keyboard movement immediately.

Choose **View → Wireframe** to see the visible triangle edges of roads, terrain chunks, and placed objects in the 3D viewport. Choose it again to restore shaded surfaces.

Right-click the 3D viewport and choose **Create → Create Road**. The first control point lands on the nearest road triangle under the original click, including elevated roads and curbs. On empty space it lands on the ground plane (Y = 0), or 20 m along the ray if the ground is out of reach. The new road starts with a level 10 m segment along world +X. Its far endpoint is selected for Shift-drag extension. Creation saves the world and supports **Ctrl+Z** and **Ctrl+Y**.

### Editing road control points

Click a road to show its control points, then click a point to select it. Press **W** for the translation arrows or **E** for the rotation rings. Drag a red (X), green (Y), or blue (Z) handle. The green arrow changes elevation. Rotating a Bézier point adjusts its tangent, while rotating a straight-road point pivots the neighboring spans around it. Release the mouse to save the edit, or press **Esc** during the drag to cancel. Each drag can be undone with **Ctrl+Z** and redone with **Ctrl+Y**.

In the 3D viewport, each cyan spline between two consecutive control points represents one road segment. Click the spline to select that segment in amber. **Shift+click** adds a segment to the selection, and **Ctrl+click** removes just the clicked segment. Press **Ctrl+Q** to split each selected segment halfway along its 3D spline. The new control points become selected and the entire subdivision can be undone in one step. Click a control point to resume point editing. In either view, **Shift+click** adds a control point and **Ctrl+click** removes it from the selection. The active point drives the transform inspector and gizmo.

With the viewport focused, **Delete** removes the selected road, segments, or control points. Deleting segments leaves a separate road for each continuous run of surviving segments. Deleting points reconnects the remaining points in order. A road is removed when it has fewer than two points left. Each deletion is saved immediately and can be undone or redone in one step.

The globe/cube button to the left of the camera speed slider toggles the 3D transform space. The editor starts in world space (globe). In local space (cube), a road point’s red X axis follows its tangent, its green Y axis is perpendicular to the grade, and its blue Z axis runs across the road. Translation arrows and rotation rings both use the chosen axes.

The grid button immediately left of the globe/cube toggles translation snapping. It starts off with **1 m** selected. With the viewport focused, **[** selects a smaller increment and **]** a larger one. The available increments are 1 cm, 10 cm, 1 m, 10 m, 100 m, and 1 km. The toolbar shows the current increment. A translation arrow measures signed steps from the drag's starting position along its world or local axis. This also applies when Shift-dragging a road endpoint to extend it. Placed objects and imported terrain chunks use the same gizmos. During a rotation ring or a top-down heading-handle drag, hold **Ctrl** to snap the change from the starting orientation to the nearest **5°**. Release Ctrl to resume free rotation. A small label beside the cursor shows the applied rotation in degrees, translation along a gizmo axis in meters, or the X and Z displacement for a top-down move. **Esc** cancels the drag. Typed Transform values keep their entered precision.

Use **,** to move sky time 30 minutes earlier and **.** to move it 30 minutes later.

The handles use three instances of `editor/arrow.glb` or `editor/rotation_gizmo.glb`.

In the top-down road view, hold the **middle mouse button** and drag to measure straight-line distance in the XZ plane. The ruler shows meters and disappears when you release the button. You can measure while laying out a draft.

To extend either end of a road, select its endpoint, hold **Shift**, and drag a translation arrow. In the top-down road view, hold **Shift** and drag the endpoint itself. The original point stays in place and the new endpoint stays selected, ready for the next segment. Top-down extensions keep the endpoint's elevation. Release to save, or press **Esc** during the drag to cancel.
