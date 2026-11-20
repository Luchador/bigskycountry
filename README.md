My project Big Sky Country

## Vehicle physics

The test car uses a rigid body with gravity, angular momentum, four raycast suspension contacts, and a rounded collision hull. Drive up a ramp to launch; the car carries its takeoff velocity and rotation into the air, then lands through the suspension and chassis contacts. Tire forces act only at grounded wheels. Roads, terrain, and world meshes provide static collision, and simulated cars exchange collision impulses.

Use **W/S** to accelerate or brake/reverse, **A/D** to steer, **Space** for the handbrake, and **F** to enter or exit a stopped car. Steering becomes less sensitive at speed, with strong grip and roll assistance for arcade handling. The handbrake loosens the rear tires for tighter turns. The chase camera stays level with the horizon and follows heading smoothly.

`CarHandling` in `shared/car.hpp` holds mass, center of mass, speed limits, braking, grip, suspension, steering, and roll assistance. `Vehicle::acceleration` controls drive acceleration. Distances are engine meters, speeds are meters per second, and angles are radians. Physics advances inside the existing 60 Hz game step, with smaller substeps for suspension stability and fast impacts. `shared/rigid_body.*` supplies reusable force, torque, integration, and collision-impulse code.

Walking or driving below the original spawn elevation does not reset the player. After an unsupported fall of more than 100 m with no landing surface below, recovery returns the controlled pawn to its most recent supported position and clears its velocity. Drivers stay in their cars. Streamed-world bounds protect landing areas whose collision is not loaded yet; after recovery, physics waits for collision at the return location. The game controls/status bar is at the top of the viewport, with **F3** diagnostics below it.

## Material textures

New roads and roads loaded from older worlds use `surfaces/roads/M_Asphalt01.bmat`. Its base color, normal, and ORM textures belong in `assets/textures/surfaces/roads/`. To change one road, drag a material from the Content Browser’s **Textures** tab onto its **Pavement material** thumbnail or directly onto the road in the 3D viewport. The game and editor wrap it along the road spline. U repeats once across each travel lane. V starts at the first control point and repeats every 5 m of 3D centerline length, including elevation changes. The pattern continues onto the shoulders. Markings, curbs, and medians use their own colors.

For a road with **Road edges** set to **Transition strip**, drag a **Transition** material onto the separate **Transition strip material** thumbnail. This also works when all selected roads have transition strips. In the Material Editor, assign base color, normal, and ORM for each side. The inner set meets the pavement or shoulder at vertex red 1. The outer set meets the terrain at red 0. Both sets use the road UVs and blend across the 40 cm strip. An unassigned strip displays its red-to-black mask.

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

`ROAD` version 13 adds a one-way flag to the road style and optional highway ramp station and attachment data to each control point. Roads from before version 13 load as two-way roads with no ramp attachments. Version 14 stores the signed distance from the host control point to the ramp attachment. Version 13 attachments start at zero offset. Version 15 adds a connector socket ID and a matching target ID to point records. Zero means unused. Version 16 appends a railroad flag to the style, followed for railroads by the embankment crest width and height in meters. The section stores roads in draw order. Its payload is whitespace-separated text in the classic locale. Each road starts with lane count, curved flag, left and right shoulder flags, three-lane layout, divided flag, divided gap, edge treatment (0 for none, 1 for a transition strip, 2 for a curb), median type (0 for concrete, 1 for grass), and the quoted material path relative to `assets/textures`, followed since version 11 by the extra one-lane width in meters. Version 12 appends the quoted transition material path, which is empty for an unassigned strip. Older worlds load with an unassigned strip. Versions before 11 load with zero extra width. Versions 1–5 use the default asphalt material. Versions 1–4 default to a concrete median. Version 1 roads load with no edge treatment. Version 4 adds the whole-road XYZ rotation (radians) after the style. Version 8 adds a persistent road ID after the rotation. Earlier worlds acquire IDs on load. Version 10 follows the ID with the paint-markings flag and a quoted, comma-separated list of road names. Earlier roads load unnamed with markings enabled. Version 18 inserts the crossing links described below. Next comes the point count, then each point’s world XYZ, manual-tangent flag, heading, pitch, and (since version 4) roll. Version 7 adds an intersection ID after each point. Zero denotes a free point. A positive ID is followed by an anchor index (0 for +X, 1 for +Z, 2 for −X, 3 for −Z in the junction’s local frame) and is valid only at a road end. Version 9 appends incoming and outgoing Bézier handle lengths to each point, expressed as fractions of their respective chord lengths. Earlier versions use one third for both. Roads from before version 7 load with free endpoints. Older roads start with zero whole-road rotation and point roll. Versions 1 and 2 load with zero pitch for manual tangents. Flags are 0 or 1. The reader rebuilds the meshes from these records. An absent or empty `ROAD` section means the world has no roads.

`ROAD` version 19 appends the dead-end transition length in meters to each point, after its version-17 surface-up vector. Zero disables the cap; positive values are valid only at free road endpoints. Earlier versions load without dead ends.

`ROAD` version 20 appends an outgoing segment lane count to each point. Zero inherits the road-wide count; 1–10 explicitly sets that segment's count. The last point's value is unused. Two lane counts after the whole-road rotation preserve neighboring lane configurations at free endpoints created by a split; zero means no external neighbor. Older worlds default these values to zero. Divided roads require even counts, and railroads cannot have lane overrides.

`ROAD` version 21 appends left and right shoulder overrides to each point after its segment lane count. Values are 0 for inherited, 1 for off, and 2 for on. Four values after the lane-neighbor counts preserve first-end left/right and last-end left/right shoulder context at splits. Older worlds default to inherited shoulders, retaining their road-wide settings. Railroad shoulder overrides must be zero.

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

The optional `OBJS` version 3 section stores one record per object. Each record contains the type, quoted GLB path, position XYZ, rotation XYZ, scale XYZ, health, and an invincibility flag (0 or 1). Type IDs are 0 for a generic object, 1 for a building, 2 for a vehicle, 3 for a prop, 4 for a character, and 5 for a marker. A marker record ends with its quoted UTF-8 label, quoted material path, and three text color channels (RGB, 0–255). Versions 1 and 2 also load. Older markers use their original mesh colors and white text. A vehicle record ends with its acceleration in meters per second squared. Existing `MESH` placements keep their original transform, including any shear.

### Mesh editor

Double-click a GLB model in the Content Browser's **Models** tab to open a floating **Mesh Editor**. Hold **LMB** in its viewport to orbit, use the mouse wheel to zoom, and press **F** to frame the model. The preview resizes with the window. The main editor remains usable while this window is open.

The right panel lists the model's named glTF material slots. Drag a `.bmat` from the Content Browser's **Textures** tab onto a slot; use **X** to clear its assignment. Scroll the panel for meshes with many slots. The preview and existing world instances update together, using the same material renderer as the game, including alpha cutout and translucent unlit materials. Unassigned slots retain their imported mesh colors. A Marker or terrain placement's explicit material override takes precedence over these model defaults.

Assignments save immediately to `model.glb.bmesh` next to the GLB in `assets/models`. The original GLB is unchanged. These are shared model settings, not world edits or world undo actions; keep the sidecar in source control alongside the model. Rebuilding copies it to `build/cooked/models`. Recook the world after changing assignments so its material dependencies and asset fingerprints are current. Models without a sidecar continue to load as before.

### Place markers

Right-click in the 3D viewport and choose **Create → Marker**. The editor places `assets/models/objects/marker/marker.glb` at the picked position and selects it. In the inspector, enter a place name in **Marker text** and press **Enter**. An empty name hides the lettering. The Outliner shows the name and the world saves it with the marker. Drag a `.bmat` from the **Textures** tab onto **Mesh material** or directly onto the marker mesh. **Clear material** restores the model’s own colors. Choose **Text color** to set the label independently. Windows opens a color picker, and the SDL inspector accepts a `#RRGGBB` value. Each marker saves its own settings and supports Undo and Redo.

In the game and the editor, the lettering faces the active camera around the world Y axis. Its lower edge is 50 game meters above the transformed mesh. The marker’s X scale controls letter width and its Y scale controls letter height. At unit scale a capital H is 100 game meters tall.

### Terrain chunks

Open the single-chunk example from the repository root with `.\build\BigSkyCountryEditor.exe examples\test-terrain.wld`.

The test grid at `assets/models/terrain/test_grid.glb` has 10 × 10 shared vertices on the XZ plane at 5 m spacing. Each placement has its own position and rotation in the Transform panel.

Drag a material from the Content Browser’s **Textures** tab onto a chunk in the 3D viewport. The assignment belongs to that placement, saves with the world, and supports **Ctrl+Z** and **Ctrl+Y**. The texture repeats every 4 m in the chunk’s local coordinates.

Selected terrain chunks have an orange boundary outline, including when multiple chunks are selected in the Outliner. The outline follows the mesh perimeter rather than the interior triangle edges and updates when the terrain is regenerated.

To fill a block, Shift-click the roads and intersections surrounding it and press **F**. You can also select the group in the Outliner and press **F** there. The chunk inherits the Outliner folder of the first boundary road in world order that belongs to a folder. If all boundary roads are at the world root, the chunk stays there. The new terrain follows the outer edge of the pavement, shoulder, transition strip, or curb and the rounded corners of generated intersections. At intersections without generated pavement, the boundary follows the attached anchors to the junction center. The interior has shared vertices at roughly 8 m spacing, with wider spacing on very large blocks. **View → Wireframe** shows the triangles. Move the surrounding roads to reshape the chunk, or select the chunk to assign a material or press **Delete**. If the perimeter opens, the chunk keeps its material and boundary links and becomes visible again when the loop closes.

The optional `TERR` version 4 section distinguishes imported meshes and generated fills. A `mesh` record stores the quoted model path, position XYZ, rotation XYZ in radians, and quoted material path. A `fill` record stores the quoted material path, a counted list of road IDs, and a counted list of intersection IDs. The editor and game regenerate the mesh from those neighbors when loading the world. Version 4 also permits an intersection-only fill for a central island. Version 3 fills and versions 1 and 2 imported chunks still load.

### Splitting roads

Select a road to show its control points, then right-click an interior control point in the 3D viewport and choose **Split Road**, below **Create**. The chosen point becomes the last point of the first road and the first point of the second road. The roads can then be selected and edited separately. The item is disabled on the first and last points because either would produce an empty road.

Both sections inherit the original road's names, material, lane and edge settings, paint setting, and Outliner folder. Bézier tangents are retained at the new ends. Attached intersections and ramp connections stay with their corresponding section, and terrain boundaries include both road IDs. Existing ramp tapers contract if a split shortens their available run. Splitting is one undo/redo step and saves with the world. Railroad control points support the same operation.

### Joining road endpoints

Select two roads with **Shift-click**, then **Shift-click** one free endpoint on each road in the 3D viewport. Press **F** to connect them. One road gains a new segment using its existing material and road settings; both roads retain their identities and Outliner folders. Any combination of start and end points is supported. Coincident endpoints are joined directly without adding a zero-length segment.

The new end follows the other road's endpoint and cross-section when it moves. On Bézier roads it also follows that endpoint's tangent. Move the original endpoint to reposition the shared join. The link saves with the world and supports **Ctrl+Z** / **Ctrl+Y**. Once the perimeter is closed, select the surrounding whole roads and intersections and press **F** again to fill it with terrain. The terrain keeps following both roads. The existing exit-to-entrance ramp workflow still creates its separate one-lane connector.

New terrain fills prefer the full selected boundary. If long roads or railroads cross again outside the intended block and enclose another pocket using only a strict subset of that block's boundary elements, that incidental pocket is skipped. Select just its own boundaries to fill it separately. Independent blocks with different boundary sets can still fill together; equal boundary sets are kept. Water holes, exact boundary seams, and bridge-owned ground patches retain their existing behavior.

Roads and railroads automatically included beneath a selected overpass provide ground seams and exclusions, but do not count as explicitly selected boundaries when choosing those pockets. This prevents their distant crossings from creating extra fills alongside the intended block. Select the lower roads or tracks explicitly when you want them to define another terrain area.

`TERR` version 9 records this choice as a trailing boolean on each `fill` record, so new fills keep the same behavior after editing, undo/redo, saving, and reopening. Existing fills from older files keep their original coverage. To correct an older fill that covers unwanted extra areas, delete that terrain chunk and recreate it from the boundary selection. Recook the world to update streamed terrain.

### Moving roads and intersections together

Click a road surface or an intersection center, then **Shift-click** other roads or intersections to add them. **Ctrl-click** a group member to remove it. The Outliner shows the same selection. In the 3D viewport, use **W** to translate or **E** to rotate about the average of the selected road pivots and intersection centers. Groups of roads can rotate on all three axes. A group containing an intersection rotates about Y to keep each crossing level. In local space, the lowest-numbered selected intersection supplies the axes, or the lowest-numbered road if the group contains only roads. The Transform panel shows the pivot’s world position and the reference member’s orientation. Editing either moves or rotates the group. Hold **Ctrl** during a rotation to snap to 5°.

In the top-down view, use the same selection gestures on the road surfaces and intersection centers. Drag the group's blue center handle to move it, or its gold handle to rotate it. A moved intersection carries its connected road ends. Moving a selected road away from an unselected intersection releases that connection. **Esc** cancels the drag and **Ctrl+Z** undoes the whole edit.

### Highway entrances and exits

Select a road with at least two lanes in the 3D viewport. Right-click the **green guide in an outer travel lane** and choose **Create Exit Ramp** or **Create Entrance Ramp**. The ramp joins the highway at the clicked location, and its free endpoint is selected for editing. Each ramp starts as a one-lane, one-way Bézier road with the highway’s pavement material. The initial 60 m taper needs clear space upstream for an exit or downstream for an entrance. An unavailable direction is grayed out.

Select the highway or its ramp to see two diamonds on the outer lane. Drag the **green diamond** to slide the ramp attachment along the road. Drag the **pink diamond** to move the narrow end of the taper. Each drag follows the lane through curves and elevation changes and stops at neighboring tapers or road ends. **Esc** cancels a drag and **Ctrl+Z** undoes a completed edit. If a later highway edit shortens the available space, the tapers contract to fit and expand again when space returns.

To connect an exit to an entrance, Shift-click both ramp surfaces in the 3D viewport, then Shift-click the free control point on each ramp. Both selected points turn amber. Press **F** to create a one-lane, one-way Bézier road from the exit to the entrance. The connector follows the position and tangent of each ramp end. **Ctrl+Z** undoes the connection.

Moving the highway station, editing its grade, or subdividing its spline carries the joined end of the ramp with it. The opposite ramp end stays editable. Removing the highway or its station leaves the ramp as a standalone road. Undo restores the connection.

### Intersection anchors

In the 3D viewport, right-click and choose **Create → Create Intersection**. On a road, the editor inserts the intersection at the clicked location along the centerline. You can also right-click the blue spline or an interior control point on a selected road. The junction follows the road heading, splits it into two roads, and connects them to opposite anchors. Each side needs room for its 10 m anchor and a remaining approach. Right-click the first or last control point to center the intersection on that endpoint. The road is trimmed to the approach anchor and the other three anchors are free. This requires room for the 10 m anchor on the road side only. An endpoint already attached to a junction cannot receive another one. **Ctrl+Z** restores the original road in one step.

Away from a road, the command places a standalone intersection using the usual surface or ground-plane placement. Pick the cross or its Outliner row to edit it again. Use **W** to move the center and **E** to rotate the junction around its vertical axis. The Transform fields also accept a location and a Y rotation. The intersection inspector lets you adjust the corner radius and assign a pavement material. The horizontal ring previews the radius. The optional `INTS` section in the world file stores these settings. Version 3 adds yaw in radians. Earlier versions load with zero yaw. Version 4 records whether the junction was inserted with through pavement. Version 5 adds the four anchor skew angles in radians. Older worlds open with zero skew.

Each intersection has four permanent control-point markers, 10 m from its center along its local X and Z axes at the intersection elevation. In the 3D or top-down view, select a road and its first or last point, **Shift + left-click** a free anchor, then press **F**. Both markers turn amber before you connect. The editor adds one segment to that end of the road and saves the connection. **Esc** cancels the pending pairing. Each anchor accepts one road end. Moving the intersection carries its attached ends. Moving a road end away, extending past it, or deleting its intersection frees the connection. These edits support undo and redo.

To angle a ramp or cross street into a junction, select the intersection, click its anchor in the 3D viewport, and press **E**. Drag the green Y ring to turn that approach up to **45°** either way. The amber line shows its heading. Hold **Ctrl** to snap the skew to **5°** increments. **Esc** cancels the drag. You can also angle a free anchor before connecting a road to it.

Skewed approaches meet the intersection at a cut perpendicular to the road, keeping the usual 5 m lane width (plus any configured extra width, shoulders, or median). Corner curves follow the actual road-edge directions and adjust their tangent points as the skew changes. The rendered road ends can move outward to make room for those curves while the four authoring anchors stay fixed. Stop lines follow the angled approach, and curbs, transition strips, and terrain boundaries share the same join vertices.

To start a new road in either view, click a free anchor, then **Shift + left-drag** from that marker. The road previews at the intersection elevation using the current road defaults. Hold **Ctrl** during the drag to snap the heading in 5° increments from that anchor’s outward direction. Release Ctrl to aim freely again. A readout beside the cursor shows the preview road's spline length in meters (including curves) in both views, updating as you drag or toggle Ctrl snapping. Release the mouse button to attach the road and select its free endpoint, or press **Esc** to cancel.

Three occupied anchors generate a flat T-junction at the intersection elevation. A loop road can use two of those anchors. Connect its first and last points to separate anchors on the same intersection, then connect an approach road to a third anchor. Connecting the fourth side opens it into a four-way crossing. The surface uses the intersection’s pavement material and corner radius. It matches each approach’s shoulders, curbs and transition strips. By default, lane markings and divided-road medians end where the road meets the junction. Wide approaches and large corner radii can move that seam beyond the 10 m anchor. Disconnecting roads updates the surface immediately. An inserted junction also has pavement between its two opposite approaches before a third road is attached. The four anchor markers remain available for editing.

Select the intersection itself to choose **Continue paint markings → Standard, North/South, or East/West**. Standard keeps the junction interior unmarked. The other modes continue the selected pair's edge, center, and lane lines across the pavement, following skewed approaches and endpoint lane settings. North/South uses the local +Z/−Z anchors; East/West uses +X/−X, so the choice rotates with the intersection. Both opposite roads must be connected with paint enabled. Differing lane counts taper the solid markings and omit dashed white dividers, as on ordinary lane transitions. This changes paint only; it does not add a raised median through the junction. Stop-line checkboxes remain independent; clear them on the through approaches if traffic there should not stop. The setting supports undo/redo and is stored in `INTS` version 6 (0 = Standard, 1 = North/South, 2 = East/West). Older worlds default to Standard.

### Y-intersections

Right-click in the viewport and choose **Create → Create Y-Intersection**. This creates three anchors: a stem and two branches, initially 120 degrees apart. Connect roads to these anchors with the usual endpoint + Shift-click + **F** workflow, or Shift-drag a free anchor to grow a road. Pavement is generated once all three anchors have roads. Use the intersection rotation gizmo to orient the junction, and rotate individual anchors to skew their approach by up to 45 degrees (Ctrl snaps to 5 degrees).

Select the intersection to edit **Size: center to anchor (m)**, from 10 to 2000 meters (default 60). Enter commits the change. Size spreads the anchors and moves the connected road endpoints; it does not scale the roads' lane widths. If an enlarged junction would move an anchor past the next road control point, move that control point outward first. Size changes support undo/redo.

**Central island** is enabled by default. The island follows an inset of the actual pavement curves, so each side expands independently to use the available space, including skewed or T-shaped layouts. At least a full approach width of pavement is retained around it. Very small junctions or very wide approaches may leave no room for an island; increase Size to reveal it. **Island corner radius (m)** rounds its corners. Curbs/transition strips continue around the island, inherited from the connected approaches (curbs take priority).

Select the Y-intersection and press **F** to fill the island with subdivided terrain. Its terrain material can then be assigned normally. This fill follows size, shape, elevation, and neighboring road edits; turning off Central island temporarily removes its generated mesh. A later F selects an existing fill instead of duplicating it. Ordinary surrounding terrain fills also recognize the island boundary. Through-paint modes remain specific to standard intersections; stop lines still have a separate checkbox at each Y anchor.

`INTS` version 7 adds the intersection shape (0 = Standard, 1 = Y), size, and central-island flag after the paint setting. Earlier intersection records retain their original four-anchor layout. New settings are saved in the world and used by both game and editor.


Click an intersection anchor in the 3D or top-down view to edit its **Stop line** checkbox. Each anchor defaults to on and remembers its setting when a road is disconnected. On a generated junction the white bar runs across the incoming travel lanes, starting at the road paint and extending 0.60 m onto the intersection. When an attached road is selected, its endpoint takes priority at the shared marker. Select the junction surface or its Outliner row, then click the anchor to inspect it.

### Lakes

In the 3D viewport, right-click and choose **Create → Create Lake**. A lake starts as a 200 m diameter circle with four shoreline control points at 90-degree intervals, with shoreline editing already active. To reshape an existing lake, select its water surface or its Lake row in the Outliner, then click **Edit Shoreline** in the inspector.

In shoreline edit mode, drag a teal point to reshape the lake in XZ, or select it and use the translation gizmo/Transform fields. Double-click between points on the shoreline to insert a point without changing the existing curve. **Shift-click** adds shoreline points to the selection; **Ctrl-click** removes them. Drag a selected point or use the translation gizmo/Location fields to move the group in XZ. **Ctrl+Shift+left-drag** selects shoreline points in a box. **Delete** removes the selected points together; at least three must remain. Only the spline preview updates during editing; the previous water and terrain meshes remain visible. **Ctrl+Z/Ctrl+Y** undo and redo draft edits without rebuilding either surface.

Click **Finish Editing**, or press **Enter** in the viewport, to rebuild both surfaces once and save. **Cancel** (or **Esc** when not dragging) restores the original shoreline without rebuilding. **Esc** during a drag cancels only that drag. Self-crossing or collapsed shorelines cannot be finished; fix the points or cancel the edit. Saving, selecting another item, changing to top-down view, or closing the editor also finishes the edit. A finished session is one undo step. Outside shoreline edit mode, the whole lake can be translated, rotated around Y, or deleted.

Terrain fills automatically leave holes for lakes and meet their exact shoreline vertices at the water's elevation. The lake supplies its own submerged terrain; the surrounding fill does not cover it. Select the surrounding roads/intersections and press **F**, as usual. Selected rivers/lakes can also define part of the outer perimeter; ordinary terrain and props in a box selection do not. Multiple lakes inside the same area are supported. Existing fills update when a lake is added, moved, deleted, or reshaped, including undo/redo and reopening the world. Shoreline draft edits keep the previous fill intact until **Finish Editing**; canceling does not rebuild it. Unchanged fills and edits to distant lakes reuse their geometry. Recook the world to include the updated holes and lake dependencies in streamed game geometry.

Water stays horizontal. A subdivided terrain basin slopes down from the exact shoreline to a maximum depth of 5 m. The terrain material slot affects the bed; `assets/textures/world/M_LakeWater.bmat` supplies the shared translucent water color. The bed participates in collision, while water does not act as a solid floor. Lakes are standalone terrain chunks; this first version does not excavate overlapping terrain or add swimming/buoyancy. Only the finished spline points and handles are saved in the world; meshes are rebuilt on Finish, undo/redo of a finished edit, or world load. Lakes also appear in the top-down overview and respect Outliner folder visibility.

### Rivers

In the 3D viewport, right-click **Create → Create River**. The new river starts at the cursor, 25 m wide and 200 m long, with **Edit River Banks** active. Two teal open splines define its banks, and larger green handles mark the center of its start and end. Drag either bank's teal points independently to change the width and course. Double-click a bank between points to insert a control point while preserving that curve; each bank may have a different point count. **Delete** removes a selected bank point, leaving at least two per bank.

Select a green end handle alone, then **Shift-drag** it to extend the river from either end. This adds a point to both banks, preserving the end's width and current elevation. Dragging an end handle without Shift moves both bank ends together. Bank points and end handles support all three translation axes and Location fields: select a point, then drag the green **Y** gizmo or enter **Location Y** to change its elevation. Raising/lowering an end handle changes both banks together; individual bank points can be adjusted independently. Direct point dragging moves horizontally at that point's current height. Double-click insertion preserves the full 3D curve.

**Shift-click** adds bank points or end handles to the selection; **Ctrl-click** removes them. **Ctrl+Shift+left-drag** selects control points in a box, limited to the river being edited. Selected points turn orange. Drag the **Y** translation arrow to raise/lower them together, or enter **Location Y** to set the group's average elevation while preserving height differences. X/Z translation works the same way, and dragging a selected point moves the group horizontally. End handles and their bank points never move the same point twice. **Z** frames the selected controls. Group moves and deletions support draft undo/redo; deletion must leave two points on each bank and cannot remove green end handles.

Only the splines update during editing; the previous water and bed remain visible. **Finish Editing / Enter** rebuilds the surface and subdivided riverbed once. **Cancel / Esc** discards the draft; Esc during a drag cancels that gesture. Ctrl+Z/Ctrl+Y work within the draft, and a finished editing session is one document undo step. Crossing/collapsed banks are rejected on Finish, keeping the edit open and the last valid geometry intact. Water follows the bank elevations, including changes in grade. The bed dips below the local water surface by up to 5 m, tapering to meet the exact shoreline, with submerged interior edges even in narrow sections. Water and bed use matching triangulation to prevent the bed crossing the surface on bends or slopes. Rivers share the lake water material; lakes remain level. The bed, rather than the water surface, supplies collision.

Rivers appear as **River** in the Outliner, support normal selection, movement, Y rotation, deletion, hiding, and top-down box selection, and participate in visibility and streaming. Road-bounded terrain fills automatically leave their footprints uncovered and follow committed bank elevations just as they follow lake shorelines. Overpass clearance uses the water surface directly beneath the deck, including interior rises. Authored `TERR` version 8 stores XYZ positions and handles for river banks; version 6/7 rivers load with zero local elevation and preserve their existing placement height. Older terrain and lake records still load. Recook the world after editing rivers for the streamed game.

To fill land bounded by riverbanks, finish bank editing, then **Shift-click** the rivers and surrounding roads, railroads, or intersections in the 3D viewport or Outliner. **Ctrl-click** removes a selected element; 2D box selection works too. Press **F** to fill the enclosed dry land. A river can complete an otherwise open road boundary, and multiple rivers can enclose land together. Road edges must actually meet the banks at compatible elevations (or through the existing overpass ground seams); open gaps are not bridged automatically. The fill preserves the committed bank edges and leaves the water and riverbed intact.

Explicit water boundary references are saved by stable ID in `TERR` version 7. Moving or reshaping a boundary river updates its fill after **Finish Editing**; drafts leave the old fill in place. Removing a required river suspends the fill until that same source returns, including through undo. Reordering Outliner items cannot reassign the bank. Water contained within a road boundary is still discovered automatically, even when it was not selected. Older worlds load with no explicit water references. Recook after creating or changing fills to update streamed geometry and collision.

### Scene outliner

The **Outliner** above Transform lists the world’s roads, intersections, terrain chunks, placed objects, and static meshes. Click an entry to select it. Selecting a road or a control point in the viewport highlights its road in the list. Selecting a single road, object, or imported terrain chunk in the list makes its transform available for editing.

Folders display `assets/textures/editor/folder_icon.png` beside their names, including empty folders. Expanded and collapsed folders (and the world root) are remembered automatically in a small `.outliner` file beside the world file, for example `world.wld.outliner`. This view state does not change the world’s unsaved indicator or add undo steps. Selecting an item in the viewport still opens its parent folders to reveal it.

Drag the horizontal bar beneath the Outliner to give more space to the Outliner or the properties panel.

Use **New folder** to name a folder. Selecting a folder first creates a subfolder. **Rename** or **F2** changes a folder name. **Ctrl-click** toggles individual entries and **Shift-click** selects a range. Drag the selection onto a folder, or drop it on the world row to move it to the root. A label follows the cursor with the dragged row’s name and a green check for an allowed move or a red blocked icon. When dragging several rows, the label also shows how many others are selected. You can also move a folder with its descendants. Multi-selection is for organizing entries.

To reorder folders, drag a folder to the gap above or below another folder and release when the black insertion line appears. The line’s indentation shows the destination level. Dropping on the middle of a folder moves the selection inside it. Several selected folders move together in their displayed order.

Right-click a folder and choose **Hide Children** or **Unhide Children** to change visibility throughout its subtree in both editor views. Hidden entries stay in the Outliner with dimmed labels. You can unhide a subfolder independently. The world saves these editor visibility settings and the game displays the full scene.

With a folder selected, **Delete** moves its contents to its parent and removes the folder. Folder edits save immediately and support **Ctrl+Z** and **Ctrl+Y**. The optional `OUTL` section in each `.wld` stores its hierarchy, folder order, and item assignments. Worlds without that section open with all entries at the root.

### Transform panel

The Transform panel sits above a selection-sensitive properties inspector. Roads and control points show the road settings. Terrain chunks show their mesh and material slot, and objects show their mesh and Invincible setting. Click the pavement to select a whole road, a visible control-point marker to edit just that point, or a placed object or terrain chunk to edit its transform. With **Use Geographic Coordinates** unchecked, locations are world coordinates in meters, shown to two decimal places. Rotations are in degrees. Only objects have editable scale.

Check **Use Geographic Coordinates** to display the selection’s latitude and longitude. Paste a Google Maps pair such as `35.0851266266639, -106.65282545913769` into **Latitude, longitude** and press **Enter**. Both horizontal axes update in one undo step. The separate elevation field uses game meters.

The world origin is **40° N, 98.5° W**. The editor projects WGS 84 coordinates using ellipsoidal Mercator with true scale at 40° N, subtracts the projected origin, then divides east and north distances by six. +X points east and +Z points north. This keeps constant-latitude lines horizontal and preserves local angles. Distance scale varies with latitude. The checkbox changes the coordinate display and input. World files continue to store game meters.

Type a value in one axis field and press **Enter** to apply it. Location and rotation fields also accept addition/subtraction expressions, such as `125+10`, `90-5`, or `-100+25-5`. Append `+10` or `-10` to the existing text to offset that displayed value; a standalone `-10` still sets an absolute negative value. Expressions use meters for location and degrees for rotation, support decimal/scientific notation, and are evaluated on Enter. Each half of the latitude/longitude pair accepts the same expressions in decimal degrees. Invalid or non-finite expressions leave the transform unchanged. **Esc** or leaving the field discards uncommitted text. Each change saves the world and supports **Ctrl+Z** and **Ctrl+Y**. The other axes keep their full precision.

With a whole road selected, press **W** for translation arrows or **E** for rotation rings. Drag a handle to move or rotate the pavement, markings, edges, and control points together. The first control point is the pivot for both the gizmo and the Transform panel. In local space the handles follow the road’s stored rotation.

For a control point, X is roll about the tangent, Y is yaw about world up, and Z is pitch above the horizontal. Point roll sets the orientation of its local editing axes. A straight point’s yaw and pitch pivot its neighboring spans. A Bézier point’s yaw and pitch set its tangent. When a segment is highlighted, the panel shows its owning road.

### Editor controls on Windows

The editor uses a dark charcoal theme. New panels and controls should use the UI color roles in `editor/editor_theme.hpp`. For a native Windows dialog, call `WindowsTheme::ApplyDialog` during `WM_INITDIALOG` to theme its standard controls. Viewport selection colors are defined in `shared/editor_colors.hpp`.

Drag a GLB model from the **Models** tab into the 3D viewport to place a **Prop**. The lowest part of the model sits at the height of the nearest road or terrain triangle under the cursor. On empty ground it sits at Y = 0. If the ray misses the ground, its pivot lands 20 m along the cursor ray. The new prop is selected. Use **W** to move it, **E** to rotate it, or **Delete** to remove it. Placement, transforms and deletion save to the world and support **Ctrl+Z** and **Ctrl+Y**.

Select any object, including a Marker, press **W**, then hold **Shift** and drag a translation arrow to place a copy in the same Outliner folder. The copy keeps the object’s properties and stays selected. **Esc** cancels the drag and **Ctrl+Z** undoes the duplication.

In the Content Browser’s **Textures** tab, select a folder to browse its texture thumbnails and materials. Right-click an empty area and choose **Add New Material**. Enter a name to save a versioned JSON `.bmat` asset in that folder. Right-click a material and choose **Duplicate**, or select it and press **Ctrl+D**. The copy is selected in the same folder and shares its source textures. Names advance to the next available number, for example `M_Asphalt01` becomes `M_Asphalt02`. Material creation and editing always write to the project’s `assets/textures` directory. Choose **Rename** (or press **F2**) to change a selected material’s filename; name collisions are rejected. Choose **Delete** (or press **Delete**) to remove a material after confirmation. These shortcuts act only while the Content Browser asset list has focus. Deletion keeps texture files and surfaces, which use their fallback appearance until another material is assigned. Material file operations are immediate and are not part of world undo/redo.

Renaming preserves existing assignments in saved worlds, undo history, and the game through small `.bmat.redirect` files beside the source materials. They are hidden from the Content Browser and copied to `build/cooked` during a build. Keep these files in source control with the renamed material. Former names remain reserved while their redirects exist; renaming a material back to one of its own former names is supported. An open Material Editor follows a rename and closes when its material is deleted. Rebuild the game after asset changes to update and prune cooked copies.

Double-click a material to open its floating **Material Editor**. Hold the left mouse button and drag in its preview viewport to orbit the sphere. Drag textures from any Content Browser folder onto the **Base color**, **Normal**, **ORM**, **Emissive**, or **Opacity** thumbnail. Use **Clear** to remove a map. The sphere updates and the material saves after each change. ORM uses red for occlusion, green for roughness, and blue for metalness. **Alpha cutout** discards pixels where base-color alpha multiplied by the opacity map’s red channel is below 0.5. **Standard** and **Terrain** both render the whole surface and use the same texture slots. Materials reference the original textures by their paths and share the same decoded images.

Choose **Unlit** for a surface whose color stays constant as the world lighting changes. Its **Emissive** map is multiplied by the **Emissive tint**, which defaults to white. The **Translucency** map’s red channel multiplied by the slider sets opacity. A black texel is invisible, and a white texel at the default slider value of **1.00** is opaque. Both slots use white when empty, so the tint and slider also work on their own.

The content browser uses a native tree of the GLB files in `assets/models`. Select a model to preview it, or click **Refresh** after changing the folder. The road inspector uses checkboxes for shoulders and drop-downs for alignment, three-lane layout, and edges. Its changes use the editor’s existing undo and save commands. The toolbar’s camera-speed slider stays in sync with the viewport mouse wheel.

New roads default to **Bézier** alignment. For a one-lane road, **Extra width (m)** adds to the 5 m travel lane in 0.5 m steps, evenly on both sides of the spline. Shoulders sit outside the widened lane. You can edit multiple one-lane roads together. Changing the lane count keeps the extra width setting for the next time the road has one lane.

For an even lane count, enable **Divided road** in the road inspector. The lane buttons then add or remove one lane in each direction. **Separation (m)** measures between the inside edges of the opposing travel lanes and changes in 0.5 m steps. At 2 m, the two 1 m paved inner shoulders meet. A wider separation adds a flush **Concrete** or **Grass** median between them. For example, a 6 m separation gives a 4 m median. Each carriageway has a solid yellow line along its inner lane edge.

Use **Tab** or **Shift+Tab** to move between the viewport and controls. In the 3D viewport, hold the **right mouse button** to look around and fly. **W/S** move forward and backward, **A/D** move left and right, and **Q/E** move down and up. Releasing the button stops keyboard movement immediately.

Press **Z** in the viewport or Scene Outliner to focus the camera on the current selection. Multiple selected items are framed together, and a selected folder frames its visible contents. The 3D view keeps its viewing direction and fits the selection with a little padding; the top-down view centers and zooms to fit. Selected control points or road segments focus just those elements. **Ctrl+Z** still undoes edits, and typing in a text field does not move the camera.

Road names appear above named roads in both the 3D viewport and top-down view. Enter names in the road inspector (including comma-separated aliases); the labels follow the road direction, stay readable as you zoom, and repeat along long visible roads. Hidden roads and unnamed roads have no labels. Overlapping labels are suppressed, so zoom in to identify roads in dense areas. **View → Road Names** toggles this editor-only overlay; it starts enabled each session and does not change the world or game rendering.

Choose **View → Wireframe** to see the visible triangle edges of roads, terrain chunks, and placed objects in the 3D viewport. Choose it again to restore shaded surfaces.

**View → Fog** toggles atmospheric fog in the editor's 3D viewport. It starts enabled each session, and the game uses the same fog by default. Distance haze blends roads, railroads, terrain, objects, and translucent water into the sky; it is denser near sea level and thins at altitude. Fog colors follow the sky's direction and time of day, so night scenes stay dark. The first 50 m remain clear. Splines, gizmos, selection outlines, labels, wireframe, material previews, and the top-down view remain clear for editing. The toggle changes only the editor view; it does not modify or save the world.

Fog is evaluated per pixel in the existing world shaders, including unlit/translucent materials, using an analytic exponential-height integral and a thin uniform haze layer. It does not rebuild meshes, invalidate resident GPU geometry, or require an extra render pass. Matching math supports the software renderer. Default density, height falloff, base height, and near distance are in `shared/world_fog.hpp`, in engine meters. This is atmospheric haze, not shadowed volumetric fog or light shafts.

**Settings → Project Settings → Optimization → View distance (km)** sets the 3D visibility radius for the game and editor. It defaults to **8 km**, with **4 / 8 / 16 km** preset buttons and a custom range of **0.1–128 km**. Choose a preset or enter a value, then press **Apply**. Distances are in engine kilometers, with no additional geographic scale conversion. The editor applies changes immediately and saves `viewDistanceMeters` in `assets/project-settings.json`, mirroring it to `build/cooked/project-settings.json` for the next game launch. Existing settings files without this key use 8 km. These are project defaults; per-player graphics overrides will come with the later streaming work.

**View → Visibility Statistics** or **F3** toggles a 3D viewport readout; **F3** also works in the game. It shows total indexed scene entries, nearby entries, submitted entries, occupied/query cells, broad-phase candidates, oversized entries, hidden/frustum-rejected entries, changed index records, and CPU sync/query time. Counts refer to static scene entries, not triangles or draw calls; dynamic pawns and editor overlays are separate. The display starts off and does not change the world or undo history.

This first visibility milestone maintains a sparse 512 m XZ grid of loaded road/railroad, intersection, terrain/lake, model-placement, and object bounds. A lightweight metadata synchronization detects edits and removals each 3D frame, but only changed bounds update cell membership; unchanged mesh placements reuse transformed bounds. Queries use distance to the nearest point of each **3D bounding box**, followed by camera-frustum rejection. Large objects occupy multiple cells and are submitted once. Extremely large bounds use a conservative overflow bucket rather than allocating a continental grid. Marker bounds include a conservative envelope for their camera-facing labels.

A matching spherical pixel cutoff in the GPU and software renderers handles geometry that straddles the distance boundary, including long roads, water, paint, selection outlines, and wireframe. It works with fog on or off and does not regenerate or re-upload unchanged resident geometry when the distance changes. Road-name spans obey the same radius. Sky, material previews, and foreground gizmos remain unaffected; the orthographic road editor keeps its existing viewport culling. Visibility follows the active camera, including the game's F1 debug camera.

**Cooked-world streaming (milestone 3):** use **File → Cook World** after building this update. The game loads and unloads nearby geometry, shared models/textures, and collision asynchronously from format-v2 packages. Separate near/far/collision payloads avoid loading full-detail distant geometry. Project Settings now includes geometry, shared-asset, and collision budgets; F3 reports residency, evictions, errors and memory pressure. Collision stays loaded around every live pawn, including parked vehicles, and physics waits if required chunks are missing. The editor keeps authored data, selections, edits and undo history resident while reclaiming unused render caches. Older packages use the authored-world fallback until recooked. See [streaming format and behavior](docs/streamable-world.md).

Optional regression tests: configure with `-DBIGSKY_BUILD_TESTS=ON`, build `BigSkyVisibilityTests`, then run `ctest --test-dir build --output-on-failure`. The tests compare indexed queries against brute force, including negative coordinates, exact cell edges, oversized geometry, altitude, transforms, compaction, deletion, and same-size replacement.

Right-click the 3D viewport and choose **Create → Create Road**. The first control point lands on the nearest road triangle under the original click, including elevated roads and curbs. On empty space it lands on the ground plane (Y = 0), or 20 m along the ray if the ground is out of reach. The new road starts with a level 10 m segment along world +X. Its far endpoint is selected for Shift-drag extension. Creation saves the world and supports **Ctrl+Z** and **Ctrl+Y**.

Select two or more roads in the viewport or Outliner to edit their properties together. The road inspector shows their combined length and marks settings that differ. Each edit changes that property on every selected road and takes one **Ctrl+Z** to undo. For mixed lane counts or separations, **+** steps above the largest value and **−** below the smallest, within the road style limits. Drop a material on the pavement slot to assign it to the selection.

In the road inspector, enter one or more comma-separated **Road names**, such as `US 40,Main Street`, and press **Enter**. The Scene Outliner displays those names. Clear **Paint markings** to hide that road’s lane lines, edge lines, and intersection stop bars. Re-enabling paint restores the stop bars enabled on its anchors.

Press **R** to switch between the 3D and top-down views. In the top-down viewport, press **N** to start or resume a road draft.

In the top-down view, **left-drag** away from a control point or transform handle to draw a selection box. Releasing selects whole roads (including railroads), intersections, terrain/lakes, objects, and static meshes whose projected geometry touches the box, at any elevation. Drag in any direction; **Shift** adds to the selection and **Ctrl** removes from it. **Esc** cancels without changing the previous selection. Hidden folder contents are excluded. A click without dragging still picks a single item, and an empty click clears the selection. Road creation and control-point dragging keep their existing gestures.

In the **3D viewport**, hold **Ctrl+Shift** and **left-drag** to draw a selection box. Releasing replaces the selection with all scene elements whose geometry touches the box, including objects behind other objects. Hidden folder contents and objects behind the camera are excluded. While editing a river or lake, the box selects only that water body's control points, using their current draft positions. An empty dragged box clears the selection without leaving water edit mode; a click without dragging leaves the selection alone. **Esc** cancels the box without changing the previous selection or ending water editing.

Objects and static meshes have lightweight bounding footprints in the top-down view, highlighted orange when selected. Imported terrain is also visible and selectable. Box selection updates the Scene Outliner without changing or saving world geometry; intersection tests run only on release.

The top-down view retains road, intersection, road-bounded terrain, rail, and tie geometry in GPU buffers on the Direct3D 12/Vulkan renderer. Panning and zooming update camera uniforms instead of reprojecting and uploading every triangle. Geometry edits invalidate the affected batches; offscreen surfaces are culled before submission, and railroad detail distance still applies. Draft previews and road-name layouts are reused until their inputs change. Other SDL renderers use cached CPU geometry, with projection/clipping repeated only when geometry or the view changes. Cached geometry is released when leaving the top-down view.

### At-grade railroad crossings

Arrange a road and a railroad so their splines cross in plan view. In the 3D viewport, right-click either surface near the crossing and choose **Create → Create Railroad Crossing**. At an existing crossing, that item becomes **Remove Railroad Crossing**. Creation/removal supports undo and redo and saves with the world.

The same command also connects two railroads at grade. For now this is a terrain-fill connection: it leaves both rail, tie, and embankment meshes unchanged, with no diamond/switch geometry. Select the railroads and the other elements enclosing the land, then press **F** to fill it. The terrain follows both embankment rims (including transition strips) and adds small sealing faces where their edge elevations differ. Rail-head heights at the crossing must be within 0.25 m; adjust the track heights first for larger differences. Elevated or nearly parallel tracks are not accepted as at-grade junctions. The connection follows spline edits and splits, supports undo/redo, and persists in `ROAD` version 26. Terrain seams are regenerated only when their sources change and are included in cooked rendering/collision.

Splitting a railroad exactly at a crossing retains the connection to both halves. Existing worlds that saved a crossing link on only one half also recover the terrain seal through adjoining selected pieces at that crossing. Include all railroad pieces bounding the land when selecting the area to fill.

The railway keeps its rail alignment, grade, gauge, and complete embankment geometry. The road's generated approaches ease to rail-head height, including its shoulders, curbs, and median. The control-point positions remain authored values. A concrete slab replaces the road surface across the entire embankment, extending 0.5 m beyond its outer toes (including transition strips) with a 3 m margin beyond the road footprint along the track. The road pavement, edge treatments, median, and paint stop at the two slab boundaries. The approaches retain their pavement material and UVs. Each rail sits in a concrete groove with 6 cm of clearance on its gauge side, 1 cm outside, and a solid floor 6 cm below the rail head. These game geometry dimensions are in `RailroadSettings`.

The slab has solid side faces extending below the embankment toes. Its perimeter joins the terrain-fill boundaries, and these side faces seal the elevation difference where the unchanged railway meets the raised crossing. Terrain stays at the railway toe where it meets a slab wall instead of lifting the whole railway boundary to deck height. Terrain fill accepts that difference only where an actual slab wall covers it; unrelated overpasses still cannot enclose terrain. Edits to either spline or embankment settings rebuild the slab, approaches, and dependent terrain. The derived geometry is cached while the world is unchanged. Removing or splitting a source railway updates its crossing links and supports undo.

Leave enough spline length on both sides for the full crossing and gradual approaches. The planner uses roughly 15 m approaches, extending them for larger height differences, and rejects a crossing without room for at least 5 m of approach or with a height difference greater than one fifth of the available approach length. Nearly parallel splines (less than about 15 degrees apart) also need a different layout. A bridge remains separate unless a crossing is explicitly created.

`ROAD` version 18 adds a crossing count after each road's names, followed by railway ID and normalized road/rail XZ stations for each link. Older worlds load with no crossing links. Generated approach and concrete slab meshes are rebuilt from these links on load. Existing version 18 crossings automatically use the slab geometry without a file-format change.

### Railroads

In **Settings → Project Settings → Optimization**, enter the **Rail and tie render distance (m)** and press **Enter** or **Apply**. The default is **300 m**. The game and editor measure to the nearest point on each control-point segment’s sampled track. Beyond that distance the embankment and transition strip remain visible. Set the distance to **0** to hide rails and ties at all distances. The editor saves the project-wide value in `assets/project-settings.json` and stages it in `build/cooked` for the game.

In the 3D viewport, right-click and choose **Create → Create Railroad** to place 50 m of standard-gauge track. Select its teal spline or control points to edit it as you would a road. **Shift-drag** either endpoint to extend the track, or select a span and press **Ctrl+Q** to subdivide it. The control-point elevation is the base of the embankment.

To break a long railroad into smaller, independently editable pieces, select it and right-click an interior control point, then choose **Split Railroad** below **Create**. The first and last points cannot be split. If there is no control point where you want the cut, select that spline segment and use **Ctrl+Q** first. Both pieces inherit the track's embankment settings, materials, names, and Outliner folder. Bézier tangents keep the rails aligned at the cut; existing crossings and terrain boundary references follow their corresponding pieces. **Ctrl+Z** and **Ctrl+Y** undo and redo the split. Subsequent edits rebuild the edited piece's track geometry rather than the original full-length railroad.

The railroad inspector adjusts the flat crest width and the bed height in meters. The initial crest is 4 m wide and 0.75 m high. The rails have 1.435 m of clear gauge and the timber ties are spaced every 0.60 m along the track. Enable **Transition strip** for a 0.40 m blend at each toe and drop a Transition material onto the strip slot. The embankment has its own material slot.

Railroad saves contain the current control points and settings, not generated track meshes or undo history. Rebuilding replaces the generated geometry. Splitting reduces how much track a subsequent edit rebuilds; additional control points and more complex curves can still increase that work.

During viewport drags, consecutive queued left-mouse movements are combined before editing the spline, preserving key and button transitions. Picking rejects road and mesh bounds that the cursor ray misses before checking individual triangles. Crossing updates reuse unchanged road/railroad pairs; changes to either source still recalculate the crossing.

### Dead ends

Select a road and right-click either free end control point, then choose **Create → Dead End**. This keeps the existing pavement in place and adds a 5 m transition beyond the endpoint, following its direction and grade. The strip spans the road and shoulders, joins side transitions, and eases raised curb edges down at the far end. Paint stops at the original endpoint.

With that endpoint selected, edit **End transition (m)** in the inspector and press **Enter**. Set it to **0** to remove the dead end. Assign a **Transition** material to the road's **Transition strip material** slot, which is available whenever the road has a dead end, even with Road edges set to None. The strip uses red vertex color to blend from the material's inner layer at the pavement to its outer layer at the far edge.

Both ends can have independent lengths. Changes save with the world and support undo/redo. Moving or rotating the road keeps its ends attached; Shift-dragging an endpoint carries its dead end to the new endpoint. Splitting retains caps only at the original outer ends. Connecting a capped endpoint to another road or intersection removes its cap. Railroads, interior control points, and connected endpoints cannot receive dead ends.

### Per-segment lanes, shoulders and division

Select a road's cyan control spline segment, then use **Segment lanes** in the inspector to change that segment's lane count. **Shift+click** adds other segments on the same road. A mixed selection displays **Mixed**; the plus/minus buttons set all selected segments to the same count. Divided roads change in steps of two.

The adjacent segments provide smooth transitions between the edited configuration and the rest of the road. For example, changing a middle segment from two to four lanes gives it four lanes, with widening and narrowing tapers on the neighboring segments. Pavement, lane markings, green lane guides, shoulders, curbs, and transition strips follow the changing width. If differently edited segments meet directly, their adjoining halves share the taper. Terrain fills follow the new outer edges, and intersections use each connecting endpoint's lane count.

Dashed white lane dividers are omitted where through lanes are added or removed. They resume on constant-width sections with their dash spacing preserved. Solid edge lines, yellow markings, and the editor's green lane guides remain visible through the transitions.

Select the whole road and change its lane count to restore a uniform lane count, clearing its lane overrides. Other inspector properties, except shoulders and division, still apply to the whole road. Lane edits save with the world and support undo/redo; splitting and extending roads preserve their authored lane counts.

With segments selected, the **Segment shoulders (5 m each)** checkboxes toggle the left and right shoulders independently for those segments. Neighboring segments provide smooth tapers between no shoulder and the full 5 m width. Differently edited segments share the transition across their adjoining halves, just like lane counts. Lane edge lines stay at the travel-lane boundary, and curbs, transition strips, and terrain fills follow the changing outside edge. Changing a shoulder with the whole road selected clears segment overrides for that side only. Mixed selections show an indeterminate checkbox; clicking it enables that shoulder for the selection. Shoulder settings save with the world and support undo/redo, splitting, subdivision, and extension.

With segments selected, **Divided road**, **Separation (m)** and **Median** apply only to those segments. A divided segment requires an even lane count. To change a five-lane road with a center turn lane into a four-lane divided road, select the destination segments, reduce their lane count to four, then enable **Divided road**. Use **5 m** separation to replace the center turn lane exactly: two 1 m inner shoulders and a 3 m median. The through lanes retain their width and their white dashed dividers continue through this transition without restarting their dash pattern. Other separation values taper the carriageways sideways. To go back to an odd lane count, disable division on those segments first.

Division settings support undo/redo, save/load, splitting, subdivision and extension. Editing division with the whole road selected clears its division overrides. Existing worlds retain their previous road settings; save and recook the world after making segment changes to update the game.

### Editing road control points

Click a road to show its control points, then click a point to select it. Press **W** for the translation arrows or **E** for the rotation rings. Drag a red (X), green (Y), or blue (Z) handle. The green arrow changes elevation. Rotating a Bézier point adjusts its tangent, while rotating a straight-road point pivots the neighboring spans around it. Release the mouse to save the edit, or press **Esc** during the drag to cancel. Each drag can be undone with **Ctrl+Z** and redone with **Ctrl+Y**.

Selecting a road with two or more lanes in the 3D viewport shows a green guide down the center of each travel lane, including center turn lanes and lanes that taper into ramps. Where the middle lane coincides with the cyan editing spline, a green rim keeps both guides visible.

In the 3D viewport, each cyan spline between two consecutive control points represents one road segment. Click the spline to select that segment in amber. **Shift+click** adds a segment to the selection, and **Ctrl+click** removes just the clicked segment. Press **Ctrl+Q** to split each selected segment halfway along its 3D spline. The new control points become selected and the entire subdivision can be undone in one step. Click a control point to resume point editing. In either view, **Shift+click** adds a control point and **Ctrl+click** removes it from the selection. With multiple points selected on a road, drag a translation arrow in 3D or drag any selected point in the top-down view to move them together. Entering a location in the Transform panel places the active point there and shifts the others by the same distance and direction. Press **Esc** during a drag to cancel, or **Ctrl+Z** after release to undo the move.

With the viewport focused, **Delete** removes the selected road, segments, or control points. Deleting segments leaves a separate road for each continuous run of surviving segments. Deleting points reconnects the remaining points in order. A road is removed when it has fewer than two points left. Each deletion is saved immediately and can be undone or redone in one step.

The globe/cube button to the left of the camera speed slider toggles the 3D transform space. The editor starts in world space (globe). In local space (cube), a road point’s red X axis follows its tangent, its green Y axis is perpendicular to the grade, and its blue Z axis runs across the road. Translation arrows and rotation rings both use the chosen axes.

The grid button immediately left of the globe/cube toggles translation snapping. It starts off with **1 m** selected. With the viewport focused, **[** selects a smaller increment and **]** a larger one. The available increments are 1 cm, 10 cm, 1 m, 10 m, 100 m, and 1 km. The toolbar shows the current increment. A translation arrow measures signed steps from the drag's starting position along its world or local axis. This applies when Shift-dragging a road or railroad endpoint in either view, when adding points to a top-down road draft, and when growing a road from an intersection anchor. Top-down free drags snap the X/Z displacement from the source point, preserving its elevation and any existing offset from the world grid. For an anchor drag with Ctrl held, the heading stays on its 5° increment and the distance along that heading snaps to the selected meter increment. Switching the snap toggle off restores free placement. Changing the step with **[ / ]** during an active extension updates its preview. Placed objects and imported terrain chunks use the same gizmos. During a rotation ring or a top-down heading-handle drag, hold **Ctrl** to snap the change from the starting orientation to the nearest **5°**. Release Ctrl to resume free rotation. A small label beside the cursor shows the applied rotation in degrees, translation along a gizmo axis in meters, or the X and Z displacement for a top-down move. **Esc** cancels the drag. Typed Transform values keep their entered precision.

Use **,** to move sky time 30 minutes earlier and **.** to move it 30 minutes later.

The handles use three instances of `editor/arrow.glb` or `editor/rotation_gizmo.glb`.

In the top-down road view, hold the **middle mouse button** and drag to measure straight-line distance in the XZ plane. The ruler shows meters and disappears when you release the button. You can measure while laying out a draft.

To extend either end of a road, select its endpoint alone, hold **Shift**, and drag a translation arrow. In the top-down road view, hold **Shift** and drag the endpoint itself. The original point stays in place and the new endpoint stays selected, ready for the next segment. Top-down extensions keep the endpoint's elevation. Release to save, or press **Esc** during the drag to cancel.

### Overpasses (first milestone)

Raise the upper road or railroad to the desired height, then select the adjoining **segments** that
will form the span. Right-click their cyan center spline (or pavement), then choose
**Create → Overpass**. Its end control points define the abutments; keep those points
beyond all of the lower roads, railroad embankments (including transition strips),
and river or lake banks. Use Ctrl+Q to add control points if necessary.
A span must cross a road, railroad, river or lake and have room beneath its 0.6 m default deck.
The command never raises either spline automatically.

For a highway over a river, railroad and road together, select the entire continuous
run of highway segments covering all three and create **one** overpass. Every crossed
element is detected automatically; the abutments are at the two outer ends, with no
internal walls between the crossings. Water-only bridges use the same command.

The road keeps its spline, lane/shoulder profiles, paint and pavement material. With
bridge segments selected, the inspector's **Deck thickness (m)** field accepts 0.1–10 m.
Leave enough clearance for vehicles when changing thickness or road elevations.
**Create → Remove Overpass** restores ordinary road segments. A single clicked bridge
segment removes its contiguous span; a multi-segment selection limits the change to
those segments. Creation, removal and thickness edits support undo/redo.

Concrete sides, an underside and end abutments are generated from the road geometry.
Terrain fill uses the approach rims and ground seams beneath the span, instead of
connecting ground to its elevated deck. The bridge owns a subdivided ground patch
underneath, excluding lower-road footprints and water channels; adjoining F-created terrain chunks share
that boundary without overlapping the underpass patch. Changes to connected lower
roads, committed river/lake banks, bridge shape, lanes or shoulders rebuild the affected geometry. Splits and
subdivision preserve the span and its dependencies. Matching split bridge
ends omit internal abutments; moving them apart restores the exposed end walls.

The ground meets the exact committed shoreline beneath the deck. Water surfaces and
their existing terrain bottoms remain unchanged; no ground sheet is placed over the
channel. Finish water edit mode before creating its bridge. Later bank edits rebuild
the bridge ground when water edit mode is finished, not while dragging control points.
River/lake clearance is measured above the water surface using the same minimum 1 m
of clear space below the deck; choose a greater height if needed. To add water to an
older span, remove its Overpass setting and recreate the full span with the water present.

This milestone supports roads and railroads over multiple roads, railroads and water bodies. Separate a
proposed span from ramps, intersections, at-grade railroad crossings and dead ends
before converting it. Stacked bridges, piers, barriers, decorative
details and material controls for the concrete
and automatic ground patch are future work. That patch currently uses the default
ground color; surrounding terrain retains its own material. Saved worlds store span
settings and stable road/water IDs, never generated bridge meshes. `ROAD` version 25
adds the water dependencies; older worlds remain readable. Recook the world after
changing these spans to update streamed geometry and collision in the game.

For road-over-railroad spans, the creation check measures clearance from the generated
rail heads, including embankment height, ties, rails and banking, rather than from the
spline at the toe. It retains the existing 1 m minimum-clear-space check; choose the
actual deck elevation to leave room for the trains you intend to use. The tracks,
ties, embankment and transition strips retain their existing geometry and materials.
Ground beneath the bridge meets the outer embankment rim and excludes the railway
footprint. Spline and embankment edits, railroad splits, saving and undo/redo use the
same dependencies as road-over-road spans. A single span can cross both roads and
railroads.

For a railroad over a road, select the railroad segments and use the same
**Create → Overpass** command. Rails, ties, gauge, ballast and transition strips keep
their existing shape and stationing. Concrete supports the full embankment profile,
with sides, an underside and abutments at the span ends. Clearance is checked from
the authored railway spline at the embankment toe (the deck's lowest unbanked edge),
not from the higher rail heads, with an allowance for the lower edge when banked.
Raise that spline enough to leave room beneath the
embankment and deck. Adjust **Deck thickness (m)** in the railroad inspector with the
bridge segments selected. The rails and ties still use the project detail-distance
setting; the bridge and embankment remain visible beyond that distance.

Railway bridge ground seams, adjacent terrain fills, dependency updates, splitting,
removal, saving and undo/redo use the shared road bridge system. Split off any
at-grade road/rail crossing before converting a railway span. `ROAD` version 23
permits bridge records on railroads, using the same fields as version 22; older
ordinary roads and railroads still load.

### Railroad single switches

Right-click a railroad spline or control point and open **Create → Left Railroad Switch** or **Right Railroad Switch**. Left and right are viewed toward the railroad's last control point; the **(Reverse)** choices face toward its first point. A single switch adds one branch to the existing through track. The branch starts about 50 m long, with an 8 m lateral offset and a tangent-matched Bézier curve. Its rails retain the 1.435 m standard gauge. The turnout shares ballast and extended sleepers with the main track.

Select the branch and move, subdivide, or Shift-drag its free control points as with other railroads. Moving the free point changes the turnout's length and angle. The joined toe follows its host's position, tangent, and rail-head height; move that host control point to relocate the switch. A switch is physically bidirectional, so the same turnout works for converging and diverging tracks. Select free endpoints on two railroads with Shift and press **F** to join them. Road-to-rail endpoint joins remain disallowed.

Switches inherit the host's embankment and material settings and outliner folder. Their stable connections survive save/load, undo/redo, subdivision, and railroad splits. Deleting the host detaches the branch; deleting the branch restores the ordinary main track. Select the joined railroads around a closed area and press **F** to fill terrain. Cooked worlds include the turnout meshes, collision, shared sleepers, and streaming dependencies. This adds fixed turnout geometry and connectivity; animated switch blades and train route control are not implemented yet.
