# Traffic highways and automatic population

## Performance testing

The game shows FPS at the top right, averaged over at least 0.25 seconds of
complete rendered frames (including vsync or the frame limiter).

The temporary player-car detection switch has been removed. NPCs always perceive
player vehicles, parked vehicles, and on-foot players. Use **Release** builds for
performance testing; Debug's additional CPU cost can dominate both physics and
scene submission, especially at high speeds and with several physics substeps.

```bat
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --config Release --target BigSkyCountry BigSkyCountryEditor
```

Press **F3** for a frame profile at the upper right. It reports main-thread
milliseconds per rendered frame, averaged over at least half a second: streaming
and collision preparation, controllers (including junctions and pawn-index sync),
physics, other simulation, scene submission, traffic debug drawing, HUD/input,
presentation/vsync, and the fallback frame limiter. These are CPU wall times, not
GPU timestamp measurements; renderer calls can include waiting for the GPU.
It also reports simulation ticks per frame, peak substeps per tick, pawn simulation
calls per frame, and the pawn whose motion requested that peak substep count.
The build label comes from CMake's selected configuration. `Debug` or
`Unspecified` may be unoptimized; use an optimized build for performance comparisons.

To investigate a slowdown, capture F3 once at low speed and once while it is slow.
F8 independently toggles traffic lane/debug drawing; F3 does not change traffic
or physics behavior. Profiling timers are disabled when F3 is hidden.

## Trying it

1. Rebuild the game and editor in **Release**. Automatic population works with the
   existing milestone 4 cook; no additional recook is needed for this update.
   If upgrading an older world, save and use **File → Cook World** to add traffic
   lanes, intersections, ramps and road-end connections.
   Select each intersection anchor to set **Traffic control: Stop/Yield/Priority**.
   Existing intersections default to Stop on every approach. Controls and stop-line
   paint are independent; both save and support undo/redo.
2. Enable **View → Traffic Lanes** to inspect directions in either editor view.
3. Traffic now appears automatically around the player. For an isolated manual
   test, press **F7** to clear traffic and pause automatic spawning. Press **F8**,
   aim the crosshair at a through lane, and press **F6**.
   Choose a loaded road within 250 m of the player, with at least 15 m left before
   the lane end and clear space for the vehicle. F1's free camera also works.
4. Press F6 again on another clear part of the lane to add a car without replacing
   existing traffic. Shift+F6 adds up to five cars, starting at the aimed point and
   working backwards at 18 m intervals. The group can be partial if it reaches a
   lane boundary, unsafe gap, or the 64-car test limit; the overlay explains why.
5. Watch the cars form queues and resume when a blocker moves away. F7 clears all
   AI traffic and pauses automatic spawning, preserving the player and their
   vehicle. **Shift+F7** toggles automatic spawning without removing existing cars.

Green arrows run in the road's authored direction; blue arrows run against it.
The last spawned car's lane and available turns are amber. Purple paths connect
intersection lanes; entry bars are red (Stop), yellow (Yield), or green (Priority). The game overlay displays
target speed, bumper gap, desired following gap, and the current driving state
for the last spawned car. It also reports nearby queries/results and car-pair
collision checks from the current physics tick, plus active junction reservations. Debug paths
are drawn over the scene for inspection, including through occluding geometry.

The initial maximum speed is **30 mph** (`TrafficLane::speedLimit`, stored in
engine meters per second). Driving distances use existing game meters; there is
no additional geographic 1/6 conversion in the traffic system.

## Lane geometry and cooking

`Road::trafficPaths` is derived during the normal pavement rebuild from its
cross-sections. It follows curves, banking, skewed intersection mouths, shoulders'
independent offsets, and changing lane/division profiles. Railroad crossings
apply their approach/slab height to the paths too. Overpasses retain the upper
road's elevation; there is no connection to a road underneath.

Through lanes follow right-hand traffic. Center turn lanes and shoulders are not
through lanes. One-way roads (including ramps) support one or more lanes. Shared
two-way single-lane roads are excluded until passing/yielding behavior exists.
New or disappearing lanes are limited to their full-width portions. Drivers merge
before a disappearing lane's full-width portion ends, and can use newly added
lanes after they are wide enough. Through-lane paths continue smoothly through
divided/undivided and center-turn-lane/median conversions. Center turn lanes and
medians never become lane-change destinations.

`TrafficNetwork::Build` orders each path in its driving direction and derives arc
lengths and bounds. IDs combine the stable road ID with a local path number and
are valid within a cooked network revision. Editing a road's topology can change
its local path IDs, so future persisted traffic must also identify the network
revision. This milestone does not persist NPC state.

Cooked format **v7** stores the directed paths, road IDs, lane ordinals, widths,
and initial speed limits in the manifest. The reader validates ownership,
identity, finite coordinates, dimensions, and nonzero segments. Runtime traffic
uses this resident metadata without loading/rebuilding authored roads. It is
additional manifest memory, outside the geometry/asset residency budgets.
Version 6 adds stable movement IDs (junction ID/local path ID), source/destination
lane IDs, approach IDs, controls, turn type, speed caps, and sampled turn paths.
Conflict lists and lookup indices are derived once on load, not each frame.
Version 7 adds directed road-end and ramp links, source/destination lane stations,
connection kind, and sampled taper paths. Older packages still load; v5 has lanes
and v6 adds intersections, but neither contains highway connections. Recook to
enable the complete milestone. Authored `INTS` v10
stores the twelve approach controls; older intersections default to Stop.
The editor caches its overlay until a road revision or control changes.

## Turns and right-of-way

Only explicitly attached intersection approaches are connected. Geometric
crossings and overpasses never create an implicit traffic connection. Paths join
actual lane mouths with tangent-continuous cubic curves. Straight movements
preserve lane order, left turns use the innermost lane, and right turns use the
outermost lane. U-turns inside intersections are excluded; approach lane changes finish before
entering a junction. Each curve is
checked against the junction pavement with lateral clearance and a curvature
limit. An unsafe path is omitted rather than crossing an island or a curb.
Standard, skewed, aligned, and supported Y-intersection turns use these rules;
full town-square circulation requires a later circulation-path implementation.

`TrafficDriver` chooses a deterministic, varied outgoing movement for each visit.
Its lookahead includes the current lane, turn, and outgoing lane, so steering
and following continue across both seams. Turn speeds are capped at 7 m/s
(about 16 mph), with additional braking for sharper curves. The initial routes
are local choices, not destination-based navigation.

`TrafficJunctions` collects all requests on the synchronized pre-physics snapshot:

- **Stop:** the front bumper waits before the mouth and speed stays below 0.2 m/s
  for 0.6 seconds. The earliest stopped arrival goes first. Arrivals within the
  same 0.2-second bucket yield to the vehicle on their right; opposing left turns
  yield to through/right turns. A stable handle breaks a full simultaneous cycle.
- **Yield:** a clear approach can continue without a mandatory stop. It yields to
  approaching Priority traffic and occupied/conflicting paths.
- **Priority:** no mandatory stop, but it still yields to occupied paths, blocked
  exits, player obstructions, and oncoming through traffic when turning left.

Swept turn corridors are precomputed into a conflict table. Shared outgoing
lanes always conflict. Compatible paths can reserve together; incompatible
paths cannot. A reservation stays until the car's rear clears the outgoing mouth.
Before entry, permission is rechecked and can be withdrawn if space disappears.
Removal/F7 clears stale requests. Cars physically entering unexpectedly also
block conflicting requests, so another car is not invited into them.

Admission requires room for the whole car and a standing gap beyond the junction,
loaded collision through the exit, and no nearby static obstruction. Exit checks
use current vehicle footprints, not predicted queue motion. This avoids entering
just because a queue might move. The player and unmanaged vehicles are checked
against the intended path using current and predicted positions. AI perception
continues braking while driving through; reservations cannot prevent a deliberate
player impact or a last-moment intrusion inside braking distance.

## Controllers and physics

`PawnController` owns possession and sanitized commands. `PlayerController`
retains the existing player input/camera behavior; `TrafficDriver` issues steering,
throttle, and the independent `PawnInput::brake` service-brake value. Reverse
and service braking are separate. Player W/S braking/reverse and the handbrake
keep their existing behavior.

Controllers update at the game's fixed 60 Hz simulation step. Cars still use the
existing rigid-body and suspension substeps. The initial driver uses a moving
look-ahead target for steering, curvature-based speed planning, and braking
distance to the lane end. It uses the same physical car as the player, rather
than setting the car's position along the spline.

## Queues, player avoidance, and spatial queries

The follower measures clearance between vehicle footprints along its lane. The
normal desired gap is **3 m plus 1.6 seconds of the follower's speed**. It chooses
a target speed from the leader's speed, gap error, and braking distance. Normal
acceleration is capped at 2.5 m/s²; a dangerously short closing gap requests full
service braking. Stopped queues hold their brakes and restart when space opens.
Tune these defaults in `TrafficFollowing` in `shared/traffic_driver.hpp`.

Dynamic perception uses current footprints and up to 2.5 seconds of predicted
motion. Other AI cars use their lane paths for prediction; the player uses their
current velocity. An on-foot player, stopped player car, or vehicle cutting across
the lane can make traffic yield. Height and lateral-footprint checks reject
separate bridge decks, adjacent lanes, and ordinary opposing lanes. Seated
characters and pawns with collision disabled are excluded. The system cannot
prevent a player from deliberately hitting traffic or appearing inside a braking
distance; vehicle physics still resolves contacts.

`PawnIndex` is a persistent 32 m XZ hash grid, with exact 3D bounds filtering.
Entries cover current bounds and predicted motion. Query results are deduplicated
and sorted by handle. Membership updates when pawns move, spawn, disappear, seat,
teleport, or change collision state. Oversized entries/queries have a bounded
fallback. Static geometry uses per-chunk triangle BVHs behind a spatial tree of
loaded chunk bounds. Wheel rays and body contacts skip unrelated chunks,
including ones kept resident by traffic elsewhere. Attachment changes invalidate
the chunk tree; the next query rebuilds it once for all cars and substeps.

All controllers observe pre-physics poses in each fixed tick. During substeps the
index follows moved pawns and contact corrections. Car pairs are checked once per
substep using nearby candidates, rather than comparing every pair in the world.
Character fitting and car/character contacts also use filtered candidates.
Handle lookup is constant-time. Dense overlapping traffic can still require many
checks, and every active car still runs suspension and rigid-body physics.

Collision is prefetched along the next 120 m of every AI lane as well as around
cars. Nearby preload requests from a queue are coalesced; exact readiness checks
remain per car. Drivers brake before unavailable collision. If support becomes
unavailable underneath one, that AI car is suspended independently of the player
and treated as stopped by other traffic. Cars pushed off their lane brake; there
is no autonomous rerouting at a dead end. Cars that fall more than 100 m below
their last support, with no landing surface below them in either loaded collision
or unloaded-world metadata, are removed along with their intersection requests.
This prevents lost cars from accumulating fall speed and increasing physics work
indefinitely. Ordinary queues, dead-end stops, and falls toward lower terrain are
preserved. F7 clears the test population.

## Highway behavior

- **Lane changes:** cars can pass a slower leader on the left and return right
  when a safe gap exists. Adjacent lanes must belong to the same road and travel
  direction. A smooth lateral blend follows the actual road curves and grade.
  Decisions are staggered and separated by a cooldown to avoid oscillation.
- **Lane reductions:** drivers look ahead and move into a surviving neighboring
  lane before the narrowing begins, making successive changes when several lanes
  disappear. If blocked, they stop with room to resume the merge when a gap opens.
  They never drive across a median or shoulder to
  invent a continuation.
- **Exits and entrances:** cooked links follow the actual pavement taper guides,
  including both sides of a divided highway and shoulders. Entrance traffic yields
  before the taper until there is room in the mainline. It checks front/rear gaps,
  closing speed, predicted motion and other accepted maneuver paths. Mainline
  traffic continues using normal following and collision avoidance.
- **Route preparation:** cars choose a local exit/turn ahead of time, then move
  one lane at a time into a legal approach lane. Approximately one in four test
  cars chooses an available exit (stable handle/visit choices); the others stay
  on the mainline. If an exit is too close to reach safely, they miss it. If a
  junction approach cannot be reached, they take a legal movement from their
  current lane, or stop if none exists. These are local routes, not navigation to
  a user-selected destination.
- **Road seams:** explicit endpoint sockets and exactly matching, aligned lane
  ends connect roads, including roads made with Split Road. Unrelated nearby
  roads, grade-separated crossings and intersections are not proximity-linked.
- **Streaming:** the chosen route and candidate adjacent lanes prefetch collision.
  A maneuver only begins with loaded, unobstructed collision along its path.
  The original lane and target lane are both considered during a transition.

F8 shows the last spawned car's planned path in pale yellow, ramp/road connections
in purple, and entrance merges in orange. The state readout reports route
preparation, changing lanes, taking an exit, or yielding for a merge. It also
reports whether automatic spawning is enabled or waiting for collision.

Try a four-lane divided highway with an exit, an entrance, and an outer lane
reduction. Place cars in both lanes and on the entrance. Block the receiving lane
with the player's car to verify waiting, then clear it. Try a 5-lane undivided to
4-lane divided conversion: the two through lanes per direction continue across
the changing center profile. Test with F3 showing **Build: Release**.

## Automatic population

Traffic is enabled on game startup. Defaults live in `shared/traffic_population.hpp`:

| Setting | Default |
| --- | --- |
| Desired NPC population | 24 cars |
| Spawn distance from the player | 150–600 m |
| Minimum distance for spawns in view | 450 m |
| Despawn distance from the player | Over 1,000 m |
| Maximum spawn attempt rate | 2 per second |
| Pending collision request timeout | 5 seconds |

Each eligible road gets one equal chance in the spawn lottery. Longer roads and
roads with more lanes do not get extra chances. After choosing a road, the system
chooses a location along its eligible lane stretches. There are no road-class,
speed-limit, city or highway density weights. Obstructed roads can still produce
fewer successful spawns. A target is a maximum desired population, not a promise
to place cars when no safe sites are available.

Sites must be outside the player's near radius and away from lane ends (8 m from
the start, with 40 m of lane remaining). Within 450 m of the camera, sites must
also be behind it. Farther sites can be in view so a fast-moving player still
encounters traffic ahead on straight roads; distant cars may appear at that range.
The population stays centered on
the controlled player pawn, including when walking or using F1's free camera.
Distances are 3D engine meters, independent of the render-distance setting.

The game indexes short lane stretches once, searches only nearby stretches when
below the target, and handles at most one pending spawn. Pending locations request
collision through the existing asynchronous streamer. A car is added only after
support, collision readiness, physical clearance and approaching-vehicle gaps are
checked. Moving or turning away invalidates a pending site. The target includes
manually spawned NPCs; manual placement can still raise the total to 64.

All traffic-owned cars, including manual ones, are retired beyond the outer
radius. Their controllers, collision bodies and junction reservations are removed,
so they no longer pin distant collision chunks. Player vehicles and other
unmanaged parked vehicles are never removed by this system. The wider despawn
radius prevents cars from repeatedly appearing and disappearing at one boundary.
Nearby queues and stopped dead-end traffic remain valid and are not culled merely
for stopping. Disabling automatic spawning doesn't disable distance cleanup.

## Current boundaries

Traffic is capped at 64 AI cars and does not persist between
sessions. AI-possessed cars cannot be entered. The initial speed limit remains
30 mph; this milestone adds highway maneuvering, not an inferred highway speed
limit. There are no traffic lights, destination routing,
roundabout/town-square circulation, or routing through curved side connectors yet.
Very short tapers or lanes without sufficient room for a safe lane change may
still stop traffic. A saturated mainline can delay merging traffic; there is no
forced timeout that makes an unsafe gap acceptable. Prediction cannot prevent a
player from deliberately hitting traffic or suddenly appearing inside braking
distance. Cars retain physical collision response throughout maneuvers.
