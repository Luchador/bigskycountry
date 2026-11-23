# Traffic intersections — milestone 3

## Temporary performance diagnostic

The game shows FPS at the top right, averaged over at least 0.25 seconds of
complete rendered frames (including vsync or the frame limiter).

NPCs temporarily ignore the car currently possessed by the player in following,
braking, intersection intrusion and blocked-exit checks. NPC-to-NPC avoidance,
on-foot player detection, parked-car detection and physical collisions remain
active. After the player exits a car, it is detected as a parked vehicle again.
The shared spatial index and vehicle physics are unchanged for this comparison.
Set `detectPlayerCars` to `true` in `shared/traffic_driver.cpp` to restore normal
player-car avoidance. This diagnostic only requires a rebuild, not a recook.

## Trying it

1. Rebuild the game and editor, save, and use **File → Cook World** again.
   Select each intersection anchor to set **Traffic control: Stop/Yield/Priority**.
   Existing intersections default to Stop on every approach. Controls and stop-line
   paint are independent; both save and support undo/redo.
2. Enable **View → Traffic Lanes** to inspect directions in either editor view.
3. In-game, press **F8**, aim the crosshair at a through lane, and press **F6**.
   Choose a loaded road within 250 m of the player, with at least 15 m left before
   the lane end and clear space for the vehicle. F1's free camera also works.
4. Press F6 again on another clear part of the lane to add a car without replacing
   existing traffic. Shift+F6 adds up to five cars, starting at the aimed point and
   working backwards at 18 m intervals. The group can be partial if it reaches a
   lane boundary, unsafe gap, or the 64-car test limit; the overlay explains why.
5. Watch the cars form queues and resume when a blocker moves away. F7 clears all
   AI traffic, preserving the player and their vehicle.

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
New or disappearing lanes are limited to their full-width portions. No lane
change or invented continuation is added through the narrowing part.

`TrafficNetwork::Build` orders each path in its driving direction and derives arc
lengths and bounds. IDs combine the stable road ID with a local path number and
are valid within a cooked network revision. Editing a road's topology can change
its local path IDs, so future persisted traffic must also identify the network
revision. This milestone does not persist NPC state.

Cooked format **v6** stores the directed paths, road IDs, lane ordinals, widths,
and initial speed limits in the manifest. The reader validates ownership,
identity, finite coordinates, dimensions, and nonzero segments. Runtime traffic
uses this resident metadata without loading/rebuilding authored roads. It is
additional manifest memory, outside the geometry/asset residency budgets.
Version 6 adds stable movement IDs (junction ID/local path ID), source/destination
lane IDs, approach IDs, controls, turn type, speed caps, and sampled turn paths.
Conflict lists and lookup indices are derived once on load, not each frame.
Versions 2–4 still load without traffic lanes; v5 loads with lane following but
no intersection turns. Recook to enable this milestone. Authored `INTS` v10
stores the twelve approach controls; older intersections default to Stop.
The editor caches its overlay until a road revision or control changes.

## Turns and right-of-way

Only explicitly attached intersection approaches are connected. Geometric
crossings and overpasses never create an implicit traffic connection. Paths join
actual lane mouths with tangent-continuous cubic curves. Straight movements
preserve lane order, left turns use the innermost lane, and right turns use the
outermost lane. U-turns and lane-changing merges are excluded. Each curve is
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

## Boundaries of this milestone

Traffic is manually placed, capped at 64 AI cars, and does not persist between
sessions. AI-possessed cars cannot be entered. This milestone does not add traffic
lights or physical sign meshes, lane changes, disappearing-lane merges, ramp/road
seam routing, roundabout/town-square circulation, destination navigation, or an
automatic population manager. Paths with no usable continuation still stop at
their ends. A continuously busy priority road can delay minor-road traffic; there
is no artificial timeout that lets a waiting vehicle ignore right-of-way.
