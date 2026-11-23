# Traffic following — milestone 2

## Trying it

1. Rebuild the game and editor. Keep your milestone-1 cook; use **File → Cook
   World** only if you have not cooked traffic lanes yet.
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
The last spawned car's lane is amber. Red bars mark path ends. The game overlay displays
target speed, bumper gap, desired following gap, and the current driving state
for the last spawned car. It also reports nearby queries/results and car-pair
collision checks from the current physics tick. Debug paths
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

Cooked format **v5** stores the directed paths, road IDs, lane ordinals, widths,
and initial speed limits in the manifest. The reader validates ownership,
identity, finite coordinates, dimensions, and nonzero segments. Runtime traffic
uses this resident metadata without loading/rebuilding authored roads. It is
additional manifest memory, outside the geometry/asset residency budgets.
Versions 2–4 still load, but have no traffic data: recook to run traffic.
Milestone 2 uses the same v5 data; a milestone-1 cook does not need recooking.
Authored `.wld` syntax is unchanged. The editor caches its debug network until a
road revision changes; camera movement does not regenerate lanes.

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
fallback. Static geometry continues using the existing collision BVHs.

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
is no autonomous recovery or despawning yet. F7 clears the test population.

## Boundaries of this milestone

Traffic is manually placed, capped at 64 AI cars, and does not persist between
sessions. AI-possessed cars cannot be entered. Paths still stop at road ends and
intersection mouths, including roads connected to another road. Lane-to-lane
links, turning movements, right-of-way, signals, route choice, merging, and
ambient population management are not implemented here. This milestone adds
vehicle following and avoidance; it is not a complete traffic-law simulation.
