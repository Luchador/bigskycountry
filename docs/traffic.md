# Traffic foundation — milestone 1

## Trying it

1. Rebuild the game and editor, then use **File → Cook World** in the editor.
2. Enable **View → Traffic Lanes** to inspect directions in either editor view.
3. In-game, press **F8**, aim the crosshair at a through lane, and press **F6**.
   Choose a loaded road within 250 m of the player, with at least 15 m left before
   the lane end and clear space for the vehicle. F1's free camera also works.
4. Follow the car. It accelerates, steers, slows for curves, and stops before the
   lane end. F6 replaces it at another valid placement; F7 removes it.

Green arrows run in the road's authored direction; blue arrows run against it.
The test car's lane is amber. Red bars mark path ends. The game overlay displays
target speed, remaining lane length, and the current driving state. Debug paths
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
Versions 2–4 still load, but have no traffic data: recook to run the test car.
Authored `.wld` syntax is unchanged. The editor caches its debug network until a
road revision changes; camera movement does not regenerate lanes.

## Controllers and physics

`PawnController` owns possession and sanitized commands. `PlayerController`
retains the existing player input/camera behavior; `TrafficDriver` issues steering,
throttle, and the new independent `PawnInput::brake` service-brake value. Reverse
and service braking are separate. Player W/S braking/reverse and the handbrake
keep their existing behavior.

Controllers update at the game's fixed 60 Hz simulation step. Cars still use the
existing rigid-body and suspension substeps. The initial driver uses a moving
look-ahead target for steering, curvature-based speed planning, and braking
distance to the lane end. It uses the same physical car as the player, rather
than setting the car's position along the spline.

Collision is prefetched along the next 120 m of the test lane as well as around
the car. The driver brakes before unavailable collision. If support becomes
unavailable underneath it, only the AI car is suspended; the player's existing
collision guard remains independent. Basic forward sphere checks stop for a
stationary obstacle. A car pushed off its lane brakes and stays there; reposition
it with F6 rather than expecting autonomous recovery.

## Boundaries of this milestone

There is one manually placed AI car, with no automatic traffic population. It
cannot be entered while AI-possessed. Paths stop at road ends and intersection
mouths, including roads connected to another road. Lane-to-lane links, turning
movements, right-of-way, signals, route choice, merging, and following moving
traffic are not implemented here. The obstacle check is a test safeguard, not
predictive collision avoidance or a guarantee against a player hitting the car.

The next milestone can add traffic following and a nearby-vehicle spatial index
on this foundation, followed by junction movement paths and traffic controls.
