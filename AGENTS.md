# DeterministicPhysics2D

This is a .NET 8 rigid-body engine with a Raylib-cs 8.1.0 sandbox. Run
`dotnet run` for graphics, `dotnet run -c Release -- --self-test` for headless
regression checks. Raylib is only for input, windowing, and primitive rendering.
Do not add external physics libraries or use System.Numerics vectors. Render
using scalar Raylib overloads; all engine geometry uses our Vec2 and Mat22.

## Architecture

- Math2D.cs: immutable Vec2, Mat22, AABB value types.
- RigidBody.cs: convex geometry, mass properties, forces and state. Polygon
  vertices are counterclockwise in algebraic coordinates; outward face normals
  are (edge.Y, -edge.X). Geometry is recentered on its area centroid.
- RevoluteJoint.cs: warm-started shared-anchor hinges, angular limits, effective
  mass solving and bounded position correction. Constraints run in creation
  order alongside the eight contact iterations; clear all joints on reset.
- CableJoint.cs: warm-started tension-only maximum-distance constraints with
  local attachment points, angular effective mass and predictive slack limits.
  Null endpoints are fixed world points. Never push endpoints apart when slack.
  Solve cables after hinges in each of eight velocity/position iterations.
  PhysicsWorld stores up to 512 cables and preserves solve order when cutting.
  Cable sag is visual only; these are massless endpoint constraints, without
  wrapping or collisions along the span.
- RagdollFactory.cs: atomic-capacity spawning of 11 bodies / 10 hinges. Each
  ragdoll has a unique nonzero CollisionGroup; equal nonzero groups suppress
  self-collision in both discrete and swept broad-phase. Different ragdolls
  and ordinary bodies still collide. RagdollPart controls presentation only.
- BroadPhaseGrid.cs: preallocated spatial hash buckets and cell entries, exact
  cell-coordinate checks for hash collisions, duplicate suppression and sorted
  unique body pairs. Entries are sorted by cell and ID for diagnostics after
  pair discovery. Never depend on hash-map iteration order. Static-static pairs
  are excluded.
- CollisionDetection.cs: circle tests, SAT, reference/incident edge clipping.
  Immutable ContactManifold has at most two points and a normal from A to B.
- ContinuousCollision.cs: swept AABBs, convex feature distances and conservative
  time-of-impact advancement with linear and angular motion bounds. Resolve
  the earliest event globally; sorted body IDs break equal-time ties. TOI
  impulses are transient and do not mutate persistent warm-start caches.
- ImpulseSolver.cs: sequential accumulated normal/friction impulses, angular
  effective mass, warm-start cache indexed by stable body pair, two-point normal
  block solve, eight iterations and Baumgarte position correction. Clear the
  cache on reset. Match warm-start contacts using local anchors and normals.
- PhysicsWorld.cs: stable ID ownership, fixed-step orchestration, drag spring,
  CCD event integration, canonical little-endian SHA-256 position/angle hashing
  and a separate kinematic hash including linear/angular velocities.
- SandboxSession.cs: preallocated tick command log, pause/single-step, JSON
  recording persistence and exact endpoint replay verification. Its monotonic
  TimelineTick survives scene resets; PhysicsWorld.Tick resets with the scene.
- Program.cs: seeded scene, scalar rendering, interaction and headless tests.

## Simulation contracts

Use fixed dt = 1/60 second. Clamp each wall-clock frame to 0.25 seconds and
consume its accumulator in fixed steps. Apply forces, solve velocities, integrate
positions with semi-implicit Euler and CCD event integration, then correct
penetrations. Input is sampled per rendered frame and queued in order before
the next fixed tick. Replay consumes those commands at the same timeline tick,
including resets, and ignores live simulation input. Reset uses new Random(42), stable
creation order and zero accumulator. The frame-600 hash means simulation tick
600, not display frame 600. Hash reporting is outside the simulation hot path.

The same seed, tick-indexed input, executable, runtime and CPU architecture must
produce bit-identical states. Floating point and System.Random do not promise
identical results across arbitrary architectures or runtime versions.

Do not allocate in Advance, Step, broad/narrow phase, CCD, ordinary input recording
or solving. Buffers are allocated once; explicit spawn/reset/cable-creation commands, tick-600
hash reporting, JSON persistence and display text are cold paths that may allocate.
The session clock uses the same fixed dt and 0.25s clamp as PhysicsWorld.Advance;
it owns its accumulator so scene resets cannot corrupt pending frame time.
Paused frames add no accumulator time; single-step consumes exactly one tick.
MaxBodies=512 is intentional: sandbox spawning stops at capacity. Joint
capacity is MaxJoints=512. A ragdoll spawn must reserve capacity for
the complete body/joint assembly before mutation. B queues SpawnRagdoll; append
new InputKind values to preserve existing recording enum IDs. Ragdolls are
optional additions, preserving the initial scene. Buffer
overflow must fail explicitly, never silently drop collisions. Geometry is
convex, finite, nondegenerate and bounded by the documented grid capacity.
CableAnchor, CancelCable and CutCable commands share the tick log. Pick bodies
in reverse creation order; empty space selects a fixed endpoint. Preserve a
first endpoint in body-local coordinates as that body moves. At least one end
must be dynamic; reject same-body attachments and endpoints closer than four
pixels in the UI. Set maximum length from the endpoint distance at attachment.
Clear cables and pending endpoint selection on reset/new recording/replay.
C toggles cable mode; two left clicks attach, right-click cancels, X cuts the
nearest displayed curve within 14 pixels. Cutting uses the same 20-segment
visual curve as rendering, including sag, with creation order breaking ties.
CCD skips slow pairs and existing support contacts (handled by the discrete
solver). Broad/narrow phase use a 0.003-unit support shell so TOI-stopped bodies
transfer weight through persistent two-point manifolds rather than freezing at
a positive gap. Recover clipped manifolds at TOI and block-solve both normal
contacts. Drag springs scale with ragdoll assembly mass and use a bounded
implicit response to lift connected limbs without destabilizing a light head.
Use Tolerance=0.002, at most 96 conservative advances per cast and 32 impacts per
tick. On unresolved casts/event exhaustion, clamp the remaining motion and expose
CcdMotionClamped; never silently integrate past an unresolved potential hit.
Rotational grazing at zero separation can use this conservative fallback.
Recordings are seed-42 sessions of this executable/runtime, not arbitrary world
snapshots. Validate replay JSON before replacing the active recording. Preserve
insertion order for commands with the same tick. At playback completion compare
both hashes and pause exactly at the recorded endpoint.

After engine edits run the Release headless checks and build. Preserve tests
for contact orientation, containment, stacking, ragdoll anchors/angular limits,
gravity/drag response, cable slack/tension, dynamic endpoint momentum, off-center
cable torque, suspended ragdolls, cutting, atomic capacity, replay through resets,
JSON round-trips, pause/single-step, accumulator behavior, thin-wall/angular CCD,
multiple impacts per tick and allocation-free stepping. Do not introduce variable-dt integration,
unordered solver traversal, or physics through Raylib.
