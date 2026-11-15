# Deterministic Physics 2D

A from-scratch .NET 8 rigid-body engine and 1280x720 Raylib-cs 8.1.0 sandbox.

```powershell
dotnet run
dotnet run -c Release -- --self-test
```

This workspace includes an ignored local NuGet.Config and package cache to work
around this machine's Windows TLS credential error. Remove that config to use
your usual NuGet sources. Fresh checkouts restore Raylib-cs from NuGet normally.

Left-click below the toolbar to spawn a circle or regular convex polygon.
Right-click and drag to pull a body with a damped spring attached at the grabbed
point. **G** shows grid occupancy and AABBs, **N** shows contact points and
normals, **V** shows velocity vectors, **R** resets the seeded scene, **Esc** exits.
Spawning stops at 512 bodies. The initial scene has a floor, walls, two ramps and
a ten-layer pyramid of 55 boxes, using `new Random(42)`.

| Key | Action |
| --- | --- |
| B | Spawn a ragdoll at the cursor (below the toolbar) |
| C | Toggle cable mode; left-click two attachment points |
| X | Cut the nearest cable under the cursor |
| Space | Pause or resume |
| . (period) | Advance exactly one tick while paused |
| F5 | Start a fresh seeded recording |
| F6 | Replay the current recording from the beginning |
| F7 | Save the recording to `replay.json` in the working directory |
| F8 | Load `replay.json` and start playback |

Ragdolls have a head, chest, pelvis, and two-piece arms and legs: 11 rigid bodies
connected by 10 warm-started hinges with neck, spine, shoulder, elbow, hip and
knee limits. Right-drag any part to lift or swing the entire ragdoll. Joint
anchors appear as small teal dots. Each ragdoll ignores its own body parts for
collision stability, while colliding with the environment and other ragdolls.
Spawning requires 11 free body slots and 10 free joint slots and is recorded
for replay. The initial pyramid scene stays unchanged; press B to add ragdolls.

To tie objects, press **C**, click a point on the first object, then a point on
the second. Empty space becomes a fixed world anchor, and static floors/ramps
can also hold one end. At least one endpoint must be on a movable object.
Attachment points stay fixed to each body's local shape as it moves or turns.
Right-click cancels an unfinished cable. Press C again to return to spawning
and dragging, or **X** near a cable to cut it. Tying and cutting are recorded
and replayed. While paused, queued clicks apply on the next step or resume.

The attachment distance sets the cable's maximum length. Cables go slack when
the endpoints move closer and apply tension when pulled apart; off-center
attachments can rotate objects. Slack spans sag visually, taut spans turn gold,
and endpoint dots show the knots. You can hang ragdolls, tie bodies together,
or connect multiple cables. There are 512 cable slots. Cables are massless
endpoint constraints: their spans do not collide with or wrap around scenery.

Recording starts automatically. Shape/ragdoll spawning, cable attachment/cutting,
grab, spring target, release and reset
commands are stored in order at fixed simulation ticks. Input queued while
paused applies on the next single-step or resumed tick. Scene reset restarts
the world's tick counter but preserves the recording's monotonic timeline.
Playback ignores live simulation inputs, verifies position and velocity hashes,
and pauses at its exact endpoint; F6 repeats it and F5 returns to recording.
Save/load happens outside the simulation loop. The bounded log stores 65,536
commands and displays a message when full. Unexecuted pending commands are
excluded from saved recordings. Replay files are intended for the same engine
build, runtime and architecture, with the default seed-42 scene.

The console logs the full SHA-256 at simulation tick 600 on launch and after each
reset. Leave the scene untouched for repeatable baseline hashes. Frame grouping
does not change the result; interactive inputs must be replayed at identical
simulation ticks. Bitwise determinism is scoped to the same executable, runtime
and CPU architecture. The hash serializes every body's X, Y and angle in stable
ID order as little-endian IEEE-754 floats. A separate kinematic hash also
includes linear and angular velocities for replay verification; neither hash
is an arbitrary-world snapshot or a serialization of the solver cache.

The simulation uses preallocated buffers, a sorted uniform spatial grid, SAT
and segment clipping, warm-started angular impulses with two-contact normal
block solving, Coulomb friction, eight iterations, and 0.3 / 0.01 positional
stabilization. Fast pairs use swept grid bounds and conservative advancement
over circle/convex-polygon feature distances, including angular motion bounds.
The earliest impact is resolved before integrating the remaining fixed-step
time, allowing multiple bounces in one tick. CCD is enabled by default and can
be disabled through `PhysicsWorld.ContinuousDetection` for comparisons.
The headless checks cover
geometry, exhaustive broad-phase comparison, restitution, angular response,
friction, stacking, replay/save/load, pause/single-step, fast thin-wall impacts,
moving-body impacts, rotating geometry, ricochets, near misses, ragdoll falls,
dragging, hinge separation/angular limits, cable slack/tension and suspension,
off-center cable torque, cutting, spawn capacity and measured
allocations. Raylib only handles the
window, input and primitive rendering; there are no external physics libraries
or System.Numerics vectors. See AGENTS.md for implementation contracts.

CCD uses a 0.002-unit contact tolerance, up to 96 conservative advances per cast
and 32 impacts per tick. Persistent contacts include a 0.003-unit separation
shell, preserving weight support after a TOI stop. Existing support contacts
use the discrete solver. If a cast
cannot converge or the event budget is exhausted, the remaining world motion
is conservatively clamped for that tick; the HUD displays `MOTION CLAMPED`.
Rotating features grazing an existing contact can trigger that fallback.
Sleeping is not implemented. Explicit spawning, scene reset, hash reporting,
file operations and display text can allocate; ordinary stepping, CCD and drag
input recording do not.
