namespace DeterministicPhysics2D;

internal static class SelfTests
{
    private static void Check(bool condition, string message)
    { if (!condition) throw new InvalidOperationException(message); }
    private static void Near(float actual, float expected, float tolerance, string message) =>
        Check(MathF.Abs(actual - expected) <= tolerance, $"{message}: expected {expected}, got {actual}");
    public static int Run()
    {
        try
        {
            Geometry(); BroadPhase(); Dynamics(); ContinuousDetection(); InputReplay(); Ragdolls(); Cables(); ReplayAndAllocations();
            Console.WriteLine("PASS: geometry, grid, impulses, friction, stacking, CCD, ragdolls/hinges, cables, pause/step, input replay/save/load, fixed timestep, and zero-allocation stepping.");
            return 0;
        }
        catch (Exception error) { Console.Error.WriteLine($"FAIL: {error}"); return 1; }
    }
    private static void Geometry()
    {
        Near(Vec2.Dot(new(2, 3), new(4, 5)), 23, 0, "Dot product");
        Near(Vec2.Cross(new Vec2(2, 3), new Vec2(4, 5)), -2, 0, "Cross product");
        Vec2 rotated = new Mat22(MathF.PI / 2) * new Vec2(1, 0);
        Near(rotated.Y, 1, 1e-6f, "Rotation");
        Near(RigidBody.Circle(default, 2, 3).Inertia, 6, 1e-6f, "Circle inertia");
        Near(RigidBody.Box(default, 2, 4, 3).Inertia, 5, 1e-6f, "Box inertia");
        Vec2[] vertices = { new(-1, -2), new(1, -2), new(1, 2), new(-1, 2) };
        Near(RigidBody.Polygon(default, vertices, 3).Inertia, 5, 1e-6f, "Polygon inertia integral");
        RigidBody c1 = RigidBody.Circle(default, 2, 1), c2 = RigidBody.Circle(new(3, 0), 2, 1);
        Check(CollisionDetection.Collide(c1, c2, out ContactManifold m), "Circle contact missing");
        Near(m.Penetration, 1, 1e-6f, "Circle depth"); Near(m.Normal.X, 1, 0, "Circle normal");
        c2.Position = default;
        Check(CollisionDetection.Collide(c1, c2, out m) && float.IsFinite(m.Normal.X), "Coincident circles");
        c2.Position = new(5, 0); Check(!CollisionDetection.Collide(c1, c2, out _), "Separated circles");
        RigidBody box = RigidBody.Box(default, 4, 4, 0);
        c1.Position = new(3, 0);
        Check(CollisionDetection.Collide(c1, box, out m), "Circle-face contact");
        Near(m.Normal.X, -1, 0, "Circle-face normal");
        Check(CollisionDetection.Collide(box, c1, out ContactManifold reverse), "Reverse circle-polygon");
        Near(reverse.Normal.X, 1, 0, "Reversed contact normal");
        c1.Position = new(3, 3);
        Check(CollisionDetection.Collide(c1, box, out m), "Circle-corner contact");
        Near(m.Penetration, 2 - MathF.Sqrt(2), 1e-5f, "Corner depth");
        c1.Position = new(3.5f, 3.5f);
        Check(!CollisionDetection.Collide(c1, box, out _), "Circle-corner false positive");
        c1.Position = new(0.5f, 0);
        Check(CollisionDetection.Collide(c1, box, out m), "Contained circle");
        Near(m.Normal.X, -1, 0, "Contained circle exit direction");
        RigidBody box2 = RigidBody.Box(new(3, 0), 4, 4, 1);
        Check(CollisionDetection.Collide(box, box2, out m) && m.ContactCount == 2, "Two-point edge clipping");
        Near(m.Penetration, 1, 1e-5f, "Polygon penetration");
        Near(m.Normal.X, 1, 1e-5f, "Polygon normal");
        box2.Position = new(5, 0); Check(!CollisionDetection.Collide(box, box2, out _), "Separated polygons");
        box2.Position = new(3, 0); box2.Angle = 0.5f;
        Check(CollisionDetection.Collide(box, box2, out m) && m.ContactCount > 0, "Rotated polygon contact");
        RigidBody outer = RigidBody.Box(default, 20, 20, 0), inner = RigidBody.Box(new(1, 0), 2, 2, 1);
        Check(CollisionDetection.Collide(outer, inner, out m), "Polygon containment");
        Near(m.Penetration, 10, 1e-5f, "Containment escape depth");
    }
    private static void BroadPhase()
    {
        PhysicsWorld world = new() { Gravity = default };
        world.Add(RigidBody.Box(new(-1, 0), 160, 160, 0));
        world.Add(RigidBody.Box(new(1, 0), 160, 160, 0));
        world.Add(RigidBody.Circle(default, 100, 1));
        world.Add(RigidBody.Circle(new(20, 0), 100, 1));
        world.Grid.Build(world.Bodies);
        Check(world.Grid.PairCount == 5, "Unique pairs, negative cells, static-static filtering");
        long previous = -1;
        for (int i = 0; i < world.Grid.PairCount; i++)
        {
            BodyPair pair = world.Grid.GetPair(i); long key = ((long)pair.A << 32) | (uint)pair.B;
            Check(key > previous && pair.A < pair.B, "Pair order"); previous = key;
        }
        Random random = new(42);
        for (int i = 0; i < 50; i++)
            world.Add(RigidBody.Circle(new(random.Next(-500, 500), random.Next(-500, 500)), random.Next(5, 90), 1));
        world.Grid.Build(world.Bodies);
        int expected = 0;
        for (int i = 0; i < world.Bodies.Length; i++)
            for (int j = i + 1; j < world.Bodies.Length; j++)
                if (!(world.Bodies[i].IsStatic && world.Bodies[j].IsStatic) && world.Bodies[i].Bounds().Overlaps(world.Bodies[j].Bounds())) expected++;
        Check(world.Grid.PairCount == expected, "Grid agrees with exhaustive AABB oracle");
    }
    private static void Dynamics()
    {
        PhysicsWorld world = new() { Gravity = default };
        RigidBody a = world.Add(RigidBody.Circle(new(-9, 0), 10, 1));
        RigidBody b = world.Add(RigidBody.Circle(new(9, 0), 10, 1));
        a.Restitution = b.Restitution = 1; a.Velocity = new(100, 0); b.Velocity = new(-100, 0);
        world.Step();
        Near(a.Velocity.X, -100, 0.001f, "Elastic impulse A");
        Near(b.Velocity.X, 100, 0.001f, "Elastic impulse B");
        world.Clear(); world.Gravity = new(0, 900);
        RigidBody floor = world.Add(RigidBody.Box(new(0, 110), 2000, 20, 0));
        RigidBody falling = world.Add(RigidBody.Box(new(0, 50), 20, 20, 1));
        falling.Restitution = 0;
        for (int i = 0; i < 600; i++) world.Step();
        Near(falling.Position.Y, 90, 0.15f, "Settled box on floor");
        Near(falling.Velocity.Y, 0, 0.5f, "Resting velocity");
        Near(floor.Position.Y, 110, 0, "Static body unchanged");
        falling.Velocity = new(100, 0);
        for (int i = 0; i < 120; i++) world.Step();
        Check(MathF.Abs(falling.Velocity.X) < 1, "Coulomb friction slows sliding");
        world.Clear(); world.Gravity = default;
        RigidBody spinning = world.Add(RigidBody.Box(default, 20, 20, 1));
        spinning.ApplyImpulse(new(10, 0), new(0, 10));
        Check(spinning.AngularVelocity < 0, "Off-center angular impulse");
        world.Grab(new(0, 5)); world.MouseTarget = new(50, 5); world.Step();
        Check(spinning.Velocity.X > 10 && spinning.AngularVelocity < 0, "Spring grab force and torque");
        world.ReleaseGrab();
        world.Clear(); world.Gravity = new(0, 900);
        world.Add(RigidBody.Box(new(0, 110), 1000, 20, 0));
        for (int i = 0; i < 5; i++)
        {
            RigidBody body = world.Add(RigidBody.Box(new(0, 90 - i * 20.1f), 20, 20, 1)); body.Restitution = 0;
        }
        for (int i = 0; i < 1200; i++) world.Step();
        for (int i = 1; i < world.Bodies.Length; i++)
        {
            Check(MathF.Abs(world.Bodies[i].Position.X) < 8, "Five-box stack lateral stability");
            Near(world.Bodies[i].Position.Y, 90 - (i - 1) * 20, 3, "Five-box stack height");
        }
        PhysicsWorld clamp = new() { Gravity = default };
        clamp.Advance(10); Check(clamp.Tick == 14 && clamp.Accumulator < PhysicsWorld.FixedDeltaTime, "0.25s frame clamp");
        clamp.Clear();
        for (int i = 0; i < 20; i++) clamp.Advance(PhysicsWorld.FixedDeltaTime / 2);
        Check(clamp.Tick == 10, "Fractional frame accumulation");
    }
    private static void ReplayAndAllocations()
    {
        PhysicsWorld first = new(); SandboxScene.Reset(first);
        for (int i = 0; i < 600; i++) first.Step();
        string expected = first.ComputeStateHash();
        Console.WriteLine($"Seed 42 | tick 600 | SHA-256 {expected}");
        for (int i = 5; i < first.Bodies.Length; i++)
        {
            RigidBody b = first.Bodies[i];
            Check(float.IsFinite(b.Position.X) && float.IsFinite(b.Position.Y) && float.IsFinite(b.Angle), "Finite pyramid state");
            Check(b.Position.X > 390 && b.Position.X < 890 && b.Position.Y < 671, "Pyramid remains on floor");
        }
        PhysicsWorld second = new(); SandboxScene.Reset(second);
        while (second.Tick < 600) second.Advance(PhysicsWorld.FixedDeltaTime * 3, 600);
        Check(expected == second.ComputeStateHash(), "Frame-grouped deterministic replay at tick 600");
        SandboxScene.Reset(second);
        for (int i = 0; i < 600; i++) second.Step();
        Check(expected == second.ComputeStateHash(), "Reset deterministic replay");
        // Warm up all geometry/solver paths before measuring current-thread allocations.
        PhysicsWorld mixed = new(); Random random = SandboxScene.Reset(mixed);
        for (int i = 0; i < 25; i++) SandboxScene.Spawn(mixed, random, new(420 + i * 16, 180 + i % 3 * 30));
        for (int i = 0; i < 240; i++) mixed.Step();
        mixed.Grab(mixed.Bodies[10].Position); mixed.MouseTarget += new Vec2(20, -40);
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 240; i++) mixed.Advance(PhysicsWorld.FixedDeltaTime);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Check(allocated == 0, $"Hot-path allocation: {allocated} bytes");
        Console.WriteLine($"Steady-state allocation across 240 mixed-shape ticks with spring grab: {allocated} bytes");
    }
    private static void ContinuousDetection()
    {
        PhysicsWorld world = new() { Gravity = default, ContinuousDetection = false };
        world.Add(RigidBody.Box(default, 2000, 1, 0));
        RigidBody bullet = world.Add(RigidBody.Circle(new(0, -50), 1, 1));
        bullet.Velocity = new(0, 12000); world.Step();
        Check(bullet.Position.Y > 1, "Discrete tunneling regression fixture");
        world.ContinuousDetection = true;
        for (int shape = 0; shape < 3; shape++)
        {
            world.Clear(); world.Add(RigidBody.Box(default, 2000, 1, 0));
            bullet = shape switch
            {
                0 => RigidBody.Circle(new(0, -50), 1, 1),
                1 => RigidBody.Box(new(0, -50), 2, 2, 1),
                _ => RigidBody.Polygon(new(0, -50), new Vec2[] { new(-1, 1), new(0, -1), new(1, 1) }, 1)
            };
            bullet.Restitution = 0; bullet.Velocity = new(0, 12000); world.Add(bullet); world.Step();
            Check(bullet.Position.Y < 0 && bullet.Velocity.Y < 1 && world.CcdEvents > 0, $"Shape {shape} thin-wall CCD: Y={bullet.Position.Y}, VY={bullet.Velocity.Y}, events={world.CcdEvents}");
            Check(!world.CcdMotionClamped, "Simple impact must converge");
        }
        world.Clear();
        RigidBody a = world.Add(RigidBody.Circle(new(-50, 0), 1, 1));
        RigidBody b = world.Add(RigidBody.Circle(new(50, 0), 1, 1));
        a.Restitution = b.Restitution = 0;
        a.Velocity = new(12000, 0); b.Velocity = new(-12000, 0); world.Step();
        Check(a.Position.X < b.Position.X && MathF.Abs(a.Velocity.X) < 1 && MathF.Abs(b.Velocity.X) < 1, "Dynamic-dynamic CCD");
        world.Clear();
        RigidBody left = world.Add(RigidBody.Box(new(-20, 0), 1, 200, 0));
        RigidBody right = world.Add(RigidBody.Box(new(20, 0), 1, 200, 0));
        bullet = world.Add(RigidBody.Circle(default, 1, 1));
        left.Restitution = right.Restitution = bullet.Restitution = 1;
        bullet.Velocity = new(10000, 0); world.Step();
        Check(MathF.Abs(bullet.Position.X) < 19 && world.CcdEvents >= 4 && !world.CcdMotionClamped, "Repeated bounces within one tick");
        Near(MathF.Abs(bullet.Velocity.X), 10000, 0.1f, "CCD elastic bounce energy");
        bullet.Position = default; bullet.Velocity = new(1_000_000, 0); world.Step();
        Check(world.CcdMotionClamped && world.CcdEvents == ContinuousCollision.MaxEvents && MathF.Abs(bullet.Position.X) < 19,
            "CCD event exhaustion clamps motion within walls");
        world.Clear();
        RigidBody rod = world.Add(RigidBody.Box(default, 60, 1, 1));
        RigidBody obstacle = world.Add(RigidBody.Circle(new(0, 24), 1, 0));
        rod.AngularVelocity = 120;
        Check(ContinuousCollision.Cast(rod, obstacle, PhysicsWorld.FixedDeltaTime, out float time, out _, out bool resolved) && resolved && time > 0,
            "Rotating polygon time of impact");
        Near(rod.Angle, 0, 0, "Cast restores angle"); Near(rod.Position.LengthSquared, 0, 0, "Cast restores position");
        world.Step(); Check(world.CcdEvents > 0, "Angular CCD event");
        // A path outside a rounded corner must remain a miss.
        world.Clear();
        RigidBody corner = world.Add(RigidBody.Box(default, 2, 2, 0));
        bullet = world.Add(RigidBody.Circle(new(-50, -2.1f), 1, 1)); bullet.Velocity = new(12000, 0);
        Check(!ContinuousCollision.Cast(bullet, corner, PhysicsWorld.FixedDeltaTime, out _, out _, out _), "CCD corner near miss");
        world.Step(); Check(bullet.Position.X > 100 && world.CcdEvents == 0, "Near miss does not stop motion");
        // TOI casting, broad-phase sweeping and impact response remain allocation-free.
        for (int i = 0; i < 10; i++)
        { bullet.Position = new(-50, 0); bullet.Velocity = new(12000, 0); world.Step(); }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 60; i++)
        { bullet.Position = new(-50, 0); bullet.Velocity = new(12000, 0); world.Step(); }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Check(allocated == 0, $"CCD allocation: {allocated} bytes");
        Console.WriteLine($"CCD thin walls, dynamic impacts, angular sweep, ricochets, near miss: PASS; allocations: {allocated} bytes");
    }
    private static void InputReplay()
    {
        SandboxSession session = new() { Paused = true };
        string initial = session.World.ComputeStateHash();
        Check(session.Advance(10) == 0 && session.TimelineTick == 0 && session.World.ComputeStateHash() == initial, "Pause freezes clock and state");
        session.SingleStep(); Check(session.TimelineTick == 1 && session.World.Tick == 1 && session.Paused, "Single step advances exactly one tick");
        session.BeginRecording(); session.Paused = false;
        for (int tick = 0; tick < 180; tick++)
        {
            if (tick == 0 || tick == 61) session.Queue(InputKind.Spawn, new(300, 200));
            if (tick == 5 || tick == 66) session.Queue(InputKind.Grab, new(300, 204));
            if (tick is > 5 and < 25 || tick is > 66 and < 85) session.Queue(InputKind.MoveTarget, new(340 + tick, 180));
            if (tick == 25 || tick == 85) session.Queue(InputKind.Release);
            if (tick == 60) session.Queue(InputKind.Reset);
            Check(session.Advance(PhysicsWorld.FixedDeltaTime) == 1, "Live session fixed tick");
        }
        Check(session.World.Tick == 120 && session.TimelineTick == 180, "Reset preserves recording timeline");
        string positions = session.World.ComputeStateHash(), kinematics = session.World.ComputeKinematicHash();
        int bodyCount = session.World.Bodies.Length;
        // Pending input must not contaminate a recording ending at the current tick.
        session.Queue(InputKind.Spawn, new(900, 200));
        string path = Path.Combine(Path.GetTempPath(), $"physics-replay-{Guid.NewGuid():N}.json");
        try
        {
            session.Save(path); Check(session.BeginReplay(), "Begin replay");
            Check(!session.Queue(InputKind.Spawn, default), "Playback ignores live inputs");
            while (!session.PlaybackFinished) session.Advance(PhysicsWorld.FixedDeltaTime * 7);
            Check(session.ReplayVerified && session.Paused && session.World.Bodies.Length == bodyCount, "Replay verifies and pauses at exact endpoint");
            Check(session.World.ComputeStateHash() == positions && session.World.ComputeKinematicHash() == kinematics, "Replay matches transform and velocity bits");
            session.Load(path);
            while (!session.PlaybackFinished) session.Advance(PhysicsWorld.FixedDeltaTime / 2);
            Check(session.ReplayVerified, "Saved replay matches under fractional frame grouping");
            File.WriteAllText(path, "{\"Version\":99,\"Ticks\":1}");
            bool rejected = false;
            try { session.Load(path); } catch (InvalidDataException) { rejected = true; }
            Check(rejected && session.World.ComputeStateHash() == positions, "Invalid replay leaves active state intact");
        }
        finally { if (File.Exists(path)) File.Delete(path); }
        session.BeginRecording(); session.Paused = true;
        int startCount = session.World.Bodies.Length;
        session.Queue(InputKind.Spawn, new(300, 200)); session.Queue(InputKind.Reset);
        session.Queue(InputKind.Spawn, new(300, 200));
        session.SingleStep(); Check(session.World.Bodies.Length == startCount + 1 && session.World.Tick == 1, "Same-tick command order including reset");
        session.BeginReplay(); session.Advance(PhysicsWorld.FixedDeltaTime); Check(session.ReplayVerified, "Same-tick command replay");
        session.BeginRecording(); session.Paused = true;
        session.Queue(InputKind.Grab, new(640, 656)); session.SingleStep();
        for (int i = 0; i < 10; i++) { session.Queue(InputKind.MoveTarget, new(640, 580)); session.SingleStep(); }
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 60; i++)
        { session.Queue(InputKind.MoveTarget, new(640 + i, 580)); session.SingleStep(); }
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Check(allocated == 0, $"Recording/drag allocation: {allocated} bytes");
        Console.WriteLine($"Pause, single-step, tick commands, reset replay, JSON round-trip: PASS; input/drag allocations: {allocated} bytes");
    }
    private static void Ragdolls()
    {
        PhysicsWorld world = new();
        world.Add(RigidBody.Box(new(0, 410), 1200, 20, 0));
        Check(RagdollFactory.TrySpawn(world, new(0, 100)), "Ragdoll spawn");
        Check(world.Bodies.Length == 12 && world.Joints.Length == 10, "Ragdoll body/joint counts");
        world.Grid.Build(world.Bodies);
        Check(world.Grid.PairCount == 0, "Ragdoll self-collision filter");
        for (int tick = 0; tick < 900; tick++)
        {
            if (tick == 150) world.Grab(world.Bodies[1].Position);
            if (tick is >= 150 and < 330) world.MouseTarget = new(80 * MathF.Sin(tick * 0.025f), 120);
            if (tick == 330) world.ReleaseGrab();
            world.Step();
            if (tick == 149) Check(world.Bodies[2].Position.Y > 200, "Ragdoll falls under gravity before dragging");
            if (tick == 329) Check((world.Bodies[1].Position - world.MouseTarget).Length < 60, "Head grab lifts the connected ragdoll toward the target");
            foreach (RevoluteJoint joint in world.Joints)
            {
                Check((joint.AnchorB - joint.AnchorA).Length < 3, $"Ragdoll hinge separation at tick {tick}");
                Check(joint.RelativeAngle >= joint.LowerAngle - 0.2f && joint.RelativeAngle <= joint.UpperAngle + 0.2f,
                    $"Ragdoll joint limit at tick {tick}: {joint.RelativeAngle}");
            }
            foreach (RigidBody body in world.Bodies)
                Check(float.IsFinite(body.Position.X) && float.IsFinite(body.Position.Y) && float.IsFinite(body.Angle), "Finite ragdoll state");
        }
        foreach (RigidBody body in world.Bodies)
            if (!body.IsStatic) Check(body.Position.Y < 415, "Ragdoll remains above floor");
        Check(world.Bodies[1].Position.Y > 250, "Released ragdoll falls back toward floor");
        foreach (RigidBody body in world.Bodies)
            Check(body.Velocity.Length < 50, "Settled ragdoll has no CCD velocity buildup");
        world.Grab(world.Bodies[1].Position); world.MouseTarget += new Vec2(20, -60);
        for (int i = 0; i < 10; i++) world.Step();
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 120; i++) world.Step();
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Check(allocated == 0, $"Ragdoll hot-path allocation: {allocated}");
        world.Clear(); Check(world.Joints.Length == 0, "Reset clears joints");
        Check(RagdollFactory.TrySpawn(world, default) && RagdollFactory.TrySpawn(world, default), "Multiple ragdoll spawn");
        Check(world.Bodies[0].CollisionGroup != world.Bodies[11].CollisionGroup, "Distinct ragdoll collision groups");
        world.Grid.Build(world.Bodies);
        Check(world.Grid.PairCount > 0, "Different ragdolls collide");
        for (int i = 0; i < world.Grid.PairCount; i++)
        {
            BodyPair pair = world.Grid.GetPair(i);
            Check(world.Bodies[pair.A].CollisionGroup != world.Bodies[pair.B].CollisionGroup, "Same-ragdoll pairs are filtered");
        }
        world.Clear();
        for (int i = 0; i < PhysicsWorld.MaxBodies - 10; i++) world.Add(RigidBody.Circle(new(i * 3, 0), 1, 1));
        Check(!RagdollFactory.TrySpawn(world, default) && world.Bodies.Length == PhysicsWorld.MaxBodies - 10 && world.Joints.Length == 0,
            "Ragdoll capacity failure is atomic");
        SandboxSession session = new();
        session.Queue(InputKind.SpawnRagdoll, new(350, 230));
        for (int i = 0; i < 240; i++) session.Advance(PhysicsWorld.FixedDeltaTime);
        Check(session.World.Joints.Length == 10, "Recorded ragdoll spawn");
        string path = Path.Combine(Path.GetTempPath(), $"ragdoll-replay-{Guid.NewGuid():N}.json");
        try
        {
            session.Save(path); session.Load(path);
            while (!session.PlaybackFinished) session.Advance(PhysicsWorld.FixedDeltaTime * 4);
            Check(session.ReplayVerified && session.World.Joints.Length == 10, "Ragdoll JSON deterministic replay");
        }
        finally { if (File.Exists(path)) File.Delete(path); }
        session.BeginRecording(); session.Queue(InputKind.SpawnRagdoll, new(350, 230));
        session.Queue(InputKind.Reset); session.Advance(PhysicsWorld.FixedDeltaTime);
        Check(session.World.Joints.Length == 0, "Recorded reset clears ragdoll joints and collision groups");
        Console.WriteLine($"Ragdoll fall, drag, hinges/limits, self-filter, capacity, replay: PASS; allocations: {allocated} bytes");
    }
    private static void Cables()
    {
        PhysicsWorld world = new() { Gravity = default };
        RigidBody body = world.Add(RigidBody.Circle(new(30, 0), 2, 1));
        CableJoint cable = new(null, body, default, body.Position, 100); world.AddCable(cable);
        body.Velocity = new(-12, 5); world.Step();
        Near(body.Velocity.X, -12, 1e-5f, "Slack cable does not push or pull");
        Near(body.Velocity.Y, 5, 1e-5f, "Slack cable preserves tangential velocity");
        body.Position = new(30, 0); body.Velocity = new(12000, 0); world.Step();
        Near(cable.Length, 100, 0.02f, "Predictive cable limit stops a fast endpoint");
        body.Velocity = new(-20, 0); world.Step();
        Near(body.Velocity.X, -20, 0.01f, "Taut cable releases when moving inward");
        world.Clear(); world.Gravity = new(0, 900);
        body = world.Add(RigidBody.Circle(new(0, 80), 2, 2));
        cable = new(null, body, default, body.Position, 100); world.AddCable(cable);
        for (int i = 0; i < 600; i++)
        { world.Step(); Check(cable.Length < 100.05f, "Hanging cable length under gravity"); }
        Near(body.Position.Y, 100, 0.05f, "Cable supports a suspended object");
        Near(body.Velocity.Y, 0, 0.1f, "Supported object has no velocity buildup");
        body.Velocity = new(60, 0);
        for (int i = 0; i < 10; i++) world.Step();
        long before = GC.GetAllocatedBytesForCurrentThread();
        for (int i = 0; i < 120; i++) world.Step();
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        Check(allocated == 0, $"Cable allocation: {allocated} bytes");
        world.Clear(); world.Gravity = default;
        RigidBody a = world.Add(RigidBody.Circle(new(-25, 0), 2, 1));
        RigidBody b = world.Add(RigidBody.Circle(new(25, 0), 2, 1));
        world.AddCable(new(a, b, a.Position, b.Position, 50));
        a.Velocity = new(-50, 0); b.Velocity = new(50, 0); world.Step();
        Near(a.Velocity.X + b.Velocity.X, 0, 1e-5f, "Cable conserves linear momentum");
        Near(a.Velocity.X, 0, 1e-5f, "Cable ties two dynamic bodies");
        world.Clear(); body = world.Add(RigidBody.Box(new(50, 0), 20, 20, 1));
        Vec2 attachment = new(50, 8);
        world.AddCable(new(null, body, default, attachment, attachment.Length));
        body.Velocity = new(100, 0); world.Step();
        Check(body.AngularVelocity > 0, "Off-center cable creates angular response");
        world.Clear(); world.Gravity = new(0, 900);
        Check(RagdollFactory.TrySpawn(world, new(0, 100)), "Cable ragdoll fixture");
        body = world.Bodies[0]; cable = new(null, body, default, body.Position, body.Position.Length); world.AddCable(cable);
        for (int i = 0; i < 600; i++)
        {
            world.Step(); Check(cable.Length <= cable.MaxLength + 1, "Cable supports connected ragdoll mass");
            foreach (RevoluteJoint joint in world.Joints)
                Check((joint.AnchorB - joint.AnchorA).Length < 3, "Suspended ragdoll stays connected");
        }
        Check(body.Position.Y < 70, "Ragdoll remains suspended");
        world.Clear(); Check(world.Cables.Length == 0, "Reset clears cables");
        CableJoint[] ordered = new CableJoint[3];
        for (int i = 0; i < ordered.Length; i++)
        {
            body = world.Add(RigidBody.Circle(new(i * 200, 80), 2, 1));
            ordered[i] = new(null, body, new(i * 200, 0), body.Position, 120); world.AddCable(ordered[i]);
        }
        Check(world.CutCable(ordered[1].DisplayPoint(0.5f)), "Cut follows visual slack curve");
        Check(world.Cables.Length == 2 && ReferenceEquals(world.Cables[0], ordered[0]) && ReferenceEquals(world.Cables[1], ordered[2]),
            "Cable removal preserves solve order");
        Check(!world.CutCable(new(-1000, -1000)) && world.Cables.Length == 2, "Cut miss does not change cables");
        bool rejected = false;
        try { world.AddCable(new(null, RigidBody.Circle(default, 1, 1), default, new(10, 0), 10)); }
        catch (ArgumentException) { rejected = true; }
        Check(rejected && world.Cables.Length == 2, "Reject foreign-world cable without mutation");
        world.Clear(); body = world.Add(RigidBody.Circle(new(10, 0), 1, 1));
        for (int i = 0; i < PhysicsWorld.MaxCables; i++) world.AddCable(new(null, body, default, body.Position, 10));
        rejected = false;
        try { world.AddCable(new(null, body, default, body.Position, 10)); }
        catch (InvalidOperationException) { rejected = true; }
        Check(rejected && world.Cables.Length == PhysicsWorld.MaxCables, "Cable capacity is explicit and atomic");
        SandboxSession session = new() { Paused = true };
        session.Queue(InputKind.Spawn, new(330, 220));
        session.Queue(InputKind.CableAnchor, new(330, 170)); session.Queue(InputKind.CableAnchor, new(330, 220));
        session.SingleStep(); Check(session.World.Cables.Length == 1 && !session.HasCableAnchor, "Same-tick tie commands");
        body = session.World.Bodies[^1];
        session.Queue(InputKind.CableAnchor, body.Position); session.Queue(InputKind.CableAnchor, body.Position);
        session.SingleStep(); Check(session.HasCableAnchor && session.World.Cables.Length == 1, "Same-body tie rejection retains first endpoint");
        session.Queue(InputKind.CancelCable); session.SingleStep(); Check(!session.HasCableAnchor, "Cancel cable endpoint selection");
        session.Paused = false;
        for (int tick = 0; tick < 180; tick++)
        {
            if (tick == 30) session.Queue(InputKind.CutCable, session.World.Cables[0].DisplayPoint(0.5f));
            if (tick == 60)
            {
                session.Queue(InputKind.CableAnchor, new(330, 170)); session.Queue(InputKind.CableAnchor, body.Position);
            }
            session.Advance(PhysicsWorld.FixedDeltaTime);
            if (tick == 30) Check(session.World.Cables.Length == 0, "Recorded cable cut");
        }
        Check(session.World.Cables.Length == 1, "Recorded cable reattachment");
        string path = Path.Combine(Path.GetTempPath(), $"cable-replay-{Guid.NewGuid():N}.json");
        try
        {
            session.Save(path); session.Load(path);
            while (!session.PlaybackFinished) session.Advance(PhysicsWorld.FixedDeltaTime * 5);
            Check(session.ReplayVerified && session.World.Cables.Length == 1, "Cable tie/cut JSON deterministic replay");
        }
        finally { if (File.Exists(path)) File.Delete(path); }
        session.BeginRecording(); session.Queue(InputKind.CableAnchor, new(330, 170)); session.Queue(InputKind.Reset);
        session.Paused = true; session.SingleStep();
        Check(!session.HasCableAnchor && session.World.Cables.Length == 0, "Reset clears pending cable endpoints");
        Console.WriteLine($"Cables: slack/tension, fast endpoint, momentum, angular response, hanging ragdoll, cut, capacity, replay: PASS; allocations: {allocated} bytes");
    }
}
