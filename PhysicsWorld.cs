using System.Buffers.Binary;
using System.Security.Cryptography;

namespace DeterministicPhysics2D;

public sealed class PhysicsWorld
{
    public const float FixedDeltaTime = 1.0f / 60.0f;
    public const int MaxBodies = 512;
    public const int MaxJoints = 512;
    public const int MaxCables = 512;
    private const int MaxPairs = MaxBodies * (MaxBodies - 1) / 2;
    private readonly RigidBody[] bodies = new RigidBody[MaxBodies];
    private readonly ContactManifold[] contacts = new ContactManifold[MaxPairs];
    private readonly ImpulseSolver solver = new(MaxPairs);
    private readonly RevoluteJoint[] joints = new RevoluteJoint[MaxJoints];
    private readonly CableJoint[] cables = new CableJoint[MaxCables];
    private int cableCount;
    private int jointCount, nextCollisionGroup;
    private int bodyCount, contactCount;
    private double accumulator;
    public Vec2 Gravity { get; set; } = new(0, 900);
    public bool ContinuousDetection { get; set; } = true;
    public int CcdEvents { get; private set; }
    public bool CcdMotionClamped { get; private set; }
    public BroadPhaseGrid Grid { get; } = new(MaxBodies);
    public ReadOnlySpan<RigidBody> Bodies => bodies.AsSpan(0, bodyCount);
    public ReadOnlySpan<ContactManifold> Contacts => contacts.AsSpan(0, contactCount);
    public ReadOnlySpan<RevoluteJoint> Joints => joints.AsSpan(0, jointCount);
    public ReadOnlySpan<CableJoint> Cables => cables.AsSpan(0, cableCount);
    public long Tick { get; private set; }
    public double Accumulator => accumulator;
    public RigidBody? GrabbedBody { get; private set; }
    public Vec2 GrabLocalAnchor { get; private set; }
    public Vec2 MouseTarget { get; set; }
    public RigidBody Add(RigidBody body)
    {
        if (bodyCount == MaxBodies) throw new InvalidOperationException("Body capacity reached.");
        if (body.Id != -1) throw new ArgumentException("Body already belongs to a world.");
        body.Id = bodyCount; bodies[bodyCount++] = body; return body;
    }
    internal int AllocateCollisionGroup() => ++nextCollisionGroup;
    public void AddJoint(RevoluteJoint joint)
    {
        if (jointCount == MaxJoints) throw new InvalidOperationException("Joint capacity reached.");
        if (joint.A.Id < 0 || joint.B.Id < 0 || joint.A.Id >= bodyCount || joint.B.Id >= bodyCount ||
            !ReferenceEquals(bodies[joint.A.Id], joint.A) || !ReferenceEquals(bodies[joint.B.Id], joint.B))
            throw new ArgumentException("Joint bodies must belong to this world.");
        for (int i = 0; i < jointCount; i++)
            if (ReferenceEquals(joints[i], joint)) throw new ArgumentException("Joint already added.");
        joints[jointCount++] = joint;
    }
    public void Clear()
    {
        for (int i = 0; i < bodyCount; i++) bodies[i].Id = -1;
        Array.Clear(bodies); Array.Clear(contacts);
        Array.Clear(joints); jointCount = nextCollisionGroup = 0;
        Array.Clear(cables); cableCount = 0;
        solver.Clear();
        bodyCount = contactCount = 0; accumulator = 0; Tick = 0; ReleaseGrab();
        CcdEvents = 0; CcdMotionClamped = false;
        Grid.Build(Bodies);
    }
    public RigidBody? PickBody(Vec2 point, bool includeStatic = true)
    {
        for (int i = bodyCount - 1; i >= 0; i--)
            if ((includeStatic || !bodies[i].IsStatic) && bodies[i].Contains(point)) return bodies[i];
        return null;
    }
    private bool OwnsEndpoint(RigidBody? body) => body is null ||
        (body.Id >= 0 && body.Id < bodyCount && ReferenceEquals(bodies[body.Id], body));
    public void AddCable(CableJoint cable)
    {
        if (cableCount == MaxCables) throw new InvalidOperationException("Cable capacity reached.");
        if (!OwnsEndpoint(cable.A) || !OwnsEndpoint(cable.B)) throw new ArgumentException("Cable bodies must belong to this world.");
        for (int i = 0; i < cableCount; i++)
            if (ReferenceEquals(cables[i], cable)) throw new ArgumentException("Cable already added.");
        cables[cableCount++] = cable;
    }
    public bool CutCable(Vec2 point, float radius = 14)
    {
        int nearest = -1; float best = radius * radius;
        for (int i = 0; i < cableCount; i++)
        {
            Vec2 start = cables[i].DisplayPoint(0);
            for (int segment = 1; segment <= 20; segment++)
            {
                Vec2 end = cables[i].DisplayPoint(segment / 20f), edge = end - start;
                float t = edge.LengthSquared > 1e-8f ? Math.Clamp(Vec2.Dot(point - start, edge) / edge.LengthSquared, 0, 1) : 0;
                float distance2 = (point - start - edge * t).LengthSquared;
                if (distance2 < best) { best = distance2; nearest = i; }
                start = end;
            }
        }
        if (nearest < 0) return false;
        // Preserve creation order of surviving constraints for deterministic solving.
        for (int i = nearest; i < cableCount - 1; i++) cables[i] = cables[i + 1];
        cables[--cableCount] = null!; return true;
    }
    public void Grab(Vec2 point)
    {
        ReleaseGrab(); MouseTarget = point;
        for (int i = bodyCount - 1; i >= 0; i--)
            if (!bodies[i].IsStatic && bodies[i].Contains(point))
            { GrabbedBody = bodies[i]; GrabLocalAnchor = bodies[i].ToLocal(point); break; }
    }
    public void ReleaseGrab() => GrabbedBody = null;
    // The optional tick boundary lets callers capture an exact tick hash even when
    // a rendered frame consumes several fixed steps. Call Advance(0) to drain the rest.
    public int Advance(float frameTime, long stopAtTick = long.MaxValue)
    {
        if (!float.IsFinite(frameTime) || frameTime < 0) throw new ArgumentOutOfRangeException(nameof(frameTime));
        accumulator += Math.Min(frameTime, 0.25f);
        int steps = 0;
        while (accumulator >= FixedDeltaTime && Tick < stopAtTick)
        {
            Step(); accumulator -= FixedDeltaTime; steps++;
        }
        return steps;
    }
    public void Step()
    {
        if (GrabbedBody is { } grabbed)
        {
            Vec2 anchor = grabbed.ToWorld(GrabLocalAnchor);
            float springMass = grabbed.Mass;
            if (grabbed.CollisionGroup != 0)
            {
                springMass = 0;
                for (int i = 0; i < bodyCount; i++)
                    if (bodies[i].CollisionGroup == grabbed.CollisionGroup) springMass += bodies[i].Mass;
            }
            bool articulated = grabbed.CollisionGroup != 0;
            float stiffness = springMass * (articulated ? 180 : 65);
            float damping = springMass * (articulated ? 20 : 14);
            Vec2 force = (MouseTarget - anchor) * stiffness - grabbed.PointVelocity(anchor) * damping;
            if (articulated)
            {
                // Implicit spring response keeps a light head from receiving an
                // unstable explicit impulse when it lifts the whole assembly.
                force /= 1 + damping * FixedDeltaTime * grabbed.InverseMass +
                    stiffness * FixedDeltaTime * FixedDeltaTime * grabbed.InverseMass;
            }
            float maxForce = springMass * (articulated ? 2500 : 30_000);
            if (force.LengthSquared > maxForce * maxForce) force = force.Normalized() * maxForce;
            grabbed.ApplyForce(force, anchor);
        }
        for (int i = 0; i < bodyCount; i++)
        {
            RigidBody body = bodies[i];
            if (!body.IsStatic)
            {
                body.Velocity += (Gravity + body.Force * body.InverseMass) * FixedDeltaTime;
                body.AngularVelocity += body.Torque * body.InverseInertia * FixedDeltaTime;
            }
            body.Force = default; body.Torque = 0;
        }
        // TOI stops at a tiny positive separation. Preserve support contacts in
        // that shell so joints can transfer weight without CCD freeze/velocity buildup.
        float contactMargin = ContinuousDetection ? ContinuousCollision.Tolerance * 1.5f : 0;
        Grid.Build(Bodies, contactMargin: contactMargin); contactCount = 0;
        for (int i = 0; i < Grid.PairCount; i++)
        {
            BodyPair pair = Grid.GetPair(i);
            if (CollisionDetection.Collide(bodies[pair.A], bodies[pair.B], out ContactManifold manifold, contactMargin))
                contacts[contactCount++] = manifold;
        }
        solver.Solve(Contacts, Joints, Cables);
        CcdEvents = 0; CcdMotionClamped = false;
        if (ContinuousDetection && NeedsContinuousPass()) IntegrateContinuously();
        else IntegratePositions(FixedDeltaTime);
        ImpulseSolver.CorrectPositions(Contacts);
        for (int iteration = 0; iteration < ImpulseSolver.Iterations; iteration++)
        {
            for (int i = 0; i < jointCount; i++) joints[i].CorrectPosition();
            for (int i = 0; i < cableCount; i++) cables[i].CorrectPosition();
        }
        Tick++;
    }
    private void IntegratePositions(float duration)
    {
        for (int i = 0; i < bodyCount; i++)
        {
            RigidBody body = bodies[i];
            if (body.IsStatic) continue;
            body.Position += body.Velocity * duration;
            body.Angle += body.AngularVelocity * duration;
        }
    }
    private void IntegrateContinuously()
    {
        CcdEvents = 0; CcdMotionClamped = false;
        float remaining = FixedDeltaTime;
        while (remaining > 0)
        {
            Grid.Build(Bodies, remaining);
            float earliest = remaining; bool found = false, converged = true;
            ContactManifold impact = default;
            for (int i = 0; i < Grid.PairCount; i++)
            {
                BodyPair pair = Grid.GetPair(i);
                if (ContinuousCollision.Cast(bodies[pair.A], bodies[pair.B], remaining,
                    out float time, out ContactManifold candidate, out bool resolved) && (!found || time < earliest))
                { earliest = time; impact = candidate; found = true; converged = resolved; }
            }
            IntegratePositions(earliest); remaining = MathF.Max(0, remaining - earliest);
            if (!found) break;
            if (!converged || CcdEvents == ContinuousCollision.MaxEvents)
            { CcdMotionClamped = true; break; }
            ImpulseSolver.SolveImpact(impact); CcdEvents++;
        }
        Grid.Build(Bodies);
    }
    private bool NeedsContinuousPass()
    {
        float minimum = float.PositiveInfinity;
        for (int i = 0; i < bodyCount; i++) minimum = MathF.Min(minimum, bodies[i].MinimumExtent);
        for (int i = 0; i < bodyCount; i++)
        {
            RigidBody body = bodies[i];
            if (body.IsStatic) continue;
            float speed = body.Velocity.Length + (body.Shape == ShapeType.Circle ? 0 : MathF.Abs(body.AngularVelocity) * body.SweepRadius);
            // If every body's bound is at most half the pair threshold, no
            // relative-motion pair can require a cast. Avoid swept-grid work.
            if (speed * FixedDeltaTime > 0.125f * minimum) return true;
        }
        return false;
    }
    public string ComputeStateHash()
        => ComputeHash(false);
    public string ComputeKinematicHash()
        => ComputeHash(true);
    private string ComputeHash(bool includeVelocities)
    {
        using IncrementalHash hash = IncrementalHash.CreateHash(HashAlgorithmName.SHA256);
        Span<byte> state = stackalloc byte[24];
        for (int i = 0; i < bodyCount; i++)
        {
            RigidBody b = bodies[i];
            BinaryPrimitives.WriteInt32LittleEndian(state, BitConverter.SingleToInt32Bits(b.Position.X));
            BinaryPrimitives.WriteInt32LittleEndian(state[4..], BitConverter.SingleToInt32Bits(b.Position.Y));
            BinaryPrimitives.WriteInt32LittleEndian(state[8..], BitConverter.SingleToInt32Bits(b.Angle));
            if (includeVelocities)
            {
                BinaryPrimitives.WriteInt32LittleEndian(state[12..], BitConverter.SingleToInt32Bits(b.Velocity.X));
                BinaryPrimitives.WriteInt32LittleEndian(state[16..], BitConverter.SingleToInt32Bits(b.Velocity.Y));
                BinaryPrimitives.WriteInt32LittleEndian(state[20..], BitConverter.SingleToInt32Bits(b.AngularVelocity));
            }
            hash.AppendData(state[..(includeVelocities ? 24 : 12)]);
        }
        return Convert.ToHexString(hash.GetHashAndReset());
    }
}
