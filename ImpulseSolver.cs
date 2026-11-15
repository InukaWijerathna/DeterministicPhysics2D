namespace DeterministicPhysics2D;

public sealed class ImpulseSolver
{
    public const int Iterations = 8;
    public const float CorrectionFactor = 0.3f, PenetrationSlop = 0.01f;
    private struct ContactConstraint
    {
        public Vec2 RA, RB, LocalA, LocalB;
        public float NormalMass, TangentMass, NormalImpulse, TangentImpulse, Bounce;
    }
    private struct PairCache
    {
        public Vec2 Normal;
        public long Stamp;
        public int Count;
    }
    private readonly ContactConstraint[] constraints;
    private readonly PairCache[] cache;
    private long stamp;
    public ImpulseSolver(int maxManifolds)
    { constraints = new ContactConstraint[maxManifolds * 2]; cache = new PairCache[maxManifolds]; }
    public void Clear()
    { Array.Clear(constraints); Array.Clear(cache); stamp = 0; }
    private static int Slot(in ContactManifold m)
    {
        int a = m.A.Id, b = m.B.Id;
        if (a > b) (a, b) = (b, a);
        return b * (b - 1) / 2 + a;
    }

    public void Solve(ReadOnlySpan<ContactManifold> manifolds, ReadOnlySpan<RevoluteJoint> joints = default,
        ReadOnlySpan<CableJoint> cables = default)
    {
        stamp++;
        for (int i = 0; i < joints.Length; i++) joints[i].Prepare();
        for (int i = 0; i < cables.Length; i++) cables[i].Prepare();
        for (int i = 0; i < manifolds.Length; i++)
        {
            ref readonly ContactManifold m = ref manifolds[i];
            int slot = Slot(m), first = slot * 2;
            ref PairCache previous = ref cache[slot];
            ContactConstraint old0 = constraints[first], old1 = constraints[first + 1];
            bool reuse = previous.Stamp == stamp - 1 && Vec2.Dot(previous.Normal, m.Normal) > 0.98f;
            int used = 0;
            Vec2 tangent = new(-m.Normal.Y, m.Normal.X);
            float restitution = MathF.Min(m.A.Restitution, m.B.Restitution);
            for (int j = 0; j < m.ContactCount; j++)
            {
                ref ContactConstraint c = ref constraints[first + j]; c = default;
                c.RA = m.GetPoint(j) - m.A.Position; c.RB = m.GetPoint(j) - m.B.Position;
                c.LocalA = m.A.Rotation.Transpose() * c.RA;
                c.LocalB = m.B.Rotation.Transpose() * c.RB;
                c.NormalMass = EffectiveMass(m.A, m.B, c.RA, c.RB, m.Normal);
                c.TangentMass = EffectiveMass(m.A, m.B, c.RA, c.RB, tangent);
                float speed = Vec2.Dot(RelativeVelocity(m.A, m.B, c.RA, c.RB), m.Normal);
                c.Bounce = speed < -30 ? -restitution * speed : 0;
                if (reuse)
                {
                    float d0 = (used & 1) == 0 && previous.Count > 0 ?
                        (c.LocalA - old0.LocalA).LengthSquared + (c.LocalB - old0.LocalB).LengthSquared : float.PositiveInfinity;
                    float d1 = (used & 2) == 0 && previous.Count > 1 ?
                        (c.LocalA - old1.LocalA).LengthSquared + (c.LocalB - old1.LocalB).LengthSquared : float.PositiveInfinity;
                    if (MathF.Min(d0, d1) < 16)
                    {
                        ContactConstraint old = d0 <= d1 ? old0 : old1;
                        used |= d0 <= d1 ? 1 : 2;
                        c.NormalImpulse = old.NormalImpulse; c.TangentImpulse = old.TangentImpulse;
                    }
                }
            }
            previous.Normal = m.Normal; previous.Count = m.ContactCount; previous.Stamp = stamp;
        }
        // Compute bounce from pre-solve velocities before applying any cached impulse.
        for (int i = 0; i < manifolds.Length; i++)
        {
            ref readonly ContactManifold m = ref manifolds[i];
            Vec2 tangent = new(-m.Normal.Y, m.Normal.X); int first = Slot(m) * 2;
            for (int j = 0; j < m.ContactCount; j++)
            {
                ref ContactConstraint c = ref constraints[first + j];
                Apply(m, c, m.Normal * c.NormalImpulse + tangent * c.TangentImpulse);
            }
        }
        for (int i = 0; i < joints.Length; i++) joints[i].WarmStart();
        for (int i = 0; i < cables.Length; i++) cables[i].WarmStart();
        for (int iteration = 0; iteration < Iterations; iteration++)
        {
            for (int i = 0; i < manifolds.Length; i++)
            {
                ref readonly ContactManifold m = ref manifolds[i];
                Vec2 tangent = new(-m.Normal.Y, m.Normal.X);
                float staticFriction = MathF.Sqrt(m.A.StaticFriction * m.B.StaticFriction);
                float dynamicFriction = MathF.Sqrt(m.A.DynamicFriction * m.B.DynamicFriction);
                int first = Slot(m) * 2;
                bool blockSolved = m.ContactCount == 2 && SolveNormalBlock(m, ref constraints[first], ref constraints[first + 1]);
                for (int j = 0; j < m.ContactCount; j++)
                {
                    ref ContactConstraint c = ref constraints[first + j];
                    if (!blockSolved)
                    {
                        float speed = Vec2.Dot(RelativeVelocity(m.A, m.B, c.RA, c.RB), m.Normal);
                        float impulse = c.NormalMass * (c.Bounce - speed);
                        float previous = c.NormalImpulse;
                        c.NormalImpulse = MathF.Max(previous + impulse, 0);
                        Apply(m, c, m.Normal * (c.NormalImpulse - previous));
                    }

                    float tangentSpeed = Vec2.Dot(RelativeVelocity(m.A, m.B, c.RA, c.RB), tangent);
                    float candidate = c.TangentImpulse - tangentSpeed * c.TangentMass;
                    float maxStatic = staticFriction * c.NormalImpulse;
                    float next = MathF.Abs(candidate) <= maxStatic ? candidate :
                        Math.Clamp(candidate, -dynamicFriction * c.NormalImpulse, dynamicFriction * c.NormalImpulse);
                    Apply(m, c, tangent * (next - c.TangentImpulse));
                    c.TangentImpulse = next;
                }
            }
            for (int i = 0; i < joints.Length; i++) joints[i].SolveVelocity();
            for (int i = 0; i < cables.Length; i++) cables[i].SolveVelocity();
        }
    }
    // Solve the two contact normals together so a flat box has balanced support.
    // Enumerate the four complementarity cases: both active, either active, neither.
    private static bool SolveNormalBlock(in ContactManifold m, ref ContactConstraint c1, ref ContactConstraint c2)
    {
        float k11 = 1 / c1.NormalMass, k22 = 1 / c2.NormalMass;
        float k12 = m.A.InverseMass + m.B.InverseMass +
            m.A.InverseInertia * Vec2.Cross(c1.RA, m.Normal) * Vec2.Cross(c2.RA, m.Normal) +
            m.B.InverseInertia * Vec2.Cross(c1.RB, m.Normal) * Vec2.Cross(c2.RB, m.Normal);
        float determinant = k11 * k22 - k12 * k12;
        if (determinant <= 1e-6f || k11 * k11 > 1000 * determinant) return false;
        float b1 = Vec2.Dot(RelativeVelocity(m.A, m.B, c1.RA, c1.RB), m.Normal) - c1.Bounce -
            k11 * c1.NormalImpulse - k12 * c2.NormalImpulse;
        float b2 = Vec2.Dot(RelativeVelocity(m.A, m.B, c2.RA, c2.RB), m.Normal) - c2.Bounce -
            k12 * c1.NormalImpulse - k22 * c2.NormalImpulse;
        float x1 = (k12 * b2 - k22 * b1) / determinant;
        float x2 = (k12 * b1 - k11 * b2) / determinant;
        if (x1 < 0 || x2 < 0)
        {
            x1 = -b1 / k11; x2 = 0;
            if (x1 < 0 || k12 * x1 + b2 < 0)
            {
                x1 = 0; x2 = -b2 / k22;
                if (x2 < 0 || k12 * x2 + b1 < 0)
                {
                    x1 = x2 = 0;
                    if (b1 < 0 || b2 < 0) return false;
                }
            }
        }
        Apply(m, c1, m.Normal * (x1 - c1.NormalImpulse));
        Apply(m, c2, m.Normal * (x2 - c2.NormalImpulse));
        c1.NormalImpulse = x1; c2.NormalImpulse = x2; return true;
    }
    private static float EffectiveMass(RigidBody a, RigidBody b, Vec2 ra, Vec2 rb, Vec2 direction)
    {
        float ca = Vec2.Cross(ra, direction), cb = Vec2.Cross(rb, direction);
        float inverse = a.InverseMass + b.InverseMass + ca * ca * a.InverseInertia + cb * cb * b.InverseInertia;
        return inverse > 0 ? 1 / inverse : 0;
    }
    private static Vec2 RelativeVelocity(RigidBody a, RigidBody b, Vec2 ra, Vec2 rb) =>
        b.Velocity + Vec2.Cross(b.AngularVelocity, rb) - a.Velocity - Vec2.Cross(a.AngularVelocity, ra);
    private static void Apply(in ContactManifold m, in ContactConstraint c, Vec2 impulse)
    {
        m.A.ApplyImpulse(-impulse, c.RA); m.B.ApplyImpulse(impulse, c.RB);
    }
    // TOI impulses are transient; they must not advance or pollute the per-tick
    // warm-start cache used for persistent stacking contacts.
    public static void SolveImpact(in ContactManifold m)
    {
        Vec2 tangent = new(-m.Normal.Y, m.Normal.X);
        Span<ContactConstraint> points = stackalloc ContactConstraint[2];
        for (int i = 0; i < m.ContactCount; i++)
        {
            ref ContactConstraint c = ref points[i]; c = default;
            c.RA = m.GetPoint(i) - m.A.Position; c.RB = m.GetPoint(i) - m.B.Position;
            c.NormalMass = EffectiveMass(m.A, m.B, c.RA, c.RB, m.Normal);
            c.TangentMass = EffectiveMass(m.A, m.B, c.RA, c.RB, tangent);
            float speed = Vec2.Dot(RelativeVelocity(m.A, m.B, c.RA, c.RB), m.Normal);
            c.Bounce = -MathF.Min(m.A.Restitution, m.B.Restitution) * MathF.Min(speed, 0);
        }
        for (int iteration = 0; iteration < Iterations; iteration++)
        {
            bool block = m.ContactCount == 2 && SolveNormalBlock(m, ref points[0], ref points[1]);
            for (int i = 0; i < m.ContactCount; i++)
            {
                ref ContactConstraint c = ref points[i];
                if (!block)
                {
                    float speed = Vec2.Dot(RelativeVelocity(m.A, m.B, c.RA, c.RB), m.Normal);
                    float previous = c.NormalImpulse;
                    c.NormalImpulse = MathF.Max(0, previous + (c.Bounce - speed) * c.NormalMass);
                    Apply(m, c, m.Normal * (c.NormalImpulse - previous));
                }
                float candidate = c.TangentImpulse - Vec2.Dot(RelativeVelocity(m.A, m.B, c.RA, c.RB), tangent) * c.TangentMass;
                float staticLimit = MathF.Sqrt(m.A.StaticFriction * m.B.StaticFriction) * c.NormalImpulse;
                float dynamicLimit = MathF.Sqrt(m.A.DynamicFriction * m.B.DynamicFriction) * c.NormalImpulse;
                float next = MathF.Abs(candidate) <= staticLimit ? candidate : Math.Clamp(candidate, -dynamicLimit, dynamicLimit);
                Apply(m, c, tangent * (next - c.TangentImpulse)); c.TangentImpulse = next;
            }
        }
    }
    public static void CorrectPositions(ReadOnlySpan<ContactManifold> manifolds)
    {
        for (int i = 0; i < manifolds.Length; i++)
        {
            ref readonly ContactManifold m = ref manifolds[i];
            float inverseMass = m.A.InverseMass + m.B.InverseMass;
            if (inverseMass <= 0) continue;
            Vec2 correction = m.Normal * (CorrectionFactor * MathF.Max(m.Penetration - PenetrationSlop, 0) / inverseMass);
            m.A.Position -= correction * m.A.InverseMass;
            m.B.Position += correction * m.B.InverseMass;
        }
    }
}
