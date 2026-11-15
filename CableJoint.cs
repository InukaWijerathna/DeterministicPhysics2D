namespace DeterministicPhysics2D;

/// <summary>A tension-only maximum-distance constraint. Null endpoints are fixed world anchors.</summary>
public sealed class CableJoint
{
    public RigidBody? A { get; }
    public RigidBody? B { get; }
    public Vec2 LocalAnchorA { get; }
    public Vec2 LocalAnchorB { get; }
    public float MaxLength { get; }
    public Vec2 AnchorA => A is null ? LocalAnchorA : A.ToWorld(LocalAnchorA);
    public Vec2 AnchorB => B is null ? LocalAnchorB : B.ToWorld(LocalAnchorB);
    public float Length => (AnchorB - AnchorA).Length;
    public bool IsTaut => Length >= MaxLength - 0.5f;
    private Vec2 ra, rb, normal;
    private float effectiveMass, bias, tension;

    public CableJoint(RigidBody? a, RigidBody? b, Vec2 anchorA, Vec2 anchorB, float maxLength)
    {
        if ((a is not null && ReferenceEquals(a, b)) || (a?.InverseMass ?? 0) + (b?.InverseMass ?? 0) <= 0 ||
            !float.IsFinite(maxLength) || maxLength <= 0 || !float.IsFinite(anchorA.X) || !float.IsFinite(anchorA.Y) ||
            !float.IsFinite(anchorB.X) || !float.IsFinite(anchorB.Y))
            throw new ArgumentException("A cable needs a dynamic endpoint, distinct bodies, finite anchors and a positive length.");
        A = a; B = b; LocalAnchorA = a is null ? anchorA : a.ToLocal(anchorA);
        LocalAnchorB = b is null ? anchorB : b.ToLocal(anchorB); MaxLength = maxLength;
    }
    private float InverseEffectiveMass(Vec2 offsetA, Vec2 offsetB, Vec2 direction)
    {
        float ca = Vec2.Cross(offsetA, direction), cb = Vec2.Cross(offsetB, direction);
        return (A?.InverseMass ?? 0) + (B?.InverseMass ?? 0) +
            ca * ca * (A?.InverseInertia ?? 0) + cb * cb * (B?.InverseInertia ?? 0);
    }
    internal void Prepare()
    {
        ra = A is null ? default : A.Rotation * LocalAnchorA;
        rb = B is null ? default : B.Rotation * LocalAnchorB;
        Vec2 delta = AnchorB - AnchorA; float length = delta.Length;
        Vec2 nextNormal = length > 1e-6f ? delta / length : default;
        if (Vec2.Dot(normal, nextNormal) < 0.9f || length < MaxLength - 0.01f) tension = 0;
        normal = nextNormal;
        float inverse = InverseEffectiveMass(ra, rb, normal);
        effectiveMass = length > 1e-6f && inverse > 0 ? 1 / inverse : 0;
        float error = length - MaxLength;
        // Predict when a slack cable will become taut during this fixed step.
        // Negative bias allows free motion until the remaining slack is consumed.
        bias = error < 0 ? error / PhysicsWorld.FixedDeltaTime :
            MathF.Min(200, 0.2f * error / PhysicsWorld.FixedDeltaTime);
    }
    private void Apply(float magnitude)
    {
        Vec2 impulse = normal * magnitude;
        A?.ApplyImpulse(impulse, ra); B?.ApplyImpulse(-impulse, rb);
    }
    internal void WarmStart() => Apply(tension);
    internal void SolveVelocity()
    {
        Vec2 va = A is null ? default : A.Velocity + Vec2.Cross(A.AngularVelocity, ra);
        Vec2 vb = B is null ? default : B.Velocity + Vec2.Cross(B.AngularVelocity, rb);
        float previous = tension;
        tension = MathF.Max(0, previous + effectiveMass * (Vec2.Dot(vb - va, normal) + bias));
        Apply(tension - previous);
    }
    internal void CorrectPosition()
    {
        Vec2 delta = AnchorB - AnchorA; float length = delta.Length;
        float error = length - MaxLength;
        if (error <= 0.005f || length < 1e-6f) return;
        Vec2 direction = delta / length;
        Vec2 offsetA = A is null ? default : A.Rotation * LocalAnchorA;
        Vec2 offsetB = B is null ? default : B.Rotation * LocalAnchorB;
        float inverse = InverseEffectiveMass(offsetA, offsetB, direction);
        if (inverse <= 0) return;
        Vec2 impulse = direction * (0.4f * MathF.Min(error - 0.005f, 10) / inverse);
        if (A is not null)
        { A.Position += impulse * A.InverseMass; A.Angle += A.InverseInertia * Vec2.Cross(offsetA, impulse); }
        if (B is not null)
        { B.Position -= impulse * B.InverseMass; B.Angle -= B.InverseInertia * Vec2.Cross(offsetB, impulse); }
    }
    // Visual sag only: the constraint limits endpoint distance and has no rope particles.
    public Vec2 DisplayPoint(float fraction)
    {
        Vec2 a = AnchorA, b = AnchorB;
        float distance2 = (b - a).LengthSquared;
        float sag = MathF.Min(MaxLength * 0.3f, MathF.Sqrt(MathF.Max(0, MaxLength * MaxLength - distance2)) * 0.35f);
        return a + (b - a) * fraction + new Vec2(0, 4 * fraction * (1 - fraction) * sag);
    }
}
