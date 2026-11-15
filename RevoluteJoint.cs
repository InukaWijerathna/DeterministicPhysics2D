namespace DeterministicPhysics2D;

/// <summary>A warm-started hinge with a shared anchor and relative angular limits.</summary>
public sealed class RevoluteJoint
{
    public RigidBody A { get; }
    public RigidBody B { get; }
    public Vec2 LocalAnchorA { get; }
    public Vec2 LocalAnchorB { get; }
    public float ReferenceAngle { get; }
    public float LowerAngle { get; }
    public float UpperAngle { get; }
    public Vec2 AnchorA => A.ToWorld(LocalAnchorA);
    public Vec2 AnchorB => B.ToWorld(LocalAnchorB);
    public float RelativeAngle => B.Angle - A.Angle - ReferenceAngle;
    private Vec2 ra, rb, bias, accumulatedImpulse;
    private float k00, k01, k11, inverseDeterminant, angularMass, angularBias, angularImpulse;
    private int limit;

    public RevoluteJoint(RigidBody a, RigidBody b, Vec2 anchor, float lowerAngle, float upperAngle)
    {
        if (ReferenceEquals(a, b) || (a.IsStatic && b.IsStatic) || !float.IsFinite(lowerAngle) ||
            !float.IsFinite(upperAngle) || lowerAngle > upperAngle || !float.IsFinite(anchor.X) || !float.IsFinite(anchor.Y))
            throw new ArgumentException("A hinge requires distinct bodies, a finite anchor and valid limits.");
        A = a; B = b; LocalAnchorA = a.ToLocal(anchor); LocalAnchorB = b.ToLocal(anchor);
        ReferenceAngle = b.Angle - a.Angle; LowerAngle = lowerAngle; UpperAngle = upperAngle;
    }
    private void BuildMass(Vec2 offsetA, Vec2 offsetB)
    {
        float mass = A.InverseMass + B.InverseMass;
        k00 = mass + A.InverseInertia * offsetA.Y * offsetA.Y + B.InverseInertia * offsetB.Y * offsetB.Y;
        k01 = -A.InverseInertia * offsetA.X * offsetA.Y - B.InverseInertia * offsetB.X * offsetB.Y;
        k11 = mass + A.InverseInertia * offsetA.X * offsetA.X + B.InverseInertia * offsetB.X * offsetB.X;
        float determinant = k00 * k11 - k01 * k01;
        inverseDeterminant = determinant > 1e-12f ? 1 / determinant : 0;
    }
    private Vec2 SolveMass(Vec2 value) => new(
        (k11 * value.X - k01 * value.Y) * inverseDeterminant,
        (k00 * value.Y - k01 * value.X) * inverseDeterminant);
    private void Apply(Vec2 impulse)
    { A.ApplyImpulse(-impulse, ra); B.ApplyImpulse(impulse, rb); }
    private void ApplyAngular(float impulse)
    { A.AngularVelocity -= A.InverseInertia * impulse; B.AngularVelocity += B.InverseInertia * impulse; }

    internal void Prepare()
    {
        ra = A.Rotation * LocalAnchorA; rb = B.Rotation * LocalAnchorB;
        BuildMass(ra, rb);
        bias = (B.Position + rb - A.Position - ra) * (0.2f / PhysicsWorld.FixedDeltaTime);
        if (bias.LengthSquared > 200 * 200) bias = bias.Normalized() * 200;
        float inverse = A.InverseInertia + B.InverseInertia;
        angularMass = inverse > 0 ? 1 / inverse : 0;
        float angle = RelativeAngle;
        int next = angle < LowerAngle ? -1 : angle > UpperAngle ? 1 : 0;
        if (next != limit || next == 0) angularImpulse = 0;
        limit = next;
        angularBias = limit == 0 ? 0 : 0.2f / PhysicsWorld.FixedDeltaTime *
            (angle - (limit < 0 ? LowerAngle : UpperAngle));
    }
    internal void WarmStart()
    { Apply(accumulatedImpulse); ApplyAngular(angularImpulse); }
    internal void SolveVelocity()
    {
        Vec2 velocity = B.Velocity + Vec2.Cross(B.AngularVelocity, rb) - A.Velocity - Vec2.Cross(A.AngularVelocity, ra);
        Vec2 impulse = SolveMass(-velocity - bias);
        accumulatedImpulse += impulse; Apply(impulse);
        if (limit == 0) return;
        float delta = -angularMass * (B.AngularVelocity - A.AngularVelocity + angularBias);
        float previous = angularImpulse;
        angularImpulse = limit < 0 ? MathF.Max(0, previous + delta) : MathF.Min(0, previous + delta);
        ApplyAngular(angularImpulse - previous);
    }
    internal void CorrectPosition()
    {
        Vec2 offsetA = A.Rotation * LocalAnchorA, offsetB = B.Rotation * LocalAnchorB;
        Vec2 error = B.Position + offsetB - A.Position - offsetA;
        if (error.LengthSquared > 64) error = error.Normalized() * 8;
        BuildMass(offsetA, offsetB);
        Vec2 impulse = SolveMass(-error * 0.4f);
        A.Position -= impulse * A.InverseMass; B.Position += impulse * B.InverseMass;
        A.Angle -= A.InverseInertia * Vec2.Cross(offsetA, impulse);
        B.Angle += B.InverseInertia * Vec2.Cross(offsetB, impulse);
        float angle = RelativeAngle;
        float angularError = angle - Math.Clamp(angle, LowerAngle, UpperAngle);
        float inverse = A.InverseInertia + B.InverseInertia;
        if (inverse <= 0) return;
        float correction = -Math.Clamp(angularError * 0.4f, -0.15f, 0.15f) / inverse;
        A.Angle -= A.InverseInertia * correction; B.Angle += B.InverseInertia * correction;
    }
}
