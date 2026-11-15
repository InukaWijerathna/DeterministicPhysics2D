namespace DeterministicPhysics2D;

/// <summary>Allocation-free conservative advancement for separated convex shapes.</summary>
public static class ContinuousCollision
{
    public const float Tolerance = 0.002f;
    public const int MaxAdvancementIterations = 96, MaxEvents = 32;

    public static float MotionBound(RigidBody a, RigidBody b) =>
        ((a.IsStatic ? Vec2.Zero : a.Velocity) - (b.IsStatic ? Vec2.Zero : b.Velocity)).Length +
        AngularBound(a) + AngularBound(b);
    private static float AngularBound(RigidBody b) => b.IsStatic || b.Shape == ShapeType.Circle ? 0 : MathF.Abs(b.AngularVelocity) * b.SweepRadius;

    // Returns a conservative event time. An unresolved cast requests a motion
    // clamp, rather than integrating blindly past a possible impact.
    public static bool Cast(RigidBody a, RigidBody b, float duration,
        out float time, out ContactManifold contact, out bool converged)
    {
        time = 0; contact = default; converged = true;
        float bound = MotionBound(a, b);
        if (bound * duration <= 0.25f * MathF.Min(a.MinimumExtent, b.MinimumExtent)) return false;
        // Existing support contacts, including the TOI separation shell, belong
        // to the persistent solver. Recasting these would repeatedly clamp an
        // articulated body's motion before its joints could transfer the load.
        if (CollisionDetection.Collide(a, b, out _, Tolerance * 1.5f)) return false;
        Vec2 startA = a.Position, startB = b.Position;
        float angleA = a.Angle, angleB = b.Angle;
        try
        {
            for (int iteration = 0; iteration < MaxAdvancementIterations; iteration++)
            {
                if (!a.IsStatic) { a.Position = startA + a.Velocity * time; a.Angle = angleA + a.AngularVelocity * time; }
                if (!b.IsStatic) { b.Position = startB + b.Velocity * time; b.Angle = angleB + b.AngularVelocity * time; }
                float distance = Distance(a, b, out contact);
                if (distance <= Tolerance)
                {
                    // Recover face normals and both clipped points in the
                    // contact shell instead of amplifying distance-roundoff.
                    if (CollisionDetection.Collide(a, b, out ContactManifold supported, Tolerance * 1.5f)) contact = supported;
                    float closing = Vec2.Dot(b.PointVelocity(contact.Point1) - a.PointVelocity(contact.Point1), contact.Normal);
                    // A departing pair must not consume the entire event budget at t=0.
                    if (closing < -0.001f) return true;
                    if (AngularBound(a) + AngularBound(b) > 0)
                    {
                        // A rotating feature can depart and re-enter later in
                        // this interval. A zero-distance advancement is unsafe;
                        // request the bounded fallback instead of skipping it.
                        converged = false; return true;
                    }
                    return false;
                }
                float advance = (distance - Tolerance * 0.5f) / bound;
                if (time + advance > duration) return false;
                if (time + advance <= time) { converged = false; return true; }
                time += advance;
            }
            converged = false; return true;
        }
        finally
        {
            a.Position = startA; a.Angle = angleA; b.Position = startB; b.Angle = angleB;
        }
    }

    public static float Distance(RigidBody a, RigidBody b, out ContactManifold contact)
    {
        if (CollisionDetection.Collide(a, b, out contact)) return 0;
        if (a.Shape == ShapeType.Circle && b.Shape == ShapeType.Circle)
        {
            Vec2 delta = b.Position - a.Position; float length = delta.Length;
            Vec2 normal = length > 1e-8f ? delta / length : new(1, 0);
            Vec2 pa = a.Position + normal * a.Radius, pb = b.Position - normal * b.Radius;
            contact = new(a, b, normal, 0, (pa + pb) * 0.5f);
            return MathF.Max(0, length - a.Radius - b.Radius);
        }
        if (a.Shape == ShapeType.Circle || b.Shape == ShapeType.Circle)
        {
            bool flip = b.Shape == ShapeType.Circle;
            RigidBody circle = flip ? b : a, polygon = flip ? a : b;
            Vec2 closest = default; float best = float.MaxValue;
            for (int i = 0; i < polygon.LocalVertices.Length; i++)
            {
                Vec2 p = ClosestOnSegment(circle.Position, polygon.WorldVertex(i), polygon.WorldVertex((i + 1) % polygon.LocalVertices.Length));
                float d = (p - circle.Position).LengthSquared;
                if (d < best) { best = d; closest = p; }
            }
            float length = MathF.Sqrt(best); Vec2 normal = (closest - circle.Position).Normalized();
            Vec2 point = (closest + circle.Position + normal * circle.Radius) * 0.5f;
            contact = new(a, b, flip ? -normal : normal, 0, point);
            return MathF.Max(0, length - circle.Radius);
        }
        Span<Vec2> va = stackalloc Vec2[32]; Span<Vec2> vb = stackalloc Vec2[32];
        int na = a.LocalVertices.Length, nb = b.LocalVertices.Length;
        for (int i = 0; i < na; i++) va[i] = a.WorldVertex(i);
        for (int i = 0; i < nb; i++) vb[i] = b.WorldVertex(i);
        float minimum = float.MaxValue; Vec2 sumA = default, sumB = default; int matches = 0;
        for (int i = 0; i < na; i++)
            for (int j = 0; j < nb; j++)
            {
                Consider(va[i], ClosestOnSegment(va[i], vb[j], vb[(j + 1) % nb]), ref minimum, ref sumA, ref sumB, ref matches);
                Consider(ClosestOnSegment(vb[j], va[i], va[(i + 1) % na]), vb[j], ref minimum, ref sumA, ref sumB, ref matches);
            }
        Vec2 pointA = sumA / matches, pointB = sumB / matches;
        contact = new(a, b, (pointB - pointA).Normalized(), 0, (pointA + pointB) * 0.5f);
        return MathF.Sqrt(minimum);
    }
    private static Vec2 ClosestOnSegment(Vec2 point, Vec2 a, Vec2 b)
    {
        Vec2 edge = b - a;
        return a + edge * Math.Clamp(Vec2.Dot(point - a, edge) / edge.LengthSquared, 0, 1);
    }
    private static void Consider(Vec2 a, Vec2 b, ref float best, ref Vec2 sumA, ref Vec2 sumB, ref int count)
    {
        float d = (b - a).LengthSquared;
        // Average equal-distance feature pairs to put face contacts at their center.
        float epsilon = 1e-6f;
        if (d < best - epsilon) { best = d; sumA = a; sumB = b; count = 1; }
        else if (MathF.Abs(d - best) <= epsilon) { sumA += a; sumB += b; count++; }
    }
}
