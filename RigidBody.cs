namespace DeterministicPhysics2D;

public enum ShapeType { Circle, Polygon }

public sealed class RigidBody
{
    public int Id { get; internal set; } = -1;
    public ShapeType Shape { get; }
    // Equal nonzero groups suppress self-collision, including CCD candidates.
    public int CollisionGroup { get; internal set; }
    public RagdollPart RagdollPart { get; internal set; }
    private readonly Vec2[] vertices, normals;
    public ReadOnlySpan<Vec2> LocalVertices => vertices;
    public ReadOnlySpan<Vec2> FaceNormals => normals;
    public float Radius { get; }
    public float SweepRadius { get; }
    public float MinimumExtent { get; }
    public Vec2 Position, Velocity, Force;
    public float Angle, AngularVelocity, Torque;
    public float Mass { get; }
    public float InverseMass { get; }
    public float Inertia { get; }
    public float InverseInertia { get; }
    public float Restitution { get; set; } = 0.1f;
    public float StaticFriction { get; set; } = 0.65f;
    public float DynamicFriction { get; set; } = 0.45f;
    public bool IsStatic => InverseMass == 0;
    public Mat22 Rotation => new(Angle);

    private RigidBody(ShapeType shape, Vec2 position, float mass, float inertia,
        float radius, Vec2[] vertices, Vec2[] normals)
    {
        if (!float.IsFinite(mass) || mass < 0) throw new ArgumentOutOfRangeException(nameof(mass));
        Shape = shape; Position = position; Mass = mass; Inertia = mass > 0 ? inertia : 0;
        InverseMass = mass > 0 ? 1 / mass : 0;
        InverseInertia = Inertia > 0 ? 1 / Inertia : 0;
        Radius = radius; this.vertices = vertices; this.normals = normals;
        if (shape == ShapeType.Circle) { SweepRadius = MinimumExtent = radius; }
        else
        {
            float radius2 = 0, extent = float.PositiveInfinity;
            for (int i = 0; i < vertices.Length; i++)
            {
                radius2 = MathF.Max(radius2, vertices[i].LengthSquared);
                extent = MathF.Min(extent, Vec2.Dot(normals[i], vertices[i]));
            }
            SweepRadius = MathF.Sqrt(radius2); MinimumExtent = extent;
        }
    }

    public static RigidBody Circle(Vec2 position, float radius, float mass)
    {
        if (!float.IsFinite(radius) || radius <= 0) throw new ArgumentOutOfRangeException(nameof(radius));
        return new(ShapeType.Circle, position, mass, 0.5f * mass * radius * radius,
            radius, Array.Empty<Vec2>(), Array.Empty<Vec2>());
    }

    public static RigidBody Box(Vec2 position, float width, float height, float mass, float angle = 0)
    {
        if (!float.IsFinite(width) || !float.IsFinite(height) || width <= 0 || height <= 0)
            throw new ArgumentOutOfRangeException(nameof(width));
        Vec2[] v = { new(-width / 2, -height / 2), new(width / 2, -height / 2),
            new(width / 2, height / 2), new(-width / 2, height / 2) };
        Vec2[] n = { new(0, -1), new(1, 0), new(0, 1), new(-1, 0) };
        return new(ShapeType.Polygon, position, mass, mass * (width * width + height * height) / 12,
            0, v, n) { Angle = angle };
    }

    // Vertices must describe a strictly convex CCW polygon. Position is its center of mass.
    public static RigidBody Polygon(Vec2 position, ReadOnlySpan<Vec2> points, float mass, float angle = 0)
    {
        if (points.Length < 3 || points.Length > 32) throw new ArgumentException("Use 3..32 convex vertices.");
        Vec2[] v = points.ToArray(), n = new Vec2[points.Length];
        float area2 = 0; Vec2 centroid = default;
        for (int i = 0; i < v.Length; i++)
        {
            Vec2 a = v[i], b = v[(i + 1) % v.Length];
            if (!float.IsFinite(a.X) || !float.IsFinite(a.Y)) throw new ArgumentException("Non-finite vertex.");
            float cross = Vec2.Cross(a, b); area2 += cross; centroid += (a + b) * cross;
        }
        if (area2 <= 1e-5f) throw new ArgumentException("Polygon must have positive CCW area.");
        centroid /= 3 * area2;
        for (int i = 0; i < v.Length; i++) v[i] -= centroid;
        float inertiaIntegral = 0;
        for (int i = 0; i < v.Length; i++)
        {
            Vec2 a = v[i], b = v[(i + 1) % v.Length], edge = b - a;
            if (edge.LengthSquared < 1e-8f) throw new ArgumentException("Degenerate edge.");
            n[i] = new Vec2(edge.Y, -edge.X).Normalized();
            for (int j = 0; j < v.Length; j++)
                if (j != i && j != (i + 1) % v.Length && Vec2.Cross(edge, v[j] - a) <= 1e-5f)
                    throw new ArgumentException("Polygon must be strictly convex and CCW.");
            inertiaIntegral += Vec2.Cross(a, b) * (Vec2.Dot(a, a) + Vec2.Dot(a, b) + Vec2.Dot(b, b));
        }
        return new(ShapeType.Polygon, position, mass, mass * inertiaIntegral / (6 * area2),
            0, v, n) { Angle = angle };
    }

    public Vec2 WorldVertex(int i) => Position + Rotation * vertices[i];
    public Vec2 ToLocal(Vec2 point) => Rotation.Transpose() * (point - Position);
    public Vec2 ToWorld(Vec2 point) => Position + Rotation * point;
    public Vec2 PointVelocity(Vec2 point) => Velocity + Vec2.Cross(AngularVelocity, point - Position);
    public void ApplyForce(Vec2 force, Vec2 point)
    {
        if (IsStatic) return;
        Force += force; Torque += Vec2.Cross(point - Position, force);
    }
    public void ApplyImpulse(Vec2 impulse, Vec2 offset)
    {
        Velocity += impulse * InverseMass;
        AngularVelocity += InverseInertia * Vec2.Cross(offset, impulse);
    }
    public bool Contains(Vec2 point)
    {
        Vec2 p = ToLocal(point);
        if (Shape == ShapeType.Circle) return p.LengthSquared <= Radius * Radius;
        for (int i = 0; i < vertices.Length; i++)
            if (Vec2.Dot(normals[i], p - vertices[i]) > 0) return false;
        return true;
    }
    public AABB Bounds()
    {
        if (Shape == ShapeType.Circle)
        { Vec2 r = new(Radius, Radius); return new(Position - r, Position + r); }
        Mat22 rotation = Rotation;
        Vec2 first = Position + rotation * vertices[0];
        float minX = first.X, maxX = first.X, minY = first.Y, maxY = first.Y;
        for (int i = 1; i < vertices.Length; i++)
        {
            Vec2 p = Position + rotation * vertices[i];
            minX = MathF.Min(minX, p.X); maxX = MathF.Max(maxX, p.X);
            minY = MathF.Min(minY, p.Y); maxY = MathF.Max(maxY, p.Y);
        }
        return new(new(minX, minY), new(maxX, maxY));
    }
    public AABB SweptBounds(float duration)
    {
        AABB start = Bounds();
        if (IsStatic || duration <= 0) return start;
        Vec2 travel = Velocity * duration;
        float rotationPadding = Shape == ShapeType.Circle ? 0 :
            MathF.Min(MathF.Abs(AngularVelocity) * duration, 2) * SweepRadius;
        Vec2 padding = new(rotationPadding + ContinuousCollision.Tolerance, rotationPadding + ContinuousCollision.Tolerance);
        return new(new Vec2(MathF.Min(start.Min.X, start.Min.X + travel.X), MathF.Min(start.Min.Y, start.Min.Y + travel.Y)) - padding,
            new Vec2(MathF.Max(start.Max.X, start.Max.X + travel.X), MathF.Max(start.Max.Y, start.Max.Y + travel.Y)) + padding);
    }
}
