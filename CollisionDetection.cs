namespace DeterministicPhysics2D;

public readonly struct ContactManifold
{
    public readonly RigidBody A, B;
    public readonly Vec2 Normal, Point1, Point2;
    public readonly float Penetration;
    public readonly int ContactCount;
    public ContactManifold(RigidBody a, RigidBody b, Vec2 normal, float penetration,
        Vec2 point1, Vec2 point2 = default, int count = 1)
    { A = a; B = b; Normal = normal; Penetration = penetration; Point1 = point1; Point2 = point2; ContactCount = count; }
    public Vec2 GetPoint(int i) => i == 0 ? Point1 : Point2;
}

public static class CollisionDetection
{
    public static bool Collide(RigidBody a, RigidBody b, out ContactManifold manifold, float margin = 0)
    {
        if (a.Shape == ShapeType.Circle && b.Shape == ShapeType.Circle) return Circles(a, b, out manifold, margin);
        if (a.Shape == ShapeType.Polygon && b.Shape == ShapeType.Polygon) return Polygons(a, b, out manifold, margin);
        if (a.Shape == ShapeType.Circle) return CirclePolygon(a, b, out manifold, margin);
        if (CirclePolygon(b, a, out ContactManifold swapped, margin))
        {
            manifold = new(a, b, -swapped.Normal, swapped.Penetration, swapped.Point1);
            return true;
        }
        manifold = default; return false;
    }
    private static bool Circles(RigidBody a, RigidBody b, out ContactManifold m, float margin)
    {
        Vec2 delta = b.Position - a.Position;
        float radius = a.Radius + b.Radius, distance2 = delta.LengthSquared;
        if (distance2 > (radius + margin) * (radius + margin)) { m = default; return false; }
        float distance = MathF.Sqrt(distance2);
        Vec2 normal = distance > 1e-6f ? delta / distance : new(1, 0);
        Vec2 point = a.Position + normal * (a.Radius - (radius - distance) * 0.5f);
        m = new(a, b, normal, MathF.Max(0, radius - distance), point); return true;
    }
    private static bool CirclePolygon(RigidBody circle, RigidBody polygon, out ContactManifold m, float margin)
    {
        Vec2 center = polygon.ToLocal(circle.Position);
        ReadOnlySpan<Vec2> vertices = polygon.LocalVertices, normals = polygon.FaceNormals;
        float separation = float.NegativeInfinity; int face = 0;
        for (int i = 0; i < vertices.Length; i++)
        {
            float d = Vec2.Dot(normals[i], center - vertices[i]);
            if (d > circle.Radius + margin) { m = default; return false; }
            if (d > separation) { separation = d; face = i; }
        }
        Vec2 a = vertices[face], b = vertices[(face + 1) % vertices.Length];
        Vec2 outward; float penetration; Vec2 localPoint;
        if (separation <= 0)
        {
            outward = normals[face]; penetration = circle.Radius - separation;
            localPoint = center - outward * separation;
        }
        else
        {
            Vec2 edge = b - a;
            float t = Math.Clamp(Vec2.Dot(center - a, edge) / edge.LengthSquared, 0, 1);
            localPoint = a + edge * t;
            Vec2 delta = center - localPoint; float distance2 = delta.LengthSquared;
            if (distance2 > (circle.Radius + margin) * (circle.Radius + margin)) { m = default; return false; }
            float distance = MathF.Sqrt(distance2);
            outward = distance > 1e-6f ? delta / distance : normals[face];
            penetration = MathF.Max(0, circle.Radius - distance);
        }
        Vec2 normal = -(polygon.Rotation * outward);
        Vec2 surface = polygon.ToWorld(localPoint);
        m = new(circle, polygon, normal, penetration,
            (surface + circle.Position + normal * circle.Radius) * 0.5f);
        return true;
    }
    private static float FindSeparation(RigidBody a, RigidBody b, out int face)
    {
        float best = float.NegativeInfinity; face = 0;
        Mat22 rotationA = a.Rotation, rotationB = b.Rotation;
        for (int i = 0; i < a.LocalVertices.Length; i++)
        {
            Vec2 normal = rotationA * a.FaceNormals[i];
            Vec2 point = a.Position + rotationA * a.LocalVertices[i];
            float minimum = float.PositiveInfinity;
            for (int j = 0; j < b.LocalVertices.Length; j++)
            {
                Vec2 other = b.Position + rotationB * b.LocalVertices[j];
                minimum = MathF.Min(minimum, Vec2.Dot(normal, other - point));
            }
            if (minimum > best) { best = minimum; face = i; }
        }
        return best;
    }
    private static bool Polygons(RigidBody a, RigidBody b, out ContactManifold m, float margin)
    {
        float sepA = FindSeparation(a, b, out int faceA);
        if (sepA > margin) { m = default; return false; }
        float sepB = FindSeparation(b, a, out int faceB);
        if (sepB > margin) { m = default; return false; }
        // Bias toward A to avoid reference-face jitter on nearly parallel faces.
        bool flip = sepB > sepA + 0.001f;
        RigidBody reference = flip ? b : a, incident = flip ? a : b;
        int face = flip ? faceB : faceA;
        Vec2 v1 = reference.WorldVertex(face);
        Vec2 v2 = reference.WorldVertex((face + 1) % reference.LocalVertices.Length);
        Vec2 tangent = (v2 - v1).Normalized();
        Vec2 normal = reference.Rotation * reference.FaceNormals[face];
        int incidentFace = 0; float minDot = float.PositiveInfinity;
        for (int i = 0; i < incident.FaceNormals.Length; i++)
        {
            float dot = Vec2.Dot(normal, incident.Rotation * incident.FaceNormals[i]);
            if (dot < minDot) { minDot = dot; incidentFace = i; }
        }
        Span<Vec2> edge = stackalloc Vec2[2];
        Span<Vec2> clipped = stackalloc Vec2[2];
        edge[0] = incident.WorldVertex(incidentFace);
        edge[1] = incident.WorldVertex((incidentFace + 1) % incident.LocalVertices.Length);
        int count = Clip(edge, 2, -tangent, Vec2.Dot(-tangent, v1), clipped);
        if (count == 0) { m = default; return false; }
        count = Clip(clipped, count, tangent, Vec2.Dot(tangent, v2), edge);
        Vec2 p1 = default, p2 = default; int contacts = 0; float penetration = 0;
        for (int i = 0; i < count; i++)
        {
            float distance = Vec2.Dot(normal, edge[i] - v1);
            if (distance > margin + 0.0001f) continue;
            Vec2 p = edge[i] - normal * (distance * 0.5f);
            if (contacts == 0) p1 = p; else p2 = p;
            contacts++; penetration += MathF.Max(-distance, 0);
        }
        if (contacts == 0) { m = default; return false; }
        m = new(a, b, flip ? -normal : normal, penetration / contacts, p1, p2, contacts);
        return true;
    }
    // Sutherland-Hodgman clipping of an incident segment against one side plane.
    private static int Clip(ReadOnlySpan<Vec2> input, int count, Vec2 normal, float offset, Span<Vec2> output)
    {
        float d0 = Vec2.Dot(normal, input[0]) - offset;
        int result = 0;
        if (d0 <= 0) output[result++] = input[0];
        if (count == 1) return result;
        float d1 = Vec2.Dot(normal, input[1]) - offset;
        if (d1 <= 0) output[result++] = input[1];
        if ((d0 < 0 && d1 > 0) || (d0 > 0 && d1 < 0))
            output[result++] = input[0] + (input[1] - input[0]) * (d0 / (d0 - d1));
        return result;
    }
}
