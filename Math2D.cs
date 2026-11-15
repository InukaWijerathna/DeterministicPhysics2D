namespace DeterministicPhysics2D;

public readonly struct Vec2
{
    public readonly float X, Y;
    public Vec2(float x, float y) { X = x; Y = y; }
    public static Vec2 Zero => default;
    public float LengthSquared => X * X + Y * Y;
    public float Length => MathF.Sqrt(LengthSquared);
    public Vec2 Normalized() => LengthSquared > 1e-12f ? this / Length : Zero;
    public static float Dot(Vec2 a, Vec2 b) => a.X * b.X + a.Y * b.Y;
    public static float Cross(Vec2 a, Vec2 b) => a.X * b.Y - a.Y * b.X;
    public static Vec2 Cross(float s, Vec2 v) => new(-s * v.Y, s * v.X);
    public static Vec2 Cross(Vec2 v, float s) => new(s * v.Y, -s * v.X);
    public static Vec2 operator +(Vec2 a, Vec2 b) => new(a.X + b.X, a.Y + b.Y);
    public static Vec2 operator -(Vec2 a, Vec2 b) => new(a.X - b.X, a.Y - b.Y);
    public static Vec2 operator -(Vec2 v) => new(-v.X, -v.Y);
    public static Vec2 operator *(Vec2 v, float s) => new(v.X * s, v.Y * s);
    public static Vec2 operator *(float s, Vec2 v) => v * s;
    public static Vec2 operator /(Vec2 v, float s) => new(v.X / s, v.Y / s);
}

public readonly struct Mat22
{
    public readonly float M00, M01, M10, M11;
    public Mat22(float radians)
    {
        float c = MathF.Cos(radians), s = MathF.Sin(radians);
        M00 = c; M01 = -s; M10 = s; M11 = c;
    }
    private Mat22(float a, float b, float c, float d)
    { M00 = a; M01 = b; M10 = c; M11 = d; }
    public Mat22 Transpose() => new(M00, M10, M01, M11);
    public static Vec2 operator *(Mat22 m, Vec2 v) =>
        new(m.M00 * v.X + m.M01 * v.Y, m.M10 * v.X + m.M11 * v.Y);
}

public readonly struct AABB
{
    public readonly Vec2 Min, Max;
    public AABB(Vec2 min, Vec2 max) { Min = min; Max = max; }
    public bool Overlaps(in AABB other) =>
        Min.X <= other.Max.X && Max.X >= other.Min.X &&
        Min.Y <= other.Max.Y && Max.Y >= other.Min.Y;
}
