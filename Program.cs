using Raylib_cs;

namespace DeterministicPhysics2D;

internal static class Program
{
    private const int Width = 1280, Height = 720;
    private static readonly Color Background = new(15, 20, 30, 255);
    private static readonly Color Text = new(220, 230, 242, 255);
    private static readonly Color Muted = new(133, 153, 177, 255);
    private static readonly Color Accent = new(99, 224, 205, 255);
    private static readonly Color ContactColor = new(255, 112, 132, 255);

    private static int Main(string[] args)
    {
        if (args.Contains("--self-test")) return SelfTests.Run();
        bool smokeTest = args.Contains("--smoke-test");
        SandboxSession session = new(); PhysicsWorld world = session.World;
        bool showGrid = false, showNormals = false, showVelocity = false;
        bool cableTool = false;
        string fileStatus = "";
        if (smokeTest)
        {
            showGrid = showNormals = showVelocity = true;
            RigidBody smokeCircle = world.Add(RigidBody.Circle(new(300, 220), 24, 1));
            Vec2[] triangle = { new(-22, 18), new(0, -22), new(22, 18) };
            world.Add(RigidBody.Polygon(new(370, 220), triangle, 1));
            world.Grab(new(300, 220));
            session.Queue(InputKind.SpawnRagdoll, new(450, 230));
            world.AddCable(new(null, smokeCircle, new(300, 175), new(300, 220), 70));
        }
        Raylib.SetConfigFlags(ConfigFlags.Msaa4xHint | (smokeTest ? ConfigFlags.HiddenWindow : 0));
        Raylib.InitWindow(Width, Height, "Deterministic Physics 2D | custom engine / fixed 60 Hz");
        Raylib.SetTargetFPS(60);
        int renderedFrames = 0;
        try
        {
            while (!Raylib.WindowShouldClose())
            {
                if (Raylib.IsKeyPressed(KeyboardKey.G)) showGrid = !showGrid;
                if (Raylib.IsKeyPressed(KeyboardKey.N)) showNormals = !showNormals;
                if (Raylib.IsKeyPressed(KeyboardKey.V)) showVelocity = !showVelocity;
                if (Raylib.IsKeyPressed(KeyboardKey.Space)) session.Paused = !session.Paused;
                if (Raylib.IsKeyPressed(KeyboardKey.F5)) { session.BeginRecording(); fileStatus = ""; }
                if (Raylib.IsKeyPressed(KeyboardKey.F6)) { session.BeginReplay(); fileStatus = ""; }
                if (Raylib.IsKeyPressed(KeyboardKey.F7) || Raylib.IsKeyPressed(KeyboardKey.F8))
                {
                    try
                    {
                        if (Raylib.IsKeyPressed(KeyboardKey.F7)) session.Save("replay.json");
                        else session.Load("replay.json");
                        fileStatus = "";
                    }
                    catch (Exception error) when (error is IOException or UnauthorizedAccessException or System.Text.Json.JsonException or InvalidOperationException)
                    { fileStatus = $"Replay file: {error.Message}"; Console.Error.WriteLine(fileStatus); }
                }
                if (Raylib.IsKeyPressed(KeyboardKey.R))
                {
                    session.Queue(InputKind.Reset);
                }
                Vec2 mouse = new(Raylib.GetMouseX(), Raylib.GetMouseY());
                if (Raylib.IsKeyPressed(KeyboardKey.C))
                {
                    cableTool = !cableTool;
                    session.Queue(InputKind.CancelCable); session.Queue(InputKind.Release);
                }
                if (Raylib.IsKeyPressed(KeyboardKey.X) && mouse.Y > 148) session.Queue(InputKind.CutCable, mouse);
                if (Raylib.IsKeyPressed(KeyboardKey.B) && mouse.Y > 148) session.Queue(InputKind.SpawnRagdoll, mouse);
                if (Raylib.IsMouseButtonPressed(MouseButton.Left) && mouse.Y > 148)
                {
                    if (cableTool) session.Queue(InputKind.CableAnchor, mouse);
                    else if (world.Bodies.Length < PhysicsWorld.MaxBodies) session.Queue(InputKind.Spawn, mouse);
                }
                if (cableTool)
                {
                    if (Raylib.IsMouseButtonPressed(MouseButton.Right)) session.Queue(InputKind.CancelCable);
                }
                else
                {
                    if (Raylib.IsMouseButtonPressed(MouseButton.Right)) session.Queue(InputKind.Grab, mouse);
                    if (Raylib.IsMouseButtonDown(MouseButton.Right)) session.Queue(InputKind.MoveTarget, mouse);
                    if (Raylib.IsMouseButtonReleased(MouseButton.Right)) session.Queue(InputKind.Release);
                }
                if (Raylib.IsKeyPressed(KeyboardKey.Period)) session.SingleStep();
                session.Advance(Raylib.GetFrameTime());
                Raylib.BeginDrawing();
                Raylib.ClearBackground(Background);
                if (showGrid) DrawGrid(world);
                foreach (RigidBody body in world.Bodies) DrawBody(body, showGrid, showVelocity);
                foreach (CableJoint cable in world.Cables) DrawCable(cable);
                if (session.HasCableAnchor)
                {
                    Vec2 anchor = session.CableAnchor;
                    Line(anchor, mouse, Accent);
                    Raylib.DrawCircle((int)anchor.X, (int)anchor.Y, 5, Accent);
                    Raylib.DrawCircleLines((int)mouse.X, (int)mouse.Y, 6, Accent);
                }
                foreach (RevoluteJoint joint in world.Joints)
                {
                    Vec2 anchor = (joint.AnchorA + joint.AnchorB) * 0.5f;
                    Raylib.DrawCircle((int)anchor.X, (int)anchor.Y, 3, Accent);
                }
                if (showNormals) DrawContacts(world);
                if (world.GrabbedBody is { } grabbed)
                {
                    Vec2 anchor = grabbed.ToWorld(world.GrabLocalAnchor);
                    Line(anchor, world.MouseTarget, Accent); Raylib.DrawCircle((int)anchor.X, (int)anchor.Y, 4, Accent);
                }
                Raylib.DrawRectangle(0, 0, Width, 148, new Color(21, 29, 43, 255));
                Raylib.DrawText("DETERMINISTIC / 2D", 24, 15, 25, Accent);
                Raylib.DrawText("LMB spawn   B ragdoll   C cable tool   X cut   RMB drag/cancel   R reset", 24, 49, 17, Text);
                Raylib.DrawText($"G grid [{On(showGrid)}]   N contacts [{On(showNormals)}]   V velocity [{On(showVelocity)}]   Tool: {(cableTool ? "CABLE / click two ends" : "SPAWN")}", 24, 77, 16, Muted);
                Raylib.DrawText("SPACE pause    . single step    F5 new recording    F6 replay    F7 save    F8 load", 24, 106, 16, Text);
                Raylib.DrawText($"{(session.Paused ? "PAUSED" : "RUNNING")} / {(session.Playback ? "REPLAY" : "RECORD")}   |   tick {world.Tick}", 807, 22, 17, Text);
                Raylib.DrawText($"{Raylib.GetFPS()} FPS / {world.Bodies.Length} bodies / {world.Cables.Length} cables", 807, 51, 16, Muted);
                Raylib.DrawText($"CCD: {world.CcdEvents} impacts{(world.CcdMotionClamped ? " / MOTION CLAMPED" : "")} | {session.InputCount} inputs", 807, 79, 16, Accent);
                Raylib.DrawText(fileStatus.Length > 0 ? fileStatus : session.Status, 24, Height - 23, 14, Muted);
                Raylib.EndDrawing();
                if (smokeTest && ++renderedFrames >= 4) break;
            }
        }
        finally { Raylib.CloseWindow(); }
        return 0;
    }
    private static string On(bool value) => value ? "on" : "off";
    private static void Line(Vec2 a, Vec2 b, Color color) => Raylib.DrawLine((int)a.X, (int)a.Y, (int)b.X, (int)b.Y, color);
    private static void DrawCable(CableJoint cable)
    {
        Color color = cable.IsTaut ? new Color(249, 195, 99, 255) : new Color(166, 153, 126, 255);
        Vec2 previous = cable.DisplayPoint(0);
        for (int i = 1; i <= 20; i++)
        {
            Vec2 point = cable.DisplayPoint(i / 20f);
            Line(previous, point, color); Line(previous + new Vec2(0, 1), point + new Vec2(0, 1), color);
            previous = point;
        }
        Vec2 a = cable.AnchorA, b = cable.AnchorB;
        Raylib.DrawCircle((int)a.X, (int)a.Y, 4, color); Raylib.DrawCircle((int)b.X, (int)b.Y, 4, color);
    }
    private static void DrawBody(RigidBody body, bool bounds, bool velocity)
    {
        int palette = body.Id % 4;
        Color fill = body.IsStatic ? new(52, 66, 87, 255) : palette switch
        {
            0 => new(60, 158, 159, 255), 1 => new(84, 122, 192, 255),
            2 => new(176, 119, 84, 255), _ => new(125, 102, 176, 255)
        };
        Color outline = body.IsStatic ? Muted : new Color(186, 216, 227, 255);
        if (body.RagdollPart != RagdollPart.None)
            fill = body.RagdollPart switch
            {
                RagdollPart.Head => new(239, 188, 136, 255),
                RagdollPart.Torso => new(62, 173, 164, 255),
                RagdollPart.Pelvis => new(61, 92, 150, 255),
                _ => new(100, 143, 198, 255)
            };
        AABB box = body.Bounds();
        if (box.Max.X < 0 || box.Min.X > Width || box.Max.Y < 0 || box.Min.Y > Height) return;
        if (body.Shape == ShapeType.Circle)
        {
            Raylib.DrawCircle((int)body.Position.X, (int)body.Position.Y, body.Radius, fill);
            Raylib.DrawCircleLines((int)body.Position.X, (int)body.Position.Y, body.Radius, outline);
            if (body.RagdollPart != RagdollPart.Head)
                Line(body.Position, body.Position + body.Rotation * new Vec2(body.Radius, 0), outline);
            if (body.RagdollPart == RagdollPart.Head)
            {
                Vec2 eyeA = body.ToWorld(new(-4, -2)), eyeB = body.ToWorld(new(4, -2));
                Raylib.DrawCircle((int)eyeA.X, (int)eyeA.Y, 2, Background);
                Raylib.DrawCircle((int)eyeB.X, (int)eyeB.Y, 2, Background);
                Line(body.ToWorld(new(-4, 5)), body.ToWorld(new(4, 5)), Background);
            }
        }
        else
        {
            Span<Vec2> vertices = stackalloc Vec2[32];
            int count = body.LocalVertices.Length;
            Mat22 rotation = body.Rotation;
            for (int i = 0; i < count; i++) vertices[i] = body.Position + rotation * body.LocalVertices[i];
            // Convex scanline fill keeps even rendering free of System.Numerics vectors.
            int first = Math.Max(0, (int)MathF.Ceiling(box.Min.Y));
            int last = Math.Min(Height - 1, (int)MathF.Floor(box.Max.Y));
            for (int y = first; y <= last; y++)
            {
                float left = float.PositiveInfinity, right = float.NegativeInfinity;
                float scan = y + 0.5f;
                for (int i = 0; i < count; i++)
                {
                    Vec2 a = vertices[i], b = vertices[(i + 1) % count];
                    if ((a.Y <= scan && b.Y > scan) || (b.Y <= scan && a.Y > scan))
                    {
                        float x = a.X + (scan - a.Y) * (b.X - a.X) / (b.Y - a.Y);
                        left = MathF.Min(left, x); right = MathF.Max(right, x);
                    }
                }
                if (left <= right) Raylib.DrawLine(Math.Clamp((int)MathF.Ceiling(left), 0, Width), y,
                    Math.Clamp((int)MathF.Floor(right), 0, Width), y, fill);
            }
            for (int i = 0; i < count; i++) Line(vertices[i], vertices[(i + 1) % count], outline);
        }
        if (bounds)
            Raylib.DrawRectangleLines((int)box.Min.X, (int)box.Min.Y,
                (int)MathF.Ceiling(box.Max.X - box.Min.X), (int)MathF.Ceiling(box.Max.Y - box.Min.Y), new Color(99, 224, 205, 115));
        if (velocity) DrawArrow(body.Position, body.Velocity * 0.15f, new Color(247, 207, 104, 255));
    }
    private static void DrawGrid(PhysicsWorld world)
    {
        int previousX = int.MinValue, previousY = int.MinValue;
        for (int i = 0; i < world.Grid.EntryCount; i++)
        {
            world.Grid.GetCell(i, out int x, out int y);
            if (x == previousX && y == previousY) continue;
            previousX = x; previousY = y;
            int size = (int)world.Grid.CellSize;
            int screenX = x * size, screenY = y * size;
            if (screenX < -size || screenX > Width || screenY < -size || screenY > Height) continue;
            Raylib.DrawRectangle(screenX, screenY, size, size, new Color(80, 128, 150, 12));
        }
        Color gridColor = new(73, 99, 118, 70);
        for (int x = 0; x < Width; x += (int)world.Grid.CellSize) Raylib.DrawLine(x, 148, x, Height, gridColor);
        for (int y = 0; y < Height; y += (int)world.Grid.CellSize) Raylib.DrawLine(0, y, Width, y, gridColor);
    }
    private static void DrawContacts(PhysicsWorld world)
    {
        foreach (ref readonly ContactManifold m in world.Contacts)
            for (int i = 0; i < m.ContactCount; i++)
            {
                Vec2 p = m.GetPoint(i);
                Raylib.DrawCircle((int)p.X, (int)p.Y, 3, ContactColor);
                DrawArrow(p, m.Normal * 23, ContactColor);
            }
    }
    private static void DrawArrow(Vec2 origin, Vec2 vector, Color color)
    {
        if (vector.LengthSquared < 1) return;
        Vec2 end = origin + vector, direction = vector.Normalized(), side = new(-direction.Y, direction.X);
        Line(origin, end, color);
        Line(end, end - direction * 6 + side * 3, color);
        Line(end, end - direction * 6 - side * 3, color);
    }
}

internal static class SandboxScene
{
    public static Random Reset(PhysicsWorld world)
    {
        world.Clear(); world.Gravity = new(0, 900);
        Random random = new(42);
        world.Add(RigidBody.Box(new(640, 690), 1250, 40, 0));
        world.Add(RigidBody.Box(new(14, 495), 24, 390, 0));
        world.Add(RigidBody.Box(new(1266, 495), 24, 390, 0));
        world.Add(RigidBody.Box(new(190, 465), 270, 20, 0, 0.23f));
        world.Add(RigidBody.Box(new(1090, 465), 270, 20, 0, -0.23f));
        const float size = 28, spacing = 28.3f;
        for (int layer = 0; layer < 10; layer++)
            for (int column = 0; column < 10 - layer; column++)
            {
                float x = 640 + (column - (9 - layer) * 0.5f) * spacing;
                float y = 670 - size / 2 - layer * spacing;
                RigidBody box = RigidBody.Box(new(x, y), size, size, 1 + (float)random.NextDouble() * 0.05f);
                box.Restitution = 0; world.Add(box);
            }
        return random;
    }
    public static void Spawn(PhysicsWorld world, Random random, Vec2 position)
    {
        float radius = 12 + (float)random.NextDouble() * 14;
        float mass = radius * radius / 225;
        RigidBody body;
        if (random.Next(2) == 0) body = RigidBody.Circle(position, radius, mass);
        else
        {
            int count = random.Next(3, 8);
            Span<Vec2> vertices = stackalloc Vec2[7];
            for (int i = 0; i < count; i++)
            {
                float angle = i * (2 * MathF.PI / count);
                vertices[i] = new(MathF.Cos(angle) * radius, MathF.Sin(angle) * radius);
            }
            body = RigidBody.Polygon(position, vertices[..count], mass, (float)random.NextDouble() * 2 * MathF.PI);
        }
        body.Restitution = 0.15f; world.Add(body);
    }
}
