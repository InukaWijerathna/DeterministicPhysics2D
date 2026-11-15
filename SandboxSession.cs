using System.Text.Json;

namespace DeterministicPhysics2D;

public enum InputKind { Spawn, Grab, MoveTarget, Release, Reset, SpawnRagdoll, CableAnchor, CancelCable, CutCable }

public readonly record struct TickInput(long Tick, InputKind Kind, float X, float Y);

/// <summary>Tick-indexed command recording and a presentation clock independent of resets.</summary>
public sealed class SandboxSession
{
    public const int MaxInputs = 65_536;
    private readonly TickInput[] inputs = new TickInput[MaxInputs];
    private int count, cursor;
    private Random random = new(42);
    private double accumulator;
    private long recordedTicks;
    private string expectedPositions = "", expectedKinematics = "";
    private bool paused;
    public PhysicsWorld World { get; } = new();
    public long TimelineTick { get; private set; }
    public bool Paused { get => paused; set => paused = value || PlaybackFinished; }
    public bool Playback { get; private set; }
    public bool PlaybackFinished => Playback && TimelineTick == recordedTicks;
    public bool ReplayVerified { get; private set; }
    public int InputCount => count;
    public string Status { get; private set; } = "Recording inputs";
    public string Tick600Hash { get; private set; } = "";
    public bool HasCableAnchor { get; private set; }
    public RigidBody? CableAnchorBody { get; private set; }
    public Vec2 CableLocalAnchor { get; private set; }
    public Vec2 CableAnchor => CableAnchorBody is null ? CableLocalAnchor : CableAnchorBody.ToWorld(CableLocalAnchor);
    public SandboxSession() => BeginRecording();

    public void BeginRecording()
    {
        random = SandboxScene.Reset(World); count = cursor = 0;
        ClearCableSelection();
        accumulator = 0; recordedTicks = TimelineTick = 0;
        Playback = ReplayVerified = false; Tick600Hash = "";
        expectedPositions = expectedKinematics = ""; Status = "Recording inputs";
    }
    public bool Queue(InputKind kind, Vec2 position = default)
    {
        if (Playback) return false;
        if (!Enum.IsDefined(kind) || !float.IsFinite(position.X) || !float.IsFinite(position.Y))
            throw new ArgumentException("Invalid tick input.");
        TickInput input = new(TimelineTick, kind, position.X, position.Y);
        // Multiple mouse samples before a fixed step need only their final target.
        if (kind == InputKind.MoveTarget && count > cursor && inputs[count - 1].Tick == TimelineTick && inputs[count - 1].Kind == kind)
        { inputs[count - 1] = input; return true; }
        if (count == MaxInputs) { Status = "Input log full: F5 starts a new recording"; return false; }
        inputs[count++] = input; return true;
    }
    public int Advance(float frameTime)
    {
        if (!float.IsFinite(frameTime) || frameTime < 0) throw new ArgumentOutOfRangeException(nameof(frameTime));
        if (Paused || PlaybackFinished) return 0;
        accumulator += Math.Min(frameTime, 0.25f);
        int steps = 0;
        while (accumulator >= PhysicsWorld.FixedDeltaTime && !Paused)
        { StepOne(); accumulator -= PhysicsWorld.FixedDeltaTime; steps++; }
        return steps;
    }
    public void SingleStep()
    {
        if (!Paused || PlaybackFinished) return;
        StepOne();
    }
    private void StepOne()
    {
        while (cursor < count && inputs[cursor].Tick == TimelineTick)
        {
            TickInput input = inputs[cursor++]; Vec2 point = new(input.X, input.Y);
            switch (input.Kind)
            {
                case InputKind.Spawn:
                    if (World.Bodies.Length < PhysicsWorld.MaxBodies) SandboxScene.Spawn(World, random, point);
                    break;
                case InputKind.SpawnRagdoll:
                    if (!RagdollFactory.TrySpawn(World, point)) Status = "Ragdoll needs 11 free bodies and 10 free joints";
                    break;
                case InputKind.Grab: World.Grab(point); break;
                case InputKind.MoveTarget: World.MouseTarget = point; break;
                case InputKind.Release: World.ReleaseGrab(); break;
                case InputKind.CableAnchor: SelectCableAnchor(point); break;
                case InputKind.CancelCable:
                    ClearCableSelection(); Status = "Cable selection cancelled"; break;
                case InputKind.CutCable:
                    Status = World.CutCable(point) ? "Cable cut" : "No cable near cursor"; break;
                case InputKind.Reset:
                    random = SandboxScene.Reset(World); ClearCableSelection();
                    Tick600Hash = ""; Status = "Recording inputs"; break;
            }
        }
        World.Step(); TimelineTick++;
        if (!Playback) recordedTicks = TimelineTick;
        if (World.Tick == 600)
        {
            Tick600Hash = World.ComputeStateHash();
            Console.WriteLine($"Timeline {TimelineTick} | simulation tick 600 | SHA-256 {Tick600Hash}");
        }
        if (PlaybackFinished)
        {
            ReplayVerified = expectedPositions == World.ComputeStateHash() && expectedKinematics == World.ComputeKinematicHash();
            Paused = true;
            Status = ReplayVerified ? "Replay verified: positions and velocities match" : "REPLAY MISMATCH: state differs";
            Console.WriteLine(Status);
        }
    }
    public bool BeginReplay()
    {
        if (recordedTicks == 0) { Status = "Run at least one tick before replaying"; return false; }
        if (!Playback)
        {
            expectedPositions = World.ComputeStateHash(); expectedKinematics = World.ComputeKinematicHash();
        }
        random = SandboxScene.Reset(World); TimelineTick = 0; cursor = 0; accumulator = 0;
        ClearCableSelection();
        Playback = true; ReplayVerified = false; Paused = false; Tick600Hash = "";
        Status = $"Replaying {recordedTicks} ticks"; return true;
    }
    private void ClearCableSelection()
    { HasCableAnchor = false; CableAnchorBody = null; CableLocalAnchor = default; }
    private void SelectCableAnchor(Vec2 point)
    {
        RigidBody? body = World.PickBody(point);
        if (!HasCableAnchor)
        {
            CableAnchorBody = body; CableLocalAnchor = body is null ? point : body.ToLocal(point);
            HasCableAnchor = true; Status = "Cable: select second object or fixed point"; return;
        }
        if (body is not null && ReferenceEquals(body, CableAnchorBody))
        { Status = "Cable: choose a different object"; return; }
        if ((body?.InverseMass ?? 0) + (CableAnchorBody?.InverseMass ?? 0) == 0)
        { Status = "Cable: at least one endpoint must be a movable object"; return; }
        Vec2 start = CableAnchor; float length = (point - start).Length;
        if (length < 4) { Status = "Cable: endpoints must be at least 4 pixels apart"; return; }
        if (World.Cables.Length == PhysicsWorld.MaxCables)
        { Status = "Cable capacity reached"; return; }
        World.AddCable(new(CableAnchorBody, body, start, point, length));
        ClearCableSelection(); Status = "Cable tied / X near a cable to cut";
    }
    public void Save(string path)
    {
        if (recordedTicks == 0) throw new InvalidOperationException("There are no completed ticks to save.");
        // Do not save commands queued for a future, unexecuted tick.
        int completed = 0;
        while (completed < count && inputs[completed].Tick < recordedTicks) completed++;
        Recording recording = new()
        {
            Ticks = recordedTicks, Inputs = inputs.AsSpan(0, completed).ToArray(),
            PositionHash = Playback ? expectedPositions : World.ComputeStateHash(),
            KinematicHash = Playback ? expectedKinematics : World.ComputeKinematicHash()
        };
        File.WriteAllText(path, JsonSerializer.Serialize(recording, new JsonSerializerOptions { WriteIndented = true }));
        Status = $"Saved {Path.GetFileName(path)} ({completed} inputs)";
    }
    public void Load(string path)
    {
        if (new FileInfo(path).Length > 16 * 1024 * 1024) throw new InvalidDataException("Replay file exceeds 16 MB.");
        Recording? recording = JsonSerializer.Deserialize<Recording>(File.ReadAllText(path));
        if (recording is null || recording.Version != 1 || recording.Ticks <= 0 || recording.Inputs is null ||
            recording.Inputs.Length > MaxInputs || !ValidHash(recording.PositionHash) || !ValidHash(recording.KinematicHash))
            throw new InvalidDataException("Unsupported or incomplete replay file.");
        long previous = -1;
        foreach (TickInput input in recording.Inputs)
        {
            if (input.Tick < previous || input.Tick < 0 || input.Tick >= recording.Ticks || !Enum.IsDefined(input.Kind) ||
                !float.IsFinite(input.X) || !float.IsFinite(input.Y) || MathF.Abs(input.X) > 100_000 || MathF.Abs(input.Y) > 100_000)
                throw new InvalidDataException("Replay contains an invalid or unordered input.");
            previous = input.Tick;
        }
        // Validation completes before mutating the active session.
        recording.Inputs.CopyTo(inputs, 0); count = recording.Inputs.Length;
        recordedTicks = recording.Ticks; expectedPositions = recording.PositionHash; expectedKinematics = recording.KinematicHash;
        Playback = true; BeginReplay();
    }
    private static bool ValidHash(string? value) => value is { Length: 64 } && value.All(Uri.IsHexDigit);
    public sealed class Recording
    {
        public int Version { get; set; } = 1;
        public long Ticks { get; set; }
        public TickInput[] Inputs { get; set; } = Array.Empty<TickInput>();
        public string PositionHash { get; set; } = "";
        public string KinematicHash { get; set; } = "";
    }
}
