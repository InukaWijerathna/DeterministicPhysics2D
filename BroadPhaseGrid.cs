namespace DeterministicPhysics2D;

public readonly struct BodyPair
{
    public readonly int A, B;
    public BodyPair(int a, int b) { A = a; B = b; }
}

public sealed class BroadPhaseGrid
{
    private readonly struct CellEntry
    {
        public readonly int X, Y, Body;
        public CellEntry(int x, int y, int body) { X = x; Y = y; Body = body; }
    }
    private static int Compare(CellEntry a, CellEntry b)
    {
        int result = a.X.CompareTo(b.X);
        if (result == 0) result = a.Y.CompareTo(b.Y);
        return result == 0 ? a.Body.CompareTo(b.Body) : result;
    }
    public float CellSize { get; }
    private readonly int capacity;
    private readonly CellEntry[] entries;
    private readonly long[] pairKeys;
    private readonly bool[] seen;
    private readonly AABB[] bounds;
    private readonly int[] bucketHeads, nextEntry;
    private int entryCount, pairCount;
    public int PairCount => pairCount;
    public int EntryCount => entryCount;
    public BroadPhaseGrid(int maxBodies, float cellSize = 64)
    {
        if (maxBodies < 2 || !float.IsFinite(cellSize) || cellSize <= 0) throw new ArgumentOutOfRangeException();
        capacity = maxBodies; CellSize = cellSize;
        entries = new CellEntry[maxBodies * 64];
        int bucketCount = 1;
        while (bucketCount < maxBodies * 128) bucketCount <<= 1;
        bucketHeads = new int[bucketCount]; nextEntry = new int[entries.Length];
        pairKeys = new long[maxBodies * (maxBodies - 1) / 2];
        seen = new bool[maxBodies * maxBodies]; bounds = new AABB[maxBodies];
    }
    public BodyPair GetPair(int index)
    {
        long key = pairKeys[index]; return new((int)(key >> 32), (int)key);
    }
    public void GetCell(int index, out int x, out int y)
    { x = entries[index].X; y = entries[index].Y; }
    public void Build(ReadOnlySpan<RigidBody> bodies, float sweepDuration = 0, float contactMargin = 0)
    {
        entryCount = pairCount = 0;
        Array.Clear(seen);
        Array.Fill(bucketHeads, -1);
        for (int i = 0; i < bodies.Length; i++)
        {
            AABB box = sweepDuration > 0 ? bodies[i].SweptBounds(sweepDuration) : bodies[i].Bounds();
            if (contactMargin > 0)
            { Vec2 padding = new(contactMargin, contactMargin); box = new(box.Min - padding, box.Max + padding); }
            bounds[i] = box;
            int x0 = Cell(box.Min.X), x1 = Cell(box.Max.X);
            int y0 = Cell(box.Min.Y), y1 = Cell(box.Max.Y);
            long count = ((long)x1 - x0 + 1) * ((long)y1 - y0 + 1);
            if (count < 0 || count > entries.Length - entryCount)
                throw new InvalidOperationException("Spatial grid capacity exceeded. Reduce body sizes or increase capacity.");
            for (int x = x0; x <= x1; x++)
                for (int y = y0; y <= y1; y++)
                {
                    int bucket = HashCell(x, y);
                    for (int other = bucketHeads[bucket]; other >= 0; other = nextEntry[other])
                    {
                        CellEntry entry = entries[other];
                        // Hash collisions must never merge different spatial cells.
                        if (entry.X != x || entry.Y != y) continue;
                        int a = entry.Body, b = i;
                        int slot = a * capacity + b;
                        if (seen[slot]) continue;
                        seen[slot] = true;
                        if ((bodies[a].IsStatic && bodies[b].IsStatic) ||
                            (bodies[a].CollisionGroup != 0 && bodies[a].CollisionGroup == bodies[b].CollisionGroup) ||
                            !bounds[a].Overlaps(bounds[b])) continue;
                        pairKeys[pairCount++] = ((long)a << 32) | (uint)b;
                    }
                    entries[entryCount] = new(x, y, i);
                    nextEntry[entryCount] = bucketHeads[bucket];
                    bucketHeads[bucket] = entryCount++;
                }
        }
        Array.Sort(pairKeys, 0, pairCount);
        // Canonical cell traversal for diagnostics; hash chains are rebuilt next tick.
        SortEntries();
    }
    private int HashCell(int x, int y) =>
        (int)(unchecked((uint)x * 73856093u ^ (uint)y * 19349663u) & (uint)(bucketHeads.Length - 1));
    // Array.Sort<T> with a custom comparer creates a Comparison<T> delegate on
    // each call on .NET 8. An in-place heapsort has no hidden managed allocations.
    private void SortEntries()
    {
        for (int root = entryCount / 2 - 1; root >= 0; root--) SiftDown(root, entryCount);
        for (int end = entryCount - 1; end > 0; end--)
        {
            (entries[0], entries[end]) = (entries[end], entries[0]);
            SiftDown(0, end);
        }
    }
    private void SiftDown(int root, int count)
    {
        while (root * 2 + 1 < count)
        {
            int child = root * 2 + 1;
            if (child + 1 < count && Compare(entries[child], entries[child + 1]) < 0) child++;
            if (Compare(entries[root], entries[child]) >= 0) return;
            (entries[root], entries[child]) = (entries[child], entries[root]);
            root = child;
        }
    }
    private int Cell(float coordinate)
    {
        float value = MathF.Floor(coordinate / CellSize);
        if (!float.IsFinite(value) || value < -1_000_000 || value > 1_000_000)
            throw new InvalidOperationException("Body escaped supported spatial grid coordinates.");
        return (int)value;
    }
}
