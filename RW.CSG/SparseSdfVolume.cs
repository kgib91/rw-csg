using System.Numerics;
using System.Runtime.CompilerServices;

namespace RW.CSG;

/// <summary>
/// Sparse narrow-band SDF volume (VDB-lite). Dense brick-handle grid over slab-allocated bricks.
/// Sign: d &gt; 0 air, d &lt; 0 material. Distances in mm, clamped to ±BandWorld. Storage Half, compute float.
/// Subtraction is monotone: d_new = max(d, −brush).
/// </summary>
public sealed class SparseSdfVolume
{
    public const int Air = -1;    // brick fully cut
    public const int Solid = -2;  // brick untouched material

    public readonly int VoxelsX;
    public readonly int VoxelsY;
    public readonly int VoxelsZ;
    public readonly int BrickSize;          // 4 / 8 / 16
    public readonly int BricksX;
    public readonly int BricksY;
    public readonly int BricksZ;
    public readonly int BrickVoxels;
    public readonly float VoxelSize;        // mm
    public readonly int BandVoxels;         // e.g. 3
    public readonly float BandWorld;        // mm
    public readonly float VolumeSizeX;      // backing-volume extent in mm
    public readonly float VolumeSizeY;
    public readonly float VolumeSizeZ;
    public readonly float StockSizeX;       // physical stock extent in mm
    public readonly float StockSizeY;
    public readonly float StockHeight;      // mm (physical stock extent in Z)

    private readonly int[] _brickIndex;                  // handle | Air | Solid
    private readonly List<Half[]> _slabs = [];
    private const int SlabBricks = 1024;
    private int _brickCount;
    private readonly Stack<int> _freeHandles = new();

    /// <summary>Minimum raise before a voxel is rewritten. Setting this to about one
    /// Half ULP at band scale (BandWorld/1024) suppresses re-stamp drift caused by Half round-down.</summary>
    public float WriteEpsilon = 1e-7f;

    // Coarse whole-volume SDF: 1 cell = 8 voxels regardless of brick size (plan §3).
    public readonly int CoarseX;
    public readonly int CoarseY;
    public readonly int CoarseZ;
    public readonly float CoarseCell;
    public readonly float[] Coarse;
    public bool CoarseEnabled = true;

    public double RemovedVolume;                          // mm³
    public readonly List<int> DirtyBricks = [];           // bricks changed by the last StampBatch

    // Diagnostic counters consumed by the opt-in benchmark suite.
    public long StatMissSkips, StatEngulfs, StatPartialBricks, StatVoxelEvals, StatVoxelWrites,
                StatColumnDists, StatRuns;

    // Binning scratch (persistent, zero steady-state alloc)
    private readonly int[] _binCount;
    private int[] _touched = new int[1 << 16];
    private int _touchedCount;
    private int[] _binStart = new int[1 << 16];
    private int[] _binSegs = new int[1 << 18];
    private double[] _brickRemoved = new double[1 << 16];
    private readonly byte[] _dirtyFaceMask;

    private const byte FaceNegX = 1 << 0;
    private const byte FacePosX = 1 << 1;
    private const byte FaceNegY = 1 << 2;
    private const byte FacePosY = 1 << 3;
    private const byte FaceNegZ = 1 << 4;
    private const byte FacePosZ = 1 << 5;

    [ThreadStatic] private static float[]? _scratch;
    [ThreadStatic] private static int[]? _survivors;
    [ThreadStatic] private static int[]? _groupedSurvivors;
    [ThreadStatic] private static int[]? _groupCount;
    [ThreadStatic] private static int[]? _groupStart;

    public SparseSdfVolume(int voxelsPerSide, float voxelSize, int brickSize = 8, int bandVoxels = 3,
        float? stockHeight = null)
        : this(voxelsPerSide, voxelsPerSide, voxelsPerSide, voxelSize, brickSize, bandVoxels,
            stockHeight ?? voxelsPerSide * voxelSize)
    {
    }

    /// <summary>Creates a rectangular volume whose physical stock fills the full backing grid.</summary>
    public SparseSdfVolume(int voxelsX, int voxelsY, int voxelsZ, float voxelSize,
        int brickSize = 8, int bandVoxels = 3)
        : this(voxelsX, voxelsY, voxelsZ, voxelSize, brickSize, bandVoxels,
            voxelsZ * voxelSize)
    {
    }

    private SparseSdfVolume(int voxelsX, int voxelsY, int voxelsZ, float voxelSize,
        int brickSize, int bandVoxels, float physicalHeight)
    {
        if (voxelsX <= 0 || voxelsY <= 0 || voxelsZ <= 0)
            throw new ArgumentOutOfRangeException(nameof(voxelsX), "Voxel dimensions must be positive.");
        if (!float.IsFinite(voxelSize) || voxelSize <= 0f)
            throw new ArgumentOutOfRangeException(nameof(voxelSize));
        if (brickSize <= 0 || voxelsX % brickSize != 0 || voxelsY % brickSize != 0
            || voxelsZ % brickSize != 0)
            throw new ArgumentException("Every voxel dimension must be divisible by the brick size.", nameof(brickSize));
        if (voxelsX % 8 != 0 || voxelsY % 8 != 0 || voxelsZ % 8 != 0)
            throw new ArgumentException("Every voxel dimension must be divisible by the 8-voxel coarse cell.", nameof(voxelsX));

        VoxelsX = voxelsX;
        VoxelsY = voxelsY;
        VoxelsZ = voxelsZ;
        VoxelSize = voxelSize;
        BrickSize = brickSize;
        BandVoxels = bandVoxels;
        BandWorld = bandVoxels * voxelSize;
        BricksX = voxelsX / brickSize;
        BricksY = voxelsY / brickSize;
        BricksZ = voxelsZ / brickSize;
        BrickVoxels = brickSize * brickSize * brickSize;
        VolumeSizeX = voxelsX * voxelSize;
        VolumeSizeY = voxelsY * voxelSize;
        VolumeSizeZ = voxelsZ * voxelSize;
        StockSizeX = VolumeSizeX;
        StockSizeY = VolumeSizeY;
        if (!float.IsFinite(physicalHeight) || physicalHeight <= 0f || physicalHeight > VolumeSizeZ)
            throw new ArgumentOutOfRangeException(nameof(physicalHeight),
                $"Stock height must be in (0, {VolumeSizeZ}] mm.");
        float brickWorld = BrickSize * VoxelSize;
        float brickLayers = physicalHeight / brickWorld;
        if (MathF.Abs(brickLayers - MathF.Round(brickLayers)) > 1e-4f)
            throw new ArgumentException(
                $"Stock height must align to the {brickWorld} mm brick height.", nameof(physicalHeight));
        StockHeight = physicalHeight;

        int brickCount = checked(BricksX * BricksY * BricksZ);
        _brickIndex = new int[brickCount];
        Array.Fill(_brickIndex, Solid);
        int stockBrickLayers = Math.Clamp(
            (int)MathF.Round(StockHeight / brickWorld), 1, BricksZ);
        if (stockBrickLayers < BricksZ)
        {
            int firstAirBrick = stockBrickLayers * BricksX * BricksY;
            Array.Fill(_brickIndex, Air, firstAirBrick, _brickIndex.Length - firstAirBrick);
        }
        _binCount = new int[_brickIndex.Length];
        _dirtyFaceMask = new byte[_brickIndex.Length];

        CoarseX = voxelsX / 8;
        CoarseY = voxelsY / 8;
        CoarseZ = voxelsZ / 8;
        CoarseCell = 8 * voxelSize;
        Coarse = new float[checked(CoarseX * CoarseY * CoarseZ)];
        InitCoarseBoxSdf();
    }

    private void InitCoarseBoxSdf()
    {
        // Signed distance to the physical stock box (negative inside).
        float sx = StockSizeX;
        float sy = StockSizeY;
        float h = StockHeight;
        for (int z = 0; z < CoarseZ; z++)
            for (int y = 0; y < CoarseY; y++)
                for (int x = 0; x < CoarseX; x++)
                {
                    float px = (x + 0.5f) * CoarseCell, py = (y + 0.5f) * CoarseCell, pz = (z + 0.5f) * CoarseCell;
                    float dx = MathF.Max(-px, px - sx), dy = MathF.Max(-py, py - sy), dz = MathF.Max(-pz, pz - h);
                    float inside = MathF.Min(0, MathF.Max(dx, MathF.Max(dy, dz)));
                    float ox = MathF.Max(dx, 0), oy = MathF.Max(dy, 0), oz = MathF.Max(dz, 0);
                    Coarse[(z * CoarseY + y) * CoarseX + x] = inside + MathF.Sqrt(ox * ox + oy * oy + oz * oz);
                }
    }

    public int BrickCount => _brickIndex.Length;
    public long AllocatedBricks => _brickCount - _freeHandles.Count;
    public long PoolBytes => (long)_slabs.Count * SlabBricks * BrickVoxels * 2;

    /// <summary>
    /// Creates an independent copy suitable for background forward replay. The source must be
    /// between stamp batches; slab copies are parallel because they are disjoint and immutable
    /// for the duration of the clone.
    /// </summary>
    public SparseSdfVolume DeepClone()
    {
        var clone = new SparseSdfVolume(
            VoxelsX, VoxelsY, VoxelsZ, VoxelSize, BrickSize, BandVoxels, StockHeight)
        {
            WriteEpsilon = WriteEpsilon,
            CoarseEnabled = CoarseEnabled,
            RemovedVolume = RemovedVolume,
            _brickCount = _brickCount,
            StatMissSkips = StatMissSkips,
            StatEngulfs = StatEngulfs,
            StatPartialBricks = StatPartialBricks,
            StatVoxelEvals = StatVoxelEvals,
            StatVoxelWrites = StatVoxelWrites,
            StatColumnDists = StatColumnDists,
            StatRuns = StatRuns
        };

        Array.Copy(_brickIndex, clone._brickIndex, _brickIndex.Length);
        Array.Copy(Coarse, clone.Coarse, Coarse.Length);
        clone.DirtyBricks.AddRange(DirtyBricks);

        int[] freeHandles = _freeHandles.ToArray();
        for (int i = freeHandles.Length - 1; i >= 0; i--)
        {
            clone._freeHandles.Push(freeHandles[i]);
        }

        var copiedSlabs = new Half[_slabs.Count][];
        Parallel.For(0, _slabs.Count, i => copiedSlabs[i] = (Half[])_slabs[i].Clone());
        clone._slabs.AddRange(copiedSlabs);
        return clone;
    }

    // ---- brick pool -------------------------------------------------------

    private int AllocHandle()
    {
        if (_freeHandles.TryPop(out int h)) return h;
        h = _brickCount++;
        if (h / SlabBricks >= _slabs.Count) _slabs.Add(new Half[SlabBricks * BrickVoxels]);
        return h;
    }

    private void FreeHandle(int h) => _freeHandles.Push(h);

    private Span<Half> BrickData(int handle)
        => _slabs[handle / SlabBricks].AsSpan(handle % SlabBricks * BrickVoxels, BrickVoxels);

    /// <summary>Brick state: Air, Solid, or a pool handle.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int BrickState(int b) => _brickIndex[b];

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public int BrickId(int bx, int by, int bz) => (bz * BricksY + by) * BricksX + bx;

    // ---- stamping ---------------------------------------------------------

    /// <summary>Stamp a batch of segments: bin to bricks (counting sort), parallel over bricks.</summary>
    public void StampBatch(PathSegment[] segments, int segStart, int segCount, ToolProfile tool,
        bool parallel = true, bool conservativeCull = true)
    {
        DirtyBricks.Clear();

        // --- 1. bin segments to bricks ---
        float brickWorld = BrickSize * VoxelSize;
        float padZlo = tool.Zmin - BandWorld - tool.Epsilon;
        float padZhi = tool.Zmax + BandWorld + tool.Epsilon;
        _touchedCount = 0;
        int totalPairs = 0;
        for (int s = segStart; s < segStart + segCount; s++)
        {
            GetBrickRange(segments[s], tool, brickWorld, padZlo, padZhi,
                          out int x0, out int x1, out int y0, out int y1, out int z0, out int z1);
            for (int bz = z0; bz <= z1; bz++)
                for (int by = y0; by <= y1; by++)
                    for (int bx = x0; bx <= x1; bx++)
                    {
                        int b = BrickId(bx, by, bz);
                        if (_binCount[b]++ == 0)
                        {
                            if (_touchedCount == _touched.Length) Array.Resize(ref _touched, _touched.Length * 2);
                            _touched[_touchedCount++] = b;
                        }
                        totalPairs++;
                    }
        }
        if (_touchedCount + 1 > _binStart.Length) Array.Resize(ref _binStart, _touchedCount + 1);
        if (totalPairs > _binSegs.Length) Array.Resize(ref _binSegs, Math.Max(totalPairs, _binSegs.Length * 2));
        if (_touchedCount > _brickRemoved.Length) Array.Resize(ref _brickRemoved, _touchedCount);

        int acc = 0;
        for (int i = 0; i < _touchedCount; i++)
        {
            _binStart[i] = acc;
            int b = _touched[i];
            int c = _binCount[b];
            _binCount[b] = acc;      // becomes write cursor, then start offset
            acc += c;
        }
        _binStart[_touchedCount] = acc;

        for (int s = segStart; s < segStart + segCount; s++)
        {
            GetBrickRange(segments[s], tool, brickWorld, padZlo, padZhi,
                          out int x0, out int x1, out int y0, out int y1, out int z0, out int z1);
            for (int bz = z0; bz <= z1; bz++)
                for (int by = y0; by <= y1; by++)
                    for (int bx = x0; bx <= x1; bx++)
                        _binSegs[_binCount[BrickId(bx, by, bz)]++] = s;
        }

        // --- 2. parallel over touched bricks ---
        if (parallel && _touchedCount > 1)
            Parallel.For(0, _touchedCount, i => StampBrick(i, segments, tool, conservativeCull));
        else
            for (int i = 0; i < _touchedCount; i++) StampBrick(i, segments, tool, conservativeCull);

        // --- 3. deterministic reductions + cleanup ---
        for (int i = 0; i < _touchedCount; i++)
        {
            RemovedVolume += _brickRemoved[i];
            _brickRemoved[i] = 0;
            _binCount[_touched[i]] = 0;
        }

        ExpandDirtyToFaceNeighbors();

        // --- 4. coarse SDF stamp (serial; ~1/512 of fine cost) ---
        if (CoarseEnabled) StampCoarse(segments.AsSpan(segStart, segCount), tool);
    }

    private void GetBrickRange(in PathSegment s, ToolProfile tool, float brickWorld,
        float padZlo, float padZhi,
        out int x0, out int x1, out int y0, out int y1, out int z0, out int z1)
    {
        // A long machine assembly may have a wide collar far above the stock. Only profile
        // heights within one narrow-band distance of this finite volume can affect its voxels.
        float influence = BandWorld + tool.Epsilon;
        float axisMinZ = MathF.Min(s.A.Z, s.B.Z), axisMaxZ = MathF.Max(s.A.Z, s.B.Z);
        float activeRadius = tool.MaxRadiusInRange(-axisMaxZ - influence,
            StockHeight - axisMinZ + influence);
        float padXY = activeRadius + influence;
        float minX = MathF.Min(s.A.X, s.B.X) - padXY, maxX = MathF.Max(s.A.X, s.B.X) + padXY;
        float minY = MathF.Min(s.A.Y, s.B.Y) - padXY, maxY = MathF.Max(s.A.Y, s.B.Y) + padXY;
        float minZ = MathF.Min(s.A.Z, s.B.Z) + padZlo, maxZ = MathF.Max(s.A.Z, s.B.Z) + padZhi;
        if (maxX < 0f || minX >= StockSizeX || maxY < 0f || minY >= StockSizeY
            || maxZ < 0f || minZ >= StockHeight)
        {
            x0 = y0 = z0 = 0;
            x1 = y1 = z1 = -1;
            return;
        }
        int maxStockBrickZ = Math.Clamp((int)MathF.Ceiling(StockHeight / brickWorld) - 1, 0, BricksZ - 1);
        x0 = Math.Clamp((int)(minX / brickWorld), 0, BricksX - 1);
        x1 = Math.Clamp((int)(maxX / brickWorld), 0, BricksX - 1);
        y0 = Math.Clamp((int)(minY / brickWorld), 0, BricksY - 1);
        y1 = Math.Clamp((int)(maxY / brickWorld), 0, BricksY - 1);
        z0 = Math.Clamp((int)(minZ / brickWorld), 0, maxStockBrickZ);
        z1 = Math.Clamp((int)(maxZ / brickWorld), 0, maxStockBrickZ);
    }

    private void StampBrick(int touchedIdx, PathSegment[] segs, ToolProfile tool,
        bool conservativeCull)
    {
        int b = _touched[touchedIdx];
        int state = BrickState(b);
        if (state == Air) return;                    // nothing left to remove

        int bs = BrickSize;
        int bx = b % BricksX, by = b / BricksX % BricksY, bz = b / (BricksX * BricksY);
        float brickWorld = bs * VoxelSize;
        float ox = bx * brickWorld, oy = by * brickWorld, oz = bz * brickWorld;
        float cx = ox + brickWorld * 0.5f, cy = oy + brickWorld * 0.5f, cz = oz + brickWorld * 0.5f;
        float ballR = 0.8660254f * brickWorld;       // half diagonal
        float bandW = BandWorld;
        float vox = VoxelSize, invVox = 1f / vox, voxVol = vox * vox * vox;

        var scratch = _scratch ??= new float[16 * 16 * 16];
        var survivors = _survivors ??= new int[256];
        bool loaded = false;
        bool changed = false;
        bool engulfed = false;
        byte faceMask = 0;
        double removed = 0;
        long evals = 0, writes = 0, misses = 0, colDists = 0, runs = 0;

        // --- pre-scan: conservative cull, engulf detection, survivor list ---
        int start = _binStart[touchedIdx], end = _binStart[touchedIdx + 1];
        int nSurv = 0;
        for (int si = start; si < end; si++)
        {
            int sIdx = _binSegs[si];
            if (!conservativeCull)
            {
                if (nSurv == survivors.Length)
                {
                    Array.Resize(ref survivors, survivors.Length * 2);
                    _survivors = survivors;
                }
                survivors[nSurv++] = sIdx;
                continue;
            }

            ref readonly PathSegment seg = ref segs[sIdx];
            SweptBrush.ConservativeRange(tool, seg.A, seg.B, cx, cy, cz, ballR, out float dmin, out float dmax);
            if (dmin >= bandW) { misses++; continue; }                 // brush misses brick
            if (dmax <= -bandW)                                        // brush engulfs whole brick
            {
                StatEngulfsInc();
                if (state == Solid) removed += (double)BrickVoxels * voxVol;
                else
                {
                    if (!loaded) { LoadScratch(state, scratch); loaded = true; }
                    for (int k = 0; k < BrickVoxels; k++) removed += Occ(scratch[k], invVox) * voxVol;
                }
                state = Air;
                loaded = false;
                engulfed = true;
                changed = true;
                faceMask = FaceNegX | FacePosX | FaceNegY | FacePosY | FaceNegZ | FacePosZ;
                break;                                                 // later segments can't add material
            }
            if (nSurv == survivors.Length) { Array.Resize(ref survivors, survivors.Length * 2); _survivors = survivors; }
            survivors[nSurv++] = sIdx;
        }

        if (!engulfed && nSurv > 0)
        {
            if (!loaded)
            {
                LoadScratch(state, scratch);
                loaded = true;
            }

            float zTol = 0.10f * vox;                // balanced level-run tolerance: less slope plateauing, keeps run throughput
            float writeEps = WriteEpsilon;
            int bs2 = bs * bs;
            Span<float> columnR2 = stackalloc float[16 * 16];
            Span<float> columnZ = stackalloc float[16 * 16];

            // Subtraction is a commutative max, so path-adjacent runs are needlessly strict:
            // every level segment in this brick whose Z lies in the same tolerance bucket can
            // share one XY nearest-stroke pass. This preserves the old ±zTol approximation but
            // changes the expensive voxel work from O(segments × brick voxels) to
            // O(segments × XY columns + occupied Z buckets × brick voxels).
            int minGroup = int.MaxValue, maxGroup = int.MinValue;
            for (int i = 0; i < nSurv; i++)
            {
                ref readonly PathSegment segment = ref segs[survivors[i]];
                if (MathF.Abs(segment.A.Z - segment.B.Z) > zTol)
                {
                    Stamp3D(tool, in segment, scratch, bs, ox, oy, oz, vox, invVox, voxVol, bandW,
                            ref removed, ref evals, ref writes, ref changed, ref faceMask);
                    continue;
                }

                int group = (int)MathF.Floor(
                    (0.5f * (segment.A.Z + segment.B.Z)) / zTol);
                minGroup = Math.Min(minGroup, group);
                maxGroup = Math.Max(maxGroup, group);
            }

            if (minGroup <= maxGroup)
            {
                int groupRange = checked(maxGroup - minGroup + 1);
                var counts = _groupCount ??= new int[Math.Max(64, groupRange)];
                if (counts.Length < groupRange)
                {
                    counts = new int[groupRange];
                    _groupCount = counts;
                }
                Array.Clear(counts, 0, groupRange);

                int groupedCount = 0;
                for (int i = 0; i < nSurv; i++)
                {
                    ref readonly PathSegment segment = ref segs[survivors[i]];
                    if (MathF.Abs(segment.A.Z - segment.B.Z) > zTol)
                    {
                        continue;
                    }
                    int group = (int)MathF.Floor(
                        (0.5f * (segment.A.Z + segment.B.Z)) / zTol) - minGroup;
                    counts[group]++;
                    groupedCount++;
                }

                var starts = _groupStart ??= new int[Math.Max(65, groupRange + 1)];
                if (starts.Length <= groupRange)
                {
                    starts = new int[groupRange + 1];
                    _groupStart = starts;
                }
                int offset = 0;
                for (int group = 0; group < groupRange; group++)
                {
                    starts[group] = offset;
                    offset += counts[group];
                    counts[group] = starts[group];
                }
                starts[groupRange] = offset;

                var grouped = _groupedSurvivors ??= new int[Math.Max(256, groupedCount)];
                if (grouped.Length < groupedCount)
                {
                    grouped = new int[groupedCount];
                    _groupedSurvivors = grouped;
                }
                for (int i = 0; i < nSurv; i++)
                {
                    int segmentIndex = survivors[i];
                    ref readonly PathSegment segment = ref segs[segmentIndex];
                    if (MathF.Abs(segment.A.Z - segment.B.Z) > zTol)
                    {
                        continue;
                    }
                    int group = (int)MathF.Floor(
                        (0.5f * (segment.A.Z + segment.B.Z)) / zTol) - minGroup;
                    grouped[counts[group]++] = segmentIndex;
                }

                for (int group = 0; group < groupRange; group++)
                {
                    int runStart = starts[group], runEnd = starts[group + 1];
                    if (runStart == runEnd)
                    {
                        continue;
                    }

                    float zRef = (minGroup + group + 0.5f) * zTol;
                    float localZMin = oz + 0.5f * vox - (zRef + zTol);
                    float localZMax = oz + (bs - 0.5f) * vox - (zRef - zTol);
                    bool radialMonotone = tool.IsRadiallyMonotone(
                        localZMin, localZMax);
                    if (!radialMonotone)
                    {
                        for (int i = runStart; i < runEnd; i++)
                        {
                            ref readonly PathSegment segment = ref segs[grouped[i]];
                            Stamp3D(tool, in segment, scratch, bs, ox, oy, oz, vox, invVox,
                                voxVol, bandW, ref removed, ref evals, ref writes, ref changed,
                                ref faceMask);
                        }
                        continue;
                    }

                    runs++;
                    float influence = bandW + tool.Epsilon;
                    float colSkipR = tool.MaxRadiusInRange(
                        localZMin - influence, localZMax + influence) + influence;

                    // Invert the local nearest-stroke loop once more: each segment visits only
                    // the voxel columns inside its expanded XY AABB. This is the same minimum
                    // as scanning every segment from every column, without the S × bs² term for
                    // columns that cannot be influenced by that segment.
                    Span<float> groupR2 = columnR2[..bs2];
                    Span<float> groupZ = columnZ[..bs2];
                    groupR2.Fill(float.MaxValue);
                    for (int k = runStart; k < runEnd; k++)
                    {
                        ref readonly PathSegment segment = ref segs[grouped[k]];
                        int x0 = Math.Clamp((int)MathF.Floor(
                            (MathF.Min(segment.A.X, segment.B.X) - colSkipR - ox) / vox),
                            0, bs - 1);
                        int x1 = Math.Clamp((int)MathF.Floor(
                            (MathF.Max(segment.A.X, segment.B.X) + colSkipR - ox) / vox),
                            0, bs - 1);
                        int y0 = Math.Clamp((int)MathF.Floor(
                            (MathF.Min(segment.A.Y, segment.B.Y) - colSkipR - oy) / vox),
                            0, bs - 1);
                        int y1 = Math.Clamp((int)MathF.Floor(
                            (MathF.Max(segment.A.Y, segment.B.Y) + colSkipR - oy) / vox),
                            0, bs - 1);
                        float ex = segment.B.X - segment.A.X;
                        float ey = segment.B.Y - segment.A.Y;
                        for (int y = y0; y <= y1; y++)
                        {
                            float py = oy + (y + 0.5f) * vox;
                            for (int x = x0; x <= x1; x++)
                            {
                                float px = ox + (x + 0.5f) * vox;
                                float wx = px - segment.A.X;
                                float wy = py - segment.A.Y;
                                float t = (wx * ex + wy * ey) * segment.InvXyLengthSquared;
                                t = t < 0 ? 0 : t > 1 ? 1 : t;
                                float qx = wx - ex * t;
                                float qy = wy - ey * t;
                                float d2 = qx * qx + qy * qy;
                                int column = y * bs + x;
                                if (d2 < groupR2[column])
                                {
                                    groupR2[column] = d2;
                                    groupZ[column] = segment.A.Z
                                        + (segment.B.Z - segment.A.Z) * t;
                                }
                                colDists++;
                            }
                        }
                    }

                    // One LUT sample per engaged voxel after the nearest stroke is known.
                    for (int y = 0; y < bs; y++)
                    {
                        for (int x = 0; x < bs; x++)
                        {
                            int column = y * bs + x;
                            float rmin = MathF.Sqrt(groupR2[column]);
                            if (rmin >= colSkipR) continue;
                            float zCol = groupZ[column];

                            int k2 = column;
                            float pz = oz + 0.5f * vox;
                            for (int z = 0; z < bs; z++, k2 += bs2, pz += vox)
                            {
                                float d = scratch[k2];
                                if (d >= bandW) continue;
                                evals++;
                                float nb = -(tool.Sample(rmin, pz - zCol) - tool.Epsilon);
                                if (nb > d + writeEps)
                                {
                                    float nd = nb > bandW ? bandW : nb;
                                    removed += (Occ(d, invVox) - Occ(nd, invVox)) * voxVol;
                                    scratch[k2] = nd;
                                    if (x == 0) faceMask |= FaceNegX;
                                    else if (x == bs - 1) faceMask |= FacePosX;
                                    if (y == 0) faceMask |= FaceNegY;
                                    else if (y == bs - 1) faceMask |= FacePosY;
                                    if (z == 0) faceMask |= FaceNegZ;
                                    else if (z == bs - 1) faceMask |= FacePosZ;
                                    writes++;
                                    changed = true;
                                }
                            }
                        }
                    }
                }
            }
        }

        Interlocked.Add(ref StatVoxelEvals, evals);
        Interlocked.Add(ref StatVoxelWrites, writes);
        Interlocked.Add(ref StatMissSkips, misses);
        Interlocked.Add(ref StatColumnDists, colDists);
        Interlocked.Add(ref StatRuns, runs);

        if (!changed) return;
        _brickRemoved[touchedIdx] = removed;

        // --- reclassify + writeback ---
        int newState;
        if (state == Air) newState = Air;
        else
        {
            float mn = float.MaxValue, mx = float.MinValue;
            for (int k = 0; k < BrickVoxels; k++) { float v = scratch[k]; if (v < mn) mn = v; if (v > mx) mx = v; }
            newState = mn >= bandW - 1e-6f ? Air : mx <= -bandW + 1e-6f ? Solid : int.MaxValue;
        }

        Half[]? dstSlab = null;
        int dstOff = 0;
        lock (_slabs) // brick ownership is exclusive; the lock only guards pool alloc/free + dirty list
        {
            int oldIdx = _brickIndex[b];
            if (newState == int.MaxValue)                              // mixed
            {
                int h = oldIdx >= 0 ? oldIdx : AllocHandle();
                _brickIndex[b] = h;
                dstSlab = _slabs[h / SlabBricks];
                dstOff = h % SlabBricks * BrickVoxels;
                Interlocked.Add(ref StatPartialBricks, 1);
            }
            else                                                       // uniform air/solid
            {
                if (oldIdx >= 0)
                {
                    FreeHandle(oldIdx);
                    _brickIndex[b] = newState;
                }
                else _brickIndex[b] = newState;
            }
            _dirtyFaceMask[b] = faceMask;
            DirtyBricks.Add(b);
        }
        if (dstSlab is not null) // Half conversion + store outside the lock (owned region, race-free)
            for (int k = 0; k < BrickVoxels; k++) dstSlab[dstOff + k] = (Half)scratch[k];
    }

    private void StatEngulfsInc() => Interlocked.Add(ref StatEngulfs, 1);

    private static void Stamp3D(ToolProfile tool, in PathSegment seg, float[] scratch, int bs,
        float ox, float oy, float oz, float vox, float invVox, float voxVol, float bandW,
        ref double removed, ref long evals, ref long writes, ref bool changed, ref byte faceMask)
    {
        var a = seg.A; var bb = seg.B;
        int k2 = 0;
        for (int z = 0; z < bs; z++)
        {
            float pz = oz + (z + 0.5f) * vox;
            for (int y = 0; y < bs; y++)
            {
                float py = oy + (y + 0.5f) * vox;
                float px = ox + 0.5f * vox;
                for (int x = 0; x < bs; x++, k2++, px += vox)
                {
                    float d = scratch[k2];
                    if (d >= bandW) continue;
                    evals++;
                    float nb = -SweptBrush.Eval(tool, a, bb, px, py, pz);
                    if (nb > d + 1e-7f)
                    {
                        float nd = nb > bandW ? bandW : nb;
                        removed += (Occ(d, invVox) - Occ(nd, invVox)) * voxVol;
                        scratch[k2] = nd;
                        if (x == 0) faceMask |= FaceNegX;
                        else if (x == bs - 1) faceMask |= FacePosX;
                        if (y == 0) faceMask |= FaceNegY;
                        else if (y == bs - 1) faceMask |= FacePosY;
                        if (z == 0) faceMask |= FaceNegZ;
                        else if (z == bs - 1) faceMask |= FacePosZ;
                        writes++;
                        changed = true;
                    }
                }
            }
        }
    }

    private void LoadScratch(int state, float[] scratch)
    {
        if (state == Solid) { Array.Fill(scratch, -BandWorld, 0, BrickVoxels); return; }
        var src = BrickData(state);
        for (int k = 0; k < BrickVoxels; k++) scratch[k] = (float)src[k];
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float Occ(float d, float invVox)
    {
        float o = 0.5f - d * invVox;
        return o < 0 ? 0 : o > 1 ? 1 : o;
    }

    private void ExpandDirtyToFaceNeighbors()
    {
        if (DirtyBricks.Count == 0)
        {
            return;
        }

        var all = new HashSet<int>(DirtyBricks.Count * 2);
        int original = DirtyBricks.Count;

        for (int i = 0; i < original; i++)
        {
            int b = DirtyBricks[i];
            all.Add(b);
            byte m = _dirtyFaceMask[b];
            _dirtyFaceMask[b] = 0;
            if (m == 0)
            {
                continue;
            }

            int bx = b % BricksX;
            int by = b / BricksX % BricksY;
            int bz = b / (BricksX * BricksY);

            // Atlas tiles duplicate voxels 0..8 (high-side border): a write on a low
            // face/edge/corner of this brick lives in the tiles of every negative-side
            // neighbor sharing that voxel — a (0,0,0) voxel write sits in 7 neighbor tiles.
            int dx0 = (m & FaceNegX) != 0 && bx > 0 ? -1 : 0;
            int dy0 = (m & FaceNegY) != 0 && by > 0 ? -1 : 0;
            int dz0 = (m & FaceNegZ) != 0 && bz > 0 ? -1 : 0;
            for (int dz = dz0; dz <= 0; dz++)
                for (int dy = dy0; dy <= 0; dy++)
                    for (int dx = dx0; dx <= 0; dx++)
                        all.Add(BrickId(bx + dx, by + dy, bz + dz));
        }

        if (all.Count == DirtyBricks.Count)
        {
            return;
        }

        DirtyBricks.Clear();
        DirtyBricks.AddRange(all);
    }

    private void StampCoarse(ReadOnlySpan<PathSegment> segsSpan, ToolProfile tool)
    {
        float cell = CoarseCell;
        var segs = segsSpan.ToArray(); // small per-batch copy for the parallel lambda
        Parallel.For(0, segs.Length, si =>
        {
            ref readonly var s = ref segs[si];
            float influence = tool.Epsilon + cell;
            float axisMinZ = MathF.Min(s.A.Z, s.B.Z), axisMaxZ = MathF.Max(s.A.Z, s.B.Z);
            float activeRadius = tool.MaxRadiusInRange(-axisMaxZ - influence,
                StockHeight - axisMinZ + influence);
            float padXY = activeRadius + influence;
            float minX = MathF.Min(s.A.X, s.B.X) - padXY, maxX = MathF.Max(s.A.X, s.B.X) + padXY;
            float minY = MathF.Min(s.A.Y, s.B.Y) - padXY, maxY = MathF.Max(s.A.Y, s.B.Y) + padXY;
            float minZ = MathF.Min(s.A.Z, s.B.Z) + tool.Zmin - cell, maxZ = MathF.Max(s.A.Z, s.B.Z) + tool.Zmax + cell;
            if (maxX < 0f || minX >= StockSizeX || maxY < 0f || minY >= StockSizeY
                || maxZ < 0f || minZ >= StockHeight)
                return;
            int maxStockCellZ = Math.Clamp((int)MathF.Ceiling(StockHeight / cell) - 1, 0, CoarseZ - 1);
            int x0 = Math.Clamp((int)(minX / cell), 0, CoarseX - 1), x1 = Math.Clamp((int)(maxX / cell), 0, CoarseX - 1);
            int y0 = Math.Clamp((int)(minY / cell), 0, CoarseY - 1), y1 = Math.Clamp((int)(maxY / cell), 0, CoarseY - 1);
            int z0 = Math.Clamp((int)(minZ / cell), 0, maxStockCellZ), z1 = Math.Clamp((int)(maxZ / cell), 0, maxStockCellZ);
            for (int z = z0; z <= z1; z++)
                for (int y = y0; y <= y1; y++)
                    for (int x = x0; x <= x1; x++)
                    {
                        int c = (z * CoarseY + y) * CoarseX + x;
                        float nb = -SweptBrush.Eval(tool, s.A, s.B,
                            (x + 0.5f) * cell, (y + 0.5f) * cell, (z + 0.5f) * cell);
                        AtomicMax(ref Coarse[c], nb);
                    }
        });
    }

    private static void AtomicMax(ref float loc, float value)
    {
        float cur = Volatile.Read(ref loc);
        while (value > cur)
        {
            float orig = Interlocked.CompareExchange(ref loc, value, cur);
            if (orig == cur) break;
            cur = orig;
        }
    }

    /// <summary>
    /// Rebuilds a conservative empty-space acceleration field from final brick states in
    /// O(coarse cells), independent of toolpath history. Mixed/solid cells are barriers; air
    /// cells store a chessboard-distance lower bound to the nearest barrier. This field is used
    /// only for safe ray steps and does not alter the fine SDF.
    /// </summary>
    public void RebuildCoarseFromBrickStates()
    {
        const int infinity = 1 << 28;
        int[] distance = new int[Coarse.Length];

        if (BrickSize == 8)
        {
            for (int i = 0; i < distance.Length; i++)
            {
                distance[i] = BrickState(i) == Air ? infinity : 0;
            }
        }
        else
        {
            for (int z = 0; z < CoarseZ; z++)
                for (int y = 0; y < CoarseY; y++)
                    for (int x = 0; x < CoarseX; x++)
                    {
                        bool air = true;
                        int vx0 = x * 8, vy0 = y * 8, vz0 = z * 8;
                        int bx0 = vx0 / BrickSize, by0 = vy0 / BrickSize, bz0 = vz0 / BrickSize;
                        int bx1 = (vx0 + 7) / BrickSize;
                        int by1 = (vy0 + 7) / BrickSize;
                        int bz1 = (vz0 + 7) / BrickSize;
                        for (int bz = bz0; air && bz <= bz1; bz++)
                            for (int by = by0; air && by <= by1; by++)
                                for (int bx = bx0; bx <= bx1; bx++)
                                    if (BrickState(BrickId(bx, by, bz)) != Air)
                                    {
                                        air = false;
                                        break;
                                    }
                        distance[(z * CoarseY + y) * CoarseX + x] = air ? infinity : 0;
                    }
        }

        // Unit-cost 26-neighbour chamfer passes compute the L-infinity cell distance.
        for (int z = 0; z < CoarseZ; z++)
            for (int y = 0; y < CoarseY; y++)
                for (int x = 0; x < CoarseX; x++)
                {
                    int index = (z * CoarseY + y) * CoarseX + x;
                    int best = distance[index];
                    if (best == 0) continue;
                    for (int dz = -1; dz <= 0; dz++)
                        for (int dy = -1; dy <= (dz == 0 ? 0 : 1); dy++)
                            for (int dx = -1; dx <= ((dz == 0 && dy == 0) ? -1 : 1); dx++)
                            {
                                int nx = x + dx, ny = y + dy, nz = z + dz;
                                if ((uint)nx >= (uint)CoarseX || (uint)ny >= (uint)CoarseY
                                    || (uint)nz >= (uint)CoarseZ) continue;
                                best = Math.Min(best,
                                    distance[(nz * CoarseY + ny) * CoarseX + nx] + 1);
                            }
                    distance[index] = best;
                }

        for (int z = CoarseZ - 1; z >= 0; z--)
            for (int y = CoarseY - 1; y >= 0; y--)
                for (int x = CoarseX - 1; x >= 0; x--)
                {
                    int index = (z * CoarseY + y) * CoarseX + x;
                    int best = distance[index];
                    if (best == 0) continue;
                    for (int dz = 1; dz >= 0; dz--)
                        for (int dy = 1; dy >= (dz == 0 ? 0 : -1); dy--)
                            for (int dx = 1; dx >= ((dz == 0 && dy == 0) ? 1 : -1); dx--)
                            {
                                int nx = x + dx, ny = y + dy, nz = z + dz;
                                if ((uint)nx >= (uint)CoarseX || (uint)ny >= (uint)CoarseY
                                    || (uint)nz >= (uint)CoarseZ) continue;
                                best = Math.Min(best,
                                    distance[(nz * CoarseY + ny) * CoarseX + nx] + 1);
                            }
                    distance[index] = best;
                }

        for (int i = 0; i < Coarse.Length; i++)
        {
            int cells = distance[i];
            Coarse[i] = cells == 0 || cells == infinity
                ? -CoarseCell
                : Math.Max(0, cells - 3) * CoarseCell;
        }
    }

    // ---- sampling (CPU raymarch bench / verification) ----------------------

    /// <summary>Voxel-center value with brick indirection (clamped at volume edges).</summary>
    public float GetVoxel(int ix, int iy, int iz)
    {
        ix = Math.Clamp(ix, 0, VoxelsX - 1);
        iy = Math.Clamp(iy, 0, VoxelsY - 1);
        iz = Math.Clamp(iz, 0, VoxelsZ - 1);
        int bs = BrickSize;
        int b = BrickId(ix / bs, iy / bs, iz / bs);
        int state = BrickState(b);
        if (state == Air) return BandWorld;
        if (state == Solid) return -BandWorld;
        int k = (iz % bs * bs + iy % bs) * bs + ix % bs;
        return (float)BrickData(state)[k];
    }

    /// <summary>Trilinear fine-field sample at world position (mm), band-clamped.</summary>
    public float SampleFine(float px, float py, float pz)
    {
        float u = px / VoxelSize - 0.5f, v = py / VoxelSize - 0.5f, w = pz / VoxelSize - 0.5f;
        int i = (int)MathF.Floor(u), j = (int)MathF.Floor(v), k = (int)MathF.Floor(w);
        float fu = u - i, fv = v - j, fw = w - k;
        float c000 = GetVoxel(i, j, k), c100 = GetVoxel(i + 1, j, k);
        float c010 = GetVoxel(i, j + 1, k), c110 = GetVoxel(i + 1, j + 1, k);
        float c001 = GetVoxel(i, j, k + 1), c101 = GetVoxel(i + 1, j, k + 1);
        float c011 = GetVoxel(i, j + 1, k + 1), c111 = GetVoxel(i + 1, j + 1, k + 1);
        float x00 = c000 + (c100 - c000) * fu, x10 = c010 + (c110 - c010) * fu;
        float x01 = c001 + (c101 - c001) * fu, x11 = c011 + (c111 - c011) * fu;
        float y0 = x00 + (x10 - x00) * fv, y1 = x01 + (x11 - x01) * fv;
        return y0 + (y1 - y0) * fw;
    }

    /// <summary>Trilinear coarse-field sample at world position (mm).</summary>
    public float SampleCoarse(float px, float py, float pz)
    {
        float u = px / CoarseCell - 0.5f, v = py / CoarseCell - 0.5f, w = pz / CoarseCell - 0.5f;
        int i = Math.Clamp((int)MathF.Floor(u), 0, CoarseX - 2);
        int j = Math.Clamp((int)MathF.Floor(v), 0, CoarseY - 2);
        int k = Math.Clamp((int)MathF.Floor(w), 0, CoarseZ - 2);
        float fu = Math.Clamp(u - i, 0, 1), fv = Math.Clamp(v - j, 0, 1), fw = Math.Clamp(w - k, 0, 1);
        var c = Coarse;
        int i000 = (k * CoarseY + j) * CoarseX + i;
        float c000 = c[i000], c100 = c[i000 + 1], c010 = c[i000 + CoarseX], c110 = c[i000 + CoarseX + 1];
        int i001 = i000 + CoarseX * CoarseY;
        float c001 = c[i001], c101 = c[i001 + 1], c011 = c[i001 + CoarseX], c111 = c[i001 + CoarseX + 1];
        float x00 = c000 + (c100 - c000) * fu, x10 = c010 + (c110 - c010) * fu;
        float x01 = c001 + (c101 - c001) * fu, x11 = c011 + (c111 - c011) * fu;
        float y0 = x00 + (x10 - x00) * fv, y1 = x01 + (x11 - x01) * fv;
        return y0 + (y1 - y0) * fw;
    }

    // ---- conservative planning probes -----------------------------------

    /// <summary>
    /// Classifies whether an oriented cutter sweep could remove material without mutating the
    /// volume. <see cref="RemovalProbeResult.DefinitelyAir"/> is returned only when every
    /// conservative candidate brick is already air; mixed or unresolved surface cases remain
    /// <see cref="RemovalProbeResult.Uncertain"/> so a planner cannot silently skip stock.
    /// </summary>
    public RemovalProbe ProbeRemoval(in OrientedToolSweep sweep, ToolProfile tool)
    {
        ArgumentNullException.ThrowIfNull(tool);
        GetOrientedSweepBrickRange(sweep, tool, out int x0, out int x1, out int y0, out int y1, out int z0, out int z1);
        if (x0 > x1 || y0 > y1 || z0 > z1)
        {
            return new RemovalProbe(
                RemovalProbeResult.DefinitelyAir,
                new RemovalProbeDiagnostics(0, 0, 0, BandWorld, BandWorld));
        }

        int samples = 0;
        int materialSamples = 0;
        int uncertainSamples = 0;
        float minimumStockDistance = float.MaxValue;
        float minimumBrushDistance = float.MaxValue;
        bool encounteredNonAir = false;
        float contactTolerance = MathF.Max(tool.Epsilon, 0.5f * VoxelSize);
        float brickWorld = BrickSize * VoxelSize;

        for (int bz = z0; bz <= z1; bz++)
        {
            for (int by = y0; by <= y1; by++)
            {
                for (int bx = x0; bx <= x1; bx++)
                {
                    int brick = BrickId(bx, by, bz);
                    int state = BrickState(brick);
                    if (state == Air)
                        continue;

                    encounteredNonAir = true;
                    float originX = bx * brickWorld;
                    float originY = by * brickWorld;
                    float originZ = bz * brickWorld;
                    for (int localZ = 0; localZ < BrickSize; localZ++)
                    {
                        for (int localY = 0; localY < BrickSize; localY++)
                        {
                            for (int localX = 0; localX < BrickSize; localX++)
                            {
                                Vector3 point = new(
                                    originX + (localX + 0.5f) * VoxelSize,
                                    originY + (localY + 0.5f) * VoxelSize,
                                    originZ + (localZ + 0.5f) * VoxelSize);
                                float brushDistance = SweptBrush.EvalOriented(
                                    tool, sweep.Start, sweep.End, sweep.Axis, point);
                                if (brushDistance > contactTolerance)
                                    continue;

                                float stockDistance = GetVoxel(
                                    bx * BrickSize + localX,
                                    by * BrickSize + localY,
                                    bz * BrickSize + localZ);
                                samples++;
                                minimumStockDistance = MathF.Min(minimumStockDistance, stockDistance);
                                minimumBrushDistance = MathF.Min(minimumBrushDistance, brushDistance);

                                if (stockDistance < -contactTolerance && brushDistance < -contactTolerance)
                                {
                                    materialSamples++;
                                }
                                else if (stockDistance <= contactTolerance)
                                {
                                    uncertainSamples++;
                                }
                            }
                        }
                    }
                }
            }
        }

        RemovalProbeResult result = materialSamples > 0
            ? RemovalProbeResult.IntersectsMaterial
            : encounteredNonAir
                ? RemovalProbeResult.Uncertain
                : RemovalProbeResult.DefinitelyAir;
        if (minimumStockDistance == float.MaxValue)
            minimumStockDistance = BandWorld;
        if (minimumBrushDistance == float.MaxValue)
            minimumBrushDistance = BandWorld;

        return new RemovalProbe(
            result,
            new RemovalProbeDiagnostics(
                samples,
                materialSamples,
                uncertainSamples,
                minimumStockDistance,
                minimumBrushDistance));
    }

    /// <summary>
    /// Measures the exact volume a sequence of oriented sweeps would remove from the current
    /// stock and the incremental volume the same path removes from virgin stock between its
    /// previous and current pass depths, without mutating this volume.
    /// </summary>
    public RemovalVolumeProbe ProbeRemovalVolume(
        IReadOnlyList<OrientedToolSweep> sweeps,
        ToolProfile tool,
        float activeAxialHeight)
    {
        ArgumentNullException.ThrowIfNull(sweeps);
        ArgumentNullException.ThrowIfNull(tool);
        if (!float.IsFinite(activeAxialHeight) || activeAxialHeight <= 0f)
            throw new ArgumentOutOfRangeException(nameof(activeAxialHeight));

        bool definitelyAir = true;
        foreach (OrientedToolSweep sweep in sweeps)
        {
            if (!IsSweepDefinitelyAir(sweep, tool))
            {
                definitelyAir = false;
                break;
            }
        }

        if (definitelyAir)
        {
            return new RemovalVolumeProbe(
                DefinitelyAir: true,
                RemovableVolume: 0d,
                FullStockRemovableVolume: 0d);
        }

        SparseSdfVolume probe = DeepClone();
        double removedBefore = probe.RemovedVolume;
        foreach (OrientedToolSweep sweep in sweeps)
            probe.StampOriented(sweep, tool);

        var fullStockProbe = new SparseSdfVolume(
            VoxelsX,
            VoxelsY,
            VoxelsZ,
            VoxelSize,
            BrickSize,
            BandVoxels,
            StockHeight)
        {
            WriteEpsilon = WriteEpsilon,
            CoarseEnabled = false
        };
        foreach (OrientedToolSweep sweep in sweeps)
        {
            Vector3 offset = sweep.Axis.ToVector() * activeAxialHeight;
            fullStockProbe.StampOriented(
                new OrientedToolSweep(sweep.Start + offset, sweep.End + offset, sweep.Axis),
                tool);
        }

        double fullStockRemovedBefore = fullStockProbe.RemovedVolume;
        foreach (OrientedToolSweep sweep in sweeps)
            fullStockProbe.StampOriented(sweep, tool);

        return new RemovalVolumeProbe(
            DefinitelyAir: false,
            RemovableVolume: probe.RemovedVolume - removedBefore,
            FullStockRemovableVolume: fullStockProbe.RemovedVolume - fullStockRemovedBefore);
    }

    /// <summary>
    /// Returns true only if every conservatively affected brick is already air. This is the
    /// planner's sole safe pruning condition and avoids fine-voxel diagnostics when any stock
    /// candidate must remain in the toolpath.
    /// </summary>
    public bool IsSweepDefinitelyAir(in OrientedToolSweep sweep, ToolProfile tool)
    {
        ArgumentNullException.ThrowIfNull(tool);
        GetOrientedSweepBrickRange(sweep, tool, out int x0, out int x1, out int y0, out int y1, out int z0, out int z1);
        if (x0 > x1 || y0 > y1 || z0 > z1)
            return true;

        for (int bz = z0; bz <= z1; bz++)
        {
            for (int by = y0; by <= y1; by++)
            {
                for (int bx = x0; bx <= x1; bx++)
                {
                    if (BrickState(BrickId(bx, by, bz)) != Air)
                        return false;
                }
            }
        }

        return true;
    }

    /// <summary>
    /// Subtracts a sweep whose profile axis is aligned to a signed stock axis. The established
    /// +Z path retains the optimized batch stamper; other faces use the same monotone SDF update
    /// with a conservative per-voxel fallback.
    /// </summary>
    public void StampOriented(in OrientedToolSweep sweep, ToolProfile tool)
    {
        ArgumentNullException.ThrowIfNull(tool);
        if (sweep.Axis == ToolAxisDirection.PositiveZ)
        {
            PathSegment[] segments = [new PathSegment(sweep.Start, sweep.End)];
            StampBatch(segments, 0, segments.Length, tool, parallel: false, conservativeCull: true);
            return;
        }

        GetOrientedSweepBrickRange(sweep, tool, out int x0, out int x1, out int y0, out int y1, out int z0, out int z1);
        DirtyBricks.Clear();
        if (x0 > x1 || y0 > y1 || z0 > z1)
            return;

        float brickWorld = BrickSize * VoxelSize;
        float inverseVoxel = 1f / VoxelSize;
        double removed = 0;
        for (int bz = z0; bz <= z1; bz++)
        {
            for (int by = y0; by <= y1; by++)
            {
                for (int bx = x0; bx <= x1; bx++)
                {
                    removed += StampOrientedBrick(
                        BrickId(bx, by, bz),
                        bx * brickWorld,
                        by * brickWorld,
                        bz * brickWorld,
                        sweep,
                        tool,
                        inverseVoxel);
                }
            }
        }

        RemovedVolume += removed;
        ExpandDirtyToFaceNeighbors();
        if (CoarseEnabled)
            RebuildCoarseFromBrickStates();
    }

    private void GetOrientedSweepBrickRange(
        in OrientedToolSweep sweep,
        ToolProfile tool,
        out int x0,
        out int x1,
        out int y0,
        out int y1,
        out int z0,
        out int z1)
    {
        Vector3 axis = sweep.Axis.ToVector();
        Vector3 firstAxialExtent = axis * tool.Zmin;
        Vector3 secondAxialExtent = axis * tool.Zmax;
        Vector3 minimumAxialExtent = Vector3.Min(firstAxialExtent, secondAxialExtent);
        Vector3 maximumAxialExtent = Vector3.Max(firstAxialExtent, secondAxialExtent);
        Vector3 radialExtent = new(
            axis.X == 0f ? tool.Rmax : 0f,
            axis.Y == 0f ? tool.Rmax : 0f,
            axis.Z == 0f ? tool.Rmax : 0f);
        Vector3 influence = radialExtent + new Vector3(BandWorld + tool.Epsilon);
        Vector3 minimum = Vector3.Min(sweep.Start, sweep.End) + minimumAxialExtent - influence;
        Vector3 maximum = Vector3.Max(sweep.Start, sweep.End) + maximumAxialExtent + influence;
        float brickWorld = BrickSize * VoxelSize;
        int maximumStockBrickZ = Math.Clamp(
            (int)MathF.Ceiling(StockHeight / brickWorld) - 1,
            0,
            BricksZ - 1);

        if (maximum.X < 0f || maximum.Y < 0f || maximum.Z < 0f
            || minimum.X >= StockSizeX || minimum.Y >= StockSizeY || minimum.Z >= StockHeight)
        {
            x0 = y0 = z0 = 0;
            x1 = y1 = z1 = -1;
            return;
        }

        x0 = Math.Clamp((int)MathF.Floor(minimum.X / brickWorld), 0, BricksX - 1);
        x1 = Math.Clamp((int)MathF.Floor(maximum.X / brickWorld), 0, BricksX - 1);
        y0 = Math.Clamp((int)MathF.Floor(minimum.Y / brickWorld), 0, BricksY - 1);
        y1 = Math.Clamp((int)MathF.Floor(maximum.Y / brickWorld), 0, BricksY - 1);
        z0 = Math.Clamp((int)MathF.Floor(minimum.Z / brickWorld), 0, maximumStockBrickZ);
        z1 = Math.Clamp((int)MathF.Floor(maximum.Z / brickWorld), 0, maximumStockBrickZ);
    }

    private double StampOrientedBrick(
        int brick,
        float originX,
        float originY,
        float originZ,
        in OrientedToolSweep sweep,
        ToolProfile tool,
        float inverseVoxel)
    {
        int state = BrickState(brick);
        if (state == Air)
            return 0;

        float[] scratch = _scratch ??= new float[16 * 16 * 16];
        LoadScratch(state, scratch);
        bool changed = false;
        byte faceMask = 0;
        double removed = 0;
        float voxelVolume = VoxelSize * VoxelSize * VoxelSize;
        int voxelIndex = 0;
        for (int localZ = 0; localZ < BrickSize; localZ++)
        {
            float z = originZ + (localZ + 0.5f) * VoxelSize;
            for (int localY = 0; localY < BrickSize; localY++)
            {
                float y = originY + (localY + 0.5f) * VoxelSize;
                for (int localX = 0; localX < BrickSize; localX++, voxelIndex++)
                {
                    float x = originX + (localX + 0.5f) * VoxelSize;
                    float current = scratch[voxelIndex];
                    if (current >= BandWorld)
                        continue;

                    float brushDistance = SweptBrush.EvalOriented(
                        tool,
                        sweep.Start,
                        sweep.End,
                        sweep.Axis,
                        new Vector3(x, y, z));
                    float next = MathF.Min(BandWorld, -brushDistance);
                    if (next <= current + WriteEpsilon)
                        continue;

                    removed += (Occ(current, inverseVoxel) - Occ(next, inverseVoxel)) * voxelVolume;
                    scratch[voxelIndex] = next;
                    if (localX == 0) faceMask |= FaceNegX;
                    else if (localX == BrickSize - 1) faceMask |= FacePosX;
                    if (localY == 0) faceMask |= FaceNegY;
                    else if (localY == BrickSize - 1) faceMask |= FacePosY;
                    if (localZ == 0) faceMask |= FaceNegZ;
                    else if (localZ == BrickSize - 1) faceMask |= FacePosZ;
                    changed = true;
                }
            }
        }

        if (!changed)
            return 0;

        float minimum = float.MaxValue;
        float maximum = float.MinValue;
        for (int index = 0; index < BrickVoxels; index++)
        {
            minimum = MathF.Min(minimum, scratch[index]);
            maximum = MathF.Max(maximum, scratch[index]);
        }

        int nextState = minimum >= BandWorld - 1e-6f
            ? Air
            : maximum <= -BandWorld + 1e-6f
                ? Solid
                : int.MaxValue;
        lock (_slabs)
        {
            int previousState = _brickIndex[brick];
            if (nextState == int.MaxValue)
            {
                int handle = previousState >= 0 ? previousState : AllocHandle();
                _brickIndex[brick] = handle;
                Span<Half> destination = BrickData(handle);
                for (int index = 0; index < BrickVoxels; index++)
                    destination[index] = (Half)scratch[index];
            }
            else
            {
                if (previousState >= 0)
                    FreeHandle(previousState);
                _brickIndex[brick] = nextState;
            }

            DirtyBricks.Add(brick);
            _dirtyFaceMask[brick] = faceMask;
        }

        return removed;
    }
}
