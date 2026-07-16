using System.Numerics;

namespace RW.CSG;

/// <summary>
/// Static uniform grid over the stock mapping cell → original (unsubdivided) segment indices.
/// Built once in O(N) with a counting sort. Cell size = brick world size.
/// <para>expandAtBuild=false (recommended): cells cover the segment AABB only; queries must expand
/// their box by (Rmax + band). expandAtBuild=true reproduces the naive variant for memory comparison.</para>
/// </summary>
public sealed class PathIndex
{
    public readonly int CellsPerSide;
    public readonly float CellSize;
    public readonly int[] CellStart;   // cells³ + 1
    public readonly int[] Refs;        // segment indices
    public readonly float BuildPad;    // half-extent added to segment AABBs at build time

    public PathIndex(PathSegment[] segments, float stockSize, float cellSize, float buildPad)
    {
        CellSize = cellSize;
        BuildPad = buildPad;
        CellsPerSide = Math.Max(1, (int)MathF.Ceiling(stockSize / cellSize));
        int n = CellsPerSide;
        long cells = (long)n * n * n;
        var count = new int[cells + 1];

        for (int s = 0; s < segments.Length; s++)
        {
            GetRange(segments[s], out int x0, out int x1, out int y0, out int y1, out int z0, out int z1);
            for (int z = z0; z <= z1; z++)
                for (int y = y0; y <= y1; y++)
                    for (int x = x0; x <= x1; x++)
                        count[(long)(z * n + y) * n + x + 1]++;
        }
        for (long i = 1; i <= cells; i++) count[i] += count[i - 1];
        CellStart = count;
        Refs = new int[count[cells]];

        var cursor = new int[cells];
        for (int s = 0; s < segments.Length; s++)
        {
            GetRange(segments[s], out int x0, out int x1, out int y0, out int y1, out int z0, out int z1);
            for (int z = z0; z <= z1; z++)
                for (int y = y0; y <= y1; y++)
                    for (int x = x0; x <= x1; x++)
                    {
                        long c = (long)(z * n + y) * n + x;
                        Refs[CellStart[c] + cursor[c]++] = s;
                    }
        }
    }

    private void GetRange(in PathSegment s, out int x0, out int x1, out int y0, out int y1, out int z0, out int z1)
    {
        int n = CellsPerSide;
        float p = BuildPad, inv = 1f / CellSize;
        x0 = Math.Clamp((int)((MathF.Min(s.A.X, s.B.X) - p) * inv), 0, n - 1);
        x1 = Math.Clamp((int)((MathF.Max(s.A.X, s.B.X) + p) * inv), 0, n - 1);
        y0 = Math.Clamp((int)((MathF.Min(s.A.Y, s.B.Y) - p) * inv), 0, n - 1);
        y1 = Math.Clamp((int)((MathF.Max(s.A.Y, s.B.Y) + p) * inv), 0, n - 1);
        z0 = Math.Clamp((int)((MathF.Min(s.A.Z, s.B.Z) - p) * inv), 0, n - 1);
        z1 = Math.Clamp((int)((MathF.Max(s.A.Z, s.B.Z) + p) * inv), 0, n - 1);
    }

    public ReadOnlySpan<int> QueryCell(int x, int y, int z)
    {
        long c = (long)(z * CellsPerSide + y) * CellsPerSide + x;
        return Refs.AsSpan(CellStart[c], CellStart[c + 1] - CellStart[c]);
    }

    public long MemoryBytes => (long)CellStart.Length * 4 + (long)Refs.Length * 4;

    public (long totalRefs, int maxPerCell, int p99PerCell, double avgNonEmpty) Stats()
    {
        long total = Refs.Length;
        int cells = CellStart.Length - 1;
        var sizes = new List<int>();
        int max = 0;
        for (int c = 0; c < cells; c++)
        {
            int len = CellStart[c + 1] - CellStart[c];
            if (len > 0) { sizes.Add(len); if (len > max) max = len; }
        }
        sizes.Sort();
        int p99 = sizes.Count > 0 ? sizes[(int)(sizes.Count * 0.99)] : 0;
        double avg = sizes.Count > 0 ? (double)total / sizes.Count : 0;
        return (total, max, p99, avg);
    }
}
