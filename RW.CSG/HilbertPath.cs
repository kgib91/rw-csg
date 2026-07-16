using System.Numerics;

namespace RW.CSG;

/// <summary>Hilbert-curve toolpath generator (XY plane + smooth egg-carton Z modulation). Units: mm.</summary>
public static class HilbertPath
{
    /// <summary>
    /// Point count is 4^order. Points span [margin, size-margin]² in XY.
    /// zFreq is in cycles/mm (not radians/mm).
    /// </summary>
    public static Vector3[] Generate(int order, float size, float margin, float zBase, float zAmp, float zFreq)
        => Generate(order, size, size, margin, zBase, zAmp, zFreq);

    /// <summary>
    /// Point count is 4^order. Points span the rectangular
    /// [margin,sizeX-margin] × [margin,sizeY-margin] XY domain.
    /// </summary>
    public static Vector3[] Generate(int order, float sizeX, float sizeY, float margin,
        float zBase, float zAmp, float zFreq)
    {
        int side = 1 << order;
        long count = (long)side * side;
        var pts = new Vector3[count];
        float scaleX = (sizeX - 2 * margin) / (side - 1);
        float scaleY = (sizeY - 2 * margin) / (side - 1);
        float angularFreq = zFreq * (2f * MathF.PI);
        for (long i = 0; i < count; i++)
        {
            pts[i] = GeneratePoint(side, i, margin, scaleX, scaleY, zBase, zAmp, angularFreq);
        }
        return pts;
    }

    /// <summary>
    /// Generates the same path directly as segments, avoiding the temporary point array and
    /// second full pass used by <see cref="Generate"/> followed by <see cref="ToSegments"/>.
    /// </summary>
    public static PathSegment[] GenerateSegments(int order, float size, float margin,
        float zBase, float zAmp, float zFreq)
        => GenerateSegments(order, size, size, margin, zBase, zAmp, zFreq);

    /// <summary>Generates a rectangular Hilbert path directly as consecutive segments.</summary>
    public static PathSegment[] GenerateSegments(int order, float sizeX, float sizeY, float margin,
        float zBase, float zAmp, float zFreq)
    {
        int side = 1 << order;
        long pointCount = (long)side * side;
        if (pointCount - 1 > int.MaxValue)
        {
            throw new ArgumentOutOfRangeException(nameof(order), "The generated segment array is too large.");
        }

        var segments = new PathSegment[(int)pointCount - 1];
        float scaleX = (sizeX - 2 * margin) / (side - 1);
        float scaleY = (sizeY - 2 * margin) / (side - 1);
        float angularFreq = zFreq * (2f * MathF.PI);

        void FillRange(int start, int end)
        {
            Vector3 previous = GeneratePoint(side, start, margin, scaleX, scaleY, zBase, zAmp, angularFreq);
            for (int i = start; i < end; i++)
            {
                Vector3 current = GeneratePoint(side, (long)i + 1, margin, scaleX, scaleY, zBase, zAmp, angularFreq);
                segments[i] = new PathSegment(previous, current);
                previous = current;
            }
        }

        int workerCount = segments.Length >= 65_536
            ? Math.Min(Environment.ProcessorCount, segments.Length)
            : 1;
        if (workerCount == 1)
        {
            FillRange(0, segments.Length);
        }
        else
        {
            Parallel.For(0, workerCount, worker =>
            {
                int start = (int)((long)segments.Length * worker / workerCount);
                int end = (int)((long)segments.Length * (worker + 1) / workerCount);
                FillRange(start, end);
            });
        }
        return segments;
    }

    /// <summary>Consecutive point pairs as segments (already stamplet-scale for order ≥ 10 on 300 mm stock).</summary>
    public static PathSegment[] ToSegments(Vector3[] pts, int start, int count)
    {
        var segs = new PathSegment[count];
        for (int i = 0; i < count; i++)
            segs[i] = new PathSegment(pts[start + i], pts[start + i + 1]);
        return segs;
    }

    private static Vector3 GeneratePoint(int side, long index, float margin, float scaleX,
        float scaleY, float zBase, float zAmp, float angularFreq)
    {
        D2XY(side, index, out int x, out int y);
        float px = margin + x * scaleX;
        float py = margin + y * scaleY;
        // Smooth XY-driven egg-carton surface: z varies by XY position, not path index.
        float z = zBase + zAmp * (MathF.Sin(px * angularFreq) * MathF.Sin(py * angularFreq));
        return new Vector3(px, py, z);
    }

    // Standard Hilbert d->(x,y) mapping.
    private static void D2XY(int side, long d, out int x, out int y)
    {
        x = 0; y = 0;
        long t = d;
        for (int s = 1; s < side; s <<= 1)
        {
            int rx = (int)(1 & (t / 2));
            int ry = (int)(1 & (t ^ rx));
            // rotate
            if (ry == 0)
            {
                if (rx == 1) { x = s - 1 - x; y = s - 1 - y; }
                (x, y) = (y, x);
            }
            x += s * rx;
            y += s * ry;
            t /= 4;
        }
    }
}
