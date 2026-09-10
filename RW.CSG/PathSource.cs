using System.Numerics;

namespace RW.CSG;

/// <summary>
/// Toolpath preprocessing helpers used by the phased integration loop.
/// Filters non-cutting moves and subdivides long cuts to a target stamplet length.
/// </summary>
public static class PathSource
{
    public readonly record struct ToolMove(Vector3 A, Vector3 B, bool IsCutting);

    /// <summary>Build move records from consecutive points (all moves marked cutting).</summary>
    public static ToolMove[] FromPoints(Vector3[] points)
    {
        if (points.Length < 2)
        {
            return [];
        }

        var moves = new ToolMove[points.Length - 1];
        for (int i = 0; i < moves.Length; i++)
        {
            moves[i] = new ToolMove(points[i], points[i + 1], true);
        }

        return moves;
    }

    /// <summary>
    /// Converts a move list to stamping segments.
    /// Non-cutting moves are skipped; cutting moves are split to the requested length
    /// and optional axial interval. Display callers can bound simplification separately.
    /// </summary>
    public static PathSegment[] BuildCuttingSegments(ReadOnlySpan<ToolMove> moves, float maxStep,
        float simplificationTolerance = 0, CancellationToken cancellationToken = default,
        float maxVerticalStep = float.PositiveInfinity)
    {
        if (moves.Length == 0)
        {
            return [];
        }

        if (!float.IsFinite(maxStep) || maxStep <= 0 || float.IsNaN(maxVerticalStep) || maxVerticalStep <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(maxStep));
        }
        if (!float.IsFinite(simplificationTolerance) || simplificationTolerance < 0)
            throw new ArgumentOutOfRangeException(nameof(simplificationTolerance));

        if (simplificationTolerance > 0) moves = CompactCuttingMoves(moves, simplificationTolerance, cancellationToken);

        int estimated = 0;
        for (int i = 0; i < moves.Length; i++)
        {
            if ((i & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
            ref readonly var move = ref moves[i];
            if (!move.IsCutting)
            {
                continue;
            }

            float len = Vector3.Distance(move.A, move.B);
            estimated = checked(estimated + Math.Max(1, (int)MathF.Ceiling(MathF.Max(len / maxStep,
                MathF.Abs(move.B.Z - move.A.Z) / maxVerticalStep))));
        }

        if (estimated == 0)
        {
            return [];
        }

        var segments = new PathSegment[estimated];
        int w = 0;
        for (int i = 0; i < moves.Length; i++)
        {
            if ((i & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
            ref readonly var move = ref moves[i];
            if (!move.IsCutting)
            {
                continue;
            }

            float len = Vector3.Distance(move.A, move.B);
            int pieces = Math.Max(1, (int)MathF.Ceiling(MathF.Max(len / maxStep,
                MathF.Abs(move.B.Z - move.A.Z) / maxVerticalStep)));
            for (int p = 0; p < pieces; p++)
            {
                float t0 = p / (float)pieces;
                float t1 = (p + 1) / (float)pieces;
                segments[w++] = new PathSegment(Vector3.Lerp(move.A, move.B, t0), Vector3.Lerp(move.A, move.B, t1));
            }
        }

        if (w == segments.Length)
        {
            return segments;
        }

        Array.Resize(ref segments, w);
        return segments;
    }

    /// <summary>Compact contiguous cutting polylines to a finite 3D chord-error bound.
    /// Callers opt in at their display resolution; exact machining/stock proofs keep zero
    /// tolerance. Gaps and non-cutting moves always break a run. Bounded windows cap work.</summary>
    public static ToolMove[] CompactCuttingMoves(ReadOnlySpan<ToolMove> moves, float tolerance,
        CancellationToken cancellationToken = default)
    {
        if (!float.IsFinite(tolerance) || tolerance <= 0) throw new ArgumentOutOfRangeException(nameof(tolerance));
        var result = new List<ToolMove>();
        var points = new List<Vector3>(1025);
        var keep = new bool[1025];
        var pending = new Stack<(int First, int Last)>();
        void Flush()
        {
            if (points.Count < 2) { points.Clear(); return; }
            Array.Clear(keep); keep[0] = keep[points.Count - 1] = true;
            pending.Push((0, points.Count - 1));
            while (pending.TryPop(out var interval))
            {
                cancellationToken.ThrowIfCancellationRequested();
                var a = points[interval.First]; var d = points[interval.Last] - a;
                float lengthSquared = d.LengthSquared(), maximum = tolerance * tolerance; int split = -1;
                for (int i = interval.First + 1; i < interval.Last; i++)
                {
                    float t = lengthSquared > 1e-20f ? Math.Clamp(Vector3.Dot(points[i] - a, d) / lengthSquared, 0, 1) : 0;
                    float error = Vector3.DistanceSquared(points[i], a + t * d);
                    if (error > maximum) { maximum = error; split = i; }
                }
                if (split < 0) continue;
                keep[split] = true; pending.Push((interval.First, split)); pending.Push((split, interval.Last));
            }
            var previous = points[0];
            for (int i = 1; i < points.Count; i++) if (keep[i])
            { result.Add(new ToolMove(previous, points[i], true)); previous = points[i]; }
            points.Clear();
        }
        int processed = 0;
        foreach (var move in moves)
        {
            if ((processed++ & 4095) == 0) cancellationToken.ThrowIfCancellationRequested();
            if (!move.IsCutting) { Flush(); result.Add(move); continue; }
            if (points.Count > 0 && points[^1] != move.A) Flush();
            if (points.Count == 0) points.Add(move.A);
            points.Add(move.B);
            if (points.Count == 1025) Flush();
        }
        Flush();
        return result.ToArray();
    }
}
