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
    /// Non-cutting moves are skipped; cutting moves are split so each segment length <= maxStep.
    /// </summary>
    public static PathSegment[] BuildCuttingSegments(ReadOnlySpan<ToolMove> moves, float maxStep)
    {
        if (moves.Length == 0)
        {
            return [];
        }

        if (maxStep <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(maxStep));
        }

        int estimated = 0;
        for (int i = 0; i < moves.Length; i++)
        {
            ref readonly var move = ref moves[i];
            if (!move.IsCutting)
            {
                continue;
            }

            float len = Vector3.Distance(move.A, move.B);
            estimated += Math.Max(1, (int)MathF.Ceiling(len / maxStep));
        }

        if (estimated == 0)
        {
            return [];
        }

        var segments = new PathSegment[estimated];
        int w = 0;
        for (int i = 0; i < moves.Length; i++)
        {
            ref readonly var move = ref moves[i];
            if (!move.IsCutting)
            {
                continue;
            }

            float len = Vector3.Distance(move.A, move.B);
            int pieces = Math.Max(1, (int)MathF.Ceiling(len / maxStep));
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
}
