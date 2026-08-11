using System.Numerics;
using System.Runtime.CompilerServices;

namespace RW.CSG;

/// <summary>One toolpath segment (stamplet when |B-A| ~ voxel scale). Units: mm.</summary>
public readonly struct PathSegment
{
    public readonly Vector3 A;
    public readonly Vector3 B;
    internal readonly float InvXyLengthSquared;

    public PathSegment(Vector3 a, Vector3 b)
    {
        A = a;
        B = b;
        float ex = b.X - a.X;
        float ey = b.Y - a.Y;
        float squared = ex * ex + ey * ey;
        InvXyLengthSquared = squared > 1e-12f ? 1f / squared : 0f;
    }
}

/// <summary>
/// Analytic swept-tool SDF evaluation (tool axis vertical, 3-axis machining).
/// Hot path: K=3 samples over t + parabolic refinement, minus the profile's epsilon margin.
/// </summary>
public static class SweptBrush
{
    /// <summary>Swept brush SDF at point p for segment A-&gt;B. ~4 bilinear LUT samples.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static float Eval(ToolProfile prof, in Vector3 a, in Vector3 b, float px, float py, float pz)
    {
        float d0 = SampleAt(prof, a.X, a.Y, a.Z, px, py, pz);
        float mx = (a.X + b.X) * 0.5f, my = (a.Y + b.Y) * 0.5f, mz = (a.Z + b.Z) * 0.5f;
        float d1 = SampleAt(prof, mx, my, mz, px, py, pz);
        float d2 = SampleAt(prof, b.X, b.Y, b.Z, px, py, pz);

        float best = MathF.Min(MathF.Min(d0, d1), d2);

        // Parabolic refinement: fit d(t) through t=0,1/2,1; evaluate at the vertex if it is a
        // strictly interior minimum.
        float denom = d0 - 2f * d1 + d2;
        if (denom > 1e-9f)
        {
            float t = 0.5f - 0.25f * (d2 - d0) / denom;
            if (t > 0f && t < 1f)
            {
                float qx = a.X + (b.X - a.X) * t, qy = a.Y + (b.Y - a.Y) * t, qz = a.Z + (b.Z - a.Z) * t;
                best = MathF.Min(best, SampleAt(prof, qx, qy, qz, px, py, pz));
            }
        }
        return best - prof.Epsilon;
    }

    /// <summary>
    /// Swept brush SDF with the profile axis aligned to an arbitrary signed stock axis.
    /// The tool tip remains at <paramref name="a"/>/<paramref name="b"/>; only the cutter's
    /// axial profile direction is transformed.
    /// </summary>
    public static float EvalOriented(
        ToolProfile prof,
        in Vector3 a,
        in Vector3 b,
        ToolAxisDirection axis,
        in Vector3 point)
    {
        Vector3 axisVector = axis.ToVector();
        float d0 = SampleOriented(prof, a, axisVector, point);
        Vector3 midpoint = (a + b) * 0.5f;
        float d1 = SampleOriented(prof, midpoint, axisVector, point);
        float d2 = SampleOriented(prof, b, axisVector, point);
        float best = MathF.Min(MathF.Min(d0, d1), d2);

        float denominator = d0 - 2f * d1 + d2;
        if (denominator > 1e-9f)
        {
            float t = 0.5f - 0.25f * (d2 - d0) / denominator;
            if (t is > 0f and < 1f)
                best = MathF.Min(best, SampleOriented(prof, Vector3.Lerp(a, b, t), axisVector, point));
        }

        return best - prof.Epsilon;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float SampleAt(ToolProfile prof, float ax, float ay, float az, float px, float py, float pz)
    {
        float dx = px - ax, dy = py - ay;
        return prof.Sample(MathF.Sqrt(dx * dx + dy * dy), pz - az);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float SampleOriented(ToolProfile prof, in Vector3 toolTip, in Vector3 axis, in Vector3 point)
    {
        Vector3 delta = point - toolTip;
        float axial = Vector3.Dot(delta, axis);
        float radialSquared = MathF.Max(0f, delta.LengthSquared() - axial * axial);
        return prof.Sample(MathF.Sqrt(radialSquared), axial);
    }

    /// <summary>
    /// Swept brush SDF for an arbitrarily LONG segment, restricted to t ∈ [tLo, tHi]:
    /// multi-start + golden-section refinement of the best local minima. The seed count scales
    /// with the WINDOW's world length (≈ 2 seeds per tool radius), so callers MUST t-clip the
    /// window to a bounded region (brick slab) first — then cost is independent of the original
    /// segment length. An unclipped long segment with fixed seeds silently misses minima
    /// (measured: 4 mm error on a 200 mm segment with 8 fixed seeds — trap §9.3).
    /// Handles multi-minima of concave (undercut) profiles via the multi-start. Includes −ε.
    /// </summary>
    public static float EvalSegment(ToolProfile prof, in Vector3 a, in Vector3 b,
        float px, float py, float pz, float tLo, float tHi, int seeds = -1)
    {
        if (seeds < 0)
        {
            float wx = (b.X - a.X) * (tHi - tLo), wy = (b.Y - a.Y) * (tHi - tLo), wz = (b.Z - a.Z) * (tHi - tLo);
            float len = MathF.Sqrt(wx * wx + wy * wy + wz * wz);
            seeds = Math.Clamp((int)(len / (0.5f * prof.Rmax)) + 3, 8, 96);
        }
        Span<float> f = stackalloc float[97];
        f = f[..(seeds + 1)];
        float dt = (tHi - tLo) / seeds;
        float best = float.MaxValue;
        for (int i = 0; i <= seeds; i++)
        {
            float t = tLo + i * dt;
            f[i] = SampleAtT(prof, a, b, t, px, py, pz);
            if (f[i] < best) best = f[i];
        }
        // refine every interior local minimum (plus the boundary minima), golden section
        for (int i = 0; i <= seeds; i++)
        {
            bool locMin = (i == 0 || f[i] <= f[i - 1]) && (i == seeds || f[i] <= f[i + 1]);
            if (!locMin) continue;
            float lo = tLo + Math.Max(0, i - 1) * dt;
            float hi = tLo + Math.Min(seeds, i + 1) * dt;
            const float invPhi = 0.6180339887f;
            float x1 = hi - (hi - lo) * invPhi, x2 = lo + (hi - lo) * invPhi;
            float f1 = SampleAtT(prof, a, b, x1, px, py, pz);
            float f2 = SampleAtT(prof, a, b, x2, px, py, pz);
            for (int it = 0; it < 16; it++)
            {
                if (f1 < f2)
                {
                    hi = x2; x2 = x1; f2 = f1;
                    x1 = hi - (hi - lo) * invPhi;
                    f1 = SampleAtT(prof, a, b, x1, px, py, pz);
                }
                else
                {
                    lo = x1; x1 = x2; f1 = f2;
                    x2 = lo + (hi - lo) * invPhi;
                    f2 = SampleAtT(prof, a, b, x2, px, py, pz);
                }
            }
            best = MathF.Min(best, MathF.Min(f1, f2));
        }
        return best - prof.Epsilon;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float SampleAtT(ToolProfile prof, in Vector3 a, in Vector3 b, float t,
        float px, float py, float pz)
        => SampleAt(prof, a.X + (b.X - a.X) * t, a.Y + (b.Y - a.Y) * t, a.Z + (b.Z - a.Z) * t, px, py, pz);

    /// <summary>
    /// Clip a segment's t-range to the axis positions that can influence any voxel of the given
    /// box (box expanded XY by Rmax+band+ε; Z by the tool's z-extent + band + ε). False = no overlap.
    /// </summary>
    public static bool ClipSegmentT(ToolProfile prof, in Vector3 a, in Vector3 b,
        float bx0, float by0, float bz0, float bx1, float by1, float bz1, float band,
        out float tLo, out float tHi)
    {
        float padXY = prof.Rmax + band + prof.Epsilon;
        float x0 = bx0 - padXY, x1 = bx1 + padXY;
        float y0 = by0 - padXY, y1 = by1 + padXY;
        float z0 = bz0 - prof.Zmax - band - prof.Epsilon;
        float z1 = bz1 - prof.Zmin + band + prof.Epsilon;
        tLo = 0f; tHi = 1f;
        if (!Clip1D(a.X, b.X - a.X, x0, x1, ref tLo, ref tHi)) return false;
        if (!Clip1D(a.Y, b.Y - a.Y, y0, y1, ref tLo, ref tHi)) return false;
        if (!Clip1D(a.Z, b.Z - a.Z, z0, z1, ref tLo, ref tHi)) return false;
        return tLo < tHi;
    }

    private static bool Clip1D(float o, float d, float lo, float hi, ref float tLo, ref float tHi)
    {
        if (MathF.Abs(d) < 1e-12f) return o >= lo && o <= hi;
        float ta = (lo - o) / d, tb = (hi - o) / d;
        if (ta > tb) (ta, tb) = (tb, ta);
        tLo = MathF.Max(tLo, ta);
        tHi = MathF.Min(tHi, tb);
        return tLo <= tHi;
    }

    /// <summary>
    /// Conservative bounds of the swept brush SDF over a ball (brick bounding sphere).
    /// dmin: bounding-sphere lower bound (~20 flops) — if dmin &gt; +band the brush misses the brick.
    /// dmax: single-LUT-sample upper bound — if dmax &lt; −band the brush engulfs the brick.
    /// Both include the −epsilon of <see cref="Eval"/> so they bound what stamping actually writes.
    /// </summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    public static void ConservativeRange(ToolProfile prof, in Vector3 a, in Vector3 b,
        float cx, float cy, float cz, float ballRadius, out float dmin, out float dmax)
    {
        float ex = b.X - a.X, ey = b.Y - a.Y;
        float wx = cx - a.X, wy = cy - a.Y;
        float ee = ex * ex + ey * ey + (b.Z - a.Z) * (b.Z - a.Z);

        // A stack gets one enclosing sphere per primitive instead of one very loose sphere for
        // the full 103 mm assembly. The minimum of their swept-sphere lower bounds is still
        // conservative for the union, while allowing distant shaft/collar sections to cull.
        dmin = float.MaxValue;
        foreach (ToolProfile.ToolBound bound in prof.Bounds)
        {
            float azBound = a.Z + bound.CenterZ;
            float wz = cz - azBound;
            float tBound = ee > 1e-12f
                ? Math.Clamp((wx * ex + wy * ey + wz * (b.Z - a.Z)) / ee, 0f, 1f)
                : 0f;
            float qx = wx - ex * tBound, qy = wy - ey * tBound;
            float qz = wz - (b.Z - a.Z) * tBound;
            float lower = MathF.Sqrt(qx * qx + qy * qy + qz * qz)
                - bound.Radius - ballRadius - prof.Epsilon;
            dmin = MathF.Min(dmin, lower);
        }

        // Upper bound: evaluate the profile at that same t (any single t bounds the min over t).
        float az = a.Z + prof.BallCenterZ;
        float wzGlobal = cz - az;
        float t = ee > 1e-12f
            ? Math.Clamp((wx * ex + wy * ey + wzGlobal * (b.Z - a.Z)) / ee, 0f, 1f)
            : 0f;
        float sx = cx - (a.X + (b.X - a.X) * t), sy = cy - (a.Y + (b.Y - a.Y) * t);
        float sz = cz - (a.Z + (b.Z - a.Z) * t);
        float d = prof.Sample(MathF.Sqrt(sx * sx + sy * sy), sz);
        dmax = d - prof.Epsilon + ballRadius;
    }
}
