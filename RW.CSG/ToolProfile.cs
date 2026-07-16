using System.Numerics;

namespace RW.CSG;

/// <summary>
/// Revolved tool profile baked to a signed-distance LUT over the (r,z) half-plane.
/// The polygon is stored MIRRORED across the axis (x in [-Rmax,+Rmax]) so the 2D polygon SDF
/// sampled at (r>=0, z) is the EXACT 3D SDF of the solid of revolution (nearest point on a
/// surface of revolution lies in the meridian plane of the query point).
/// Convention: d &lt; 0 inside the tool (material of the cutter), d &gt; 0 outside. Units: millimeters.
/// </summary>
public sealed class ToolProfile
{
    public readonly float[] Lut;          // row-major [j*LutWidth + i], i->r, j->z
    public readonly int LutWidth, LutHeight;
    public readonly float Rmax, Zmin, Zmax;
    public readonly float Z0;             // z of LUT row 0
    public readonly float CellR, CellZ, InvCellR, InvCellZ;
    public readonly float Epsilon;        // half LUT cell: subtracted to avoid bilinear overestimation
    public readonly float Pad;            // LUT domain margin beyond tool bounds
    public readonly float BallRadius;     // bounding sphere of the tool, centered on axis at BallCenterZ
    public readonly float BallCenterZ;
    /// <summary>True if the LUT SDF is non-decreasing in r at every z (single radial interval —
    /// holds for all common cutters incl. single-flange undercuts). Required for stroke-merged
    /// stamping, which substitutes min-over-radii before the profile lookup.</summary>
    public readonly bool MonotoneR;

    /// <summary>The source stack when this profile was baked from revolved 2D primitives.</summary>
    public SweptVolumeStack? ProfileStack { get; }

    internal readonly record struct ToolBound(float CenterZ, float Radius);
    internal readonly ToolBound[] Bounds;

    private readonly Vector2[]? _poly;    // mirrored closed polygon (for exact reference evals)
    private readonly bool[] _monotoneRows;

    public ToolProfile(Vector2[] mirroredPolygon, int lutSize, float pad)
        : this(mirroredPolygon, null, lutSize, lutSize, pad)
    {
    }

    /// <summary>
    /// Bake an exact stacked-primitive profile. Separate radial/axial dimensions keep a long,
    /// slender machine tool accurate without wasting a square 1024² LUT.
    /// </summary>
    public ToolProfile(SweptVolumeStack stack, int radialLutSize = 256,
        int axialLutSize = 256, float pad = 2f)
        : this(null, stack, radialLutSize, axialLutSize, pad)
    {
    }

    private ToolProfile(Vector2[]? mirroredPolygon, SweptVolumeStack? stack,
        int lutWidth, int lutHeight, float pad)
    {
        if (lutWidth < 2) throw new ArgumentOutOfRangeException(nameof(lutWidth));
        if (lutHeight < 2) throw new ArgumentOutOfRangeException(nameof(lutHeight));
        if (!float.IsFinite(pad) || pad <= 0f) throw new ArgumentOutOfRangeException(nameof(pad));
        if (mirroredPolygon is null && (stack is null || stack.Segments.Count == 0))
            throw new ArgumentException("A tool profile needs a polygon or at least one stacked primitive.");
        if (mirroredPolygon is not null && mirroredPolygon.Length < 3)
            throw new ArgumentException("A profile polygon needs at least three vertices.", nameof(mirroredPolygon));

        _poly = mirroredPolygon;
        ProfileStack = stack is null ? null : new SweptVolumeStack(stack.Segments);
        LutWidth = lutWidth;
        LutHeight = lutHeight;
        Pad = pad;
        float rmax = 0, zmin = float.MaxValue, zmax = float.MinValue;
        if (_poly is not null)
        {
            foreach (var v in _poly)
            {
                rmax = MathF.Max(rmax, MathF.Abs(v.X));
                zmin = MathF.Min(zmin, v.Y);
                zmax = MathF.Max(zmax, v.Y);
            }
        }
        else
        {
            rmax = ProfileStack!.MaxRadius;
            zmin = 0f;
            zmax = ProfileStack.TotalHeight;
        }
        Rmax = rmax; Zmin = zmin; Zmax = zmax;
        float halfH = (zmax - zmin) * 0.5f;
        BallCenterZ = (zmin + zmax) * 0.5f;
        BallRadius = MathF.Sqrt(rmax * rmax + halfH * halfH);

        float domR = rmax + pad;
        float domZ = (zmax - zmin) + 2 * pad;
        Z0 = zmin - pad;
        CellR = domR / (lutWidth - 1);
        CellZ = domZ / (lutHeight - 1);
        InvCellR = 1f / CellR;
        InvCellZ = 1f / CellZ;
        Epsilon = 0.5f * MathF.Max(CellR, CellZ);

        Lut = new float[lutWidth * lutHeight];
        for (int j = 0; j < lutHeight; j++)
        {
            float z = Z0 + j * CellZ;
            for (int i = 0; i < lutWidth; i++)
                Lut[j * lutWidth + i] = ExactSdf(new Vector2(i * CellR, z));
        }

        _monotoneRows = new bool[lutHeight];
        bool allMonotone = true;
        for (int j = 0; j < lutHeight; j++)
        {
            bool rowMonotone = true;
            for (int i = 1; i < lutWidth; i++)
                if (Lut[j * lutWidth + i] < Lut[j * lutWidth + i - 1] - 1e-4f)
                {
                    rowMonotone = false;
                    break;
                }
            _monotoneRows[j] = rowMonotone;
            allMonotone &= rowMonotone;
        }
        MonotoneR = allMonotone;

        if (ProfileStack is null)
        {
            Bounds = [new ToolBound(BallCenterZ, BallRadius)];
        }
        else
        {
            Bounds = new ToolBound[ProfileStack.Segments.Count];
            for (int i = 0; i < Bounds.Length; i++)
            {
                SweptVolumeSegment segment = ProfileStack.Segments[i];
                float halfHeight = 0.5f * segment.Height;
                float segmentRadius = MathF.Max(segment.BottomRadius, segment.TopRadius);
                Bounds[i] = new ToolBound(segment.AxialOffset + halfHeight,
                    MathF.Sqrt(segmentRadius * segmentRadius + halfHeight * halfHeight));
            }
        }
    }

    /// <summary>Exact source-profile SDF (reference / verification path, not the hot loop).</summary>
    public float ExactSdf(Vector2 p)
    {
        if (ProfileStack is not null)
            return ProfileStack.EvaluateSdf(MathF.Abs(p.X), p.Y);

        var poly = _poly!;
        float d = float.MaxValue;
        float s = 1f;
        for (int i = 0, k = poly.Length - 1; i < poly.Length; k = i++)
        {
            Vector2 a = poly[k], b = poly[i];
            Vector2 e = b - a, w = p - a;
            float t = Math.Clamp(Vector2.Dot(w, e) / Vector2.Dot(e, e), 0f, 1f);
            Vector2 diff = w - e * t;
            d = MathF.Min(d, Vector2.Dot(diff, diff));
            bool c0 = p.Y >= a.Y, c1 = p.Y < b.Y, c2 = e.X * w.Y > e.Y * w.X;
            if ((c0 && c1 && c2) || (!c0 && !c1 && !c2)) s = -s;
        }
        return s * MathF.Sqrt(d);
    }

    /// <summary>Bilinear LUT sample at radius r (>=0) and height z. Clamped at domain edges
    /// (out-of-domain values saturate near +Pad, i.e. "far air" — safe: no material change).</summary>
    public float Sample(float r, float z)
    {
        float u = r * InvCellR;
        float v = (z - Z0) * InvCellZ;
        int width = LutWidth, height = LutHeight;
        if (u < 0) u = 0; else if (u > width - 1.0001f) u = width - 1.0001f;
        if (v < 0) v = 0; else if (v > height - 1.0001f) v = height - 1.0001f;
        int i = (int)u, j = (int)v;
        float fu = u - i, fv = v - j;
        var lut = Lut;
        int idx = j * width + i;
        float d00 = lut[idx], d10 = lut[idx + 1];
        float d01 = lut[idx + width], d11 = lut[idx + width + 1];
        float d0 = d00 + (d10 - d00) * fu;
        float d1 = d01 + (d11 - d01) * fu;
        return d0 + (d1 - d0) * fv;
    }

    /// <summary>Whether radial SDF lookup is monotone throughout a particular axial interval.</summary>
    public bool IsRadiallyMonotone(float zMin, float zMax)
    {
        if (zMin > zMax) (zMin, zMax) = (zMax, zMin);
        int j0 = Math.Clamp((int)MathF.Floor((zMin - Z0) * InvCellZ), 0, LutHeight - 1);
        int j1 = Math.Clamp((int)MathF.Ceiling((zMax - Z0) * InvCellZ), 0, LutHeight - 1);
        for (int j = j0; j <= j1; j++)
            if (!_monotoneRows[j]) return false;
        return true;
    }

    /// <summary>Maximum physical profile radius that exists within an axial interval.</summary>
    public float MaxRadiusInRange(float zMin, float zMax)
        => ProfileStack?.MaxRadiusInRange(zMin, zMax) ?? Rmax;

    // ---- factory profiles -------------------------------------------------

    /// <summary>Cylinder: radius x height, tip at z=0 (cuts below the spindle).</summary>
    public static ToolProfile FlatEndmill(float radius, float height, int lutSize = 64, float pad = 2f)
        => new([new(-radius, 0), new(radius, 0), new(radius, height), new(-radius, height)], lutSize, pad);

    /// <summary>Undercut tool: wide disc (discR x discH) below a narrow shank — concave (r,z) profile.</summary>
    public static ToolProfile TSlotCutter(float shankR, float discR, float discH, float totalH,
                                          int lutSize = 64, float pad = 2f)
        => new([
            new(-discR, 0), new(discR, 0),
            new(discR, discH), new(shankR, discH),
            new(shankR, totalH), new(-shankR, totalH),
            new(-shankR, discH), new(-discR, discH),
        ], lutSize, pad);
}
