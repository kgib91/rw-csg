using System.Runtime.CompilerServices;

namespace RW.CSG;

/// <summary>Prepared exact meridian distance for the existing primitive stack semantics.
/// Axial and radial wall bounds prune only surfaces farther than the best exact candidate.</summary>
internal sealed class PreparedSweptVolumeSdf
{
    private readonly Section[] _sections;
    private readonly Shoulder[] _shoulders;
    private readonly float _height, _bottomCap, _topCap;
    private readonly struct Shoulder(float z, float r0, float r1)
    {
        internal readonly float Z = z, R0 = r0, R1 = r1;
    }
    private readonly struct Section(SweptVolumeSegment s)
    {
        internal readonly SweptVolumeSegmentType Type = s.Type;
        internal readonly float Z0 = s.AxialOffset, Z1 = s.AxialOffset + s.Height;
        internal readonly float R0 = s.BottomRadius, R1 = s.TopRadius;
        internal readonly float H = s.Height, DR = s.TopRadius - s.BottomRadius;
        internal readonly float InvH = 1 / s.Height;
        internal readonly float InvLength2 = 1 / ((s.TopRadius - s.BottomRadius) * (s.TopRadius - s.BottomRadius) + s.Height * s.Height);
        // The existing torus evaluator uses the circle distance throughout the
        // axial slab, including its inner radial half. Preserve those semantics.
        internal readonly float MinR = s.Type == SweptVolumeSegmentType.Torus ? 2 * s.BottomRadius - s.TopRadius : MathF.Min(s.BottomRadius, s.TopRadius), MaxR = MathF.Max(s.BottomRadius, s.TopRadius);

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal float LowerBoundSquared(float r, float z)
        {
            float dr = MathF.Max(0, MathF.Max(MinR - r, r - MaxR));
            float dz = MathF.Max(0, MathF.Max(Z0 - z, z - Z1));
            return dr * dr + dz * dz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal bool Contains(float r, float z)
        {
            if (z < Z0 || z > Z1) return false;
            if (Type == SweptVolumeSegmentType.Hemisphere)
            {
                float dz = Z1 - z;
                return r * r + dz * dz < R1 * R1;
            }
            if (Type == SweptVolumeSegmentType.Torus)
            {
                if (r < R0) return true;
                float dz = Z1 - z, dr = r - R0, minor = R1 - R0;
                return dr * dr + dz * dz < minor * minor;
            }
            return r < R0 + DR * ((z - Z0) * InvH);
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        internal float DistanceSquared(float r, float z)
        {
            if (Type == SweptVolumeSegmentType.Hemisphere)
            {
                float dz = z - Z1;
                if (dz > 0) { float dr = r - R1; return dr * dr + dz * dz; }
                if (z < Z0) { dz = z - Z0; return r * r + dz * dz; }
                float d = MathF.Sqrt(r * r + dz * dz) - R1;
                return d * d;
            }
            if (Type == SweptVolumeSegmentType.Torus)
            {
                float dz = z - Z1, dr = r - R0;
                if (dz > 0) { dr = r - R1; return dr * dr + dz * dz; }
                if (z < Z0) { dz = z - Z0; return dr * dr + dz * dz; }
                float d = MathF.Sqrt(dr * dr + dz * dz) - (R1 - R0);
                return d * d;
            }
            float px = r - R0, pz = z - Z0;
            float t = Math.Clamp((px * DR + pz * H) * InvLength2, 0, 1);
            px -= DR * t; pz -= H * t;
            return px * px + pz * pz;
        }
    }

    internal PreparedSweptVolumeSdf(SweptVolumeStack stack)
    {
        _height = stack.TotalHeight;
        _sections = stack.Segments.Select(s => new Section(s)).ToArray();
        _topCap = _sections[^1].R1;
        _bottomCap = _sections[0].Type != SweptVolumeSegmentType.Hemisphere && _sections[0].R0 > .001f ? _sections[0].R0 : -1;
        var shoulders = new List<Shoulder>();
        for (int i = 0; i + 1 < _sections.Length; i++)
        {
            float lo = MathF.Min(_sections[i].R1, _sections[i + 1].R0), hi = MathF.Max(_sections[i].R1, _sections[i + 1].R0);
            if (hi - lo > .001f) shoulders.Add(new(_sections[i].Z1, lo, hi));
        }
        _shoulders = shoulders.ToArray();
    }

    [MethodImpl(MethodImplOptions.AggressiveOptimization)]
    internal float Evaluate(float r, float z)
    {
        r = MathF.Abs(r);
        // Find the wall alongside the query first. Its distance is normally the
        // final result and tightly bounds all more distant axial sections.
        int active = 0;
        while (active + 1 < _sections.Length && z > _sections[active].Z1) active++;
        ref readonly Section nearest = ref _sections[active];
        bool inside = nearest.Contains(r, z)
            || active + 1 < _sections.Length && z == nearest.Z1 && _sections[active + 1].Contains(r, z);
        float best = nearest.DistanceSquared(r, z);
        for (int i = 0; i < _sections.Length; i++)
        {
            if (i == active) continue;
            ref readonly Section other = ref _sections[i];
            if (other.LowerBoundSquared(r, z) > best) continue;
            best = MathF.Min(best, other.DistanceSquared(r, z));
        }
        foreach (var shoulder in _shoulders)
        {
            float dz = z - shoulder.Z;
            if (dz * dz > best) continue;
            float dr = r - Math.Clamp(r, shoulder.R0, shoulder.R1);
            best = MathF.Min(best, dr * dr + dz * dz);
        }
        float topDz = z - _height;
        if (topDz * topDz <= best)
        {
            float dr = MathF.Max(0, r - _topCap);
            best = MathF.Min(best, dr * dr + topDz * topDz);
        }
        if (_bottomCap >= 0 && z * z <= best)
        {
            float dr = MathF.Max(0, r - _bottomCap);
            best = MathF.Min(best, dr * dr + z * z);
        }
        float distance = MathF.Sqrt(best);
        return inside ? -distance : distance;
    }
}
