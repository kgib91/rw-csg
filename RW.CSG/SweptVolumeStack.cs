namespace RW.CSG;

/// <summary>Primitive used by a revolved, tip-to-shaft machine-tool profile.</summary>
public enum SweptVolumeSegmentType
{
    Cylinder = 0,
    Cone = 1,
    Hemisphere = 2,
    Torus = 3,
    TruncatedCone = 4
}

/// <summary>
/// One closed-profile wall segment. The 2D primitive lives in the (radius, axial-height)
/// half-plane and becomes a 3D solid when revolved around the tool axis. Units are millimeters.
/// </summary>
public readonly struct SweptVolumeSegment
{
    public SweptVolumeSegmentType Type { get; }
    public float AxialOffset { get; }
    public float Height { get; }
    public float BottomRadius { get; }
    public float TopRadius { get; }

    private SweptVolumeSegment(SweptVolumeSegmentType type, float axialOffset, float height,
        float bottomRadius, float topRadius)
    {
        Type = type;
        AxialOffset = axialOffset;
        Height = height;
        BottomRadius = bottomRadius;
        TopRadius = topRadius;
    }

    public static SweptVolumeSegment Cylinder(float radius, float height)
        => Create(SweptVolumeSegmentType.Cylinder, height, radius, radius);

    public static SweptVolumeSegment Cone(float bottomRadius, float topRadius, float height)
        => Create(SweptVolumeSegmentType.Cone, height, bottomRadius, topRadius);

    public static SweptVolumeSegment Hemisphere(float radius)
        => Create(SweptVolumeSegmentType.Hemisphere, radius, 0f, radius);

    /// <summary>Quarter-circle bull-nose wall. Height equals the minor radius.</summary>
    public static SweptVolumeSegment Torus(float majorRadius, float minorRadius)
        => Create(SweptVolumeSegmentType.Torus, minorRadius, majorRadius, majorRadius + minorRadius);

    public static SweptVolumeSegment TruncatedCone(float bottomRadius, float topRadius, float height)
        => Create(SweptVolumeSegmentType.TruncatedCone, height, bottomRadius, topRadius);

    private static SweptVolumeSegment Create(SweptVolumeSegmentType type, float height,
        float bottomRadius, float topRadius)
    {
        if (!float.IsFinite(height) || height <= 0f)
            throw new ArgumentOutOfRangeException(nameof(height), "Segment height must be finite and positive.");
        if (!float.IsFinite(bottomRadius) || bottomRadius < 0f)
            throw new ArgumentOutOfRangeException(nameof(bottomRadius), "Radius must be finite and non-negative.");
        if (!float.IsFinite(topRadius) || topRadius < 0f)
            throw new ArgumentOutOfRangeException(nameof(topRadius), "Radius must be finite and non-negative.");
        if (type == SweptVolumeSegmentType.Hemisphere && topRadius <= 0f)
            throw new ArgumentOutOfRangeException(nameof(topRadius), "Hemisphere radius must be positive.");
        if (type == SweptVolumeSegmentType.Torus && topRadius <= bottomRadius)
            throw new ArgumentOutOfRangeException(nameof(topRadius), "Torus minor radius must be positive.");

        return new SweptVolumeSegment(type, 0f, height, bottomRadius, topRadius);
    }

    internal SweptVolumeSegment AtOffset(float axialOffset)
        => new(Type, axialOffset, Height, BottomRadius, TopRadius);

    internal float RadiusAt(float axialHeight)
    {
        float z0 = AxialOffset;
        float z1 = z0 + Height;
        float z = Math.Clamp(axialHeight, z0, z1);

        if (Type == SweptVolumeSegmentType.Hemisphere)
        {
            float dz = z1 - z;
            return MathF.Sqrt(MathF.Max(0f, TopRadius * TopRadius - dz * dz));
        }

        if (Type == SweptVolumeSegmentType.Torus)
        {
            float minor = TopRadius - BottomRadius;
            float dz = z1 - z;
            return BottomRadius + MathF.Sqrt(MathF.Max(0f, minor * minor - dz * dz));
        }

        float t = (z - z0) / Height;
        return BottomRadius + (TopRadius - BottomRadius) * t;
    }

    internal float OuterWallDistanceSquared(float radius, float axialHeight)
    {
        float z0 = AxialOffset;
        float z1 = z0 + Height;

        if (Type == SweptVolumeSegmentType.Hemisphere)
        {
            float sphereRadius = TopRadius;
            float dz = axialHeight - z1;
            if (dz > 0f)
                return Square(radius - sphereRadius) + dz * dz;
            if (axialHeight < z0)
                return radius * radius + Square(axialHeight - z0);

            float circleDistance = MathF.Sqrt(radius * radius + dz * dz) - sphereRadius;
            return circleDistance * circleDistance;
        }

        if (Type == SweptVolumeSegmentType.Torus)
        {
            float minorRadius = TopRadius - BottomRadius;
            float dz = axialHeight - z1;
            if (axialHeight > z1)
                return Square(radius - TopRadius) + dz * dz;
            if (axialHeight < z0)
                return Square(radius - BottomRadius) + Square(axialHeight - z0);

            float circleDistance = MathF.Sqrt(Square(radius - BottomRadius) + dz * dz) - minorRadius;
            return circleDistance * circleDistance;
        }

        float px = radius - BottomRadius;
        float pz = axialHeight - z0;
        float ex = TopRadius - BottomRadius;
        float ez = Height;
        float denom = ex * ex + ez * ez;
        float t = Math.Clamp((px * ex + pz * ez) / denom, 0f, 1f);
        return Square(px - ex * t) + Square(pz - ez * t);
    }

    private static float Square(float value) => value * value;
}

/// <summary>
/// Ordered stack of revolved 2D primitives. Segment zero begins at the tool tip (axial height
/// zero); later segments continue toward the shaft. The stack is a radial solid, with explicit
/// shoulder faces where adjacent radii differ.
/// </summary>
public sealed class SweptVolumeStack
{
    private readonly List<SweptVolumeSegment> _segments = [];

    public IReadOnlyList<SweptVolumeSegment> Segments => _segments;
    public float TotalHeight { get; private set; }
    public float MaxRadius { get; private set; }

    public SweptVolumeStack() { }

    public SweptVolumeStack(IEnumerable<SweptVolumeSegment> segments)
    {
        foreach (SweptVolumeSegment segment in segments)
            Add(segment);
    }

    public void Add(SweptVolumeSegment segment)
    {
        SweptVolumeSegment placed = segment.AtOffset(TotalHeight);
        _segments.Add(placed);
        TotalHeight += placed.Height;
        MaxRadius = MathF.Max(MaxRadius, MathF.Max(placed.BottomRadius, placed.TopRadius));
    }

    /// <summary>Indices of joins whose two radii differ by more than the supplied tolerance.</summary>
    public List<int> GetDiscontinuities(float tolerance = 0.001f)
    {
        var result = new List<int>();
        for (int i = 0; i + 1 < _segments.Count; i++)
            if (MathF.Abs(_segments[i].TopRadius - _segments[i + 1].BottomRadius) > tolerance)
                result.Add(i);
        return result;
    }

    /// <summary>
    /// Exact signed distance in the meridian half-plane. Negative is inside the revolved tool;
    /// positive is outside. This follows the legacy stack semantics for primitive walls,
    /// shoulders, the top cap, and a flat bottom cap.
    /// </summary>
    public float EvaluateSdf(float radius, float axialHeight)
    {
        if (_segments.Count == 0)
            return float.PositiveInfinity;

        radius = MathF.Abs(radius);
        float minDistanceSquared = float.MaxValue;
        float insideRadius = 0f;
        bool inAxialRange = axialHeight >= 0f && axialHeight <= TotalHeight;

        foreach (SweptVolumeSegment segment in _segments)
        {
            float z0 = segment.AxialOffset;
            float z1 = z0 + segment.Height;
            if (inAxialRange && axialHeight >= z0 && axialHeight <= z1)
                insideRadius = MathF.Max(insideRadius, segment.RadiusAt(axialHeight));

            minDistanceSquared = MathF.Min(minDistanceSquared,
                segment.OuterWallDistanceSquared(radius, axialHeight));
        }

        // A discontinuous join is closed by an annular shoulder.
        for (int i = 0; i + 1 < _segments.Count; i++)
        {
            SweptVolumeSegment current = _segments[i];
            SweptVolumeSegment next = _segments[i + 1];
            float r0 = MathF.Min(current.TopRadius, next.BottomRadius);
            float r1 = MathF.Max(current.TopRadius, next.BottomRadius);
            if (r1 - r0 <= 0.001f)
                continue;

            float closestRadius = Math.Clamp(radius, r0, r1);
            float dr = radius - closestRadius;
            float dz = axialHeight - (current.AxialOffset + current.Height);
            minDistanceSquared = MathF.Min(minDistanceSquared, dr * dr + dz * dz);
        }

        // Shaft-end cap.
        SweptVolumeSegment last = _segments[^1];
        float topDr = radius - Math.Clamp(radius, 0f, last.TopRadius);
        float topDz = axialHeight - TotalHeight;
        minDistanceSquared = MathF.Min(minDistanceSquared, topDr * topDr + topDz * topDz);

        // Flat tip cap. A hemisphere already closes at its point on the axis.
        SweptVolumeSegment first = _segments[0];
        if (first.Type != SweptVolumeSegmentType.Hemisphere && first.BottomRadius > 0.001f)
        {
            float bottomDr = radius - Math.Clamp(radius, 0f, first.BottomRadius);
            minDistanceSquared = MathF.Min(minDistanceSquared,
                bottomDr * bottomDr + axialHeight * axialHeight);
        }

        bool inside = inAxialRange && radius < insideRadius;
        float distance = MathF.Sqrt(minDistanceSquared);
        return inside ? -distance : distance;
    }

    /// <summary>
    /// Maximum solid radius in an axial interval. Used to avoid binning distant shaft/collar
    /// primitives when only the tip can intersect the finite stock volume.
    /// </summary>
    public float MaxRadiusInRange(float axialMin, float axialMax)
    {
        if (axialMin > axialMax)
            (axialMin, axialMax) = (axialMax, axialMin);

        float result = 0f;
        foreach (SweptVolumeSegment segment in _segments)
        {
            float z0 = MathF.Max(axialMin, segment.AxialOffset);
            float z1 = MathF.Min(axialMax, segment.AxialOffset + segment.Height);
            if (z0 > z1)
                continue;

            result = MathF.Max(result,
                MathF.Max(segment.RadiusAt(z0), segment.RadiusAt(z1)));
        }
        return result;
    }

    public static SweptVolumeStack CreateFlatEndMill(float radius, float cuttingLength)
        => new([SweptVolumeSegment.Cylinder(radius, cuttingLength)]);

    public static SweptVolumeStack CreateBallNose(float radius, float cuttingLength)
        => new([
            SweptVolumeSegment.Hemisphere(radius),
            SweptVolumeSegment.Cylinder(radius, cuttingLength)
        ]);

    public static SweptVolumeStack CreateVBit(float angleDegrees, float maxRadius, float barrelLength = 10f)
    {
        float halfAngle = angleDegrees * (MathF.PI / 360f);
        float height = maxRadius / MathF.Tan(halfAngle);
        return new([
            SweptVolumeSegment.Cone(0f, maxRadius, height),
            SweptVolumeSegment.Cylinder(maxRadius, barrelLength)
        ]);
    }

    public static SweptVolumeStack CreateBullNose(float radius, float cornerRadius, float cuttingLength)
        => new([
            SweptVolumeSegment.Torus(radius - cornerRadius, cornerRadius),
            SweptVolumeSegment.Cylinder(radius, cuttingLength)
        ]);
}
