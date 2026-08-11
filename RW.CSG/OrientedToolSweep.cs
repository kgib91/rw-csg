using System.Numerics;

namespace RW.CSG;

/// <summary>
/// Direction of the tool profile's positive axial coordinate in stock space.
/// </summary>
public enum ToolAxisDirection
{
    PositiveZ,
    NegativeZ,
    PositiveX,
    NegativeX,
    PositiveY,
    NegativeY
}

public static class ToolAxisDirectionExtensions
{
    public static Vector3 ToVector(this ToolAxisDirection direction) => direction switch
    {
        ToolAxisDirection.PositiveZ => Vector3.UnitZ,
        ToolAxisDirection.NegativeZ => -Vector3.UnitZ,
        ToolAxisDirection.PositiveX => Vector3.UnitX,
        ToolAxisDirection.NegativeX => -Vector3.UnitX,
        ToolAxisDirection.PositiveY => Vector3.UnitY,
        ToolAxisDirection.NegativeY => -Vector3.UnitY,
        _ => throw new ArgumentOutOfRangeException(nameof(direction), direction, null)
    };
}

/// <summary>
/// A cutter-tip move in canonical stock coordinates with a fixed axis-aligned tool orientation.
/// </summary>
public readonly record struct OrientedToolSweep(
    Vector3 Start,
    Vector3 End,
    ToolAxisDirection Axis);

/// <summary>
/// Conservative material-intersection classification for a proposed tool sweep.
/// </summary>
public enum RemovalProbeResult
{
    DefinitelyAir,
    IntersectsMaterial,
    Uncertain
}

/// <summary>
/// Falsifiable diagnostics emitted with each nonmutating removal probe.
/// </summary>
public readonly record struct RemovalProbeDiagnostics(
    int SampleCount,
    int MaterialSampleCount,
    int UncertainSampleCount,
    float MinimumStockDistance,
    float MinimumBrushDistance);

/// <summary>
/// Result of checking a tool sweep against the current volume without changing it.
/// </summary>
public readonly record struct RemovalProbe(
    RemovalProbeResult Result,
    RemovalProbeDiagnostics Diagnostics);

/// <summary>
/// Exact removable volume measured without changing the source stock volume.
/// </summary>
public readonly record struct RemovalVolumeProbe(
    bool DefinitelyAir,
    double RemovableVolume);