namespace RW.CSG.Avalonia;

internal static class LaunchOptions
{
    public static string GraphicsBackend { get; set; } = "default";
    public static string? SnapshotPath { get; set; }
    public static int SnapshotWaitFrames { get; set; } = 180;
    public static int DebugView { get; set; }
    public static bool ShaderCacheEnabled { get; set; } = true;
    public static bool RebuildShaderCache { get; set; }

    /// <summary>Non-null => run a GL bench window instead of the app.</summary>
    public static string? DiagnosticsMode { get; set; }
}
