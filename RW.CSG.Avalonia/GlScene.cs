using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Drawing;
using System.Drawing.Imaging;
using System.IO;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Security.Cryptography;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.OpenGL;
using Avalonia.OpenGL.Controls;
using global::RW.CSG;
using Brushes = Avalonia.Media.Brushes;
using DrawingContext = Avalonia.Media.DrawingContext;

namespace RW.CSG.Avalonia;

internal sealed class GlDiagnosticsWindow : Window
{
    public GlDiagnosticsWindow(string mode)
    {
        Title = $"RW.CSG GL bench: {mode}";
        Width = 1920; Height = 1080;
        SystemDecorations = SystemDecorations.None;
        Content = new GlSceneControl(mode);
    }
}

/// <summary>
/// Phase-0 GPU benches on the REAL Avalonia+ANGLE ES3.0 context:
///  upload  (B3): scattered 9³-tile texSubImage3D into a 3D atlas + indirection texel updates.
///  raymarch(B4): full §7 hierarchy fragment shader at 1920×1080 offscreen, glFinish-timed.
///  seek: repeatable viewer stock rebuilds with CPU, GPU upload, and end-to-frame timings.
/// Prints to console and exits.
/// </summary>
internal sealed unsafe class GlSceneControl : OpenGlControlBase
{
    private const float CameraTanHalfY = 0.55f; // Matches shader ray setup (ndc.y in [-0.5, 0.5] scaled by 1.1)
    private const int ViewerVoxelsX = 3200;
    private const int ViewerVoxelsY = 1600;
    private const int ViewerVoxelsZ = 40;
    private const float ViewerVoxelSize = 0.25f;
    private const int SeekStampBatchSize = 131_072;
    private const int InlineForwardSeekLimit = 2_048;
    private const int InlineBulkDirtyThreshold = 32_768;
    private const int SeekDebounceMilliseconds = 120;
    private const int ViewerAtlasTilesPerAxis = 56;
    public static IReadOnlyList<string> DebugViewNames { get; } =
    [
        "Shaded",
        "Hit classification",
        "March cost",
        "Brick topology",
        "Voxel phase",
        "SDF residual",
        "Surface normals"
    ];

    private static readonly string[] DebugViewLegends =
    [
        "Shaded material preview",
        "Blue solid | Green filtered | Orange early-raw | Yellow late-raw | White reversed/thin | Magenta tangent | Red miss | Cyan degenerate",
        "Blue low cost → cyan → green → yellow → red high cost; misses are dim",
        "Stable pseudo-color per shifted 8³ brick; white marks atlas brick planes",
        "RGB = local XYZ phase inside each voxel; white marks voxel planes",
        "Raw field at reconstructed hit: blue inside | white zero | red outside",
        "RGB encodes the XYZ normal from [-1, +1]"
    ];

    private readonly string _mode;
    private bool IsSeekBench => string.Equals(_mode, "seek", StringComparison.OrdinalIgnoreCase);
    private bool IsViewer => string.Equals(_mode, "viewer", StringComparison.OrdinalIgnoreCase) || IsSeekBench;
    private bool IsSeam => string.Equals(_mode, "seam", StringComparison.OrdinalIgnoreCase);
    private bool IsLive => string.Equals(_mode, "live", StringComparison.OrdinalIgnoreCase);
    public GlSceneControl(string mode)
    {
        _mode = mode;
        DebugView = Math.Clamp(LaunchOptions.DebugView, 0, DebugViewNames.Count - 1);
        Focusable = true;
    }

    private bool _isPaused;
    public bool IsPaused => Volatile.Read(ref _isPaused);
    public int ToolpathSegmentCount => Volatile.Read(ref _liveSegmentCount);
    public int ToolpathStep
    {
        get
        {
            int requested = Volatile.Read(ref _requestedSeekCursor);
            if (requested >= 0)
            {
                return requested;
            }
            return Math.Clamp(Volatile.Read(ref _liveCursor), 0, ToolpathSegmentCount);
        }
    }
    public bool IsSeeking => Volatile.Read(ref _requestedSeekCursor) >= 0
        || Volatile.Read(ref _seekBuildActive) != 0;
    public int SeekProgressPermille => Volatile.Read(ref _seekProgressPermille);
    public bool IsSeekProgressIndeterminate => Volatile.Read(ref _seekProgressIndeterminate) != 0;
    public string SeekProgressStage => Volatile.Read(ref _seekProgressStage);
    public int DebugView { get; private set; }
    public string DebugLegend => DebugViewLegends[DebugView];

    public void TogglePause() => Volatile.Write(ref _isPaused, !Volatile.Read(ref _isPaused));
    public void SeekToStep(int step)
    {
        int count = ToolpathSegmentCount;
        if (count <= 0)
        {
            return;
        }

        int target = Math.Clamp(step, 0, count);
        if (Volatile.Read(ref _requestedSeekCursor) == target)
        {
            return;
        }

        int generation = Interlocked.Increment(ref _seekRequestGeneration);
        Volatile.Write(ref _isPaused, true);
        Volatile.Write(ref _seekRequestedAt, Stopwatch.GetTimestamp());
        Interlocked.Exchange(ref _requestedSeekCursor, target);
        ReportSeekProgress(generation, 0, "Waiting for seek…", indeterminate: true);
    }

    private void ReportSeekProgress(int generation, int permille, string stage,
        bool indeterminate = false)
    {
        if (generation != Volatile.Read(ref _seekRequestGeneration))
        {
            return;
        }

        Volatile.Write(ref _seekProgressStage, stage);
        Volatile.Write(ref _seekProgressIndeterminate, indeterminate ? 1 : 0);
        Volatile.Write(ref _seekProgressPermille, Math.Clamp(permille, 0, 1000));
    }
    public void SetDebugView(int view) => DebugView = Math.Clamp(view, 0, DebugViewNames.Count - 1);
    public void CycleDebugView() => SetDebugView((DebugView + 1) % DebugViewNames.Count);
    public string GetHudText()
    {
        if (!Volatile.Read(ref _ready))
        {
            return StartupTrace.HudText;
        }

        SparseSdfVolume? volume = _vol;
        string seek = IsSeeking ? " | seeking" : string.Empty;
        string lastSeek = _viewerLastSeekMs > 0 ? $" | last seek {_viewerLastSeekMs / 1000.0:F2}s" : string.Empty;
        return $"stock {volume?.StockSizeX ?? 0:F0}×{volume?.StockSizeY ?? 0:F0}×{volume?.StockHeight ?? 0:F0} mm"
            + $" | fps {(_viewerLastFrameMs > 0 ? 1000.0 / _viewerLastFrameMs : 0):F1}"
            + $" | frame {_viewerLastFrameMs:F2} ms | st/s {_viewerLastStPerSec:F0}"
            + $" | step {ToolpathStep}/{ToolpathSegmentCount}{seek}{lastSeek} | up {_viewerLastUploads}"
            + $" | bricks {volume?.AllocatedBricks ?? 0}"
            + $" | tool {_liveTool?.ProfileStack?.Segments.Count ?? 0} seg / {_liveTool?.Zmax ?? 0:F0} mm"
            + $" | {DebugViewNames[DebugView]}"
            + $" | {StartupTrace.HudText}";
    }

    private static SparseSdfVolume CreateViewerStockVolume()
        => new(ViewerVoxelsX, ViewerVoxelsY, ViewerVoxelsZ, ViewerVoxelSize, 8, 3);

    private static ToolProfile CreateSceneToolProfile()
    {
        // Attached Profile Editor stack, stored in the same tip -> shaft order as legacy:
        // cone Ø2 -> Ø10 x 15, collar cylinder Ø15 x 8, shaft cylinder Ø6.35 x 80 mm.
        // SweptVolumeSegment takes radii, while Profile Editor dimensions are diameters.
        var stack = new SweptVolumeStack([
            SweptVolumeSegment.Cone(1f, 5f, 15f),
            SweptVolumeSegment.Cylinder(7.5f, 8f),
            SweptVolumeSegment.Cylinder(3.175f, 80f)
        ]);
        return new ToolProfile(stack, radialLutSize: 256, axialLutSize: 1024, pad: 2f);
    }

    // ---- GL bindings (via eglGetProcAddress; ANGLE returns core entry points) ----
    private delegate* unmanaged[Stdcall]<int, int, int, int, int, int, int, int, int, void*, void> _texImage3D;
    private delegate* unmanaged[Stdcall]<int, int, int, int, int, int, int, int, int, int, void*, void> _texSubImage3D;
    private delegate* unmanaged[Stdcall]<int, uint*, void> _genTextures;
    private delegate* unmanaged[Stdcall]<int, int, void> _bindTexture;         // target, tex
    private delegate* unmanaged[Stdcall]<int, int, int, void> _texParameteri;
    private delegate* unmanaged[Stdcall]<int, int, void> _pixelStorei;
    private delegate* unmanaged[Stdcall]<int, void> _activeTexture;
    private delegate* unmanaged[Stdcall]<int, int, int, int, int, int, int, int, void*, void> _texImage2D;
    private delegate* unmanaged[Stdcall]<int, uint*, void> _genFramebuffers;
    private delegate* unmanaged[Stdcall]<int, int, void> _bindFramebuffer;
    private delegate* unmanaged[Stdcall]<int, int, int, int, int, void> _framebufferTexture2D;
    private delegate* unmanaged[Stdcall]<int, int> _checkFramebufferStatus;
    private delegate* unmanaged[Stdcall]<int, int, int, int, void> _viewport;
    private delegate* unmanaged[Stdcall]<float, float, float, float, void> _clearColor;
    private delegate* unmanaged[Stdcall]<int, void> _clear;
    private delegate* unmanaged[Stdcall]<int, int> _createShader;
    private delegate* unmanaged[Stdcall]<int, int, byte**, int*, void> _shaderSource;
    private delegate* unmanaged[Stdcall]<int, void> _compileShader;
    private delegate* unmanaged[Stdcall]<int, int, int*, void> _getShaderiv;
    private delegate* unmanaged[Stdcall]<int, int, int*, byte*, void> _getShaderInfoLog;
    private delegate* unmanaged[Stdcall]<int> _createProgram;
    private delegate* unmanaged[Stdcall]<int, int, void> _attachShader;
    private delegate* unmanaged[Stdcall]<int, void> _linkProgram;
    private delegate* unmanaged[Stdcall]<int, int, int*, void> _getProgramiv;
    private delegate* unmanaged[Stdcall]<int, int, int*, byte*, void> _getProgramInfoLog;
    private delegate* unmanaged[Stdcall]<int, int, int, void> _programParameteri;
    private delegate* unmanaged[Stdcall]<int, int, int*, int*, void*, void> _getProgramBinary;
    private delegate* unmanaged[Stdcall]<int, int, void*, int, void> _programBinary;
    private delegate* unmanaged[Stdcall]<int, void> _deleteShader;
    private delegate* unmanaged[Stdcall]<int, void> _deleteProgram;
    private delegate* unmanaged[Stdcall]<int, void> _useProgram;
    private delegate* unmanaged[Stdcall]<int, byte*, int> _getUniformLocation;
    private delegate* unmanaged[Stdcall]<int, float, void> _uniform1f;
    private delegate* unmanaged[Stdcall]<int, float, float, float, void> _uniform3f;
    private delegate* unmanaged[Stdcall]<int, float, float, float, float, void> _uniform4f;
    private delegate* unmanaged[Stdcall]<int, int, void> _uniform1i;
    private delegate* unmanaged[Stdcall]<int, int, int, void> _drawArrays;
    private delegate* unmanaged[Stdcall]<void> _finish;
    private delegate* unmanaged[Stdcall]<int, int, int, int, int, int, void*, void> _readPixels;
    private delegate* unmanaged[Stdcall]<int> _getError;
    private delegate* unmanaged[Stdcall]<int, byte*> _getString;
    private delegate* unmanaged[Stdcall]<int, uint*, void> _genVertexArrays;
    private delegate* unmanaged[Stdcall]<int, void> _bindVertexArray;
    private delegate* unmanaged[Stdcall]<int, int*, void> _getIntegerv;
    private delegate* unmanaged[Stdcall]<int, void> _disable;
    private delegate* unmanaged[Stdcall]<byte, byte, byte, byte, void> _colorMask;
    private delegate* unmanaged[Stdcall]<int, int, void> _bindBuffer;
    private delegate* unmanaged[Stdcall]<int, int, float*, void> _getUniformfv;
    private delegate* unmanaged[Stdcall]<int, void> _readBuffer;
    private delegate* unmanaged[Stdcall]<int, uint*, void> _genBuffers;
    private delegate* unmanaged[Stdcall]<int, nint, void*, int, void> _bufferData;
    private delegate* unmanaged[Stdcall]<int, void> _enableVertexAttribArray;
    private delegate* unmanaged[Stdcall]<int, int, int, byte, int, void*, void> _vertexAttribPointer;
    private delegate* unmanaged[Stdcall]<int, uint*, void> _genRenderbuffers;
    private delegate* unmanaged[Stdcall]<int, int, void> _bindRenderbuffer;
    private delegate* unmanaged[Stdcall]<int, int, int, int, void> _renderbufferStorage;
    private delegate* unmanaged[Stdcall]<int, int, int, int, void> _framebufferRenderbuffer;
    private delegate* unmanaged[Stdcall]<int, int, int, int, int, int, int, int, int, int, void> _blitFramebuffer;
    private delegate* unmanaged[Stdcall]<int, uint*, void> _deleteTextures;

    // GL enums
    private const int TEXTURE_3D = 0x806F, TEXTURE_2D = 0x0DE1;
    private const int R16F = 0x822D, R8 = 0x8229, RED = 0x1903, RGBA8 = 0x8058, RGBA = 0x1908;
    private const int HALF_FLOAT = 0x140B, UNSIGNED_BYTE = 0x1401;
    private const int MIN_FILTER = 0x2801, MAG_FILTER = 0x2800, LINEAR = 0x2601, NEAREST = 0x2600;
    private const int WRAP_S = 0x2802, WRAP_T = 0x2803, WRAP_R = 0x8072, CLAMP = 0x812F;
    private const int UNPACK_ALIGNMENT = 0x0CF5, TEXTURE0 = 0x84C0;
    private const int FRAMEBUFFER = 0x8D40, COLOR_ATTACHMENT0 = 0x8CE4, FB_COMPLETE = 0x8CD5;
    private const int COLOR_BUFFER_BIT = 0x4000, TRIANGLES = 0x0004;
    private const int FRAGMENT_SHADER = 0x8B30, VERTEX_SHADER = 0x8B31, COMPILE_STATUS = 0x8B81, LINK_STATUS = 0x8B82;
    private const int VENDOR = 0x1F00, RENDERER = 0x1F01, VERSION = 0x1F02,
        SHADING_LANGUAGE_VERSION = 0x8B8C;
    private const int PROGRAM_BINARY_LENGTH = 0x8741, PROGRAM_BINARY_RETRIEVABLE_HINT = 0x8257;
    private const int ShaderCacheMagic = 0x47534352; // "RCSG" in little-endian storage
    private const int ShaderCacheVersion = 1;
    private const int MaxCachedProgramBytes = 64 * 1024 * 1024;

    // scene / bench state
    private const int W = 1920, H = 1080;
    private bool _ready, _failed;
    private int _frame;
    private SparseSdfVolume? _vol;
    private int _atlas16, _atlas8, _indir, _coarseTex, _prog, _vao;
    private int _tilesPerAxis, _atlasTexels;
    private ushort* _staging;             // native tile staging
    private byte* _stagingB;              // R8 variant
    private (int x, int y, int z)[] _scatter = [];
    private readonly Stopwatch _sw = new();
    private readonly System.Collections.Generic.List<double> _times = [];
    private int _phase;
    private Vector3 _eye, _fwd, _right, _up;
    private int _uEye, _uJit;
    private const int MaxToolSegments = 8;
    private int _uShowTool, _uToolPos, _uToolSegmentCount, _uToolHeight, _uToolMaxR, _uDebugView;
    private readonly int[] _uToolSegments = new int[MaxToolSegments];
    private readonly int[] _uToolSegmentTypes = new int[MaxToolSegments];
    private int[] _brickToTile = [];
    private readonly Stack<int> _freeAtlasTiles = new();
    private bool _seamUpdated;
    private Vector3 _seamP0, _seamP1;
    private PathSegment[] _liveSegs = [];
    private int _liveCursor;
    private int _liveSegmentCount;
    private int _requestedSeekCursor = -1;
    private long _seekRequestedAt;
    private int _seekRequestGeneration;
    private int _seekProgressPermille;
    private int _seekProgressIndeterminate;
    private string _seekProgressStage = string.Empty;
    private int _seekBuildActive;
    private int _seekBuildTarget = -1;
    private CancellationTokenSource? _seekBuildCancellation;
    private Task<ViewerSeekResult>? _seekBuildTask;
    private DirtyBrickSet? _liveDirty;
    private int[] _liveDrain = [];
    private ToolProfile? _liveTool;
    private readonly List<double> _liveFrameMs = [];
    private long _liveStamped;
    private int _uSeamMode;
    private float _displayStockZ;
    private int _nextTile;
    private byte[] _indirCpu = [];
    private int _atlasGrowthCount;
    private double _atlasGrowthMs;
    private double _tileUploadSubmitMs;
    private double _tileUploadFinishMs;
    private double _tileGatherMs;
    private double _atlasSubImageMs;
    private double _indirSubImageMs;
    private long _mixedTileUploads;
    private long _newTileUploads;
    private readonly string? _snapshotPath = LaunchOptions.SnapshotPath;
    private readonly int _snapshotWaitFrames = LaunchOptions.SnapshotWaitFrames;
    private bool _snapshotCaptured;
    private int _lastRenderWidth;
    private int _lastRenderHeight;
    private Vector3 _toolPos;
    private double _viewerLastFrameMs;
    private double _viewerLastStPerSec;
    private int _viewerLastUploads;
    private double _viewerLastSeekMs;
    private ViewerSeekMeasurement? _pendingSeekMeasurement;
    private readonly List<ViewerSeekMeasurement> _seekBenchMeasurements = [];
    private int _seekBenchPhase;
    private Vector3 _camTarget;
    private float _camYaw;
    private float _camPitch;
    private float _camDistance;
    private bool _camNeedsFit = true;
    private bool _dragRotate;
    private bool _dragPan;
    private global::Avalonia.Point _lastPointer;
    private string _glVendor = "unknown";
    private string _glRenderer = "unknown";
    private string _glVersion = "unknown";
    private string _glslVersion = "unknown";

    private sealed record ViewerSeekResult(
        int Generation,
        int From,
        int Target,
        SparseSdfVolume Volume,
        ViewerGpuSnapshot GpuSnapshot,
        long RequestedAt,
        long ScheduledAt,
        long WorkerStartedAt,
        double VolumeInitMs,
        double StampMs,
        double TotalMs);

    private sealed class ViewerGpuSnapshot : IDisposable
    {
        private nint _atlas;

        public ViewerGpuSnapshot(int[] brickToTile, byte[] indirection, ushort[] coarse,
            nint atlas, long atlasBytes, int atlasDepthTexels, int mixedTiles,
            int tilesPerAxis, int atlasTexels, double stateSetupMs, double gatherMs)
        {
            BrickToTile = brickToTile;
            Indirection = indirection;
            Coarse = coarse;
            _atlas = atlas;
            AtlasBytes = atlasBytes;
            AtlasDepthTexels = atlasDepthTexels;
            MixedTiles = mixedTiles;
            TilesPerAxis = tilesPerAxis;
            AtlasTexels = atlasTexels;
            StateSetupMs = stateSetupMs;
            GatherMs = gatherMs;
        }

        public int[] BrickToTile { get; }
        public byte[] Indirection { get; }
        public ushort[] Coarse { get; }
        public ushort* Atlas => (ushort*)_atlas;
        public long AtlasBytes { get; }
        public int AtlasDepthTexels { get; }
        public int MixedTiles { get; }
        public int TilesPerAxis { get; }
        public int AtlasTexels { get; }
        public double StateSetupMs { get; }
        public double GatherMs { get; }

        public void Dispose()
        {
            nint atlas = Interlocked.Exchange(ref _atlas, 0);
            if (atlas != 0)
            {
                NativeMemory.Free((void*)atlas);
            }
        }
    }

    private readonly record struct ViewerGpuRebuildMetrics(
        int Uploaded,
        long MixedTiles,
        long NewTiles,
        double PriorFrameDrainMs,
        double StateSetupMs,
        double MixedUploadMs,
        double UploadSubmitMs,
        double UploadFinishMs,
        double GatherMs,
        double AtlasSubImageMs,
        double IndirectionSubImageMs,
        double TotalMs,
        int AtlasGrowths,
        double AtlasGrowthMs);

    private sealed class ViewerSeekMeasurement
    {
        public string Kind { get; init; } = string.Empty;
        public int From { get; init; }
        public int Target { get; init; }
        public int Uploaded { get; init; }
        public long MixedTiles { get; init; }
        public long NewTiles { get; init; }
        public long RequestedAt { get; init; }
        public long AppliedAt { get; init; }
        public double DispatchWaitMs { get; init; }
        public double WorkerQueueMs { get; init; }
        public double VolumeInitMs { get; init; }
        public double StampMs { get; init; }
        public double CpuTotalMs { get; init; }
        public double PriorFrameDrainMs { get; init; }
        public double GpuStateMs { get; init; }
        public double GpuUploadMs { get; init; }
        public double UploadSubmitMs { get; init; }
        public double UploadFinishMs { get; init; }
        public double GatherMs { get; init; }
        public double AtlasSubImageMs { get; init; }
        public double IndirectionSubImageMs { get; init; }
        public double GpuTotalMs { get; init; }
        public int AtlasGrowths { get; init; }
        public double AtlasGrowthMs { get; init; }
        public double RequestToAppliedMs { get; init; }
        public double ApplyToFrameMs { get; set; }
        public double RequestToFrameMs { get; set; }
        public double UpdatedFrameFinishMs { get; set; }
        public double FrameMs { get; set; }
    }

    /// <summary>OpenGlControlBase presents through a composition child visual, which has no
    /// scene-graph draw ops and is therefore NOT hit-testable — pointer events fall through to
    /// the window and none of the OnPointer* overrides ever fire. Filling a transparent rect
    /// gives the control real render content so it receives pointer input.</summary>
    public override void Render(DrawingContext context)
    {
        context.FillRectangle(Brushes.Transparent, new Rect(Bounds.Size));
        base.Render(context);
    }

    public void ViewerPointerPressed(PointerPressedEventArgs e)
    {
        if (!IsViewer || e.Handled)
        {
            return;
        }

        Focus();
        _lastPointer = e.GetPosition(this);
        var p = e.GetCurrentPoint(this);
        var kind = p.Properties.PointerUpdateKind;
        bool leftPress = kind == PointerUpdateKind.LeftButtonPressed || p.Properties.IsLeftButtonPressed;
        bool rightPress = kind == PointerUpdateKind.RightButtonPressed || p.Properties.IsRightButtonPressed;
        bool middlePress = kind == PointerUpdateKind.MiddleButtonPressed || p.Properties.IsMiddleButtonPressed;
        bool shiftLeft = leftPress && e.KeyModifiers.HasFlag(KeyModifiers.Shift);
        _dragPan = rightPress || middlePress || shiftLeft;
        _dragRotate = leftPress && !_dragPan;
        if (!_dragRotate && !_dragPan && p.Pointer.Type == PointerType.Touch)
        {
            _dragRotate = true;
        }
        e.Pointer.Capture(this);
        e.Handled = _dragPan || _dragRotate;
    }

    protected override void OnPointerPressed(PointerPressedEventArgs e)
    {
        ViewerPointerPressed(e);
        base.OnPointerPressed(e);
    }

    public void ViewerPointerMoved(PointerEventArgs e)
    {
        if (!IsViewer || e.Handled)
        {
            return;
        }

        var pt = e.GetCurrentPoint(this);
        if (!pt.Properties.IsLeftButtonPressed && !pt.Properties.IsRightButtonPressed && !pt.Properties.IsMiddleButtonPressed
            && pt.Pointer.Type != PointerType.Touch)
        {
            _dragRotate = false;
            _dragPan = false;
            return;
        }

        if (!_dragRotate && !_dragPan)
        {
            bool shiftLeft = pt.Properties.IsLeftButtonPressed && e.KeyModifiers.HasFlag(KeyModifiers.Shift);
            _dragPan = pt.Properties.IsRightButtonPressed || pt.Properties.IsMiddleButtonPressed || shiftLeft;
            _dragRotate = pt.Properties.IsLeftButtonPressed && !_dragPan;
            if (!_dragRotate && !_dragPan && pt.Pointer.Type == PointerType.Touch)
            {
                _dragRotate = true;
            }
            _lastPointer = e.GetPosition(this);   // avoid a jump when drag activates mid-move
        }

        if (!_dragRotate && !_dragPan)
        {
            return;
        }

        var now = e.GetPosition(this);
        var dx = (float)(now.X - _lastPointer.X);
        var dy = (float)(now.Y - _lastPointer.Y);
        _lastPointer = now;

        if (_dragRotate)
        {
            _camYaw -= dx * 0.010f;
            _camPitch = Math.Clamp(_camPitch - dy * 0.010f, -1.45f, 1.45f);
        }

        if (_dragPan)
        {
            float h = MathF.Max(1f, (float)Bounds.Height);
            float worldPerPixel = 2f * _camDistance * CameraTanHalfY / h;
            _camTarget += (-_right * dx + _up * dy) * worldPerPixel;
        }

        e.Handled = true;
    }

    protected override void OnPointerMoved(PointerEventArgs e)
    {
        ViewerPointerMoved(e);
        base.OnPointerMoved(e);
    }

    public void ViewerPointerReleased(PointerReleasedEventArgs e)
    {
        if (!IsViewer)
        {
            return;
        }

        _dragRotate = false;
        _dragPan = false;
        e.Pointer.Capture(null);
        e.Handled = true;
    }

    protected override void OnPointerReleased(PointerReleasedEventArgs e)
    {
        ViewerPointerReleased(e);
        base.OnPointerReleased(e);
    }

    protected override void OnPointerCaptureLost(PointerCaptureLostEventArgs e)
    {
        _dragRotate = false;
        _dragPan = false;
        base.OnPointerCaptureLost(e);
    }

    public void ViewerPointerWheel(PointerWheelEventArgs e)
    {
        if (!IsViewer || e.Handled)
        {
            return;
        }

        float delta = (float)e.Delta.Y;
        float zoom = MathF.Exp(-delta * 0.30f);
        float stock = _vol is null ? 512f * 0.3f : MathF.Max(_vol.StockSizeX, _vol.StockSizeY);
        float minDist = MathF.Max((_vol?.VoxelSize ?? 0.3f) * 2f, 0.2f);
        float maxDist = stock * 20f;
        _camDistance = Math.Clamp(_camDistance * zoom, minDist, maxDist);
        _camNeedsFit = false;
        e.Handled = true;
    }

    protected override void OnPointerWheelChanged(PointerWheelEventArgs e)
    {
        ViewerPointerWheel(e);
        base.OnPointerWheelChanged(e);
    }

    private void FitViewerCameraToScreen(int w, int h)
    {
        if (_vol is null)
        {
            return;
        }

        float stockX = _vol.StockSizeX;
        float stockY = _vol.StockSizeY;
        float toolRadius = _liveTool?.Rmax ?? 0f;
        float toolHeight = _liveTool?.Zmax ?? 0f;
        Vector3 sceneMin = new(-toolRadius, -toolRadius, 0f);
        Vector3 sceneMax = new(stockX + toolRadius, stockY + toolRadius,
            _displayStockZ + 0.5f + toolHeight);
        _camTarget = 0.5f * (sceneMin + sceneMax);
        _camYaw = 0.82f;
        _camPitch = -0.58f;

        float aspect = w / (float)Math.Max(1, h);
        float tanHalfX = CameraTanHalfY * aspect;
        float tanHalfMin = MathF.Min(CameraTanHalfY, tanHalfX);
        float halfFovMin = MathF.Atan(tanHalfMin);
        float radius = 0.5f * (sceneMax - sceneMin).Length();
        _camDistance = radius / MathF.Sin(halfFovMin) * 1.15f;
        _camNeedsFit = false;
    }

    private void UpdateViewerCamera(float aspect)
    {
        float cp = MathF.Cos(_camPitch);
        Vector3 fwd = new(cp * MathF.Cos(_camYaw), cp * MathF.Sin(_camYaw), MathF.Sin(_camPitch));
        if (fwd.LengthSquared() < 1e-8f)
        {
            fwd = new Vector3(0, 1, -0.25f);
        }
        _fwd = Vector3.Normalize(fwd);

        Vector3 right = Vector3.Cross(_fwd, Vector3.UnitZ);
        if (right.LengthSquared() < 1e-8f)
        {
            right = Vector3.UnitX;
        }
        _right = Vector3.Normalize(right);
        _up = Vector3.Normalize(Vector3.Cross(_right, _fwd));
        _eye = _camTarget - _fwd * _camDistance;
    }

    private IntPtr Ptr(GlInterface gl, string name)
    {
        var p = gl.GetProcAddress(name);
        if (p == IntPtr.Zero) { Console.WriteLine($"MISSING GL ENTRY POINT: {name}"); _failed = true; }
        return p;
    }

    protected override void OnOpenGlInit(GlInterface gl)
    {
        using var timing = StartupTrace.Measure("OpenGL scene initialization");
        try { InitCore(gl); }
        catch (Exception ex)
        {
            StartupTrace.Record($"FAILED OpenGL initialization: {ex.GetType().Name}: {ex.Message}");
            Console.WriteLine($"GL BENCH INIT FAILED: {ex}");
            Environment.Exit(1);
        }
    }

    private void InitCore(GlInterface gl)
    {
        var stageTimer = Stopwatch.StartNew();
        _texImage3D = (delegate* unmanaged[Stdcall]<int, int, int, int, int, int, int, int, int, void*, void>)Ptr(gl, "glTexImage3D");
        _texSubImage3D = (delegate* unmanaged[Stdcall]<int, int, int, int, int, int, int, int, int, int, void*, void>)Ptr(gl, "glTexSubImage3D");
        _genTextures = (delegate* unmanaged[Stdcall]<int, uint*, void>)Ptr(gl, "glGenTextures");
        _bindTexture = (delegate* unmanaged[Stdcall]<int, int, void>)Ptr(gl, "glBindTexture");
        _texParameteri = (delegate* unmanaged[Stdcall]<int, int, int, void>)Ptr(gl, "glTexParameteri");
        _pixelStorei = (delegate* unmanaged[Stdcall]<int, int, void>)Ptr(gl, "glPixelStorei");
        _activeTexture = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glActiveTexture");
        _texImage2D = (delegate* unmanaged[Stdcall]<int, int, int, int, int, int, int, int, void*, void>)Ptr(gl, "glTexImage2D");
        _genFramebuffers = (delegate* unmanaged[Stdcall]<int, uint*, void>)Ptr(gl, "glGenFramebuffers");
        _bindFramebuffer = (delegate* unmanaged[Stdcall]<int, int, void>)Ptr(gl, "glBindFramebuffer");
        _framebufferTexture2D = (delegate* unmanaged[Stdcall]<int, int, int, int, int, void>)Ptr(gl, "glFramebufferTexture2D");
        _checkFramebufferStatus = (delegate* unmanaged[Stdcall]<int, int>)Ptr(gl, "glCheckFramebufferStatus");
        _viewport = (delegate* unmanaged[Stdcall]<int, int, int, int, void>)Ptr(gl, "glViewport");
        _clearColor = (delegate* unmanaged[Stdcall]<float, float, float, float, void>)Ptr(gl, "glClearColor");
        _clear = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glClear");
        _createShader = (delegate* unmanaged[Stdcall]<int, int>)Ptr(gl, "glCreateShader");
        _shaderSource = (delegate* unmanaged[Stdcall]<int, int, byte**, int*, void>)Ptr(gl, "glShaderSource");
        _compileShader = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glCompileShader");
        _getShaderiv = (delegate* unmanaged[Stdcall]<int, int, int*, void>)Ptr(gl, "glGetShaderiv");
        _getShaderInfoLog = (delegate* unmanaged[Stdcall]<int, int, int*, byte*, void>)Ptr(gl, "glGetShaderInfoLog");
        _createProgram = (delegate* unmanaged[Stdcall]<int>)Ptr(gl, "glCreateProgram");
        _attachShader = (delegate* unmanaged[Stdcall]<int, int, void>)Ptr(gl, "glAttachShader");
        _linkProgram = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glLinkProgram");
        _getProgramiv = (delegate* unmanaged[Stdcall]<int, int, int*, void>)Ptr(gl, "glGetProgramiv");
        _getProgramInfoLog = (delegate* unmanaged[Stdcall]<int, int, int*, byte*, void>)Ptr(gl, "glGetProgramInfoLog");
        _programParameteri = (delegate* unmanaged[Stdcall]<int, int, int, void>)gl.GetProcAddress("glProgramParameteri");
        _getProgramBinary = (delegate* unmanaged[Stdcall]<int, int, int*, int*, void*, void>)gl.GetProcAddress("glGetProgramBinary");
        _programBinary = (delegate* unmanaged[Stdcall]<int, int, void*, int, void>)gl.GetProcAddress("glProgramBinary");
        _deleteShader = (delegate* unmanaged[Stdcall]<int, void>)gl.GetProcAddress("glDeleteShader");
        _deleteProgram = (delegate* unmanaged[Stdcall]<int, void>)gl.GetProcAddress("glDeleteProgram");
        _useProgram = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glUseProgram");
        _getUniformLocation = (delegate* unmanaged[Stdcall]<int, byte*, int>)Ptr(gl, "glGetUniformLocation");
        _uniform1f = (delegate* unmanaged[Stdcall]<int, float, void>)Ptr(gl, "glUniform1f");
        _uniform3f = (delegate* unmanaged[Stdcall]<int, float, float, float, void>)Ptr(gl, "glUniform3f");
        _uniform4f = (delegate* unmanaged[Stdcall]<int, float, float, float, float, void>)Ptr(gl, "glUniform4f");
        _uniform1i = (delegate* unmanaged[Stdcall]<int, int, void>)Ptr(gl, "glUniform1i");
        _drawArrays = (delegate* unmanaged[Stdcall]<int, int, int, void>)Ptr(gl, "glDrawArrays");
        _finish = (delegate* unmanaged[Stdcall]<void>)Ptr(gl, "glFinish");
        _readPixels = (delegate* unmanaged[Stdcall]<int, int, int, int, int, int, void*, void>)Ptr(gl, "glReadPixels");
        _getError = (delegate* unmanaged[Stdcall]<int>)Ptr(gl, "glGetError");
        _getString = (delegate* unmanaged[Stdcall]<int, byte*>)Ptr(gl, "glGetString");
        _genVertexArrays = (delegate* unmanaged[Stdcall]<int, uint*, void>)Ptr(gl, "glGenVertexArrays");
        _bindVertexArray = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glBindVertexArray");
        _getIntegerv = (delegate* unmanaged[Stdcall]<int, int*, void>)Ptr(gl, "glGetIntegerv");
        _disable = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glDisable");
        _colorMask = (delegate* unmanaged[Stdcall]<byte, byte, byte, byte, void>)Ptr(gl, "glColorMask");
        _bindBuffer = (delegate* unmanaged[Stdcall]<int, int, void>)Ptr(gl, "glBindBuffer");
        _getUniformfv = (delegate* unmanaged[Stdcall]<int, int, float*, void>)Ptr(gl, "glGetUniformfv");
        _readBuffer = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glReadBuffer");
        _genBuffers = (delegate* unmanaged[Stdcall]<int, uint*, void>)Ptr(gl, "glGenBuffers");
        _bufferData = (delegate* unmanaged[Stdcall]<int, nint, void*, int, void>)Ptr(gl, "glBufferData");
        _enableVertexAttribArray = (delegate* unmanaged[Stdcall]<int, void>)Ptr(gl, "glEnableVertexAttribArray");
        _vertexAttribPointer = (delegate* unmanaged[Stdcall]<int, int, int, byte, int, void*, void>)Ptr(gl, "glVertexAttribPointer");
        _genRenderbuffers = (delegate* unmanaged[Stdcall]<int, uint*, void>)Ptr(gl, "glGenRenderbuffers");
        _bindRenderbuffer = (delegate* unmanaged[Stdcall]<int, int, void>)Ptr(gl, "glBindRenderbuffer");
        _renderbufferStorage = (delegate* unmanaged[Stdcall]<int, int, int, int, void>)Ptr(gl, "glRenderbufferStorage");
        _framebufferRenderbuffer = (delegate* unmanaged[Stdcall]<int, int, int, int, void>)Ptr(gl, "glFramebufferRenderbuffer");
        _blitFramebuffer = (delegate* unmanaged[Stdcall]<int, int, int, int, int, int, int, int, int, int, void>)Ptr(gl, "glBlitFramebuffer");
        _deleteTextures = (delegate* unmanaged[Stdcall]<int, uint*, void>)Ptr(gl, "glDeleteTextures");
        if (_failed) { Environment.Exit(1); }
        StartupTrace.Record($"OpenGL entry points resolved ({stageTimer.Elapsed.TotalMilliseconds:F1} ms)");

        _glVendor = GetGlString(VENDOR);
        _glRenderer = GetGlString(RENDERER);
        _glVersion = GetGlString(VERSION);
        _glslVersion = GetGlString(SHADING_LANGUAGE_VERSION);
        Console.WriteLine($"# glbench '{_mode}' | renderer: {_glRenderer}");
        Console.WriteLine($"#   version: {_glVersion}");

        // ---- CPU scene source ----
        // Keep benches on their measured 512³ contract. The viewer uses an exact rectangular
        // 800 × 400 × 10 mm sparse grid at 0.25 mm/voxel.
        stageTimer.Restart();
        var vol = IsViewer
            ? CreateViewerStockVolume()
            : new SparseSdfVolume(512, 0.3f, 8, 3);
        var tool = ToolProfile.FlatEndmill(3, 12, 128);
        var swc = Stopwatch.StartNew();
        if (IsViewer)
        {
            // Viewer starts from untouched stock so the egg-carton surface is revealed over time.
            _displayStockZ = vol.StockHeight;
        }
        else
        {
            _displayStockZ = vol.StockHeight;
            var pts = HilbertPath.Generate(9, vol.StockSizeX, 8, vol.StockSizeX - 1.5f, 0.5f, 0.02f);
            var segs = HilbertPath.ToSegments(pts, 0, pts.Length - 1);
            for (int s = 0; s < segs.Length; s += 4096)
                vol.StampBatch(segs, s, Math.Min(4096, segs.Length - s), tool, parallel: true);
        }
        _vol = vol;
        StartupTrace.Record($"CPU volume prepared ({stageTimer.Elapsed.TotalMilliseconds:F1} ms)");

        // mixed bricks → tiles
        stageTimer.Restart();
        var mixed = new System.Collections.Generic.List<int>();
        int bricksX = vol.BricksX, bricksY = vol.BricksY, bricksZ = vol.BricksZ;
        for (int b = 0; b < vol.BrickCount; b++) if (vol.BrickState(b) >= 0) mixed.Add(b);
        _brickToTile = new int[vol.BrickCount];
        Array.Fill(_brickToTile, -1);
        int tileFloor = IsViewer
            ? ViewerAtlasTilesPerAxis * ViewerAtlasTilesPerAxis * ViewerAtlasTilesPerAxis
            : IsLive ? 32_768 : 4096;
        _tilesPerAxis = (int)Math.Ceiling(Math.Cbrt(Math.Max(mixed.Count, tileFloor)));
        _atlasTexels = _tilesPerAxis * 9;
        Console.WriteLine($"# carve {swc.ElapsedMilliseconds} ms | mixed bricks {mixed.Count} | atlas {_atlasTexels}³ R16F = {(long)_atlasTexels * _atlasTexels * _atlasTexels * 2 / 1e6:F0} MB | indirection {bricksX}×{bricksY}×{bricksZ}");
        StartupTrace.Record($"atlas layout planned ({stageTimer.Elapsed.TotalMilliseconds:F1} ms)");

        _pixelStorei(UNPACK_ALIGNMENT, 1);

        // ---- atlas (R16F) filled from the real volume, bulk upload ----
        stageTimer.Restart();
        long atlasBytes = (long)_atlasTexels * _atlasTexels * _atlasTexels * 2;
        ushort* atlasBuf = mixed.Count == 0
            ? null
            : (ushort*)NativeMemory.AllocZeroed((nuint)atlasBytes);
        if (atlasBuf != null)
        {
            Parallel.For(0, mixed.Count, mi =>
            {
                int b = mixed[mi];
                int bx = b % bricksX, by = b / bricksX % bricksY, bz = b / (bricksX * bricksY);
                int tx = mi % _tilesPerAxis, ty = mi / _tilesPerAxis % _tilesPerAxis, tz = mi / (_tilesPerAxis * _tilesPerAxis);
                for (int z = 0; z < 9; z++)
                    for (int y = 0; y < 9; y++)
                    {
                        long row = ((long)(tz * 9 + z) * _atlasTexels + ty * 9 + y) * _atlasTexels + tx * 9;
                        for (int x = 0; x < 9; x++)
                            atlasBuf[row + x] = BitConverter.HalfToUInt16Bits((Half)_vol!.GetVoxel(bx * 8 + x, by * 8 + y, bz * 8 + z));
                    }
            });
        }
        _atlas16 = GenTex3D(R16F, _atlasTexels, LINEAR);
        var swu = Stopwatch.StartNew();
        _texImage3D(TEXTURE_3D, 0, R16F, _atlasTexels, _atlasTexels, _atlasTexels, 0, RED, HALF_FLOAT, atlasBuf);
        _finish();
        if (atlasBuf == null)
            Console.WriteLine($"# initial empty atlas allocation: {swu.Elapsed.TotalMilliseconds:F1} ms ({atlasBytes / 1e6:F0} MB reserved)");
        else
            Console.WriteLine($"# initial full atlas upload: {swu.Elapsed.TotalMilliseconds:F1} ms ({atlasBytes / swu.Elapsed.TotalSeconds / 1e9:F2} GB/s)");
        if (atlasBuf != null) NativeMemory.Free(atlasBuf);
        StartupTrace.Record($"atlas texture prepared ({stageTimer.Elapsed.TotalMilliseconds:F1} ms)");

        // ---- rectangular brick indirection RGBA8 ----
        stageTimer.Restart();
        _indirCpu = new byte[vol.BrickCount * 4];
        for (int mi = 0; mi < mixed.Count; mi++)
        {
            int b = mixed[mi];
            _brickToTile[b] = mi;
            _indirCpu[(long)b * 4 + 0] = (byte)(mi % _tilesPerAxis);
            _indirCpu[(long)b * 4 + 1] = (byte)(mi / _tilesPerAxis % _tilesPerAxis);
            _indirCpu[(long)b * 4 + 2] = (byte)(mi / (_tilesPerAxis * _tilesPerAxis));
            _indirCpu[(long)b * 4 + 3] = 255;
        }
        _nextTile = mixed.Count;
        _freeAtlasTiles.Clear();
        for (int b = 0; b < vol.BrickCount; b++)
        {
            int st = vol.BrickState(b);
            if (st == SparseSdfVolume.Solid) _indirCpu[(long)b * 4 + 3] = 128;  // solid
            else if (st == SparseSdfVolume.Air) _indirCpu[(long)b * 4 + 3] = 0; // air
        }
        _indir = GenTex3D(RGBA8, bricksX, NEAREST);
        fixed (byte* indPtr = _indirCpu)
            _texImage3D(TEXTURE_3D, 0, RGBA8, bricksX, bricksY, bricksZ, 0, RGBA, UNSIGNED_BYTE, indPtr);

        // ---- coarse R16F ----
        long coarseCount = (long)vol.CoarseX * vol.CoarseY * vol.CoarseZ;
        ushort* cb = (ushort*)NativeMemory.Alloc((nuint)(coarseCount * 2));
        for (long i = 0; i < coarseCount; i++) cb[i] = BitConverter.HalfToUInt16Bits((Half)vol.Coarse[i]);
        _coarseTex = GenTex3D(R16F, vol.CoarseX, LINEAR);
        _texImage3D(TEXTURE_3D, 0, R16F, vol.CoarseX, vol.CoarseY, vol.CoarseZ, 0, RED, HALF_FLOAT, cb);
        NativeMemory.Free(cb);
        StartupTrace.Record($"indirection and coarse textures uploaded ({stageTimer.Elapsed.TotalMilliseconds:F1} ms)");

        if (_mode == "upload")
        {
            _atlas8 = GenTex3D(R8, _atlasTexels, LINEAR);
            _texImage3D(TEXTURE_3D, 0, R8, _atlasTexels, _atlasTexels, _atlasTexels, 0, RED, UNSIGNED_BYTE, null);
            _staging = (ushort*)NativeMemory.Alloc((nuint)(4000L * 729 * 2));
            _stagingB = (byte*)NativeMemory.Alloc((nuint)(4000L * 729));
            for (long i = 0; i < 4000L * 729; i++) { _staging[i] = BitConverter.HalfToUInt16Bits((Half)0.5f); _stagingB[i] = 127; }
            var rng = new Random(1);
            _scatter = new (int, int, int)[4000];
            for (int i = 0; i < _scatter.Length; i++)
                _scatter[i] = (rng.Next(_tilesPerAxis) * 9, rng.Next(_tilesPerAxis) * 9, rng.Next(_tilesPerAxis) * 9);
        }
        else
        {
            if (IsLive || IsViewer)
            {
                SetupLiveMode();
                if (IsSeekBench)
                {
                    Volatile.Write(ref _isPaused, true);
                }
            }
            BuildRaymarch();
            if (_liveTool is not null) UploadToolProfile(_liveTool);
            if (IsSeam)
            {
                if (!SetupSeamLine())
                {
                    throw new Exception("seam setup failed: no adjacent mixed tiles found");
                }
            }
        }
        Volatile.Write(ref _ready, true);
    }

    private int GenTex3D(int ifmt, int size, int filter)
    {
        uint t;
        _genTextures(1, &t);
        _bindTexture(TEXTURE_3D, (int)t);
        _texParameteri(TEXTURE_3D, MIN_FILTER, filter);
        _texParameteri(TEXTURE_3D, MAG_FILTER, filter);
        _texParameteri(TEXTURE_3D, WRAP_S, CLAMP);
        _texParameteri(TEXTURE_3D, WRAP_T, CLAMP);
        _texParameteri(TEXTURE_3D, WRAP_R, CLAMP);
        return (int)t;
    }

    // ------------------------------------------------------------------ render loop
    protected override void OnOpenGlRender(GlInterface gl, int fb)
    {
        if (!Volatile.Read(ref _ready)) { RequestNextFrameRendering(); return; }
        try
        {
            if (_mode == "upload") UploadFrame();
            else if (IsLive) LiveFrame(fb);
            else if (IsViewer) ViewerFrame(fb);
            else if (IsSeam) SeamFrame(fb);
            else RaymarchFrame(fb);
            StartupTrace.CompleteFirstFrame();
        }
        catch (Exception ex) { Console.WriteLine($"GL BENCH RENDER FAILED: {ex}"); Environment.Exit(1); }
        RequestNextFrameRendering();
    }

    // ------------------------------------------------------------------ B3 upload
    private static readonly (string label, int tiles, bool r8, bool indir)[] UploadConfigs =
    {
        ("R16F  100 tiles/frame", 100, false, false),
        ("R16F  250 tiles/frame", 250, false, false),
        ("R16F  500 tiles/frame", 500, false, false),
        ("R16F 1000 tiles/frame", 1000, false, false),
        ("R16F 2000 tiles/frame", 2000, false, false),
        ("R16F 4000 tiles/frame", 4000, false, false),
        ("R8    500 tiles/frame", 500, true, false),
        ("R8   2000 tiles/frame", 2000, true, false),
        ("R16F  500 + indirection texels", 500, false, true),
    };

    /// <summary>Never trust inherited GL state (§9.7): Skia leaves scissor/blend/PBOs live.</summary>
    private void NeutralizeState()
    {
        const int SCISSOR_TEST = 0x0C11, CULL_FACE = 0x0B44, DEPTH_TEST = 0x0B71, BLEND = 0x0BE2,
                  STENCIL_TEST = 0x0B90, RASTERIZER_DISCARD = 0x8C89, DITHER = 0x0BD0,
                  PIXEL_PACK_BUFFER = 0x88EB, PIXEL_UNPACK_BUFFER = 0x88EC;
        _disable(SCISSOR_TEST); _disable(CULL_FACE); _disable(DEPTH_TEST); _disable(BLEND);
        _disable(STENCIL_TEST); _disable(RASTERIZER_DISCARD); _disable(DITHER);
        _colorMask(1, 1, 1, 1);
        _bindBuffer(PIXEL_PACK_BUFFER, 0); _bindBuffer(PIXEL_UNPACK_BUFFER, 0);
    }

    private void UploadFrame()
    {
        const int warm = 15, timed = 45;
        NeutralizeState();
        var (label, tiles, r8, withIndir) = UploadConfigs[_phase];
        int atlas = r8 ? _atlas8 : _atlas16;
        int fmtType = r8 ? UNSIGNED_BYTE : HALF_FLOAT;

        _finish();
        _sw.Restart();
        _bindTexture(TEXTURE_3D, atlas);
        for (int i = 0; i < tiles; i++)
        {
            var (x, y, z) = _scatter[i];
            void* src = r8 ? _stagingB + (long)i * 729 : _staging + (long)i * 729;
            _texSubImage3D(TEXTURE_3D, 0, x, y, z, 9, 9, 9, RED, fmtType, src);
        }
        if (withIndir)
        {
            _bindTexture(TEXTURE_3D, _indir);
            byte* one = stackalloc byte[4] { 1, 2, 3, 255 };
            for (int i = 0; i < tiles; i++)
                _texSubImage3D(TEXTURE_3D, 0, i % 64, i / 64 % 64, i % 32, 1, 1, 1, RGBA, UNSIGNED_BYTE, one);
        }
        _finish();
        _sw.Stop();

        _frame++;
        if (_frame > warm) _times.Add(_sw.Elapsed.TotalMilliseconds);
        if (_frame >= warm + timed)
        {
            _times.Sort();
            double avg = 0; foreach (var t in _times) avg += t;
            avg /= _times.Count;
            double mb = tiles * 729.0 * (r8 ? 1 : 2) / 1e6;
            Console.WriteLine($"{label,-32} | avg {avg,7:F2} ms  p95 {_times[(int)(_times.Count * 0.95)],7:F2} ms | {mb:F2} MB/frame → {mb / (avg / 1000) / 1000:F2} GB/s | 60fps budget: {(avg <= 16.6 ? "OK" : "OVER")}");
            _times.Clear();
            _frame = 0;
            _phase++;
            if (_phase >= UploadConfigs.Length)
            {
                int err = _getError();
                Console.WriteLine($"# done, glGetError={err:X}");
                Console.Out.Flush();
                Environment.Exit(0);
            }
        }
    }

    private bool SetupSeamLine()
    {
        if (_vol is null)
        {
            return false;
        }

        int bricksX = _vol.BricksX, bricksY = _vol.BricksY;
        float brickW = 8 * _vol.VoxelSize;
        for (int b = 0; b < _brickToTile.Length; b++)
        {
            if (_brickToTile[b] < 0)
            {
                continue;
            }

            int bx = b % bricksX;
            int by = b / bricksX % bricksY;
            int bz = b / (bricksX * bricksY);
            if (bx + 1 >= bricksX)
            {
                continue;
            }

            int n = _vol.BrickId(bx + 1, by, bz);
            if (_brickToTile[n] < 0)
            {
                continue;
            }

            float faceX = (bx + 1) * brickW;
            float yc = (by + 0.5f) * brickW;
            float zc = (bz + 0.5f) * brickW;
            _seamP0 = new Vector3(faceX - 1.8f * _vol.VoxelSize, yc, zc);
            _seamP1 = new Vector3(faceX + 1.8f * _vol.VoxelSize, yc, zc);
            Console.WriteLine($"# seam line set at bx={bx}->{bx + 1}, by={by}, bz={bz}");
            return true;
        }

        return false;
    }

    private void ApplySeamCarveUpdate()
    {
        if (_vol is null || _seamUpdated)
        {
            return;
        }

        var tool = ToolProfile.FlatEndmill(3, 12, 128);
        var segs = new PathSegment[64];
        Vector3 a = _seamP0;
        Vector3 b = _seamP1;
        for (int i = 0; i < segs.Length; i++)
        {
            float t0 = i / (float)segs.Length;
            float t1 = (i + 1) / (float)segs.Length;
            segs[i] = new PathSegment(Vector3.Lerp(a, b, t0), Vector3.Lerp(a, b, t1));
        }

        _vol.StampBatch(segs, 0, segs.Length, tool, parallel: true);
        UploadDirtyMappedBricks(_vol.DirtyBricks);
        _seamUpdated = true;
        Console.WriteLine($"# seam update stamped: dirty={_vol.DirtyBricks.Count}");
    }

    private void UploadDirtyMappedBricks(IReadOnlyList<int> dirty)
    {
        if (_vol is null || dirty.Count == 0)
        {
            return;
        }

        Span<int> temp = stackalloc int[dirty.Count];
        for (int i = 0; i < dirty.Count; i++) temp[i] = dirty[i];
        UploadDirtyMappedBricks(temp, dirty.Count);
    }

    private void UploadDirtyMappedBricks(ReadOnlySpan<int> dirty, int count)
    {
        if (_vol is null || count <= 0 || _indirCpu.Length == 0)
        {
            return;
        }

        var submitTimer = Stopwatch.StartNew();
        bool measurePhases = IsSeeking;
        long gatherTicks = 0;
        long atlasTicks = 0;
        long indirectionTicks = 0;
        long mixedTiles = 0;
        long newTiles = 0;
        _bindTexture(TEXTURE_3D, _atlas16);
        Span<Half> tile = stackalloc Half[9 * 9 * 9];
        int bricksX = _vol.BricksX, bricksY = _vol.BricksY;
        int updated = 0;
        for (int i = 0; i < count; i++)
        {
            int brick = dirty[i];
            if ((uint)brick < (uint)_brickToTile.Length
                && _vol.BrickState(brick) < 0
                && _brickToTile[brick] >= 0)
            {
                ReleaseAtlasTile(brick);
            }
        }

        for (int i = 0; i < count; i++)
        {
            int b = dirty[i];
            if ((uint)b >= (uint)_brickToTile.Length)
            {
                continue;
            }

            int st = _vol.BrickState(b);
            int tileIndex = _brickToTile[b];
            if (st >= 0 && tileIndex < 0)
            {
                tileIndex = _brickToTile[b] = AllocateAtlasTile();
                if (measurePhases) newTiles++;
            }

            int tx = 0, ty = 0, tz = 0;
            if (st >= 0 && tileIndex >= 0)
            {
                if (measurePhases) mixedTiles++;
                long gatherStartedAt = measurePhases ? Stopwatch.GetTimestamp() : 0;
                int bx = b % bricksX, by = b / bricksX % bricksY, bz = b / (bricksX * bricksY);
                int k = 0;
                for (int z = 0; z < 9; z++)
                    for (int y = 0; y < 9; y++)
                        for (int x = 0; x < 9; x++)
                            tile[k++] = (Half)_vol.GetVoxel(bx * 8 + x, by * 8 + y, bz * 8 + z);
                if (measurePhases) gatherTicks += Stopwatch.GetTimestamp() - gatherStartedAt;

                tx = tileIndex % _tilesPerAxis;
                ty = tileIndex / _tilesPerAxis % _tilesPerAxis;
                tz = tileIndex / (_tilesPerAxis * _tilesPerAxis);
                long atlasStartedAt = measurePhases ? Stopwatch.GetTimestamp() : 0;
                fixed (Half* p = tile)
                {
                    _texSubImage3D(TEXTURE_3D, 0, tx * 9, ty * 9, tz * 9, 9, 9, 9, RED, HALF_FLOAT, p);
                }
                if (measurePhases) atlasTicks += Stopwatch.GetTimestamp() - atlasStartedAt;
                updated++;
            }

            int off = b * 4;
            if (st >= 0 && tileIndex >= 0)
            {
                _indirCpu[off + 0] = (byte)tx;
                _indirCpu[off + 1] = (byte)ty;
                _indirCpu[off + 2] = (byte)tz;
                _indirCpu[off + 3] = 255;
            }
            else
            {
                _indirCpu[off + 0] = 0;
                _indirCpu[off + 1] = 0;
                _indirCpu[off + 2] = 0;
                _indirCpu[off + 3] = st == SparseSdfVolume.Air ? (byte)0 : (byte)128;
            }
        }

        long indirectionStartedAt = measurePhases ? Stopwatch.GetTimestamp() : 0;
        _bindTexture(TEXTURE_3D, _indir);
        fixed (byte* indirection = _indirCpu)
        {
            _texSubImage3D(TEXTURE_3D, 0, 0, 0, 0,
                _vol.BricksX, _vol.BricksY, _vol.BricksZ,
                RGBA, UNSIGNED_BYTE, indirection);
        }
        _bindTexture(TEXTURE_3D, _atlas16);
        if (measurePhases) indirectionTicks += Stopwatch.GetTimestamp() - indirectionStartedAt;

        submitTimer.Stop();
        var finishTimer = new Stopwatch();
        if (!IsViewer || IsSeekBench)
        {
            finishTimer.Start();
            _finish();
            finishTimer.Stop();
        }
        _tileUploadSubmitMs += submitTimer.Elapsed.TotalMilliseconds;
        _tileUploadFinishMs += finishTimer.Elapsed.TotalMilliseconds;
        if (measurePhases)
        {
            double tickToMs = 1000.0 / Stopwatch.Frequency;
            _tileGatherMs += gatherTicks * tickToMs;
            _atlasSubImageMs += atlasTicks * tickToMs;
            _indirSubImageMs += indirectionTicks * tickToMs;
            _mixedTileUploads += mixedTiles;
            _newTileUploads += newTiles;
        }
        if (IsSeam)
        {
            Console.WriteLine($"# seam upload updated tiles={updated}");
        }
    }

    private int AllocateAtlasTile()
    {
        if (_freeAtlasTiles.TryPop(out int recycled))
        {
            return recycled;
        }

        int capacity = _tilesPerAxis * _tilesPerAxis * _tilesPerAxis;
        if (_nextTile >= capacity)
        {
            int reclaimed = ReclaimUniformAtlasTiles();
            if (_freeAtlasTiles.TryPop(out recycled))
            {
                Console.WriteLine($"# atlas reused {reclaimed:N0} uniform-brick slots at capacity");
                return recycled;
            }
            GrowAtlas();
        }
        return _nextTile++;
    }

    private void ReleaseAtlasTile(int brick)
    {
        int tileIndex = _brickToTile[brick];
        if (tileIndex < 0)
        {
            return;
        }

        _brickToTile[brick] = -1;
        _freeAtlasTiles.Push(tileIndex);
        int offset = brick * 4;
        _indirCpu[offset + 0] = 0;
        _indirCpu[offset + 1] = 0;
        _indirCpu[offset + 2] = 0;
        _indirCpu[offset + 3] = _vol!.BrickState(brick) == SparseSdfVolume.Air
            ? (byte)0
            : (byte)128;
    }

    private int ReclaimUniformAtlasTiles()
    {
        int reclaimed = 0;
        for (int brick = 0; brick < _brickToTile.Length; brick++)
        {
            if (_brickToTile[brick] >= 0 && _vol!.BrickState(brick) < 0)
            {
                ReleaseAtlasTile(brick);
                reclaimed++;
            }
        }
        return reclaimed;
    }

    /// <summary>Emergency fallback beyond the viewer's measured 56³ capacity: grow the atlas,
    /// re-gather active bricks from the source volume, rebuild indirection, and swap textures.</summary>
    private void GrowAtlas()
    {
        int newTpa = _tilesPerAxis + 4;
        int newTexels = newTpa * 9;
        int bricksX = _vol!.BricksX, bricksY = _vol.BricksY, bricksZ = _vol.BricksZ;
        var sw = Stopwatch.StartNew();

        int reclaimed = ReclaimUniformAtlasTiles();
        var mapped = new System.Collections.Generic.List<int>(_nextTile - _freeAtlasTiles.Count);
        for (int b = 0; b < _brickToTile.Length; b++) if (_brickToTile[b] >= 0) mapped.Add(b);

        long bytes = (long)newTexels * newTexels * newTexels * 2;
        ushort* buf = (ushort*)NativeMemory.AllocZeroed((nuint)bytes);
        Parallel.For(0, mapped.Count, mi =>
        {
            int b = mapped[mi];
            int ti = _brickToTile[b];
            int bx = b % bricksX, by = b / bricksX % bricksY, bz = b / (bricksX * bricksY);
            int tx = ti % newTpa, ty = ti / newTpa % newTpa, tz = ti / (newTpa * newTpa);
            for (int z = 0; z < 9; z++)
                for (int y = 0; y < 9; y++)
                {
                    long row = ((long)(tz * 9 + z) * newTexels + ty * 9 + y) * newTexels + tx * 9;
                    for (int x = 0; x < 9; x++)
                        buf[row + x] = BitConverter.HalfToUInt16Bits((Half)_vol.GetVoxel(bx * 8 + x, by * 8 + y, bz * 8 + z));
                }
        });

        uint old = (uint)_atlas16;
        int tex = GenTex3D(R16F, newTexels, LINEAR);
        _texImage3D(TEXTURE_3D, 0, R16F, newTexels, newTexels, newTexels, 0, RED, HALF_FLOAT, buf);
        NativeMemory.Free(buf);
        _deleteTextures(1, &old);
        _atlas16 = tex;
        _tilesPerAxis = newTpa;
        _atlasTexels = newTexels;

        // Remap indirection tile coordinates after changing the atlas shape.
        for (int mi = 0; mi < mapped.Count; mi++)
        {
            int b = mapped[mi];
            int ti = _brickToTile[b];
            long off = (long)b * 4;
            _indirCpu[off + 0] = (byte)(ti % newTpa);
            _indirCpu[off + 1] = (byte)(ti / newTpa % newTpa);
            _indirCpu[off + 2] = (byte)(ti / (newTpa * newTpa));
        }
        _bindTexture(TEXTURE_3D, _indir);
        fixed (byte* ip = _indirCpu)
            _texImage3D(TEXTURE_3D, 0, RGBA8, bricksX, bricksY, bricksZ, 0, RGBA, UNSIGNED_BYTE, ip);
        _bindTexture(TEXTURE_3D, _atlas16);

        _useProgram(_prog);
        Set1("uTiles", _tilesPerAxis);
        Set1("uAtlasTexels", _atlasTexels);
        sw.Stop();
        _atlasGrowthCount++;
        _atlasGrowthMs += sw.Elapsed.TotalMilliseconds;
        Console.WriteLine($"# atlas grown to {newTpa}\u00b3 tiles ({newTexels}\u00b3 R16F = {bytes / 1e6:F0} MB) in {sw.Elapsed.TotalMilliseconds:F1} ms, active={mapped.Count}, reclaimed={reclaimed}, free={_freeAtlasTiles.Count}");
    }

    private void ViewerFrame(int fb)
    {
        const int capPerFrame = 1167;                      // ≈ 70k stamplets/s at 60 fps
        var swFrame = Stopwatch.StartNew();
        int take = 0;
        int uploaded = 0;
        bool seeking = ProcessViewerSeek(ref uploaded);
        if (!seeking && _vol is not null && _liveTool is not null && _liveDirty is not null)
        {
            int budget = IsPaused ? 0 : capPerFrame;
            take = Math.Min(budget, _liveSegs.Length - _liveCursor);
            if (take > 0)
            {
                _vol.StampBatch(_liveSegs, _liveCursor, take, _liveTool,
                    parallel: true, conservativeCull: false);
                _toolPos = _liveSegs[_liveCursor + take - 1].B;
                Volatile.Write(ref _liveCursor, _liveCursor + take);
                foreach (int b in _vol.DirtyBricks) _liveDirty.Mark(b);
            }
            uploaded = _liveDirty.DrainTo(_liveDrain);
            UploadDirtyMappedBricks(_liveDrain, uploaded);
        }
        else if (seeking && _liveDirty is not null && _liveDirty.Count > 0)
        {
            int n = _liveDirty.DrainTo(_liveDrain);
            UploadDirtyMappedBricks(_liveDrain, n);
            uploaded += n;
        }
        RaymarchFrame(fb);
        StartupTrace.CompleteFirstFrame();
        if (_liveCursor >= _liveSegs.Length || _frame >= _snapshotWaitFrames)
        {
            TryCaptureSnapshot(fb);
        }
        swFrame.Stop();
        _viewerLastFrameMs = swFrame.Elapsed.TotalMilliseconds;
        _viewerLastStPerSec = seeking ? 0 : take * 60.0;
        _viewerLastUploads = uploaded;
        CompletePendingSeekMeasurement();
        AdvanceSeekBenchmark();
    }

    private bool ProcessViewerSeek(ref int uploaded)
    {
        int requested = Volatile.Read(ref _requestedSeekCursor);
        if (requested < 0)
        {
            return false;
        }

        SetToolPositionForCursor(requested);

        if (_seekBuildTask is not null)
        {
            if (_seekBuildTarget != requested && _seekBuildCancellation is { IsCancellationRequested: false })
            {
                Console.WriteLine($"# seek rebuild cancellation requested: step={_seekBuildTarget} -> {requested}");
                _seekBuildCancellation.Cancel();
            }
            if (!_seekBuildTask.IsCompleted)
            {
                return true;
            }

            Task<ViewerSeekResult> completed = _seekBuildTask;
            int completedTarget = _seekBuildTarget;
            ViewerGpuSnapshot? completedSnapshot = null;
            try
            {
                if (completed.IsFaulted)
                {
                    Console.WriteLine($"# seek to step {completedTarget} failed: {completed.Exception?.GetBaseException()}");
                    Interlocked.CompareExchange(ref _requestedSeekCursor, -1, completedTarget);
                }
                else if (!completed.IsCanceled)
                {
                    ViewerSeekResult result = completed.Result;
                    completedSnapshot = result.GpuSnapshot;
                    if (result.Target == Volatile.Read(ref _requestedSeekCursor)
                        && result.Generation == Volatile.Read(ref _seekRequestGeneration))
                    {
                        Console.WriteLine($"# seek CPU rebuilt: step={result.Target}/{_liveSegs.Length}, init={result.VolumeInitMs:F1} ms, stamp={result.StampMs:F1} ms, total={result.TotalMs:F1} ms");
                        ReportSeekProgress(result.Generation, 920, "Uploading render data…",
                            indeterminate: true);
                        ViewerGpuRebuildMetrics gpu;
                        try
                        {
                            gpu = ApplyViewerGpuSnapshot(result.Volume, completedSnapshot);
                        }
                        finally
                        {
                            completedSnapshot.Dispose();
                            completedSnapshot = null;
                        }
                        _vol = result.Volume;
                        Volatile.Write(ref _liveCursor, result.Target);
                        _liveDirty = new DirtyBrickSet(_brickToTile.Length);
                        uploaded += gpu.Uploaded;
                        SetToolPositionForCursor(result.Target);
                        ReportSeekProgress(result.Generation, 1000, "Seek complete");
                        Interlocked.CompareExchange(ref _requestedSeekCursor, -1, result.Target);
                        long appliedAt = Stopwatch.GetTimestamp();
                        _pendingSeekMeasurement = new ViewerSeekMeasurement
                        {
                            Kind = "rebuild",
                            From = result.From,
                            Target = result.Target,
                            Uploaded = gpu.Uploaded,
                            MixedTiles = gpu.MixedTiles,
                            NewTiles = gpu.NewTiles,
                            RequestedAt = result.RequestedAt,
                            AppliedAt = appliedAt,
                            DispatchWaitMs = Stopwatch.GetElapsedTime(result.RequestedAt, result.ScheduledAt).TotalMilliseconds,
                            WorkerQueueMs = Stopwatch.GetElapsedTime(result.ScheduledAt, result.WorkerStartedAt).TotalMilliseconds,
                            VolumeInitMs = result.VolumeInitMs,
                            StampMs = result.StampMs,
                            CpuTotalMs = result.TotalMs,
                            PriorFrameDrainMs = gpu.PriorFrameDrainMs,
                            GpuStateMs = gpu.StateSetupMs,
                            GpuUploadMs = gpu.MixedUploadMs,
                            UploadSubmitMs = gpu.UploadSubmitMs,
                            UploadFinishMs = gpu.UploadFinishMs,
                            GatherMs = gpu.GatherMs,
                            AtlasSubImageMs = gpu.AtlasSubImageMs,
                            IndirectionSubImageMs = gpu.IndirectionSubImageMs,
                            GpuTotalMs = gpu.TotalMs,
                            AtlasGrowths = gpu.AtlasGrowths,
                            AtlasGrowthMs = gpu.AtlasGrowthMs,
                            RequestToAppliedMs = Stopwatch.GetElapsedTime(result.RequestedAt, appliedAt).TotalMilliseconds
                        };
                        Console.WriteLine($"# seek GPU rebuilt: prior frame drain={gpu.PriorFrameDrainMs:F1} ms, state enqueue={gpu.StateSetupMs:F1} ms, mixed upload={gpu.MixedUploadMs:F1} ms (submit={gpu.UploadSubmitMs:F1}, finish={gpu.UploadFinishMs:F1}, gather={gpu.GatherMs:F1}, atlas calls={gpu.AtlasSubImageMs:F1}, indirection calls={gpu.IndirectionSubImageMs:F1}), total={gpu.TotalMs:F1} ms, atlas grows={gpu.AtlasGrowths}/{gpu.AtlasGrowthMs:F1} ms, tiles mixed/new={gpu.MixedTiles}/{gpu.NewTiles}");
                        Console.WriteLine($"# seek applied: step={result.Target}/{_liveSegs.Length}, mixed uploads={uploaded}");
                    }
                }
            }
            finally
            {
                completedSnapshot?.Dispose();
                _seekBuildTask = null;
                _seekBuildCancellation?.Dispose();
                _seekBuildCancellation = null;
                _seekBuildTarget = -1;
                Volatile.Write(ref _seekBuildActive, 0);
            }
            return true;
        }

        long requestedAt = Volatile.Read(ref _seekRequestedAt);
        if (Stopwatch.GetElapsedTime(requestedAt).TotalMilliseconds < SeekDebounceMilliseconds)
        {
            return true;
        }

        requested = Math.Clamp(Volatile.Read(ref _requestedSeekCursor), 0, _liveSegs.Length);
        int current = Volatile.Read(ref _liveCursor);
        if (requested == current)
        {
            ReportSeekProgress(Volatile.Read(ref _seekRequestGeneration), 1000, "Seek complete");
            Interlocked.CompareExchange(ref _requestedSeekCursor, -1, requested);
            return true;
        }

        if (requested > current && requested - current <= InlineForwardSeekLimit
            && _vol is not null && _liveTool is not null && _liveDirty is not null)
        {
            Volatile.Write(ref _seekBuildActive, 1);
            try
            {
                int generation = Volatile.Read(ref _seekRequestGeneration);
                ReportSeekProgress(generation, 80, "Applying nearby seek…",
                    indeterminate: true);
                long applyStartedAt = Stopwatch.GetTimestamp();
                var stampTimer = Stopwatch.StartNew();
                StampVolumeRange(_vol, _liveSegs, current, requested - current, _liveTool,
                    CancellationToken.None, _liveDirty);
                stampTimer.Stop();
                Volatile.Write(ref _liveCursor, requested);
                ViewerGpuRebuildMetrics gpu;
                string kind;
                if (_liveDirty.Count >= InlineBulkDirtyThreshold)
                {
                    kind = "inline-bulk";
                    gpu = RebuildViewerGpuState();
                    _liveDirty = new DirtyBrickSet(_brickToTile.Length);
                }
                else
                {
                    kind = "inline";
                    int growthCountBefore = _atlasGrowthCount;
                    double growthMsBefore = _atlasGrowthMs;
                    double submitMsBefore = _tileUploadSubmitMs;
                    double finishMsBefore = _tileUploadFinishMs;
                    double gatherMsBefore = _tileGatherMs;
                    double atlasMsBefore = _atlasSubImageMs;
                    double indirectionMsBefore = _indirSubImageMs;
                    long mixedTilesBefore = _mixedTileUploads;
                    long newTilesBefore = _newTileUploads;
                    var priorFrameTimer = new Stopwatch();
                    if (IsSeekBench)
                    {
                        priorFrameTimer.Start();
                        _finish();
                        priorFrameTimer.Stop();
                    }
                    var uploadTimer = Stopwatch.StartNew();
                    int incrementalUploads = DrainAllViewerUploads();
                    uploadTimer.Stop();
                    gpu = new ViewerGpuRebuildMetrics(
                        incrementalUploads,
                        _mixedTileUploads - mixedTilesBefore,
                        _newTileUploads - newTilesBefore,
                        priorFrameTimer.Elapsed.TotalMilliseconds,
                        0,
                        uploadTimer.Elapsed.TotalMilliseconds,
                        _tileUploadSubmitMs - submitMsBefore,
                        _tileUploadFinishMs - finishMsBefore,
                        _tileGatherMs - gatherMsBefore,
                        _atlasSubImageMs - atlasMsBefore,
                        _indirSubImageMs - indirectionMsBefore,
                        priorFrameTimer.Elapsed.TotalMilliseconds + uploadTimer.Elapsed.TotalMilliseconds,
                        _atlasGrowthCount - growthCountBefore,
                        _atlasGrowthMs - growthMsBefore);
                }
                int inlineUploads = gpu.Uploaded;
                uploaded += inlineUploads;
                SetToolPositionForCursor(requested);
                ReportSeekProgress(generation, 1000, "Seek complete");
                Interlocked.CompareExchange(ref _requestedSeekCursor, -1, requested);
                long appliedAt = Stopwatch.GetTimestamp();
                _pendingSeekMeasurement = new ViewerSeekMeasurement
                {
                    Kind = kind,
                    From = current,
                    Target = requested,
                    Uploaded = inlineUploads,
                    MixedTiles = gpu.MixedTiles,
                    NewTiles = gpu.NewTiles,
                    RequestedAt = requestedAt,
                    AppliedAt = appliedAt,
                    DispatchWaitMs = Stopwatch.GetElapsedTime(requestedAt, applyStartedAt).TotalMilliseconds,
                    StampMs = stampTimer.Elapsed.TotalMilliseconds,
                    CpuTotalMs = stampTimer.Elapsed.TotalMilliseconds,
                    PriorFrameDrainMs = gpu.PriorFrameDrainMs,
                    GpuStateMs = gpu.StateSetupMs,
                    GpuUploadMs = gpu.MixedUploadMs,
                    UploadSubmitMs = gpu.UploadSubmitMs,
                    UploadFinishMs = gpu.UploadFinishMs,
                    GatherMs = gpu.GatherMs,
                    AtlasSubImageMs = gpu.AtlasSubImageMs,
                    IndirectionSubImageMs = gpu.IndirectionSubImageMs,
                    GpuTotalMs = gpu.TotalMs,
                    AtlasGrowths = gpu.AtlasGrowths,
                    AtlasGrowthMs = gpu.AtlasGrowthMs,
                    RequestToAppliedMs = Stopwatch.GetElapsedTime(requestedAt, appliedAt).TotalMilliseconds
                };
                Console.WriteLine($"# seek applied {kind}: step={requested}/{_liveSegs.Length}, stamp={stampTimer.Elapsed.TotalMilliseconds:F1} ms, prior frame drain={gpu.PriorFrameDrainMs:F1} ms, upload={gpu.MixedUploadMs:F1} ms (submit={gpu.UploadSubmitMs:F1}, finish={gpu.UploadFinishMs:F1}), uploaded={inlineUploads}, mixed={gpu.MixedTiles}, new={gpu.NewTiles}");
            }
            finally
            {
                Volatile.Write(ref _seekBuildActive, 0);
            }
            return true;
        }

        StartViewerSeekBuild(current, requested);
        return true;
    }

    private void StartViewerSeekBuild(int from, int target)
    {
        if (_liveTool is null)
        {
            Interlocked.CompareExchange(ref _requestedSeekCursor, -1, target);
            return;
        }

        PathSegment[] segments = _liveSegs;
        ToolProfile tool = _liveTool;
        SparseSdfVolume? forwardBase = target > from ? _vol : null;
        int replayStart = forwardBase is null ? 0 : from;
        var cancellation = new CancellationTokenSource();
        CancellationToken token = cancellation.Token;
        int generation = Volatile.Read(ref _seekRequestGeneration);
        long requestedAt = Volatile.Read(ref _seekRequestedAt);
        long scheduledAt = Stopwatch.GetTimestamp();
        _seekBuildCancellation = cancellation;
        _seekBuildTarget = target;
        Volatile.Write(ref _seekBuildActive, 1);
        _seekBuildTask = Task.Run(() =>
        {
            ViewerGpuSnapshot? gpuSnapshot = null;
            try
            {
                long workerStartedAt = Stopwatch.GetTimestamp();
                var totalTimer = Stopwatch.StartNew();
                ReportSeekProgress(generation, 20, forwardBase is null
                    ? "Creating stock…"
                    : "Copying current stock…", indeterminate: true);
                var initTimer = Stopwatch.StartNew();
                var replacement = forwardBase?.DeepClone() ?? CreateViewerStockVolume();
                initTimer.Stop();
                token.ThrowIfCancellationRequested();

                int replayCount = target - replayStart;
                ReportSeekProgress(generation, replayCount == 0 ? 700 : 80,
                    "Replaying toolpath…");
                var stampTimer = Stopwatch.StartNew();
                replacement.CoarseEnabled = false;
                StampVolumeRange(replacement, segments, replayStart, replayCount,
                    tool, token, dirty: null,
                    progress: (done, total) => ReportSeekProgress(generation,
                        80 + (total == 0 ? 620 : (int)(620L * done / total)),
                        "Replaying toolpath…"));
                ReportSeekProgress(generation, 720, "Building coarse field…",
                    indeterminate: true);
                replacement.RebuildCoarseFromBrickStates();
                replacement.CoarseEnabled = true;
                stampTimer.Stop();
                token.ThrowIfCancellationRequested();
                totalTimer.Stop();

                gpuSnapshot = PrepareViewerGpuSnapshot(replacement, token, generation);
                token.ThrowIfCancellationRequested();
                ViewerGpuSnapshot resultSnapshot = gpuSnapshot;
                gpuSnapshot = null;
                return new ViewerSeekResult(generation, from, target, replacement,
                    resultSnapshot, requestedAt, scheduledAt, workerStartedAt,
                    initTimer.Elapsed.TotalMilliseconds,
                    stampTimer.Elapsed.TotalMilliseconds, totalTimer.Elapsed.TotalMilliseconds);
            }
            finally
            {
                gpuSnapshot?.Dispose();
            }
        }, token);
        Console.WriteLine($"# seek rebuild started: from={from}, base={replayStart}, step={target}/{segments.Length}, dispatch wait={Stopwatch.GetElapsedTime(requestedAt, scheduledAt).TotalMilliseconds:F1} ms");
    }

    private static void StampVolumeRange(SparseSdfVolume volume, PathSegment[] segments,
        int start, int count, ToolProfile tool, CancellationToken cancellationToken,
        DirtyBrickSet? dirty, Action<int, int>? progress = null)
    {
        int end = start + count;
        for (int cursor = start; cursor < end; cursor += SeekStampBatchSize)
        {
            cancellationToken.ThrowIfCancellationRequested();
            int take = Math.Min(SeekStampBatchSize, end - cursor);
            volume.StampBatch(segments, cursor, take, tool,
                parallel: true, conservativeCull: false);
            if (dirty is not null)
            {
                foreach (int brick in volume.DirtyBricks)
                {
                    dirty.Mark(brick);
                }
            }
            progress?.Invoke(cursor + take - start, count);
        }
    }

    private int DrainAllViewerUploads()
    {
        if (_liveDirty is null)
        {
            return 0;
        }

        int uploaded = 0;
        while (_liveDirty.Count > 0)
        {
            int n = _liveDirty.DrainTo(_liveDrain);
            UploadDirtyMappedBricks(_liveDrain, n);
            uploaded += n;
        }
        return uploaded;
    }

    private ViewerGpuRebuildMetrics RebuildViewerGpuState()
    {
        if (_vol is null)
        {
            return default;
        }

        ViewerGpuSnapshot snapshot = PrepareViewerGpuSnapshot(
            _vol, CancellationToken.None, generation: -1);
        try
        {
            return ApplyViewerGpuSnapshot(_vol, snapshot);
        }
        finally
        {
            snapshot.Dispose();
        }
    }

    private ViewerGpuSnapshot PrepareViewerGpuSnapshot(SparseSdfVolume volume,
        CancellationToken cancellationToken, int generation)
    {
        void Report(int permille, string stage, bool indeterminate = false)
        {
            if (generation >= 0)
            {
                ReportSeekProgress(generation, permille, stage, indeterminate);
            }
        }

        Report(750, "Indexing render bricks…", indeterminate: true);
        var stateTimer = Stopwatch.StartNew();
        int[] brickToTile = new int[volume.BrickCount];
        Array.Fill(brickToTile, -1);
        byte[] indirection = new byte[volume.BrickCount * 4];
        var mixed = new List<int>();
        for (int brick = 0; brick < volume.BrickCount; brick++)
        {
            if ((brick & 16_383) == 0)
            {
                cancellationToken.ThrowIfCancellationRequested();
            }

            int state = volume.BrickState(brick);
            if (state >= 0)
            {
                mixed.Add(brick);
            }
            else
            {
                indirection[brick * 4 + 3] = state == SparseSdfVolume.Air
                    ? (byte)0
                    : (byte)128;
            }
        }

        const int tileFloor = ViewerAtlasTilesPerAxis
            * ViewerAtlasTilesPerAxis * ViewerAtlasTilesPerAxis;
        int tilesPerAxis = (int)Math.Ceiling(Math.Cbrt(Math.Max(mixed.Count, tileFloor)));
        while ((long)tilesPerAxis * tilesPerAxis * tilesPerAxis < mixed.Count)
        {
            tilesPerAxis++;
        }
        int atlasTexels = tilesPerAxis * 9;
        for (int tileIndex = 0; tileIndex < mixed.Count; tileIndex++)
        {
            int brick = mixed[tileIndex];
            brickToTile[brick] = tileIndex;
            int offset = brick * 4;
            indirection[offset + 0] = (byte)(tileIndex % tilesPerAxis);
            indirection[offset + 1] = (byte)(tileIndex / tilesPerAxis % tilesPerAxis);
            indirection[offset + 2] = (byte)(tileIndex / (tilesPerAxis * tilesPerAxis));
            indirection[offset + 3] = 255;
        }

        ushort[] coarse = new ushort[volume.Coarse.Length];
        for (int i = 0; i < coarse.Length; i++)
        {
            coarse[i] = BitConverter.HalfToUInt16Bits((Half)volume.Coarse[i]);
        }
        stateTimer.Stop();

        Report(780, "Gathering render atlas…");
        var gatherTimer = Stopwatch.StartNew();
        int tileLayerSize = tilesPerAxis * tilesPerAxis;
        int atlasDepthTexels = mixed.Count == 0
            ? 0
            : ((mixed.Count + tileLayerSize - 1) / tileLayerSize) * 9;
        long atlasBytes = (long)atlasTexels * atlasTexels
            * atlasDepthTexels * sizeof(ushort);
        ushort* atlas = mixed.Count == 0
            ? null
            : (ushort*)NativeMemory.AllocZeroed((nuint)atlasBytes);
        if (mixed.Count > 0 && atlas == null)
        {
            throw new OutOfMemoryException(
                $"Unable to allocate {atlasBytes / 1e6:F0} MB seek atlas staging buffer.");
        }

        try
        {
            const int gatherChunkSize = 256;
            int chunkCount = (mixed.Count + gatherChunkSize - 1) / gatherChunkSize;
            int completedChunks = 0;
            var options = new ParallelOptions { CancellationToken = cancellationToken };
            Parallel.For(0, chunkCount, options, chunk =>
            {
                int start = chunk * gatherChunkSize;
                int end = Math.Min(start + gatherChunkSize, mixed.Count);
                int bricksX = volume.BricksX;
                int bricksY = volume.BricksY;
                for (int tileIndex = start; tileIndex < end; tileIndex++)
                {
                    int brick = mixed[tileIndex];
                    int bx = brick % bricksX;
                    int by = brick / bricksX % bricksY;
                    int bz = brick / (bricksX * bricksY);
                    int tx = tileIndex % tilesPerAxis;
                    int ty = tileIndex / tilesPerAxis % tilesPerAxis;
                    int tz = tileIndex / (tilesPerAxis * tilesPerAxis);
                    for (int z = 0; z < 9; z++)
                        for (int y = 0; y < 9; y++)
                        {
                            long row = ((long)(tz * 9 + z) * atlasTexels + ty * 9 + y)
                                * atlasTexels + tx * 9;
                            for (int x = 0; x < 9; x++)
                            {
                                atlas[row + x] = BitConverter.HalfToUInt16Bits(
                                    (Half)volume.GetVoxel(
                                        bx * 8 + x, by * 8 + y, bz * 8 + z));
                            }
                        }
                }

                int done = Interlocked.Increment(ref completedChunks);
                if ((done & 7) == 0 || done == chunkCount)
                {
                    Report(780 + (chunkCount == 0 ? 120 : 120 * done / chunkCount),
                        "Gathering render atlas…");
                }
            });
            gatherTimer.Stop();
            Report(900, "Uploading render data…", indeterminate: true);
            return new ViewerGpuSnapshot(brickToTile, indirection, coarse,
                (nint)atlas, atlasBytes, atlasDepthTexels, mixed.Count,
                tilesPerAxis, atlasTexels, stateTimer.Elapsed.TotalMilliseconds,
                gatherTimer.Elapsed.TotalMilliseconds);
        }
        catch
        {
            if (atlas != null)
            {
                NativeMemory.Free(atlas);
            }
            throw;
        }
    }

    private ViewerGpuRebuildMetrics ApplyViewerGpuSnapshot(
        SparseSdfVolume volume, ViewerGpuSnapshot snapshot)
    {
        var totalTimer = Stopwatch.StartNew();
        var priorFrameTimer = new Stopwatch();
        if (IsSeekBench)
        {
            priorFrameTimer.Start();
            _finish();
            priorFrameTimer.Stop();
        }

        NeutralizeState();
        if (snapshot.TilesPerAxis != _tilesPerAxis
            || snapshot.AtlasTexels != _atlasTexels)
        {
            throw new InvalidOperationException(
                $"Seek atlas dimensions changed: {snapshot.TilesPerAxis}³ tiles, expected {_tilesPerAxis}³.");
        }

        var uploadTimer = Stopwatch.StartNew();
        var atlasTimer = Stopwatch.StartNew();
        _bindTexture(TEXTURE_3D, _atlas16);
        if (snapshot.Atlas != null)
        {
            _texSubImage3D(TEXTURE_3D, 0, 0, 0, 0,
                snapshot.AtlasTexels, snapshot.AtlasTexels, snapshot.AtlasDepthTexels,
                RED, HALF_FLOAT, snapshot.Atlas);
        }
        atlasTimer.Stop();
        double atlasMs = atlasTimer.Elapsed.TotalMilliseconds;

        var indirectionTimer = Stopwatch.StartNew();
        _bindTexture(TEXTURE_3D, _indir);
        fixed (byte* indirection = snapshot.Indirection)
        {
            _texSubImage3D(TEXTURE_3D, 0, 0, 0, 0,
                volume.BricksX, volume.BricksY, volume.BricksZ,
                RGBA, UNSIGNED_BYTE, indirection);
        }
        indirectionTimer.Stop();
        double indirectionMs = indirectionTimer.Elapsed.TotalMilliseconds;

        _bindTexture(TEXTURE_3D, _coarseTex);
        fixed (ushort* coarse = snapshot.Coarse)
        {
            _texSubImage3D(TEXTURE_3D, 0, 0, 0, 0,
                volume.CoarseX, volume.CoarseY, volume.CoarseZ,
                RED, HALF_FLOAT, coarse);
        }
        _bindTexture(TEXTURE_3D, _atlas16);

        _brickToTile = snapshot.BrickToTile;
        _indirCpu = snapshot.Indirection;
        _nextTile = snapshot.MixedTiles;
        _freeAtlasTiles.Clear();

        var finishTimer = new Stopwatch();
        if (IsSeekBench)
        {
            finishTimer.Start();
            _finish();
            finishTimer.Stop();
        }
        uploadTimer.Stop();
        totalTimer.Stop();

        double submitMs = uploadTimer.Elapsed.TotalMilliseconds
            - finishTimer.Elapsed.TotalMilliseconds;
        _tileUploadSubmitMs += submitMs;
        _tileUploadFinishMs += finishTimer.Elapsed.TotalMilliseconds;
        _tileGatherMs += snapshot.GatherMs;
        _atlasSubImageMs += atlasMs;
        _indirSubImageMs += indirectionMs;
        _mixedTileUploads += snapshot.MixedTiles;
        _newTileUploads += snapshot.MixedTiles;

        Console.WriteLine(
            $"# atlas bulk rebuilt: {snapshot.MixedTiles} tiles in {snapshot.TilesPerAxis}³ "
            + $"({snapshot.AtlasTexels}×{snapshot.AtlasTexels}×{snapshot.AtlasDepthTexels} "
            + $"R16F = {snapshot.AtlasBytes / 1e6:F0} MB staged), "
            + $"gather={snapshot.GatherMs:F1} ms, upload={atlasMs:F1} ms");

        return new ViewerGpuRebuildMetrics(
            snapshot.MixedTiles,
            snapshot.MixedTiles,
            snapshot.MixedTiles,
            priorFrameTimer.Elapsed.TotalMilliseconds,
            snapshot.StateSetupMs,
            uploadTimer.Elapsed.TotalMilliseconds,
            submitMs,
            finishTimer.Elapsed.TotalMilliseconds,
            snapshot.GatherMs,
            atlasMs,
            indirectionMs,
            snapshot.StateSetupMs + snapshot.GatherMs + totalTimer.Elapsed.TotalMilliseconds,
            0,
            0);
    }

    private void CompletePendingSeekMeasurement()
    {
        ViewerSeekMeasurement? measurement = _pendingSeekMeasurement;
        if (measurement is null)
        {
            return;
        }

        var updatedFrameTimer = Stopwatch.StartNew();
        if (IsSeekBench)
        {
            _finish();
        }
        updatedFrameTimer.Stop();
        long frameReadyAt = Stopwatch.GetTimestamp();
        measurement.UpdatedFrameFinishMs = updatedFrameTimer.Elapsed.TotalMilliseconds;
        measurement.ApplyToFrameMs = Stopwatch.GetElapsedTime(measurement.AppliedAt, frameReadyAt).TotalMilliseconds;
        measurement.RequestToFrameMs = Stopwatch.GetElapsedTime(measurement.RequestedAt, frameReadyAt).TotalMilliseconds;
        measurement.FrameMs = _viewerLastFrameMs + measurement.UpdatedFrameFinishMs;
        _viewerLastFrameMs = measurement.FrameMs;
        _viewerLastSeekMs = measurement.RequestToFrameMs;
        Console.WriteLine(
            $"# seek timing: kind={measurement.Kind}, from={measurement.From}, step={measurement.Target}/{_liveSegs.Length}, "
            + $"dispatch={measurement.DispatchWaitMs:F1} ms, worker queue={measurement.WorkerQueueMs:F1} ms, "
            + $"CPU init={measurement.VolumeInitMs:F1} ms, CPU stamp={measurement.StampMs:F1} ms, CPU total={measurement.CpuTotalMs:F1} ms, "
            + $"prior frame drain={measurement.PriorFrameDrainMs:F1} ms, GPU state={measurement.GpuStateMs:F1} ms, "
            + $"GPU upload={measurement.GpuUploadMs:F1} ms (submit={measurement.UploadSubmitMs:F1}, finish={measurement.UploadFinishMs:F1}), GPU total={measurement.GpuTotalMs:F1} ms, "
            + $"gather={measurement.GatherMs:F1} ms, atlas calls={measurement.AtlasSubImageMs:F1} ms, indirection calls={measurement.IndirectionSubImageMs:F1} ms, "
            + $"atlas grows={measurement.AtlasGrowths}/{measurement.AtlasGrowthMs:F1} ms, "
            + $"updated draw={measurement.ApplyToFrameMs:F1} ms (finish={measurement.UpdatedFrameFinishMs:F1}), "
            + $"request-to-applied={measurement.RequestToAppliedMs:F1} ms, request-to-frame-ready={measurement.RequestToFrameMs:F1} ms, "
            + $"dirty={measurement.Uploaded}, mixed/new={measurement.MixedTiles}/{measurement.NewTiles}");

        if (IsSeekBench)
        {
            _seekBenchMeasurements.Add(measurement);
        }
        _pendingSeekMeasurement = null;
    }

    private void AdvanceSeekBenchmark()
    {
        if (!IsSeekBench || IsSeeking || _pendingSeekMeasurement is not null)
        {
            return;
        }

        int count = ToolpathSegmentCount;
        if (count <= 0)
        {
            return;
        }

        if (_seekBenchPhase == 0)
        {
            _seekBenchPhase = 1;
            int target = (int)((long)count * 3 / 5);
            Console.WriteLine($"# seek bench test 1/4: cold rebuild 0 -> {target} (60%)");
            SeekToStep(target);
            return;
        }

        if (_seekBenchPhase == 1 && _seekBenchMeasurements.Count >= 1)
        {
            _seekBenchPhase = 2;
            int target = Math.Min(count, ToolpathStep + 10_000);
            Console.WriteLine($"# seek bench test 2/4: nearby forward seek {ToolpathStep} -> {target}");
            SeekToStep(target);
            return;
        }

        if (_seekBenchPhase == 2 && _seekBenchMeasurements.Count >= 2)
        {
            _seekBenchPhase = 3;
            Console.WriteLine($"# seek bench test 3/4: large forward seek {ToolpathStep} -> {count}");
            SeekToStep(count);
            return;
        }

        if (_seekBenchPhase == 3 && _seekBenchMeasurements.Count >= 3)
        {
            _seekBenchPhase = 4;
            int target = count / 3;
            Console.WriteLine($"# seek bench test 4/4: large backward seek {ToolpathStep} -> {target}");
            SeekToStep(target);
            return;
        }

        if (_seekBenchPhase == 4 && _seekBenchMeasurements.Count >= 4)
        {
            _seekBenchPhase = 5;
            Console.WriteLine("# seek bench summary");
            foreach (ViewerSeekMeasurement measurement in _seekBenchMeasurements)
            {
                Console.WriteLine(
                    $"seek bench: {measurement.Kind,-7} {measurement.From,7} -> {measurement.Target,7} | "
                    + $"total {measurement.RequestToFrameMs,9:F1} ms | CPU {measurement.CpuTotalMs,9:F1} ms | "
                    + $"GPU {measurement.GpuTotalMs,8:F1} ms | draw {measurement.ApplyToFrameMs,7:F1} ms | "
                    + $"grows {measurement.AtlasGrowths}/{measurement.AtlasGrowthMs:F1} ms | "
                    + $"uploads {measurement.Uploaded}");
            }
            Console.Out.Flush();
            Environment.Exit(0);
        }
    }

    private void UploadCoarseVolume()
    {
        if (_vol is null)
        {
            return;
        }

        long sampleCount = (long)_vol.CoarseX * _vol.CoarseY * _vol.CoarseZ;
        ushort* coarse = (ushort*)NativeMemory.Alloc((nuint)(sampleCount * sizeof(ushort)));
        try
        {
            for (long i = 0; i < sampleCount; i++)
            {
                coarse[i] = BitConverter.HalfToUInt16Bits((Half)_vol.Coarse[i]);
            }
            _bindTexture(TEXTURE_3D, _coarseTex);
            _texImage3D(TEXTURE_3D, 0, R16F,
                _vol.CoarseX, _vol.CoarseY, _vol.CoarseZ, 0, RED, HALF_FLOAT, coarse);
            _bindTexture(TEXTURE_3D, _atlas16);
        }
        finally
        {
            NativeMemory.Free(coarse);
        }
    }

    private void SetToolPositionForCursor(int cursor)
    {
        if (_liveSegs.Length == 0)
        {
            return;
        }

        int clamped = Math.Clamp(cursor, 0, _liveSegs.Length);
        _toolPos = clamped == 0 ? _liveSegs[0].A : _liveSegs[clamped - 1].B;
    }

    private void SetupLiveMode()
    {
        using var timing = StartupTrace.Measure("live scene CPU setup");
        if (_vol is null)
        {
            return;
        }

        float stockX = _vol.StockSizeX;
        float stockY = _vol.StockSizeY;
        const int order = 11;
        using (StartupTrace.Measure("Hilbert toolpath generation"))
        {
            _liveSegs = HilbertPath.GenerateSegments(
            order, stockX, stockY, 8, _displayStockZ - 1.8f, 0.5f, 0.02f);
        }
        Volatile.Write(ref _liveSegmentCount, _liveSegs.Length);
        Volatile.Write(ref _liveCursor, 0);
        SetToolPositionForCursor(0);
        using (StartupTrace.Measure("tool profile LUT bake"))
        {
            _liveTool = CreateSceneToolProfile();
        }
        _liveDirty = new DirtyBrickSet(_brickToTile.Length);
        int uploadCap = IsViewer ? 1000 : 200;
        _liveDrain = new int[uploadCap];
        _liveFrameMs.Clear();
        _liveStamped = 0;
        Console.WriteLine($"# live mode: stock={stockX:F0}×{stockY:F0}×{_vol.StockHeight:F0} mm, order={order}, segments={_liveSegs.Length}, cap/frame=1167, uploadCap={uploadCap}, tool={_liveTool.ProfileStack?.Segments.Count} primitives/{_liveTool.Zmax:F2} mm/R{_liveTool.Rmax:F2}");
    }

    private void LiveFrame(int fb)
    {
        const int capPerFrame = 1167;
        const int targetFrames = 3600;

        if (_vol is null || _liveTool is null || _liveDirty is null)
        {
            throw new Exception("live mode not initialized");
        }

        var swFrame = Stopwatch.StartNew();
        int take = Math.Min(capPerFrame, _liveSegs.Length - _liveCursor);
        if (take > 0)
        {
            _vol.StampBatch(_liveSegs, _liveCursor, take, _liveTool,
                parallel: true, conservativeCull: false);
            _toolPos = _liveSegs[_liveCursor + take - 1].B;
            _liveCursor += take;
            _liveStamped += take;
            foreach (int b in _vol.DirtyBricks) _liveDirty.Mark(b);
            int n = _liveDirty.DrainTo(_liveDrain);
            UploadDirtyMappedBricks(_liveDrain, n);
        }

        RaymarchFrame(fb);

        swFrame.Stop();
        _liveFrameMs.Add(swFrame.Elapsed.TotalMilliseconds);

        if ((_liveFrameMs.Count % 60) == 0)
        {
            Console.WriteLine($"# live progress: frame={_liveFrameMs.Count} cursor={_liveCursor}/{_liveSegs.Length} lastMs={_liveFrameMs[^1]:F2}");
        }

        if (_liveFrameMs.Count >= targetFrames || _liveCursor >= _liveSegs.Length)
        {
            double sum = 0;
            double max = 0;
            foreach (double ms in _liveFrameMs)
            {
                sum += ms;
                if (ms > max) max = ms;
            }
            double avg = sum / _liveFrameMs.Count;
            bool pass = avg <= 16.6;
            Console.WriteLine($"live pipeline: frames={_liveFrameMs.Count}, stamped={_liveStamped}, avgFrame={avg:F2} ms maxFrame={max:F2} ms => {(pass ? "PASS" : "FAIL")}");
            Console.Out.Flush();
            Environment.Exit(pass ? 0 : 1);
        }
    }

    // ------------------------------------------------------------------ B4 raymarch
    private const string VS = """
        #version 300 es
        layout(location = 0) in vec2 aPos;
        void main() { gl_Position = vec4(aPos, 0.0, 1.0); }
        """;

    private const string FS = """
        #version 300 es
        precision highp float;
        precision highp sampler3D;
        uniform sampler3D uAtlas;   // R16F linear
        uniform sampler3D uIndir;   // RGBA8 nearest
        uniform sampler3D uCoarse;  // R16F linear
        uniform vec3 uEye, uFwd, uRight, uUp;
        uniform vec3 uStock, uVolume;
        uniform float uVox, uBand, uBrick, uTiles, uAtlasTexels, uRes, uResY;
        uniform vec3 uJit;
        uniform float uFlat;
        uniform float uSeamMode;
        uniform vec3 uSeamP0, uSeamP1;
        uniform float uShowTool;
        uniform vec3 uToolPos;
        uniform int uToolSegmentCount;
        uniform vec4 uToolSegments[8]; // axial offset, height, bottom radius, top radius
        uniform int uToolSegmentTypes[8];
        uniform float uToolHeight;
        uniform float uToolMaxR;
        uniform int uDebugView;
        out vec4 o;

        float stockAt(vec3 p) {
            vec3 boxQ = abs(p - 0.5 * uStock) - 0.5 * uStock;
            return length(max(boxQ, vec3(0.0)))
                + min(max(boxQ.x, max(boxQ.y, boxQ.z)), 0.0);
        }

        float voxelAt(ivec3 voxel) {
            ivec3 voxelCount = ivec3(floor(uVolume / uVox + 0.5));
            ivec3 vi = clamp(voxel, ivec3(0), voxelCount - ivec3(1));
            ivec3 brick = vi / 8;
            vec4 ind = texelFetch(uIndir, brick, 0);
            if (ind.a < 0.25) return uBand;
            if (ind.a < 0.75) return -uBand;

            ivec3 tile = ivec3(floor(ind.rgb * 255.0 + 0.5));
            ivec3 local = vi - brick * 8;
            return texelFetch(uAtlas, tile * 9 + local, 0).r;
        }

        // Exact atlas sampling: tiles hold voxels 0..8 of a brick (texel 8 duplicates the
        // +neighbor's first voxel plane). Regions are shifted by half a voxel so every
        // trilinear pair of a MIXED brick lies inside one tile. A uniform brick has no tile;
        // at its positive border, reconstruct explicitly from global voxel values so an
        // AIR/SOLID -> MIXED transition stays continuous instead of becoming a brick step.
        float fieldAt(vec3 p) {
            vec3 q = clamp(p, vec3(0.5 * uVox), uVolume - vec3(0.5 * uVox));
            vec3 vg = q / uVox - 0.5;              // global voxel-center coordinate
            vec3 cell = floor(vg * 0.125);         // half-voxel-shifted brick region
            vec3 local = vg - cell * 8.0;
            vec4 ind = texelFetch(uIndir, ivec3(cell), 0);
            float sampled;
            if (ind.a < 0.25) {
                sampled = uBand;
            } else if (ind.a < 0.75) {
                sampled = -uBand;
            } else {
                vec3 tile = floor(ind.rgb * 255.0 + 0.5);
                vec3 tc = (tile * 9.0 + (local + 0.5)) / uAtlasTexels;
                sampled = texture(uAtlas, tc).r;
            }

            if (ind.a < 0.75 && any(greaterThanEqual(local, vec3(7.0)))) {
                ivec3 v0 = ivec3(floor(vg));
                vec3 f = fract(vg);
                float c000 = voxelAt(v0);
                float c100 = voxelAt(v0 + ivec3(1, 0, 0));
                float c010 = voxelAt(v0 + ivec3(0, 1, 0));
                float c110 = voxelAt(v0 + ivec3(1, 1, 0));
                float c001 = voxelAt(v0 + ivec3(0, 0, 1));
                float c101 = voxelAt(v0 + ivec3(1, 0, 1));
                float c011 = voxelAt(v0 + ivec3(0, 1, 1));
                float c111 = voxelAt(v0 + ivec3(1, 1, 1));
                sampled = mix(
                    mix(mix(c000, c100, f.x), mix(c010, c110, f.x), f.y),
                    mix(mix(c001, c101, f.x), mix(c011, c111, f.x), f.y),
                    f.z);
            }

            // The texture exists only inside the stock, but post-hit reconstruction samples on
            // both sides of its boundary. Clamping alone extended material into infinity and
            // shifted the filtered root before t0, producing parallel raw/dark perimeter bands.
            return max(sampled, stockAt(p));        // intersection: remaining material ∩ stock
        }

        // A small isotropic reconstruction filter used only after the fast field has found the
        // surface.  Marching still uses the exact trilinear field; filtering refinement/normals
        // removes voxel-sized facets without multiplying the cost of every empty-space step.
        float surfaceAt(vec3 p) {
            float r = 0.38 * uVox;
            float tetra = 0.25 * (
                fieldAt(p + vec3( r,  r,  r)) +
                fieldAt(p + vec3( r, -r, -r)) +
                fieldAt(p + vec3(-r,  r, -r)) +
                fieldAt(p + vec3(-r, -r,  r)));
            // Keep a center contribution so sub-voxel/thin wall crossings survive the filter.
            // A pure tetra average erased those crossings and forced raw yellow fallback bands.
            return mix(fieldAt(p), tetra, 0.65);
        }

        vec3 heatColor(float x) {
            x = clamp(x, 0.0, 1.0);
            return clamp(vec3(
                1.5 - abs(4.0 * x - 3.0),
                1.5 - abs(4.0 * x - 2.0),
                1.5 - abs(4.0 * x - 1.0)), 0.0, 1.0);
        }

        float sq(float x) { return x * x; }

        float toolSegmentRadius(vec4 seg, int segmentType, float z) {
            float z0 = seg.x;
            float z1 = z0 + seg.y;
            float zc = clamp(z, z0, z1);
            if (segmentType == 2) {
                float dz = z1 - zc;
                return sqrt(max(0.0, seg.w * seg.w - dz * dz));
            }
            if (segmentType == 3) {
                float minorRadius = seg.w - seg.z;
                float dz = z1 - zc;
                return seg.z + sqrt(max(0.0, minorRadius * minorRadius - dz * dz));
            }
            float u = (zc - z0) / max(seg.y, 1e-6);
            return mix(seg.z, seg.w, u);
        }

        float toolOuterWallDistanceSquared(float r, float z, vec4 seg, int segmentType) {
            float z0 = seg.x;
            float z1 = z0 + seg.y;
            if (segmentType == 2) {
                float dz = z - z1;
                if (dz > 0.0) return sq(r - seg.w) + dz * dz;
                if (z < z0) return r * r + sq(z - z0);
                return sq(length(vec2(r, dz)) - seg.w);
            }
            if (segmentType == 3) {
                float dz = z - z1;
                float minorRadius = seg.w - seg.z;
                if (z > z1) return sq(r - seg.w) + dz * dz;
                if (z < z0) return sq(r - seg.z) + sq(z - z0);
                return sq(length(vec2(r - seg.z, dz)) - minorRadius);
            }

            vec2 p = vec2(r - seg.z, z - z0);
            vec2 edge = vec2(seg.w - seg.z, seg.y);
            float u = clamp(dot(p, edge) / max(dot(edge, edge), 1e-8), 0.0, 1.0);
            return dot(p - edge * u, p - edge * u);
        }

        // Exact signed distance to the closed radial profile used to bake the CPU carving LUT.
        float toolProfileAt(float r, float z) {
            if (uToolSegmentCount <= 0) return 1e6;

            float minDistanceSquared = 1e20;
            float insideRadius = 0.0;
            bool inAxialRange = z >= 0.0 && z <= uToolHeight;
            for (int i = 0; i < 8; i++) {
                if (i >= uToolSegmentCount) break;
                vec4 seg = uToolSegments[i];
                int segmentType = uToolSegmentTypes[i];
                if (inAxialRange && z >= seg.x && z <= seg.x + seg.y)
                    insideRadius = max(insideRadius, toolSegmentRadius(seg, segmentType, z));
                minDistanceSquared = min(minDistanceSquared,
                    toolOuterWallDistanceSquared(r, z, seg, segmentType));
            }

            // Annular shoulder faces at radius-discontinuous joins.
            for (int i = 0; i < 7; i++) {
                if (i + 1 >= uToolSegmentCount) break;
                vec4 current = uToolSegments[i];
                vec4 next = uToolSegments[i + 1];
                float r0 = min(current.w, next.z);
                float r1 = max(current.w, next.z);
                if (r1 - r0 > 0.001) {
                    float dr = r - clamp(r, r0, r1);
                    float dz = z - (current.x + current.y);
                    minDistanceSquared = min(minDistanceSquared, dr * dr + dz * dz);
                }
            }

            vec4 last = uToolSegments[uToolSegmentCount - 1];
            float topDr = r - clamp(r, 0.0, last.w);
            float topDz = z - uToolHeight;
            minDistanceSquared = min(minDistanceSquared, topDr * topDr + topDz * topDz);

            vec4 first = uToolSegments[0];
            if (uToolSegmentTypes[0] != 2 && first.z > 0.001) {
                float bottomDr = r - clamp(r, 0.0, first.z);
                minDistanceSquared = min(minDistanceSquared, bottomDr * bottomDr + z * z);
            }

            float d = sqrt(minDistanceSquared);
            return inAxialRange && r < insideRadius ? -d : d;
        }

        float toolAt(vec3 p) {
            vec3 q = p - uToolPos;
            return toolProfileAt(length(q.xy), q.z);
        }

        int toolSegmentAt(float axialHeight) {
            for (int i = 0; i < 8; i++) {
                if (i >= uToolSegmentCount) break;
                vec4 seg = uToolSegments[i];
                if (axialHeight <= seg.x + seg.y + 0.001) return i;
            }
            return max(uToolSegmentCount - 1, 0);
        }

        bool traceTool(vec3 eye, vec3 dir, out float hitT, out vec3 hitNormal,
                       out int hitSegment) {
            hitT = 1e30;
            hitNormal = vec3(0.0, 0.0, 1.0);
            hitSegment = 0;
            if (uShowTool < 0.5 || uToolSegmentCount <= 0) return false;

            vec3 safeDir = vec3(
                abs(dir.x) < 1e-6 ? (dir.x < 0.0 ? -1e-6 : 1e-6) : dir.x,
                abs(dir.y) < 1e-6 ? (dir.y < 0.0 ? -1e-6 : 1e-6) : dir.y,
                abs(dir.z) < 1e-6 ? (dir.z < 0.0 ? -1e-6 : 1e-6) : dir.z);
            vec3 boxMin = uToolPos + vec3(-uToolMaxR, -uToolMaxR, 0.0);
            vec3 boxMax = uToolPos + vec3( uToolMaxR,  uToolMaxR, uToolHeight);
            vec3 ta = (boxMin - eye) / safeDir;
            vec3 tb = (boxMax - eye) / safeDir;
            vec3 near3 = min(ta, tb);
            vec3 far3 = max(ta, tb);
            float nearT = max(max(near3.x, near3.y), near3.z);
            float farT = min(min(far3.x, far3.y), far3.z);
            if (nearT >= farT || farT <= 0.0) return false;

            float t = max(nearT, 0.0);
            float closest = 1e20;
            float closestT = t;
            for (int i = 0; i < 192; i++) {
                if (t > farT) break;
                float d = toolAt(eye + dir * t);
                if (d < closest) { closest = d; closestT = t; }
                if (d <= 0.01) {
                    hitT = t;
                    vec3 hp = eye + dir * hitT;
                    float e = max(0.02, 0.25 * uVox);
                    vec3 gradient = vec3(
                        toolAt(hp + vec3(e, 0.0, 0.0)) - toolAt(hp - vec3(e, 0.0, 0.0)),
                        toolAt(hp + vec3(0.0, e, 0.0)) - toolAt(hp - vec3(0.0, e, 0.0)),
                        toolAt(hp + vec3(0.0, 0.0, e)) - toolAt(hp - vec3(0.0, 0.0, e)));
                    hitNormal = length(gradient) > 1e-6 ? normalize(gradient) : -dir;
                    hitSegment = toolSegmentAt(hp.z - uToolPos.z);
                    return true;
                }
                t += max(d, 0.01);
            }

            // Recover a true tangent that approached to sub-pixel distance without crossing.
            if (closest < 0.04) {
                hitT = closestT;
                vec3 hp = eye + dir * hitT;
                float e = max(0.02, 0.25 * uVox);
                vec3 gradient = vec3(
                    toolAt(hp + vec3(e, 0.0, 0.0)) - toolAt(hp - vec3(e, 0.0, 0.0)),
                    toolAt(hp + vec3(0.0, e, 0.0)) - toolAt(hp - vec3(0.0, e, 0.0)),
                    toolAt(hp + vec3(0.0, 0.0, e)) - toolAt(hp - vec3(0.0, 0.0, e)));
                hitNormal = length(gradient) > 1e-6 ? normalize(gradient) : -dir;
                hitSegment = toolSegmentAt(hp.z - uToolPos.z);
                return true;
            }
            return false;
        }

        vec3 shadeTool(vec3 hp, vec3 nrm, int segmentIndex, vec3 viewDir) {
            // Cutting cone, collar, and shaft use distinct machine-metal finishes so the
            // physical joins remain legible at the true 103 mm scale.
            vec3 base = segmentIndex == 0 ? vec3(0.55, 0.40, 0.16)
                : segmentIndex == 1 ? vec3(0.23, 0.31, 0.34)
                : vec3(0.48, 0.56, 0.58);
            vec3 lightDir = normalize(vec3(0.5, -0.6, 0.8));
            float diffuse = max(dot(nrm, lightDir), 0.0);
            vec3 halfDir = normalize(lightDir - viewDir);
            float specular = pow(max(dot(nrm, halfDir), 0.0), 42.0);
            float rim = pow(1.0 - max(dot(nrm, -viewDir), 0.0), 3.0);
            return base * (0.28 + 0.72 * diffuse) + vec3(0.75) * specular + base * rim * 0.35;
        }

        void main() {
            if (uFlat > 0.5) { o = vec4(1.0, 0.0, 1.0, 1.0); return; }
            if (uSeamMode > 0.5) {
                float u = gl_FragCoord.x / uRes;
                vec3 p = mix(uSeamP0, uSeamP1, clamp(u, 0.0, 1.0));
                float d = fieldAt(p);
                float g = clamp(0.5 + 0.5 * d / 0.6, 0.0, 1.0);
                o = vec4(g, g, g, 1.0);
                return;
            }
            vec2 px = gl_FragCoord.xy;
            // NOTE: Avalonia composites the control FB with a top-left origin, i.e. the GL image
            // is presented vertically FLIPPED. Compensate here (px.y/uResY - 0.5, not 0.5 - ...).
            vec2 ndc = vec2(px.x / uRes - 0.5, px.y / uResY - 0.5);
            vec3 dir = normalize(uFwd + uRight * ndc.x * 1.1 * (uRes / uResY) + uUp * ndc.y * 1.1);
            vec3 eye = uEye + uJit;
            float toolHitT;
            vec3 toolHitNormal;
            int toolHitSegment;
            bool toolHit = traceTool(eye, dir, toolHitT, toolHitNormal, toolHitSegment);

            vec3 dirSafe = vec3(
                abs(dir.x) < 1e-6 ? (dir.x < 0.0 ? -1e-6 : 1e-6) : dir.x,
                abs(dir.y) < 1e-6 ? (dir.y < 0.0 ? -1e-6 : 1e-6) : dir.y,
                abs(dir.z) < 1e-6 ? (dir.z < 0.0 ? -1e-6 : 1e-6) : dir.z);
            vec3 inv = 1.0 / dirSafe;
            vec3 tA = (vec3(0.0) - eye) * inv, tB = (uStock - eye) * inv;
            vec3 tmin = min(tA, tB), tmax = max(tA, tB);
            float t0 = max(max(tmin.x, tmin.y), tmin.z);
            float t1 = min(min(tmax.x, tmax.y), tmax.z);
            if (t0 >= t1) {
                if (toolHit) {
                    vec3 toolPoint = eye + dir * toolHitT;
                    o = vec4(shadeTool(toolPoint, toolHitNormal, toolHitSegment, dir), 1.0);
                } else {
                    o = vec4(0.08, 0.09, 0.11, 1.0);
                }
                return;
            }

            float t = max(t0, 0.0) + 1e-4;
            bool hit = false;
            bool solidHit = false;
            bool grazingHit = false;
            bool filteredHit = false;
            bool atStockEntry = true;
            int filterReject = 0;
            int steps = 0;
            float tPrev = t;
            float closestD = uBand;
            float closestT = t;
            for (int i = 0; i < 384; i++) {
                if (t >= t1) break;
                steps++;
                vec3 p = eye + dir * t;
                float dc = texture(uCoarse, p / uVolume).r;
                if (dc > uBrick) { tPrev = t; t += dc - 0.6 * uBrick; continue; }
                vec4 ind = texture(uIndir, p / uVolume);
                if (ind.a < 0.25) {                     // AIR brick: jump to brick exit
                    atStockEntry = false;
                    vec3 cell = floor(p / uBrick);
                    vec3 exitP = vec3(
                        dirSafe.x >= 0.0 ? (cell.x + 1.0) * uBrick : cell.x * uBrick,
                        dirSafe.y >= 0.0 ? (cell.y + 1.0) * uBrick : cell.y * uBrick,
                        dirSafe.z >= 0.0 ? (cell.z + 1.0) * uBrick : cell.z * uBrick);
                    vec3 tt = (exitP - p) / dirSafe;
                    float tStep = min(min(tt.x, tt.y), tt.z);
                    tPrev = t;
                    t += max(tStep + 1e-4, 1e-4);
                    continue;
                }
                if (ind.a < 0.75) {
                    // The initial SOLID entry is the exact stock box. After an AIR jump,
                    // however, the continuous zero can lie up to half a voxel from the brick
                    // plane, so refine fieldAt instead of exposing that plane as geometry.
                    if (atStockEntry) { hit = true; solidHit = true; break; }
                    float d = fieldAt(p);
                    if (d < 0.0) { hit = true; break; }
                    tPrev = t;
                    t += max(d, 0.35 * uVox);
                    continue;
                }
                float d = fieldAt(p);                     // exact seam-free tile sample
                atStockEntry = false;
                if (d < closestD) { closestD = d; closestT = t; }
                if (d < 0.0) { hit = true; break; }
                tPrev = t;
                t += max(d, 0.35 * uVox);
            }
            // A true tangent can converge without crossing. Recover only sub-voxel pinholes;
            // the old 0.5*band test accepted points up to 1.5 voxels away and made thick dark halos.
            if (!hit && closestD < 0.18 * uVox) {
                hit = true;
                grazingHit = true;
                t = closestT;
            }
            if (!hit) {
                if (toolHit) {
                    vec3 toolPoint = eye + dir * toolHitT;
                    o = vec4(shadeTool(toolPoint, toolHitNormal, toolHitSegment, dir), 1.0);
                } else if (uDebugView == 1 && closestD < 0.999 * uBand) {
                    float nearMiss = 1.0 - clamp(closestD / uBand, 0.0, 1.0);
                    o = vec4(mix(vec3(0.08, 0.09, 0.11), vec3(1.0, 0.05, 0.02), nearMiss), 1.0);
                } else if (uDebugView == 2) {
                    float cost = clamp(log2(float(steps) + 1.0) / 8.0, 0.0, 1.0);
                    o = vec4(0.42 * heatColor(cost), 1.0);
                } else {
                    o = vec4(0.08, 0.09, 0.11, 1.0);
                }
                return;
            }

            float hitT = t;
            if (!solidHit && !grazingHit) {
                // First locate the exact trilinear crossing, then refine the locally filtered
                // crossing in a one-voxel interval. The latter is what removes the staircase
                // silhouette; falling back to the exact crossing preserves very thin features.
                float lo = tPrev, hi = t;
                for (int i = 0; i < 7; i++) {
                    float mid = 0.5 * (lo + hi);
                    if (fieldAt(eye + dir * mid) < 0.0) hi = mid; else lo = mid;
                }
                hitT = 0.5 * (lo + hi);
                float slo = max(t0, hitT - uVox);
                float shi = min(t1, hitT + uVox);
                float dlo = surfaceAt(eye + dir * slo);
                float dhi = surfaceAt(eye + dir * shi);
                if (dlo >= 0.0 && dhi <= 0.0) {
                    for (int i = 0; i < 6; i++) {
                        float mid = 0.5 * (slo + shi);
                        if (surfaceAt(eye + dir * mid) < 0.0) shi = mid; else slo = mid;
                    }
                    hitT = 0.5 * (slo + shi);
                    filteredHit = true;
                } else {
                    if (dlo < 0.0) filterReject += 1;  // smoothed crossing moved before interval
                    if (dhi > 0.0) filterReject += 2;  // crossing moved after/vanished in interval
                }
            }
            if (toolHit && toolHitT < hitT) {
                vec3 toolPoint = eye + dir * toolHitT;
                o = vec4(shadeTool(toolPoint, toolHitNormal, toolHitSegment, dir), 1.0);
                return;
            }
            vec3 hp = eye + dir * hitT;
            float h = 1.15 * uVox;
            // Uniform SOLID bricks have no atlas samples, but their visible entry surface is
            // the stock boundary. Derive its normal from the same box SDF used by fieldAt;
            // falling back to -dir made untouched stock view-facing while adjacent carved
            // bricks used geometric normals, producing a conspicuous color discontinuity.
            vec3 g = solidHit ? vec3(
                stockAt(hp + vec3(h, 0, 0)) - stockAt(hp - vec3(h, 0, 0)),
                stockAt(hp + vec3(0, h, 0)) - stockAt(hp - vec3(0, h, 0)),
                stockAt(hp + vec3(0, 0, h)) - stockAt(hp - vec3(0, 0, h)))
                : vec3(
                    surfaceAt(hp + vec3(h, 0, 0)) - surfaceAt(hp - vec3(h, 0, 0)),
                    surfaceAt(hp + vec3(0, h, 0)) - surfaceAt(hp - vec3(0, h, 0)),
                    surfaceAt(hp + vec3(0, 0, h)) - surfaceAt(hp - vec3(0, 0, h)));
            float gl = length(g);
            vec3 nrm = gl > 1e-6 ? (g / gl) : -dir;

            if (uDebugView == 1) {
                vec3 diagnostic = solidHit ? vec3(0.10, 0.38, 1.00)
                    : grazingHit ? vec3(1.00, 0.00, 0.85)
                    : gl <= 1e-6 ? vec3(0.00, 1.00, 1.00)
                    : filteredHit ? vec3(0.05, 0.95, 0.30)
                    : filterReject == 1 ? vec3(1.00, 0.35, 0.00)
                    : filterReject == 2 ? vec3(1.00, 0.78, 0.00)
                    : vec3(1.00);
                o = vec4(diagnostic, 1.0);
                return;
            }
            if (uDebugView == 2) {
                float cost = clamp(log2(float(steps) + 1.0) / 8.0, 0.0, 1.0);
                o = vec4(heatColor(cost), 1.0);
                return;
            }
            if (uDebugView == 3) {
                vec3 brickCoord = (hp / uVox - 0.5) * 0.125;
                vec3 brickId = floor(brickCoord);
                vec3 idColor = 0.25 + 0.65 * vec3(
                    fract(sin(dot(brickId, vec3(127.1, 311.7,  74.7))) * 43758.5453),
                    fract(sin(dot(brickId, vec3(269.5, 183.3, 246.1))) * 43758.5453),
                    fract(sin(dot(brickId, vec3(113.5, 271.9, 124.6))) * 43758.5453));
                vec3 phase = fract(brickCoord);
                vec3 edge3 = min(phase, 1.0 - phase);
                float edge = min(edge3.x, min(edge3.y, edge3.z));
                float line = 1.0 - smoothstep(0.025, 0.09, edge);
                o = vec4(mix(idColor, vec3(1.0), line), 1.0);
                return;
            }
            if (uDebugView == 4) {
                vec3 phase = fract(hp / uVox - 0.5);
                vec3 edge3 = min(phase, 1.0 - phase);
                float edge = min(edge3.x, min(edge3.y, edge3.z));
                float line = 1.0 - smoothstep(0.04, 0.12, edge);
                o = vec4(mix(0.12 + 0.76 * phase, vec3(1.0), line), 1.0);
                return;
            }
            if (uDebugView == 5) {
                float residual = clamp(fieldAt(hp) / (0.5 * uVox), -1.0, 1.0);
                vec3 zero = vec3(0.92);
                vec3 residualColor = residual < 0.0
                    ? mix(zero, vec3(0.03, 0.22, 1.0), -residual)
                    : mix(zero, vec3(1.0, 0.08, 0.02), residual);
                o = vec4(residualColor, 1.0);
                return;
            }
            if (uDebugView == 6) {
                o = vec4(0.5 + 0.5 * nrm, 1.0);
                return;
            }

            float ndl = max(dot(nrm, normalize(vec3(0.5, -0.6, 0.8))), 0.0);
            float cut = smoothstep(uStock.z - 2.2, uStock.z - 1.4, hp.z);    // crude cut-floor tint
            vec3 col = mix(vec3(0.75, 0.72, 0.68), vec3(0.85, 0.62, 0.35), 1.0 - cut);
            col *= 0.38 + 0.62 * ndl;
            o = vec4(col, 1.0);
        }
        """;

    private void BuildRaymarch()
    {
        using var timing = StartupTrace.Measure("raymarch shader setup");
        if (!TryLoadCachedProgram())
        {
            BuildRaymarchProgramFromSource();
        }

        Vector3 stock = new(_vol!.StockSizeX, _vol.StockSizeY, _vol.StockHeight);
        Vector3 volumeSize = new(_vol.VolumeSizeX, _vol.VolumeSizeY, _vol.VolumeSizeZ);
        float xySpan = MathF.Max(stock.X, stock.Y);
        _eye = new Vector3(stock.X * 0.5f, stock.Y * 0.5f + xySpan * 0.75f, _displayStockZ * 4.00f);
        var target = new Vector3(stock.X * 0.5f, stock.Y * 0.5f, _displayStockZ * 0.50f);
        _fwd = Vector3.Normalize(target - _eye);
        _right = Vector3.Normalize(Vector3.Cross(_fwd, Vector3.UnitZ));
        _up = Vector3.Cross(_right, _fwd);

        _useProgram(_prog);
        SetTex("uAtlas", 0, _atlas16);
        SetTex("uIndir", 1, _indir);
        SetTex("uCoarse", 2, _coarseTex);
        Set3("uStock", stock); Set3("uVolume", volumeSize); Set1("uVox", _vol.VoxelSize); Set1("uBand", _vol.BandWorld);
        Set1("uBrick", 8 * _vol.VoxelSize); Set1("uTiles", _tilesPerAxis); Set1("uAtlasTexels", _atlasTexels);
        Set3("uFwd", _fwd); Set3("uRight", _right); Set3("uUp", _up);
        _uEye = Loc("uEye"); _uJit = Loc("uJit");
        _uSeamMode = Loc("uSeamMode");
        _uShowTool = Loc("uShowTool");
        _uToolPos = Loc("uToolPos");
        _uToolSegmentCount = Loc("uToolSegmentCount");
        _uToolHeight = Loc("uToolHeight");
        _uToolMaxR = Loc("uToolMaxR");
        for (int i = 0; i < MaxToolSegments; i++)
        {
            _uToolSegments[i] = Loc($"uToolSegments[{i}]");
            _uToolSegmentTypes[i] = Loc($"uToolSegmentTypes[{i}]");
        }
        _uDebugView = Loc("uDebugView");
        _uniform3f(_uEye, _eye.X, _eye.Y, _eye.Z);
        _uniform1f(_uSeamMode, 0f);
        Set3("uSeamP0", Vector3.Zero);
        Set3("uSeamP1", Vector3.Zero);
        _uniform1f(_uShowTool, 0f);
        _uniform3f(_uToolPos, 0f, 0f, 0f);
        _uniform1i(_uToolSegmentCount, 0);
        _uniform1f(_uToolHeight, 0f);
        _uniform1f(_uToolMaxR, 0f);
        _uniform1i(_uDebugView, DebugView);
        _camNeedsFit = IsViewer;
    }

    private void BuildRaymarchProgramFromSource()
    {
        StartupTrace.SetShaderCacheStatus(LaunchOptions.ShaderCacheEnabled
            ? "cache miss"
            : "cache disabled");

        int vs;
        using (StartupTrace.Measure("vertex shader compilation"))
        {
            vs = Compile(VERTEX_SHADER, VS);
        }

        int fs;
        try
        {
            using (StartupTrace.Measure("fragment shader compilation"))
            {
                fs = Compile(FRAGMENT_SHADER, FS);
            }
        }
        catch
        {
            DeleteShader(vs);
            throw;
        }

        try
        {
            _prog = _createProgram();
            if (_programParameteri != null)
            {
                _programParameteri(_prog, PROGRAM_BINARY_RETRIEVABLE_HINT, 1);
            }
            _attachShader(_prog, vs);
            _attachShader(_prog, fs);
            using (StartupTrace.Measure("shader program link"))
            {
                _linkProgram(_prog);
                ValidateProgramLink(_prog, "link");
            }
        }
        finally
        {
            DeleteShader(vs);
            DeleteShader(fs);
        }

        TrySaveCachedProgram();
    }

    private bool TryLoadCachedProgram()
    {
        if (!LaunchOptions.ShaderCacheEnabled)
        {
            StartupTrace.SetShaderCacheStatus("cache disabled");
            StartupTrace.Record("shader binary cache disabled");
            return false;
        }
        if (LaunchOptions.RebuildShaderCache)
        {
            StartupTrace.SetShaderCacheStatus("cache refresh");
            StartupTrace.Record("shader binary cache refresh requested");
            return false;
        }
        if (_programBinary == null || _getProgramBinary == null || _programParameteri == null)
        {
            StartupTrace.SetShaderCacheStatus("cache unsupported");
            StartupTrace.Record("shader program binaries are unavailable on this OpenGL context");
            return false;
        }

        string path = GetShaderCachePath();
        if (!File.Exists(path))
        {
            StartupTrace.SetShaderCacheStatus("cache miss");
            StartupTrace.Record($"shader binary cache miss: {path}");
            return false;
        }

        using var timing = StartupTrace.Measure("shader binary cache load");
        try
        {
            using var stream = new FileStream(path, FileMode.Open, FileAccess.Read, FileShare.Read);
            using var reader = new BinaryReader(stream, Encoding.UTF8, leaveOpen: false);
            int magic = reader.ReadInt32();
            int version = reader.ReadInt32();
            int format = reader.ReadInt32();
            int length = reader.ReadInt32();
            if (magic != ShaderCacheMagic || version != ShaderCacheVersion
                || length <= 0 || length > MaxCachedProgramBytes
                || stream.Length - stream.Position != length)
            {
                throw new InvalidDataException("invalid shader cache header");
            }

            byte[] binary = reader.ReadBytes(length);
            int program = _createProgram();
            fixed (byte* data = binary)
            {
                _programBinary(program, format, data, binary.Length);
            }

            int linked;
            _getProgramiv(program, LINK_STATUS, &linked);
            if (linked == 0)
            {
                DeleteProgram(program);
                throw new InvalidDataException("driver rejected the cached program binary");
            }

            _prog = program;
            StartupTrace.SetShaderCacheStatus("cache hit");
            StartupTrace.Record($"shader binary cache hit ({binary.Length / 1024.0:F1} KiB)");
            return true;
        }
        catch (Exception ex)
        {
            StartupTrace.SetShaderCacheStatus("cache rejected");
            StartupTrace.Record($"shader binary cache rejected: {ex.Message}");
            TryDeleteCacheFile(path);
            return false;
        }
    }

    private void TrySaveCachedProgram()
    {
        if (!LaunchOptions.ShaderCacheEnabled || _getProgramBinary == null
            || _programBinary == null || _programParameteri == null)
        {
            return;
        }

        using var timing = StartupTrace.Measure("shader binary cache save");
        string? temporaryPath = null;
        try
        {
            int capacity;
            _getProgramiv(_prog, PROGRAM_BINARY_LENGTH, &capacity);
            if (capacity <= 0 || capacity > MaxCachedProgramBytes)
            {
                StartupTrace.Record($"driver returned an invalid program binary size: {capacity}");
                return;
            }

            byte[] binary = new byte[capacity];
            int length;
            int format;
            fixed (byte* data = binary)
            {
                _getProgramBinary(_prog, capacity, &length, &format, data);
            }
            if (length <= 0 || length > capacity)
            {
                StartupTrace.Record($"driver returned an invalid program binary length: {length}");
                return;
            }

            string path = GetShaderCachePath();
            Directory.CreateDirectory(Path.GetDirectoryName(path)!);
            temporaryPath = $"{path}.{Environment.ProcessId}.tmp";
            using (var stream = new FileStream(temporaryPath, FileMode.Create, FileAccess.Write,
                       FileShare.None, 64 * 1024, FileOptions.WriteThrough))
            using (var writer = new BinaryWriter(stream, Encoding.UTF8, leaveOpen: false))
            {
                writer.Write(ShaderCacheMagic);
                writer.Write(ShaderCacheVersion);
                writer.Write(format);
                writer.Write(length);
                writer.Write(binary, 0, length);
            }
            File.Move(temporaryPath, path, overwrite: true);
            temporaryPath = null;
            StartupTrace.Record($"shader binary cached ({length / 1024.0:F1} KiB): {path}");
        }
        catch (Exception ex)
        {
            StartupTrace.Record($"shader binary cache write skipped: {ex.Message}");
        }
        finally
        {
            if (temporaryPath is not null)
            {
                TryDeleteCacheFile(temporaryPath);
            }
        }
    }

    private string GetShaderCachePath()
    {
        string identity = string.Join('\n',
            $"rw-csg-shader-cache-v{ShaderCacheVersion}",
            _glVendor,
            _glRenderer,
            _glVersion,
            _glslVersion,
            RuntimeInformation.ProcessArchitecture.ToString(),
            VS,
            FS);
        string hash = Convert.ToHexString(SHA256.HashData(Encoding.UTF8.GetBytes(identity)));
        string root = Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData);
        if (string.IsNullOrWhiteSpace(root))
        {
            root = Path.GetTempPath();
        }
        return Path.Combine(root, "RW.CSG", "ShaderCache", $"raymarch-{hash}.glbin");
    }

    private void ValidateProgramLink(int program, string operation)
    {
        int ok;
        _getProgramiv(program, LINK_STATUS, &ok);
        if (ok != 0)
        {
            return;
        }

        byte* log = stackalloc byte[4096];
        int length;
        _getProgramInfoLog(program, 4096, &length, log);
        throw new Exception(operation + ": " + Marshal.PtrToStringAnsi((IntPtr)log, length));
    }

    private void DeleteShader(int shader)
    {
        if (shader != 0 && _deleteShader != null)
        {
            _deleteShader(shader);
        }
    }

    private void DeleteProgram(int program)
    {
        if (program != 0 && _deleteProgram != null)
        {
            _deleteProgram(program);
        }
    }

    private static void TryDeleteCacheFile(string path)
    {
        try
        {
            File.Delete(path);
        }
        catch
        {
            // A stale cache is harmless; the source compiler remains the fallback.
        }
    }

    private string GetGlString(int name)
    {
        byte* value = _getString(name);
        return value == null ? "unknown" : Marshal.PtrToStringAnsi((IntPtr)value) ?? "unknown";
    }

    private void UploadToolProfile(ToolProfile tool)
    {
        SweptVolumeStack? stack = tool.ProfileStack;
        if (stack is null)
            throw new InvalidOperationException("The visible machine tool requires a stacked primitive profile.");
        if (stack.Segments.Count > MaxToolSegments)
            throw new InvalidOperationException($"The renderer supports at most {MaxToolSegments} tool primitives.");

        _useProgram(_prog);
        _uniform1i(_uToolSegmentCount, stack.Segments.Count);
        _uniform1f(_uToolHeight, stack.TotalHeight);
        _uniform1f(_uToolMaxR, stack.MaxRadius);
        for (int i = 0; i < MaxToolSegments; i++)
        {
            if (i < stack.Segments.Count)
            {
                SweptVolumeSegment segment = stack.Segments[i];
                _uniform4f(_uToolSegments[i], segment.AxialOffset, segment.Height,
                    segment.BottomRadius, segment.TopRadius);
                _uniform1i(_uToolSegmentTypes[i], (int)segment.Type);
            }
            else
            {
                _uniform4f(_uToolSegments[i], 0f, 0f, 0f, 0f);
                _uniform1i(_uToolSegmentTypes[i], 0);
            }
        }
    }

    /// <summary>FBOs and VAOs are container objects: NOT shared across EGL share contexts.
    /// Avalonia inits and renders on different contexts, so these must be created lazily on
    /// the RENDER context (measured: an init-context FBO binds "successfully" but all writes
    /// vanish — the id names a different object).</summary>
    private void EnsureRenderObjects()
    {
        if (_vao != 0) return;
        const int ARRAY_BUFFER = 0x8892, STATIC_DRAW = 0x88E4, FLOAT = 0x1406;
        _pixelStorei(UNPACK_ALIGNMENT, 1);   // render context ≠ init context: don't inherit assumptions
        uint vao, vbo;
        _genVertexArrays(1, &vao);
        _bindVertexArray((int)vao);
        _genBuffers(1, &vbo);
        _bindBuffer(ARRAY_BUFFER, (int)vbo);
        float* verts = stackalloc float[6] { -1, -1, 3, -1, -1, 3 };
        _bufferData(ARRAY_BUFFER, 6 * sizeof(float), verts, STATIC_DRAW);
        _enableVertexAttribArray(0);
        _vertexAttribPointer(0, 2, FLOAT, 0, 0, null);
        _vao = (int)vao;
        Console.WriteLine($"# render-context objects created: vao={_vao} err={_getError():X}");
    }

    private void RaymarchFrame(int fb)
    {
        const int warm = 20, timed = 100;
        int kPasses = IsLive || IsViewer ? 1 : 4;
        const int VERTEX_ARRAY_BINDING = 0x85B5;
        int prevVao;
        _getIntegerv(VERTEX_ARRAY_BINDING, &prevVao);
        EnsureRenderObjects();
        NeutralizeState();
        _useProgram(_prog);
        _bindVertexArray(_vao);
        _activeTexture(TEXTURE0 + 0); _bindTexture(TEXTURE_3D, _atlas16);
        _activeTexture(TEXTURE0 + 1); _bindTexture(TEXTURE_3D, _indir);
        _activeTexture(TEXTURE0 + 2); _bindTexture(TEXTURE_3D, _coarseTex);
        // Draw into AVALONIA's control fb: user-created FBOs black-hole under this ANGLE setup
        // (writes vanish silently, no errors — see repo memory); the real Phase-2 renderer
        // draws into the control fb anyway, so this is also the representative path.
        double scale = (VisualRoot?.RenderScaling ?? 1.0);
        int w = Math.Max(64, (int)(Bounds.Width * scale));
        int h = Math.Max(64, (int)(Bounds.Height * scale));
        _lastRenderWidth = w;
        _lastRenderHeight = h;
        _bindFramebuffer(FRAMEBUFFER, fb);
        _viewport(0, 0, w, h);
        if (_frame == 0)
        {
            Console.WriteLine($"# rendering at {w}x{h} into control fb={fb}");
        }
        _uniform1f(_uSeamMode, 0f);
        _uniform1i(_uDebugView, DebugView);
        Set1("uRes", w); Set1("uResY", h);
        if (IsViewer)
        {
            _uniform1f(_uShowTool, 1f);
            _uniform3f(_uToolPos, _toolPos.X, _toolPos.Y, _toolPos.Z);
        }
        else
        {
            _uniform1f(_uShowTool, 0f);
        }

        if (IsViewer)
        {
            if (_camNeedsFit) FitViewerCameraToScreen(w, h);
            UpdateViewerCamera(w / (float)Math.Max(1, h));
            Set3("uFwd", _fwd); Set3("uRight", _right); Set3("uUp", _up);
            _uniform3f(_uEye, _eye.X, _eye.Y, _eye.Z);
        }

        if (!IsLive && !IsViewer) _finish();
        _sw.Restart();
        for (int k = 0; k < kPasses; k++)
        {
            _uniform3f(_uJit, 0.001f * k, 0.0013f * k, 0);
            _drawArrays(TRIANGLES, 0, 3);
        }
        if (!IsLive && !IsViewer) _finish();
        _sw.Stop();

        _frame++;
        if (!IsViewer && _frame > warm) _times.Add(_sw.Elapsed.TotalMilliseconds / kPasses);
        if (_frame == warm)   // one-time sanity readback (from the control fb we just drew)
        {
            var pix = stackalloc byte[4 * 16];
            _readPixels(w / 2 - 2, h / 2 - 2, 4, 4, RGBA, UNSIGNED_BYTE, pix);
            int lit = 0;
            for (int i = 0; i < 16; i++)
                if (Math.Max(pix[i * 4], Math.Max(pix[i * 4 + 1], pix[i * 4 + 2])) > 40) lit++;
            Console.WriteLine($"# sanity: {lit}/16 centre pixels lit (expect >0), first px=({pix[0]},{pix[1]},{pix[2]}) glGetError={_getError():X}");
        }
        if (!IsViewer && !IsLive && !IsSeam && _frame >= warm + timed)
        {
            _times.Sort();
            double avg = 0; foreach (var t in _times) avg += t;
            avg /= _times.Count;
            double mpx = w * (double)h / 1e6;
            double at1080 = avg * (1920.0 * 1080 / 1e6) / mpx;
            Console.WriteLine($"raymarch {w}x{h} | avg {avg:F2} ms/pass  p95 {_times[(int)(_times.Count * 0.95)]:F2} | {avg / mpx:F2} ms/Mpx → est {at1080:F2} ms @1080p ({1000 / at1080:F0} FPS) | 60fps budget: {(at1080 <= 16.6 ? "OK" : "OVER")}");
            Console.Out.Flush();
            Environment.Exit(0);
        }
        _bindVertexArray(prevVao);
    }

    private void TryCaptureSnapshot(int fb)
    {
        if (_snapshotCaptured || string.IsNullOrWhiteSpace(_snapshotPath) || _vol is null)
        {
            return;
        }

        if (!OperatingSystem.IsWindowsVersionAtLeast(6, 1))
        {
            _snapshotCaptured = true;
            Console.WriteLine("# snapshot skipped: PNG capture currently requires Windows 7 or later");
            return;
        }

        if (_frame < _snapshotWaitFrames && _liveCursor < _liveSegs.Length)
        {
            return;
        }

        int w = _lastRenderWidth;
        int h = _lastRenderHeight;
        if (w <= 0 || h <= 0)
        {
            return;
        }

        _finish();
        byte[] pixels = new byte[w * h * 4];
        fixed (byte* p = pixels)
        {
            _bindFramebuffer(FRAMEBUFFER, fb);
            _readPixels(0, 0, w, h, RGBA, UNSIGNED_BYTE, p);
        }

        string path = Path.GetFullPath(_snapshotPath);
        Directory.CreateDirectory(Path.GetDirectoryName(path) ?? ".");

        using var bitmap = new Bitmap(w, h, PixelFormat.Format32bppArgb);
        var rect = new Rectangle(0, 0, w, h);
        var data = bitmap.LockBits(rect, ImageLockMode.WriteOnly, PixelFormat.Format32bppArgb);
        try
        {
            unsafe
            {
                byte* dstBase = (byte*)data.Scan0;
                int stride = data.Stride;
                fixed (byte* srcBase = pixels)
                {
                    for (int y = 0; y < h; y++)
                    {
                        byte* srcRow = srcBase + (h - 1 - y) * w * 4;
                        byte* dstRow = dstBase + y * stride;
                        for (int x = 0; x < w; x++)
                        {
                            byte r = srcRow[x * 4 + 0];
                            byte g = srcRow[x * 4 + 1];
                            byte b = srcRow[x * 4 + 2];
                            byte a = srcRow[x * 4 + 3];
                            dstRow[x * 4 + 0] = b;
                            dstRow[x * 4 + 1] = g;
                            dstRow[x * 4 + 2] = r;
                            dstRow[x * 4 + 3] = a;
                        }
                    }
                }
            }
        }
        finally
        {
            bitmap.UnlockBits(data);
        }

        bitmap.Save(path, ImageFormat.Png);
        _snapshotCaptured = true;
        Console.WriteLine($"# snapshot saved: {path}");
        Environment.Exit(0);
    }

    private void SeamFrame(int fb)
    {
        const int warm = 20, timed = 60;
        const int VERTEX_ARRAY_BINDING = 0x85B5;
        int prevVao;
        _getIntegerv(VERTEX_ARRAY_BINDING, &prevVao);
        EnsureRenderObjects();
        NeutralizeState();
        _useProgram(_prog);
        _bindVertexArray(_vao);
        _activeTexture(TEXTURE0 + 0); _bindTexture(TEXTURE_3D, _atlas16);
        _activeTexture(TEXTURE0 + 1); _bindTexture(TEXTURE_3D, _indir);
        _activeTexture(TEXTURE0 + 2); _bindTexture(TEXTURE_3D, _coarseTex);

        double scale = (VisualRoot?.RenderScaling ?? 1.0);
        int w = Math.Max(256, (int)(Bounds.Width * scale));
        int h = Math.Max(128, (int)(Bounds.Height * scale));
        _bindFramebuffer(FRAMEBUFFER, fb);
        _viewport(0, 0, w, h);
        Set1("uRes", w);
        Set1("uResY", h);
        _uniform1f(_uSeamMode, 1f);
        Set3("uSeamP0", _seamP0);
        Set3("uSeamP1", _seamP1);

        if (_frame == warm && !_seamUpdated)
        {
            ApplySeamCarveUpdate();
        }

        _finish();
        _sw.Restart();
        _drawArrays(TRIANGLES, 0, 3);
        _finish();
        _sw.Stop();

        byte[] row = new byte[w * 4];
        fixed (byte* rp = row)
        {
            _readPixels(0, h / 2, w, 1, RGBA, UNSIGNED_BYTE, rp);
        }

        double maxGpuJump = 0;
        double maxCpuDiff = 0;
        if (_vol is not null)
        {
            for (int x = 1; x < w; x++)
            {
                double g0 = row[(x - 1) * 4] / 255.0;
                double g1 = row[x * 4] / 255.0;
                double jump = Math.Abs(g1 - g0);
                if (jump > maxGpuJump) maxGpuJump = jump;
            }

            for (int x = 0; x < w; x++)
            {
                float t = x / (float)Math.Max(1, w - 1);
                Vector3 p = Vector3.Lerp(_seamP0, _seamP1, t);
                float d = _vol.SampleFine(p.X, p.Y, p.Z);
                double cpuGray = Math.Clamp(0.5 + 0.5 * d / 0.6, 0.0, 1.0);
                double gpuGray = row[x * 4] / 255.0;
                double diff = Math.Abs(gpuGray - cpuGray);
                if (diff > maxCpuDiff) maxCpuDiff = diff;
            }
        }

        _frame++;
        if (_frame > warm)
        {
            _times.Add(maxGpuJump);
        }

        if (_frame >= warm + timed)
        {
            _times.Sort();
            double p95Jump = _times[(int)(_times.Count * 0.95)];
            bool pass = p95Jump < 0.06 && maxCpuDiff < 0.08;
            Console.WriteLine($"seam readback: p95 adjacent-jump={p95Jump:F4}, max cpu-vs-gpu gray diff={maxCpuDiff:F4} => {(pass ? "PASS" : "FAIL")}");
            Console.Out.Flush();
            Environment.Exit(pass ? 0 : 1);
        }

        _bindVertexArray(prevVao);
    }

    // ------------------------------------------------------------------ GL helpers
    private int Compile(int kind, string src)
    {
        int sh = _createShader(kind);
        var bytes = System.Text.Encoding.UTF8.GetBytes(src);
        fixed (byte* b = bytes)
        {
            byte* pb = b; int len = bytes.Length;
            _shaderSource(sh, 1, &pb, &len);
        }
        _compileShader(sh);
        int ok;
        _getShaderiv(sh, COMPILE_STATUS, &ok);
        if (ok == 0)
        {
            byte* log = stackalloc byte[4096]; int ll;
            _getShaderInfoLog(sh, 4096, &ll, log);
            throw new Exception("compile: " + Marshal.PtrToStringAnsi((IntPtr)log, ll));
        }
        return sh;
    }

    private int Loc(string name)
    {
        var b = System.Text.Encoding.ASCII.GetBytes(name + "\0");
        fixed (byte* p = b) return _getUniformLocation(_prog, p);
    }

    private void Set1(string n, float v) => _uniform1f(Loc(n), v);
    private void Set3(string n, Vector3 v) => _uniform3f(Loc(n), v.X, v.Y, v.Z);
    private void SetTex(string n, int unit, int tex) => _uniform1i(Loc(n), unit);
}
