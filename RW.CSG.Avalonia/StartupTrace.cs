using System;
using System.Diagnostics;
using System.Threading;

namespace RW.CSG.Avalonia;

/// <summary>
/// Lightweight wall-clock startup tracing. Messages go to both the console and diagnostic
/// listeners, while the latest stage is exposed to the in-window loading HUD.
/// </summary>
internal static class StartupTrace
{
    private static readonly object Gate = new();
    private static long _startedAt;
    private static long _firstFrameAt;
    private static string _currentStage = "process entry";
    private static string _shaderCache = "not checked";

    public static void Start()
    {
        lock (Gate)
        {
            _startedAt = Stopwatch.GetTimestamp();
            _firstFrameAt = 0;
            _currentStage = "process entry";
            _shaderCache = "not checked";
        }
        Write("START process");
    }

    public static double ElapsedMilliseconds
    {
        get
        {
            EnsureStarted();
            return Stopwatch.GetElapsedTime(Volatile.Read(ref _startedAt)).TotalMilliseconds;
        }
    }

    public static string CurrentStage
    {
        get
        {
            lock (Gate)
            {
                return _currentStage;
            }
        }
    }

    public static string HudText
    {
        get
        {
            long firstFrame = Volatile.Read(ref _firstFrameAt);
            if (firstFrame == 0)
            {
                return $"initializing {ElapsedMilliseconds / 1000.0:F1}s | {CurrentStage}";
            }

            double startupMs = Stopwatch.GetElapsedTime(Volatile.Read(ref _startedAt), firstFrame)
                .TotalMilliseconds;
            string cache;
            lock (Gate)
            {
                cache = _shaderCache;
            }
            return $"startup {startupMs / 1000.0:F2}s | shader {cache}";
        }
    }

    public static IDisposable Measure(string stage)
    {
        EnsureStarted();
        lock (Gate)
        {
            _currentStage = stage;
        }
        Write($"BEGIN {stage}");
        return new TimedStage(stage);
    }

    public static void Mark(string message)
    {
        EnsureStarted();
        lock (Gate)
        {
            _currentStage = message;
        }
        Write(message);
    }

    public static void Record(string message)
    {
        EnsureStarted();
        Write(message);
    }

    public static void SetShaderCacheStatus(string status)
    {
        lock (Gate)
        {
            _shaderCache = status;
        }
    }

    public static void CompleteFirstFrame()
    {
        EnsureStarted();
        long now = Stopwatch.GetTimestamp();
        if (Interlocked.CompareExchange(ref _firstFrameAt, now, 0) == 0)
        {
            lock (Gate)
            {
                _currentStage = "ready";
            }
            Write("READY first frame submitted");
        }
    }

    private static void EnsureStarted()
    {
        if (Volatile.Read(ref _startedAt) != 0)
        {
            return;
        }

        long now = Stopwatch.GetTimestamp();
        Interlocked.CompareExchange(ref _startedAt, now, 0);
    }

    private static void Write(string message)
    {
        long startedAt = Volatile.Read(ref _startedAt);
        double elapsedMs = startedAt == 0
            ? 0
            : Stopwatch.GetElapsedTime(startedAt).TotalMilliseconds;
        string line = $"# startup +{elapsedMs,10:F1} ms [T{Environment.CurrentManagedThreadId}] {message}";
        Console.WriteLine(line);
        Trace.WriteLine(line);
    }

    private sealed class TimedStage(string stage) : IDisposable
    {
        private readonly long _started = Stopwatch.GetTimestamp();
        private int _disposed;

        public void Dispose()
        {
            if (Interlocked.Exchange(ref _disposed, 1) != 0)
            {
                return;
            }

            double elapsedMs = Stopwatch.GetElapsedTime(_started).TotalMilliseconds;
            Write($"END   {stage} ({elapsedMs:F1} ms)");
        }
    }
}