using Avalonia;
using System;
using System.Collections.Generic;

namespace RW.CSG.Avalonia;

class Program
{
    [STAThread]
    public static void Main(string[] args)
    {
        StartupTrace.Start();
        for (var i = 0; i < args.Length; i++)
        {
            if (string.Equals(args[i], "--bench", StringComparison.OrdinalIgnoreCase) && i + 1 < args.Length)
            {
                var name = args[i + 1].Trim();
                if (name is "upload" or "raymarch" or "seam" or "live" or "seek")
                {
                    LaunchOptions.DiagnosticsMode = name;   // needs a real GL context → run the app shell
                    break;
                }
                Environment.Exit(Benches.Run(name));
            }
        }
        ParseLaunchOptions(args);
        StartupTrace.Mark("starting Avalonia desktop lifetime");
        BuildAvaloniaApp().StartWithClassicDesktopLifetime(args);
    }

    private static void ParseLaunchOptions(string[] args)
    {
        for (var i = 0; i < args.Length; i++)
        {
            if (string.Equals(args[i], "--graphics-backend", StringComparison.OrdinalIgnoreCase) && i + 1 < args.Length)
            {
                LaunchOptions.GraphicsBackend = args[++i].Trim();
            }
            else if (string.Equals(args[i], "--snapshot", StringComparison.OrdinalIgnoreCase) && i + 1 < args.Length)
            {
                LaunchOptions.SnapshotPath = args[++i].Trim();
            }
            else if (string.Equals(args[i], "--snapshot-wait", StringComparison.OrdinalIgnoreCase) && i + 1 < args.Length)
            {
                if (int.TryParse(args[++i].Trim(), out int waitFrames) && waitFrames >= 0)
                {
                    LaunchOptions.SnapshotWaitFrames = waitFrames;
                }
            }
            else if (string.Equals(args[i], "--debug-view", StringComparison.OrdinalIgnoreCase) && i + 1 < args.Length)
            {
                if (int.TryParse(args[++i].Trim(), out int debugView))
                {
                    LaunchOptions.DebugView = Math.Clamp(debugView, 0, 6);
                }
            }
            else if (string.Equals(args[i], "--no-shader-cache", StringComparison.OrdinalIgnoreCase))
            {
                LaunchOptions.ShaderCacheEnabled = false;
            }
            else if (string.Equals(args[i], "--rebuild-shader-cache", StringComparison.OrdinalIgnoreCase))
            {
                LaunchOptions.RebuildShaderCache = true;
            }
        }
    }

    // Avalonia configuration, don't remove; also used by visual designer.
    public static AppBuilder BuildAvaloniaApp()
        => AppBuilder.Configure<App>()
            .UsePlatformDetect()
            .With(new Win32PlatformOptions
            {
                RenderingMode = ResolveWin32RenderingModes(LaunchOptions.GraphicsBackend)
            })
            .WithInterFont()
            .LogToTrace();

    private static IReadOnlyList<Win32RenderingMode> ResolveWin32RenderingModes(string backend)
    {
        if (string.Equals(backend, "wgl", StringComparison.OrdinalIgnoreCase)
            || string.Equals(backend, "opengl", StringComparison.OrdinalIgnoreCase)
            || string.Equals(backend, "desktop", StringComparison.OrdinalIgnoreCase))
        {
            return [Win32RenderingMode.Wgl, Win32RenderingMode.AngleEgl, Win32RenderingMode.Software];
        }

        if (string.Equals(backend, "vulkan", StringComparison.OrdinalIgnoreCase))
        {
            return [Win32RenderingMode.Vulkan, Win32RenderingMode.Software];
        }

        // Default: ANGLE (GL ES 3.0). WGL desktop GL had intermittent FBO failures on some drivers.
        return [Win32RenderingMode.AngleEgl, Win32RenderingMode.Software];
    }
}
