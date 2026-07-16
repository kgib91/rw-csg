using Avalonia;
using Avalonia.Controls.ApplicationLifetimes;
using Avalonia.Markup.Xaml;

namespace RW.CSG.Avalonia;

public partial class App : Application
{
    public override void Initialize()
    {
        using var timing = StartupTrace.Measure("Avalonia XAML initialization");
        AvaloniaXamlLoader.Load(this);
    }

    public override void OnFrameworkInitializationCompleted()
    {
        using var timing = StartupTrace.Measure("framework initialization");
        if (ApplicationLifetime is IClassicDesktopStyleApplicationLifetime desktop)
        {
            desktop.MainWindow = LaunchOptions.DiagnosticsMode is { } mode
                ? new GlDiagnosticsWindow(mode)
                : new MainWindow();
        }

        base.OnFrameworkInitializationCompleted();
    }
}