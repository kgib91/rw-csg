using System;
using Avalonia;
using Avalonia.Controls;
using Avalonia.Input;
using Avalonia.Interactivity;
using Avalonia.Layout;
using Avalonia.Media;
using Avalonia.Threading;
using Avalonia.VisualTree;

namespace RW.CSG.Avalonia;

public partial class MainWindow : Window
{
    private readonly GlSceneControl _scene;
    private readonly TextBlock _hud;
    private readonly Button _playPause;
    private readonly ComboBox _debugView;
    private readonly TextBlock _debugLegend;
    private readonly TextBlock _stepValue;

    public MainWindow()
    {
        using var timing = StartupTrace.Measure("main window construction");
        InitializeComponent();

        _scene = new GlSceneControl("viewer");
        _hud = new TextBlock
        {
            Foreground = Brushes.White,
            FontSize = 13,
            Margin = new Thickness(10, 54, 10, 0),
            HorizontalAlignment = HorizontalAlignment.Right,
            VerticalAlignment = VerticalAlignment.Top,
            Text = "initializing..."
        };
        _playPause = new Button
        {
            Content = "Pause",
            MinWidth = 90,
            Margin = new Thickness(0, 0, 8, 0)
        };
        _playPause.Click += (_, _) =>
        {
            _scene.TogglePause();
            _playPause.Content = _scene.IsPaused ? "Play" : "Pause";
        };

        var stepLabel = new TextBlock
        {
            Text = "Toolpath step",
            Foreground = Brushes.White,
            VerticalAlignment = VerticalAlignment.Center,
            Margin = new Thickness(0, 0, 8, 0)
        };
        bool updatingStepSlider = false;
        bool scrubbingStepSlider = false;
        var stepSlider = new Slider
        {
            Minimum = 0,
            Maximum = 1,
            Value = 0,
            SmallChange = 1,
            LargeChange = 10_000,
            Width = 320,
            IsEnabled = false
        };
        stepSlider.PropertyChanged += (_, e) =>
        {
            if (!updatingStepSlider && scrubbingStepSlider && stepSlider.IsEnabled
                && e.Property == Slider.ValueProperty)
                _scene.SeekToStep((int)Math.Round(stepSlider.Value));
        };
        stepSlider.AddHandler(PointerPressedEvent, (object? _, PointerPressedEventArgs _) =>
        {
            scrubbingStepSlider = true;
        }, RoutingStrategies.Tunnel);
        stepSlider.AddHandler(PointerReleasedEvent, (object? _, PointerReleasedEventArgs _) =>
        {
            if (scrubbingStepSlider && stepSlider.IsEnabled)
                _scene.SeekToStep((int)Math.Round(stepSlider.Value));
            scrubbingStepSlider = false;
        }, RoutingStrategies.Tunnel);
        stepSlider.AddHandler(KeyDownEvent, (object? _, KeyEventArgs e) =>
        {
            if (!stepSlider.IsEnabled)
                return;

            double? value = e.Key switch
            {
                Key.Left or Key.Down => stepSlider.Value - stepSlider.SmallChange,
                Key.Right or Key.Up => stepSlider.Value + stepSlider.SmallChange,
                Key.PageDown => stepSlider.Value - stepSlider.LargeChange,
                Key.PageUp => stepSlider.Value + stepSlider.LargeChange,
                Key.Home => stepSlider.Minimum,
                Key.End => stepSlider.Maximum,
                _ => null
            };
            if (value is null)
                return;

            stepSlider.Value = Math.Clamp(value.Value, stepSlider.Minimum, stepSlider.Maximum);
            _scene.SeekToStep((int)Math.Round(stepSlider.Value));
            e.Handled = true;
        }, RoutingStrategies.Tunnel);
        _stepValue = new TextBlock
        {
            Text = "0 / 0",
            Foreground = Brushes.White,
            VerticalAlignment = VerticalAlignment.Center,
            MinWidth = 185
        };
        var seekProgress = new ProgressBar
        {
            Minimum = 0,
            Maximum = 1000,
            Value = 0,
            Width = 130,
            Height = 8,
            IsVisible = false,
            VerticalAlignment = VerticalAlignment.Center
        };

        var debugLabel = new TextBlock
        {
            Text = "Debug (F3)",
            Foreground = Brushes.White,
            VerticalAlignment = VerticalAlignment.Center,
            Margin = new Thickness(8, 0, 0, 0)
        };
        _debugView = new ComboBox
        {
            ItemsSource = GlSceneControl.DebugViewNames,
            SelectedIndex = _scene.DebugView,
            MinWidth = 190
        };
        _debugView.SelectionChanged += (_, _) => _scene.SetDebugView(_debugView.SelectedIndex);

        _debugLegend = new TextBlock
        {
            Foreground = Brushes.White,
            FontSize = 13,
            Text = _scene.DebugLegend
        };
        var legendPanel = new Border
        {
            Background = new SolidColorBrush(Color.FromArgb(190, 18, 20, 24)),
            CornerRadius = new CornerRadius(4),
            Padding = new Thickness(8, 5),
            Margin = new Thickness(10),
            HorizontalAlignment = HorizontalAlignment.Left,
            VerticalAlignment = VerticalAlignment.Bottom,
            Child = _debugLegend
        };

        var topBar = new StackPanel
        {
            Orientation = Orientation.Horizontal,
            HorizontalAlignment = HorizontalAlignment.Left,
            VerticalAlignment = VerticalAlignment.Top,
            Margin = new Thickness(10),
            Spacing = 8,
            Children =
            {
                _playPause, stepLabel, stepSlider, seekProgress, _stepValue, debugLabel, _debugView
            }
        };

        var root = new Grid();
        root.Children.Add(_scene);
        root.Children.Add(topBar);
        root.Children.Add(_hud);
        root.Children.Add(legendPanel);
        Content = root;

        AddHandler(KeyDownEvent, (object? _, KeyEventArgs e) =>
        {
            if (e.Key != Key.F3)
            {
                return;
            }
            _scene.CycleDebugView();
            _debugView.SelectedIndex = _scene.DebugView;
            _debugLegend.Text = _scene.DebugLegend;
            e.Handled = true;
        }, RoutingStrategies.Tunnel);

        // The GL control renders through a composition child visual, so hit-testing can fall
        // through it to the window. Forward window-level pointer events to the scene unless
        // they originate from the scene itself (already handled) or the top-bar UI.
        bool ForwardToScene(RoutedEventArgs e)
        {
            if (e.Source is not Visual v)
            {
                return true;
            }
            if (v == _scene || _scene.IsVisualAncestorOf(v))
            {
                return false;
            }
            return !topBar.IsVisualAncestorOf(v);
        }

        AddHandler(PointerPressedEvent, (object? _, PointerPressedEventArgs e) =>
        {
            if (ForwardToScene(e)) _scene.ViewerPointerPressed(e);
        }, RoutingStrategies.Tunnel);
        AddHandler(PointerMovedEvent, (object? _, PointerEventArgs e) =>
        {
            if (ForwardToScene(e)) _scene.ViewerPointerMoved(e);
        }, RoutingStrategies.Tunnel);
        AddHandler(PointerReleasedEvent, (object? _, PointerReleasedEventArgs e) =>
        {
            if (ForwardToScene(e)) _scene.ViewerPointerReleased(e);
        }, RoutingStrategies.Tunnel);
        AddHandler(PointerWheelChangedEvent, (object? _, PointerWheelEventArgs e) =>
        {
            if (ForwardToScene(e)) _scene.ViewerPointerWheel(e);
        }, RoutingStrategies.Tunnel);

        var timer = new DispatcherTimer
        {
            Interval = TimeSpan.FromMilliseconds(200)
        };
        timer.Tick += (_, _) =>
        {
            _hud.Text = _scene.GetHudText();
            _playPause.Content = _scene.IsPaused ? "Play" : "Pause";
            _debugLegend.Text = _scene.DebugLegend;

            int count = _scene.ToolpathSegmentCount;
            int step = _scene.ToolpathStep;
            bool seeking = _scene.IsSeeking;
            if (!scrubbingStepSlider)
            {
                updatingStepSlider = true;
                try
                {
                    stepSlider.IsEnabled = count > 0;
                    stepSlider.Maximum = Math.Max(1, count);
                    stepSlider.Value = Math.Clamp(step, 0, count);
                }
                finally
                {
                    updatingStepSlider = false;
                }
            }
            seekProgress.IsVisible = seeking;
            seekProgress.IsIndeterminate = seeking && _scene.IsSeekProgressIndeterminate;
            seekProgress.Value = _scene.SeekProgressPermille;
            _stepValue.Text = $"{step:N0} / {count:N0}"
                + (seeking ? $"  {_scene.SeekProgressStage}" : string.Empty);
        };
        timer.Start();
        Opened += (_, _) => StartupTrace.Mark("main window opened");
    }
}
