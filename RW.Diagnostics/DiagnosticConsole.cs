using System.Diagnostics;

namespace RW.Diagnostics;

/// <summary>
/// A project-level Console alias keeps existing bracketed diagnostic namespaces intact.
/// Writes use System.Console's current writers so capture/redirection still works.
/// </summary>
public static class DiagnosticConsole
{
    public static DiagnosticOutput Error { get; } = new(true);
    public static DiagnosticOutput Out { get; } = new(false);
    public static event ConsoleCancelEventHandler? CancelKeyPress
    {
        add => System.Console.CancelKeyPress += value;
        remove => System.Console.CancelKeyPress -= value;
    }

    public static void WriteLine(ref DiagnosticMessageHandler message) => Out.WriteLine(ref message);
    public static void WriteLine(string? message) => Out.WriteLine(message);
    public static void WriteLine(string format, params object?[] values) => Out.WriteLine(format, values);
    public static void WriteLine() => Out.WriteLine("");

    [Conditional("DEBUG")]
    public static void DebugWriteLine(ref DiagnosticMessageHandler message) => Out.WriteLine(ref message);
    [Conditional("DEBUG")]
    public static void DebugWriteLine(string? message) => Out.WriteLine(message);
    [Conditional("TRACE")]
    public static void TraceWriteLine(ref DiagnosticMessageHandler message) => Out.WriteLine(ref message);
    [Conditional("TRACE")]
    public static void TraceWriteLine(string? message) => Out.WriteLine(message);
}

public sealed class DiagnosticOutput
{
    private readonly bool _error;
    internal DiagnosticOutput(bool error) => _error = error;
    private TextWriter Writer => _error ? System.Console.Error : System.Console.Out;

    public void WriteLine(ref DiagnosticMessageHandler message)
    {
        if (message.Enabled)
        {
            string text = message.GetText();
            if (!message.RequiresFinalFilter || DiagnosticLog.IsMessageEnabled(text.AsSpan())) Writer.WriteLine(text);
        }
        else message.Discard();
    }

    public void WriteLine(string? message)
    {
        if (DiagnosticLog.IsMessageEnabled(message.AsSpan())) Writer.WriteLine(message);
    }

    public void WriteLine(string format, params object?[] values)
    {
        if (DiagnosticLog.IsMessageEnabled(format.AsSpan())) Writer.WriteLine(format, values);
    }

    public void Flush() => Writer.Flush();
}
