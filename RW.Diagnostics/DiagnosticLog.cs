using System.Runtime.CompilerServices;

namespace RW.Diagnostics;

/// <summary>Process-wide, opt-in diagnostic namespaces shared by desktop and geometry code.</summary>
public static class DiagnosticLog
{
    public const string NamespaceEnvironmentVariable = "RW_LOG_NAMESPACES";
    private static readonly NamespaceFilter ProcessFilter = new(
        Environment.GetEnvironmentVariable(NamespaceEnvironmentVariable) ?? "");
    private static readonly AsyncLocal<NamespaceFilter?> ScopedFilter = new();
    private static NamespaceFilter Filter => ScopedFilter.Value ?? ProcessFilter;

    public static bool IsAnyEnabled => Filter.Names.Length != 0;
    public static bool IsEnabled(string name) => Filter.Matches(name.AsSpan());

    /// <summary>Checks a dynamic child area without allocating a combined namespace string.</summary>
    public static bool IsEnabled(string parent, string area)
    {
        if (!IsAnyEnabled) return false;
        int length = checked(parent.Length + 1 + area.Length);
        Span<char> name = length <= 256 ? stackalloc char[length] : new char[length];
        parent.AsSpan().CopyTo(name);
        name[parent.Length] = '.';
        area.AsSpan().CopyTo(name[(parent.Length + 1)..]);
        return Filter.Matches(name);
    }

    /// <summary>Explicit selection for an evaluation/test; flows to child tasks without changing other evaluations.</summary>
    public static IDisposable PushNamespaces(params string[] names)
    {
        var previous = ScopedFilter.Value;
        ScopedFilter.Value = new NamespaceFilter(string.Join(",", names));
        return new Scope(previous);
    }

    internal static bool IsMessageEnabled(ReadOnlySpan<char> message)
    {
        if (!IsAnyEnabled) return false;
        message = message.TrimStart();
        if (!message.IsEmpty && message[0] == '[')
        {
            int end = message.IndexOf(']');
            if (end > 1) return Filter.Matches(message.Slice(1, end - 1));
        }
        return IsEnabled("General");
    }

    internal static bool CouldEnableTagPrefix(ReadOnlySpan<char> prefix) => Filter.CouldMatchPrefix(prefix);

    private sealed class NamespaceFilter
    {
        internal readonly string[] Names;

        internal NamespaceFilter(string selection) => Names = selection.Split(
            new[] { ',', ';', ' ', '\t', '\r', '\n' }, StringSplitOptions.RemoveEmptyEntries | StringSplitOptions.TrimEntries);

        internal bool Matches(ReadOnlySpan<char> name)
        {
            foreach (string selected in Names)
            {
                if (selected.EndsWith('*'))
                {
                    if (name.StartsWith(selected.AsSpan(0, selected.Length - 1), StringComparison.OrdinalIgnoreCase)) return true;
                }
                else if (name.Equals(selected.AsSpan(), StringComparison.OrdinalIgnoreCase) ||
                         (name.Length > selected.Length && name[selected.Length] is '.' or ':' &&
                          name.StartsWith(selected.AsSpan(), StringComparison.OrdinalIgnoreCase))) return true;
            }
            return false;
        }

        internal bool CouldMatchPrefix(ReadOnlySpan<char> prefix)
        {
            foreach (string selected in Names)
            {
                var name = selected.AsSpan();
                bool wildcard = name.EndsWith("*", StringComparison.Ordinal);
                if (wildcard) name = name[..^1];
                if (name.StartsWith(prefix, StringComparison.OrdinalIgnoreCase) ||
                    (prefix.StartsWith(name, StringComparison.OrdinalIgnoreCase) &&
                     (wildcard || prefix.Length == name.Length || prefix[name.Length] is '.' or ':'))) return true;
            }
            return false;
        }
    }

    private sealed class Scope : IDisposable
    {
        private readonly NamespaceFilter? _previous;
        internal Scope(NamespaceFilter? previous) => _previous = previous;
        public void Dispose() => ScopedFilter.Value = _previous;
    }
}

/// <summary>
/// Stops interpolation at the first literal when its [namespace] is disabled.
/// The compiler also skips every argument when no namespaces are selected.
/// </summary>
[InterpolatedStringHandler]
public ref struct DiagnosticMessageHandler
{
    private DefaultInterpolatedStringHandler _builder;
    private bool _firstLiteral;
    private bool _initialized;
    private readonly int _literalLength;
    private readonly int _formattedCount;
    internal bool Enabled;
    internal bool RequiresFinalFilter;

    public DiagnosticMessageHandler(int literalLength, int formattedCount, out bool shouldAppend)
    {
        Enabled = shouldAppend = DiagnosticLog.IsAnyEnabled;
        _firstLiteral = true;
        _initialized = false;
        _literalLength = literalLength;
        _formattedCount = formattedCount;
        _builder = default;
        RequiresFinalFilter = false;
    }

    public bool AppendLiteral(string value)
    {
        if (_firstLiteral)
        {
            _firstLiteral = false;
            var first = value.AsSpan().TrimStart();
            RequiresFinalFilter = !first.IsEmpty && first[0] == '[' && first.IndexOf(']') < 0;
            Enabled = RequiresFinalFilter
                ? DiagnosticLog.CouldEnableTagPrefix(first[1..])
                : DiagnosticLog.IsMessageEnabled(first);
            if (Enabled) Initialize();
        }
        if (Enabled) _builder.AppendLiteral(value);
        return Enabled;
    }

    // A message that starts with a value has no static tag: it belongs to General.
    private bool BeforeValue()
    {
        if (_firstLiteral)
        {
            _firstLiteral = false;
            Enabled = DiagnosticLog.IsEnabled("General");
            if (Enabled) Initialize();
        }
        return Enabled;
    }

    public bool AppendFormatted<T>(T value)
    {
        if (!BeforeValue()) return false;
        _builder.AppendFormatted(value);
        return true;
    }
    public bool AppendFormatted<T>(T value, string? format)
    {
        if (!BeforeValue()) return false;
        _builder.AppendFormatted(value, format);
        return true;
    }
    public bool AppendFormatted<T>(T value, int alignment, string? format = null)
    {
        if (!BeforeValue()) return false;
        _builder.AppendFormatted(value, alignment, format);
        return true;
    }

    private void Initialize()
    {
        _builder = new DefaultInterpolatedStringHandler(_literalLength, _formattedCount);
        _initialized = true;
    }

    internal string GetText() => _initialized ? _builder.ToStringAndClear() : "";
    internal void Discard()
    {
        // Return the builder's pooled buffer even when AppendLiteral rejected the tag.
        if (_initialized) _builder.ToStringAndClear();
    }
}
