namespace RW.CSG;

/// <summary>
/// Newest-wins dirty queue keyed by brick id.
/// A brick can be marked repeatedly but appears once until drained.
/// </summary>
public sealed class DirtyBrickSet
{
    private readonly bool[] _present;
    private readonly Queue<int> _queue;

    public DirtyBrickSet(int brickCount)
    {
        if (brickCount <= 0)
        {
            throw new ArgumentOutOfRangeException(nameof(brickCount));
        }

        _present = new bool[brickCount];
        _queue = new Queue<int>(Math.Min(brickCount, 4096));
    }

    public int Count => _queue.Count;

    public void Mark(int brickId)
    {
        if ((uint)brickId >= (uint)_present.Length)
        {
            throw new ArgumentOutOfRangeException(nameof(brickId));
        }

        if (_present[brickId])
        {
            return;
        }

        _present[brickId] = true;
        _queue.Enqueue(brickId);
    }

    public int DrainTo(Span<int> destination)
    {
        int n = Math.Min(destination.Length, _queue.Count);
        for (int i = 0; i < n; i++)
        {
            int b = _queue.Dequeue();
            _present[b] = false;
            destination[i] = b;
        }

        return n;
    }
}
