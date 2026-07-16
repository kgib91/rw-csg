using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Numerics;
using System.Runtime;
using System.Threading.Tasks;
using global::RW.CSG;

namespace RW.CSG.Avalonia;

/// <summary>Opt-in validation and performance diagnostics. Invoked only by an explicit
/// `--bench &lt;name&gt;`; normal viewer startup never touches this class.</summary>
internal static class Benches
{
    public static int Run(string name)
    {
        Console.WriteLine($"# bench '{name}' | {Environment.ProcessorCount} cores | serverGC={GCSettings.IsServerGC} | {(Debugger.IsAttached ? "DEBUGGER" : "no-debugger")}");
#if DEBUG
        Console.WriteLine("!! DEBUG build — numbers are meaningless, use -c Release");
#endif
        try
        {
            switch (name)
            {
                case "sweep": Sweep(); return 0;
                case "verify": Verify(); return 0;
                case "stamp": Stamp(); return 0;
                case "volume": VolumeDrift(); return 0;
                case "pathindex": PathIndexBench(); return 0;
                case "epsilon": Epsilon(); return 0;
                case "march": March(); return 0;
                case "staging": Staging(); return 0;
                case "rebake": RebakeBench(); return 0;
                case "mesh": MeshBench(); return 0;
                case "liveheadless": LiveHeadless(); return 0;
                case "stack": StackProfile(); return 0;
                case "startup": Startup(); return 0;
                case "seekcpu": SeekCpu(); return 0;
                default:
                    Console.WriteLine("unknown bench. available: sweep verify stamp volume pathindex epsilon march staging rebake mesh liveheadless stack startup seekcpu upload raymarch seam live seek");
                    return 2;
            }
        }
        catch (Exception ex)
        {
            Console.WriteLine($"BENCH FAILED: {ex}");
            return 1;
        }
        finally { Console.Out.Flush(); }
    }

    private static double TimeMs(Action a)
    {
        var sw = Stopwatch.StartNew();
        a();
        return sw.Elapsed.TotalMilliseconds;
    }

    private static void SeekCpu()
    {
        const int voxelsX = 3200, voxelsY = 1600, voxelsZ = 40;
        const float voxelSize = 0.25f;
        const int limit = 1 << 20;
        var segments = HilbertPath.GenerateSegments(
            11, 800f, 400f, 8f, 8.2f, 0.5f, 0.02f);
        var stack = new SweptVolumeStack([
            SweptVolumeSegment.Cone(1f, 5f, 15f),
            SweptVolumeSegment.Cylinder(7.5f, 8f),
            SweptVolumeSegment.Cylinder(3.175f, 80f)
        ]);
        var tool = new ToolProfile(stack, radialLutSize: 256, axialLutSize: 1024, pad: 2f);
        float[]? replayedCoarse = null;

        foreach ((int batchSize, bool coarse) in new[] { (limit, true), (limit, false) })
        {
            var volume = new SparseSdfVolume(
            voxelsX, voxelsY, voxelsZ, voxelSize, 8, 3) { CoarseEnabled = coarse };
            var timer = Stopwatch.StartNew();
            for (int cursor = 0; cursor < limit; cursor += batchSize)
            {
                volume.StampBatch(segments, cursor, Math.Min(batchSize, limit - cursor),
                    tool, parallel: true, conservativeCull: false);
            }
            if (!coarse)
            {
                volume.RebuildCoarseFromBrickStates();
                float maxCoarseExcess = float.MinValue;
                for (int i = 0; i < volume.Coarse.Length; i++)
                {
                    if (volume.Coarse[i] > volume.CoarseCell)
                    {
                        maxCoarseExcess = MathF.Max(maxCoarseExcess,
                            volume.Coarse[i] - replayedCoarse![i]);
                    }
                }
                if (maxCoarseExcess > 1e-6f)
                    throw new Exception(
                        $"topology coarse field exceeds replayed field by {maxCoarseExcess:F6} mm");
            }
            else
            {
                replayedCoarse = (float[])volume.Coarse.Clone();
            }
            timer.Stop();
            double fingerprint = 0;
            for (int i = 0; i < 100_000; i++)
            {
                int x = (i * 104_729 + 17) % voxelsX;
                int y = (i * 130_363 + 29) % voxelsY;
                int z = (i * 53 + 7) % voxelsZ;
                fingerprint += volume.GetVoxel(x, y, z) * (1 + (i & 15));
            }
            Console.WriteLine(
                $"seekcpu: batch={batchSize,9:N0} coarse={coarse,-5} | {timer.Elapsed.TotalMilliseconds,9:F1} ms | "
                + $"{limit / timer.Elapsed.TotalSeconds / 1000,7:F1} kst/s | "
                + $"miss={volume.StatMissSkips:N0} engulf={volume.StatEngulfs:N0} "
                + $"evals={volume.StatVoxelEvals:N0} "
                + $"col={volume.StatColumnDists:N0} runs={volume.StatRuns:N0} "
                + $"mixed={volume.AllocatedBricks:N0} "
                + $"removed={volume.RemovedVolume:F3} fp={fingerprint:F6}");
        }
    }

    private static void Startup()
    {
        const int order = 8;
        Vector3[] points = HilbertPath.Generate(order, 200f, 8f, 8.2f, 0.5f, 0.02f);
        PathSegment[] expected = HilbertPath.ToSegments(points, 0, points.Length - 1);
        PathSegment[] actual = HilbertPath.GenerateSegments(order, 200f, 8f, 8.2f, 0.5f, 0.02f);
        if (actual.Length != expected.Length)
            throw new Exception($"direct Hilbert segment count mismatch: {actual.Length} != {expected.Length}");
        for (int i = 0; i < actual.Length; i++)
            if (actual[i].A != expected[i].A || actual[i].B != expected[i].B)
                throw new Exception($"direct Hilbert segment mismatch at {i}: {actual[i]} != {expected[i]}");

        PathSegment[] rectangular = HilbertPath.GenerateSegments(
            order, 800f, 400f, 8f, 8.2f, 0.5f, 0.02f);
        float minX = float.MaxValue, maxX = float.MinValue;
        float minY = float.MaxValue, maxY = float.MinValue;
        foreach (ref readonly PathSegment segment in rectangular.AsSpan())
        {
            minX = MathF.Min(minX, MathF.Min(segment.A.X, segment.B.X));
            maxX = MathF.Max(maxX, MathF.Max(segment.A.X, segment.B.X));
            minY = MathF.Min(minY, MathF.Min(segment.A.Y, segment.B.Y));
            maxY = MathF.Max(maxY, MathF.Max(segment.A.Y, segment.B.Y));
        }
        if (minX != 8f || maxX != 792f || minY != 8f || maxY != 392f)
            throw new Exception($"rectangular Hilbert bounds mismatch: X={minX}..{maxX}, Y={minY}..{maxY}");

        var volume = new SparseSdfVolume(320, 160, 40, 0.25f, 8, 3);
        if (volume.StockSizeX != 80f || volume.StockSizeY != 40f || volume.StockHeight != 10f
            || volume.BricksX != 40 || volume.BricksY != 20 || volume.BricksZ != 5
            || volume.CoarseX != 40 || volume.CoarseY != 20 || volume.CoarseZ != 5
            || volume.BrickId(39, 19, 4) != volume.BrickCount - 1)
            throw new Exception("rectangular sparse-volume dimensions or strides are incorrect");
        if (volume.SampleCoarse(40f, 20f, 5f) >= 0f)
            throw new Exception("rectangular coarse field does not classify the stock interior as solid");

        var edgeTool = ToolProfile.FlatEndmill(1f, 4f, 32);
        PathSegment[] edgeCut =
        [
            new(new Vector3(78f, 38f, 8f), new Vector3(79f, 39f, 8f))
        ];
        volume.StampBatch(edgeCut, 0, 1, edgeTool, parallel: false);
        if (volume.DirtyBricks.Count == 0 || volume.SampleFine(78.5f, 38.5f, 9f) <= 0f)
            throw new Exception("rectangular edge stamping did not update the expected region");

        SparseSdfVolume clone = volume.DeepClone();
        for (int i = 0; i < 1000; i++)
        {
            int x = i * 37 % volume.VoxelsX;
            int y = i * 53 % volume.VoxelsY;
            int z = i * 17 % volume.VoxelsZ;
            if (clone.GetVoxel(x, y, z) != volume.GetVoxel(x, y, z))
                throw new Exception("sparse-volume clone differs from its source");
        }
        PathSegment[] cloneOnlyCut =
        [
            new(new Vector3(20f, 20f, 8f), new Vector3(21f, 20f, 8f))
        ];
        clone.StampBatch(cloneOnlyCut, 0, 1, edgeTool, parallel: false);
        if (clone.SampleFine(20.5f, 20f, 9f) <= 0f
            || volume.SampleFine(20.5f, 20f, 9f) >= 0f)
            throw new Exception("sparse-volume clone does not mutate independently");

        Console.WriteLine($"PASS startup: square parity {actual.Length:N0} segments; rectangular path {minX}..{maxX} × {minY}..{maxY}; volume {volume.StockSizeX}×{volume.StockSizeY}×{volume.StockHeight} mm");
    }

    private static void StackProfile()
    {
        var stack = new SweptVolumeStack([
            SweptVolumeSegment.Cone(1f, 5f, 15f),
            SweptVolumeSegment.Cylinder(7.5f, 8f),
            SweptVolumeSegment.Cylinder(3.175f, 80f)
        ]);
        var tool = new ToolProfile(stack, 256, 1024, 2f);

        if (stack.Segments.Count != 3 || MathF.Abs(stack.TotalHeight - 103f) > 1e-5f
            || MathF.Abs(stack.MaxRadius - 7.5f) > 1e-5f)
            throw new Exception("stack dimensions do not match the attached 103 mm / Ø15 profile");
        if (stack.GetDiscontinuities().Count != 2)
            throw new Exception("expected the attached profile's two explicit shoulder joins");

        (float r, float z, bool inside)[] probes =
        [
            (0.5f, 0.5f, true),
            (2.5f, 7.5f, true),
            (4.0f, 7.5f, false),
            (7.0f, 19.0f, true),
            (5.0f, 30.0f, false),
            (3.0f, 80.0f, true),
            (3.5f, 80.0f, false)
        ];
        foreach (var probe in probes)
        {
            float exact = stack.EvaluateSdf(probe.r, probe.z);
            float baked = tool.Sample(probe.r, probe.z);
            if ((exact < 0f) != probe.inside || (baked < tool.Epsilon) != probe.inside)
                throw new Exception($"profile sign mismatch at r={probe.r}, z={probe.z}: exact={exact}, lut={baked}");
        }

        float stockActiveRadius = tool.MaxRadiusInRange(-2f, 4f);
        if (stockActiveRadius > 2.5f)
            throw new Exception($"far shaft/collar leaked into the tip cutting footprint: R={stockActiveRadius}");
        bool tipRowsMonotone = tool.IsRadiallyMonotone(-1f, 3f);
        if (!tipRowsMonotone)
            throw new Exception("stock-intersecting cone rows lost the stroke-run radial-monotone fast path");

        const float vox = 0.25f;
        var volume = new SparseSdfVolume(128, vox, 8, 3) { CoarseEnabled = false };
        var cut = new[] { new PathSegment(new Vector3(16f, 16f, 30f), new Vector3(16.25f, 16f, 30f)) };
        volume.StampBatch(cut, 0, 1, tool, parallel: false);
        // One millimeter below the original top sees the cone near R1.27, not the old R3
        // cylinder and not the R7.5 collar that is still 14 mm above the stock surface.
        float carvedInside = volume.SampleFine(17.1f, 16f, 31f);
        float uncutOutside = volume.SampleFine(18.0f, 16f, 31f);
        if (carvedInside <= 0f || uncutOutside >= 0f)
            throw new Exception($"stacked cone was not used by StampBatch: inside={carvedInside}, outside={uncutOutside}");

        Console.WriteLine($"PASS stack: {stack.Segments.Count} primitives, H={stack.TotalHeight:F2} mm, Rmax={stack.MaxRadius:F2} mm, LUT={tool.LutWidth}x{tool.LutHeight}, epsilon={tool.Epsilon:F4} mm, cone carve={carvedInside:F3}/{uncutOutside:F3}, tipRun={tipRowsMonotone}");
    }

    // =====================================================================
    // B1 — swept-brush evaluation throughput (doc1 #1: THE primary unknown)
    //      + conservative-range cost (doc1 #2)
    // =====================================================================
    private static void Sweep()
    {
        const float vox = 0.3f;
        var rng = new Random(42);

        // Realistic point set: voxel positions inside stamp AABBs of 1024 distinct stamplets.
        const int nSegs = 1024;
        var segs = new PathSegment[nSegs];
        var pos = new Vector3[nSegs * 64];
        for (int i = 0; i < nSegs; i++)
        {
            var a = new Vector3(Rand(rng, 20, 280), Rand(rng, 20, 280), Rand(rng, 5, 10));
            var d = new Vector3(Rand(rng, -1, 1), Rand(rng, -1, 1), Rand(rng, -0.2f, 0.2f));
            var b = a + Vector3.Normalize(d) * vox;
            segs[i] = new PathSegment(a, b);
            for (int j = 0; j < 64; j++)
                pos[i * 64 + j] = a + new Vector3(Rand(rng, -4, 4), Rand(rng, -4, 4), Rand(rng, -1, 13));
        }

        foreach (var (label, tool) in new (string, ToolProfile)[]
        {
            ("flat r3 LUT64", ToolProfile.FlatEndmill(3, 12, 64)),
            ("flat r3 LUT256", ToolProfile.FlatEndmill(3, 12, 256)),
            ("tslot LUT64", ToolProfile.TSlotCutter(1.5f, 4, 2, 12, 64)),
            ("tslot LUT256", ToolProfile.TSlotCutter(1.5f, 4, 2, 12, 256)),
        })
        {
            // warmup + blackhole
            float sink = 0;
            for (int i = 0; i < nSegs * 64; i++) sink += SweptBrush.Eval(tool, segs[i / 64].A, segs[i / 64].B, pos[i].X, pos[i].Y, pos[i].Z);

            const long evalsTarget = 20_000_000;
            int reps = (int)(evalsTarget / (nSegs * 64));
            double stMs = TimeMs(() =>
            {
                float s = 0;
                for (int r = 0; r < reps; r++)
                    for (int i = 0; i < nSegs * 64; i++)
                    {
                        ref readonly var sg = ref segs[i >> 6];
                        s += SweptBrush.Eval(tool, sg.A, sg.B, pos[i].X, pos[i].Y, pos[i].Z);
                    }
                sink += s;
            });
            long evals = (long)reps * nSegs * 64;
            double stNs = stMs * 1e6 / evals;

            double mtMs = TimeMs(() =>
            {
                Parallel.For(0, reps * 4, r =>
                {
                    float s = 0;
                    for (int i = 0; i < nSegs * 64; i++)
                    {
                        ref readonly var sg = ref segs[i >> 6];
                        s += SweptBrush.Eval(tool, sg.A, sg.B, pos[i].X, pos[i].Y, pos[i].Z);
                    }
                    if (s == float.MinValue) Console.Write("");
                });
            });
            double mtEvalsPerSec = (double)evals * 4 / (mtMs / 1000);

            Console.WriteLine($"{label,-16} ST {stNs,6:F1} ns/eval = {evals / (stMs / 1000) / 1e6,7:F1} M evals/s | MT {mtEvalsPerSec / 1e6,7:F1} M evals/s (x{mtEvalsPerSec / (evals / (stMs / 1000)):F1}) [{sink:E1}]");
        }

        // Conservative range cost (doc1 #2)
        var t64 = ToolProfile.FlatEndmill(3, 12, 64);
        float ballR = 0.8660254f * 8 * vox;
        float sink2 = 0;
        const int rr = 400;
        double rangeMs = TimeMs(() =>
        {
            for (int r = 0; r < rr; r++)
                for (int i = 0; i < nSegs * 64; i++)
                {
                    ref readonly var sg = ref segs[i >> 6];
                    SweptBrush.ConservativeRange(t64, sg.A, sg.B, pos[i].X, pos[i].Y, pos[i].Z, ballR, out float dmin, out float dmax);
                    sink2 += dmin + dmax;
                }
        });
        long rCalls = (long)rr * nSegs * 64;
        Console.WriteLine($"ConservativeRange: {rangeMs * 1e6 / rCalls,6:F1} ns/call ({rCalls / (rangeMs / 1000) / 1e6:F0} M/s) [{sink2:E1}]");
        Console.WriteLine($"→ stamplet budget @70k/s on {Environment.ProcessorCount} cores: {Environment.ProcessorCount * 1e9 / 70_000:F0} ns/stamplet total");
    }

    // =====================================================================
    // verify — stamped grid vs brute-force exact brush-list evaluation
    // =====================================================================
    private static void Verify()
    {
        const float vox = 0.3f;
        var vol = new SparseSdfVolume(128, vox, 8, 3);
        vol.CoarseEnabled = false;
        var tool = ToolProfile.TSlotCutter(1.5f, 4, 2, 12, 64);   // undercut profile
        var rng = new Random(7);

        // random-walk path in the middle of the small stock
        const int nSegs = 200;
        var segs = new PathSegment[nSegs];
        var p = new Vector3(19, 19, 15);
        for (int i = 0; i < nSegs; i++)
        {
            var d = new Vector3(Rand(rng, -1, 1), Rand(rng, -1, 1), Rand(rng, -0.3f, 0.3f));
            var q = p + Vector3.Normalize(d) * vox;
            q.X = Math.Clamp(q.X, 10, 28); q.Y = Math.Clamp(q.Y, 10, 28); q.Z = Math.Clamp(q.Z, 12, 20);
            segs[i] = new PathSegment(p, q);
            p = q;
        }
        vol.StampBatch(segs, 0, nSegs, tool, parallel: true);
        Console.WriteLine($"stamped {nSegs} segs: {vol.DirtyBricks.Count} dirty bricks, evals={vol.StatVoxelEvals}, engulfs={vol.StatEngulfs}");

        // reference: max(-B, min(B, max_s(-exactBrush))) per voxel; compare voxels in dirty bricks
        float band = vol.BandWorld;
        double maxAbsErr = 0, sumErr = 0; long n = 0;
        var dirty = vol.DirtyBricks.ToArray();
        var errLock = new object();
        Parallel.For(0, Math.Min(dirty.Length, 64), bi =>
        {
            int b = dirty[bi * Math.Max(1, dirty.Length / 64)];
            int bs = vol.BrickSize;
            int bx = b % vol.BricksX;
            int by = b / vol.BricksX % vol.BricksY;
            int bz = b / (vol.BricksX * vol.BricksY);
            double lmax = 0, lsum = 0; long ln = 0;
            for (int v = 0; v < 27; v++) // 27 sample voxels per brick
            {
                int ix = bx * bs + v % 3 * 3, iy = by * bs + v / 3 % 3 * 3, iz = bz * bs + v / 9 * 3;
                float stored = vol.GetVoxel(ix, iy, iz);
                float px = (ix + 0.5f) * vox, py = (iy + 0.5f) * vox, pz = (iz + 0.5f) * vox;
                float refd = -band;
                foreach (ref readonly var s in segs.AsSpan())
                {
                    float e = -EvalExact(tool, s.A, s.B, new Vector3(px, py, pz), 32);
                    if (e > refd) refd = e;
                }
                refd = MathF.Min(refd, band);
                // stamped value uses Eval = LUT interp − ε ⇒ expected bias ∈ [0, ~2ε]; measure vs (ref + ε)
                double err = stored - Math.Clamp(refd + tool.Epsilon, -band, band);
                lmax = Math.Max(lmax, Math.Abs(err)); lsum += Math.Abs(err); ln++;
            }
            lock (errLock) { maxAbsErr = Math.Max(maxAbsErr, lmax); sumErr += lsum; n += ln; }
        });
        Console.WriteLine($"vs exact brush-list (ε={tool.Epsilon:F3} mm, voxel={vox} mm): maxErr={maxAbsErr:F4} mm, avgErr={sumErr / n:F4} mm over {n} voxels");
        Console.WriteLine(maxAbsErr < tool.Epsilon + 0.5f * vox ? "PASS (within ε + interp tolerance)" : "FAIL — investigate before trusting other benches");
    }

    private static float EvalExact(ToolProfile profile, in Vector3 a, in Vector3 b,
        in Vector3 point, int steps)
    {
        float best = float.MaxValue;
        for (int i = 0; i <= steps; i++)
        {
            float t = i / (float)steps;
            float qx = point.X - (a.X + (b.X - a.X) * t);
            float qy = point.Y - (a.Y + (b.Y - a.Y) * t);
            float qz = point.Z - (a.Z + (b.Z - a.Z) * t);
            best = MathF.Min(best, profile.ExactSdf(
                new Vector2(MathF.Sqrt(qx * qx + qy * qy), qz)));
        }
        return best;
    }

    private static float Rand(Random r, float lo, float hi) => lo + (float)r.NextDouble() * (hi - lo);

    // =====================================================================
    // B2 — StampBatch throughput @1024³ virtual on a dense Hilbert path
    //      (doc1 #1 primary unknown, #6 brick size, #7 band width, #9 upload volume)
    // =====================================================================
    private static void Stamp()
    {
        const float vox = 0.3f;
        const int order = 10;                       // 1,048,575 stamplet segments, ~0.285 mm spacing
        float stock = 1024 * vox;                   // 307.2 mm
        var tool = ToolProfile.FlatEndmill(3, 12, 64);

        var ptsShallow = HilbertPath.Generate(order, stock, 8, stock - 1.5f, 0.5f, 0.02f); // 1.5±0.5 mm deep
        var shallow = HilbertPath.ToSegments(ptsShallow, 0, ptsShallow.Length - 1);
        var ptsDeep = HilbertPath.Generate(order, stock, 8, stock - 6f, 0.5f, 0.02f);      // 6 mm deep
        var deep = HilbertPath.ToSegments(ptsDeep, 0, ptsDeep.Length - 1);

        Console.WriteLine($"stock {stock} mm, voxel {vox} mm, path {shallow.Length} stamplets, target 70k/s");
        Console.WriteLine("config                     | kstamp/s | evals/st | miss% | engulf | dirty/bat | bricks  poolMB | GC0/1/2 | removed mm³");

        RunStamp("8³ B3 ST shallow (100k)", shallow, 100_000, vox, 8, 3, false, true, tool);
        RunStamp("8³ B3 MT shallow (300k)", shallow, 300_000, vox, 8, 3, true, true, tool);
        RunStamp("8³ B3 MT shallow FULL 1M", shallow, shallow.Length, vox, 8, 3, true, true, tool);
        RunStamp("8³ B3 MT no-coarse(300k)", shallow, 300_000, vox, 8, 3, true, false, tool);
        RunStamp("4³ B3 MT shallow (300k)", shallow, 300_000, vox, 4, 3, true, true, tool);
        RunStamp("16³ B3 MT shallow(300k)", shallow, 300_000, vox, 16, 3, true, true, tool);
        RunStamp("8³ B4 MT shallow (300k)", shallow, 300_000, vox, 8, 4, true, true, tool);
        RunStamp("8³ B3 MT deep6mm (300k)", deep, 300_000, vox, 8, 3, true, true, tool);
    }

    private static void RunStamp(string label, PathSegment[] segs, int limit, float vox,
        int brick, int band, bool parallel, bool coarse, ToolProfile tool)
    {
        var vol = new SparseSdfVolume(1024, vox, brick, band) { CoarseEnabled = coarse };
        int batchSize = 1167;                       // ≈ 70k/s ÷ 60 fps
        int g0 = GC.CollectionCount(0), g1 = GC.CollectionCount(1), g2 = GC.CollectionCount(2);
        long dirtySum = 0; int batches = 0;
        var sw = Stopwatch.StartNew();
        for (int s = 0; s < limit; s += batchSize)
        {
            vol.StampBatch(segs, s, Math.Min(batchSize, limit - s), tool, parallel);
            dirtySum += vol.DirtyBricks.Count;
            batches++;
        }
        sw.Stop();
        double kps = limit / sw.Elapsed.TotalSeconds / 1000;
        double evalsPer = (double)vol.StatVoxelEvals / limit;
        long rangeTests = vol.StatMissSkips + vol.StatEngulfs + vol.StatPartialBricks; // ~lower bound
        double missPct = 100.0 * vol.StatMissSkips / Math.Max(1, rangeTests);
        Console.WriteLine($"{label,-26} | {kps,8:F1} | {evalsPer,8:F0} | {missPct,5:F1} | {vol.StatEngulfs,6} | {dirtySum / Math.Max(1, batches),9} | {vol.AllocatedBricks,6} {vol.PoolBytes / 1e6,7:F0} | {GC.CollectionCount(0) - g0}/{GC.CollectionCount(1) - g1}/{GC.CollectionCount(2) - g2} | {vol.RemovedVolume,10:F0}");
    }

    // =====================================================================
    // volume — doc1 #3: removed-volume accumulation accuracy + idempotence drift
    // =====================================================================
    private static void VolumeDrift()
    {
        const float vox = 0.3f;
        var tool = ToolProfile.FlatEndmill(3, 12, 128);

        // A) ground truth: one straight slot through a small block, analytic volume
        {
            var vol = new SparseSdfVolume(256, vox, 8, 3) { CoarseEnabled = false };
            float stock = 256 * vox;                  // 76.8 mm
            float depth = 2.0f, zTip = stock - depth;
            float x0 = 10, x1 = stock - 10, y = stock / 2;
            int n = (int)((x1 - x0) / (vox * 0.95f));
            var segs = new PathSegment[n];
            for (int i = 0; i < n; i++)
            {
                float xa = x0 + (x1 - x0) * i / n, xb = x0 + (x1 - x0) * (i + 1) / n;
                segs[i] = new PathSegment(new Vector3(xa, y, zTip), new Vector3(xb, y, zTip));
            }
            vol.StampBatch(segs, 0, n, tool, parallel: true);
            // slot = rectangle (x1-x0)×2r + two half-discs, depth deep; tool fattened by ε
            float rEff = 3 + tool.Epsilon;
            double analytic = ((x1 - x0) * 2 * rEff + Math.PI * rEff * rEff) * (depth + tool.Epsilon);
            Console.WriteLine($"A) slot: removed={vol.RemovedVolume,10:F1} mm³ analytic(ε-fattened)={analytic,10:F1} → err {100 * (vol.RemovedVolume - analytic) / analytic,6:F2} %");
        }

        // B) idempotence drift: re-stamp the SAME batch many times — removed must stay ~0
        foreach (float writeEps in new[] { 0f, 0.0009f })   // 0 vs ~1 Half-ULP at band scale
        {
            var vol = new SparseSdfVolume(256, vox, 8, 3) { CoarseEnabled = false, WriteEpsilon = writeEps };
            float stock = 256 * vox;
            float zTip = stock - 2.0f;
            var segs = new PathSegment[200];
            for (int i = 0; i < segs.Length; i++)
                segs[i] = new PathSegment(new Vector3(15 + i * 0.28f, 38, zTip), new Vector3(15 + (i + 1) * 0.28f, 38, zTip));
            vol.StampBatch(segs, 0, segs.Length, tool, parallel: true);
            double after1 = vol.RemovedVolume;
            long writes1 = vol.StatVoxelWrites;
            const int reps = 200;
            for (int r = 0; r < reps; r++) vol.StampBatch(segs, 0, segs.Length, tool, parallel: true);
            double drift = vol.RemovedVolume - after1;
            Console.WriteLine($"B) writeEps={writeEps:F4}: 1st stamp removed={after1:F2} mm³; {reps} re-stamps drift={drift:E2} mm³ ({100 * drift / after1:F3} %), extra writes={vol.StatVoxelWrites - writes1}");
        }
    }

    // =====================================================================
    // pathindex — doc1 #5: memory at 4M segments, worst-case cell occupancy
    // =====================================================================
    private static void PathIndexBench()
    {
        const float vox = 0.3f;
        float stock = 1024 * vox;
        float cell = 8 * vox;                        // brick-sized cells (2.4 mm)
        float toolPad = 3 + 3 * vox + 0.13f;         // Rmax + band + ε ≈ 4.03 mm

        // 4.19M-segment Hilbert (the demo path)
        var pts = HilbertPath.Generate(11, stock, 8, stock - 1.5f, 0.5f, 0.02f);
        var hil = HilbertPath.ToSegments(pts, 0, pts.Length - 1);

        // adversarial: 4M-segment raster finishing pass, 0.05 mm stepover, in a 15×15 mm patch
        var fin = new PathSegment[4_000_000];
        {
            const int perLine = 300; const float segLen = 0.05f; int line = 0;
            for (int i = 0; i < fin.Length; i++)
            {
                int k = i % perLine;
                if (k == 0) line++;
                float y = 100 + line * 0.05f % 15, x = 100 + k * segLen;
                float z = 300 - 0.001f * line;
                fin[i] = new PathSegment(new Vector3(x, y, z), new Vector3(x + segLen, y, z));
            }
        }

        foreach (var (label, segs, pad) in new[]
        {
            ("hilbert4M tight(query-expand)", hil, 0f),
            ("hilbert4M naive(build-expand)", hil, toolPad),
            ("raster4M  tight(query-expand)", fin, 0f),
            ("raster4M  naive(build-expand)", fin, toolPad),
        })
        {
            var sw = Stopwatch.StartNew();
            var idx = new PathIndex(segs, stock, cell, pad);
            sw.Stop();
            var (total, max, p99, avg) = idx.Stats();
            Console.WriteLine($"{label}: build {sw.ElapsedMilliseconds,5} ms | {idx.MemoryBytes / 1e6,7:F1} MB | refs {total / 1e6,6:F1} M | per-cell avg {avg,7:F1} max {max,6} p99 {p99,5}");
        }
    }

    // =====================================================================
    // epsilon — doc2 #3: does the −ε brush fattening STACK over overlapping passes?
    // =====================================================================
    private static void Epsilon()
    {
        const float vox = 0.3f;
        float stock = 256 * vox;                     // 76.8 mm
        float depth = 1.8f, zFloor = stock - depth;  // analytic floor plane

        Console.WriteLine("floor error vs analytic (negative = overcut). voxel=0.3, flat endmill r=3");
        Console.WriteLine("config                       | ε mm   | mean err | max |err|");
        foreach (int lut in new[] { 32, 64, 128, 256 })
        foreach (int passes in new[] { 1, 10, 100 })
        {
            var tool = ToolProfile.FlatEndmill(3, 12, lut);
            var vol = new SparseSdfVolume(256, vox, 8, 3) { CoarseEnabled = false, WriteEpsilon = 0.0009f };
            // raster over a 24×24 mm patch, 1 mm stepover, repeated `passes` times
            var segs = new List<PathSegment>();
            for (int j = 0; j <= 24; j++)
            {
                float y = 26 + j;
                for (float x = 26; x < 50; x += 0.28f)
                    segs.Add(new PathSegment(new Vector3(x, y, zFloor), new Vector3(MathF.Min(x + 0.28f, 50), y, zFloor)));
            }
            var arr = segs.ToArray();
            for (int p = 0; p < passes; p++) vol.StampBatch(arr, 0, arr.Length, tool, parallel: true);

            // zero-crossing of the floor under the patch centre region, via bisection on SampleFine
            double sum = 0, maxAbs = 0; int n = 0;
            for (float x = 32; x <= 44; x += 0.77f)
                for (float y = 32; y <= 44; y += 0.77f)
                {
                    float lo = zFloor - 2, hi = zFloor + 2;  // d(lo)<0 material below? NB: below floor = material (d<0), above = air (d>0)
                    for (int it = 0; it < 40; it++)
                    {
                        float mid = 0.5f * (lo + hi);
                        if (vol.SampleFine(x, y, mid) < 0) lo = mid; else hi = mid;
                    }
                    double err = 0.5 * (lo + hi) - zFloor;
                    sum += err; maxAbs = Math.Max(maxAbs, Math.Abs(err)); n++;
                }
            Console.WriteLine($"LUT {lut,3} passes {passes,3}          | {tool.Epsilon,6:F4} | {sum / n,8:F4} | {maxAbs,8:F4}");
        }
    }

    // =====================================================================
    // march — doc1 #11 (Lipschitz pessimism) + doc2 #4 (swiss-cheese coarse):
    //          CPU sphere-trace step counts through the real hierarchy
    // =====================================================================
    private static void March()
    {
        const float vox = 0.3f;
        const int N = 512;
        float stock = N * vox;                       // 153.6 mm

        // scenario 1: fresh block (nothing carved)
        var fresh = new SparseSdfVolume(N, vox, 8, 3);

        // scenario 2: heavily carved (order-9 Hilbert, 262k stamplets — millions of max() ops)
        var carved = new SparseSdfVolume(N, vox, 8, 3);
        {
            var tool = ToolProfile.FlatEndmill(3, 12, 64);
            var pts = HilbertPath.Generate(9, stock, 8, stock - 1.5f, 0.5f, 0.02f);
            var segs = HilbertPath.ToSegments(pts, 0, pts.Length - 1);
            for (int s = 0; s < segs.Length; s += 4096)
                carved.StampBatch(segs, s, Math.Min(4096, segs.Length - s), tool, parallel: true);
        }

        // scenario 3: swiss cheese — 400 narrow deep drill holes (r=1 mm < coarse cell 2.4 mm)
        var swiss = new SparseSdfVolume(N, vox, 8, 3);
        {
            var drill = ToolProfile.FlatEndmill(1, 60, 64);
            var rng = new Random(11);
            var plunges = new PathSegment[400];
            for (int i = 0; i < plunges.Length; i++)
            {
                float x = Rand(rng, 15, stock - 15), y = Rand(rng, 15, stock - 15);
                plunges[i] = new PathSegment(new Vector3(x, y, stock - 45), new Vector3(x, y, stock + 1));
            }
            swiss.StampBatch(plunges, 0, plunges.Length, drill, parallel: true);
        }

        Console.WriteLine("scenario | rays    | hit%  | avg steps | p99 | max | krays/s (1 core)");
        MarchScenario("fresh   ", fresh);
        MarchScenario("carved  ", carved);
        MarchScenario("swiss   ", swiss);
    }

    private static void MarchScenario(string label, SparseSdfVolume vol)
    {
        float stock = vol.StockSizeX, vox = vol.VoxelSize;
        var eye = new Vector3(stock * 0.5f, -0.55f * stock, stock * 1.35f);
        var target = new Vector3(stock * 0.5f, stock * 0.5f, stock * 0.72f);
        var fwd = Vector3.Normalize(target - eye);
        var right = Vector3.Normalize(Vector3.Cross(fwd, Vector3.UnitZ));
        var up = Vector3.Cross(right, fwd);
        const int W = 200, H = 200;

        long steps = 0, hits = 0; int maxSteps = 0;
        var perRay = new int[W * H];
        var sw = Stopwatch.StartNew();
        for (int py = 0; py < H; py++)
            for (int px = 0; px < W; px++)
            {
                var dir = Vector3.Normalize(fwd + right * ((px + 0.5f) / W - 0.5f) * 1.1f + up * (0.5f - (py + 0.5f) / H) * 1.1f);
                int s = MarchRay(vol, eye, dir, out bool hit);
                perRay[py * W + px] = s;
                steps += s; if (hit) hits++; if (s > maxSteps) maxSteps = s;
            }
        sw.Stop();
        Array.Sort(perRay);
        Console.WriteLine($"{label} | {W * H,7} | {100.0 * hits / (W * H),5:F1} | {(double)steps / (W * H),9:F1} | {perRay[(int)(W * H * 0.99)],3} | {maxSteps,3} | {W * H / sw.Elapsed.TotalSeconds / 1000,7:F1}");
    }

    private static int MarchRay(SparseSdfVolume vol, Vector3 o, Vector3 d, out bool hit)
    {
        hit = false;
        float stock = vol.StockSizeX, vox = vol.VoxelSize;
        // AABB entry
        float t0 = 0, t1 = float.MaxValue;
        for (int a = 0; a < 3; a++)
        {
            float da = a == 0 ? d.X : a == 1 ? d.Y : d.Z, oa = a == 0 ? o.X : a == 1 ? o.Y : o.Z;
            if (MathF.Abs(da) < 1e-9f) { if (oa < 0 || oa > stock) return 0; continue; }
            float ta = (0 - oa) / da, tb = (stock - oa) / da;
            if (ta > tb) (ta, tb) = (tb, ta);
            t0 = MathF.Max(t0, ta); t1 = MathF.Min(t1, tb);
        }
        if (t0 >= t1) return 0;

        float brickW = vol.BrickSize * vox;
        float t = t0 + 1e-4f;
        int steps = 0;
        float coarseCell = vol.CoarseCell;
        while (t < t1 && steps < 512)
        {
            steps++;
            float px = o.X + d.X * t, py = o.Y + d.Y * t, pz = o.Z + d.Z * t;
            float dc = vol.SampleCoarse(px, py, pz);
            if (dc > coarseCell) { t += dc - 0.5f * coarseCell; continue; }        // far field

            int bx = (int)(px / brickW), by = (int)(py / brickW), bz = (int)(pz / brickW);
            bx = Math.Clamp(bx, 0, vol.BricksX - 1);
            by = Math.Clamp(by, 0, vol.BricksY - 1);
            bz = Math.Clamp(bz, 0, vol.BricksZ - 1);
            int state = vol.BrickState(vol.BrickId(bx, by, bz));
            if (state == SparseSdfVolume.Air)
            {
                // skip to brick exit
                float ex = d.X > 0 ? (bx + 1) * brickW : bx * brickW;
                float ey = d.Y > 0 ? (by + 1) * brickW : by * brickW;
                float ez = d.Z > 0 ? (bz + 1) * brickW : bz * brickW;
                float tx = MathF.Abs(d.X) > 1e-9f ? (ex - o.X) / d.X : float.MaxValue;
                float ty = MathF.Abs(d.Y) > 1e-9f ? (ey - o.Y) / d.Y : float.MaxValue;
                float tz = MathF.Abs(d.Z) > 1e-9f ? (ez - o.Z) / d.Z : float.MaxValue;
                t = MathF.Min(tx, MathF.Min(ty, tz)) + 1e-3f;
                continue;
            }
            if (state == SparseSdfVolume.Solid)
            {
                // crossing air → SOLID brick = hitting an uncarved face (no band bricks exist there;
                // the coarse SDF is what bounds the approach, exactly as in the shader design)
                hit = true;
                return steps;
            }

            float df = vol.SampleFine(px, py, pz);
            if (df < 0)
            {
                // bisection refine (counts as the 2-3 taps of the shader)
                steps += 3;
                hit = true;
                return steps;
            }
            t += MathF.Max(df, 0.35f * vox);
        }
        return steps;
    }

    // =====================================================================
    // staging — doc2 #1: gather dirty 9³ tiles into upload staging buffers
    //          managed-alloc-per-brick vs reused managed vs NativeMemory (+MT)
    // =====================================================================
    private static unsafe void Staging()
    {
        const float vox = 0.3f;
        var vol = new SparseSdfVolume(512, vox, 8, 3) { CoarseEnabled = false };
        var tool = ToolProfile.FlatEndmill(3, 12, 64);
        var pts = HilbertPath.Generate(8, vol.StockSizeX, 8, vol.StockSizeX - 1.5f, 0.5f, 0.02f);
        var segs = HilbertPath.ToSegments(pts, 0, pts.Length - 1);
        for (int s = 0; s < segs.Length; s += 4096)
            vol.StampBatch(segs, s, Math.Min(4096, segs.Length - s), tool, parallel: true);

        // collect real mixed bricks
        var bricks = new List<(int bx, int by, int bz)>();
        for (int bz = 0; bz < vol.BricksZ; bz++)
            for (int by = 0; by < vol.BricksY; by++)
                for (int bx = 0; bx < vol.BricksX; bx++)
                    if (vol.BrickState(vol.BrickId(bx, by, bz)) >= 0) bricks.Add((bx, by, bz));
        Console.WriteLine($"{bricks.Count} mixed bricks available; gathering 500 tiles (9³ = 2916 B) × 600 frames = {500 * 600 * 2916 / 1e6:F0} MB");

        const int perFrame = 500, frames = 600, tile = 9 * 9 * 9;
        var rng = new Random(5);
        var picks = new (int bx, int by, int bz)[perFrame];
        for (int i = 0; i < perFrame; i++) picks[i] = bricks[rng.Next(bricks.Count)];

        // (a) fresh managed Half[] per brick per frame (the GC trap)
        {
            int g0 = GC.CollectionCount(0), g1 = GC.CollectionCount(1);
            var sw = Stopwatch.StartNew();
            long sink = 0;
            for (int f = 0; f < frames; f++)
                for (int i = 0; i < perFrame; i++)
                {
                    var buf = new Half[tile];
                    GatherTile(vol, picks[i], buf.AsSpan());
                    sink += (int)(float)buf[0];
                }
            sw.Stop();
            Report("a) new Half[] per brick   ", sw, g0, g1, sink);
        }
        // (b) one reused managed buffer per frame
        {
            var frameBuf = new Half[perFrame * tile];
            int g0 = GC.CollectionCount(0), g1 = GC.CollectionCount(1);
            var sw = Stopwatch.StartNew();
            long sink = 0;
            for (int f = 0; f < frames; f++)
            {
                for (int i = 0; i < perFrame; i++)
                    GatherTile(vol, picks[i], frameBuf.AsSpan(i * tile, tile));
                sink += (int)(float)frameBuf[0];
            }
            sw.Stop();
            Report("b) reused managed buffer  ", sw, g0, g1, sink);
        }
        // (c) persistent NativeMemory staging (what the GL upload path needs)
        {
            ushort* native = (ushort*)System.Runtime.InteropServices.NativeMemory.Alloc((nuint)(perFrame * tile * 2));
            int g0 = GC.CollectionCount(0), g1 = GC.CollectionCount(1);
            var sw = Stopwatch.StartNew();
            long sink = 0;
            for (int f = 0; f < frames; f++)
            {
                for (int i = 0; i < perFrame; i++)
                    GatherTile(vol, picks[i], new Span<Half>(native + i * tile, tile));
                sink += native[0];
            }
            sw.Stop();
            Report("c) NativeMemory persistent", sw, g0, g1, sink);
            // (d) same, parallel gather
            g0 = GC.CollectionCount(0); g1 = GC.CollectionCount(1);
            sw.Restart();
            for (int f = 0; f < frames; f++)
            {
                ushort* nb = native;
                Parallel.For(0, perFrame, i => GatherTile(vol, picks[i], new Span<Half>(nb + i * tile, tile)));
            }
            sw.Stop();
            Report("d) NativeMemory parallel  ", sw, g0, g1, 0);
            System.Runtime.InteropServices.NativeMemory.Free(native);
        }

        static void Report(string label, Stopwatch sw, int g0, int g1, long sink)
        {
            double mb = 500.0 * 600 * 2916 / 1e6;
            Console.WriteLine($"{label}: {sw.ElapsedMilliseconds,5} ms = {mb / sw.Elapsed.TotalSeconds,7:F0} MB/s | per frame {sw.Elapsed.TotalMilliseconds / 600,6:F3} ms | GC0 +{GC.CollectionCount(0) - g0} GC1 +{GC.CollectionCount(1) - g1} [{sink}]");
        }
    }

    private static void GatherTile(SparseSdfVolume vol, (int bx, int by, int bz) b, Span<Half> dst)
    {
        int bs = vol.BrickSize;
        int x0 = b.bx * bs, y0 = b.by * bs, z0 = b.bz * bs;
        int k = 0;
        for (int z = 0; z < 9; z++)
            for (int y = 0; y < 9; y++)
                for (int x = 0; x < 9; x++)
                    dst[k++] = (Half)vol.GetVoxel(x0 + x, y0 + y, z0 + z);
    }

    // =====================================================================
    // rebake — §4/§6 claims: segment eval cost independent of length, multi-start
    //          handles undercut multi-minima, t-clip + RebakeBrick == live-stamped result,
    //          and the adversarial hot-cell rebake latency ceiling.
    // =====================================================================
    private static void RebakeBench()
    {
        const float vox = 0.3f;
        var tool = ToolProfile.TSlotCutter(1.5f, 4, 2, 12, 128);   // undercut: multi-minima risk
        var rng = new Random(9);

        // 1) length independence + accuracy vs dense exact scan
        foreach (var (label, a, b) in new (string, Vector3, Vector3)[]
        {
            ("short 0.3 mm  ", new(50, 50, 20), new(50.3f, 50, 20)),
            ("long 200 mm sloped", new(20, 30, 18), new(220, 90, 24)),
        })
        {
            var pts = new Vector3[2000];
            for (int i = 0; i < pts.Length; i++)
            {
                float t = (float)rng.NextDouble();
                var q = a + (b - a) * t;
                pts[i] = q + new Vector3(Rand(rng, -5, 5), Rand(rng, -5, 5), Rand(rng, -2, 14));
            }
            double maxErr = 0;
            const float band = 3 * vox;
            foreach (var p in pts.AsSpan(0, 300))
            {
                float got = SweptBrush.EvalSegment(tool, a, b, p.X, p.Y, p.Z, 0, 1);
                float exact = EvalExact(tool, a, b, p, 512) - tool.Epsilon;
                // band-clamped comparison: what stamping actually consumes
                maxErr = Math.Max(maxErr, Math.Abs(Math.Clamp(got, -band, band) - Math.Clamp(exact, -band, band)));
            }
            var sw = Stopwatch.StartNew();
            float sink = 0;
            const int reps = 50;
            for (int r = 0; r < reps; r++)
                foreach (var p in pts) sink += SweptBrush.EvalSegment(tool, a, b, p.X, p.Y, p.Z, 0, 1);
            sw.Stop();
            Console.WriteLine($"1) {label}: {sw.Elapsed.TotalMilliseconds * 1e6 / (reps * pts.Length),7:F0} ns/eval | maxErr vs dense-exact {maxErr,7:F4} mm [{sink:E1}]");
        }

        // 2) rebaked brick == live-stamped volume (long segment unsubdivided in the index)
        {
            var vol = new SparseSdfVolume(128, vox, 8, 3) { CoarseEnabled = false };
            float stock = vol.StockSizeX;                           // 38.4 mm
            var longSeg = new PathSegment(new Vector3(6, 8, 26), new Vector3(32, 30, 28));
            // live: subdivide to stamplets
            int nSub = (int)(Vector3.Distance(longSeg.A, longSeg.B) / (0.95f * vox));
            var live = new List<PathSegment>();
            for (int i = 0; i < nSub; i++)
                live.Add(new PathSegment(Vector3.Lerp(longSeg.A, longSeg.B, i / (float)nSub),
                                         Vector3.Lerp(longSeg.A, longSeg.B, (i + 1) / (float)nSub)));
            var p0 = new Vector3(20, 12, 27);
            var walk = new List<PathSegment>();
            for (int i = 0; i < 250; i++)
            {
                var d = new Vector3(Rand(rng, -1, 1), Rand(rng, -1, 1), Rand(rng, -0.3f, 0.3f));
                var q = p0 + Vector3.Normalize(d) * vox;
                q.X = Math.Clamp(q.X, 8, 30); q.Y = Math.Clamp(q.Y, 8, 30); q.Z = Math.Clamp(q.Z, 24, 30);
                walk.Add(new PathSegment(p0, q)); p0 = q;
            }
            var liveAll = live.Concat(walk).ToArray();
            vol.StampBatch(liveAll, 0, liveAll.Length, tool, parallel: true);

            var original = new[] { longSeg }.Concat(walk).ToArray();   // UNsubdivided in the index
            var idx = new PathIndex(original, stock, 8 * vox, 0f);
            var scratch = new List<int>();
            var brick = new float[512];
            double maxDelta = 0; long evalsTot = 0; int bricksDone = 0;
            var sw = Stopwatch.StartNew();
            foreach (int bId in vol.DirtyBricks.Take(60))
            {
                int bx = bId % vol.BricksX;
                int by = bId / vol.BricksX % vol.BricksY;
                int bz = bId / (vol.BricksX * vol.BricksY);
                var (ev, _) = Rebake.RebakeBrick(idx, original, tool, int.MaxValue,
                    bx * 8 * vox, by * 8 * vox, bz * 8 * vox, 8, vox, vol.BandWorld, brick, scratch);
                evalsTot += ev; bricksDone++;
                for (int v = 0; v < 512; v++)
                {
                    int ix = bx * 8 + v % 8, iy = by * 8 + v / 8 % 8, iz = bz * 8 + v / 64;
                    maxDelta = Math.Max(maxDelta, Math.Abs(vol.GetVoxel(ix, iy, iz) - brick[v]));
                }
            }
            sw.Stop();
            Console.WriteLine($"2) rebake vs live-stamped: {bricksDone} bricks, maxΔ={maxDelta:F4} mm ({maxDelta / vox:F2} voxel), {sw.Elapsed.TotalMilliseconds / bricksDone:F2} ms/brick avg, {evalsTot / bricksDone} evals/brick");
            Console.WriteLine(maxDelta < 2.5f * vox ? "   PASS (≤ stamplet-chord + ε + zTol tolerance)" : "   FAIL — investigate");
        }

        // 3) hot-cell ceiling: adversarial 0.05 mm-stepover raster (≈ 20k refs in one cell)
        {
            float stock = 1024 * vox;
            var fin = new PathSegment[2_000_000];
            const int perLine = 300; const float segLen = 0.05f; int line = 0;
            for (int i = 0; i < fin.Length; i++)
            {
                int k = i % perLine;
                if (k == 0) line++;
                float y = 100 + line * 0.05f % 15, x = 100 + k * segLen;
                float z = 300 - 0.001f * line;
                fin[i] = new PathSegment(new Vector3(x, y, z), new Vector3(x + segLen, y, z));
            }
            var idx = new PathIndex(fin, stock, 8 * vox, 0f);
            float zFloor = 300 - 0.001f * line;                     // deepest cut
            int bxx = (int)(107.5f / (8 * vox)), byy = (int)(107.5f / (8 * vox)), bzz = (int)(zFloor / (8 * vox));
            var brick = new float[512];
            var scratch = new List<int>();
            var flat = ToolProfile.FlatEndmill(3, 12, 128);
            var sw = Stopwatch.StartNew();
            var (ev, used) = Rebake.RebakeBrick(idx, fin, flat, int.MaxValue,
                bxx * 8 * vox, byy * 8 * vox, bzz * 8 * vox, 8, vox, 3 * vox, brick, scratch);
            sw.Stop();
            Console.WriteLine($"3) hot-cell floor brick: {sw.ElapsedMilliseconds} ms | candidates {scratch.Count}, used {used}, evals {ev}");
            Console.WriteLine("   (async view-prioritized replay absorbs this; stroke-run trick applies to level rebake if ever needed)");
        }
    }

    // =====================================================================
    // mesh — Phase 1 remainder: iso-surface export from current volume state
    // =====================================================================
    private static void MeshBench()
    {
        const float vox = 0.3f;
        var volume = new SparseSdfVolume(256, vox, 8, 3);
        var tool = ToolProfile.FlatEndmill(3f, 12f, 128);
        float stock = volume.StockSizeX;

        var pts = HilbertPath.Generate(8, stock, 8f, stock - 1.5f, 0.5f, 0.02f);
        var segs = HilbertPath.ToSegments(pts, 0, pts.Length - 1);
        for (int i = 0; i < segs.Length; i += 4096)
            volume.StampBatch(segs, i, Math.Min(4096, segs.Length - i), tool, parallel: true);

        var sw = Stopwatch.StartNew();
        var mesh = MeshExport.Extract(volume, step: 1);
        sw.Stop();

        Console.WriteLine($"mesh: triangles={mesh.Triangles.Count}, vertices={mesh.Vertices.Count}, normals={mesh.Normals.Count}, ms={sw.Elapsed.TotalMilliseconds:F1}");
        Console.WriteLine(mesh.Triangles.Count > 0 ? "mesh PASS" : "mesh FAIL (empty mesh)");
    }

    // =====================================================================
    // liveheadless — Phase 3 I3 gate: composed headless frame loop throughput
    // =====================================================================
    private static void LiveHeadless()
    {
        const float vox = 0.3f;
        const int targetSt = 70_000;
        const int frames = 3600;
        const int capPerFrame = 1167;

        var vol = new SparseSdfVolume(1024, vox, 8, 3);
        var tool = ToolProfile.FlatEndmill(3, 12, 128);
        float stock = vol.StockSizeX;
        var pts = HilbertPath.Generate(11, stock, 8, stock - 1.5f, 0.5f, 0.02f);
        var segs = HilbertPath.ToSegments(pts, 0, pts.Length - 1);

        var dirty = new DirtyBrickSet(vol.BrickCount);
        int[] drain = new int[1000];
        int cursor = 0;
        long stamped = 0;
        var sw = Stopwatch.StartNew();

        for (int f = 0; f < frames && cursor < segs.Length; f++)
        {
            int take = Math.Min(capPerFrame, segs.Length - cursor);
            vol.StampBatch(segs, cursor, take, tool, parallel: true);
            cursor += take;
            stamped += take;

            foreach (int b in vol.DirtyBricks) dirty.Mark(b);
            _ = dirty.DrainTo(drain);
        }

        sw.Stop();
        double stPerSec = stamped / sw.Elapsed.TotalSeconds;
        bool pass = stPerSec >= 170_000;
        Console.WriteLine($"liveheadless: stamped={stamped}, time={sw.Elapsed.TotalSeconds:F2}s, throughput={stPerSec:F0} st/s (target>=170000) => {(pass ? "PASS" : "FAIL")}");
        Console.WriteLine($"pace check: target workload baseline={targetSt} st/s, processed frames={Math.Min(frames, (int)Math.Ceiling(stamped / (double)capPerFrame))}");
    }
}
