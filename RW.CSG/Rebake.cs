namespace RW.CSG;

/// <summary>
/// Re-bake a brick from the original (unsubdivided) toolpath via PathIndex — the §6 primitive
/// behind LOD zoom, backward scrub and brick eviction. Monotone ⇒ time filter = index compare.
/// </summary>
public static class Rebake
{
    /// <summary>
    /// Rebuild one brick's band-clamped SDF from scratch using segments with index ≤ cursor.
    /// outBrick = bs³ floats. Returns (LUT-ish evals, segments visited after cull).
    /// </summary>
    public static (long evals, int segsUsed) RebakeBrick(
        PathIndex index, PathSegment[] segs, ToolProfile tool, int cursor,
        float bx0, float by0, float bz0, int bs, float vox, float band, float[] outBrick,
        List<int> scratchList)
    {
        float bx1 = bx0 + bs * vox, by1 = by0 + bs * vox, bz1 = bz0 + bs * vox;
        Array.Fill(outBrick, -band, 0, bs * bs * bs);

        // gather candidate segments: index stores TIGHT AABBs ⇒ expand the query box
        float pad = tool.Rmax + band + tool.Epsilon;
        float zPadLo = tool.Zmax + band + tool.Epsilon;   // segment z can be this far ABOVE the box
        float zPadHi = -tool.Zmin + band + tool.Epsilon;  // ... and this far below
        int n = index.CellsPerSide;
        float inv = 1f / index.CellSize;
        int cx0 = Math.Clamp((int)((bx0 - pad) * inv), 0, n - 1), cx1 = Math.Clamp((int)((bx1 + pad) * inv), 0, n - 1);
        int cy0 = Math.Clamp((int)((by0 - pad) * inv), 0, n - 1), cy1 = Math.Clamp((int)((by1 + pad) * inv), 0, n - 1);
        int cz0 = Math.Clamp((int)((bz0 - zPadLo) * inv), 0, n - 1), cz1 = Math.Clamp((int)((bz1 + zPadHi) * inv), 0, n - 1);
        scratchList.Clear();
        for (int cz = cz0; cz <= cz1; cz++)
            for (int cy = cy0; cy <= cy1; cy++)
                for (int cx = cx0; cx <= cx1; cx++)
                    foreach (int s in index.QueryCell(cx, cy, cz))
                        if (s <= cursor) scratchList.Add(s);
        scratchList.Sort();

        float ccx = (bx0 + bx1) * 0.5f, ccy = (by0 + by1) * 0.5f, ccz = (bz0 + bz1) * 0.5f;
        float ballR = 0.8660254f * bs * vox;
        int nVox = bs * bs * bs;

        // --- cull + t-clip candidates (path order preserved) ---
        int cand = 0;
        var candSeg = new int[scratchList.Count];
        var candMin = new float[scratchList.Count];
        var candT = new (float lo, float hi)[scratchList.Count];
        int prev = -1;
        for (int li = 0; li < scratchList.Count; li++)
        {
            int s = scratchList[li];
            if (s == prev) continue;                       // dedupe (sorted)
            prev = s;
            ref readonly var seg = ref segs[s];
            if (!SweptBrush.ClipSegmentT(tool, seg.A, seg.B, bx0, by0, bz0, bx1, by1, bz1, band,
                                         out float tLo, out float tHi)) continue;
            var ca = seg.A + (seg.B - seg.A) * tLo;
            var cb = seg.A + (seg.B - seg.A) * tHi;
            SweptBrush.ConservativeRange(tool, ca, cb, ccx, ccy, ccz, ballR, out float dmin, out _);
            if (dmin >= band) continue;
            candSeg[cand] = s; candMin[cand] = dmin; candT[cand] = (tLo, tHi); cand++;
        }

        // --- group consecutive LEVEL candidates into runs (stroke-run, same as live stamping) ---
        float zTol = 0.10f * vox;
        int nRuns = 0;
        var runStart = new int[Math.Max(1, cand)];
        var runEnd = new int[Math.Max(1, cand)];
        var runKey = new float[Math.Max(1, cand)];
        var runLevel = new bool[Math.Max(1, cand)];
        var runZ = new float[Math.Max(1, cand)];
        for (int i = 0; i < cand;)
        {
            ref readonly var s0 = ref segs[candSeg[i]];
            float zRef = s0.A.Z;
            bool level = MathF.Abs(s0.A.Z - s0.B.Z) <= zTol
                && tool.IsRadiallyMonotone(
                    bz0 + 0.5f * vox - (zRef + zTol),
                    bz1 - 0.5f * vox - (zRef - zTol));
            int j = i + 1;
            if (level)
                while (j < cand && j - i < 16 && candSeg[j] == candSeg[j - 1] + 1)
                {
                    ref readonly var sn = ref segs[candSeg[j]];
                    if (!(MathF.Abs(sn.A.Z - zRef) <= zTol && MathF.Abs(sn.B.Z - zRef) <= zTol)) break;
                    j++;
                }
            float key = float.MaxValue;
            for (int k = i; k < j; k++) if (candMin[k] < key) key = candMin[k];
            runStart[nRuns] = i; runEnd[nRuns] = j; runKey[nRuns] = key;
            runLevel[nRuns] = level; runZ[nRuns] = zRef; nRuns++;
            i = j;
        }
        // deepest-first ordering + adaptive stop: once no remaining run can raise the current
        // brick minimum, the rest of the tail is provably a no-op (monotone max).
        var runOrder = new int[nRuns];
        for (int i = 0; i < nRuns; i++) runOrder[i] = i;
        Array.Sort(runKey, runOrder, 0, nRuns);

        int unsaturated = nVox;
        float brickMin = -band;                            // min over voxels (monotone rising)
        bool minDirty = false;
        long evals = 0;
        int used = 0;
        float colSkipR = tool.Rmax + band + tool.Epsilon;
        int bs2 = bs * bs;

        for (int oi = 0; oi < nRuns && unsaturated > 0; oi++)
        {
            if (minDirty)
            {
                brickMin = float.MaxValue;
                for (int k = 0; k < nVox; k++) if (outBrick[k] < brickMin) brickMin = outBrick[k];
                minDirty = false;
            }
            if (-runKey[oi] <= brickMin + 1e-6f) break;    // sorted ⇒ nothing later raises anything

            int r = runOrder[oi];
            int i0 = runStart[r], i1 = runEnd[r];
            used += i1 - i0;

            if (runLevel[r])
            {
                // column-memoized: one 2D distance to the run's clipped polyline per column,
                // then one LUT sample per engaged voxel; columns use the z interpolated at
                // their closest polyline point (matches live stamping, no plateau quantization)
                float zRef = runZ[r];
                for (int y = 0; y < bs; y++)
                {
                    float py = by0 + (y + 0.5f) * vox;
                    for (int x = 0; x < bs; x++)
                    {
                        float px = bx0 + (x + 0.5f) * vox;
                        float r2min = float.MaxValue;
                        float zCol = zRef;
                        for (int k = i0; k < i1; k++)
                        {
                            ref readonly var sk = ref segs[candSeg[k]];
                            (float tLo, float tHi) = candT[k];
                            float ax = sk.A.X + (sk.B.X - sk.A.X) * tLo, ay = sk.A.Y + (sk.B.Y - sk.A.Y) * tLo;
                            float az = sk.A.Z + (sk.B.Z - sk.A.Z) * tLo;
                            float bxx = sk.A.X + (sk.B.X - sk.A.X) * tHi, byy = sk.A.Y + (sk.B.Y - sk.A.Y) * tHi;
                            float bzz = sk.A.Z + (sk.B.Z - sk.A.Z) * tHi;
                            float ex = bxx - ax, ey = byy - ay;
                            float wx = px - ax, wy = py - ay;
                            float ee = ex * ex + ey * ey;
                            float t = ee > 1e-12f ? Math.Clamp((wx * ex + wy * ey) / ee, 0f, 1f) : 0f;
                            float qx = wx - ex * t, qy = wy - ey * t;
                            float d2 = qx * qx + qy * qy;
                            if (d2 < r2min) { r2min = d2; zCol = az + (bzz - az) * t; }
                        }
                        float rmin = MathF.Sqrt(r2min);
                        if (rmin >= colSkipR) continue;
                        int k2 = y * bs + x;
                        float pz = bz0 + 0.5f * vox;
                        for (int z = 0; z < bs; z++, k2 += bs2, pz += vox)
                        {
                            float d = outBrick[k2];
                            if (d >= band) continue;
                            evals++;
                            float nb = -(tool.Sample(rmin, pz - zCol) - tool.Epsilon);
                            if (nb > d)
                            {
                                float nd = nb > band ? band : nb;
                                outBrick[k2] = nd;
                                if (nd >= band) unsaturated--;
                                if (d <= brickMin + 1e-6f) minDirty = true;
                            }
                        }
                    }
                }
            }
            else
            {
                for (int k = i0; k < i1; k++)
                {
                    ref readonly var seg = ref segs[candSeg[k]];
                    (float tLo, float tHi) = candT[k];
                    float raiseCap = -candMin[k];          // this segment can never write above this
                    int v = 0;
                    for (int z = 0; z < bs; z++)
                    {
                        float pz = bz0 + (z + 0.5f) * vox;
                        for (int y = 0; y < bs; y++)
                        {
                            float py = by0 + (y + 0.5f) * vox;
                            for (int x = 0; x < bs; x++, v++)
                            {
                                float d = outBrick[v];
                                if (d >= band || raiseCap <= d) continue;
                                float px = bx0 + (x + 0.5f) * vox;
                                evals++;
                                float nb = -SweptBrush.EvalSegment(tool, seg.A, seg.B, px, py, pz, tLo, tHi);
                                if (nb > d)
                                {
                                    float nd = nb > band ? band : nb;
                                    outBrick[v] = nd;
                                    if (nd >= band) unsaturated--;
                                    if (d <= brickMin + 1e-6f) minDirty = true;
                                }
                            }
                        }
                    }
                }
            }
        }
        return (evals, used);
    }
}
