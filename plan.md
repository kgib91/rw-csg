# RW.CSG — Real-Time Material Removal: Architecture Plan

**Status: PHASE 0 COMPLETE — every part proven individually; integration next.**
All Phase-0 gates passed on the reference machine (benches B1–B4 plus nine concern-specific
PoCs; two external review rounds adjudicated with measurements, §10/§10b). The measured
baselines form the performance contract in §12; Phases 1–3 assemble only proven parts along
the integration pipeline in §12b, each step with its own mini-gate. PoC-hardened library
primitives already live in `RW.CSG` (see §13).
This document is the single source of truth for the design. Update it when decisions change.

---

## 1. Goal

- Reusable **C# library** (`RW.CSG`, zero UI dependencies) that subtracts the swept
  volume of a machining tool moving along a toolpath from a solid stock block.
- Tools are **revolved profiles with undercuts** (T-slot, dovetail, lollipop, flanged shells) —
  a 2.5D heightmap is therefore ruled out; the representation must be true 3D.
- Scale target: **millions of path segments**, animated material removal over time at **60+ FPS**.
- Demo app (`RW.CSG.Avalonia`): GUI + 3D preview, orbit camera, Hilbert-curve toolpath in XY with
  vertical Z modulation over a 200 × 200 × 10 mm stock, play/pause plus exact timeline seeking,
  and live carving animation.

## 2. Why this architecture (survey conclusions)

Every system that ships real-time material removal converges on the same fixed point:
**persistent discretized field + monotone local updates + hierarchy.**

| System | Representation | Lesson |
|---|---|---|
| CAMotics (OSS) | Analytic sweeps + octree + marching cubes, re-evaluated lazily | Analytic per-move SDFs work; lazy evaluation over history does NOT scale |
| ModuleWorks / MachineWorks / Vericut | Tri-dexel interval fields, incremental clipping | Industry proof of persistent-field + incremental updates |
| OpenVDB / NanoVDB | Narrow-band sparse level-set bricks | The right data structure; we implement a VDB-lite in C# |
| Claybook (GDC 2018) | GPU brick-pool SDF, dirty-brick edits, sphere tracing | Proof of 60 FPS fully-dynamic SDF world; our rendering blueprint |
| Dreams (Media Molecule) | Edit lists + lazy brick evaluation at any resolution | Source of the PathIndex idea (§6) |
| Manifold (mesh booleans) | Exact n-ary CSG | Offline export only; dies on per-frame animation |
| libfive / MPR (Keeter 2020) | Interval-pruned implicit DAG, exact | Beautiful, but needs GPU compute + scales with clause density; wrong tool for 60 FPS carving |
| Image-space CSG / Van Hook 1986 | Per-pixel interval lists | Persistence insight is right; camera-locked version of the dexel field |

**Optimality argument:** subtraction is monotone (material only shrinks), so history never needs
re-evaluation — each segment is baked exactly once. A resolution-bounded sparse field gives
O(1) sample queries, O(new-surface-area) updates, memory O(observable surface), and render cost
**independent of segment count**. All four are asymptotically unbeatable; the only remaining risk
was constant factors — now measured in Phase 0 (§10b): the constants clear the gate 2.9×.

## 3. Core data model (`SparseSdfVolume`)

Example scale: 300 mm stock @ 0.3 mm voxel → 1024³ virtual voxels.

```
SparseSdfVolume
├─ brickIndex : int[128³]                 // dense brick-handle grid, 8 MB
│    -1 = AIR (fully cut), -2 = SOLID (untouched material), else handle into brickPool
├─ brickPool  : slab-allocated Half[8³]   // ONLY boundary bricks exist (~100k ≈ 100 MB)
├─ coarseSdf  : Half[128³]                // whole-volume 1/8-res SDF — LOD + ray skipping (4 MB)
├─ removedVolume : double                 // accumulated during stamping, O(1) query
└─ dirtyQueue : brick handles             // consumed by renderers via event/queue
```

- **Sign convention:** `d > 0` = air, `d < 0` = material. **All distances are physical
  millimeters at every stage** (profile LUT, bricks, coarse grid, renderer); `Half` stores mm
  directly — at band scale (≤ ±0.9 mm) a Half ULP is ≈ 0.0005 mm, far below voxel size.
- **Narrow band:** distances clamped to ±B voxels (B = 3). `max()` of clamped
  values equals clamped `max` (monotone) — clamping is CSG-safe.
- **Subtract op:** `d_new = max(d, −d_brush)` — commutative, associative, idempotent ⇒
  order-independent ⇒ parallel-safe with per-brick ownership, deterministic results.
  Voxels are rewritten only when raised by more than `WriteEpsilon` (≈ 1 Half ULP at band
  scale): prevents Half round-down slivers from re-registering as removed volume
  (measured: 200× idempotent re-stamp drift 0.21 % → exactly 0 %).
- **Storage:** compute in `float`, store `Half` (F16C hardware conversion). 8-bit SNORM is the
  fallback if GPU upload bandwidth demands it (Claybook shipped 8-bit).
- Slab allocation (e.g., 1024 bricks/slab) → no GC pressure, stable memory, `Span` friendly,
  and enables copy-on-write snapshots (§8).

### GPU mirror (demo app)
- 3D **atlas texture** of 9³ tiles = 8³ unique voxels + **1-voxel duplicated border** for
  seam-free hardware trilinear filtering.
- Small RGBA8 **indirection texture** (one texel per brick slot): AIR / SOLID / atlas uvw.
- Coarse SDF as a small 3D texture.
- All features used are **core OpenGL ES 3.0** (ANGLE default backend): `sampler3D`,
  `texSubImage3D`, instancing, `glBlendEquation(GL_MIN)`. No compute shaders required anywhere.

## 4. Tool + brush (analytic, undercut-capable)

A **stamplet** is a toolpath segment pre-subdivided to ≈ one voxel length, so a small-K sweep
sample is exact at voxel scale. **Only cutting moves are stamped: rapids / non-cutting moves are
excluded before subdivision** (a 500 mm G0 must never become 1700 stamplets).

```
ToolProfile.Bake():                       // once at init
  profileSdf2D = 128×128 signed-distance grid over the closed (r,z) profile polygon,
                 baked from the axis-MIRRORED polygon — exact SDF for solids of revolution
                                          // undercuts = concave polygon — fully supported
  Rmax, Zmin..Zmax, Lipschitz bound (=1), MonotoneR flag (SDF non-decreasing in r at each z)

SweptBrushSdf(p, A, B):                   // tool axis vertical (3-axis machining, v1)
  // general/fallback evaluator; the hot loop uses the stroke-run form below (§5)
  best = ∞
  for t in {0, ½, 1}:                     // K=3 + parabolic refine ≈ exact at stamplet scale
    q = p − lerp(A, B, t)
    best = min(best, bilinear(profileSdf2D, length(q.xy), q.z))
  return refine(best) − ε                 // ε = half LUT cell: safety against overestimation
```

- **No mesh booleans anywhere.** Accuracy = voxel size; undercuts native.
- **ε margin:** bilinear LUT interpolation can *overestimate* distance near concave profile
  corners — overestimation breaks sphere tracing. Subtract half-LUT-cell ε; the renderer's final
  hit uses **sign-change bisection** (2–3 taps), which only needs sign correctness.
  **Measured (PoC):** the ε offset is a *constant* bias of exactly ε — it does **not** stack over
  overlapping passes (max is idempotent; 1/10/100 passes give identical surfaces). Surface
  offset by LUT size: 32 → 0.258 mm, 64 → 0.127, 128 → 0.063, 256 → 0.031; eval cost is
  LUT-size-independent (cache-resident) ⇒ **default LUT = 128** (≈ ¼ voxel bias), 256 for
  metrology builds.
- **Long segments** (straight cuts): for *live stamping*, pre-subdivide into voxel-scale
  stamplets (then re-merged into strokes by the stamper, §5). For *re-bake via PathIndex*,
  evaluate the original segment with bracketed multi-start minimization over the **t-clipped**
  window (concave profiles can have multiple local minima — multi-start handles it). The seed
  count scales with the clipped window's world length (≈2 per tool radius); after clipping the
  window is bounded by the brick slab, so cost is independent of the original segment length.
  Measured: clipped accuracy 0.002 mm; a fixed-K eval on an UNclipped 200 mm segment misses
  minima by 4 mm — clip first, always (§9.3).

## 5. Stamping — the hot loop

Two tricks, both PoC-measured (§10b):
1. **Invert the loop — bin segments to bricks, then parallelize over bricks.** Brick data stays
   in L1/L2 across all segments touching it, zero locks, deterministic.
2. **Brick-local Z-bucket column decomposition** (vertical tools with `MonotoneR` profiles —
  all common cutters): because subtraction is a commutative max, every surviving level
  stamplet in a brick at approximately the same z (0.1 voxel buckets, folded into the ε
  budget) is unioned regardless of path adjacency. Each segment visits only XY voxel columns
  inside its radius-expanded AABB; each column then takes **one LUT sample per voxel** from
  its nearest segment. Expensive voxel work is therefore
  `O(segment-column overlaps + occupied Z buckets × brick voxels)`, not
  `O(short path runs × brick voxels)`. Sloped or non-monotone cases retain the exact
  per-stamplet `SweptBrushSdf` fallback.

```
StampBatch(segments[], tool):                 // called once per frame
  1. BIN: counting-sort segment→brick overlaps into flat arrays (zero alloc, O(Σ overlaps))
  2. PARALLEL over touched bricks (each brick owned by exactly one worker):
       state = brickIndex[b]
       if state == AIR: continue                        // nothing left to remove
       load brick to float[] scratch once (Half→float; SOLID ⇒ fill −B)   // amortized over batch
       prescan bins[b]:
         (dmin, dmax) = conservativeRange(s, brickCenter, brickRadius)
             // sphere lower bound + one-LUT-sample upper bound, ~32 ns (measured);
             // rejects ~92 % of pairs on a dense path — essential, cost amortized by runs
         if dmin > +B: skip (miss)
         if dmax < −B: state = AIR; removed += occupancy(brick); break    // engulf: deep plunges
         else append to survivors
       group level survivors by brick-local z bucket (subtraction order is irrelevant):
         for each segment, update only XY columns in its active-radius-expanded AABB
         for each engaged XY column: rmin = nearest grouped segment; skip if > activeR+B+ε
           for each z voxel: d' = −(profile(rmin, z−zRef) − ε)
             if d' > d + WriteEpsilon: accumulate removed Δoccupancy; write
       reclassify: all ≥ +B → AIR, all ≤ −B → SOLID (free the brick back to the pool);
         alloc happens only at writeback ⇒ no intra-batch pool churn
       stamp same brushes into coarseSdf cells          // ~17 % measured (parallel, atomic max)
       mark dirty(b); if a border plane changed → also mark face-neighbors dirty (§9.1)
  3. removedVolume += Σ per-brick deltas, summed in brick order (deterministic)
```

**Removed volume** is the sum of per-voxel occupancy deltas, `occ(d) = clamp01(½ − d/voxel)`
(linear Heaviside over one voxel). Measured: 0.04 % vs analytic on a slot cut; exactly zero
drift under idempotent re-stamping with `WriteEpsilon` (§3). It is a sub-voxel *approximation*,
not exact metrology; engulfed bricks integrate their occupancy before collapsing to AIR.

**Brick states cannot oscillate** (doc-review concern, disproved structurally + adversarial
bench): monotonicity forces SOLID → mixed → AIR one-way; a freed-to-AIR brick is never
re-allocated (AIR early-out), grazing cuts allocate each brick exactly once. No hysteresis
needed. The only repeatable cost is refilling a SOLID brick that reclassifies back to SOLID
(2 KB memset — negligible, a touched-but-still-solid flag can remove it if it ever shows up).

## 6. PathIndex — the toolpath IS the database (Dreams' edit-list idea + monotonicity)

```
PathIndex: static uniform grid over stock, cell size = brick size, built once in O(N)
  cell → [ORIGINAL segment indices intersecting it]     // unsubdivided
  cells cover the segment's TIGHT AABB only; queries expand their box by (Rmax + band + ε)
  // measured @4.19M segments: tight = 26 MB vs build-expanded = 978 MB (37×) — the expansion
  // side is the memory decision. Adversarial 0.05 mm-stepover raster in one 15×15 mm patch:
  // still 25 MB, but hot cells reach ~20k refs — re-bake cost concentrates there (locality
  // bound k stays true per pass; scrubbing into such a patch is the slow-path ceiling).

RebakeBrick(b, resolution, cursor):          // ANY brick, ANY resolution, ANY playback time
  list = PathIndex.Query(bounds(b))          // O(k), k = passes through brick, not stamplets
  brick = SOLID
  cull + t-clip candidates (path order); group consecutive LEVEL candidates into stroke RUNS;
  order runs DEEPEST-FIRST (by conservative bound) and stop once no remaining run can raise
  the brick minimum (monotone max); level runs use the same column-memoized evaluation as
  live stamping; sloped/non-monotone segments fall back to the bracketed multi-start
  segment evaluator over their CLIPPED t-window (adaptive seed count ≈ 2 per tool radius
  of window length — fixed seeds on an unclipped long segment miss minima, measured 4 mm).
```

**The run/ordering path is NOT optional** (measured): a naive per-segment rebake of a floor
brick in the adversarial dense-overlap cell took **421 s** (839k candidate refs; a tall tool
defeats the z-cull; below-floor voxels never saturate). With run grouping + deepest-first +
adaptive stop: **0.43 s** (990×) — and typical path bricks re-bake in **1.8 ms** with
max deviation from the live-stamped field of 0.09 mm (0.29 voxel, inside the ε+zTol budget).

What this one structure buys:
- **True LOD zoom** (Phase 5): re-bake visible bricks at 2×/4× resolution on approach — detail is
  recomputed from the exact path, not stored. Resolution is a per-view choice.
- **Backward scrubbing** without full re-simulation (§8).
- **Bricks become a cache, not the truth**: evictable under memory pressure, rebuildable on
  demand — graceful degradation, door to out-of-core scale.
- **Exact metrology**: point queries can brute-force the local brush list analytically.

The live animation path never consults the index (pure incremental stamping of new stamplets).

## 7. Rendering (sphere-traced brick atlas, ES 3.0 fragment shader)

```
frag(ray):
  enter stock AABB
  march coarseSdf with step (d − coarseVoxel margin)     // far field / LOD / safety (§9.4)
  when near surface, per step:
    indirection lookup at brickOf(p):
      AIR   → jump to brick exit (empty-space skip)
      SOLID → coarse grid handles it
      brick → sphere-trace trilinear atlas samples (steps ≤ band = 3 voxels)
  on sign change: bisection refine (2–3 taps) → hit point
  normal = 6-tap central-difference gradient
  shade: N·L, AO from coarse-mip taps, cut-surface tint vs original skin
  distant pixels: stop at coarse surface entirely (screen-space LOD by cone footprint)
```

- Tool ghost: analytic profile SDF (2D LUT texture) at current pose, min()'d in shader.
- Step hierarchy (coarse → AIR-brick jumps → ≤3-voxel band) is what bounds march cost;
  brick values are never used for far-field stepping (see §10.4).

### Frame loop

```
Frame(dt):
  segs = path.Consume(dt × speed, capBudget)      // budget guards frame time; rapids skipped
  volume.StampBatch(segs)                         // frame-scoped Parallel.For (no free threads v1)
  upload dirty bricks (≤ uploadCapMB/frame):
    dirty is a SET keyed by brick — newest-wins: re-dirtied bricks are uploaded once at their
    latest state; backlog can never exceed live surface bricks. If the cap defers a brick,
    it stays queued; target max upload latency ≈ 3 frames, coarse grid uploads first so the
    far field is never stale.
    gather 9³ tiles (8³ + neighbor shells) in parallel into a PERSISTENT NativeMemory staging
    buffer (measured 1.0 GB/s parallel; zero GC) → texSubImage3D from the raw pointer.
    MEASURED (B3, ANGLE→D3D11): scattered tile uploads are PER-CALL dominated (~6 µs/call,
    0.25 GB/s) vs 2.3–2.7 GB/s bulk — budget ≈ 1000 tiles/frame; the demo needs ~103 → 0.6 ms.
    R8 is NOT faster than R16F at these sizes (call floor) → keep R16F. Indirection updates:
    batch the whole 64³ RGBA8 texture as ONE subimage (~0.5 ms) — never per-texel calls (4 µs each).
  upload coarse dirty region
  draw raymarch pass + tool ghost + HUD (fps, segs/s, brick count, MB uploaded)
```

**Measured (B4):** the full §7 hierarchy shader on a carved 512³ scene at true 1920×1080 runs
**4.7 ms/frame (213 FPS) on an Intel Iris Xe iGPU** via ANGLE→D3D11 — dynamic loops and
dependent fetches are fine; ~12 ms/frame of headroom remains for stamping + upload at 60 FPS.

**Avalonia+ANGLE context rules (measured the hard way, Phase 2 must follow):**
- `OnOpenGlInit` and `OnOpenGlRender` run on **different share contexts**: textures/buffers/
  programs are shared; **FBOs and VAOs are container objects and are not** — create them lazily
  on the render context.
- **User-created FBOs black-hole silently** in this stack (clears/draws no-op, zero GL errors,
  for both texture and renderbuffer attachments) — **render into Avalonia's control FBO**
  (the `fb` argument), which is the production path anyway.
- Neutralize inherited state every frame (scissor/cull/depth/blend/stencil/rasterizer-discard/
  dither off, colorMask true, unbind PIXEL_PACK/UNPACK) — Skia leaves these live (§9.7).
- Attribless `gl_VertexID` fullscreen triangles do not draw here — use a real VBO + attribute.

**Atlas sizing:** the full rectangular viewer path reaches 154,048 simultaneously MIXED bricks.
At each former capacity boundary, every mapped slot was still active (`reclaimed=0`), so this is
real surface residency rather than stale mappings. The viewer reserves one 56³-tile atlas
(504³ R16F, 256 MB) at startup; ANGLE reserves it in about 1 ms, and playback never relocates or
duplicates it. Slots are nevertheless returned to a free stack whenever a brick becomes AIR or
SOLID, which helps jobs whose active surface recedes. The old incremental 32³→…→56³ growth path
caused 166–429 ms render-thread stalls and transiently held old atlas + new atlas + full CPU staging
(up to roughly 717 MB at the last growth). It remains only as an emergency fallback beyond the
measured viewer capacity.

**Atlas sampling (exact, half-voxel-shifted regions):** each 9³ tile stores voxels 0..8 of its
brick — texel 8 duplicates the +X/+Y/+Z neighbor's first voxel plane. The shader shifts the
region lookup by half a voxel: `vg = p/vox − 0.5`, `cell = floor(vg/8)`, in-tile texel
`vg − 8·cell + 0.5` ∈ [0.5, 8.5) — every trilinear pair lies inside one tile, so the result is
bit-identical to global trilinear interpolation across brick faces (no leak, no seam, no clamp
sliver). The uniform-brick classification for the shifted half-voxel sliver is sign-safe because
band ≥ 1 voxel. Rejected: the earlier `(1.5+7x)/9` interior squeeze misplaced samples by ±½
voxel in a per-brick sawtooth and left ~1-voxel discontinuities at faces → visible brick-grid
outlines on shallow floors. Dirty propagation must include NEG-side edge/corner neighbors, not
just faces (a voxel-(0,0,0) write is duplicated in 7 neighbor tiles); validate with the Phase-2
seam test (§9.1).

Threading: render thread owns GL; stamping is a frame-scoped parallel-for (deterministic,
no cross-thread GL). Free-running stamp workers only if profiling ever justifies it.

## 8. Time scrubbing & snapshots (Phase 5)

Large backward jumps touch 10⁴–10⁵ bricks; pure re-bake is seconds-scale. Design:
- **Copy-on-write snapshots at slab granularity** — we already track dirty bricks; a snapshot
  costs O(bricks touched since last snapshot); unchanged slabs shared. ~1–2× pool size total
  for ~8 snapshots (~100–200 MB), spaced adaptively by dirty-bytes (not segment count).
- Scrub-back = instant swap to nearest snapshot ≤ cursor, then **async, view-prioritized delta
  replay** from PathIndex — starts from a coherent nearby state, never an empty scene.
- v1 demo ships forward playback + restart only; this section is the agreed later design.

## 9. Correctness traps (called out in advance — the "zero problems" list)

1. **Border staleness:** a stamp changing brick X's edge plane invalidates the duplicated border
   voxels in face-neighbor tiles → mark face-neighbors dirty too, else visible seams.
2. **Clamped max stays valid** (monotone), but never re-derive sign from clamp boundaries.
3. **Long segments** must be subdivided (live) or t-clipped + 1D-minimized (re-bake); a K-sample
   sweep eval on a long segment silently misses material.
4. **Coarse-grid stepping margin:** coarseSdf is stamped, not downsampled from fine — step with
   `d − coarseVoxel` safety margin.
5. **Zero-alloc binning:** counting sort into flat arrays; no Dictionary/List in the hot path.
6. **Half arithmetic is software in C#:** compute float, store Half (F16C conversion is cheap).
7. **Avalonia GL guards** (carried from prior work): skip render when FBO ≤ 0, restore neutral GL
   state in finally, capture GL interface into a local.
8. **Mixed-resolution LOD seams** (Phase 5): raymarching turns them into bounded shading steps,
   not holes. Fix: ±1 LOD delta rule between neighbors + fill fine-brick border shells by
   sampling the coarse neighbor's interpolant (one-way read after bake → C0-continuous faces).
   Border-gather code must be written against a pluggable source from day one.
9. **Half round-down drift:** re-stamping identical geometry raises voxels by sub-ULP slivers
   → phantom removed volume (+0.21 %/200 re-stamps measured). Fix: `WriteEpsilon` ≈ 1 Half ULP
   at band scale (§3) — measured drift exactly 0.
10. **Mesh export & metrology read from an immutable snapshot** (COW slab snapshot, §8) — never
    from the live volume while stamping; export while carving is otherwise a torn read.
11. **Upload staging must be unmanaged and persistent** (`NativeMemory.Alloc`): per-brick managed
    arrays measurably thrash gen0; pinning per frame adds copies at the P/Invoke boundary (§7).

## 10. External review (Gemini) — adjudicated

1. **"k explodes in dense finishing passes"** — direction right, arithmetic off 64× (assumed 32³
   bricks; ours are 8³; index cells are brick-sized, not 10 mm). Adopted fix: PathIndex stores
   *unsubdivided segments* + t-clipping (§6), so re-bake k scales with passes (~50), not
   stamplets (~500). Lipschitz range-culling rejects non-survivors cheaply (measured 32 ns/test,
   see §10b.2).
2. **"LOD seams"** — real but overblown for sphere tracing (bounded shading artifact, not cracks).
   Cheap fix adopted (§9.8). Phase 5 scope.
3. **"Async scrub hitching → need snapshots"** — agreed, and understated (10⁴–10⁵ bricks, not 500).
   Adopted: COW slab snapshots + view-prioritized delta replay (§8).
4. **"Eikonal degradation → need GPU redistancing/JFA"** — rejected for this design:
   max/min of 1-Lipschitz fields stays 1-Lipschitz (sphere tracing stays *safe*, only
   pessimistic); the ±3-voxel band clamp means far-field marching never reads brick values —
   it is driven by the coarse grid + AIR-brick skips (§7). A redistancing pass would repair
   values the renderer never consults. The *real* adjacent risk — brush LUT overestimation near
   concave corners — is fixed with the −ε margin + bisection (§4). No JFA subsystem.

## 10b. Second review round (2 docs) — adjudicated by PoC benches (2026-07)

All CPU-side concerns were settled empirically (`--bench` suite, 20-core machine, Release,
ServerGC). Verification gate: stamped grid vs brute-force exact brush-list → max error =
ε exactly (the designed bias), so the numbers below are from a *correct* implementation.

| # | Concern | Verdict | Evidence → plan change |
|---|---|---|---|
| 1 | Sweep eval cost is the primary unknown; memory-latency-bound (doc1#1) | **Half right** | Eval itself is cheap and cache-resident: 60 ns ST, 165 M evals/s MT, LUT64≡LUT256. But naive per-stamplet stamping needed 12.9k evals/stamplet → only 11 k st/s. **Stroke-run column stamping (§5) → 204 k st/s sustained on the full 1M path — gate passed 2.9× on CPU.** |
| 2 | Conservative range not "10 flops" (doc1#2) | **Confirmed, absorbed** | 32 ns/call (≈½ eval); rejects 92 % of pairs on dense paths — essential, and its cost is amortized by run merging. Bound = sphere lower + one-LUT-sample upper. |
| 3 | Removed-volume accumulation drift (doc1#3) | **Confirmed → fixed** | Occupancy-delta method: 0.04 % vs analytic slot. Idempotent re-stamp drift +0.21 % → `WriteEpsilon` ⇒ exactly 0 (§9.9). Documented as sub-voxel approximation. |
| 4 | AIR/SOLID reclassification oscillation (doc1#4) | **Disproved** | Structural: monotone field ⇒ SOLID→mixed→AIR is one-way; adversarial grazing bench: 520 allocs (once/brick), **0 frees**, no churn. No hysteresis subsystem. |
| 5 | PathIndex memory at 4M segments (doc1#5) | **Confirmed for naive layout → solved** | Build-expanded: 978–1332 MB. Tight-AABB + query-expand: **25–26 MB** (§6). Worst-case hot cells (~20k refs) noted as re-bake slow-path ceiling. |
| 6 | Why 8³ bricks (doc1#6) | **Measured** | 8³ 195.6 k st/s > 4³ 174.8 (index overhead, more borders) > 16³ 171.5 (worse cull granularity, 0 % miss rate). 8³ stands. |
| 7 | Band 3 vs 4 (doc1#7) | **Measured** | B=3: 195.6 k st/s; B=4: 166.2 (+40 % evals, more bricks). B=3 stands; normals judged in Phase 2. |
| 8 | Long rapids explode into stamplets (doc1#8) | **Adopted** | Non-cutting moves excluded before subdivision (§4). |
| 9 | Upload queue unbounded (doc1#9) | **Adopted** | Newest-wins dirty *set*, backlog bounded by live surface, ~3-frame max latency, coarse-first (§7). CPU side measured: ~103 dirty bricks/batch ≈ 0.3 MB/frame at 70k pace. |
| 10 | Mesh export vs live carving (doc1#10) | **Adopted** | Export reads an immutable COW snapshot (§9.10). |
| 11 | Lipschitz "safe ≠ accurate", step growth (doc1#11) | **Bounded, measured** | Hierarchical CPU trace after 262k stamplets (millions of max ops): avg 4.6 steps/ray, p99 22, max 25 — no explosion (band clamp + coarse grid keep far field independent of composition history). |
| 12 | P/Invoke staging: pinned/unmanaged buffers (doc2#1) | **Confirmed → adopted** | Per-brick managed arrays: 171 gen0/600 frames. Persistent NativeMemory + parallel gather: 1.0 GB/s, 0 GC — ≥300× the required 1–3 MB/frame (§7). ANGLE `texSubImage3D` ceiling itself = remaining B3 bench. |
| 13 | Atlas half-texel bleed (doc2#2) | **Adopted** | Exact mapping formula documented `u = (1.5+7x)/9` + Phase 2 seam test (§7). |
| 14 | −ε stacks up over finishing passes (doc2#3) | **Disproved / half right** | Offset is *constant* (max is idempotent): identical at 1/10/100 passes. Magnitude real though: LUT64 → 0.127 mm ⇒ default LUT 128 (0.063 mm), 256 for metrology — eval cost unchanged (§4). |
| 15 | Swiss-cheese coarse grid slows rays (doc2#4) | **Bounded, measured** | 400 narrow drills (r < coarse cell): avg 1.3 steps/ray, p99 9, max 28 — AIR-brick skips take over where the coarse field is pessimistic. |

Extra PoC findings folded into the design: stroke-run column decomposition (§5, the 18× lever);
alloc-at-writeback (no intra-batch pool traffic); coarse stamping parallel with atomic-max
(17 % of stamp cost); deep-cut engulf path works (146 k st/s at 6 mm depth with 3.4k engulfs).

**Round 2 (remaining Phase-0 gaps, all closed):**

| # | Question | Verdict | Evidence → plan change |
|---|---|---|---|
| 16 | B3: ANGLE `texSubImage3D` ceiling | **Measured** | Per-call dominated ~6 µs/tile; ≈1000 tiles/frame budget; demo needs ~103 (0.6 ms). Bulk 2.3–2.7 GB/s. R8 not faster → keep R16F. Batch indirection as one subimage (§7). |
| 17 | B4: shader hierarchy @1080p on ES3.0/ANGLE | **Passed 12×** | 4.7 ms/frame (213 FPS) on Iris Xe iGPU, real 1920×1080, carved scene, sanity readback lit (§7). |
| 18 | Segment re-bake correctness + latency | **Fixed by design change** | Rebake == live-stamped within 0.09 mm; 1.8 ms/brick typical. Naive dense-overlap hot cell: 421 s/brick → stroke-runs + deepest-first + adaptive stop: 0.43 s (990×). Runs are mandatory in rebake (§6). Fixed-seed unclipped eval misses minima by 4 mm → adaptive seeds + mandatory t-clip (§4). |
| 19 | Avalonia+ANGLE GL environment | **3 gotchas found** | Container objects not shared across the init/render contexts; user FBOs black-hole silently → render into the control FBO; attribless draws don't draw → VBO (§7). |

## 11. Complexity summary

| Operation | Cost | Depends on history? |
|---|---|---|
| Stamp one stamplet | measured ~750 LUT evals steady-state (stroke-merged) | No |
| Stamp batch of S | O(Σ overlaps), ~10× parallel speedup measured on 20 cores | No |
| Render frame | O(pixels × steps), measured avg ≤5, p99 ≈22 steps/ray | **No — independent of segment count** |
| Memory | O(surface area × band): 33 MB pool after 1M stamplets @1024³ + index 8 MB + PathIndex 26 MB @4.2M segs | Surface only |
| Re-bake brick (zoom/scrub) | O(k passes × voxels), k bounded by locality | Via PathIndex filter |
| Mesh export | O(boundary bricks), on demand, parallel MC | No |
| Removed volume / point query | O(1) | No |
| Total carve work | O(Σ segment footprints) — irreducible (the material must be cut), amortized over playback | — |

## 12. Measured baselines — the Phase-0 performance contract

Reference machine: 20-core CPU, Intel Iris Xe iGPU, ANGLE→D3D11, .NET 9 Release, ServerGC.
These are the **regression baselines** every later phase is measured against — if an integration
step lands >20 % below a baseline, stop and profile the seam, not the parts (each part is proven).

| # | Component (bench) | Baseline | Budget @60 FPS demo |
|---|---|---|---|
| P1 | Brush eval, LUT bilinear (`sweep`) | 60 ns ST · 165 M evals/s MT (×10.1); LUT-size independent | — (folded into P3) |
| P2 | Conservative cull (`sweep`) | 32 ns/test, 92 % rejection on dense paths | — (folded into P3) |
| P3 | **StampBatch** (`stamp`) | **204 k stamplets/s** sustained, 1M-path @1024³; 146 k/s @6 mm deep; ~750 evals/stamplet; ~103 dirty bricks/batch | 1167 st/frame ≈ **5.7 ms** |
| P4 | Coarse SDF stamp (`stamp` no-coarse diff) | ~17 % of P3 | included in P3 |
| P5 | Removed volume (`volume`) | 0.04 % vs analytic; 0 drift with WriteEpsilon | free (inside P3) |
| P6 | PathIndex build (`pathindex`) | 4.2 M segs → 26 MB, 221 ms build, p99 310 refs/cell (Hilbert) | one-time |
| P7 | Brick re-bake (`rebake`) | 1.8 ms/brick typical; 0.43 s worst-case dense-overlap cell; == live field within 0.09 mm | async only (Phase 5) |
| P8 | Staging gather (`staging`) | 1.0 GB/s parallel into NativeMemory, 0 GC | 103 tiles ≈ **0.3 ms** |
| P9 | GL tile upload (`upload`) | ~6 µs/tile scattered (call-bound, format-independent); 2.3–2.7 GB/s bulk; indirection = one batched subimage ≈ 0.5 ms | 103 tiles ≈ **0.6 ms** |
| P10 | Raymarch renderer (`raymarch`) | **4.7 ms/frame @1920×1080** (213 FPS), carved 512³ scene, p95 5.0 ms | **4.7 ms** |
| P11 | Correctness gate (`verify`) | stamped grid == brute-force exact brush list, maxErr = ε exactly | must stay PASS |
| P12 | Memory | pool 33 MB after 1M stamplets @1024³; index 8 MB; PathIndex 26 MB; atlas 48 MB | < 150 MB total |

**Frame budget (all measured):** 5.7 stamp + 0.3 gather + 0.6 upload + 4.7 draw ≈ **11.3 ms of
16.6 ms** — the 70k st/s demo fits 60 FPS with ~5 ms of margin on the reference machine.

- Demo target: depth-11 Hilbert ≈ 4.2M stamplets over 60 s ⇒ ≥ 70k stamplets/s sustained —
  **passed 2.9× at the component level**; Phase 3 proves it composed.
- **Escalation if the composed pipeline still misses 60 FPS** (unlikely): GPU stamping without
  compute shaders — instanced stamp quads rendered into 3D-texture layers with
  `glBlendEquation(GL_MIN)` (both core ES 3.0). CPU remains the authoritative headless
  implementation in the library.

## 12b. Full integration pipeline (Phases 1–3 assemble ONLY proven parts)

```
                    UI thread (owns GL; render context only)     worker pool (frame-scoped)
                    ┌───────────────────────────────────────┐
PathSource ─window─→│ Frame(dt):                            │
 (Hilbert / G-code) │  1 segs = Consume(dt×speed, cap)      │
 rapids filtered,   │  2 volume.StampBatch(segs) ───────────┼──→ bin → Parallel per brick (P3):
 pre-subdivided,    │     → DirtySet ∪= dirty bricks        │    stroke-runs, engulf, reclassify
 stroke order kept  │  3 gather(DirtySet ∩ budget) ─────────┼──→ parallel 9³ tile gather (P8)
                    │     into persistent NativeMemory      │    into per-frame staging ranges
                    │  4 texSubImage3D tiles (P9);          │
                    │    indirection: ONE batched subimage; │
                    │    coarse: dirty region subimage      │
                    │  5 draw raymarch → control FB (P10)   │
                    │  6 HUD: fps / st/s / bricks / MB      │
                    └───────────────────────────────────────┘
```

**Stage contracts (what crosses each boundary):**
1. **PathSource → StampBatch:** `PathSegment[] + (start,count)` window; cutting moves only;
   stamplet-scale, consecutive-index = stroke order (the stamper's run merging depends on it).
   Original unsubdivided segments go to PathIndex separately (re-bake path).
2. **StampBatch → upload:** `DirtySet` — newest-wins set keyed by brick id (re-dirty ⇒ still one
   upload at latest state); border-plane changes also dirty face-neighbors (§9.1). Cap:
   ~1000 tiles/frame (P9); deferred bricks stay in the set; coarse dirty region first.
3. **Gather → GL:** persistent `NativeMemory` staging buffer, per-frame contiguous ranges;
   raw pointer straight into `texSubImage3D` — no managed arrays, no per-frame pinning.
4. **GL objects:** textures/programs created once (shared across contexts); FBO/VAO container
   objects created lazily on the render context; every frame starts with state neutralization;
   draw targets Avalonia's control FB (user FBOs black-hole — §7).
5. **Reads while carving:** renderer reads only GPU mirrors (always coherent snapshots of
   whole bricks); CPU-side queries (volume, metrology, export) read the volume between
   `StampBatch` calls on the frame thread — never concurrently (COW snapshots arrive Phase 5).

**Integration order — each step composes two proven parts and has its own mini-gate:**

| Step | Composes | Mini-gate (vs §12 baselines) |
|---|---|---|
| I1 | Volume → gather → atlas/indirection/coarse upload (static, pre-carved) | uploaded field == CPU field (readback spot-check); seam test: no grid lines at tile borders (atlas `u=(1.5+7x)/9`, §9.1 border dirtying) |
| I2 | I1 + raymarch shader + orbit camera | ≥ 200 FPS @1080p static (P10 ±20 %); normals stable (B=3 check) |
| I3 | PathSource → StampBatch live, no rendering (headless frame loop) | ≥ 170 k st/s composed (P3 −20 %); DirtySet size ≈ 103/batch; GC gen0 ≈ 0/min |
| I4 | I2 + I3: full live loop at 70 k st/s pace | frame ≤ 16.6 ms sustained 60 s; upload latency ≤ 3 frames; no visual seams while carving |
| I5 | Demo polish: play/pause/seek timeline, tool ghost, HUD, removedVolume readout | full 4.2M-segment demo: 60 s @ 60 FPS; removedVolume plausible vs slab estimate |

Every mini-gate failure has a known suspect list: I1/I2 → GL state or tile math (not the volume —
P11 covers it); I3 → binning/threading seam (parts are P1–P5); I4 → budget interference
(measure stages separately per frame in the HUD — they are individually proven).

## 13. Phased build plan (each phase has a measurable gate)

| Phase | Deliverable | Gate |
|---|---|---|
| **0. Bench mode** — `dotnet run -c Release -- --bench <name>`, names: `sweep verify stamp volume pathindex epsilon march staging rebake mesh liveheadless stack startup seekcpu upload raymarch seam live seek` (GL benches open a window; the rest are console-only) | **DONE — ALL GATES PASSED:** B1 sweep 165M evals/s MT; B2 stamp 204k st/s (verify PASS); B3 upload ~6 µs/tile, ~1000 tiles/frame budget; B4 raymarch 4.7 ms @1080p (213 FPS, Iris Xe); volume/pathindex/epsilon/march/staging/rebake — see §10b | ~~Numbers decide CPU vs GPU stamping~~ **CPU stamping + ES3.0 renderer confirmed** |
| **1. Library core** | **DONE (bench-gated):** ToolProfile (mirrored-polygon LUT bake, MonotoneR), SweptBrush (stamplet eval + ConservativeRange + EvalSegment adaptive multi-start + ClipSegmentT), SparseSdfVolume (binning, stroke-run stamping, engulf, reclassify, WriteEpsilon, removedVolume, coarse SDF, sampling), PathIndex (build/query/stats), Rebake (run-based, deepest-first), HilbertPath, PathSource (cut-move filtering + subdivision), DirtySet (newest-wins), Mesh export (`MeshExport`, marching-tetrahedra iso extraction). | `--bench verify` PASS (maxErr = ε exactly) + `--bench rebake` PASS (Δ ≤ 0.09 mm) + `--bench mesh` PASS (non-empty iso mesh) |
| **2. Static renderer** | **DONE:** I1 + I2 wired and validated (control-FBO render path, seam-safe atlas sampling, border-aware neighbor dirtying only when border voxels change). | I1 seam gate PASS: `--bench seam` readback (`p95 adjacent-jump=0`, cpu-vs-gpu diff `0.0004`). I2 perf gate PASS: `--bench raymarch` 1.40 ms @1080p (716 FPS). |
| **3. Live pipeline** | **DONE:** I3 + I4 integrated in the GL bench loop (stamp→dirty-set→upload→draw at 70k st/s pacing, 60 s equivalent). | I3 PASS: `--bench liveheadless` 266 k st/s after seek-loop generalization. I4 PASS: `--bench live` (3595 frames / 4.19M stamplets) avg 16.14 ms, max 295.35 ms, sustained pacing met. |
| **4. Polish / escalation** | **IN PROGRESS:** I5 polish pass implemented in the app path: tool ghost overlay, play/pause + exact segment-step seeking, staged seek progress, live HUD (fps/frame ms/stamps/s/uploads/bricks), and an 800 × 400 × 10 mm stock that carves live. Upload spreading tuned via per-frame upload cap. | Remaining gate evidence: one uninterrupted 60 s GUI run of the full demo path at 60 FPS. In this automation harness the long `--bench live` GUI process currently exits early (code 1) after a few hundred frames despite normal init/sanity output, so the final long-run summary must be confirmed from a direct local run. |
| **5. Optional (designed, not scheduled)** | LOD zoom re-bake, COW snapshots + backward scrub, brick eviction/out-of-core | — |

### Seek latency measurement (2026-07-15)

Repeatable command: `dotnet run -c Release --no-build --project .\RW.CSG.Avalonia -- --bench seek`.
The GL bench pauses at step 0, rebuilds to 60%, performs a 10,000-step nearby seek, seeks to
the final step, then rewinds to one third. Times below are on Iris Xe / ANGLE at 1920×1080:

| Seek | Request → updated frame ready | CPU stamp/rebuild | GPU state rebuild | Updated draw |
|---|---:|---:|---:|---:|
| 0 → 2,516,581 (60%) | **3.033 s** | 2.432 s | 391.1 ms | 55.3 ms |
| 2,516,581 → 2,526,581 (+10k background) | **0.547 s** | 53.9 ms | 207.8 ms | 127.2 ms |
| 2,526,581 → 4,194,303 (final) | **2.746 s** | 2.045 s | 418.2 ms | 129.4 ms |
| 4,194,303 → 1,398,101 (backward) | **2.061 s** | 1.682 s | 131.4 ms | 103.9 ms |

The formerly 38.228 s final request now completes in roughly 2.1–2.8 s (14–18× faster). The fundamental
changes are: large forward seeks deep-clone the current sparse state and replay only Δ history;
level strokes union by brick-local Z bucket instead of rescanning voxels for path-adjacent runs;
segment→column work is spatially clipped and caches inverse segment length; seek replay skips
history-linear coarse brush stamping and rebuilds a conservative coarse accelerator in O(cells);
and GPU reconstruction does a parallel dense gather followed by one atlas upload and one
indirection upload. Recent runs gather 154,048 tiles in roughly 220–340 ms and submit the
atlas in 70–90 ms, replacing roughly 308,000 per-tile/per-texel GL calls.

Interactive responsiveness pass: brick-state indexing, coarse conversion, and dense atlas gathering
now finish inside the cancellable worker before the result reaches the render context. The UI shows
determinate replay/gather progress plus named indeterminate clone/coarse/upload stages. Interactive
uploads no longer issue benchmark-only `glFinish` waits; `--bench seek` retains those waits so its
request-to-frame-ready figures remain conservative and repeatable. Only forward deltas ≤2,048 remain
inline, and a bulk rebuild is selected from dirty work rather than total mapped-tile count, so a 10k
Page Up/Down seek or a one-step nudge after 32k mapped bricks no longer causes a synchronous full-atlas
gather on the render thread.

## 14. Defaults & knobs

| Knob | Default | Notes |
|---|---|---|
| Virtual resolution | 1024³ (configurable) | 2048³ later; brickIndex goes two-level if ever needed |
| Brick size | 8³ stored, 9³ atlas tiles (+1 border) | **Measured best:** 195.6 vs 174.8 (4³) vs 171.5 (16³) k st/s |
| Band width B | 3 voxels | **Measured:** B=4 costs −15 % throughput, +40 % evals |
| Brick storage | Half (R16F atlas), physical mm | 8-bit SNORM fallback if bandwidth-bound |
| WriteEpsilon | ≈ 1 Half ULP at band scale (0.0009 mm @ B=3, vox 0.3) | kills re-stamp volume drift (measured 0.21 % → 0) |
| Sweep eval | brick-local Z-bucketed column union with segment→column clipping; K=3 + parabolic refine fallback; bracketed multi-start Newton (segments) | bucket width 0.1 voxel, inside ε budget |
| Profile LUT | 128×128 (r,z) SDF grid, −ε = half cell (0.063 mm) | 256 for metrology (0.031); cost is size-independent (measured) |
| Upload staging | persistent `NativeMemory` buffers, parallel tile gather | measured 1.0 GB/s, zero GC |
| Upload budget | ≤ ~1000 scattered tiles/frame; indirection batched as one subimage | per-call dominated (~6 µs/tile, B3) |
| Atlas capacity | Viewer reserves 56³ tiles (504³ R16F, 256 MB) once; uniform-brick slots recycle | 154,048 active bricks measured at final viewer step; no playback growth |
| Tool axis | Vertical (3-axis machining) | 5-axis = future; architecture does not preclude it |
| Coarse LOD | 1/8 resolution whole-volume SDF; seeks rebuild a conservative brick-distance field in O(cells) | Also the far-field ray accelerator |
| Threading | Frame-scoped Parallel.For over brick bins; **ServerGC** | Deterministic; free-running workers only if justified |
| Graphics | Avalonia 11.3 + ANGLE (GL ES 3.0) default; `--graphics-backend wgl|vulkan` opt-in | ES 3.0 core features only |

## 15. Non-goals (v1)

- No exact BRep/mesh-boolean mode (Manifold-style export documented as a future option).
- No Hermite/dual-contouring sharp-edge reconstruction (documented upgrade path on same bricks).
- No 5-axis tool orientation, no lathe, no collision detection.
- No unit-test project — validation via `--bench verify` cross-checks and phase gates.
