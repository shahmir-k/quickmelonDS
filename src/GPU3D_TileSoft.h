/*
    Copyright 2016-2026 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#pragma once

#include <atomic>

#include "GPU3D.h"     // Renderer3D, GPU3D, Vertex, Polygon, s32/s64/u32/u16
#include "Platform.h"  // Thread / Semaphore (band-worker pool, DraStic-style)

namespace melonDS
{

class SoftRenderer;   // composite (GPU_Soft.cpp); ctor param, matches SoftRenderer3D

// ---------------------------------------------------------------------------------------------
// TileRenderer3D -- the TRUE DraStic-style tile software 3D renderer (behind LITEV_SOFT3D_DRASTIC).
//
// P1-P3 (skeleton, geometry, texture) are unchanged. Phase 4 was REBUILT to DraStic's actual
// architecture (algo spec / drastic-decompile core_gpu3d_raster.c), which fixes the P4c failure
// (the old 6-plane melonDS hybrid was +63% core-ms / +244% L1D):
//
//   * TWO planes only (DraStic band layout): TileColor (RGBA) + TileDepthId (depth[23:0]|id[31:24]).
//     No Attr plane, no under-slot. 2 * 16 * 256 * 4 = 32KB -> L1D-resident.  (fix #1)
//   * AA/edge = a NEIGHBOUR-RECOMPUTE POST-PASS (DraStic shade_edge_detect/shade_edge_colour):
//     after fill, compare each pixel's (depth,id) to its 4 neighbours; at a poly-id discontinuity
//     mark/recolour the edge from the neighbours. NO per-pixel coverage, NO under-slot.  (fix #2)
//   * PER-BLOCK POLYGON BUCKETING (DraStic geom_bucket_polys): each 16-line block iterates only the
//     polys overlapping it, not all RenderNumPolygons.  (fix #3)
//   * COMPACT contiguous per-frame vertex array (16-32B, direct-indexed) for the raster's geometry
//     reads, so the per-block re-read is cache-cheap.  (fix #4)
//   * TRANSLUCENCY = DraStic's 2-plane blend (span_blend_over) into the colour plane (approximate;
//     no under-layer).  Fog = per-pixel from the tile depth.  (fix #5/#6)
//
// Synchronous (threading is P5); GetLine never blocks. Owns SEPARATE framebuffers; the reference
// SoftRenderer3D is untouched, so we A/B behind the flag. See GPU3D_TileSoft.cpp.
// ---------------------------------------------------------------------------------------------
class TileRenderer3D : public Renderer3D
{
public:
    TileRenderer3D(melonDS::GPU3D& gpu3D, SoftRenderer& parent) noexcept;
    ~TileRenderer3D() override;

    bool Init() override;
    void Reset() override;

    void RenderFrame() override;
    void FinishRendering() override;
    void RestartFrame() override;

    u32* GetLine(int line) override;

    bool NeedsShaderCompile() override { return false; }
    void ShaderCompileStep(int& current, int& count) override {}

private:
    // ---- Reused melonDS edge/interpolation math ----------------------------------------------
    // Verbatim (trimmed) copies of SoftRenderer3D's Interpolator + Slope -- the correct, tuned
    // DS-accurate edge DDA + perspective attribute interpolation. Copied (not shared) so the
    // reference renderer's header stays byte-for-byte untouched. The AA/coverage (EdgeParams) and
    // NEON batch are dropped -- DraStic's AA is a post-pass, not per-pixel coverage.

    template<int dir>
    class Interpolator
    {
    public:
        constexpr Interpolator() {}
        constexpr Interpolator(s32 x0, s32 x1, s32 w0, s32 w1, bool wbuffer) { Setup(x0, x1, w0, w1, wbuffer); }

        constexpr void Setup(s32 x0, s32 x1, s32 w0, s32 w1, bool wbuffer)
        {
            this->x0 = x0;
            this->x1 = x1;
            this->xdiff = x1 - x0;
            this->wbuffer = wbuffer;

            if (this->xdiff != 0) this->xrecip_z = (1<<22) / this->xdiff;
            else                  this->xrecip_z = 0;

            u32 mask = dir ? 0x7E : 0x7F;
            if ((w0 == w1) && !(w0 & mask) && !(w1 & mask)) this->linear = true;
            else                                            this->linear = false;

            if (dir)
            {
                this->w0n = w0 >> 1;
                this->w0d = (w0 + ((w0 & ~w1) & 1)) >> 1;
                this->w1d = w1 >> 1;
                this->shift = 9;
            }
            else
            {
                this->w0n = w0;
                this->w0d = w0;
                this->w1d = w1;
                this->shift = 8;
            }
        }

        constexpr void SetX(s32 x)
        {
            x -= x0;
            this->x = x;
            if ((xdiff != 0) && ((!linear) || wbuffer))
            {
                u32 num = (x * w0n) << shift;
                u32 den = (x * w0d) + ((xdiff-x) * w1d);
                if (den == 0) yfactor = 0;
                else          yfactor = num / den;
            }
        }

        constexpr s32 Interpolate(s32 y0, s32 y1) const
        {
            if (xdiff == 0 || y0 == y1) return y0;
            if (!linear)
            {
                if (y0 < y1) return y0 + (((y1-y0) * yfactor) >> shift);
                else         return y1 + (((y0-y1) * ((1<<shift)-yfactor)) >> shift);
            }
            else
            {
                if (y0 < y1) return y0 + (s64)(y1-y0) * x / xdiff;
                else         return y1 + (s64)(y0-y1) * (xdiff - x) / xdiff;
            }
        }

        constexpr s32 InterpolateZ(s32 z0, s32 z1) const
        {
            if (xdiff == 0 || z0 == z1) return z0;
            if (wbuffer)
            {
                if (z0 < z1) return z0 + (((s64)(z1-z0) * yfactor) >> shift);
                else         return z1 + (((s64)(z0-z1) * ((1<<shift)-yfactor)) >> shift);
            }
            else
            {
                s32 base = 0, disp = 0, factor = 0;
                if (z0 < z1) { base = z0; disp = z1 - z0; factor = x; }
                else         { base = z1; disp = z0 - z1; factor = xdiff - x; }
                if (dir)
                {
                    int shift = 0;
                    while (disp > 0x3FF) { disp >>= 1; shift++; }
                    return base + ((((s64)disp * factor * xrecip_z) >> 22) << shift);
                }
                else
                {
                    disp >>= 9;
                    return base + (((s64)disp * factor * xrecip_z) >> 13);
                }
            }
        }

    private:
        s32 x0, x1, xdiff, x;
        int shift;
        bool linear;
        bool wbuffer;
        s32 xrecip_z;
        s32 w0n, w0d, w1d;
        u32 yfactor;
    };

    template<int side>
    class Slope
    {
    public:
        constexpr Slope() {}

        constexpr s32 SetupDummy(s32 x0, bool wbuffer)
        {
            dx = 0;
            this->x0 = x0; this->xmin = x0; this->xmax = x0;
            Increment = 0; XMajor = false;
            Interp.Setup(0, 0, 0, 0, wbuffer);
            Interp.SetX(0);
            return x0;
        }

        constexpr s32 Setup(s32 x0, s32 x1, s32 y0, s32 y1, s32 w0, s32 w1, s32 y, bool wbuffer)
        {
            this->x0 = x0;
            this->y = y;

            if (x1 > x0)      { this->xmin = x0;   this->xmax = x1-1;      this->Negative = false; }
            else if (x1 < x0) { this->xmin = x1;   this->xmax = x0-1;      this->Negative = true;  }
            else              { this->xmin = x0;   this->xmax = this->xmin; this->Negative = false; }

            xlen = xmax+1 - xmin;
            ylen = y1 - y0;

            if (ylen == 0) Increment = 0;
            else if (ylen == xlen && xlen != 1) Increment = 0x40000;
            else
            {
                s32 yrecip = (1<<18) / ylen;
                Increment = (x1-x0) * yrecip;
                if (Increment < 0) Increment = -Increment;
            }

            XMajor = (Increment > 0x40000);

            if constexpr (side)
            {
                if (XMajor)              dx = Negative ? (0x20000 + 0x40000) : (Increment - 0x20000);
                else if (Increment != 0) dx = Negative ? 0x40000 : 0;
                else                     dx = 0;
            }
            else
            {
                if (XMajor)              dx = Negative ? ((Increment - 0x20000) + 0x40000) : 0x20000;
                else if (Increment != 0) dx = Negative ? 0x40000 : 0;
                else                     dx = 0;
            }

            dx += (y - y0) * Increment;
            s32 x = XVal();

            int interpoffset = (Increment >= 0x40000) && (side ^ Negative);
            Interp.Setup(y0-interpoffset, y1-interpoffset, w0, w1, wbuffer);
            Interp.SetX(y);
            return x;
        }

        constexpr s32 Step()
        {
            dx += Increment;
            y++;
            s32 x = XVal();
            Interp.SetX(y);
            return x;
        }

        constexpr s32 XVal() const
        {
            s32 ret = Negative ? (x0 - (dx >> 18)) : (x0 + (dx >> 18));
            if (ret < xmin)      ret = xmin;
            else if (ret > xmax) ret = xmax;
            return ret;
        }

        s32 Increment;
        bool Negative;
        bool XMajor;
        Interpolator<1> Interp;

    private:
        s32 x0, xmin, xmax;
        s32 xlen, ylen;
        s32 dx;
        s32 y;
    };

    // ---- integration state -------------------------------------------------------------------
    SoftRenderer& Parent;   // unused so far; retained for later phases (2D/3D overlap hooks)

    static constexpr int OutWidth  = 256;
    static constexpr int OutHeight = 192;

    // Full-frame colour output -- the ONLY plane GetLine exposes. Double-buffered so an async-2D
    // consumer of frame N never sees frame N+1's half-written plane.
#ifdef LITEV_TILE_COORD
    // DraStic decouple step 2: 3 color buffers (one rastering, one displaying, one free) so the
    // emu can run a frame ahead of the raster without the workers' target colliding with the 2D
    // consumer's. WorkerColIdx = the buffer captured at dispatch (workers read it, NOT the live
    // RenderIdx, which the emu advances while it builds ahead).
    static constexpr int NCOL = 3;
#else
    static constexpr int NCOL = 2;
#endif
#ifdef LITEV_TILE_COORD
    // DraStic decouple (teardown 07): DraStic runs a dedicated 3D coordinator thread so its
    // emu thread hands off geometry and never blocks on the raster. Step 1: double-buffer the
    // geometry the workers read, so the emu can build frame N+1 while the workers still raster
    // N. NGEOM=2 rotates the arena; GeomWrite = emu build target, GeomRead = the index the
    // workers were dispatched with. (NGEOM=1 flag-OFF -> single buffer, codegen-identical.)
    //
    // NGEOM must cover the emu's max geometry lookahead. The color ring bounds the emu to NCOL-1
    // frames produced-but-not-consumed (each still holding its geometry), PLUS the one the emu is
    // building = NCOL geometry buffers. With NGEOM=2 < NCOL=3, when the emu ran 2 frames ahead (more
    // likely at 2D pipe-depth 2) it LAPPED GeomWrite onto a buffer the workers were still reading ->
    // tile blocks rasterized from half-overwritten geometry rendered BLACK (the horizontal-bar
    // flicker; confirmed a worker-raster corruption, not a consumer read-ahead: GetLine logged zero
    // unready-row reads while the output showed bars). Size it to the ring.
    static constexpr int NGEOM = NCOL;
#else
    static constexpr int NGEOM = 1;
#endif
    int GeomWrite = 0;   // emu-thread: geometry buffer BuildFrameGeom writes
    int GeomRead  = 0;   // set at worker dispatch; workers read this geometry buffer
    u32 ColorOut[NCOL][OutWidth * OutHeight] = {};
    int WorkerColIdx = 0;
    std::atomic<int> ConsumeIdx { 0 };   // last FULLY-completed buffer (retained; frame-done gate)
    int RenderIdx = 0;                    // buffer the workers write this frame (emu-thread only)

    // Async consumer plumbing (reference-style per-row readiness):
    //   DisplayIdx = the newest kicked buffer (published per RenderFrame).
    //   RowReady[buf][y] = 1 once row y of ColorOut[buf] is composited. PER-BUFFER, because the next
    //   RenderFrame (VCount 215) runs while the 2D consumer pass (kicked at VBlank, VCount 192) may
    //   still be mid-frame -- the old single-buffered reset clobbered the consumer's view mid-pass and
    //   forced it into an ~11ms whole-frame stall (measured: 467 stalls/10s x ~11ms). The consumer
    //   LATCHES its buffer once per pass (at line 0 -> ConsumerPassIdx); the 215 kick resets only the
    //   OTHER buffer's flags, so the in-progress pass's view stays intact, and GetLine waits ONLY for
    //   the specific row it needs via the per-row-post semaphore (Sema_RowPub, below).
    std::atomic<int> DisplayIdx { 0 };
    std::atomic<u8>  RowReady[NCOL][OutHeight] = {};
    int ConsumerPassIdx = 0;   // 2D-consumer-thread only: buffer latched at the start of its pass

    u32  ScrolledLine[256] = {};
    bool FrameIdentical = false;

    // ---- tile framebuffer (DraStic 2-plane band, PER WORKER) ---------------------------------
    // TWO planes, not six. TileH=16 (DraStic block height).
    //   Color   = R | G<<8 | B<<16 | A<<24   (A = 5-bit alpha, from the shaded pixel)
    //   DepthId = depth[23:0] | id[31:24]     (id byte packs poly-id + fog + translucent)
    // 2 * 16 * 256 * 4 = 32KB -> fits ONE A55 core's 32KB L1D. There is ONE tile PER WORKER thread
    // (each pinned to its own core), so each core rasters into its own L1D-resident 32KB tile --
    // the DraStic per-band-worker private-tile model. 3 workers => 96KB total across 3 cores' L1D.
    static constexpr int TileH = 16;
    static constexpr int NumBlocks = OutHeight / TileH;   // 12
    static constexpr int NB = 3;                          // band-worker threads (cores {0,1,2})

    // Decode-once texture cache (reused melonDS design), made PER-WORKER so the 3 raster threads
    // never contend on it (mirrors the reference's per-band TexCaches[]). One 4MB arena/worker,
    // allocated lazily on first use.
    static constexpr u32 TexCacheArenaTexels = 1u << 21;   // 2M texels (4MB u16)
    static constexpr u32 TexCacheSlots = 512;
    struct TexCacheEntry { u32 Param, Pal, Offset; s32 W, H; };
    struct TexCacheState
    {
        u16* Arena = nullptr;
        u32  Used = 0;
        u32  Count = 0;
        TexCacheEntry Entries[TexCacheSlots];
    };

    struct BandScratch
    {
        u32 Color  [TileH * OutWidth];   // 32KB tile: fits ONE core's L1D
        u32 DepthId[TileH * OutWidth];
        TexCacheState Tex;               // this worker's private decode-once cache
    };
    BandScratch Band[NB];

    // id-byte layout (bits 24..31 of DepthId):
    static constexpr u32 ID_POLY_MASK = 0x3F;    // bits 24-29 : poly-id (edge-detect compares this)
    static constexpr u32 ID_FOG       = 0x40;    // bit  30    : fog enable
    static constexpr u32 ID_TRANSL    = 0x80;    // bit  31    : written by a translucent blend

    // Per-frame config (computed on the emu thread before the workers wake; read-only in
    // workers). Double-buffered by the geometry index (Cfg[GeomWrite] built by the emu,
    // Cfg[GeomRead] read by the workers) so under LITEV_TILE_COORD the emu can compute frame
    // N+1's config while the workers still read N's. NGEOM=1 flag-OFF => single Cfg[0].
    struct FrameCfg {
        u32  ClearColorVal   = 0;
        u32  ClearDepthIdVal = 0;
        bool DoPost   = false;     // any post-pass (edge/AA/fog) this frame
        bool NeedHalo = false;     // edge/AA -> needs up/down neighbour rows (fog is same-(x,y))
        u32  BorderColor[OutWidth], BorderDepthId[OutWidth];   // off-screen (clear) neighbour rows
    };
    FrameCfg Cfg[NGEOM];

    // ---- CONTIGUOUS worker ranges (in-ORDER row production) -----------------------------------
    // The async-2D consumer (GPU_Soft::RenderBand) reads GetLine TOP-TO-BOTTOM, so it stalls on
    // RowReady[line] unless the 3D publishes rows in order. Round-robin blocks + a last-worker
    // boundary pass finished row 0 LAST -> the consumer serialised fully behind the 3D -> the emu's
    // VBlank barrier (FlushAsyncRender) waited for 3D+2D. FIX: give each worker a CONTIGUOUS block
    // range and produce its rows STRICTLY top-to-bottom (within-range deferred carry), so RowReady
    // climbs in order and the consumer pipelines behind the workers. Only the NB-1 inter-worker
    // SEAM boundaries (rows R-1 / R across a range edge) can't be done in-worker; each worker
    // publishes its range's first-2 / last-2 rows here and the last worker finishes those ~4 seam
    // rows after the barrier (the consumer reaches them late enough that they're already ready).
    static constexpr int BlocksPerWorker = NumBlocks / NB;   // 12/3 = 4  (rows per worker = 64)
    static_assert(NumBlocks % NB == 0, "NumBlocks must divide evenly across workers");
    // Each worker publishes its range-FIRST 2 raster rows ([0]=range-first, [1]=first+1); worker w-1
    // uses them (+ its own local carry) to finish the (w-1|w) seam. Worker w's own range-last rows
    // stay in its local carry (no need to publish them).
    u32  SeamColor  [NB][2][OutWidth];
    u32  SeamDepthId[NB][2][OutWidth];
    // Set (release) by worker w once it has published its range-FIRST rows (right after rastering
    // its first block). Worker w-1 waits on it to finish the (w-1|w) seam EARLY -- as soon as both
    // sides are available, long before the full-frame barrier -- so the in-order consumer that has
    // pipelined worker w-1's range never stalls at that seam row.
    std::atomic<u8> SeamFirstReady[NB] = {};

    // ---- band-worker pool (DraStic gpu3d_render_worker model; reference EnsureBandPool pattern) --
    Platform::Thread*    BandThreads[NB]   = {};
    Platform::Semaphore* BandStartSema[NB] = {};
    Platform::Semaphore* Sema_FrameDone    = nullptr;   // gate: posted once the frame is complete
    // Per-row publish semaphore (the reference's Sema_ScanlineCount pattern): workers post once per
    // published row (+ a 192-count flush at frame end so a late waiter can never sleep past the
    // frame). A stalled GetLine waits on it in a re-check loop -> a REAL block (no spinning on the
    // cores the workers need), woken within one row's production instead of at frame end.
    Platform::Semaphore* Sema_RowPub       = nullptr;
    std::atomic_bool     PoolRunning       { false };
    std::atomic<int>     BandsRemaining     { 0 };      // workers still rendering this frame
    std::atomic_bool     FrameComplete      { true };   // true when no frame is in flight

    // Publish one composited row: set its ready flag (release) then post the row semaphore so a
    // blocked consumer wakes immediately.
    inline void PublishRow(std::atomic<u8>* rdy, s32 y)
    {
        rdy[y].store(1, std::memory_order_release);
        Platform::Semaphore_Post(Sema_RowPub);
    }

#ifdef LITEV_TILE_COORD
    // The 3D coordinator thread (DraStic FUN_0015f2c8): owns the raster DISPATCH + the
    // WaitFrameComplete, so the emu thread hands off a built frame and returns WITHOUT blocking
    // on the raster. The emu is bounded to <= NCOL-1 frames in flight (Produced - Consumed),
    // waiting on Sema_RingFree only when it would lap the display buffer (rare: raster < frame).
    Platform::Thread*    CoordThread    = nullptr;
    Platform::Semaphore* Sema_CoordWork = nullptr;   // emu posts per produced frame; coord waits
    Platform::Semaphore* Sema_RingFree  = nullptr;   // coord posts when a ring slot frees; emu waits
    std::atomic_bool     CoordRunning   { false };
    std::atomic<u32>     FrameProduced  { 0 };
    std::atomic<u32>     FrameConsumed  { 0 };
    struct PendFrame { int geomIdx; int colIdx; };
    PendFrame PendRing[NCOL];
    void CoordFunc();        // coordinator thread body
    void CoordDispatch(int geomIdx, int colIdx);   // reset+dispatch+wait one frame's raster
#endif
        void EnsureBandPool();
    void ShutdownBandPool();
    void BandWorkerFunc(int idx);      // thread entry: pin, then loop { wait-start; WorkerRender; }
    void WorkerRender(int idx);        // raster + in-order post-pass for this worker's contiguous range
    void ComputeClearConfig();         // clear vals + DoPost/NeedHalo + border (emu thread)
    void WaitFrameComplete();          // block until the in-flight frame's workers finish

    u32  CalculateFogDensity(u32 z) const;
    // One post-pass row: edge-detect (4-neighbour depth+poly-id) -> recolour(edge-mark)/blend(AA),
    // then per-pixel fog. Reads the PRISTINE tile planes (color/depthid) + up/down neighbour rows
    // (tile, carry, or border) and writes the composited scanline to `dst` (the ColorOut row) --
    // the tile planes are never modified, so neighbour colours stay rasterized (DraStic out-plane).
    void PostPassRow(s32 y, u32* dst, const u32* color, const u32* depthid,
                     const u32* upColor, const u32* upDepthId,
                     const u32* downColor, const u32* downDepthId);

    // ---- compact per-frame geometry (fix #4) --------------------------------------------------
    // A contiguous, direct-indexed vertex snapshot built once per frame so the per-block raster
    // re-reads are cache-cheap (vs pointer-chasing the scattered 64B melonDS Vertex + poly->FinalW/Z).
    struct TileVtx            // 32 bytes, contiguous
    {
        s32 x, y;             // FinalPosition (screen)
        s32 w, z;             // FinalW, FinalZ (per poly-vertex)
        u16 r, g, b;          // FinalColor (9-bit)
        s16 s, t;             // TexCoords
    };
    static constexpr u32 MaxPolys = 2048;               // DS hardware cap
    static constexpr u32 VtxArenaSlots = MaxPolys * 10; // <= 10 verts/poly after clipping
    TileVtx* VtxArena[NGEOM] = {};                        // heap arena(s), reused each frame

    // Compact per-poly record: the raster-relevant scalars, snapshotted once per frame.
    struct TilePoly
    {
        u32 vtxBase;          // first TileVtx index for this poly's ring
        u16 numVerts;
        u16 vtop, vbot;       // ring index of top/bottom vertex
        s16 ytop, ybot;       // screen Y span
        bool facing;          // FacingView (winding)
        bool wbuffer;
        u32 attr;             // poly->Attr
        u32 texparam, texpal;
    };
    TilePoly FramePolys[NGEOM][MaxPolys];
    u32 FrameNumPolys[NGEOM] = {};

    // Per-block bucket: indices into FramePolys that overlap block b (fix #3).
    u16 Bucket[NGEOM][NumBlocks][MaxPolys];
    u16 BucketCount[NGEOM][NumBlocks];

    void BuildFrameGeom(int gi);   // build compact verts + per-block buckets into buffer gi

    // ---- texture (P3) -- reused melonDS decode-once cache (per-worker; see BandScratch::Tex) ----
    bool TexCacheDirty = true;   // shared; workers' caches are reset (emu thread) when it flips

    struct PolyShade
    {
        const u16* texcache;
        s32  texW, texH;
        u32  texparam, texpal;
        const u16* toontbl;
        u32  polyalpha;
        u8   alpharef;
        bool texEnable, toon, highlight, decal, wireframe;
        u32  idword;            // DraStic id byte pre-shifted into bits 24-31 (poly-id|fog)
        bool translDepthWrite;  // Attr bit 11
    };

    void DecodeTexel(u32 texparam, u32 texpal, s32 s, s32 t, u16* color, u8* alpha) const;
    void TextureLookup(u32 texparam, u32 texpal, s16 s, s16 t, u16* color, u8* alpha) const;
    const u16* ResolveTexCache(TexCacheState& tc, u32 texparam, u32 texpal, s32* outW, s32* outH);
    u32 TexAddr(const PolyShade& ps, s16 s, s16 t) const;
    u32 ShadeTexel(const PolyShade& ps, u32 vr, u32 vg, u32 vb, s16 s, s16 t) const;

    // ---- translucency (DraStic 2-plane span_blend_over; no under-layer) -------------------------
    u32 AlphaBlend(u32 srccolor, u32 dstcolor, u32 alpha) const;
    void PlotTranslucentTile(BandScratch& bs, u32 idx, u32 color, s32 z, u32 idword);

    // ---- raster (all operate on the calling worker's private BandScratch tile) -----------------
    void ClearTile(BandScratch& bs);
    void RasterPolyInTile(BandScratch& bs, const TilePoly& poly, s32 ty0, s32 ty1);
    void RenderSpanInTile(BandScratch& bs, const TilePoly& poly, s32 ty0, const PolyShade& ps,
                          Slope<0>& sl, Slope<1>& sr,
                          u32 cvl, u32 nvl, u32 cvr, u32 nvr, s32 xl, s32 xr, s32 y);
    void FlushTile(BandScratch& bs, u32* out, s32 ty0);
};

}
