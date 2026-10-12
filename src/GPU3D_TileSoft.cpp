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

// Whole translation unit is compiled ONLY under the flag, so flag-OFF the object file is empty
// (no new symbols) and the shipping build links byte-for-byte as today.
#ifdef LITEV_SOFT3D_DRASTIC

#include "GPU3D_TileSoft.h"
#if defined(__ANDROID__) || defined(__linux__)
#include <sys/prctl.h>
#endif

#include <cstring>
#include <thread>   // std::this_thread::yield (brief cross-worker seam wait)

#if (defined(LITEV_TILE_SPANVEC) || defined(LITEV_TILE_POSTVEC)) && defined(__aarch64__)
#include <arm_neon.h>
#endif
#if defined(LITEV_GEOM_NEON2) && defined(__ARM_NEON)
#include <arm_neon.h>   // BuildFrameGeom vertex-pack (coalesce the per-vertex scatter stores)
#endif

#include "GPU.h"     // GPU: MakeVRAMFlat_*Coherent, VRAMDirty_*/VRAMMap_*
#include "GPU3D.h"   // GPU3D: RenderPolygonRAM, RenderNumPolygons, RenderClearAttr*, RenderXPos, ...
#include "LiteProfile.h"   // async attribution counters (compiles to nothing unless LITEV_PROFILE=1)
#include "LitevSoftProf.h" // tile-coordinator ownership-barrier attribution

#ifdef LITEV_TOPO_PIN
#include "LitevCores.h"
#endif
#if defined(__ANDROID__) && defined(LITEV_PIN_RENDER) && defined(LITEV_TOPO_PIN)
// Each band worker on its own core of the fastest non-emu cluster (RG DS: 0, 1, 2); the
// coordinator with the other light render threads.
static void litevPinTileWorker(int idx) { melonDS::LitevTopo::PinSelf(melonDS::LitevTopo::CoreRole::RenderParallel, idx); }
static void litevPinTileCoord() { melonDS::LitevTopo::PinSelf(melonDS::LitevTopo::CoreRole::RenderCritical); }
#elif defined(__ANDROID__) && defined(LITEV_PIN_RENDER)
#include <sched.h>
#include "LitevCores.h"
// Pin each tile band-worker to its OWN core, off the emu's (the fastest remaining cores first;
// on the RG DS: 0, 1, 2, with the emu on 3). A shared mask let the scheduler double two workers
// onto one core and leave another idle (measured: all 3 rarely on distinct cores), so the 3-way
// raster ran ~2-wide and inflated the render gate the emu waits on. A distinct core per worker
// guarantees the parallel spread.
static void litevPinTileWorker(int idx)
{
    const std::vector<int>& others = melonDS::LitevCores::Get().Others;
    if (others.empty()) return;
    cpu_set_t set; CPU_ZERO(&set);
    CPU_SET(others[idx % others.size()], &set);
    sched_setaffinity(0, sizeof(set), &set);
}
#else
static void litevPinTileWorker(int) {}
#endif
#if !(defined(__ANDROID__) && defined(LITEV_PIN_RENDER) && defined(LITEV_TOPO_PIN))
static void litevPinTileCoord() {}
#endif

#if defined(__ANDROID__)
#include <sys/system_properties.h>
#include <cstdlib>
static int litevTileProp(const char* name, int def) {
    char b[8] = {0};
    if (__system_property_get(name, b) > 0) return (atoi(b) != 0) ? 1 : 0;
    return def;
}
// Unclamped-ish int prop (range-limited) so prefetch DISTANCES can be swept on-device
// without a rebuild (debug.litev.geopf_poly / geopf_vtx / geopf_vl).
static int litevTilePropInt(const char* name, int def, int lo, int hi) {
    char b[16] = {0};
    if (__system_property_get(name, b) > 0) {
        int v = atoi(b);
        if (v < lo) v = lo;
        if (v > hi) v = hi;
        return v;
    }
    return def;
}
#else
static int litevTileProp(const char*, int def) { return def; }
static int litevTilePropInt(const char*, int def, int, int) { return def; }
#endif

#ifdef __APPLE__
extern "C" int thread_selfcounts(int type, void* buf, size_t nbytes);   // libsystem_kernel: this thread's PMU counts
#endif
#include <ctime>
#include <cstdio>
#include <cstdlib>
// diagnostic (env LITEV_S3D_CPU=1): thread CPU time (+ instructions on Apple) of the tile workers and
// the coordinator, per produced 3D frame; printed every 300 frames (same pattern as LITEV_S2D_CPU)
static const bool s3dCpu = getenv("LITEV_S3D_CPU") != nullptr;
static std::atomic<unsigned long long> s3dWorkNs{0}, s3dWorkIns{0}, s3dCoordNs{0}, s3dCoordIns{0}, s3dFrames{0}, s3dKept{0}, s3dTexDecodes{0};
struct S3DCpuScope
{
    std::atomic<unsigned long long>& ns; std::atomic<unsigned long long>& ins;
    timespec c0{}; unsigned long long pc0[2] = {};
    S3DCpuScope(std::atomic<unsigned long long>& n, std::atomic<unsigned long long>& i) : ns(n), ins(i)
    {
        if (!s3dCpu) return;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c0);
#ifdef __APPLE__
        thread_selfcounts(1, pc0, sizeof(pc0));
#endif
    }
    ~S3DCpuScope()
    {
        if (!s3dCpu) return;
        timespec c1; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
        unsigned long long pc1[2] = {};
#ifdef __APPLE__
        thread_selfcounts(1, pc1, sizeof(pc1));
#endif
        ns.fetch_add((unsigned long long)(c1.tv_sec - c0.tv_sec) * 1000000000ull + c1.tv_nsec - c0.tv_nsec, std::memory_order_relaxed);
        ins.fetch_add(pc1[0] - pc0[0], std::memory_order_relaxed);
    }
};
static void s3dCpuFrame()
{
    if (!s3dCpu) return;
    const unsigned long long f = s3dFrames.fetch_add(1, std::memory_order_relaxed) + 1;
    if (f % 300 == 0)
        fprintf(stderr, "S3D_CPU frames=%llu worker_ms_per_frame=%.4f coord_ms_per_frame=%.4f worker_instr_per_frame=%llu coord_instr_per_frame=%llu kept=%llu texdecodes=%llu\n",
                f, s3dWorkNs.load() / 1e6 / f, s3dCoordNs.load() / 1e6 / f, s3dWorkIns.load() / f, s3dCoordIns.load() / f, s3dKept.load(), s3dTexDecodes.load());
}

#if defined(LITEV_TILE_TEXSHARED) && defined(LITEV_TILE_COORD)
#define TILE_TEXSH 1
#ifdef LITEV_TILE_SPANFAST
#define TILE_SPANFAST 1
#endif
#include <algorithm>
// flat texture / palette VRAM ranges (addresses wrap like ReadVRAMFlat_*)
static void texFlatCopy(const melonDS::u8* base, melonDS::u32 mask, melonDS::u32 addr, melonDS::u32 len, melonDS::u8* dst)
{
    while (len) { const melonDS::u32 a = addr & mask, n = std::min(len, mask + 1 - a); memcpy(dst, base + a, n); dst += n; addr += n; len -= n; }
}
static bool texFlatEqual(const melonDS::u8* base, melonDS::u32 mask, melonDS::u32 addr, melonDS::u32 len, const melonDS::u8* src)
{
    while (len) { const melonDS::u32 a = addr & mask, n = std::min(len, mask + 1 - a); if (memcmp(src, base + a, n)) return false; src += n; addr += n; len -= n; }
    return true;
}
static const bool texShVerify = getenv("LITEV_TEXSHARED_VERIFY") != nullptr;   // re-decode every hit, abort on a mismatch
#endif

namespace melonDS
{

TileRenderer3D::TileRenderer3D(melonDS::GPU3D& gpu3D, SoftRenderer& parent) noexcept
    : Renderer3D(gpu3D), Parent(parent)
{
#ifdef LITEV_TOPO_PIN
    NB = LitevTopo::TileWorkers(NumBlocks, NBMax);   // fixed for this renderer's lifetime
    BlocksPerWorker = NumBlocks / NB;
#endif
    for (int b = 0; b < NCOL; b++)
        for (int y = 0; y < OutHeight; y++) RowReady[b][y].store(1, std::memory_order_relaxed);
}

TileRenderer3D::~TileRenderer3D()
{
    ShutdownBandPool();                 // join workers before freeing anything they touch
    for (int b = 0; b < NBMax; b++)
        delete[] Band[b].Tex.Arena;
    for (int g = 0; g < NGEOM; g++) delete[] VtxArena[g];
#if defined(LITEV_TILE_SAMEFRAME) && defined(LITEV_TILE_COORD)
    delete LastCfg; delete[] LastPolys; delete[] LastVtx;
#endif
}

bool TileRenderer3D::Init()
{
    for (int g = 0; g < NGEOM; g++) if (!VtxArena[g]) VtxArena[g] = new TileVtx[VtxArenaSlots]();
    return true;
}

void TileRenderer3D::Reset()
{
    // NOTE: the SoftRenderer composite calls Rend3D->Reset() but never Rend3D->Init()
    // (the reference SoftRenderer3D allocates in Reset()/lazily too), so allocate the
    // per-frame compact vertex arena here or BuildFrameGeom() null-derefs on frame 1.
#ifdef LITEV_TILE_COORD
    // Drain the pipeline: no queued frame may reference geometry/buffers we are about to wipe.
    if (PoolRunning.load(std::memory_order_relaxed))
        while (FrameProduced.load(std::memory_order_acquire) != FrameConsumed.load(std::memory_order_acquire))
            Platform::Semaphore_Wait(Sema_RingFree);
#endif
    WaitFrameComplete();                 // no worker may be mid-frame while we wipe state
    for (int g = 0; g < NGEOM; g++) if (!VtxArena[g]) VtxArena[g] = new TileVtx[VtxArenaSlots]();
    memset(ColorOut, 0, sizeof(ColorOut));
    for (int b = 0; b < NB; b++)
    {
        memset(Band[b].Color,   0, sizeof(Band[b].Color));
        memset(Band[b].DepthId, 0, sizeof(Band[b].DepthId));
        Band[b].Tex.Used = 0;
        Band[b].Tex.Count = 0;
    }
    DisplayIdx.store(0, std::memory_order_relaxed);
    for (int b = 0; b < NCOL; b++)
        for (int y = 0; y < OutHeight; y++) RowReady[b][y].store(1, std::memory_order_relaxed);   // blank = complete
    ConsumerPassIdx = 0;
    RenderIdx = 0;
    FrameIdentical = false;
    TexCacheDirty = true;
    GeomWrite = 0; GeomRead = 0;
    for (int g = 0; g < NGEOM; g++) FrameNumPolys[g] = 0;
#if defined(LITEV_TILE_SAMEFRAME) && defined(LITEV_TILE_COORD)
    LastValid = false;   // ColorOut was wiped
#endif
#ifdef TILE_TEXSH
    TexSh.clear(); TexShBytes = 0; TexGen++;
#endif
}

// ---------------------------------------------------------------------------------------------
// Texture (ported verbatim from SoftRenderer3D -- HW-accurate, so the reference is untouched)
// ---------------------------------------------------------------------------------------------

void TileRenderer3D::DecodeTexel(u32 texparam, u32 texpal, s32 s, s32 t, u16* color, u8* alpha) const
{
    u32 vramaddr = (texparam & 0xFFFF) << 3;
    s32 width = 8 << ((texparam >> 20) & 0x7);

    u8 alpha0;
    if (texparam & (1<<29)) alpha0 = 0;
    else                    alpha0 = 31;

    switch ((texparam >> 26) & 0x7)
    {
    case 1: // A3I5
        {
            vramaddr += ((t * width) + s);
            u8 pixel = GPU.ReadVRAMFlat_Texture<u8>(vramaddr);
            texpal <<= 4;
            *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + ((pixel&0x1F)<<1));
            *alpha = ((pixel >> 3) & 0x1C) + (pixel >> 6);
        }
        break;

    case 2: // 4-color
        {
            vramaddr += (((t * width) + s) >> 2);
            u8 pixel = GPU.ReadVRAMFlat_Texture<u8>(vramaddr);
            pixel >>= ((s & 0x3) << 1);
            pixel &= 0x3;
            texpal <<= 3;
            *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + (pixel<<1));
            *alpha = (pixel==0) ? alpha0 : 31;
        }
        break;

    case 3: // 16-color
        {
            vramaddr += (((t * width) + s) >> 1);
            u8 pixel = GPU.ReadVRAMFlat_Texture<u8>(vramaddr);
            if (s & 0x1) pixel >>= 4;
            else         pixel &= 0xF;
            texpal <<= 4;
            *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + (pixel<<1));
            *alpha = (pixel==0) ? alpha0 : 31;
        }
        break;

    case 4: // 256-color
        {
            vramaddr += ((t * width) + s);
            u8 pixel = GPU.ReadVRAMFlat_Texture<u8>(vramaddr);
            texpal <<= 4;
            *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + (pixel<<1));
            *alpha = (pixel==0) ? alpha0 : 31;
        }
        break;

    case 5: // compressed (4x4)
        {
            vramaddr += ((t & 0x3FC) * (width>>2)) + (s & 0x3FC);
            vramaddr += (t & 0x3);
            vramaddr &= 0x7FFFF;

            u32 slot1addr = 0x20000 + ((vramaddr & 0x1FFFC) >> 1);
            if (vramaddr >= 0x40000)
                slot1addr += 0x10000;

            u8 val;
            if (vramaddr >= 0x20000 && vramaddr < 0x40000)
                val = 0;
            else
            {
                val = GPU.ReadVRAMFlat_Texture<u8>(vramaddr);
                val >>= (2 * (s & 0x3));
            }

            u16 palinfo = GPU.ReadVRAMFlat_Texture<u16>(slot1addr);
            u32 paloffset = (palinfo & 0x3FFF) << 2;
            texpal <<= 4;

            switch (val & 0x3)
            {
            case 0:
                *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset);
                *alpha = 31;
                break;
            case 1:
                *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset + 2);
                *alpha = 31;
                break;
            case 2:
                if ((palinfo >> 14) == 1)
                {
                    u16 color0 = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset);
                    u16 color1 = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset + 2);
                    u32 r0 = color0 & 0x001F; u32 g0 = color0 & 0x03E0; u32 b0 = color0 & 0x7C00;
                    u32 r1 = color1 & 0x001F; u32 g1 = color1 & 0x03E0; u32 b1 = color1 & 0x7C00;
                    u32 r = (r0 + r1) >> 1;
                    u32 g = ((g0 + g1) >> 1) & 0x03E0;
                    u32 b = ((b0 + b1) >> 1) & 0x7C00;
                    *color = r | g | b;
                }
                else if ((palinfo >> 14) == 3)
                {
                    u16 color0 = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset);
                    u16 color1 = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset + 2);
                    u32 r0 = color0 & 0x001F; u32 g0 = color0 & 0x03E0; u32 b0 = color0 & 0x7C00;
                    u32 r1 = color1 & 0x001F; u32 g1 = color1 & 0x03E0; u32 b1 = color1 & 0x7C00;
                    u32 r = (r0*5 + r1*3) >> 3;
                    u32 g = ((g0*5 + g1*3) >> 3) & 0x03E0;
                    u32 b = ((b0*5 + b1*3) >> 3) & 0x7C00;
                    *color = r | g | b;
                }
                else
                    *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset + 4);
                *alpha = 31;
                break;
            case 3:
                if ((palinfo >> 14) == 2)
                {
                    *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset + 6);
                    *alpha = 31;
                }
                else if ((palinfo >> 14) == 3)
                {
                    u16 color0 = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset);
                    u16 color1 = GPU.ReadVRAMFlat_TexPal<u16>(texpal + paloffset + 2);
                    u32 r0 = color0 & 0x001F; u32 g0 = color0 & 0x03E0; u32 b0 = color0 & 0x7C00;
                    u32 r1 = color1 & 0x001F; u32 g1 = color1 & 0x03E0; u32 b1 = color1 & 0x7C00;
                    u32 r = (r0*3 + r1*5) >> 3;
                    u32 g = ((g0*3 + g1*5) >> 3) & 0x03E0;
                    u32 b = ((b0*3 + b1*5) >> 3) & 0x7C00;
                    *color = r | g | b;
                    *alpha = 31;
                }
                else
                {
                    *color = 0;
                    *alpha = 0;
                }
                break;
            }
        }
        break;

    case 6: // A5I3
        {
            vramaddr += ((t * width) + s);
            u8 pixel = GPU.ReadVRAMFlat_Texture<u8>(vramaddr);
            texpal <<= 4;
            *color = GPU.ReadVRAMFlat_TexPal<u16>(texpal + ((pixel&0x7)<<1));
            *alpha = (pixel >> 3);
        }
        break;

    case 7: // direct colour
        {
            vramaddr += (((t * width) + s) << 1);
            *color = GPU.ReadVRAMFlat_Texture<u16>(vramaddr);
            *alpha = (*color & 0x8000) ? 31 : 0;
        }
        break;
    }
}

void TileRenderer3D::TextureLookup(u32 texparam, u32 texpal, s16 s, s16 t, u16* color, u8* alpha) const
{
    s32 width  = 8 << ((texparam >> 20) & 0x7);
    s32 height = 8 << ((texparam >> 23) & 0x7);

    s32 si = s >> 4;
    s32 ti = t >> 4;

    if (texparam & (1<<16))
    {
        if (texparam & (1<<18)) { if (si & width)  si = (width-1)  - (si & (width-1));  else si = (si & (width-1)); }
        else                    si &= width-1;
    }
    else { if (si < 0) si = 0; else if (si >= width) si = width-1; }

    if (texparam & (1<<17))
    {
        if (texparam & (1<<19)) { if (ti & height) ti = (height-1) - (ti & (height-1)); else ti = (ti & (height-1)); }
        else                    ti &= height-1;
    }
    else { if (ti < 0) ti = 0; else if (ti >= height) ti = height-1; }

    DecodeTexel(texparam, texpal, si, ti, color, alpha);
}

const u16* TileRenderer3D::ResolveTexCache(TexCacheState& tcref, u32 texparam, u32 texpal, s32* outW, s32* outH)
{
    s32 W = 8 << ((texparam >> 20) & 0x7);
    s32 H = 8 << ((texparam >> 23) & 0x7);
    *outW = W;
    *outH = H;

    TexCacheState* tc = &tcref;

    for (u32 i = 0; i < tc->Count; i++)
    {
        TexCacheEntry& e = tc->Entries[i];
        if (e.Param == texparam && e.Pal == texpal)
            return tc->Arena + e.Offset;
    }

    u32 need = (u32)W * (u32)H;
    if (need > TexCacheArenaTexels)
        return nullptr;

    if (!tc->Arena)
        tc->Arena = new u16[TexCacheArenaTexels];

    if (tc->Used + need > TexCacheArenaTexels || tc->Count >= TexCacheSlots)
    {
        tc->Used = 0;
        tc->Count = 0;
    }

    u32 off = tc->Used;
    u16* dst = tc->Arena + off;
    for (s32 tt = 0; tt < H; tt++)
    {
        u16* row = dst + (u32)tt * (u32)W;
        for (s32 ss = 0; ss < W; ss++)
        {
            u16 c; u8 a;
            DecodeTexel(texparam, texpal, ss, tt, &c, &a);
            row[ss] = (u16)((c & 0x7FFF) | (a >= 16 ? 0x8000 : 0));
        }
    }

    tc->Used += need;
    TexCacheEntry& e = tc->Entries[tc->Count++];
    e.Param = texparam; e.Pal = texpal; e.Offset = off; e.W = W; e.H = H;
    return dst;
}

u32 TileRenderer3D::TexAddr(const PolyShade& ps, s16 s, s16 t) const
{
    s32 si = s >> 4;
    s32 ti = t >> 4;

    if (ps.texparam & (1<<16))
    {
        if (ps.texparam & (1<<18)) { if (si & ps.texW) si = (ps.texW-1) - (si & (ps.texW-1)); else si = (si & (ps.texW-1)); }
        else                       si &= ps.texW-1;
    }
    else { if (si < 0) si = 0; else if (si >= ps.texW) si = ps.texW-1; }

    if (ps.texparam & (1<<17))
    {
        if (ps.texparam & (1<<19)) { if (ti & ps.texH) ti = (ps.texH-1) - (ti & (ps.texH-1)); else ti = (ti & (ps.texH-1)); }
        else                       ti &= ps.texH-1;
    }
    else { if (ti < 0) ti = 0; else if (ti >= ps.texH) ti = ps.texH-1; }

    return (u32)ti * (u32)ps.texW + (u32)si;
}

u32 TileRenderer3D::ShadeTexel(const PolyShade& ps, u32 vr, u32 vg, u32 vb, s16 s, s16 t) const
{
    u8 r, g, b, a;

    if (ps.toon)
    {
        if (ps.highlight) { vg = vr; vb = vr; }
        else
        {
            u16 tc = ps.toontbl[vr >> 1];
            vr = (tc << 1) & 0x3E; if (vr) vr++;
            vg = (tc >> 4) & 0x3E; if (vg) vg++;
            vb = (tc >> 9) & 0x3E; if (vb) vb++;
        }
    }

    if (ps.texEnable)
    {
#ifdef TILE_TEXSH
        const u32 tp = ps.tex32[TexAddr(ps, s, t)];
        const u8 tr = tp & 0x3F, tg = (tp >> 8) & 0x3F, tb = (tp >> 16) & 0x3F, talpha = tp >> 24;
#else
        u16 tcolor; u8 talpha;
        if (ps.texcache)
        {
            u16 packed = ps.texcache[TexAddr(ps, s, t)];
            tcolor = (u16)(packed & 0x7FFF);
            talpha = (packed & 0x8000) ? 31 : 0;
        }
        else
        {
            TextureLookup(ps.texparam, ps.texpal, s, t, &tcolor, &talpha);
        }

        u8 tr = (tcolor << 1) & 0x3E; if (tr) tr++;
        u8 tg = (tcolor >> 4) & 0x3E; if (tg) tg++;
        u8 tb = (tcolor >> 9) & 0x3E; if (tb) tb++;
#endif

        if (ps.decal)
        {
            if (talpha == 0)       { r = vr; g = vg; b = vb; }
            else if (talpha == 31) { r = tr; g = tg; b = tb; }
            else
            {
                r = ((tr * talpha) + (vr * (31-talpha))) >> 5;
                g = ((tg * talpha) + (vg * (31-talpha))) >> 5;
                b = ((tb * talpha) + (vb * (31-talpha))) >> 5;
            }
            a = ps.polyalpha;
        }
        else
        {
            r = ((tr+1) * (vr+1) - 1) >> 6;
            g = ((tg+1) * (vg+1) - 1) >> 6;
            b = ((tb+1) * (vb+1) - 1) >> 6;
            a = ((talpha+1) * (ps.polyalpha+1) - 1) >> 5;
        }
    }
    else
    {
        r = vr; g = vg; b = vb; a = ps.polyalpha;
    }

    if (ps.highlight)
    {
        u16 tc = ps.toontbl[vr >> 1];
        u8 hr = (tc << 1) & 0x3E; if (hr) hr++;
        u8 hg = (tc >> 4) & 0x3E; if (hg) hg++;
        u8 hb = (tc >> 9) & 0x3E; if (hb) hb++;
        int rr = r + hr; if (rr > 63) rr = 63; r = (u8)rr;
        int gg = g + hg; if (gg > 63) gg = 63; g = (u8)gg;
        int bb = b + hb; if (bb > 63) bb = 63; b = (u8)bb;
    }

    if (ps.wireframe) a = 31;

    return r | (g << 8) | (b << 16) | ((u32)a << 24);
}

// DS 5-bit alpha blend (ported verbatim). Blend-disabled -> src replaces, keeping max alpha.
u32 TileRenderer3D::AlphaBlend(u32 srccolor, u32 dstcolor, u32 alpha) const
{
    u32 dstalpha = dstcolor >> 24;
    if (dstalpha == 0)
        return srccolor;

    u32 srcR = srccolor & 0x3F;
    u32 srcG = (srccolor >> 8) & 0x3F;
    u32 srcB = (srccolor >> 16) & 0x3F;

    if (Cfg[GeomRead].DispCnt & (1<<3))
    {
        u32 dstR = dstcolor & 0x3F;
        u32 dstG = (dstcolor >> 8) & 0x3F;
        u32 dstB = (dstcolor >> 16) & 0x3F;
        alpha++;
        srcR = ((srcR * alpha) + (dstR * (32-alpha))) >> 5;
        srcG = ((srcG * alpha) + (dstG * (32-alpha))) >> 5;
        srcB = ((srcB * alpha) + (dstB * (32-alpha))) >> 5;
        alpha--;
    }

    if (alpha > dstalpha)
        dstalpha = alpha;

    return srcR | (srcG << 8) | (srcB << 16) | (dstalpha << 24);
}

// ---------------------------------------------------------------------------------------------
// Compact per-frame geometry snapshot + per-block polygon bucketing (fix #3 / #4)
//   Mirrors DraStic's geom_bucket_polys (Stage 2.2) + its 16-byte ds_vertex. Once per frame we
//   snapshot each drawable polygon's vertex ring into a contiguous, direct-indexed TileVtx array
//   and bucket the poly index into every 16-line block it overlaps, so RasterScene iterates ONLY
//   the polys overlapping a block -- not all RenderNumPolygons re-checked per block.
// ---------------------------------------------------------------------------------------------
void TileRenderer3D::BuildFrameGeom(int gi)
{
    const int gw = gi;
    u16 (&Bucket)[NumBlocks][MaxPolys] = this->Bucket[gw];
    u16 (&BucketCount)[NumBlocks] = this->BucketCount[gw];
    TilePoly (&FramePolys)[MaxPolys] = this->FramePolys[gw];
    TileVtx* VtxArena = this->VtxArena[gw];
    u32 FrameNumPolys = 0;
    for (int b = 0; b < NumBlocks; b++) BucketCount[b] = 0;
    u32 vtxUsed = 0;
    Polygon** const polySrc = GPU3D.RenderPolygonRAM.data();
    u32 npolys = GPU3D.RenderNumPolygons;
    if (npolys > MaxPolys) npolys = MaxPolys;

#ifdef LITEV_GEOM_PREFETCH2
    // Default ON. Runtime prop so it can be A/B'd off without a rebuild
    // (debug.litev.geoprefetch=0); read once per frame (negligible).
    const bool geoPf = litevTileProp("debug.litev.geoprefetch", 1);
#endif
#ifdef LITEV_GEOM_PREFETCH2
    // Staged/deep prefetch distances, tunable on-device without rebuild. The shallow (1-poly)
    // geoprefetch measured only ~-1pp because on the in-order A55 the gather is ~150-190 cyc/vert
    // (fully-exposed DRAM miss) while useful work is only ~10-15 cyc/vert: to overlap ~180 cyc you
    // need ~4 polys of lookahead, and PF_POLY > PF_VTX so the Polygon struct is already warm when
    // we chase its Vertices[] to prefetch the actual (cold) vertex lines.
    const int pfPoly = litevTilePropInt("debug.litev.geopf_poly", 8, 0, 64);  // Polygon-struct lookahead
    const int pfVtx  = litevTilePropInt("debug.litev.geopf_vtx",  4, 0, 64);  // Vertex-line   lookahead
    const int pfVL   = litevTilePropInt("debug.litev.geopf_vl",   2, 1, 10);  // vertex lines prefetched/poly
#endif

    for (u32 i = 0; i < npolys; i++)
    {
        Polygon* poly = polySrc[i];

#if defined(LITEV_GEOM_PREFETCH2)
        // Staged, deep, multi-line software-pipelined prefetch of the pointer-chased Vertex
        // structs — the cold ~64B line fetch (poly->Vertices[v] -> sv) that dominates this
        // loop's latency. PURE prefetch + const reads the loop already does -> byte-identical
        // output -> MP-safe (Category-A guest-identical, Category-B settled-PPM identical).
        if (geoPf)
        {
            // Stage A — warm the Polygon struct pfPoly polys ahead, ALL 4 lines the loop reads
            // (Vertices@0-79, FinalZ/FinalW@84-163, Attr/flags/YTop/VTop@168-219). The ptr array
            // polySrc[] is hot/sequential, so indexing it far ahead never stalls.
            const u32 ja = i + (u32)pfPoly;
            if (ja < npolys)
            {
                const char* pj = (const char*)polySrc[ja];
                __builtin_prefetch(pj + 0,   0, 1);
                __builtin_prefetch(pj + 64,  0, 1);
                __builtin_prefetch(pj + 128, 0, 1);
                __builtin_prefetch(pj + 192, 0, 1);
            }
            // Stage B — warm the cold Vertex lines pfVtx polys ahead. np is already warm
            // (Stage A prefetched it pfPoly-pfVtx iters ago) so these ptr-loads hit. Prefetch
            // FEW lines from DEEP (not all nnv from shallow): [0] (strip-reused/first vert) and
            // [2] (start of the fresh contiguous run) — the A55 next-line HW prefetcher covers
            // [3..], and issuing fewer prefetches avoids flooding the ~4-6 L1 fill buffers.
            const u32 jb = i + (u32)pfVtx;
            if (jb < npolys)
            {
                const Polygon* np = polySrc[jb];
                u32 nnv = np->NumVertices; if (nnv > 10) nnv = 10;
                if (nnv > 0) __builtin_prefetch(np->Vertices[0], 0, 1);
                for (u32 k = 2; k < nnv && k < (u32)(pfVL + 1); k++)
                    __builtin_prefetch(np->Vertices[k], 0, 1);
            }
        }
#endif

        // Poly-level scope-outs (later phases): degenerate, flat single-scanline, lines, shadows,
        // wireframe (alpha 0). Doing them here keeps the buckets drawable-only.
        if (poly->Degenerate) continue;
        if (poly->YTop == poly->YBottom) continue;
        if (poly->Type == 1) continue;
        if (poly->IsShadow || poly->IsShadowMask) continue;
        if (((poly->Attr >> 16) & 0x1F) == 0) continue;

        const u32 nverts = poly->NumVertices;
        if (vtxUsed + nverts > VtxArenaSlots) break;   // arena full (defensive; ~never hit)

        const u32 pidx = FrameNumPolys;
        TilePoly& tp = FramePolys[pidx];
        tp.vtxBase  = vtxUsed;
        tp.numVerts = (u16)nverts;
        tp.vtop     = (u16)poly->VTop;
        tp.vbot     = (u16)poly->VBottom;
        tp.ytop     = (s16)poly->YTop;
        tp.ybot     = (s16)poly->YBottom;
        tp.facing   = poly->FacingView;
        tp.wbuffer  = poly->WBuffer;
        tp.attr     = poly->Attr;
        tp.texparam = poly->TexParam;
        tp.texpal   = poly->TexPalette;

        // Snapshot the vertex ring contiguously (position + W/Z + colour + texcoords).
#if defined(LITEV_GEOM_NEON2) && defined(__ARM_NEON)
        // LITEV_GEOM_NEON2: coalesce the four independent 4-byte scatter stores of
        // {x,y,w,z} into ONE 16-byte vector store. x,y come from the pointer-chased
        // Vertex (FinalPosition, contiguous s32x2); w,z from the poly's FinalW/FinalZ
        // arrays. Byte-identical to the scalar path — the same four s32 values land
        // at the same TileVtx offsets; only the store-queue traffic drops (4 str -> 1
        // str q). NOTE: the read side (cold Vertex fetch) still dominates this loop's
        // latency; validate this lever by PMU store/backend-stall reduction, not fps.
        for (u32 v = 0; v < nverts; v++)
        {
            const Vertex* sv = poly->Vertices[v];
            TileVtx& dv = VtxArena[vtxUsed + v];
            int32x2_t xy = vld1_s32(sv->FinalPosition);        // {FinalPosition[0], [1]}
            int32x2_t wz = { poly->FinalW[v], poly->FinalZ[v] };
            vst1q_s32(&dv.x, vcombine_s32(xy, wz));            // dv.x, dv.y, dv.w, dv.z
            dv.r = (u16)sv->FinalColor[0];
            dv.g = (u16)sv->FinalColor[1];
            dv.b = (u16)sv->FinalColor[2];
            dv.s = (s16)sv->TexCoords[0];
            dv.t = (s16)sv->TexCoords[1];
        }
#else
        for (u32 v = 0; v < nverts; v++)
        {
            const Vertex* sv = poly->Vertices[v];
            TileVtx& dv = VtxArena[vtxUsed + v];
            dv.x = sv->FinalPosition[0];
            dv.y = sv->FinalPosition[1];
            dv.w = poly->FinalW[v];
            dv.z = poly->FinalZ[v];
            dv.r = (u16)sv->FinalColor[0];
            dv.g = (u16)sv->FinalColor[1];
            dv.b = (u16)sv->FinalColor[2];
            dv.s = (s16)sv->TexCoords[0];
            dv.t = (s16)sv->TexCoords[1];
        }
#endif
        vtxUsed += nverts;

        // Bucket into every 16-line block the poly overlaps (YBottom is exclusive).
        int b0 = poly->YTop / TileH;
        int b1 = (poly->YBottom - 1) / TileH;
        if (b0 < 0) b0 = 0;
        if (b1 >= NumBlocks) b1 = NumBlocks - 1;
        for (int b = b0; b <= b1; b++)
            Bucket[b][BucketCount[b]++] = (u16)pidx;

        FrameNumPolys++;
    }
    this->FrameNumPolys[gw] = FrameNumPolys;
}

// ---------------------------------------------------------------------------------------------
// Raster
// ---------------------------------------------------------------------------------------------

void TileRenderer3D::ClearTile(BandScratch& bs)
{
    const u32 c  = Cfg[GeomRead].ClearColorVal;
    const u32 di = Cfg[GeomRead].ClearDepthIdVal;
    for (int i = 0; i < TileH * OutWidth; i++)
    {
        bs.Color[i]   = c;
        bs.DepthId[i] = di;
    }
    memset(bs.TranslId, 0xFF, sizeof(bs.TranslId));
}

void TileRenderer3D::FlushTile(BandScratch& bs, u32* out, s32 ty0)
{
    for (int yt = 0; yt < TileH; yt++)
    {
        const s32 y = ty0 + yt;
        if (y >= OutHeight) break;
        memcpy(&out[y * OutWidth], &bs.Color[yt * OutWidth], OutWidth * sizeof(u32));
    }
}

// DraStic 2-plane translucent write (span_blend_over): blend src over the tile colour; conditional
// depth/id write (z == -1 => no depth write). NO under-layer. Approximate -- melonDS's exact
// same-translucent-poly no-reblend rule is dropped (the user authorized approximation).
void TileRenderer3D::PlotTranslucentTile(BandScratch& bs, u32 idx, u32 color, s32 z, u32 idword)
{
    // a translucent pixel isn't drawn over one from a translucent polygon with the same ID (the
    // pieces of one translucent object blend once: Pokemon White's gift-box light beams got
    // doubled-up white seams without this)
    const u8 pid = (idword >> 24) & ID_POLY_MASK;
    if (bs.TranslId[idx] == pid) return;
    bs.TranslId[idx] = pid;
    bs.Color[idx] = AlphaBlend(color, bs.Color[idx], color >> 24);
    if (z != -1)
        bs.DepthId[idx] = ((u32)z & 0x00FFFFFF) | idword;   // idword carries ID_TRANSL
}

u32 TileRenderer3D::CalculateFogDensity(u32 z) const
{
    const FrameCfg& cfg = Cfg[GeomRead];
    u32 densityid, densityfrac;
    if (z < cfg.FogOffset)
    {
        densityid = 0;
        densityfrac = 0;
    }
    else
    {
        z -= cfg.FogOffset;
        z = (z >> 2) << cfg.FogShift;
        densityid = z >> 17;
        if (densityid >= 32) { densityid = 32; densityfrac = 0; }
        else                   densityfrac = z & 0x1FFFF;
    }

    u32 density =
        ((cfg.FogDensityTable[densityid] * (0x20000-densityfrac)) +
         (cfg.FogDensityTable[densityid+1] * densityfrac)) >> 17;
    if (density >= 127) density = 128;
    return density;
}

// ---------------------------------------------------------------------------------------------
// Post-pass: DraStic neighbour-recompute edge/AA (shade_edge_detect + shade_edge_colour) + fog.
//   Reads the 2 PRISTINE tile planes (colour + depth/id) and writes the composited scanline to the
//   full-frame output. NO stored coverage, NO under-slot. For each pixel: compare its (depth,
//   poly-id) to its 4 neighbours (left/right this row, up/down the neighbour rows); at a poly-id
//   discontinuity where a neighbour is DEEPER (closer wins -> this pixel is the edge), either
//   recolour from RenderEdgeTable (edge-marking) or blend toward that neighbour's colour (AA
//   approximation -- "apply AA from the neighbours"). Then per-pixel fog from the tile depth.
// ---------------------------------------------------------------------------------------------
void TileRenderer3D::PostPassRow(s32 y, u32* dst, const u32* color, const u32* depthid,
                                 const u32* upColor, const u32* upDepthId,
                                 const u32* downColor, const u32* downDepthId)
{
    (void)y;
    const FrameCfg& cfg = Cfg[GeomRead];
    const u32 dispcnt = cfg.DispCnt;
    const bool edgemark = (dispcnt & (1<<5)) != 0;
    const bool aa       = (dispcnt & (1<<4)) != 0;
    const bool fog      = (dispcnt & (1<<7)) != 0;
    const bool detect   = edgemark || aa;

    // fog constants (hoisted)
    const bool fogcolor = !(dispcnt & (1<<6));
    u32 fogR = (cfg.FogColor << 1) & 0x3E; if (fogR) fogR++;
    u32 fogG = (cfg.FogColor >> 4) & 0x3E; if (fogG) fogG++;
    u32 fogB = (cfg.FogColor >> 9) & 0x3E; if (fogB) fogB++;
    u32 fogA = (cfg.FogColor >> 16) & 0x1F;

    const u32 borderDI = Cfg[GeomRead].ClearDepthIdVal;
    const u32 borderC  = Cfg[GeomRead].ClearColorVal;

#if defined(LITEV_TILE_POSTVEC) && defined(__aarch64__)
    auto pix = [&](int x) __attribute__((always_inline))
#else
    for (int x = 0; x < OutWidth; x++)
#endif
    {
        const u32 cdi = depthid[x];
        u32 outc = color[x];

        if (detect)
        {
            const u32 cz  = cdi & 0x00FFFFFF;
            const u32 cid = (cdi >> 24) & ID_POLY_MASK;

            // 4 neighbours (depth/id + colour); off-screen -> clear border.
            const u32 lDI = (x > 0)          ? depthid[x-1] : borderDI;
            const u32 lC  = (x > 0)          ? color[x-1]   : borderC;
            const u32 rDI = (x < OutWidth-1) ? depthid[x+1] : borderDI;
            const u32 rC  = (x < OutWidth-1) ? color[x+1]   : borderC;

#ifdef LITEV_TILE_POSTFAST
            // an edge needs a neighbour with another polygon id: none -> the scan below finds nothing
            if (!(((lDI ^ cdi) | (rDI ^ cdi) | (upDepthId[x] ^ cdi) | (downDepthId[x] ^ cdi)) & (ID_POLY_MASK << 24)))
                goto postfog;
#endif
            {
            u32 nDI[4] = { lDI, rDI, upDepthId[x], downDepthId[x] };
            u32 nC [4] = { lC,  rC,  upColor[x],   downColor[x]   };

            bool edge = false;
            u32  nbColor = 0;
            for (int k = 0; k < 4; k++)
            {
                const u32 nz  = nDI[k] & 0x00FFFFFF;
                const u32 nid = (nDI[k] >> 24) & ID_POLY_MASK;
                if (nid != cid && nz > cz)   // neighbour deeper + different poly => this is the edge
                {
                    edge = true;
                    nbColor = nC[k];
                    break;
                }
            }

            if (edge)
            {
                if (edgemark)
                {
                    u16 ec = cfg.EdgeTable[cid >> 3];
                    u32 er = (ec << 1) & 0x3E; if (er) er++;
                    u32 eg = (ec >> 4) & 0x3E; if (eg) eg++;
                    u32 eb = (ec >> 9) & 0x3E; if (eb) eb++;
                    outc = er | (eg << 8) | (eb << 16) | (outc & 0xFF000000);
                }
                else   // AA: blend the edge pixel halfway toward the deeper neighbour's colour
                {
                    u32 sr = outc & 0x3F, sg = (outc >> 8) & 0x3F, sb = (outc >> 16) & 0x3F;
                    u32 nr = nbColor & 0x3F, ng = (nbColor >> 8) & 0x3F, nb = (nbColor >> 16) & 0x3F;
                    outc = ((sr + nr) >> 1) | (((sg + ng) >> 1) << 8) | (((sb + nb) >> 1) << 16)
                         | (outc & 0xFF000000);
                }
            }
            }
        }
#ifdef LITEV_TILE_POSTFAST
    postfog:
#endif

        if (fog && ((cdi >> 24) & ID_FOG))
        {
            u32 density = CalculateFogDensity(cdi & 0x00FFFFFF);
            u32 srcR = outc & 0x3F;
            u32 srcG = (outc >> 8) & 0x3F;
            u32 srcB = (outc >> 16) & 0x3F;
            u32 srcA = (outc >> 24) & 0x1F;
            if (fogcolor)
            {
                srcR = ((fogR * density) + (srcR * (128-density))) >> 7;
                srcG = ((fogG * density) + (srcG * (128-density))) >> 7;
                srcB = ((fogB * density) + (srcB * (128-density))) >> 7;
            }
            srcA = ((fogA * density) + (srcA * (128-density))) >> 7;
            outc = srcR | (srcG << 8) | (srcB << 16) | (srcA << 24);
        }

        dst[x] = outc;
    }
#if defined(LITEV_TILE_POSTVEC) && defined(__aarch64__)
    ;
    // four pixels at a time: a group whose pixels all have no neighbour with another polygon id (or
    // no edge/AA) and no fog to apply is copied; any other group runs the per-pixel code above
    const uint32x4_t idm = vdupq_n_u32(ID_POLY_MASK << 24), fogm = vdupq_n_u32((u32)ID_FOG << 24);
    uint32x4_t prev = vdupq_n_u32(borderDI), cur = vld1q_u32(depthid);
    for (int x = 0; x < OutWidth; x += 4)
    {
        const uint32x4_t next = (x + 4 < OutWidth) ? vld1q_u32(depthid + x + 4) : vdupq_n_u32(borderDI);
        uint32x4_t bad = fog ? vandq_u32(cur, fogm) : vdupq_n_u32(0);
        if (detect)
        {
            const uint32x4_t l = vextq_u32(prev, cur, 3), r = vextq_u32(cur, next, 1);
            const uint32x4_t u = vld1q_u32(upDepthId + x), dn = vld1q_u32(downDepthId + x);
            bad = vorrq_u32(bad, vandq_u32(vorrq_u32(vorrq_u32(veorq_u32(l, cur), veorq_u32(r, cur)),
                                                     vorrq_u32(veorq_u32(u, cur), veorq_u32(dn, cur))), idm));
        }
        if (vmaxvq_u32(bad) == 0) vst1q_u32(dst + x, vld1q_u32(color + x));
        else for (int k = 0; k < 4; k++) pix(x + k);
        prev = cur; cur = next;
    }
#endif
}

#ifdef TILE_SPANFAST
// The span pixel loop of RenderSpanInTile, specialised: TEX = texturing on (shared-cache texels),
// MODE 0 modulate / 1 decal / 2 toon / 3 highlight. Every value is computed exactly as the generic
// loop computes it (same expressions and types; ShadeTexel inlined with the texel passed in), with
// the per-span / per-polygon constants in locals. Attribute k = base[k] + ((d[k] * f) >> 8),
// f = the perspective factor, or 256 - it when bit k of rev is set: Interpolator<0>::Interpolate's
// non-linear case (y0 < y1: y0 + (y1-y0)*f; else y1 + (y0-y1)*(256-f); y0 == y1: d = 0).
template<bool TEX, int MODE>
void TileRenderer3D::SpanFill(BandScratch& bs, const PolyShade& ps, Interpolator<0>& ix, s32 rowbase, s32 x, s32 xlim,
                              s32 zl, s32 zr, bool wbuf, const s32* base, const u32* d, u32 rev)
{
    u32* const color = bs.Color;
    u32* const depth = bs.DepthId;
    const u16* const toontbl = ps.toontbl;
    const u32 polyalpha = ps.polyalpha;
    const u8  alpharef = ps.alpharef;
    const bool wireframe = ps.wireframe;
    const u32 idword = ps.idword;
    const bool translDW = ps.translDepthWrite;
    const u32* const tex = TEX ? ps.tex32 : nullptr;
    const s32 tw = ps.texW, th = ps.texH;
    const u32 tp = ps.texparam;
    const s32 b0 = base[0], b1 = base[1], b2 = base[2], b3 = base[3], b4 = base[4];
    const u32 d0 = d[0], d1 = d[1], d2 = d[2], d3 = d[3], d4 = d[4];

    for (; x <= xlim; x++)
    {
        s32 z;
        if (wbuf) { ix.SetXPos(x); ix.SetXFactor(); z = ix.InterpolateZ(zl, zr); }
        else      { ix.SetXPos(x); z = ix.InterpolateZ(zl, zr); }
        const u32 idx = (u32)(rowbase + x);
        if (!((u32)z < (depth[idx] & 0x00FFFFFF))) continue;
        if (!wbuf) ix.SetXFactor();

        const u32 yf = ix.Factor(), yr = 256u - yf;
        u32 vr = (u32)((s32)((u32)b0 + ((d0 * ((rev & 1)  ? yr : yf)) >> 8)) >> 3) & 0x3F;
        u32 vg = (u32)((s32)((u32)b1 + ((d1 * ((rev & 2)  ? yr : yf)) >> 8)) >> 3) & 0x3F;
        u32 vb = (u32)((s32)((u32)b2 + ((d2 * ((rev & 4)  ? yr : yf)) >> 8)) >> 3) & 0x3F;

        u8 r, g, b, a;
        if constexpr (MODE == 3) { vg = vr; vb = vr; }
        else if constexpr (MODE == 2)
        {
            u16 tc = toontbl[vr >> 1];
            vr = (tc << 1) & 0x3E; if (vr) vr++;
            vg = (tc >> 4) & 0x3E; if (vg) vg++;
            vb = (tc >> 9) & 0x3E; if (vb) vb++;
        }
        if constexpr (TEX)
        {
            const s16 s = (s16)(s32)((u32)b3 + ((d3 * ((rev & 8)  ? yr : yf)) >> 8));
            const s16 t = (s16)(s32)((u32)b4 + ((d4 * ((rev & 16) ? yr : yf)) >> 8));
            s32 si = s >> 4, ti = t >> 4;
            if (tp & (1<<16))
            {
                if (tp & (1<<18)) { if (si & tw) si = (tw-1) - (si & (tw-1)); else si = (si & (tw-1)); }
                else              si &= tw-1;
            }
            else { if (si < 0) si = 0; else if (si >= tw) si = tw-1; }
            if (tp & (1<<17))
            {
                if (tp & (1<<19)) { if (ti & th) ti = (th-1) - (ti & (th-1)); else ti = (ti & (th-1)); }
                else              ti &= th-1;
            }
            else { if (ti < 0) ti = 0; else if (ti >= th) ti = th-1; }
            const u32 texel = tex[(u32)ti * (u32)tw + (u32)si];
            const u8 tr = texel & 0x3F, tg = (texel >> 8) & 0x3F, tb = (texel >> 16) & 0x3F, talpha = texel >> 24;
            if constexpr (MODE == 1)
            {
                if (talpha == 0)       { r = vr; g = vg; b = vb; }
                else if (talpha == 31) { r = tr; g = tg; b = tb; }
                else
                {
                    r = ((tr * talpha) + (vr * (31-talpha))) >> 5;
                    g = ((tg * talpha) + (vg * (31-talpha))) >> 5;
                    b = ((tb * talpha) + (vb * (31-talpha))) >> 5;
                }
                a = polyalpha;
            }
            else
            {
                r = ((tr+1) * (vr+1) - 1) >> 6;
                g = ((tg+1) * (vg+1) - 1) >> 6;
                b = ((tb+1) * (vb+1) - 1) >> 6;
                a = ((talpha+1) * (polyalpha+1) - 1) >> 5;
            }
        }
        else { r = vr; g = vg; b = vb; a = polyalpha; }
        if constexpr (MODE == 3)
        {
            u16 tc = toontbl[vr >> 1];
            u8 hr = (tc << 1) & 0x3E; if (hr) hr++;
            u8 hg = (tc >> 4) & 0x3E; if (hg) hg++;
            u8 hb = (tc >> 9) & 0x3E; if (hb) hb++;
            int rr = r + hr; if (rr > 63) rr = 63; r = (u8)rr;
            int gg = g + hg; if (gg > 63) gg = 63; g = (u8)gg;
            int bb = b + hb; if (bb > 63) bb = 63; b = (u8)bb;
        }
        if (wireframe) a = 31;
        const u32 col = r | (g << 8) | (b << 16) | ((u32)a << 24);

        if (a <= alpharef) continue;   // alpha test
        if (a == 31)
        {
            color[idx] = col;
            depth[idx] = ((u32)z & 0x00FFFFFF) | idword;
        }
        else
            PlotTranslucentTile(bs, idx, col, translDW ? z : -1, idword | (ID_TRANSL << 24));
    }
}

#ifdef LITEV_TILE_SPANLOCAL
// SpanFill with the span's interpolation state in locals (SpanFill reads and writes the Interpolator
// through memory every pixel: the tile stores may alias it). Same values: xv = x - xstart,
// num = (u32)(xv * wl) << 8 and den = xv * wl + (xdiff - xv) * wr (u32) step by constants mod 2^32;
// the z-buffer depth base + ((disp * factor * xrecip_z) >> 13) steps its 64-bit product by
// +-disp * xrecip_z; the w-buffer depth uses the factor as InterpolateZ does. xdiff != 0, non-linear.
template<bool TEX, int MODE, bool WBUF>
void TileRenderer3D::SpanFillLocal(BandScratch& bs, const PolyShade& ps, s32 rowbase, s32 x, s32 xlim, s32 xstart,
                                   s32 xdiff, s32 wl, s32 wr, s32 zl, s32 zr, const s32* base, const u32* d, u32 rev)
{
    u32* const color = bs.Color;
    u32* const depth = bs.DepthId;
    const u16* const toontbl = ps.toontbl;
    const u32 polyalpha = ps.polyalpha;
    const u8  alpharef = ps.alpharef;
    const bool wireframe = ps.wireframe;
    const u32 idword = ps.idword;
    const bool translDW = ps.translDepthWrite;
    const u32* const tex = TEX ? ps.tex32 : nullptr;
    const s32 tw = ps.texW, th = ps.texH;
    const u32 tp = ps.texparam;
    const bool swrap = tp & (1<<16), sflip = tp & (1<<18), twrap = tp & (1<<17), tflip = tp & (1<<19);
    const s32 b0 = base[0], b1 = base[1], b2 = base[2], b3 = base[3], b4 = base[4];
    const u32 d0 = d[0], d1 = d[1], d2 = d[2], d3 = d[3], d4 = d[4];

    s32 xv = x - xstart;
    u32 num = (u32)(xv * wl) << 8;
    u32 den = (u32)(xv * wl) + (u32)((xdiff - xv) * wr);
    const u32 dnum = (u32)wl << 8, dden = (u32)wl - (u32)wr;

    // z-buffer: z = zbase + (P >> 13), P = disp * xrecip_z * factor (exact in 64 bits)
    s32 zbase = zl; s64 P = 0, dP = 0;
    // w-buffer: z = zlo + ((zd * (zup ? f : 256 - f)) >> 8)
    const bool zup = zl < zr;
    const s64 zd = zup ? (s64)(zr - zl) : (s64)(zl - zr);
    const s32 zlo = zup ? zl : zr;
    if (!WBUF && zl != zr)
    {
        const s32 xrecip_z = (1<<22) / xdiff;
        s32 disp; s32 factor;
        if (zup) { zbase = zl; disp = zr - zl; factor = xv; }
        else     { zbase = zr; disp = zl - zr; factor = xdiff - xv; }
        disp >>= 9;
        const s64 K = (s64)disp * xrecip_z;
        P = K * factor;
        dP = zup ? K : -K;
    }

    for (; x <= xlim; x++, num += dnum, den += dden, P += dP)
    {
        u32 yf = 0;
        s32 z;
        if (WBUF)
        {
            yf = den ? num / den : 0;
            z = (zl == zr) ? zl : (s32)(zlo + ((zd * (s64)(zup ? yf : 256u - yf)) >> 8));
        }
        else
            z = (s32)(zbase + (P >> 13));
        const u32 idx = (u32)(rowbase + x);
        if (!((u32)z < (depth[idx] & 0x00FFFFFF))) continue;
        if (!WBUF) yf = den ? num / den : 0;

        const u32 yr = 256u - yf;
        u32 vr = (u32)((s32)((u32)b0 + ((d0 * ((rev & 1)  ? yr : yf)) >> 8)) >> 3) & 0x3F;
        u32 vg = (u32)((s32)((u32)b1 + ((d1 * ((rev & 2)  ? yr : yf)) >> 8)) >> 3) & 0x3F;
        u32 vb = (u32)((s32)((u32)b2 + ((d2 * ((rev & 4)  ? yr : yf)) >> 8)) >> 3) & 0x3F;

        u8 r, g, b, a;
        if constexpr (MODE == 3) { vg = vr; vb = vr; }
        else if constexpr (MODE == 2)
        {
            u16 tc = toontbl[vr >> 1];
            vr = (tc << 1) & 0x3E; if (vr) vr++;
            vg = (tc >> 4) & 0x3E; if (vg) vg++;
            vb = (tc >> 9) & 0x3E; if (vb) vb++;
        }
        if constexpr (TEX)
        {
            const s16 s = (s16)(s32)((u32)b3 + ((d3 * ((rev & 8)  ? yr : yf)) >> 8));
            const s16 t = (s16)(s32)((u32)b4 + ((d4 * ((rev & 16) ? yr : yf)) >> 8));
            s32 si = s >> 4, ti = t >> 4;
            if (swrap)
            {
                if (sflip) { if (si & tw) si = (tw-1) - (si & (tw-1)); else si = (si & (tw-1)); }
                else       si &= tw-1;
            }
            else { if (si < 0) si = 0; else if (si >= tw) si = tw-1; }
            if (twrap)
            {
                if (tflip) { if (ti & th) ti = (th-1) - (ti & (th-1)); else ti = (ti & (th-1)); }
                else       ti &= th-1;
            }
            else { if (ti < 0) ti = 0; else if (ti >= th) ti = th-1; }
            const u32 texel = tex[(u32)ti * (u32)tw + (u32)si];
            const u8 tr = texel & 0x3F, tg = (texel >> 8) & 0x3F, tb = (texel >> 16) & 0x3F, talpha = texel >> 24;
            if constexpr (MODE == 1)
            {
                if (talpha == 0)       { r = vr; g = vg; b = vb; }
                else if (talpha == 31) { r = tr; g = tg; b = tb; }
                else
                {
                    r = ((tr * talpha) + (vr * (31-talpha))) >> 5;
                    g = ((tg * talpha) + (vg * (31-talpha))) >> 5;
                    b = ((tb * talpha) + (vb * (31-talpha))) >> 5;
                }
                a = polyalpha;
            }
            else
            {
                r = ((tr+1) * (vr+1) - 1) >> 6;
                g = ((tg+1) * (vg+1) - 1) >> 6;
                b = ((tb+1) * (vb+1) - 1) >> 6;
                a = ((talpha+1) * (polyalpha+1) - 1) >> 5;
            }
        }
        else { r = vr; g = vg; b = vb; a = polyalpha; }
        if constexpr (MODE == 3)
        {
            u16 tc = toontbl[vr >> 1];
            u8 hr = (tc << 1) & 0x3E; if (hr) hr++;
            u8 hg = (tc >> 4) & 0x3E; if (hg) hg++;
            u8 hb = (tc >> 9) & 0x3E; if (hb) hb++;
            int rr = r + hr; if (rr > 63) rr = 63; r = (u8)rr;
            int gg = g + hg; if (gg > 63) gg = 63; g = (u8)gg;
            int bb = b + hb; if (bb > 63) bb = 63; b = (u8)bb;
        }
        if (wireframe) a = 31;
        const u32 col = r | (g << 8) | (b << 16) | ((u32)a << 24);

        if (a <= alpharef) continue;   // alpha test
        if (a == 31)
        {
            color[idx] = col;
            depth[idx] = ((u32)z & 0x00FFFFFF) | idword;
        }
        else
            PlotTranslucentTile(bs, idx, col, translDW ? z : -1, idword | (ID_TRANSL << 24));
    }
}

#if defined(LITEV_TILE_SPANVEC) && defined(__aarch64__)
// SpanFillLocal (z-buffer, modulate or decal) four pixels at a time: the same per-pixel values in
// u32/s32/s64 lanes (num/den/the 64-bit depth product step per lane, the divide per lane, the
// attributes base + ((d * f) >> 8) in u32, s16 texcoords, clamp/repeat/flip, texel gather,
// modulate/decal, alpha test). Opaque pixels are written as a masked 4-wide store, translucent
// ones go through PlotTranslucentTile one by one (every pixel of a span has its own index, so the
// order between them doesn't matter). The last < 4 pixels run in SpanFillLocal.
// Inspired by DSperate (https://github.com/beebono/DSperate, GPL-3.0-or-later, commit f5fa35e,
// src/core/gpu/render3d.cpp span_shade / kernels_neon.cpp span_attrs: NEON span shading). No code copied.
template<bool TEX, bool DECAL, bool WBUF>
void TileRenderer3D::SpanVec(BandScratch& bs, const PolyShade& ps, s32 rowbase, s32 x, s32 xlim, s32 xstart,
                             s32 xdiff, s32 wl, s32 wr, s32 zl, s32 zr, const s32* base, const u32* d, u32 rev)
{
    if (x + 3 <= xlim)
    {
        u32* const color = bs.Color;
        u32* const depth = bs.DepthId;
        const u32 polyalpha = ps.polyalpha;
        const u32 alpharef = ps.alpharef;
        const u32 idword = ps.idword;
        const bool translDW = ps.translDepthWrite;
        const u32* const tex = TEX ? ps.tex32 : nullptr;
        const s32 tw = ps.texW, th = ps.texH;
        const u32 tp = ps.texparam;
        const bool swrap = tp & (1<<16), sflip = tp & (1<<18), twrap = tp & (1<<17), tflip = tp & (1<<19);

        static const s32 laneIdx[4] = { 0, 1, 2, 3 };
        const int32x4_t lane = vld1q_s32(laneIdx);
        const uint32x4_t lane_u = vreinterpretq_u32_s32(lane);

        const s32 xv = x - xstart;
        const u32 num0 = (u32)(xv * wl) << 8;
        const u32 den0 = (u32)(xv * wl) + (u32)((xdiff - xv) * wr);
        const u32 dnum = (u32)wl << 8, dden = (u32)wl - (u32)wr;
        uint32x4_t numv = vmlaq_n_u32(vdupq_n_u32(num0), lane_u, dnum);
        uint32x4_t denv = vmlaq_n_u32(vdupq_n_u32(den0), lane_u, dden);
        const uint32x4_t dnum4 = vdupq_n_u32(dnum * 4), dden4 = vdupq_n_u32(dden * 4);

        s32 zbase = zl; s64 P = 0, dP = 0;
        // w-buffer: z = zlo + ((zd * (zup ? f : 256 - f)) >> 8); zd < 2^31 and the u32 factor keep the
        // product below 2^63, so the unsigned 32x32->64 multiply gives the signed product
        const bool zup = zl < zr;
        const s64 zd = zup ? (s64)(zr - zl) : (s64)(zl - zr);
        if (WBUF && zd < 0) { SpanFillLocal<TEX, DECAL ? 1 : 0, true>(bs, ps, rowbase, x, xlim, xstart, xdiff, wl, wr, zl, zr, base, d, rev); return; }
        const uint32x2_t zdv = vdup_n_u32((u32)zd);
        const int32x4_t zlov = vdupq_n_s32(zup ? zl : zr);
        if (!WBUF && zl != zr)
        {
            const s32 xrecip_z = (1<<22) / xdiff;
            s32 disp, factor;
            if (zl < zr) { zbase = zl; disp = zr - zl; factor = xv; }
            else         { zbase = zr; disp = zl - zr; factor = xdiff - xv; }
            disp >>= 9;
            const s64 K = (s64)disp * xrecip_z;
            P = K * factor;
            dP = (zl < zr) ? K : -K;
        }
        int64x2_t Plo = { P, P + dP }, Phi = { P + 2*dP, P + 3*dP };
        const int64x2_t dP4 = vdupq_n_s64(dP * 4);
        const int32x4_t zbv = vdupq_n_s32(zbase);

        const uint32x4_t m24 = vdupq_n_u32(0x00FFFFFF), m3f = vdupq_n_u32(0x3F), one = vdupq_n_u32(1);
        const uint32x4_t v256 = vdupq_n_u32(256), v31 = vdupq_n_u32(31), v0 = vdupq_n_u32(0);
        const uint32x4_t vpa = vdupq_n_u32(polyalpha), vref = vdupq_n_u32(alpharef), vid = vdupq_n_u32(idword);
        const int32x4_t twm1 = vdupq_n_s32(tw - 1), thm1 = vdupq_n_s32(th - 1), vtw = vdupq_n_s32(tw), vth = vdupq_n_s32(th), zero_s = vdupq_n_s32(0);
        uint32x4_t bv[5], dv[5];
        for (int k = 0; k < 5; k++) { bv[k] = vdupq_n_u32((u32)base[k]); dv[k] = vdupq_n_u32(d[k]); }

        for (; x + 3 <= xlim; x += 4)
        {
            const uint32x4_t nv = numv, dn = denv;
            numv = vaddq_u32(numv, dnum4); denv = vaddq_u32(denv, dden4);
            auto divide = [&]() __attribute__((always_inline)) {
                u32 na[4], da[4], q[4];
                vst1q_u32(na, nv); vst1q_u32(da, dn);
                for (int k = 0; k < 4; k++) q[k] = da[k] ? na[k] / da[k] : 0;
                return vld1q_u32(q);
            };
            const u32 idx = (u32)(rowbase + x);
            const uint32x4_t dold = vld1q_u32(&depth[idx]);
            int32x4_t z; uint32x4_t yf;
            if (WBUF)
            {
                yf = divide();
                if (zl == zr) z = zlov;
                else
                {
                    const uint32x4_t fz = zup ? yf : vsubq_u32(v256, yf);
                    const uint64x2_t plo = vmull_u32(zdv, vget_low_u32(fz)), phi = vmull_u32(zdv, vget_high_u32(fz));
                    z = vaddq_s32(zlov, vreinterpretq_s32_u32(vcombine_u32(vmovn_u64(vshrq_n_u64(plo, 8)), vmovn_u64(vshrq_n_u64(phi, 8)))));
                }
            }
            else
            {
                z = vaddq_s32(zbv, vcombine_s32(vmovn_s64(vshrq_n_s64(Plo, 13)), vmovn_s64(vshrq_n_s64(Phi, 13))));
                Plo = vaddq_s64(Plo, dP4); Phi = vaddq_s64(Phi, dP4);
            }
            const uint32x4_t pass = vcltq_u32(vreinterpretq_u32_s32(z), vandq_u32(dold, m24));
            if (vmaxvq_u32(pass) == 0) continue;
            if (!WBUF) yf = divide();
            const uint32x4_t yr = vsubq_u32(v256, yf);
            uint32x4_t att[5];
            for (int k = 0; k < (TEX ? 5 : 3); k++)
                att[k] = vaddq_u32(bv[k], vshrq_n_u32(vmulq_u32(dv[k], (rev & (1u << k)) ? yr : yf), 8));
            const uint32x4_t vr = vandq_u32(vreinterpretq_u32_s32(vshrq_n_s32(vreinterpretq_s32_u32(att[0]), 3)), m3f);
            const uint32x4_t vg = vandq_u32(vreinterpretq_u32_s32(vshrq_n_s32(vreinterpretq_s32_u32(att[1]), 3)), m3f);
            const uint32x4_t vb = vandq_u32(vreinterpretq_u32_s32(vshrq_n_s32(vreinterpretq_s32_u32(att[2]), 3)), m3f);

            uint32x4_t r, g, b, a;
            if constexpr (TEX)
            {
                // (s16) then >> 4
                int32x4_t si = vshrq_n_s32(vshlq_n_s32(vreinterpretq_s32_u32(att[3]), 16), 20);
                int32x4_t ti = vshrq_n_s32(vshlq_n_s32(vreinterpretq_s32_u32(att[4]), 16), 20);
                if (swrap)
                {
                    const int32x4_t m = vandq_s32(si, twm1);
                    si = sflip ? vbslq_s32(vtstq_s32(si, vtw), vsubq_s32(twm1, m), m) : m;
                }
                else si = vmaxq_s32(vminq_s32(si, twm1), zero_s);
                if (twrap)
                {
                    const int32x4_t m = vandq_s32(ti, thm1);
                    ti = tflip ? vbslq_s32(vtstq_s32(ti, vth), vsubq_s32(thm1, m), m) : m;
                }
                else ti = vmaxq_s32(vminq_s32(ti, thm1), zero_s);
                const uint32x4_t ti_u = vmlaq_u32(vreinterpretq_u32_s32(si), vreinterpretq_u32_s32(ti), vreinterpretq_u32_s32(vtw));
                uint32x4_t texel = vdupq_n_u32(tex[vgetq_lane_u32(ti_u, 0)]);
                texel = vsetq_lane_u32(tex[vgetq_lane_u32(ti_u, 1)], texel, 1);
                texel = vsetq_lane_u32(tex[vgetq_lane_u32(ti_u, 2)], texel, 2);
                texel = vsetq_lane_u32(tex[vgetq_lane_u32(ti_u, 3)], texel, 3);
                const uint32x4_t tr = vandq_u32(texel, m3f), tg = vandq_u32(vshrq_n_u32(texel, 8), m3f);
                const uint32x4_t tb = vandq_u32(vshrq_n_u32(texel, 16), m3f), ta = vshrq_n_u32(texel, 24);
                if constexpr (DECAL)
                {
                    const uint32x4_t inv = vsubq_u32(v31, ta);
                    const uint32x4_t a0 = vceqq_u32(ta, v0), a31 = vceqq_u32(ta, v31);
                    r = vbslq_u32(a0, vr, vbslq_u32(a31, tr, vshrq_n_u32(vmlaq_u32(vmulq_u32(tr, ta), vr, inv), 5)));
                    g = vbslq_u32(a0, vg, vbslq_u32(a31, tg, vshrq_n_u32(vmlaq_u32(vmulq_u32(tg, ta), vg, inv), 5)));
                    b = vbslq_u32(a0, vb, vbslq_u32(a31, tb, vshrq_n_u32(vmlaq_u32(vmulq_u32(tb, ta), vb, inv), 5)));
                    a = vpa;
                }
                else
                {
                    r = vshrq_n_u32(vsubq_u32(vmulq_u32(vaddq_u32(tr, one), vaddq_u32(vr, one)), one), 6);
                    g = vshrq_n_u32(vsubq_u32(vmulq_u32(vaddq_u32(tg, one), vaddq_u32(vg, one)), one), 6);
                    b = vshrq_n_u32(vsubq_u32(vmulq_u32(vaddq_u32(tb, one), vaddq_u32(vb, one)), one), 6);
                    a = vshrq_n_u32(vsubq_u32(vmulq_u32(vaddq_u32(ta, one), vaddq_u32(vpa, one)), one), 5);
                }
            }
            else { r = vr; g = vg; b = vb; a = vpa; }
            if (ps.wireframe) a = v31;
            const uint32x4_t col = vorrq_u32(vorrq_u32(r, vshlq_n_u32(g, 8)), vorrq_u32(vshlq_n_u32(b, 16), vshlq_n_u32(a, 24)));
            const uint32x4_t ok = vandq_u32(pass, vcgtq_u32(a, vref));
            const uint32x4_t opq = vandq_u32(ok, vceqq_u32(a, v31));
            if (vmaxvq_u32(opq))
            {
                vst1q_u32(&color[idx], vbslq_u32(opq, col, vld1q_u32(&color[idx])));
                vst1q_u32(&depth[idx], vbslq_u32(opq, vorrq_u32(vandq_u32(vreinterpretq_u32_s32(z), m24), vid), dold));
            }
            const uint32x4_t trl = vbicq_u32(ok, opq);
            if (vmaxvq_u32(trl))
            {
                u32 tm[4], cl[4]; s32 zz[4];
                vst1q_u32(tm, trl); vst1q_u32(cl, col); vst1q_s32(zz, z);
                for (int k = 0; k < 4; k++)
                    if (tm[k]) PlotTranslucentTile(bs, idx + k, cl[k], translDW ? zz[k] : -1, idword | (ID_TRANSL << 24));
            }
        }
    }
    if (x <= xlim)
        SpanFillLocal<TEX, DECAL ? 1 : 0, WBUF>(bs, ps, rowbase, x, xlim, xstart, xdiff, wl, wr, zl, zr, base, d, rev);
}
#endif
#endif
#endif

// Span fill for one scanline of one polygon into the 2-plane tile. Perspective-correct S/T and
// colour interpolation, per-pixel alpha test, less-than depth test. Opaque -> write colour + (depth
// | id). Translucent (shaded alpha < 31) -> DraStic 2-plane blend. NO per-pixel AA coverage.
void TileRenderer3D::RenderSpanInTile(BandScratch& bs, const TilePoly& poly, s32 ty0, const PolyShade& ps,
                                      Slope<0>& sl, Slope<1>& sr,
                                      u32 cvl, u32 nvl, u32 cvr, u32 nvr,
                                      s32 xstart, s32 xend, s32 y)
{
    const TileVtx* V = &VtxArena[GeomRead][poly.vtxBase];
    const bool wbuf = poly.wbuffer;

    s32 wl = sl.Interp.Interpolate(V[cvl].w, V[nvl].w);
    s32 wr = sr.Interp.Interpolate(V[cvr].w, V[nvr].w);
    s32 zl = sl.Interp.InterpolateZ(V[cvl].z, V[nvl].z);
    s32 zr = sr.Interp.InterpolateZ(V[cvr].z, V[nvr].z);

    // Right vertical edges are pushed 1px left (matches the reference).
    if (sr.Increment == 0 && (sl.Increment != 0 || xstart != xend) && (xend != 0))
        xend--;

    const Interpolator<1>* is;
    const Interpolator<1>* ie;
    u32 clc, cln, crc, crn;
    if (xstart > xend)
    {
        clc = cvr; cln = nvr; crc = cvl; crn = nvl;
        is = &sr.Interp; ie = &sl.Interp;
        s32 t;
        t = xstart; xstart = xend; xend = t;
        t = wl; wl = wr; wr = t;
        t = zl; zl = zr; zr = t;
    }
    else
    {
        clc = cvl; cln = nvl; crc = cvr; crn = nvr;
        is = &sl.Interp; ie = &sr.Interp;
    }

    const s32 rl = is->Interpolate(V[clc].r, V[cln].r);
    const s32 gl = is->Interpolate(V[clc].g, V[cln].g);
    const s32 bl = is->Interpolate(V[clc].b, V[cln].b);
    const s32 rr = ie->Interpolate(V[crc].r, V[crn].r);
    const s32 gr = ie->Interpolate(V[crc].g, V[crn].g);
    const s32 br = ie->Interpolate(V[crc].b, V[crn].b);

    const s32 sl_s = is->Interpolate(V[clc].s, V[cln].s);
    const s32 sl_t = is->Interpolate(V[clc].t, V[cln].t);
    const s32 sr_s = ie->Interpolate(V[crc].s, V[crn].s);
    const s32 sr_t = ie->Interpolate(V[crc].t, V[crn].t);

    Interpolator<0> ix(xstart, xend + 1, wl, wr, wbuf);

    s32 x = xstart; if (x < 0) x = 0;
    s32 xlim = xend; if (xlim > OutWidth - 1) xlim = OutWidth - 1;

    const s32 rowbase = (y - ty0) * OutWidth;
#ifdef TILE_SPANFAST
    if (!ix.Linear())
    {
        s32 base[5]; u32 dd[5]; u32 rev = 0;
        const s32 y0s[5] = { rl, gl, bl, sl_s, sl_t }, y1s[5] = { rr, gr, br, sr_s, sr_t };
        for (int k = 0; k < 5; k++)
        {
            if (y0s[k] <= y1s[k]) { base[k] = y0s[k]; dd[k] = (u32)(y1s[k] - y0s[k]); }
            else                  { base[k] = y1s[k]; dd[k] = (u32)(y0s[k] - y1s[k]); rev |= 1u << k; }
        }
        const int mode = ps.decal ? 1 : ps.toon ? (ps.highlight ? 3 : 2) : 0;
#ifdef LITEV_TILE_SPANLOCAL
        const s32 xdiff = xend + 1 - xstart;
        if (xdiff != 0)
        {
#if defined(LITEV_TILE_SPANVEC) && defined(__aarch64__)
            if (mode < 2)
            {
#define SV(T, D) (wbuf ? SpanVec<T, D, true>(bs, ps, rowbase, x, xlim, xstart, xdiff, wl, wr, zl, zr, base, dd, rev) \
                       : SpanVec<T, D, false>(bs, ps, rowbase, x, xlim, xstart, xdiff, wl, wr, zl, zr, base, dd, rev))
                if (ps.texEnable) { if (mode) SV(true, true); else SV(true, false); }
                else              { if (mode) SV(false, true); else SV(false, false); }
#undef SV
                return;
            }
#endif
#define SFL(T, M) (wbuf ? SpanFillLocal<T, M, true>(bs, ps, rowbase, x, xlim, xstart, xdiff, wl, wr, zl, zr, base, dd, rev) \
                        : SpanFillLocal<T, M, false>(bs, ps, rowbase, x, xlim, xstart, xdiff, wl, wr, zl, zr, base, dd, rev))
            switch ((ps.texEnable ? 4 : 0) | mode)
            {
            case 0: SFL(false, 0); break;
            case 1: SFL(false, 1); break;
            case 2: SFL(false, 2); break;
            case 3: SFL(false, 3); break;
            case 4: SFL(true, 0); break;
            case 5: SFL(true, 1); break;
            case 6: SFL(true, 2); break;
            default: SFL(true, 3); break;
            }
#undef SFL
            return;
        }
#endif
        switch ((ps.texEnable ? 4 : 0) | mode)
        {
        case 0: SpanFill<false, 0>(bs, ps, ix, rowbase, x, xlim, zl, zr, wbuf, base, dd, rev); break;
        case 1: SpanFill<false, 1>(bs, ps, ix, rowbase, x, xlim, zl, zr, wbuf, base, dd, rev); break;
        case 2: SpanFill<false, 2>(bs, ps, ix, rowbase, x, xlim, zl, zr, wbuf, base, dd, rev); break;
        case 3: SpanFill<false, 3>(bs, ps, ix, rowbase, x, xlim, zl, zr, wbuf, base, dd, rev); break;
        case 4: SpanFill<true, 0>(bs, ps, ix, rowbase, x, xlim, zl, zr, wbuf, base, dd, rev); break;
        case 5: SpanFill<true, 1>(bs, ps, ix, rowbase, x, xlim, zl, zr, wbuf, base, dd, rev); break;
        case 6: SpanFill<true, 2>(bs, ps, ix, rowbase, x, xlim, zl, zr, wbuf, base, dd, rev); break;
        default: SpanFill<true, 3>(bs, ps, ix, rowbase, x, xlim, zl, zr, wbuf, base, dd, rev); break;
        }
        return;
    }
#endif
    for (; x <= xlim; x++)
    {
        ix.SetX(x);
        const s32 z = ix.InterpolateZ(zl, zr);
        const u32 idx = (u32)(rowbase + x);

        // less-than depth test (closer wins)
        if ((u32)z < (bs.DepthId[idx] & 0x00FFFFFF))
        {
            const u32 vr = (u32)(ix.Interpolate(rl, rr) >> 3) & 0x3F;   // FinalColor(9-bit) -> 6-bit
            const u32 vg = (u32)(ix.Interpolate(gl, gr) >> 3) & 0x3F;
            const u32 vb = (u32)(ix.Interpolate(bl, br) >> 3) & 0x3F;
            const s16 s  = (s16)ix.Interpolate(sl_s, sr_s);
            const s16 t  = (s16)ix.Interpolate(sl_t, sr_t);

            const u32 color = ShadeTexel(ps, vr, vg, vb, s, t);
            const u8  alpha = (u8)(color >> 24);

            if (alpha <= ps.alpharef) continue;   // alpha test

            if (alpha == 31)
            {
                bs.Color[idx]   = color;
                bs.DepthId[idx] = ((u32)z & 0x00FFFFFF) | ps.idword;
            }
            else
            {
                // translucent: conditional depth write (Attr bit 11); id gets the translucent bit.
                const s32 plotZ = ps.translDepthWrite ? z : -1;
                PlotTranslucentTile(bs, idx, color, plotZ, ps.idword | (ID_TRANSL << 24));
            }
        }
    }
}

// Rasterize one polygon's rows in [ty0, ty1) into the tile (compact-vertex reads). Sets up the
// left/right edge walk at the tile's first relevant scanline and steps down.
void TileRenderer3D::RasterPolyInTile(BandScratch& bs, const TilePoly& poly, s32 ty0, s32 ty1)
{
    const TileVtx* V = &VtxArena[GeomRead][poly.vtxBase];
    const u32 nverts = poly.numVerts;
    const bool wbuf  = poly.wbuffer;
    const u32 vbot   = poly.vbot;

    u32 cvl = poly.vtop, cvr = poly.vtop, nvl, nvr;
    if (poly.facing)
    {
        nvl = cvl + 1; if (nvl >= nverts) nvl = 0;
        nvr = (cvr == 0) ? nverts - 1 : cvr - 1;
    }
    else
    {
        nvl = (cvl == 0) ? nverts - 1 : cvl - 1;
        nvr = cvr + 1; if (nvr >= nverts) nvr = 0;
    }

    Slope<0> sl;
    Slope<1> sr;

    const s32 ystart = (poly.ytop > ty0) ? poly.ytop : ty0;
    const s32 yend   = (poly.ybot < ty1) ? poly.ybot : ty1;
    if (ystart >= yend) return;

    // Per-poly shading constants (hoisted once; ResolveTexCache decodes this poly's texture once).
    PolyShade ps;
    {
        const u32 attr      = poly.attr;
        const u32 blendmode = (attr >> 4) & 0x3;
        const FrameCfg& cfg = Cfg[GeomRead];
        const u32 dispcnt   = cfg.DispCnt;
        ps.toon      = (blendmode == 2);
        ps.highlight = ps.toon && (dispcnt & (1<<1));
        ps.decal     = (blendmode & 0x1) != 0;
        ps.texparam  = poly.texparam;
        const u32 texfmt = (ps.texparam >> 26) & 0x7;
        ps.texEnable = (dispcnt & 0x1) && (texfmt != 0);
        ps.texpal    = poly.texpal;
        ps.toontbl   = cfg.ToonTable;
        ps.polyalpha = (attr >> 16) & 0x1F;
        ps.wireframe = (ps.polyalpha == 0);
        ps.alpharef  = cfg.AlphaRef;
        // DraStic id byte pre-shifted into bits 24-31: poly-id (opaque, 6 bits) | fog-enable (bit 6).
        const u32 polyid = (attr >> 24) & ID_POLY_MASK;
        const u32 fogbit = (attr & (1u<<15)) ? ID_FOG : 0;
        ps.idword    = ((polyid | fogbit) << 24);
        ps.translDepthWrite = (attr & (1u<<11)) != 0;
        ps.texcache  = nullptr;
        ps.texW = 0; ps.texH = 0;
        // the cache keeps 1 bit of alpha: the graded-alpha formats (A3I5 = 1, A5I3 = 6) look texels
        // up directly, else their soft alpha became opaque (Pokemon White's town mist: white blobs)
#ifdef TILE_TEXSH
        if (ps.texEnable)
        {
            ps.tex32 = PolyTex[&poly - FramePolys[GeomRead]];
            ps.texW = 8 << ((ps.texparam >> 20) & 0x7);
            ps.texH = 8 << ((ps.texparam >> 23) & 0x7);
        }
#else
        if (ps.texEnable && texfmt != 1 && texfmt != 6)
            ps.texcache = ResolveTexCache(bs.Tex, ps.texparam, ps.texpal, &ps.texW, &ps.texH);
#endif
    }

    auto advL = [&](s32 yy) -> s32
    {
        while (yy >= V[nvl].y && cvl != vbot)
        {
            cvl = nvl;
            if (poly.facing) { nvl = cvl + 1; if (nvl >= nverts) nvl = 0; }
            else             { nvl = (cvl == 0) ? nverts - 1 : cvl - 1; }
        }
        return sl.Setup(V[cvl].x, V[nvl].x, V[cvl].y, V[nvl].y, V[cvl].w, V[nvl].w, yy, wbuf);
    };
    auto advR = [&](s32 yy) -> s32
    {
        while (yy >= V[nvr].y && cvr != vbot)
        {
            cvr = nvr;
            if (poly.facing) { nvr = (cvr == 0) ? nverts - 1 : cvr - 1; }
            else             { nvr = cvr + 1; if (nvr >= nverts) nvr = 0; }
        }
        return sr.Setup(V[cvr].x, V[nvr].x, V[cvr].y, V[nvr].y, V[cvr].w, V[nvr].w, yy, wbuf);
    };

    s32 xl = advL(ystart);
    s32 xr = advR(ystart);

    for (s32 y = ystart; y < yend; y++)
    {
        if (y >= V[nvl].y && cvl != vbot) xl = advL(y);
        if (y >= V[nvr].y && cvr != vbot) xr = advR(y);

        RenderSpanInTile(bs, poly, ty0, ps, sl, sr, cvl, nvl, cvr, nvr, xl, xr, y);

        xl = sl.Step();
        xr = sr.Step();
    }
}

// Per-frame clear/config, computed on the EMU thread before the workers wake (read-only in workers).
void TileRenderer3D::ComputeClearConfig()
{
    // Per-frame 3D clear (uniform colour/depth/id). Clear-IMAGE (bit 14) is scoped out.
    const u32 clearz = (u32)((((GPU3D.RenderClearAttr2 & 0x7FFF) * 0x200) + 0x1FF) & 0x00FFFFFF);
    {
        u32 r = (GPU3D.RenderClearAttr1 << 1) & 0x3E; if (r) r++;
        u32 g = (GPU3D.RenderClearAttr1 >> 4) & 0x3E; if (g) g++;
        u32 b = (GPU3D.RenderClearAttr1 >> 9) & 0x3E; if (b) b++;
        u32 a = (GPU3D.RenderClearAttr1 >> 16) & 0x1F;
        Cfg[GeomWrite].ClearColorVal = r | (g << 8) | (b << 16) | (a << 24);
    }
    {
        const u32 polyid = (GPU3D.RenderClearAttr1 >> 24) & ID_POLY_MASK;
        const u32 fogbit = (GPU3D.RenderClearAttr1 & (1u<<15)) ? ID_FOG : 0;
        Cfg[GeomWrite].ClearDepthIdVal = clearz | ((polyid | fogbit) << 24);
    }
    Cfg[GeomWrite].DoPost   = (GPU3D.RenderDispCnt & ((1<<4) | (1<<5) | (1<<7))) != 0;
    Cfg[GeomWrite].NeedHalo = (GPU3D.RenderDispCnt & ((1<<4) | (1<<5))) != 0;   // edge/AA read up/down; fog doesn't
    FrameCfg& c = Cfg[GeomWrite];
    c.DispCnt  = GPU3D.RenderDispCnt;
    c.AlphaRef = GPU3D.RenderAlphaRef;
    memcpy(c.ToonTable, GPU3D.RenderToonTable, sizeof(c.ToonTable));
    memcpy(c.EdgeTable, GPU3D.RenderEdgeTable, sizeof(c.EdgeTable));
    c.FogColor  = GPU3D.RenderFogColor;
    c.FogOffset = GPU3D.RenderFogOffset;
    c.FogShift  = GPU3D.RenderFogShift;
    memcpy(c.FogDensityTable, GPU3D.RenderFogDensityTable, sizeof(c.FogDensityTable));
    for (int x = 0; x < OutWidth; x++) { Cfg[GeomWrite].BorderColor[x] = Cfg[GeomWrite].ClearColorVal; Cfg[GeomWrite].BorderDepthId[x] = Cfg[GeomWrite].ClearDepthIdVal; }
}

// One worker's share of the frame: a CONTIGUOUS block range [idx*BPW, (idx+1)*BPW), rows
// [idx*64, idx*64+64). The worker rasters its blocks into its OWN L1D-resident tile (Band[idx]) and
// composites its rows STRICTLY TOP-TO-BOTTOM -- a within-range deferred carry threads the edge/AA
// post-pass across the range's internal block boundaries -- so RowReady[] climbs in order and the
// in-order 2D consumer (GetLine) pipelines behind it (row 0 is ready almost immediately). Only the
// inter-worker SEAM rows (this range's first row, up-neighbour in the previous worker's range; and
// last row, down-neighbour in the next worker's range) are deferred: the worker publishes its 4
// boundary raster rows and the last worker finishes the ~4 seam rows after the barrier.
void TileRenderer3D::WorkerRender(int idx)
{
    BandScratch& bs = Band[idx];
    const int gr = GeomRead;
    const u16 (&Bucket)[NumBlocks][MaxPolys] = this->Bucket[gr];
    const u16 (&BucketCount)[NumBlocks] = this->BucketCount[gr];
    const TilePoly (&FramePolys)[MaxPolys] = this->FramePolys[gr];
    const TileVtx* VtxArena = this->VtxArena[gr];
#ifdef LITEV_TILE_COORD
    const int ci = WorkerColIdx;
#else
    const int ci = RenderIdx;
#endif
    u32* out = ColorOut[ci];
    std::atomic<u8>* rdy = RowReady[ci];          // this frame's row-ready bank (per-buffer)

    const int Blo = idx * BlocksPerWorker;
    const int Bhi = Blo + BlocksPerWorker;   // exclusive

    // Within-range deferred carry (LOCAL to this call): a block's last row + its up-neighbour,
    // held until the NEXT block in THIS range supplies the down-neighbour.
    bool carryValid = false;
    s32  carryY = 0;
    u32  carryColor[OutWidth], carryDepthId[OutWidth];
    u32  carryUpColor[OutWidth], carryUpDepthId[OutWidth];

    for (int block = Blo; block < Bhi; block++)
    {
        const s32 ty0 = block * TileH;
        ClearTile(bs);

        // Only the polys bucketed to THIS block (fix #3), in RenderPolygonRAM order (opaque-first
        // then translucent back-to-front) -- the correct DS draw order.
        const u16* bucket = Bucket[block];
        const u32  bcount = BucketCount[block];
        for (u32 bi = 0; bi < bcount; bi++)
            RasterPolyInTile(bs, FramePolys[bucket[bi]], ty0, ty0 + TileH);

        // Publish this range's FIRST 2 raster rows + flag them, so worker idx-1 can finish the
        // (idx-1|idx) seam early. (Worker idx uses its own local carry for its (idx|idx+1) seam, so
        // only the range-first rows need publishing.)
        if (block == Blo)
        {
            memcpy(SeamColor  [idx][0], &bs.Color  [0],        OutWidth * sizeof(u32));
            memcpy(SeamDepthId[idx][0], &bs.DepthId[0],        OutWidth * sizeof(u32));
            memcpy(SeamColor  [idx][1], &bs.Color  [OutWidth], OutWidth * sizeof(u32));
            memcpy(SeamDepthId[idx][1], &bs.DepthId[OutWidth], OutWidth * sizeof(u32));
            SeamFirstReady[idx].store(1, std::memory_order_release);
        }

        if (!Cfg[GeomRead].DoPost)
        {
            FlushTile(bs, out, ty0);
            for (int r = 0; r < TileH; r++)                    // per-row readiness (see GetLine)
                PublishRow(rdy, ty0 + r);
            WakeConsumer();
            continue;
        }

        if (!Cfg[GeomRead].NeedHalo)
        {
            // fog-only: same-(x,y), no neighbours -> composite every row in order.
            for (int r = 0; r < TileH; r++)
            {
                const s32 y = ty0 + r;
                const s32 o = r * OutWidth;
                PostPassRow(y, &out[y * OutWidth], &bs.Color[o], &bs.DepthId[o],
                            Cfg[GeomRead].BorderColor, Cfg[GeomRead].BorderDepthId, Cfg[GeomRead].BorderColor, Cfg[GeomRead].BorderDepthId);
                PublishRow(rdy, y);
            }
            WakeConsumer();
            continue;
        }

        // edge/AA: within-range carry, producing rows in order.
        // (1) finish the previous block's deferred last row using THIS block's row 0 as its down.
        if (carryValid)
        {
            PostPassRow(carryY, &out[carryY * OutWidth], carryColor, carryDepthId,
                        carryUpColor, carryUpDepthId, &bs.Color[0], &bs.DepthId[0]);
            PublishRow(rdy, carryY);
        }

        // (2) composite rows [rStart, TileH-1). Skip the range-FIRST seam row (idx>0, row Rlo): its
        //     up-neighbour is the previous worker's range -> finished by worker idx-1's seam pass.
        const int rStart = (block == Blo && idx > 0) ? 1 : 0;
        for (int r = rStart; r < TileH - 1; r++)
        {
            const s32 y = ty0 + r;
            const s32 o = r * OutWidth;
            const u32* uc = (r == 0) ? (carryValid ? carryColor   : Cfg[GeomRead].BorderColor)   : &bs.Color  [o-OutWidth];
            const u32* ud = (r == 0) ? (carryValid ? carryDepthId : Cfg[GeomRead].BorderDepthId) : &bs.DepthId[o-OutWidth];
            PostPassRow(y, &out[y * OutWidth], &bs.Color[o], &bs.DepthId[o],
                        uc, ud, &bs.Color[o+OutWidth], &bs.DepthId[o+OutWidth]);
            PublishRow(rdy, y);
        }
        WakeConsumer();

        // (3) defer this block's last row (+ its up-neighbour) to the next block in this range.
        const int last = TileH - 1;
        const s32 lo = last * OutWidth;
        memcpy(carryColor,     &bs.Color  [lo],          OutWidth * sizeof(u32));
        memcpy(carryDepthId,   &bs.DepthId[lo],          OutWidth * sizeof(u32));
        memcpy(carryUpColor,   &bs.Color  [lo-OutWidth], OutWidth * sizeof(u32));
        memcpy(carryUpDepthId, &bs.DepthId[lo-OutWidth], OutWidth * sizeof(u32));
        carryY = ty0 + last;
        carryValid = true;
    }

    // Finish this range's LAST row.
    if (Cfg[GeomRead].DoPost && Cfg[GeomRead].NeedHalo && carryValid)
    {
        if (idx == NB - 1)
        {
            // last worker: this range's last row (191) has the bottom border as its down-neighbour.
            PostPassRow(carryY, &out[carryY * OutWidth], carryColor, carryDepthId,
                        carryUpColor, carryUpDepthId, Cfg[GeomRead].BorderColor, Cfg[GeomRead].BorderDepthId);
            PublishRow(rdy, carryY);
            WakeConsumer();
        }
        else
        {
            // Inter-worker seam (idx | idx+1): finish it as soon as worker idx+1 has published its
            // range-first rows -- long before the full-frame barrier -- so the in-order consumer that
            // has just pipelined this range never stalls at the seam row. carryY == this range's last
            // row (Rhi-1); carry* holds its raster + up-neighbour; Seam[idx+1] holds the next range's
            // first 2 raster rows.
            while (SeamFirstReady[idx + 1].load(std::memory_order_acquire) == 0)
                std::this_thread::yield();

            const s32 rowA = carryY;        // this range's last row
            const s32 rowB = carryY + 1;    // next range's first row
            PostPassRow(rowA, &out[rowA * OutWidth], carryColor, carryDepthId,
                        carryUpColor, carryUpDepthId, SeamColor[idx+1][0], SeamDepthId[idx+1][0]);
            PublishRow(rdy, rowA);
            PostPassRow(rowB, &out[rowB * OutWidth], SeamColor[idx+1][0], SeamDepthId[idx+1][0],
                        carryColor, carryDepthId, SeamColor[idx+1][1], SeamDepthId[idx+1][1]);
            PublishRow(rdy, rowB);
            WakeConsumer();
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Band-worker pool (DraStic gpu3d_render_worker model; reference SoftRenderer3D::EnsureBandPool)
// ---------------------------------------------------------------------------------------------

void TileRenderer3D::EnsureBandPool()
{
    if (PoolRunning.load(std::memory_order_relaxed)) return;

    Sema_FrameDone = Platform::Semaphore_Create();
    Sema_RowPub    = Platform::Semaphore_Create();
    for (int b = 0; b < NB; b++)
        BandStartSema[b] = Platform::Semaphore_Create();

    FrameComplete.store(true, std::memory_order_relaxed);
    PoolRunning.store(true, std::memory_order_relaxed);
    for (int b = 0; b < NB; b++)
        BandThreads[b] = Platform::Thread_Create([this, b]() { BandWorkerFunc(b); });
#ifdef LITEV_TILE_COORD
    Sema_CoordWork = Platform::Semaphore_Create();
    Sema_RingFree  = Platform::Semaphore_Create();
    FrameProduced.store(0, std::memory_order_relaxed);
    FrameConsumed.store(0, std::memory_order_relaxed);
    CoordRunning.store(true, std::memory_order_relaxed);
    CoordThread = Platform::Thread_Create([this]() { CoordFunc(); });
#endif
}

void TileRenderer3D::ShutdownBandPool()
{
    if (!PoolRunning.load(std::memory_order_relaxed)) return;

#ifdef LITEV_TILE_COORD
    // Drain every queued frame, then stop the coordinator before the workers it dispatches.
    while (FrameProduced.load(std::memory_order_acquire) != FrameConsumed.load(std::memory_order_acquire))
        Platform::Semaphore_Wait(Sema_RingFree);
    CoordRunning.store(false, std::memory_order_release);
    Platform::Semaphore_Post(Sema_CoordWork);          // wake the coordinator so it observes the flag
    Platform::Thread_Wait(CoordThread);
    Platform::Thread_Free(CoordThread);
    CoordThread = nullptr;
    Platform::Semaphore_Free(Sema_CoordWork); Sema_CoordWork = nullptr;
    Platform::Semaphore_Free(Sema_RingFree);  Sema_RingFree  = nullptr;
#endif
    WaitFrameComplete();                              // never tear down mid-frame
    PoolRunning.store(false, std::memory_order_relaxed);
    for (int b = 0; b < NB; b++)                      // wake each worker so it observes the flag + returns
        Platform::Semaphore_Post(BandStartSema[b]);
    for (int b = 0; b < NB; b++)
    {
        Platform::Thread_Wait(BandThreads[b]);
        Platform::Thread_Free(BandThreads[b]);
        BandThreads[b] = nullptr;
    }
    for (int b = 0; b < NB; b++)
    {
        Platform::Semaphore_Free(BandStartSema[b]);
        BandStartSema[b] = nullptr;
    }
    Platform::Semaphore_Free(Sema_FrameDone);
    Sema_FrameDone = nullptr;
    Platform::Semaphore_Free(Sema_RowPub);
    Sema_RowPub = nullptr;
}

void TileRenderer3D::BandWorkerFunc(int idx)
{
    litevPinTileWorker(idx);
#if defined(__ANDROID__) || defined(__linux__)
    // Name the raster workers so the bench harness (race-tune.sh, root) can renice them.
    // These 3 workers + the 2D compositor oversubscribe cores 0-2; at equal priority a
    // worker can wait behind the 2D thread for a full scheduling slice on kick (measured
    // wake-latency spikes → the emu's RenderFrame WaitFrameComplete stall). A negative nice
    // lets a kicked worker preempt immediately, cutting the spike. Pure scheduling — the
    // raster OUTPUT is identical regardless of when the workers run.
    { char nm[16]; snprintf(nm, sizeof nm, "s3d-tile%d", idx); prctl(PR_SET_NAME, nm, 0, 0, 0); }
#endif
    for (;;)
    {
        Platform::Semaphore_Wait(BandStartSema[idx]);       // acquire: sees this frame's published args
        if (!PoolRunning.load(std::memory_order_relaxed))
            return;

        {
            S3DCpuScope cs(s3dWorkNs, s3dWorkIns);
            WorkerRender(idx);
        }

        // Barrier-free frame completion: each worker already finished its own (idx|idx+1) seam early
        // (in WorkerRender), so the LAST worker to arrive (acq_rel fetch_sub reads 1) only publishes
        // the consume buffer + opens the completion gate. Its acquire also synchronizes-with every
        // other worker's writes (all rows composited before their fetch_subs).
        if (BandsRemaining.fetch_sub(1, std::memory_order_acq_rel) == 1)
        {
            FrameComplete.store(true, std::memory_order_release);
            Platform::Semaphore_Post(Sema_FrameDone);
            // Row-wait flush: a consumer that checked FrameComplete==false and is about to block on
            // Sema_RowPub (the check-then-wait race) must still wake -- flood the row semaphore so no
            // waiter can sleep past the end of the frame. Residue is cleared next RenderFrame.
            Platform::Semaphore_Post(Sema_RowPub, OutHeight);
        }
    }
}

// Block until the in-flight frame's workers have finished (idempotent; called from GetLine 192x,
// FinishRendering, RenderFrame, Reset -- possibly cross-thread). Fast path is a lock-free flag load;
// the gate re-post keeps Sema_FrameDone signalled for every other waiter (reset next frame).
void TileRenderer3D::WaitFrameComplete()
{
    if (FrameComplete.load(std::memory_order_acquire)) return;
    Platform::Semaphore_Wait(Sema_FrameDone);
    Platform::Semaphore_Post(Sema_FrameDone);
}

// ---------------------------------------------------------------------------------------------
// Renderer3D contract
// ---------------------------------------------------------------------------------------------

void TileRenderer3D::RenderFrame()
{
    EnsureBandPool();       // spawn the 3 pinned workers once (first frame)
#ifndef LITEV_TILE_COORD
    WaitFrameComplete();    // the previous frame's workers must be done before we touch shared state
#endif

    // VRAM-coherent bookkeeping -- mirrors SoftRenderer3D::RenderFrame (emu thread).
    auto textureDirty = GPU.VRAMDirty_Texture.DeriveState(GPU.VRAMMap_Texture, GPU);
    auto texPalDirty  = GPU.VRAMDirty_TexPal.DeriveState(GPU.VRAMMap_TexPal, GPU);
#ifdef LITEV_TILE_COORD
    // The coordinator may still be decoding the previous frame's textures while
    // this emu thread reaches the next VCount-215.  Updating VRAMFlat_Texture or
    // clearing the per-band decode caches in that window races the worker and can
    // make host-speed changes alter only the local framebuffer. When texture input
    // will change, drain the coordinator before mutating either shared input.
    // Default ON: the 2,000-frame trace and settled-PPM gate are byte-exact; the
    // property remains available for a same-binary diagnostic A/B.
    const bool textureInputDirty = textureDirty.Begin() != textureDirty.End()
                                || texPalDirty.Begin() != texPalDirty.End();
    if (textureInputDirty && litevTileProp("debug.litev.texbarrier", 1))
    {
        const double texBarrierStart = LSP_NOW();
        while (FrameProduced.load(std::memory_order_acquire) != FrameConsumed.load(std::memory_order_acquire))
            Platform::Semaphore_Wait(Sema_RingFree);
        LSP_ADD(EmuTexBarrier, LSP_NOW() - texBarrierStart);
    }
#endif
    bool textureChanged = GPU.MakeVRAMFlat_TextureCoherent(textureDirty);
    bool texPalChanged  = GPU.MakeVRAMFlat_TexPalCoherent(texPalDirty);
    FrameIdentical = !(textureChanged || texPalChanged) && GPU3D.RenderFrameIdentical;

    // Decode-once caches are per-worker; reset them all here (workers idle) when VRAM changed.
    TexCacheDirty = textureChanged || texPalChanged;
    if (TexCacheDirty)
        for (int b = 0; b < NB; b++) { Band[b].Tex.Used = 0; Band[b].Tex.Count = 0; }

    BuildFrameGeom(GeomWrite);   // compact vertex snapshot + per-block buckets (emu thread)
    ComputeClearConfig();   // clear vals + DoPost/NeedHalo + border (emu thread)
#ifdef LITEV_TILE_SAMEFRAME
    Cfg[GeomWrite].TexDirty = TexCacheDirty;
#endif
#ifdef TILE_TEXSH
    if (TexCacheDirty) TexGen++;
    Cfg[GeomWrite].TexGen = TexGen;
#endif

#ifdef LITEV_TILE_COORD
    // DraStic decouple: the emu HANDS OFF the built frame to the coordinator thread and returns
    // WITHOUT waiting on the raster. Bounded to <= NCOL-1 frames in flight so a free color buffer
    // always exists (rare wait on Sema_RingFree only when the raster falls a full frame behind).
    const int col = (RenderIdx + 1) % NCOL;
    while ((FrameProduced.load(std::memory_order_acquire)
          - FrameConsumed.load(std::memory_order_acquire)) >= (u32)(NCOL - 1))
        Platform::Semaphore_Wait(Sema_RingFree);
    const u32 slot = FrameProduced.load(std::memory_order_relaxed);
    PendRing[slot % NCOL] = { GeomWrite, col };
    // Not-ready from the moment it is PRODUCED (not when the coordinator later dispatches it):
    // a 2D pass latched to this frame must wait for its rows even before the dispatch, never
    // read the buffer's previous (3-frames-old) contents. Its previous frame's consumer is done
    // (the 2D barrier drains kick N-2 before VCount 215 of N reuses the buffer).
    for (int y = 0; y < OutHeight; y++) RowReady[col][y].store(0, std::memory_order_relaxed);
    RenderIdx = col;
    FrameProduced.fetch_add(1, std::memory_order_release);
    Platform::Semaphore_Post(Sema_CoordWork);        // wake the coordinator to raster this frame
    GeomWrite = (GeomWrite + 1) % NGEOM;             // next frame builds the other geometry buffer
#else
    GeomRead = GeomWrite;
    RenderIdx ^= 1;         // workers write the buffer NOT currently being consumed

    // Per-row consumer readiness: clear ONLY the new frame's row-flag bank (RowReady[RenderIdx]) --
    // the other bank belongs to the PREVIOUS frame, which a mid-pass 2D consumer (kicked at VBlank,
    // still running at this VCount-215 kick) has latched; wiping it was the measured ~11ms-per-frame
    // stall. Drain the row semaphore's stale counts, then publish DisplayIdx = the new buffer.
    for (int y = 0; y < OutHeight; y++) RowReady[RenderIdx][y].store(0, std::memory_order_relaxed);
    for (int b = 0; b < NB; b++) SeamFirstReady[b].store(0, std::memory_order_relaxed);
    Platform::Semaphore_Reset(Sema_RowPub);

    // Reset frame-completion state BEFORE publishing DisplayIdx (see CoordDispatch): otherwise a 2D
    // consumer can latch the fresh buffer while FrameComplete is still true from the previous frame,
    // escape its RowReady wait, and read not-yet-rasterized rows = black-bar flicker.
    Platform::Semaphore_Reset(Sema_FrameDone);
    BandsRemaining.store(NB, std::memory_order_relaxed);
    FrameComplete.store(false, std::memory_order_relaxed);
    DisplayIdx.store(RenderIdx, std::memory_order_release);   // publish LAST
    for (int b = 0; b < NB; b++)
        Platform::Semaphore_Post(BandStartSema[b]);

    GeomWrite = (GeomWrite + 1) % NGEOM;   // next frame builds the other geometry buffer
#endif
}

#ifdef LITEV_TILE_COORD
// The coordinator publishes one queued frame to the 2D consumer, dispatches the 3 band workers,
// and blocks (on THIS thread, not the emu) until they finish. Same sequence the single-frame
// RenderFrame did — just owned by the coordinator so the emu never waits on the raster.
#ifdef TILE_TEXSH
// Decode one texture into e (exact: the per-texel DecodeTexel, expanded as ShadeTexel expands it)
// and record the VRAM byte ranges the decode read (a superset is fine: it only costs a re-decode).
void TileRenderer3D::DecodeTexShared(TexShEntry& e, u32 texparam, u32 texpal) const
{
    const s32 W = 8 << ((texparam >> 20) & 0x7), H = 8 << ((texparam >> 23) & 0x7);
    const u32 fmt = (texparam >> 26) & 0x7, addr = (texparam & 0xFFFF) << 3, n = (u32)W * (u32)H;
    e.texels.resize(n);
    u32* out = e.texels.data();
    for (s32 t = 0; t < H; t++)
        for (s32 x = 0; x < W; x++)
        {
            u16 c; u8 a;
            DecodeTexel(texparam, texpal, x, t, &c, &a);
            u32 r = (c << 1) & 0x3E; if (r) r++;
            u32 g = (c >> 4) & 0x3E; if (g) g++;
            u32 b = (c >> 9) & 0x3E; if (b) b++;
            *out++ = r | (g << 8) | (b << 16) | ((u32)a << 24);
        }
    e.ranges.clear();
    auto tex = [&](u32 a, u32 len) { e.ranges.push_back({a, len, false}); };
    auto pal = [&](u32 a, u32 len) { e.ranges.push_back({a, len, true}); };
    switch (fmt)
    {
    case 1: tex(addr, n);     pal(texpal << 4, 64);  break;   // A3I5
    case 2: tex(addr, n / 4); pal(texpal << 3, 8);   break;   // 4-colour
    case 3: tex(addr, n / 2); pal(texpal << 4, 32);  break;   // 16-colour
    case 4: tex(addr, n);     pal(texpal << 4, 512); break;   // 256-colour
    case 6: tex(addr, n);     pal(texpal << 4, 16);  break;   // A5I3
    case 7: tex(addr, n * 2); break;                          // direct
    case 5:   // 4x4: the bounds of every texel / slot-1 / palette address DecodeTexel reads
    {
        u32 vlo = ~0u, vhi = 0, slo = ~0u, shi = 0, plo = ~0u, phi = 0;
        for (s32 t = 0; t < H; t += 1)
            for (s32 x = 0; x < W; x += 4)   // the 4 texels of a block row share va / slot1
            {
                u32 va = (addr + ((t & 0x3FC) * (W >> 2)) + (x & 0x3FC) + (t & 0x3)) & 0x7FFFF;
                u32 s1 = 0x20000 + ((va & 0x1FFFC) >> 1);
                if (va >= 0x40000) s1 += 0x10000;
                const u32 po = (GPU.ReadVRAMFlat_Texture<u16>(s1) & 0x3FFF) << 2;
                vlo = std::min(vlo, va); vhi = std::max(vhi, va);
                slo = std::min(slo, s1); shi = std::max(shi, s1);
                plo = std::min(plo, po); phi = std::max(phi, po);
            }
        tex(vlo, vhi - vlo + 1);
        tex(slo, shi - slo + 2);
        pal((texpal << 4) + plo, phi - plo + 8);
        break;
    }
    }
    size_t total = 0;
    for (const TexRange& r : e.ranges) total += r.len;
    e.src.resize(total);
    u8* d = e.src.data();
    for (const TexRange& r : e.ranges)
    {
        texFlatCopy(r.pal ? GPU.VRAMFlat_TexPal : GPU.VRAMFlat_Texture, r.pal ? 0x1FFFF : 0x7FFFF, r.addr, r.len, d);
        d += r.len;
    }
}

// Coordinator, before the workers start: a texel pointer for every textured polygon of frame gi.
void TileRenderer3D::ResolveTexShared(int gi)
{
    TexShDecodes = 0;
    TexShFrame++;
    if (TexShBytes > TexShBudget)   // over budget: drop what the last frame didn't use, then anything older than this frame
        for (int pass = 0; pass < 2 && TexShBytes > TexShBudget; pass++)
            for (auto it = TexSh.begin(); it != TexSh.end();)
                if (it->second.used + (pass == 0 ? 1 : 0) < TexShFrame)
                {
                    TexShBytes -= it->second.texels.size() * 4 + it->second.src.size();
                    it = TexSh.erase(it);
                }
                else ++it;

    const FrameCfg& cfg = Cfg[gi];
    if (!(cfg.DispCnt & 1)) return;   // texturing off: no polygon reads PolyTex
    const TilePoly* polys = FramePolys[gi];
    const u32 n = FrameNumPolys[gi];
    u64 lastKey = ~0ull;
    const u32* lastTex = nullptr;
    for (u32 i = 0; i < n; i++)
    {
        const u32 tp = polys[i].texparam, fmt = (tp >> 26) & 0x7;
        if (!fmt) continue;
        const u64 key = (tp & 0x3FF0FFFF) | ((u64)(fmt == 7 ? 0 : polys[i].texpal) << 32);
        if (key == lastKey) { PolyTex[i] = lastTex; continue; }
        auto [it, fresh] = TexSh.try_emplace(key);
        TexShEntry& e = it->second;
        bool decode = fresh;
        if (!fresh && e.gen != cfg.TexGen)
        {
            const u8* sp = e.src.data();
            for (const TexRange& r : e.ranges)
            {
                if (!texFlatEqual(r.pal ? GPU.VRAMFlat_TexPal : GPU.VRAMFlat_Texture, r.pal ? 0x1FFFF : 0x7FFFF, r.addr, r.len, sp)) { decode = true; break; }
                sp += r.len;
            }
        }
        if (decode)
        {
            TexShBytes -= e.texels.size() * 4 + e.src.size();
            DecodeTexShared(e, tp, polys[i].texpal);
            TexShBytes += e.texels.size() * 4 + e.src.size();
            TexShDecodes++;
            if (s3dCpu) s3dTexDecodes.fetch_add(1, std::memory_order_relaxed);
        }
        else if (texShVerify)
        {
            TexShEntry chk;
            DecodeTexShared(chk, tp, polys[i].texpal);
            if (chk.texels != e.texels)
            {
                fprintf(stderr, "LITEV_TEXSHARED_VERIFY: stale entry texparam %08x pal %04x (gen %u/%u)\n", tp, polys[i].texpal, e.gen, cfg.TexGen);
                abort();
            }
        }
        e.gen = cfg.TexGen;
        e.used = TexShFrame;
        PolyTex[i] = lastTex = e.texels.data();
        lastKey = key;
    }
}
#endif

#ifdef LITEV_TILE_SAMEFRAME
// Does frame gi repeat the last kept frame's inputs? Everything the raster reads: the compact
// polygons and vertices (bucket lists derive from them) and the per-frame render state. Textures:
// TexDirtySinceLast (no texture VRAM rewrite since that frame was rastered).
bool TileRenderer3D::SameAsLast(int gi, u32 nvtx) const
{
    const u32 n = FrameNumPolys[gi];
    if (!LastValid || n != LastNumPolys || nvtx != LastNumVtx) return false;
    const FrameCfg& a = Cfg[gi];
    const FrameCfg& b = *LastCfg;
    if (a.ClearColorVal != b.ClearColorVal || a.ClearDepthIdVal != b.ClearDepthIdVal || a.DispCnt != b.DispCnt
        || a.AlphaRef != b.AlphaRef || a.FogColor != b.FogColor || a.FogOffset != b.FogOffset || a.FogShift != b.FogShift
        || memcmp(a.ToonTable, b.ToonTable, sizeof(a.ToonTable)) || memcmp(a.EdgeTable, b.EdgeTable, sizeof(a.EdgeTable))
        || memcmp(a.FogDensityTable, b.FogDensityTable, sizeof(a.FogDensityTable)))
        return false;
    return !memcmp(FramePolys[gi], LastPolys, n * sizeof(TilePoly))
        && !memcmp(VtxArena[gi], LastVtx, nvtx * sizeof(TileVtx));   // arenas are zeroed: padding compares equal
}

void TileRenderer3D::SaveLast(int gi, u32 nvtx, int col)
{
    if (!LastCfg) { LastCfg = new FrameCfg; LastPolys = new TilePoly[MaxPolys]; LastVtx = new TileVtx[VtxArenaSlots](); }
    *LastCfg = Cfg[gi];
    memcpy(LastPolys, FramePolys[gi], FrameNumPolys[gi] * sizeof(TilePoly));
    memcpy(LastVtx, VtxArena[gi], nvtx * sizeof(TileVtx));
    LastNumPolys = FrameNumPolys[gi];
    LastNumVtx = nvtx;
    LastCol = col;
    LastValid = true;
    TexDirtySinceLast = false;
}
#endif

void TileRenderer3D::CoordDispatch(int geomIdx, int colIdx)
{
#ifdef TILE_TEXSH
    ResolveTexShared(geomIdx);
#endif
#ifdef LITEV_TILE_SAMEFRAME
    const u32 np = FrameNumPolys[geomIdx];
    const u32 nvtx = np ? FramePolys[geomIdx][np - 1].vtxBase + FramePolys[geomIdx][np - 1].numVerts : 0;
#ifdef TILE_TEXSH
    // every texel the frame reads comes from the shared cache: unchanged unless an entry was
    // (re)decoded, and a frame that decodes is rastered, so no decode since the kept frame
    TexDirtySinceLast = TexShDecodes != 0;
#else
    TexDirtySinceLast |= Cfg[geomIdx].TexDirty;
#endif
    if (!TexDirtySinceLast && SameAsLast(geomIdx, nvtx))
    {
        // same inputs -> same picture: copy the kept output, every row ready at once
        memcpy(ColorOut[colIdx], ColorOut[LastCol], sizeof(ColorOut[0]));
        LastCol = colIdx;
        DisplayIdx.store(colIdx, std::memory_order_release);
        for (int y = 0; y < OutHeight; y++) RowReady[colIdx][y].store(1, std::memory_order_release);
        Platform::Semaphore_Post(Sema_RowPub, OutHeight);
        if (s3dCpu) s3dKept.fetch_add(1, std::memory_order_relaxed);
        return;
    }
#endif
    GeomRead = geomIdx;
    WorkerColIdx = colIdx;
    for (int b = 0; b < NB; b++) SeamFirstReady[b].store(0, std::memory_order_relaxed);
    Platform::Semaphore_Reset(Sema_RowPub);
    // Reset frame-completion state BEFORE publishing DisplayIdx. The 2D consumer (GetLine) latches
    // DisplayIdx (acquire) and, on an unready row, ESCAPES its RowReady wait when FrameComplete is
    // set. If DisplayIdx were published first (as it was), a consumer could latch this fresh buffer
    // while FrameComplete was still true from the PREVIOUS frame, escape immediately, and read rows
    // the workers have not rasterized yet -> intermittent black-bar flicker on the top screen. The
    // DisplayIdx release store below pairs with the consumer's acquire load, so FrameComplete=false
    // and the cleared RowReady bank are guaranteed visible before the new index is seen.
    Platform::Semaphore_Reset(Sema_FrameDone);
    BandsRemaining.store(NB, std::memory_order_relaxed);
    FrameComplete.store(false, std::memory_order_relaxed);
    DisplayIdx.store(colIdx, std::memory_order_release);   // publish LAST
    for (int b = 0; b < NB; b++)
        Platform::Semaphore_Post(BandStartSema[b]);
    WaitFrameComplete();                             // block for the workers (coordinator thread)
#ifdef LITEV_TILE_SAMEFRAME
    SaveLast(geomIdx, nvtx, colIdx);
#endif
}

void TileRenderer3D::CoordFunc()
{
    litevPinTileCoord();
    for (;;)
    {
        Platform::Semaphore_Wait(Sema_CoordWork);
        if (!CoordRunning.load(std::memory_order_acquire)) return;
        const u32 c = FrameConsumed.load(std::memory_order_acquire);
        const PendFrame pf = PendRing[c % NCOL];
        {
            S3DCpuScope cs(s3dCoordNs, s3dCoordIns);
            CoordDispatch(pf.geomIdx, pf.colIdx);
        }
        s3dCpuFrame();
        FrameConsumed.fetch_add(1, std::memory_order_release);
        Platform::Semaphore_Post(Sema_RingFree);     // a ring slot freed -> unblock a waiting emu
    }
}
#endif

void TileRenderer3D::FinishRendering()
{
    // NON-BLOCKING (async): the emu must NOT stall at VBlank waiting for the render. Frame N's raster
    // overlaps the emu's frame N+1 compute (~15ms window, easily hides the ~6ms render wall). The only
    // wait, if any, is at the NEXT RenderFrame's start (WaitFrameComplete) -- off the emu critical path
    // (the emu has already done frame N's work by then). This mirrors the reference's async pipeline
    // (its Finish3DRendering barrier is skipped whenever the game has render headroom).
}

void TileRenderer3D::RestartFrame()
{
}

u32* TileRenderer3D::GetLine(int line)
{
    return GetLineAt(line, GPU3D.RenderXPos, GPU3D.AbortFrame);
}

u32* TileRenderer3D::GetLineAt(int line, u16 xpos, bool abort)
{
    if (abort || line < 0 || line >= OutHeight)
    {
        memset(ScrolledLine, 0, sizeof(ScrolledLine));
        return ScrolledLine;
    }

    // Per-row readiness (the reference's Sema_ScanlineCount pattern): on an unready row the
    // consumer blocks on Sema_RowPub (posted per published row) and re-checks, so it sleeps only
    // until THAT row lands, then keeps pipelining behind the workers.
#if defined(LITEV_TILE_COORD) && defined(LITEV_SOFT2D_THREADED)
    // The buffer was latched for this pass at its VBlank kick (LatchConsumer): the frame produced
    // last before the kick, as on hardware. (It used to latch, at line 0 on the 2D thread, the last
    // frame that happened to be COMPLETE -- one or two frames older depending on raster timing, so
    // identical runs displayed/captured different 3D frames.) Its rows were marked not-ready when it
    // was produced, so this waits however far behind the raster is. No hang: a produced frame is
    // always dispatched and publishes all 192 rows; the coordinator never waits on the 2D.
    const int idx = ConsumerPassIdx;
#else
    if (line == 0)
#ifdef LITEV_TILE_COORD
        ConsumerPassIdx = RenderIdx;   // inline consumer = the emu thread: the newest produced frame
#else
        ConsumerPassIdx = DisplayIdx.load(std::memory_order_acquire);
#endif
    const int idx = ConsumerPassIdx;
#endif
    std::atomic<u8>& rdy = RowReady[idx][line];

#if LITEV_PROFILE
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.TileGetLineCalls);
    const bool stalled = (rdy.load(std::memory_order_acquire) == 0);
    const uint64_t _t0 = stalled ? melonDS::LiteProfile::NowNs() : 0;
    if (stalled) LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.TileGetLineStalls);
#endif
    while (rdy.load(std::memory_order_acquire) == 0)
    {
#ifndef LITEV_TILE_COORD
        // stale latch (boot: a pass can start before the first 3D kick): no more rows are coming
        if (DisplayIdx.load(std::memory_order_acquire) != idx) break;
        if (FrameComplete.load(std::memory_order_acquire)) break;
#endif
        Platform::Semaphore_Wait(Sema_RowPub);
    }
#if LITEV_PROFILE
    if (stalled)
        LITE_PROFILE_ADD_VALUE(melonDS::LiteProfile::g_Frame.TileGetLineStallNs,
                               melonDS::LiteProfile::NowNs() - _t0);
#endif

    u32* plane   = ColorOut[idx];
    u32* rawline = &plane[(u32)line * OutWidth];

    // 3D-layer X scroll (RenderXPos) -- byte-for-byte the SoftRenderer3D::GetLine logic.
    if (xpos == 0)
        return rawline;

    if (xpos & 0x100)
    {
        int i = 0, j = xpos;
        for (; j < 512; i++, j++) ScrolledLine[i] = 0;
        for (j = 0; i < 256; i++, j++) ScrolledLine[i] = rawline[j];
    }
    else
    {
        int i = 0, j = xpos;
        for (; j < 256; i++, j++) ScrolledLine[i] = rawline[j];
        for (; i < 256; i++) ScrolledLine[i] = 0;
    }
    return ScrolledLine;
}

}

#endif // LITEV_SOFT3D_DRASTIC
