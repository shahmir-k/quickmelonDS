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

#if defined(LITEV_GEOM_NEON2) && defined(__ARM_NEON)
#include <arm_neon.h>   // BuildFrameGeom vertex-pack (coalesce the per-vertex scatter stores)
#endif

#include "GPU.h"     // GPU: MakeVRAMFlat_*Coherent, VRAMDirty_*/VRAMMap_*
#include "GPU3D.h"   // GPU3D: RenderPolygonRAM, RenderNumPolygons, RenderClearAttr*, RenderXPos, ...
#include "LiteProfile.h"   // async attribution counters (compiles to nothing unless LITEV_PROFILE=1)
#include "LitevSoftProf.h" // tile-coordinator ownership-barrier attribution

#if defined(__ANDROID__) && defined(LITEV_PIN_RENDER)
#include <sched.h>
// Pin each tile band-worker to its OWN core among {0,1,2}, off the emu's core (3). The shared
// {0,1,2} mask let the scheduler double two workers onto one core and leave another idle (measured:
// all 3 rarely on distinct cores), so the 3-way raster ran ~2-wide and inflated the render gate the
// emu waits on. A distinct core per worker guarantees the parallel spread.
static void litevPinTileWorker(int idx)
{
    cpu_set_t set; CPU_ZERO(&set);
    CPU_SET(idx % 3, &set);
    sched_setaffinity(0, sizeof(set), &set);
}
#else
static void litevPinTileWorker(int) {}
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

namespace melonDS
{

TileRenderer3D::TileRenderer3D(melonDS::GPU3D& gpu3D, SoftRenderer& parent) noexcept
    : Renderer3D(gpu3D), Parent(parent)
{
}

TileRenderer3D::~TileRenderer3D()
{
    ShutdownBandPool();                 // join workers before freeing anything they touch
    for (int b = 0; b < NB; b++)
        delete[] Band[b].Tex.Arena;
    for (int g = 0; g < NGEOM; g++) delete[] VtxArena[g];
}

bool TileRenderer3D::Init()
{
    for (int g = 0; g < NGEOM; g++) if (!VtxArena[g]) VtxArena[g] = new TileVtx[VtxArenaSlots];
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
    for (int g = 0; g < NGEOM; g++) if (!VtxArena[g]) VtxArena[g] = new TileVtx[VtxArenaSlots];
    memset(ColorOut, 0, sizeof(ColorOut));
    for (int b = 0; b < NB; b++)
    {
        memset(Band[b].Color,   0, sizeof(Band[b].Color));
        memset(Band[b].DepthId, 0, sizeof(Band[b].DepthId));
        Band[b].Tex.Used = 0;
        Band[b].Tex.Count = 0;
    }
    ConsumeIdx.store(0, std::memory_order_relaxed);
    DisplayIdx.store(0, std::memory_order_relaxed);
    for (int b = 0; b < NCOL; b++)
        for (int y = 0; y < OutHeight; y++) RowReady[b][y].store(0, std::memory_order_relaxed);
    ConsumerPassIdx = 0;
    RenderIdx = 0;
    FrameIdentical = false;
    TexCacheDirty = true;
    GeomWrite = 0; GeomRead = 0;
    for (int g = 0; g < NGEOM; g++) FrameNumPolys[g] = 0;
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

    if (GPU3D.RenderDispCnt & (1<<3))
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


    for (u32 i = 0; i < npolys; i++)
    {
        Polygon* poly = polySrc[i];


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
    bs.Color[idx] = AlphaBlend(color, bs.Color[idx], color >> 24);
    if (z != -1)
        bs.DepthId[idx] = ((u32)z & 0x00FFFFFF) | idword;   // idword carries ID_TRANSL
}

u32 TileRenderer3D::CalculateFogDensity(u32 z) const
{
    u32 densityid, densityfrac;
    if (z < GPU3D.RenderFogOffset)
    {
        densityid = 0;
        densityfrac = 0;
    }
    else
    {
        z -= GPU3D.RenderFogOffset;
        z = (z >> 2) << GPU3D.RenderFogShift;
        densityid = z >> 17;
        if (densityid >= 32) { densityid = 32; densityfrac = 0; }
        else                   densityfrac = z & 0x1FFFF;
    }

    u32 density =
        ((GPU3D.RenderFogDensityTable[densityid] * (0x20000-densityfrac)) +
         (GPU3D.RenderFogDensityTable[densityid+1] * densityfrac)) >> 17;
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
    const u32 dispcnt = GPU3D.RenderDispCnt;
    const bool edgemark = (dispcnt & (1<<5)) != 0;
    const bool aa       = (dispcnt & (1<<4)) != 0;
    const bool fog      = (dispcnt & (1<<7)) != 0;
    const bool detect   = edgemark || aa;

    // fog constants (hoisted)
    const bool fogcolor = !(dispcnt & (1<<6));
    u32 fogR = (GPU3D.RenderFogColor << 1) & 0x3E; if (fogR) fogR++;
    u32 fogG = (GPU3D.RenderFogColor >> 4) & 0x3E; if (fogG) fogG++;
    u32 fogB = (GPU3D.RenderFogColor >> 9) & 0x3E; if (fogB) fogB++;
    u32 fogA = (GPU3D.RenderFogColor >> 16) & 0x1F;

    const u32 borderDI = Cfg[GeomRead].ClearDepthIdVal;
    const u32 borderC  = Cfg[GeomRead].ClearColorVal;

    for (int x = 0; x < OutWidth; x++)
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
                    u16 ec = GPU3D.RenderEdgeTable[cid >> 3];
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
}

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
        const u32 dispcnt   = GPU3D.RenderDispCnt;
        ps.toon      = (blendmode == 2);
        ps.highlight = ps.toon && (dispcnt & (1<<1));
        ps.decal     = (blendmode & 0x1) != 0;
        ps.texparam  = poly.texparam;
        const u32 texfmt = (ps.texparam >> 26) & 0x7;
        ps.texEnable = (dispcnt & 0x1) && (texfmt != 0);
        ps.texpal    = poly.texpal;
        ps.toontbl   = GPU3D.RenderToonTable;
        ps.polyalpha = (attr >> 16) & 0x1F;
        ps.wireframe = (ps.polyalpha == 0);
        ps.alpharef  = GPU3D.RenderAlphaRef;
        // DraStic id byte pre-shifted into bits 24-31: poly-id (opaque, 6 bits) | fog-enable (bit 6).
        const u32 polyid = (attr >> 24) & ID_POLY_MASK;
        const u32 fogbit = (attr & (1u<<15)) ? ID_FOG : 0;
        ps.idword    = ((polyid | fogbit) << 24);
        ps.translDepthWrite = (attr & (1u<<11)) != 0;
        ps.texcache  = nullptr;
        ps.texW = 0; ps.texH = 0;
        if (ps.texEnable)
            ps.texcache = ResolveTexCache(bs.Tex, ps.texparam, ps.texpal, &ps.texW, &ps.texH);
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

        WorkerRender(idx);

        // Barrier-free frame completion: each worker already finished its own (idx|idx+1) seam early
        // (in WorkerRender), so the LAST worker to arrive (acq_rel fetch_sub reads 1) only publishes
        // the consume buffer + opens the completion gate. Its acquire also synchronizes-with every
        // other worker's writes (all rows composited before their fetch_subs).
        if (BandsRemaining.fetch_sub(1, std::memory_order_acq_rel) == 1)
        {
#ifdef LITEV_TILE_COORD
            // The COORD workers write WorkerColIdx (set at dispatch); RenderIdx is the emu's newest,
            // possibly-still-queued buffer. Publish the buffer THESE workers just completed so the
            // consumer latches a fully-rasterized frame.
            ConsumeIdx.store(WorkerColIdx, std::memory_order_release);
#else
            ConsumeIdx.store(RenderIdx, std::memory_order_release);
#endif
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
void TileRenderer3D::CoordDispatch(int geomIdx, int colIdx)
{
    GeomRead = geomIdx;
    WorkerColIdx = colIdx;
    for (int y = 0; y < OutHeight; y++) RowReady[colIdx][y].store(0, std::memory_order_relaxed);
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
}

void TileRenderer3D::CoordFunc()
{
    for (;;)
    {
        Platform::Semaphore_Wait(Sema_CoordWork);
        if (!CoordRunning.load(std::memory_order_acquire)) return;
        const u32 c = FrameConsumed.load(std::memory_order_acquire);
        const PendFrame pf = PendRing[c % NCOL];
        CoordDispatch(pf.geomIdx, pf.colIdx);
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
    if (GPU3D.AbortFrame || line < 0 || line >= OutHeight)
    {
        memset(ScrolledLine, 0, sizeof(ScrolledLine));
        return ScrolledLine;
    }

    // Per-row readiness (the reference's Sema_ScanlineCount pattern). The consumer LATCHES its
    // buffer once per pass (line 0): the next RenderFrame (VCount 215) fires mid-pass and flips
    // DisplayIdx / resets the OTHER bank's flags, but this pass's bank + flags stay intact -- that
    // mid-pass clobber was the measured once-per-frame ~11ms stall. On an unready row the consumer
    // blocks on Sema_RowPub (posted per published row) and re-checks: it sleeps only until THAT row
    // lands (~0.1-1ms), not until frame end, then keeps pipelining behind the workers.
    // No hang: (a) an in-flight frame always publishes all 192 rows (each post wakes the waiter);
    // (b) the frame-end flush posts 192 extra so a waiter racing frame-completion still wakes;
    // (c) DisplayIdx-changed / FrameComplete escapes cover a stale latch (boot: a consumer pass can
    // start before the first 3D kick) -- both mean no more rows are coming for this bank.
    if (line == 0)
#ifdef LITEV_TILE_COORD
        // Latch the last FULLY-COMPLETE 3D frame, not the in-progress buffer (DisplayIdx). The emu is
        // already decoupled by the coordinator, so compositing a complete frame costs no emu fps and
        // eliminates the read-ahead black-bar race: every row of ConsumeIdx is already published and
        // its buffer is not reused for another NCOL-1 frames (~2 frames of margin vs one ~16ms pass).
        // Boot: ConsumeIdx=0 is a black-cleared buffer until the first frame completes.
        ConsumerPassIdx = ConsumeIdx.load(std::memory_order_acquire);
#else
        ConsumerPassIdx = DisplayIdx.load(std::memory_order_acquire);
#endif
    const int idx = ConsumerPassIdx;
    std::atomic<u8>& rdy = RowReady[idx][line];

#if LITEV_PROFILE
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.TileGetLineCalls);
    const bool stalled = (rdy.load(std::memory_order_acquire) == 0);
    const uint64_t _t0 = stalled ? melonDS::LiteProfile::NowNs() : 0;
    if (stalled) LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.TileGetLineStalls);
#endif
    while (rdy.load(std::memory_order_acquire) == 0)
    {
        if (DisplayIdx.load(std::memory_order_acquire) != idx) break;  // latched frame superseded/stale
        if (FrameComplete.load(std::memory_order_acquire)) break;      // frame done -> flags published
        Platform::Semaphore_Wait(Sema_RowPub);                         // real block until the next row
    }
#if LITEV_PROFILE
    if (stalled)
        LITE_PROFILE_ADD_VALUE(melonDS::LiteProfile::g_Frame.TileGetLineStallNs,
                               melonDS::LiteProfile::NowNs() - _t0);
#endif

    u32* plane   = ColorOut[idx];
    u32* rawline = &plane[(u32)line * OutWidth];

    // 3D-layer X scroll (RenderXPos) -- byte-for-byte the SoftRenderer3D::GetLine logic.
    u16 xpos = GPU3D.RenderXPos;
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
