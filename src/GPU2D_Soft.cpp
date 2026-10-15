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

#include "GPU_Soft.h"
#include "GPU_ColorOp.h"

#if defined(LITEV_SOFT2D_NEON) && defined(__aarch64__)
#include "GPU2D_NEON.h"
#endif

#if defined(LITEV_SOFT2D_OBJNEON) && defined(__aarch64__)
#include <arm_neon.h>
#endif

#if defined(S2D_PLANES) || defined(S2D_OBJROW)
#include "GPU2D_Planes.h"
#endif

namespace melonDS
{

SoftRenderer2D::SoftRenderer2D(melonDS::GPU2D& gpu2D, SoftRenderer& parent)
    : Renderer2D(gpu2D), Parent(parent)
{
    // mosaic table is initialized at compile-time
    // Default the render-time palette base to the live palette; the async render
    // path overrides this per-frame with a snapshot (PaletteSnap).
    CurPalette = GPU.Palette;
}

#ifdef LITEV_SOFT2D_THREADED
#if defined(LITEV_SOFT2D_DEPTH2)
void SoftRenderer2D::CopyLineSnaps(int slot)
{
    memcpy(LineSnapR[slot], LineSnap, sizeof(LineSnap));
    memcpy(SprSnapR[slot],  SprSnap,  sizeof(SprSnap));
}
#else
void SoftRenderer2D::CopyLineSnaps()
{
    memcpy(LineSnapR, LineSnap, sizeof(LineSnap));
    memcpy(SprSnapR,  SprSnap,  sizeof(SprSnap));
}
#endif
#endif

SoftRenderer2D::~SoftRenderer2D()
{
}

void SoftRenderer2D::Reset()
{
    memset(BGOBJLine, 0, sizeof(BGOBJLine));
    memset(WindowMask, 0, sizeof(WindowMask));
    memset(OBJLine, 0, sizeof(OBJLine));
    memset(OBJWindow, 0, sizeof(OBJWindow));

    NumSprites = 0;
}

u32 SoftRenderer2D::ColorComposite(int i, u32 val1, u32 val2) const
{
    u32 coloreffect = 0;
    u32 eva, evb;

    u32 flag1 = val1 >> 24;
    u32 flag2 = val2 >> 24;

    u32 blendCnt = GPU2D.BlendCnt;

    u32 target2;
    if      (flag2 & 0x80) target2 = 0x1000;
    else if (flag2 & 0x40) target2 = 0x0100;
    else                   target2 = flag2 << 8;

    if ((flag1 & 0x80) && (blendCnt & target2))
    {
        // sprite blending

        coloreffect = 1;

        if (flag1 & 0x40)
        {
            eva = flag1 & 0x1F;
            evb = 16 - eva;
        }
        else
        {
            eva = GPU2D.EVA;
            evb = GPU2D.EVB;
        }
    }
    else if ((flag1 & 0x40) && (blendCnt & target2))
    {
        // 3D layer blending

        coloreffect = 4;
    }
    else
    {
        if      (flag1 & 0x80) flag1 = 0x10;
        else if (flag1 & 0x40) flag1 = 0x01;

        if ((blendCnt & flag1) && (WindowMask[i] & 0x20))
        {
            coloreffect = (blendCnt >> 6) & 0x3;

            if (coloreffect == 1)
            {
                if (blendCnt & target2)
                {
                    eva = GPU2D.EVA;
                    evb = GPU2D.EVB;
                }
                else
                    coloreffect = 0;
            }
        }
    }

    switch (coloreffect)
    {
        case 0: return val1;
        case 1: return ColorBlend4(val1, val2, eva, evb);
        case 2: return ColorBrightnessUp(val1, GPU2D.EVY, 0x8);
        case 3: return ColorBrightnessDown(val1, GPU2D.EVY, 0x7);
        case 4: return ColorBlend5(val1, val2);
    }

    return val1;
}

void SoftRenderer2D::DrawScanline(u32 line)
{
    u32* dst = Parent.Output2D[GPU2D.Num];

    if (!GPU2D.Enabled)
    {
        // if this 2D unit is disabled in POWCNT, the output is a fixed color
        // (black for unit A, white for unit B)
        u32 fillcolor = (GPU2D.Num == 0) ? 0xFF000000 : 0xFF3F3F3F;
        for (int i = 0; i < 256; i++)
            dst[i] = fillcolor;

        return;
    }

    if (GPU2D.ForcedBlank)
    {
        // forced blank
        for (int i = 0; i < 256; i++)
            dst[i] = 0xFF3F3F3F;

        return;
    }

    if (GPU2D.Num == 0)
    {
        auto bgDirty = GPU.VRAMDirty_ABG.DeriveState(GPU.VRAMMap_ABG, GPU);
        GPU.MakeVRAMFlat_ABGCoherent(bgDirty);
        auto bgExtPalDirty = GPU.VRAMDirty_ABGExtPal.DeriveState(GPU.VRAMMap_ABGExtPal, GPU);
        GPU.MakeVRAMFlat_ABGExtPalCoherent(bgExtPalDirty);
        auto objExtPalDirty = GPU.VRAMDirty_AOBJExtPal.DeriveState(&GPU.VRAMMap_AOBJExtPal, GPU);
        GPU.MakeVRAMFlat_AOBJExtPalCoherent(objExtPalDirty);
    }
    else
    {
        auto bgDirty = GPU.VRAMDirty_BBG.DeriveState(GPU.VRAMMap_BBG, GPU);
        GPU.MakeVRAMFlat_BBGCoherent(bgDirty);
        auto bgExtPalDirty = GPU.VRAMDirty_BBGExtPal.DeriveState(GPU.VRAMMap_BBGExtPal, GPU);
        GPU.MakeVRAMFlat_BBGExtPalCoherent(bgExtPalDirty);
        auto objExtPalDirty = GPU.VRAMDirty_BOBJExtPal.DeriveState(&GPU.VRAMMap_BOBJExtPal, GPU);
        GPU.MakeVRAMFlat_BOBJExtPalCoherent(objExtPalDirty);
    }

    // render BG layers and sprites
    DrawScanline_BGOBJ(line, dst);
}

#ifdef LITEV_SOFT2D_THREADED
// ---- liteV banded deferred software 2D ----
// Capture the per-scanline-mutable register state on the emu thread, at the exact
// LCD scanline moment the inline path would have rendered this line — so raster
// effects (mid-frame scroll/window/blend changes) stay bit-exact when the actual
// raster runs later, off the critical path.
void SoftRenderer2D::SnapshotLineState(u32 line)
{
    S2DLineState& s = LineSnap[line];
    s.Enabled     = GPU2D.Enabled;
    s.ForcedBlank = GPU2D.ForcedBlank;
    s.DispCnt     = GPU2D.DispCnt;
    s.LayerEnable = GPU2D.LayerEnable;
    for (int i = 0; i < 4; i++)
    {
        s.BGCnt[i]  = GPU2D.BGCnt[i];
        s.BGXPos[i] = GPU2D.BGXPos[i];
        s.BGYPos[i] = GPU2D.BGYPos[i];
        s.WinCnt[i]     = GPU2D.WinCnt[i];
        s.Win0Coords[i] = GPU2D.Win0Coords[i];
        s.Win1Coords[i] = GPU2D.Win1Coords[i];
    }
    for (int i = 0; i < 2; i++)
    {
        s.BGXRefInternal[i] = GPU2D.BGXRefInternal[i];
        s.BGYRefInternal[i] = GPU2D.BGYRefInternal[i];
        s.BGRotA[i] = GPU2D.BGRotA[i];
        s.BGRotC[i] = GPU2D.BGRotC[i];
        s.BGMosaicSize[i]  = GPU2D.BGMosaicSize[i];
        s.OBJMosaicSize[i] = GPU2D.OBJMosaicSize[i];
    }
    s.BGMosaicLine = GPU2D.BGMosaicLine;
    s.BlendCnt = GPU2D.BlendCnt;
    s.EVA = GPU2D.EVA; s.EVB = GPU2D.EVB; s.EVY = GPU2D.EVY;
    s.Win0Active = GPU2D.Win0Active;
    s.Win1Active = GPU2D.Win1Active;
}

void SoftRenderer2D::LoadLineState(const S2DLineState& s)
{
    GPU2D.Enabled     = s.Enabled;
    GPU2D.ForcedBlank = s.ForcedBlank;
    GPU2D.DispCnt     = s.DispCnt;
    GPU2D.LayerEnable = s.LayerEnable;
    for (int i = 0; i < 4; i++)
    {
        GPU2D.BGCnt[i]  = s.BGCnt[i];
        GPU2D.BGXPos[i] = s.BGXPos[i];
        GPU2D.BGYPos[i] = s.BGYPos[i];
        GPU2D.WinCnt[i]     = s.WinCnt[i];
        GPU2D.Win0Coords[i] = s.Win0Coords[i];
        GPU2D.Win1Coords[i] = s.Win1Coords[i];
    }
    for (int i = 0; i < 2; i++)
    {
        GPU2D.BGXRefInternal[i] = s.BGXRefInternal[i];
        GPU2D.BGYRefInternal[i] = s.BGYRefInternal[i];
        GPU2D.BGRotA[i] = s.BGRotA[i];
        GPU2D.BGRotC[i] = s.BGRotC[i];
        GPU2D.BGMosaicSize[i]  = s.BGMosaicSize[i];
        GPU2D.OBJMosaicSize[i] = s.OBJMosaicSize[i];
    }
    GPU2D.BGMosaicLine = s.BGMosaicLine;
    GPU2D.BlendCnt = s.BlendCnt;
    GPU2D.EVA = s.EVA; GPU2D.EVB = s.EVB; GPU2D.EVY = s.EVY;
    GPU2D.Win0Active = s.Win0Active;
    GPU2D.Win1Active = s.Win1Active;
}

void SoftRenderer2D::SnapshotSprState(u32 line)
{
    S2DSprState& s = SprSnap[line];
    s.Enabled   = GPU2D.Enabled;
    s.OBJEnable = GPU2D.OBJEnable;
    s.DispCnt   = GPU2D.DispCnt;
    s.OBJMosaicLine = GPU2D.OBJMosaicLine;
    s.OBJMosaicSize[0] = GPU2D.OBJMosaicSize[0];
    s.OBJMosaicSize[1] = GPU2D.OBJMosaicSize[1];
}

void SoftRenderer2D::LoadSprState(const S2DSprState& s)
{
    GPU2D.Enabled   = s.Enabled;
    GPU2D.OBJEnable = s.OBJEnable;
    GPU2D.DispCnt   = s.DispCnt;
    GPU2D.OBJMosaicLine = s.OBJMosaicLine;
    GPU2D.OBJMosaicSize[0] = s.OBJMosaicSize[0];
    GPU2D.OBJMosaicSize[1] = s.OBJMosaicSize[1];
}

// Once-per-frame VRAM coherence (the DeriveState/MakeVRAMFlat the inline path did
// per-scanline). Run at VBlank before the deferred raster.
void SoftRenderer2D::SyncVRAM_BG()
{
    if (GPU2D.Num == 0)
    {
        auto bgDirty = GPU.VRAMDirty_ABG.DeriveState(GPU.VRAMMap_ABG, GPU);
        GPU.MakeVRAMFlat_ABGCoherent(bgDirty);
        auto bgExtPalDirty = GPU.VRAMDirty_ABGExtPal.DeriveState(GPU.VRAMMap_ABGExtPal, GPU);
        GPU.MakeVRAMFlat_ABGExtPalCoherent(bgExtPalDirty);
        auto objExtPalDirty = GPU.VRAMDirty_AOBJExtPal.DeriveState(&GPU.VRAMMap_AOBJExtPal, GPU);
        GPU.MakeVRAMFlat_AOBJExtPalCoherent(objExtPalDirty);
    }
    else
    {
        auto bgDirty = GPU.VRAMDirty_BBG.DeriveState(GPU.VRAMMap_BBG, GPU);
        GPU.MakeVRAMFlat_BBGCoherent(bgDirty);
        auto bgExtPalDirty = GPU.VRAMDirty_BBGExtPal.DeriveState(GPU.VRAMMap_BBGExtPal, GPU);
        GPU.MakeVRAMFlat_BBGExtPalCoherent(bgExtPalDirty);
        auto objExtPalDirty = GPU.VRAMDirty_BOBJExtPal.DeriveState(&GPU.VRAMMap_BOBJExtPal, GPU);
        GPU.MakeVRAMFlat_BOBJExtPalCoherent(objExtPalDirty);
    }
}

void SoftRenderer2D::SyncVRAM_OBJ()
{
    if (GPU2D.Num == 0)
    {
        auto objDirty = GPU.VRAMDirty_AOBJ.DeriveState(GPU.VRAMMap_AOBJ, GPU);
        GPU.MakeVRAMFlat_AOBJCoherent(objDirty);
    }
    else
    {
        auto objDirty = GPU.VRAMDirty_BOBJ.DeriveState(GPU.VRAMMap_BOBJ, GPU);
        GPU.MakeVRAMFlat_BOBJCoherent(objDirty);
    }
}

// Deferred BG+OBJ raster for one scanline, restoring that line's snapshot first.
// (Milestone 1: single-thread batched at VBlank, reusing DrawScanline_BGOBJ.)
void SoftRenderer2D::DrawScanlineDeferred(const S2DLineState& s, u32 line, u32* dst)
{
    LastLineHas3D = false;
    if (!s.Enabled)
    {
        u32 fillcolor = (GPU2D.Num == 0) ? 0xFF000000 : 0xFF3F3F3F;
        for (int i = 0; i < 256; i++) dst[i] = fillcolor;
        return;
    }
    if (s.ForcedBlank)
    {
        for (int i = 0; i < 256; i++) dst[i] = 0xFF3F3F3F;
        return;
    }

    LoadLineState(s);
    DrawScanline_BGOBJ(line, dst);
}

void SoftRenderer2D::DrawSpritesDeferred(const S2DSprState& s, u32 line)
{
    LoadSprState(s);
    DrawSprites(line);
}

u64 SoftRenderer2D::PipeTraceSpriteHash() const
{
    u64 h = 1469598103934665603ULL;
    const u8* p = (const u8*)OBJLine;
    for (size_t i = 0; i < sizeof(OBJLine); i++) { h ^= p[i]; h *= 1099511628211ULL; }
    p = WindowMask;
    for (size_t i = 0; i < sizeof(WindowMask); i++) { h ^= p[i]; h *= 1099511628211ULL; }
    h ^= NumSprites;
    return h * 1099511628211ULL;
}
#endif // LITEV_SOFT2D_THREADED

#define DoDrawBG(type, line, num) \
    do \
    { \
        if ((bgCnt[num] & (1<<6)) && (GPU2D.BGMosaicSize[0] > 0)) \
        { \
            DrawBG_##type<true>(line, num); \
        } \
        else \
        { \
            DrawBG_##type<false>(line, num); \
        } \
    } while (false)

#define DoDrawBG_Large(line) \
    do \
    { \
        if ((bgCnt[2] & (1<<6)) && (GPU2D.BGMosaicSize[0] > 0)) \
        { \
            DrawBG_Large<true>(line); \
        } \
        else \
        { \
            DrawBG_Large<false>(line); \
        } \
    } while (false)

#ifdef LITEV_SOFT2D_SPRLIST
#define S2D_OBJPRIO(i) && ((OBJPrioMask >> (i)) & 1)   // some non-window sprite of priority i drawn
#else
#define S2D_OBJPRIO(i)
#endif
template<u32 bgmode>
void SoftRenderer2D::DrawScanlineBGMode(u32 line)
{
    u32 dispCnt = GPU2D.DispCnt;
    u16* bgCnt = GPU2D.BGCnt;
    for (int i = 3; i >= 0; i--)
    {
        if ((bgCnt[3] & 0x3) == i)
        {
            if (GPU2D.LayerEnable & (1<<3))
            {
                if (bgmode >= 3)
                    DoDrawBG(Extended, line, 3);
                else if (bgmode >= 1)
                    DoDrawBG(Affine, line, 3);
                else
                    DoDrawBG(Text, line, 3);
            }
        }
        if ((bgCnt[2] & 0x3) == i)
        {
            if (GPU2D.LayerEnable & (1<<2))
            {
                if (bgmode == 5)
                    DoDrawBG(Extended, line, 2);
                else if (bgmode == 4 || bgmode == 2)
                    DoDrawBG(Affine, line, 2);
                else
                    DoDrawBG(Text, line, 2);
            }
        }
        if ((bgCnt[1] & 0x3) == i)
        {
            if (GPU2D.LayerEnable & (1<<1))
            {
                DoDrawBG(Text, line, 1);
            }
        }
        if ((bgCnt[0] & 0x3) == i)
        {
            if (GPU2D.LayerEnable & (1<<0))
            {
                if (!GPU2D.Num && (dispCnt & 0x8))
                    DrawBG_3D();
                else
                    DoDrawBG(Text, line, 0);
            }
        }
        if ((GPU2D.LayerEnable & (1<<4)) && NumSprites S2D_OBJPRIO(i))
        {
            InterleaveSprites(i);
        }

    }
}

void SoftRenderer2D::DrawScanlineBGMode6(u32 line)
{
    u32 dispCnt = GPU2D.DispCnt;
    u16* bgCnt = GPU2D.BGCnt;
    for (int i = 3; i >= 0; i--)
    {
        if ((bgCnt[2] & 0x3) == i)
        {
            if (GPU2D.LayerEnable & (1<<2))
            {
                DoDrawBG_Large(line);
            }
        }
        if ((bgCnt[0] & 0x3) == i)
        {
            if (GPU2D.LayerEnable & (1<<0))
            {
                if ((!GPU2D.Num) && (dispCnt & 0x8))
                    DrawBG_3D();
            }
        }
        if ((GPU2D.LayerEnable & (1<<4)) && NumSprites S2D_OBJPRIO(i))
        {
            InterleaveSprites(i);
        }
    }
}

void SoftRenderer2D::DrawScanlineBGMode7(u32 line)
{
    u32 dispCnt = GPU2D.DispCnt;
    u16* bgCnt = GPU2D.BGCnt;
    // mode 7 only has text-mode BG0 and BG1

    for (int i = 3; i >= 0; i--)
    {
        if ((bgCnt[1] & 0x3) == i)
        {
            if (GPU2D.LayerEnable & (1<<1))
            {
                DoDrawBG(Text, line, 1);
            }
        }
        if ((bgCnt[0] & 0x3) == i)
        {
            if (GPU2D.LayerEnable & (1<<0))
            {
                if (!GPU2D.Num && (dispCnt & 0x8))
                    DrawBG_3D();
                else
                    DoDrawBG(Text, line, 0);
            }
        }
        if ((GPU2D.LayerEnable & (1<<4)) && NumSprites S2D_OBJPRIO(i))
        {
            InterleaveSprites(i);
        }
    }
}

void SoftRenderer2D::DrawScanline_BGOBJ(u32 line, u32* dst)
{
#ifdef S2D_PLANES
    // the hybrid's 3D descriptor lines keep the layer-push path (Under3D bookkeeping), unless
    // LITEV_HYB_PLANES (the select keeps it)
#ifdef LITEV_HYB_PLANES
    if (CurPalConv && (HybPlanes || !(HybridDesc && !GPU2D.Num && (GPU2D.DispCnt & 0x8))))
#else
    if (CurPalConv && !(HybridDesc && !GPU2D.Num && (GPU2D.DispCnt & 0x8)))
#endif
    {
        DrawScanlinePlanes(line, dst);
        return;
    }
#endif
    u64 backdrop;
    if (GPU2D.Num)
        backdrop = *(u16*)&CurPalette[0x400];
    else
        backdrop = *(u16*)&CurPalette[0];

    {
        u8 r = (backdrop & 0x001F) << 1;
        u8 g = ((backdrop & 0x03E0) >> 4) | ((backdrop & 0x8000) >> 15);
        u8 b = (backdrop & 0x7C00) >> 9;

        backdrop = r | (g << 8) | (b << 16) | 0x20000000;
        backdrop |= (backdrop << 32);

        for (int i = 0; i < 256; i+=2)
            *(u64*)&BGOBJLine[i] = backdrop;
        for (int i = 256; i < 512; i+=2)
            *(u64*)&BGOBJLine[i] = 0;
    }

    if (GPU2D.DispCnt & 0xE000)
        GPU2D.CalculateWindowMask(WindowMask, OBJWindow);
    else
        memset(WindowMask, 0xFF, 256);

    ApplySpriteMosaicX();
    CurBGXMosaicTable = MosaicTable[GPU2D.BGMosaicSize[0]].data();

    switch (GPU2D.DispCnt & 0x7)
    {
        case 0: DrawScanlineBGMode<0>(line); break;
        case 1: DrawScanlineBGMode<1>(line); break;
        case 2: DrawScanlineBGMode<2>(line); break;
        case 3: DrawScanlineBGMode<3>(line); break;
        case 4: DrawScanlineBGMode<4>(line); break;
        case 5: DrawScanlineBGMode<5>(line); break;
        case 6: DrawScanlineBGMode6(line); break;
        case 7: DrawScanlineBGMode7(line); break;
    }

    // color special effects
    // can likely be optimized

    if (HybridDesc && !GPU2D.Num && (GPU2D.DispCnt & 0x8))
    {
        HybridCompositeLine(dst);
        return;
    }

#if defined(LITEV_SOFT2D_NEON) && defined(__aarch64__)
    // 4-wide NEON port of the per-pixel ColorComposite loop (bit-exact).
    GPU2DNeon::ColorCompositeLine(dst, BGOBJLine, WindowMask,
                                  GPU2D.BlendCnt, GPU2D.EVA, GPU2D.EVB, GPU2D.EVY);
#else
    for (int i = 0; i < 256; i++)
    {
        u32 val1 = BGOBJLine[i];
        u32 val2 = BGOBJLine[256+i];

        dst[i] = ColorComposite(i, val1, val2);
    }
#endif
}


#ifdef S2D_PLANES
// ---- LITEV_SOFT2D_PLANES: layer planes + one select/resolve/composite pass per line ----
// Inspired by DSperate's 2D line pipeline (https://github.com/beebono/DSperate, GPL-3.0-or-later,
// commit f5fa35e, src/core/gpu/engine2d.cpp render_line / draw_bg_text / select_layers /
// ext_pal18, docs/techniques/03-2d-engines-and-scanline-pipeline.md). Same pixels as the
// layer-push path: each layer in the push order (priority 3..0, BG3..BG0 then OBJ) replaces the
// top where it is opaque and its window bit is set, pushing the old top to the second slot; the
// colours are then resolved for the top/second only, and ColorCompositeLine applies the effects.

// Pixels until p (+= d per pixel) leaves its 8-texel (0x800) cell, at least 1. Within a cell an
// affine map's entry, its overflow test and the tile row source stay the same.
[[gnu::always_inline]] static inline u32 S2DCellSteps(s32 p, s32 d)
{
    if (d == 0) return 256;
    const u32 dist = d > 0 ? (u32)((p | 0x7FF) - p) : (u32)(p - (p & ~0x7FF));
    const u32 ad = d > 0 ? (u32)d : (u32)-d;
    // a power-of-two step (unscaled or 90-degree maps) divides by a shift
    return ((ad & (ad - 1)) ? dist / ad : dist >> __builtin_ctz(ad)) + 1;
}

#ifdef LITEV_SOFT2D_AFFNEON
// LITEV_SOFT2D_AFFNEON: one map cell's span of an affine / extended tiled BG, 16 pixels at a time.
// Inside a cell every pixel reads the same 64-byte tile, so the texels are a table lookup (TBL) at
// offset (((y >> 8) & 7) << 3 | ((x >> 8) & 7)) ^ flip; only the low 11 bits of x/y matter there,
// so the coordinates step in u16 lanes (exact mod 2^16). Writes whole 16-lane chunks: lanes past n
// land in the next cells' range (rewritten by them) or in the plane's tail padding. Returns the OR of
// the n valid values.
[[gnu::always_inline]] static inline u32 S2DCellGather(const u8* tile, s32 x, s32 y, s32 rotA, s32 rotC, u32 n, u32 flip, u16 palbase, u16* v)
{
    const uint8x16x4_t t = vld1q_u8_x4(tile);
    const uint16x8_t lane0 = {0, 1, 2, 3, 4, 5, 6, 7};
    const uint16x8_t lane1 = vaddq_u16(lane0, vdupq_n_u16(8));
    const uint16x8_t ax = vdupq_n_u16((u16)rotA), ay = vdupq_n_u16((u16)rotC);
    const uint16x8_t sx0 = vmulq_u16(lane0, ax), sx1 = vmulq_u16(lane1, ax);
    const uint16x8_t sy0 = vmulq_u16(lane0, ay), sy1 = vmulq_u16(lane1, ay);
    const uint16x8_t v7 = vdupq_n_u16(7);
    const uint8x16_t vflip = vdupq_n_u8((u8)flip), vlane = vcombine_u8(vmovn_u16(lane0), vmovn_u16(lane1));
    const uint16x8_t vpal = vdupq_n_u16(palbase);
    uint16x8_t acc = vdupq_n_u16(0);
    u16 bx = (u16)x, by = (u16)y;
    const u16 stx = (u16)(rotA * 16), sty = (u16)(rotC * 16);
    for (u32 j = 0; j < n; j += 16, bx += stx, by += sty, v += 16)
    {
        const uint16x8_t x0 = vaddq_u16(vdupq_n_u16(bx), sx0), x1 = vaddq_u16(vdupq_n_u16(bx), sx1);
        const uint16x8_t y0 = vaddq_u16(vdupq_n_u16(by), sy0), y1 = vaddq_u16(vdupq_n_u16(by), sy1);
        const uint16x8_t o0 = vorrq_u16(vshlq_n_u16(vandq_u16(vshrq_n_u16(y0, 8), v7), 3), vandq_u16(vshrq_n_u16(x0, 8), v7));
        const uint16x8_t o1 = vorrq_u16(vshlq_n_u16(vandq_u16(vshrq_n_u16(y1, 8), v7), 3), vandq_u16(vshrq_n_u16(x1, 8), v7));
        const uint8x16_t off = veorq_u8(vcombine_u8(vmovn_u16(o0), vmovn_u16(o1)), vflip);
        uint8x16_t c = vqtbl4q_u8(t, off);
        const uint16x8_t c0 = vmovl_u8(vget_low_u8(c)), c1 = vmovl_u8(vget_high_u8(c));
        const uint16x8_t r0 = vandq_u16(vtstq_u16(c0, c0), vorrq_u16(c0, vpal));
        const uint16x8_t r1 = vandq_u16(vtstq_u16(c1, c1), vorrq_u16(c1, vpal));
        vst1q_u16(v, r0); vst1q_u16(v + 8, r1);
        // only the valid lanes count towards "any"
        c = vandq_u8(c, vcltq_u8(vlane, vdupq_n_u8((u8)std::min<u32>(n - j, 16))));
        acc = vorrq_u16(acc, vmovl_u8(vget_low_u8(c)));
        acc = vorrq_u16(acc, vmovl_u8(vget_high_u8(c)));
    }
    return vmaxvq_u16(acc) ? 0x8000u : 0u;
}
#endif

// A bitmap BG line on the identity matrix (rotA = 0x100, rotC = 0, no mosaic): the row is fixed and
// the texel column is (rotX >> 8) + i, wrapping at the bitmap width or transparent outside it. The
// line's texels (es bytes each, 0 outside) are copied in runs, split where the VRAM mask wraps.
// Inspired by DSperate's bitmap_row_degenerate (src/core/gpu/engine2d.cpp).
static void S2DBitmapRow(const u8* vram, u32 vmask, u32 base, u32 rowoff, u32 width, bool wrap, s32 rx, u32 es, u8* out)
{
    auto emit = [&](u32 texel, u32 i, u32 n)
    {
        u32 addr = base + (rowoff + texel) * es;
        while (n)
        {
            const u32 a = addr & vmask;
            const u32 m = std::min(n, (vmask + 1 - a) / es);
            memcpy(out + i * es, vram + a, m * es);
            addr += m * es; i += m; n -= m;
        }
    };
    const s32 x0 = rx >> 8;
    if (wrap)
    {
        u32 i = 0, x = (u32)x0 & (width - 1);
        while (i < 256) { const u32 n = std::min(256 - i, width - x); emit(x, i, n); i += n; x = 0; }
        return;
    }
    memset(out, 0, 256 * es);
    const s32 i0 = std::max(0, -x0), i1 = std::min<s32>(256, (s32)width - x0);
    if (i0 < i1) emit((u32)(x0 + i0), (u32)i0, (u32)(i1 - i0));
}

// 8-bit indices -> plane values (0x8000 | idx, 0 for index 0); returns whether any is opaque
static bool S2DPlaneFrom8(const u8* idx, u16* v)
{
    uint8x16_t acc = vdupq_n_u8(0);
    for (u32 i = 0; i < 256; i += 16)
    {
        const uint8x16_t x = vld1q_u8(idx + i);
        acc = vorrq_u8(acc, x);
        const uint16x8_t lo = vmovl_u8(vget_low_u8(x)), hi = vmovl_u8(vget_high_u8(x));
        vst1q_u16(v + i, vorrq_u16(lo, vandq_u16(vtstq_u16(lo, lo), vdupq_n_u16(0x8000))));
        vst1q_u16(v + i + 8, vorrq_u16(hi, vandq_u16(vtstq_u16(hi, hi), vdupq_n_u16(0x8000))));
    }
    return vmaxvq_u8(acc) != 0;
}

// direct colours -> colour records by position (dc) + plane values (0x8000 | position where bit 15
// is set); returns whether any is opaque
static bool S2DPlaneFromDirect(const u16* col, u32* dc, u16* v)
{
    const uint16x8_t lane = {0, 1, 2, 3, 4, 5, 6, 7}, op = vdupq_n_u16(0x8000), m3e = vdupq_n_u16(0x3E);
    uint16x8_t acc = vdupq_n_u16(0);
    for (u32 i = 0; i < 256; i += 8)
    {
        const uint16x8_t c = vld1q_u16(col + i);
        const uint16x8_t o = vtstq_u16(c, op);
        acc = vorrq_u16(acc, o);
        vst1q_u16(v + i, vandq_u16(o, vorrq_u16(op, vaddq_u16(lane, vdupq_n_u16((u16)i)))));
        // S2DConvColor(c & 0x7FFF): r = (c & 0x1F) << 1, g = (c & 0x3E0) >> 4, b = (c & 0x7C00) >> 9
        uint8x8x4_t px;
        px.val[0] = vmovn_u16(vandq_u16(vshlq_n_u16(c, 1), m3e));
        px.val[1] = vmovn_u16(vandq_u16(vshrq_n_u16(c, 4), m3e));
        px.val[2] = vmovn_u16(vandq_u16(vshrq_n_u16(c, 9), m3e));
        px.val[3] = vdup_n_u8(0);
        vst4_u8((u8*)(dc + i), px);
    }
    return vmaxvq_u16(acc) != 0;
}

// the extended palettes in palmask (bit = palette) converted, if they changed since the copy
void SoftRenderer2D::ValidateBGExt(u32 slot, u32 palmask)
{
    for (u32 m = palmask & ~(u32)(BGExtChecked >> (slot * 16)) & 0xFFFF; m; m &= m - 1)
    {
        const u32 p = __builtin_ctz(m);
        const u64 bit = 1ull << (slot * 16 + p);
        BGExtChecked |= bit;
        const u16* src = GPU2D.GetBGExtPal(slot, p);
        u16* copy = &BGExtCopy[slot][p * 256];
        if ((BGExtHave & bit) && !memcmp(src, copy, 512)) continue;
        memcpy(copy, src, 512);
        u32* d = &BGExtConv[slot][p * 256];
        for (int i = 0; i < 256; i++) d[i] = S2DConvColor(copy[i]);
        BGExtHave |= bit;
    }
}

void SoftRenderer2D::ValidateOBJExt(u32 p)
{
    OBJExtChecked |= 1u << p;
    const u16* src = GPU2D.GetOBJExtPal() + p * 256;
    u16* copy = &OBJExtCopy[p * 256];
    if ((OBJExtHave & (1u << p)) && !memcmp(src, copy, 512)) return;
    memcpy(copy, src, 512);
    u32* d = &OBJExtConv[p * 256];
    for (int i = 0; i < 256; i++) d[i] = S2DConvColor(copy[i]);
    OBJExtHave |= 1u << p;
}

namespace
{
// the winning values through their tables, with the BGOBJLine layer flags: out = tab[t][v & 0x7FFF]
// | flag[t]. Pixel-indexed layers (OBJ, 3D, direct-colour BGs) carry the pixel position as value.
void S2DResolve(const u16* v, const u8* tid, const u32* const* tab, const u32* flag, u32 posmask, u32* out)
{
    for (u32 i = 0; i < 256; i += 16)
    {
        if (!S2DPlanes::UniformU8(tid + i))
        {
            for (u32 k = i; k < i + 16; k++) { const u32 t = tid[k]; out[k] = tab[t][v[k] & 0x7FFF] | flag[t]; }
            continue;
        }
        const u32 t = tid[i];
        const u32* tb = tab[t]; const u32 f = flag[t];
        if (t >= 5 && t != 6)
        {
            // backdrop / nothing: one record (TID_BACKDROP, TID_NONE; the value is always 0)
            const uint32x4_t c = vdupq_n_u32(tb[0] | f);
            for (u32 k = i; k < i + 16; k += 4) vst1q_u32(out + k, c);
            continue;
        }
        if ((posmask >> t) & 1)
        {
            // a position-indexed layer (3D, OBJ, direct colour): its records in place
            const uint32x4_t f3 = vdupq_n_u32(f);
            for (u32 k = i; k < i + 16; k += 4) vst1q_u32(out + k, vorrq_u32(vld1q_u32(tb + k), f3));
            continue;
        }
        for (u32 k = i; k < i + 16; k += 4)
        {
            u64 a; memcpy(&a, v + k, 8);
            out[k]   = tb[a & 0x7FFF] | f;         out[k+1] = tb[(a >> 16) & 0x7FFF] | f;
            out[k+2] = tb[(a >> 32) & 0x7FFF] | f; out[k+3] = tb[(a >> 48) & 0x7FFF] | f;
        }
    }
}
}

#ifdef LITEV_HYB_PLANES
// hybrid descriptor lines: the 3D placeholder is "opaque" everywhere (its window decides) and
// resolves to the record 0x40000000 (colour 0)
alignas(16) static const u32 S2DHybZero[256] = {};
alignas(16) static const struct S2DHybOpaqueT { u32 v[256]; S2DHybOpaqueT() { for (u32& x : v) x = 0xFF000000; } } S2DHybOpaqueInit;
static const u32* const S2DHybOpaque3D = S2DHybOpaqueInit.v;
#endif

#define S2D_SEL(fn, ...) (second ? (win ? fn<true, true>(__VA_ARGS__) : fn<true, false>(__VA_ARGS__)) \
                                 : (win ? fn<false, true>(__VA_ARGS__) : fn<false, false>(__VA_ARGS__)))

void SoftRenderer2D::DrawScanlinePlanes(u32 line, u32* dst)
{
    using namespace S2DPlanes;
    const u32 dispCnt = GPU2D.DispCnt;
    const u16* bgCnt = GPU2D.BGCnt;
    const u32 layers = GPU2D.LayerEnable;

    const bool win = dispCnt & 0xE000;
    if (win) GPU2D.CalculateWindowMask(WindowMask, OBJWindow);
    else memset(WindowMask, 0xFF, 256);
    ApplySpriteMosaicX();
    CurBGXMosaicTable = MosaicTable[GPU2D.BGMosaicSize[0]].data();

    // 1. the BG planes. Layer kind per mode as in DrawScanlineBGMode<>/6/7: 1 text, 2 affine,
    // 3 extended, 4 large. BG0 is the 3D layer on engine A when DISPCNT bit 3 is set.
    static constexpr u8 kinds[8][4] = {{1,1,1,1}, {1,1,1,2}, {1,1,2,2}, {1,1,1,3}, {1,1,2,3}, {1,1,3,3}, {0,0,4,0}, {1,1,0,0}};
    const u32 mode = dispCnt & 7;
    const bool l3d = !GPU2D.Num && (dispCnt & 0x8) && (layers & 1);
#ifdef LITEV_HYB_PLANES
    // LITEV_HYB_PLANES: a hybrid descriptor line (as DrawScanline_BGOBJ's layer-push path): the 3D
    // is a placeholder (record 0x40000000) wherever its window bit is set, the top and second
    // layers it pushes down become Under3D, and HybridCompositeLine writes the descriptors. The
    // effect shortcuts below assume a known 3D alpha: off for these lines (both layers kept).
    const bool hyb = HybridDesc && !GPU2D.Num && (dispCnt & 0x8);
#else
    constexpr bool hyb = false;
#endif
    const bool mosaicOn = GPU2D.BGMosaicSize[0] > 0;
    u32 present = 0;
    for (u32 n = 0; n < 4; n++)
    {
        PlaneAny[n] = false;
        if (!(layers & (1u << n)) || (n == 0 && !GPU2D.Num && (dispCnt & 0x8))) continue;
        const bool mos = (bgCnt[n] & (1<<6)) && mosaicOn;
        switch (kinds[mode][n])
        {
        case 1: if (mos) PlaneBG_Text<true>(line, n); else PlaneBG_Text<false>(line, n); break;
        case 2: if (mos) PlaneBG_Affine<true>(n); else PlaneBG_Affine<false>(n); break;
        case 3: if (mos) PlaneBG_Extended<true>(n); else PlaneBG_Extended<false>(n); break;
        case 4: if (mos) PlaneBG_Large<true>(); else PlaneBG_Large<false>(); break;
        default: break;
        }
        if (PlaneAny[n]) present |= 1u << n;
    }

    // 2. what the colour effects can read on this line (a "no" must hold for every pixel)
#ifdef LITEV_SOFT2D_SPRLIST
    const bool objOn = (layers & 0x10) && NumSprites && OBJPrioMask;
#else
    const bool objOn = (layers & 0x10) && NumSprites;
#endif
    bool objBlend = false;   // a semi-transparent or bitmap sprite pixel (flag 0x80)
    if (objOn)
    {
        uint32x4_t acc = vdupq_n_u32(0);
        for (u32 i = 0; i < 256; i += 4) acc = vorrq_u32(acc, vld1q_u32(OBJLine + i));
        objBlend = vmaxvq_u32(acc) >> 31;
        // the opaque sprite pixels' colours (InterleaveSprites' palette choice), by position
        const u32* cvo = CurPalConv + (GPU2D.Num ? 0x300 : 0x100);
        const uint32x4_t vop = vdupq_n_u32(OBJ_IsOpaque);
        for (u32 i = 0; i < 256; i += 16)
        {
            const uint32x4_t o = vorrq_u32(vorrq_u32(vld1q_u32(OBJLine + i), vld1q_u32(OBJLine + i + 4)),
                                           vorrq_u32(vld1q_u32(OBJLine + i + 8), vld1q_u32(OBJLine + i + 12)));
            if (!vmaxvq_u32(vandq_u32(o, vop))) continue;
            for (u32 k = i; k < i + 16; k++)
            {
                const u32 px = OBJLine[k];
                if (!(px & OBJ_IsOpaque)) continue;
                const u32 fl = px & 0xFF000000;
                if (px & OBJ_DirectColor)      ObjCol[k] = S2DConvColor(px & 0x7FFF) | fl;
                else if (px & OBJ_StandardPal) ObjCol[k] = cvo[px & 0xFF] | fl;
                else
                {
                    if (!((OBJExtChecked >> ((px >> 8) & 0xF)) & 1)) ValidateOBJExt((px >> 8) & 0xF);
                    ObjCol[k] = OBJExtConv[px & 0xFFF] | fl;
                }
            }
        }
    }
    const u32 bc = GPU2D.BlendCnt;
    // identity coefficients make the BLDCNT effect a no-op on the colours (EVA 16/EVB 0, EVY 0); it
    // then only sets the alpha byte to 0xFF, which no consumer reads beyond "non-zero" (capture)
    // and the line records' alpha is non-zero anyway. Inspired by DSperate's effect_possible.
    const u32 ceRaw = (bc >> 6) & 3;
    const u32 ce = ((ceRaw == 1 && GPU2D.EVA == 16 && GPU2D.EVB == 0) || (ceRaw >= 2 && GPU2D.EVY == 0)) ? 0 : ceRaw;
    // first targets that can be on top: backdrop, the BG planes, OBJ, 3D (as BG0)
    const bool fxPossible = ce && (bc & (0x20 | present | (objOn ? 0x10 : 0) | (l3d ? 0x01 : 0)));
    // The second layer matters only through a second-target hit (BLDCNT bits 8-13): a sprite blend
    // (flag 0x80), the 3D layer's alpha blend, or a BLDCNT alpha blend (effect 1). A 3D pixel over a
    // second target blends with alpha+1 of 32 (alpha 31 keeps it) and is exempt from the BLDCNT
    // effect, so with no translucent 3D pixel the hit only matters if a brightness effect could
    // apply to the 3D layer (first target bit 0).
    bool l3dHit = false;
    if (l3d && !hyb && (bc & 0x3F00))
    {
        if ((ce == 2 || ce == 3) && (bc & 0x01)) l3dHit = true;
        else
        {
            // any 3D pixel drawn (alpha byte non-zero) with alpha (bits 24-28) below 31
            const uint32x4_t m1f = vdupq_n_u32(0x1F), z = vdupq_n_u32(0);
            uint32x4_t acc = z;
            for (u32 i = 0; i < 256; i += 4)
            {
                const uint32x4_t a = vshrq_n_u32(vld1q_u32(Cur3DLine + i), 24);
                acc = vorrq_u32(acc, vbicq_u32(vmvnq_u32(vceqq_u32(a, z)), vceqq_u32(vandq_u32(a, m1f), m1f)));
            }
            l3dHit = vmaxvq_u32(acc) != 0;
        }
    }
    const bool second = hyb || ((bc & 0x3F00) && objBlend) || l3dHit || (fxPossible && ce == 1);
    const bool flat = !second && !fxPossible;

    if (flat && !present && !objOn && !l3d)
    {
        // nothing but the backdrop on the line
        const uint32x4_t c = vdupq_n_u32(CurPalConv[GPU2D.Num ? 0x200 : 0] | 0x20000000);
        for (u32 i = 0; i < 256; i += 4) vst1q_u32(dst + i, c);
        return;
    }

    // the topmost layer in push order (priority 0 first; within one, OBJ, then BG0..BG3): when it is
    // a BG plane opaque everywhere with no window, it alone decides the top (DSperate resolve16_one)
    if (!second && !win)
    {
        int top = -1;
        for (int prio = 0; prio < 4 && top < 0; prio++)
        {
#ifdef LITEV_SOFT2D_SPRLIST
            if (objOn && ((OBJPrioMask >> prio) & 1)) break;
#else
            if (objOn) break;
#endif
            for (int n = 0; n < 4; n++)
            {
                if ((bgCnt[n] & 3) != (u32)prio) continue;
                if (n == 0 && l3d) { prio = 4; break; }
                if (present & (1u << n)) { top = n; break; }
            }
        }
        if (top >= 0)
        {
            const u16* pv = PlaneV(top);
            uint16x8_t acc = vdupq_n_u16(0xFFFF);
            for (u32 i = 0; i < 256; i += 8) acc = vandq_u16(acc, vld1q_u16(pv + i));
            if (vminvq_u16(acc) & 0x8000)
            {
                const u32* tb = PlaneTab[top]; const u32 f = 0x01000000u << top;
                u32* const out = flat ? dst : BGOBJLine;
                if (tb == DirCol[top])
                    for (u32 i = 0; i < 256; i += 4) vst1q_u32(out + i, vorrq_u32(vld1q_u32(tb + i), vdupq_n_u32(f)));
                else
                    for (u32 i = 0; i < 256; i += 4)
                    {
                        u64 a; memcpy(&a, pv + i, 8);
                        out[i]   = tb[a & 0x7FFF] | f;         out[i+1] = tb[(a >> 16) & 0x7FFF] | f;
                        out[i+2] = tb[(a >> 32) & 0x7FFF] | f; out[i+3] = tb[(a >> 48) & 0x7FFF] | f;
                    }
                if (!flat) FadeLine(BGOBJLine, WindowMask, bc, GPU2D.EVY, dst);
                return;
            }
        }
    }

    // 3. priority select, in the layer-push order
    memset(Top16, 0, sizeof(Top16)); memset(TopTid, TID_BACKDROP, sizeof(TopTid));
    if (second) { memset(Sec16, 0, sizeof(Sec16)); memset(SecTid, TID_NONE, sizeof(SecTid)); }
    for (int prio = 3; prio >= 0; prio--)
    {
        for (int n = 3; n >= 0; n--)
        {
            if ((bgCnt[n] & 3) != (u32)prio) continue;
            if (n == 0 && l3d)
            {
#ifdef LITEV_HYB_PLANES
                if (hyb)
                {
                    // Under3D = the top and second here (read only where the 3D is pushed)
                    memcpy(UTop16, Top16, sizeof(Top16)); memcpy(UTopTid, TopTid, sizeof(TopTid));
                    memcpy(USec16, Sec16, sizeof(Sec16)); memcpy(USecTid, SecTid, sizeof(SecTid));
                    S2D_SEL(Select3D, S2DHybOpaque3D, WindowMask, TID_3D, Top16, TopTid, Sec16, SecTid);
                }
                else
#endif
                S2D_SEL(Select3D, Cur3DLine, WindowMask, TID_3D, Top16, TopTid, Sec16, SecTid);
            }
            else if (present & (1u << n))
                S2D_SEL(SelectBG, PlaneV(n), WindowMask, (u8)(1u << n), (u8)n, Top16, TopTid, Sec16, SecTid);
        }
#ifdef LITEV_SOFT2D_SPRLIST
        if (objOn && ((OBJPrioMask >> prio) & 1))
#else
        if (objOn)
#endif
            S2D_SEL(SelectOBJ, OBJLine, (u32)OBJ_OpaPrioMask, ((u32)prio << 16) | OBJ_IsOpaque, WindowMask, (u8)TID_OBJ, Top16, TopTid, Sec16, SecTid);
    }

    // 4. resolve the top (and second) colours, then the colour effects
    const u32* pal = CurPalConv + (GPU2D.Num ? 0x200 : 0);
    const u32 backdrop = pal[0] | 0x20000000;   // DrawScanline_BGOBJ's backdrop record
    static const u32 zero = 0;
    const u32* tab[8];
    u32 flag[8];
    for (u32 n = 0; n < 4; n++) { tab[n] = PlaneTab[n]; flag[n] = 0x01000000u << n; }
    tab[TID_OBJ] = ObjCol; flag[TID_OBJ] = 0;
    tab[TID_BACKDROP] = &backdrop; flag[TID_BACKDROP] = 0;
#ifdef LITEV_HYB_PLANES
    tab[TID_3D] = hyb ? S2DHybZero : Cur3DLine; flag[TID_3D] = 0x40000000;
#else
    tab[TID_3D] = Cur3DLine; flag[TID_3D] = 0x40000000;
#endif
    tab[TID_NONE] = &zero; flag[TID_NONE] = 0;
    u32 posmask = (1u << TID_3D) | (1u << TID_OBJ);
    for (u32 n = 0; n < 4; n++) if (PlaneTab[n] == DirCol[n]) posmask |= 1u << n;
    if (flat)
    {
        S2DResolve(Top16, TopTid, tab, flag, posmask, dst);
        return;
    }
    S2DResolve(Top16, TopTid, tab, flag, posmask, BGOBJLine);
    if (!second)
    {
        FadeLine(BGOBJLine, WindowMask, bc, GPU2D.EVY, dst);   // the second layer can't matter
        return;
    }
    S2DResolve(Sec16, SecTid, tab, flag, posmask, BGOBJLine + 256);
#ifdef LITEV_HYB_PLANES
    if (hyb)
    {
        if (l3d)
        {
            S2DResolve(UTop16, UTopTid, tab, flag, posmask, Under3D[0]);
            S2DResolve(USec16, USecTid, tab, flag, posmask, Under3D[1]);
        }
        HybridCompositeLine(dst);
        return;
    }
#endif
    GPU2DNeon::ColorCompositeLine(dst, BGOBJLine, WindowMask, bc, GPU2D.EVA, GPU2D.EVB, GPU2D.EVY);
}
#undef S2D_SEL

template<bool mosaic>
void SoftRenderer2D::PlaneBG_Text(u32 line, u32 bgnum)
{
    using namespace S2DPlanes;
    const u16 bgcnt = GPU2D.BGCnt[bgnum];
    u16 xoff = GPU2D.BGXPos[bgnum];
    u16 yoff = GPU2D.BGYPos[bgnum];
    if (bgcnt & (1<<6)) yoff += GPU2D.BGMosaicLine;
    else                yoff += line;

    const u32 widexmask = (bgcnt & (1<<14)) ? 0x100 : 0;
    const bool c256 = bgcnt & (1<<7);
    const bool extpal = GPU2D.DispCnt & (1<<30);
    const u32 extpalslot = ((bgnum<2) && (bgcnt&0x2000)) ? (2+bgnum) : bgnum;

    u8* bgvram;
    u32 bgvrammask;
    GPU2D.GetBGVRAM(bgvram, bgvrammask);
    u32 tilesetaddr, tilemapaddr;
    if (GPU2D.Num)
    {
        tilesetaddr = ((bgcnt & 0x003C) << 12);
        tilemapaddr = ((bgcnt & 0x1F00) << 3);
    }
    else
    {
        tilesetaddr = ((GPU2D.DispCnt & 0x07000000) >> 8) + ((bgcnt & 0x003C) << 12);
        tilemapaddr = ((GPU2D.DispCnt & 0x38000000) >> 11) + ((bgcnt & 0x1F00) << 3);
    }
    if (bgcnt & (1<<15))
    {
        tilemapaddr += ((yoff & 0x1F8) << 3);
        if (bgcnt & (1<<14))
            tilemapaddr += ((yoff & 0x100) << 3);
    }
    else
        tilemapaddr += ((yoff & 0xF8) << 3);

    const bool ext = c256 && extpal;
    if (ext) PlaneTab[bgnum] = BGExtConv[extpalslot];
    else PlaneTab[bgnum] = CurPalConv + (GPU2D.Num ? 0x200 : 0);
    u16* const v = PlaneV(bgnum);
    const u32 yrow = yoff & 7;

    if constexpr (!mosaic)
    {
        // the 33 map entries and tile rows under the line, then the rows expanded at -scroll
        // (DSperate draw_bg_text). The rows are 4/8-byte aligned and the VRAM mask is a power of
        // two minus one, so a masked row address covers the whole row.
        alignas(16) u8 rows[33 * 8 + 8];
        u8 ctl[40];
        // map entries: a screen block's row is 32 contiguous entries (64-byte aligned, so a masked
        // run never wraps), the next block (wide maps) or the same row again after it
        // (the line starts in the first of the two copied rows, at tx; tx + 32 <= 63)
        alignas(16) u16 maprow[64];
        {
            const u32 blk = (xoff & widexmask) ? 1 : 0;
            memcpy(maprow, &bgvram[(tilemapaddr + (blk << 11)) & bgvrammask], 64);
            memcpy(maprow + 32, &bgvram[(tilemapaddr + ((widexmask ? blk ^ 1 : blk) << 11)) & bgvrammask], 64);
        }
        const u16* const tiles = maprow + ((xoff >> 3) & 31);
        u64 acc = 0;
        if (c256)
        {
            for (u32 t = 0; t < 33; t++)
            {
                const u16 tile = tiles[t];
                const u32 row = (tile & (1<<11)) ? (7 - yrow) : yrow;
                u64 px; memcpy(&px, &bgvram[(tilesetaddr + ((tile & 0x03FF) << 6) + (row << 3)) & bgvrammask], 8);
                memcpy(rows + t * 8, &px, 8);
                acc |= px;
            }
            if (ext && acc) ValidateBGExt(extpalslot, 0xFFFF);   // the slot once a frame
        }
        else
        {
            for (u32 t = 0; t < 33; t++)
            {
                const u16 tile = tiles[t];
                const u32 row = (tile & (1<<11)) ? (7 - yrow) : yrow;
                u32 px; memcpy(&px, &bgvram[(tilesetaddr + ((tile & 0x03FF) << 5) + (row << 2)) & bgvrammask], 4);
                memcpy(rows + t * 4, &px, 4);
                acc |= px;
            }
        }
        if (!acc) { PlaneAny[bgnum] = false; return; }
        TextCtl(tiles, ctl);
        u16* const dv = v - (xoff & 7);
        PlaneAny[bgnum] = c256 ? TextRow256(rows, ctl, 33, ext, dv) : TextRow16(rows, ctl, 33, dv);
    }
    else
    {
        // the layer-push path's mosaic loop, writing plane values
        if (ext) ValidateBGExt(extpalslot, 0xFFFF);
        u16 curtile = *(u16*)&bgvram[(tilemapaddr + ((xoff & 0xF8) >> 2) + ((xoff & widexmask) << 3)) & bgvrammask];
        u32 pixelsaddr = 0, palbase = 0;
        auto setup = [&]()
        {
            const u32 row = (curtile & (1<<11)) ? (7 - yrow) : yrow;
            if (c256)
            {
                palbase = ext ? ((u32)(curtile >> 12) << 8) : 0;
                pixelsaddr = tilesetaddr + ((curtile & 0x03FF) << 6) + (row << 3);
            }
            else
            {
                palbase = (curtile & 0xF000) >> 8;
                pixelsaddr = tilesetaddr + ((curtile & 0x03FF) << 5) + (row << 2);
            }
        };
        setup();
        u32 lastxpos = xoff;
        u32 any = 0;
        for (int i = 0; i < 256; i++)
        {
            u32 xpos = xoff - CurBGXMosaicTable[i];
            if ((xpos >> 3) != (lastxpos >> 3))
            {
                curtile = *(u16*)&bgvram[(tilemapaddr + ((xpos & 0xF8) >> 2) + ((xpos & widexmask) << 3)) & bgvrammask];
                setup();
                lastxpos = xpos;
            }
            const u32 tilexoff = (curtile & (1<<10)) ? (7-(xpos&0x7)) : (xpos&0x7);
            u32 color;
            if (c256) color = bgvram[(pixelsaddr + tilexoff) & bgvrammask];
            else if (tilexoff & 0x1) color = bgvram[(pixelsaddr + (tilexoff >> 1)) & bgvrammask] >> 4;
            else color = bgvram[(pixelsaddr + (tilexoff >> 1)) & bgvrammask] & 0x0F;
            const u16 val = color ? (u16)(LV_OPAQUE | palbase | color) : 0;
            v[i] = val;
            any |= val;
            xoff++;
        }
        PlaneAny[bgnum] = any != 0;
    }
}

template<bool mosaic>
void SoftRenderer2D::PlaneBG_Affine(u32 bgnum)
{
    const u16 bgcnt = GPU2D.BGCnt[bgnum];
    u32 coordmask, yshift;
    switch ((bgcnt >> 14) & 0x3)
    {
        case 0: coordmask = 0x07800; yshift = 7; break;
        case 1: coordmask = 0x0F800; yshift = 8; break;
        case 2: coordmask = 0x1F800; yshift = 9; break;
        default: coordmask = 0x3F800; yshift = 10; break;
    }
    const u32 overflowmask = (bgcnt & (1<<13)) ? 0 : ~(coordmask | 0x7FF);
    const s16 rotA = GPU2D.BGRotA[bgnum-2];
    const s16 rotC = GPU2D.BGRotC[bgnum-2];
    s32 rotX = GPU2D.BGXRefInternal[bgnum-2];
    s32 rotY = GPU2D.BGYRefInternal[bgnum-2];

    u8* bgvram;
    u32 bgvrammask;
    GPU2D.GetBGVRAM(bgvram, bgvrammask);
    u32 tilesetaddr, tilemapaddr;
    if (GPU2D.Num)
    {
        tilesetaddr = ((bgcnt & 0x003C) << 12);
        tilemapaddr = ((bgcnt & 0x1F00) << 3);
    }
    else
    {
        tilesetaddr = ((GPU2D.DispCnt & 0x07000000) >> 8) + ((bgcnt & 0x003C) << 12);
        tilemapaddr = ((GPU2D.DispCnt & 0x38000000) >> 11) + ((bgcnt & 0x1F00) << 3);
    }
    PlaneTab[bgnum] = CurPalConv + (GPU2D.Num ? 0x200 : 0);
    u16* const v = PlaneV(bgnum);
    yshift -= 3;
    u32 any = 0;
    if constexpr (!mosaic)
    {
        // a map cell at a time: one map entry per span of pixels inside the same 8x8 tile
        for (u32 i = 0; i < 256; )
        {
            s32 x = rotX, y = rotY;
            const u32 n = std::min<u32>(std::min(S2DCellSteps(x, rotA), S2DCellSteps(y, rotC)), 256 - i);
            if ((x | y) & overflowmask)
                memset(&v[i], 0, n * 2);
            else
            {
                const u32 curtile = bgvram[(tilemapaddr + ((((y & coordmask) >> 11) << yshift) + ((x & coordmask) >> 11))) & bgvrammask];
                const u32 tilebase = tilesetaddr + (curtile << 6);
#ifdef LITEV_SOFT2D_AFFNEON
                any |= S2DCellGather(&bgvram[tilebase & bgvrammask], x, y, rotA, rotC, n, 0, 0x8000, &v[i]);
#else
                for (u32 k = i; k < i + n; k++)
                {
                    const u32 color = bgvram[(tilebase + ((((u32)y >> 8) & 0x7) << 3) + (((u32)x >> 8) & 0x7)) & bgvrammask];
                    const u16 val = color ? (u16)(0x8000 | color) : 0;
                    v[k] = val;
                    any |= val;
                    x += rotA;
                    y += rotC;
                }
#endif
            }
            rotX += (s32)n * rotA;
            rotY += (s32)n * rotC;
            i += n;
        }
        PlaneAny[bgnum] = any != 0;
        return;
    }
    for (int i = 0; i < 256; i++)
    {
        s32 finalX, finalY;
        if (mosaic)
        {
            int im = CurBGXMosaicTable[i];
            finalX = rotX - (im * rotA);
            finalY = rotY - (im * rotC);
        }
        else
        {
            finalX = rotX;
            finalY = rotY;
        }
        u16 val = 0;
        if (!((finalX|finalY) & overflowmask))
        {
            const u32 curtile = bgvram[(tilemapaddr + ((((finalY & coordmask) >> 11) << yshift) + ((finalX & coordmask) >> 11))) & bgvrammask];
            const u32 color = bgvram[(tilesetaddr + (curtile << 6) + (((finalY >> 8) & 0x7) << 3) + ((finalX >> 8) & 0x7)) & bgvrammask];
            if (color) val = 0x8000 | color;
        }
        v[i] = val;
        any |= val;
        rotX += rotA;
        rotY += rotC;
    }
    PlaneAny[bgnum] = any != 0;
}

template<bool mosaic>
void SoftRenderer2D::PlaneBG_Extended(u32 bgnum)
{
    const u16 bgcnt = GPU2D.BGCnt[bgnum];
    u8* bgvram;
    u32 bgvrammask;
    GPU2D.GetBGVRAM(bgvram, bgvrammask);
    const bool extpal = GPU2D.DispCnt & (1<<30);
    const s16 rotA = GPU2D.BGRotA[bgnum-2];
    const s16 rotC = GPU2D.BGRotC[bgnum-2];
    s32 rotX = GPU2D.BGXRefInternal[bgnum-2];
    s32 rotY = GPU2D.BGYRefInternal[bgnum-2];
    u16* const v = PlaneV(bgnum);
    PlaneTab[bgnum] = CurPalConv + (GPU2D.Num ? 0x200 : 0);
    u32 any = 0;

#define S2D_FINAL() \
        s32 finalX, finalY; \
        if (mosaic) { int im = CurBGXMosaicTable[i]; finalX = rotX - (im * rotA); finalY = rotY - (im * rotC); } \
        else { finalX = rotX; finalY = rotY; }

    if (bgcnt & (1<<7))
    {
        // bitmap modes
        u32 xmask, ymask, yshift;
        switch ((bgcnt >> 14) & 0x3)
        {
            case 0: xmask = 0x07FFF; ymask = 0x07FFF; yshift = 7; break;
            case 1: xmask = 0x0FFFF; ymask = 0x0FFFF; yshift = 8; break;
            case 2: xmask = 0x1FFFF; ymask = 0x0FFFF; yshift = 9; break;
            default: xmask = 0x1FFFF; ymask = 0x1FFFF; yshift = 9; break;
        }
        const u32 ofxmask = (bgcnt & (1<<13)) ? 0 : ~xmask;
        const u32 ofymask = (bgcnt & (1<<13)) ? 0 : ~ymask;
        const u32 tilemapaddr = ((bgcnt & 0x1F00) << 6);
        if (bgcnt & (1<<2))
        {
            // direct colour bitmap: the colours by position, the plane holds the position
            u32* const dc = DirCol[bgnum];
            PlaneTab[bgnum] = dc;
            if (!mosaic && rotA == 0x100 && rotC == 0)
            {
                if (!ofymask || !((u32)rotY & ofymask))
                {
                    alignas(16) u16 col[256];
                    S2DBitmapRow(bgvram, bgvrammask, tilemapaddr, (((u32)rotY & ymask) >> 8) << yshift, (xmask >> 8) + 1, !ofxmask, rotX, 2, (u8*)col);
                    PlaneAny[bgnum] = S2DPlaneFromDirect(col, dc, v);
                }
                return;
            }
            for (int i = 0; i < 256; i++)
            {
                S2D_FINAL()
                u16 val = 0;
                if (!(finalX & ofxmask) && !(finalY & ofymask))
                {
                    const u16 color = *(u16*)&bgvram[(tilemapaddr + (((((finalY & ymask) >> 8) << yshift) + ((finalX & xmask) >> 8)) << 1)) & bgvrammask];
                    if (color & 0x8000) { dc[i] = S2DConvColor(color & 0x7FFF); val = 0x8000 | i; }
                }
                v[i] = val;
                any |= val;
                rotX += rotA;
                rotY += rotC;
            }
        }
        else
        {
            if (!mosaic && rotA == 0x100 && rotC == 0)
            {
                if (!ofymask || !((u32)rotY & ofymask))
                {
                    alignas(16) u8 idx[256];
                    S2DBitmapRow(bgvram, bgvrammask, tilemapaddr, (((u32)rotY & ymask) >> 8) << yshift, (xmask >> 8) + 1, !ofxmask, rotX, 1, idx);
                    PlaneAny[bgnum] = S2DPlaneFrom8(idx, v);
                }
                return;
            }
            for (int i = 0; i < 256; i++)
            {
                S2D_FINAL()
                u16 val = 0;
                if (!(finalX & ofxmask) && !(finalY & ofymask))
                {
                    const u32 color = bgvram[(tilemapaddr + (((finalY & ymask) >> 8) << yshift) + ((finalX & xmask) >> 8)) & bgvrammask];
                    if (color) val = 0x8000 | color;
                }
                v[i] = val;
                any |= val;
                rotX += rotA;
                rotY += rotC;
            }
        }
    }
    else
    {
        // mixed affine/text mode: 16-bit map entries, 256-colour tiles, extended palettes
        u32 coordmask, yshift;
        switch ((bgcnt >> 14) & 0x3)
        {
            case 0: coordmask = 0x07800; yshift = 7; break;
            case 1: coordmask = 0x0F800; yshift = 8; break;
            case 2: coordmask = 0x1F800; yshift = 9; break;
            default: coordmask = 0x3F800; yshift = 10; break;
        }
        const u32 overflowmask = (bgcnt & (1<<13)) ? 0 : ~(coordmask | 0x7FF);
        u32 tilesetaddr, tilemapaddr;
        if (GPU2D.Num)
        {
            tilesetaddr = ((bgcnt & 0x003C) << 12);
            tilemapaddr = ((bgcnt & 0x1F00) << 3);
        }
        else
        {
            tilesetaddr = ((GPU2D.DispCnt & 0x07000000) >> 8) + ((bgcnt & 0x003C) << 12);
            tilemapaddr = ((GPU2D.DispCnt & 0x38000000) >> 11) + ((bgcnt & 0x1F00) << 3);
        }
        if (extpal) PlaneTab[bgnum] = BGExtConv[bgnum];
        yshift -= 3;
        if constexpr (!mosaic)
        {
            u32 palmask = 0;
            // a map cell at a time (see PlaneBG_Affine)
            for (u32 i = 0; i < 256; )
            {
                s32 x = rotX, y = rotY;
                const u32 n = std::min<u32>(std::min(S2DCellSteps(x, rotA), S2DCellSteps(y, rotC)), 256 - i);
                if ((x | y) & overflowmask)
                    memset(&v[i], 0, n * 2);
                else
                {
                    const u16 curtile = *(u16*)&bgvram[(tilemapaddr + (((((y & coordmask) >> 11) << yshift) + ((x & coordmask) >> 11)) << 1)) & bgvrammask];
                    const u32 tilebase = tilesetaddr + ((curtile & 0x03FF) << 6);
                    const u32 palbase = 0x8000 | (extpal ? ((u32)(curtile >> 12) << 8) : 0);
                    palmask |= 1u << (curtile >> 12);
                    // (tileyoff << 3) + tilexoff, each flipped as 7 - off = off ^ 7
                    const u32 flip = ((curtile & (1<<10)) ? 0x07 : 0) | ((curtile & (1<<11)) ? 0x38 : 0);
#ifdef LITEV_SOFT2D_AFFNEON
                    any |= S2DCellGather(&bgvram[tilebase & bgvrammask], x, y, rotA, rotC, n, flip, (u16)palbase, &v[i]);
#else
                    for (u32 k = i; k < i + n; k++)
                    {
                        const u32 off = (((((u32)y >> 8) & 0x7) << 3) | (((u32)x >> 8) & 0x7)) ^ flip;
                        const u32 color = bgvram[(tilebase + off) & bgvrammask];
                        const u16 val = color ? (u16)(palbase | color) : 0;
                        v[k] = val;
                        any |= val;
                        x += rotA;
                        y += rotC;
                    }
#endif
                }
                rotX += (s32)n * rotA;
                rotY += (s32)n * rotC;
                i += n;
            }
            if (extpal && any) ValidateBGExt(bgnum, palmask);
            PlaneAny[bgnum] = any != 0;
            return;
        }
        if (extpal) ValidateBGExt(bgnum, 0xFFFF);
        u32 lastmapoff = ~0u, tilebase = 0, palbase = 0, flip = 0;
        for (int i = 0; i < 256; i++)
        {
            S2D_FINAL()
            u16 val = 0;
            if (!((finalX|finalY) & overflowmask))
            {
                // consecutive pixels on the same map entry (scaled/rotated maps) reuse it
                const u32 mapoff = (((((finalY & coordmask) >> 11) << yshift) + ((finalX & coordmask) >> 11)) << 1);
                if (mapoff != lastmapoff)
                {
                    lastmapoff = mapoff;
                    const u16 curtile = *(u16*)&bgvram[(tilemapaddr + mapoff) & bgvrammask];
                    tilebase = tilesetaddr + ((curtile & 0x03FF) << 6);
                    palbase = extpal ? ((u32)(curtile >> 12) << 8) : 0;
                    flip = ((curtile & (1<<10)) ? 0x07 : 0) | ((curtile & (1<<11)) ? 0x38 : 0);
                }
                // (tileyoff << 3) + tilexoff, each flipped as 7 - off = off ^ 7
                const u32 off = (((((u32)finalY >> 8) & 0x7) << 3) | (((u32)finalX >> 8) & 0x7)) ^ flip;
                const u32 color = bgvram[(tilebase + off) & bgvrammask];
                if (color) val = (u16)(0x8000 | palbase | color);
            }
            v[i] = val;
            any |= val;
            rotX += rotA;
            rotY += rotC;
        }
    }
#undef S2D_FINAL
    PlaneAny[bgnum] = any != 0;
}

template<bool mosaic>
void SoftRenderer2D::PlaneBG_Large()
{
    const u16 bgcnt = GPU2D.BGCnt[2];
    u32 xmask, ymask, yshift;
    switch ((bgcnt >> 14) & 0x3)
    {
        case 0: xmask = 0x1FFFF; ymask = 0x3FFFF; yshift = 9; break;
        case 1: xmask = 0x3FFFF; ymask = 0x1FFFF; yshift = 10; break;
        case 2: xmask = 0x1FFFF; ymask = 0x0FFFF; yshift = 9; break;
        default: xmask = 0x1FFFF; ymask = 0x1FFFF; yshift = 9; break;
    }
    const u32 ofxmask = (bgcnt & (1<<13)) ? 0 : ~xmask;
    const u32 ofymask = (bgcnt & (1<<13)) ? 0 : ~ymask;
    const s16 rotA = GPU2D.BGRotA[0];
    const s16 rotC = GPU2D.BGRotC[0];
    s32 rotX = GPU2D.BGXRefInternal[0];
    s32 rotY = GPU2D.BGYRefInternal[0];
    u8* bgvram;
    u32 bgvrammask;
    GPU2D.GetBGVRAM(bgvram, bgvrammask);
    PlaneTab[2] = CurPalConv + (GPU2D.Num ? 0x200 : 0);
    u16* const v = PlaneV(2);
    u32 any = 0;
    if (!mosaic && rotA == 0x100 && rotC == 0)
    {
        PlaneAny[2] = false;
        if (!ofymask || !((u32)rotY & ofymask))
        {
            alignas(16) u8 idx[256];
            S2DBitmapRow(bgvram, bgvrammask, 0, (((u32)rotY & ymask) >> 8) << yshift, (xmask >> 8) + 1, !ofxmask, rotX, 1, idx);
            PlaneAny[2] = S2DPlaneFrom8(idx, v);
        }
        return;
    }
    for (int i = 0; i < 256; i++)
    {
        s32 finalX, finalY;
        if (mosaic)
        {
            int im = CurBGXMosaicTable[i];
            finalX = rotX - (im * rotA);
            finalY = rotY - (im * rotC);
        }
        else
        {
            finalX = rotX;
            finalY = rotY;
        }
        u16 val = 0;
        if (!(finalX & ofxmask) && !(finalY & ofymask))
        {
            const u32 color = bgvram[((((finalY & ymask) >> 8) << yshift) + ((finalX & xmask) >> 8)) & bgvrammask];
            if (color) val = 0x8000 | color;
        }
        v[i] = val;
        any |= val;
        rotX += rotA;
        rotY += rotC;
    }
    PlaneAny[2] = any != 0;
}
#endif // S2D_PLANES


void SoftRenderer2D::DrawPixel(u32* dst, u16 color, u32 flag)
{
    u8 r = (color & 0x001F) << 1;
    u8 g = ((color & 0x03E0) >> 4) | ((color & 0x8000) >> 15);
    u8 b = (color & 0x7C00) >> 9;

    *(dst+256) = *dst;
    *dst = r | (g << 8) | (b << 16) | flag;
}

// Hybrid descriptor composite (scalar). Same decisions as ColorComposite, with the 3D
// pixel left symbolic: the GPU merge (or HybridResolvePixel) applies it per Nx pixel.
void SoftRenderer2D::HybridCompositeLine(u32* dst)
{
    LastLineHas3D = true;
    const u32 blendCnt = GPU2D.BlendCnt;
#if defined(LITEV_HYB_COMPOSITE_VEC) && defined(LITEV_HYB_COMPOSITE_NEON) && defined(LITEV_SOFT2D_NEON) && defined(__aarch64__)
    if (HybCompVec)
    {
        HybridCompositeLineVec(dst);
        return;
    }
#endif
#if defined(LITEV_HYB_COMPOSITE_NEON) && defined(LITEV_SOFT2D_NEON) && defined(__aarch64__)
    // The ColorComposite each pixel needs, a whole line at a time with the bit-exact NEON
    // compositor. A pixel needs exactly one: of the two layers under the 3D when the 3D is on
    // top, else of its own two layers. So pick each pixel's inputs, then composite once.
    alignas(16) u32 ccIn[512], cc[256];
    for (int i = 0; i < 256; i++)
    {
        const bool top3D = (BGOBJLine[i] >> 24 & 0xC0) == 0x40;
        ccIn[i]       = top3D ? Under3D[0][i] : BGOBJLine[i];
        ccIn[256 + i] = top3D ? Under3D[1][i] : BGOBJLine[256 + i];
    }
    GPU2DNeon::ColorCompositeLine(cc, ccIn, WindowMask, blendCnt, GPU2D.EVA, GPU2D.EVB, GPU2D.EVY);
#define CC_TOP(i) cc[i]
#define CC_UNDER(i) cc[i]
#else
#define CC_TOP(i) ColorComposite(i, BGOBJLine[i], BGOBJLine[256+i])
#define CC_UNDER(i) ColorComposite(i, Under3D[0][i], Under3D[1][i])
#endif
    for (int i = 0; i < 256; i++)
    {
        u32 val1 = BGOBJLine[i];
        u32 val2 = BGOBJLine[256+i];
        u32 flag1 = val1 >> 24;
        u32 flag2 = val2 >> 24;

        if ((flag1 & 0xC0) == 0x40)
        {
            // 3D on top. Under it: Under3D[0] (blend partner), then Under3D[1].
            u32 u1 = Under3D[0][i];
            u32 f2 = u1 >> 24;
            u32 target2;
            if      (f2 & 0x80) target2 = 0x1000;
            else if (f2 & 0x40) target2 = 0x0100;
            else                target2 = f2 << 8;

            u32 mode;
            if (blendCnt & target2)
                mode = 4;   // 3D alpha blend (no window colour-effect check)
            else
            {
                mode = 0;
                if ((blendCnt & 0x0001) && (WindowMask[i] & 0x20))
                {
                    mode = (blendCnt >> 6) & 0x3;
                    if (mode == 1) mode = 0;   // alpha blend needs a target-2 layer below
                }
            }
            dst[i]     = (u1 & 0xFFFFFF) | (mode << 29) | (((u32)GPU2D.EVY & 0x1F) << 24);
            dst[256+i] = CC_UNDER(i) & 0xFFFFFF;
        }
        else if ((flag2 & 0xC0) == 0x40)
        {
            // 3D directly under val1. If the 3D pixel is transparent, val1 sits on the
            // layer that was under the 3D (Under3D[0]).
            u32 under = Under3D[0][i];
            u32 eva = 16, evb = 0;
            bool blend = false;
            if ((flag1 & 0x80) && (blendCnt & 0x0100))
            {
                // semi-transparent / bitmap sprite over the 3D
                blend = true;
                if (flag1 & 0x40) { eva = flag1 & 0x1F; evb = 16 - eva; }
                else              { eva = GPU2D.EVA;    evb = GPU2D.EVB; }
            }
            else if (!(flag1 & 0x80))
            {
                u32 t1 = (flag1 & 0x40) ? 0x01 : flag1;
                if ((blendCnt & t1) && (WindowMask[i] & 0x20) &&
                    ((blendCnt >> 6) & 0x3) == 1 && (blendCnt & 0x0100))
                {
                    blend = true;
                    eva = GPU2D.EVA; evb = GPU2D.EVB;
                }
            }
            // no blend: the 3D only matters through the fallback, so blend 16:0 with
            // val1 already carrying any brightness effect
            dst[i]     = ((blend ? val1 : CC_TOP(i)) & 0xFFFFFF) | (1u << 29) | ((eva & 0x1F) << 24);
            dst[256+i] = (ColorComposite(i, val1, under) & 0xFFFFFF) | ((evb & 0x1F) << 24);
        }
        else
        {
            dst[i]     = (CC_TOP(i) & 0xFFFFFF) | (7u << 29);
        }
    }
#undef CC_TOP
#undef CC_UNDER
}

#if defined(LITEV_HYB_COMPOSITE_VEC) && defined(LITEV_HYB_COMPOSITE_NEON) && defined(LITEV_SOFT2D_NEON) && defined(__aarch64__)
// LITEV_HYB_COMPOSITE_VEC: HybridCompositeLine with every step 16 pixels at a time (same words:
// a pixel without the 3D keeps its old dst[256+i], as the scalar loop leaves it). Pixel kinds:
// 3D on top (descriptor of the layers under it), 3D directly under the top layer, no 3D.
void SoftRenderer2D::HybridCompositeLineVec(u32* dst)
{
    const u32 bc = GPU2D.BlendCnt;
    const u32 EVA = GPU2D.EVA, EVB = GPU2D.EVB, EVY = GPU2D.EVY;
    alignas(16) u32 ccIn[512], cc[256];
    // the ColorComposite each pixel needs: of the two layers under the 3D when the 3D is on top,
    // else of its own two layers; and whether any pixel has the 3D directly under its top layer
    const uint32x4_t cC0 = vdupq_n_u32(0xC0000000), c40 = vdupq_n_u32(0x40000000);
    uint32x4_t anyUnder = vdupq_n_u32(0);
    for (u32 i = 0; i < 256; i += 4)
    {
        const uint32x4_t v1 = vld1q_u32(BGOBJLine + i), v2 = vld1q_u32(BGOBJLine + 256 + i);
        const uint32x4_t top3D = vceqq_u32(vandq_u32(v1, cC0), c40);
        anyUnder = vorrq_u32(anyUnder, vbicq_u32(vceqq_u32(vandq_u32(v2, cC0), c40), top3D));
        vst1q_u32(ccIn + i, vbslq_u32(top3D, vld1q_u32(Under3D[0] + i), v1));
        vst1q_u32(ccIn + 256 + i, vbslq_u32(top3D, vld1q_u32(Under3D[1] + i), v2));
    }
    GPU2DNeon::ColorCompositeLine(cc, ccIn, WindowMask, bc, EVA, EVB, EVY);
    // 3D under the top layer: ColorComposite(top, layer under the 3D) for dst[256+i]
    alignas(16) u32 cc2[256];
    const bool under = vmaxvq_u32(anyUnder) != 0;
    if (under)
    {
        memcpy(ccIn, BGOBJLine, 256 * 4);
        memcpy(ccIn + 256, Under3D[0], 256 * 4);
        GPU2DNeon::ColorCompositeLine(cc2, ccIn, WindowMask, bc, EVA, EVB, EVY);
    }

    const uint8x16_t k80 = vdupq_n_u8(0x80), k40 = vdupq_n_u8(0x40), kC0 = vdupq_n_u8(0xC0), k20 = vdupq_n_u8(0x20), k1F = vdupq_n_u8(0x1F);
    const uint8x16_t z = vdupq_n_u8(0);
    // 3D on top: mode 4 if the layer under it is a second target (OBJ bit 12, 3D bit 8, else its
    // flag << 8), else the BLDCNT brightness effect (window bit 0x20, first-target bit 0)
    const uint8x16_t hitObj = vdupq_n_u8((bc & 0x1000) ? 0xFF : 0), hit3D = vdupq_n_u8((bc & 0x0100) ? 0xFF : 0);
    const uint8x16_t bcHi = vdupq_n_u8((u8)(bc >> 8)), bcLo = vdupq_n_u8((u8)bc);
    u32 fxMode = (bc >> 6) & 3; if (fxMode == 1) fxMode = 0;
    const uint8x16_t fxOn = vdupq_n_u8((bc & 1) ? 0xFF : 0);
    const uint8x16_t topByte4 = vdupq_n_u8((u8)((4 << 5) | (EVY & 0x1F)));
    const uint8x16_t topByteFx = vdupq_n_u8((u8)((fxMode << 5) | (EVY & 0x1F)));
    // 3D under the top layer: blend with the top's own alpha (semi-transparent / bitmap sprite) or
    // the BLDCNT alpha blend
    const uint8x16_t objBlend = vdupq_n_u8((bc & 0x0100) ? 0xFF : 0);
    const uint8x16_t bgBlend = vdupq_n_u8((((bc >> 6) & 3) == 1 && (bc & 0x0100)) ? 0xFF : 0);
    const uint8x16_t vEVA = vdupq_n_u8((u8)EVA), vEVB = vdupq_n_u8((u8)EVB), k16 = vdupq_n_u8(16);
    const uint8x16_t k7 = vdupq_n_u8(7 << 5), k1 = vdupq_n_u8(1 << 5);
    for (u32 i = 0; i < 256; i += 16)
    {
        const uint8x16x4_t V1 = vld4q_u8(reinterpret_cast<const u8*>(BGOBJLine + i));
        const uint8x16x4_t V2 = vld4q_u8(reinterpret_cast<const u8*>(BGOBJLine + 256 + i));
        const uint8x16x4_t U = vld4q_u8(reinterpret_cast<const u8*>(Under3D[0] + i));
        const uint8x16x4_t C = vld4q_u8(reinterpret_cast<const u8*>(cc + i));
        const uint8x16_t win = vld1q_u8(WindowMask + i);
        const uint8x16_t f1 = V1.val[3], f2 = V2.val[3], fu = U.val[3];
        const uint8x16_t top3D = vceqq_u8(vandq_u8(f1, kC0), k40);
        const uint8x16_t und3D = vbicq_u8(vceqq_u8(vandq_u8(f2, kC0), k40), top3D);
        const uint8x16_t winFx = vtstq_u8(win, k20);

        // 3D on top
        const uint8x16_t hit = vbslq_u8(vtstq_u8(fu, k80), hitObj, vbslq_u8(vtstq_u8(fu, k40), hit3D, vtstq_u8(fu, bcHi)));
        const uint8x16_t byteTop = vbslq_u8(hit, topByte4, vbslq_u8(vandq_u8(fxOn, winFx), topByteFx, vandq_u8(topByteFx, k1F)));

        // 3D under the top layer
        const uint8x16_t s80 = vtstq_u8(f1, k80), s40 = vtstq_u8(f1, k40);
        const uint8x16_t blendA = vandq_u8(s80, objBlend);
        const uint8x16_t blendB = vandq_u8(vbicq_u8(vandq_u8(vtstq_u8(f1, bcLo), winFx), s80), bgBlend);
        const uint8x16_t ownA = vandq_u8(f1, k1F);
        const uint8x16_t evaA = vbslq_u8(s40, ownA, vEVA), evbA = vbslq_u8(s40, vsubq_u8(k16, ownA), vEVB);
        const uint8x16_t eva = vbslq_u8(blendA, evaA, vbslq_u8(blendB, vEVA, k16));
        const uint8x16_t evb = vbslq_u8(blendA, evbA, vbslq_u8(blendB, vEVB, z));
        const uint8x16_t blend = vorrq_u8(blendA, blendB);

        uint8x16x4_t o;
        for (int c = 0; c < 3; c++)
            o.val[c] = vbslq_u8(top3D, U.val[c], vbslq_u8(vandq_u8(und3D, blend), V1.val[c], C.val[c]));
        o.val[3] = vbslq_u8(top3D, byteTop, vbslq_u8(und3D, vorrq_u8(k1, vandq_u8(eva, k1F)), k7));
        vst4q_u8(reinterpret_cast<u8*>(dst + i), o);

        // dst[256+i]: the composite under the 3D (3D on top), ColorComposite(top, under) | EVB
        // (3D under), else unchanged. The old words are loaded only if a pixel keeps its own: the
        // descriptor buffer is cold (a load miss stalls the in-order A55; stores don't).
        const uint8x16_t has3D = vorrq_u8(top3D, und3D);
        uint8x16x4_t q;
        if (vminvq_u8(has3D)) q.val[0] = q.val[1] = q.val[2] = q.val[3] = z;
        else if (!vmaxvq_u8(has3D)) continue;
        else q = vld4q_u8(reinterpret_cast<const u8*>(dst + 256 + i));
        if (under)
        {
            const uint8x16x4_t C2 = vld4q_u8(reinterpret_cast<const u8*>(cc2 + i));
            for (int c = 0; c < 3; c++) q.val[c] = vbslq_u8(top3D, C.val[c], vbslq_u8(und3D, C2.val[c], q.val[c]));
        }
        else
            for (int c = 0; c < 3; c++) q.val[c] = vbslq_u8(top3D, C.val[c], q.val[c]);
        q.val[3] = vbslq_u8(top3D, z, vbslq_u8(und3D, vandq_u8(evb, k1F), q.val[3]));
        vst4q_u8(reinterpret_cast<u8*>(dst + 256 + i), q);
    }
}
#endif

// CPU version of the hybrid merge (HybridMergeFS.glsl) for one native pixel: c3d is the
// 3D pixel in software format (6-bit RGB, 5-bit alpha in bits 24-28). Used for display
// capture in the hybrid renderer and for the descriptor self-check.
u32 SoftRenderer2D::HybridResolvePixel(u32 val1, u32 val2, u32 c3d)
{
    const u32 mode = val1 >> 29, ev1 = (val1 >> 24) & 0x1F, ev2 = (val2 >> 24) & 0x1F;
    const u32 rgb1 = val1 & 0xFFFFFF;
    if (mode == 7) return rgb1 | 0xFF000000;
    if ((c3d >> 24) == 0) return (val2 & 0xFFFFFF) | 0xFF000000;
    switch (mode)
    {
    case 4: return ColorBlend5(c3d, rgb1);
    case 1: return ColorBlend4(rgb1, c3d, ev1, ev2);
    case 2: return ColorBrightnessUp(c3d, ev1, 0x8);
    case 3: return ColorBrightnessDown(c3d, ev1, 0x7);
    default: return c3d | 0x40000000;
    }
}

void SoftRenderer2D::DrawBG_3D()
{
    if (HybridDesc)
    {
        // placeholder (flag 0x40, alpha 0) where the window allows 3D; remember the two
        // layers under it for the descriptor
        for (int i = 0; i < 256; i++)
        {
            if (!(WindowMask[i] & 0x01)) continue;
            Under3D[0][i] = BGOBJLine[i];
            Under3D[1][i] = BGOBJLine[256+i];
            BGOBJLine[i+256] = BGOBJLine[i];
            BGOBJLine[i] = 0x40000000;
        }
        return;
    }

#ifdef LITEV_SOFT2D_THREADED
    const u32* out3d = Cur3DLine;
#else
    const u32* out3d = Parent.Output3D;
#endif
#if defined(LITEV_SOFT2D_BG3DNEON) && defined(__ARM_NEON)
    GPU2DNeon::DrawBG3DLine(BGOBJLine, out3d, WindowMask);
#else
    for (int i = 0; i < 256; i++)
    {
        u32 c = out3d[i];

        if ((c >> 24) == 0) continue;
        if (!(WindowMask[i] & 0x01)) continue;

        BGOBJLine[i+256] = BGOBJLine[i];
        BGOBJLine[i] = c | 0x40000000;
    }
#endif
}

#ifdef LITEV_SOFT2D_BITMASK
// LITEV_SOFT2D_BITMASK: the extended-palette slot base looked up once per layer line, not per
// tile/pixel (GetBGExtPal is an out-of-line call): slot base + palette * 256 entries.
#define S2D_EXTPAL(slot, p) (extbase + ((u32)(p) << 8))
#else
#define S2D_EXTPAL(slot, p) GPU2D.GetBGExtPal(slot, p)
#endif

template<bool mosaic>
void SoftRenderer2D::DrawBG_Text(u32 line, u32 bgnum)
{
    // workaround for backgrounds missing on aarch64 with lto build
    asm volatile ("" : : : "memory");

    u16 bgcnt = GPU2D.BGCnt[bgnum];

    u32 tilesetaddr, tilemapaddr;
    u16* pal;
    u32 extpal, extpalslot;

    u16 xoff = GPU2D.BGXPos[bgnum];
    u16 yoff = GPU2D.BGYPos[bgnum];

    if (bgcnt & (1<<6))
        yoff += GPU2D.BGMosaicLine;
    else
        yoff += line;
    /*u16 yoff = GPU2D.BGYPos[bgnum] + line;

    if (bgcnt & (1<<6))
    {
        // vertical mosaic
        yoff -= GPU2D.BGMosaicY;
    }*/

    u32 widexmask = (bgcnt & (1<<14)) ? 0x100 : 0;

    extpal = (GPU2D.DispCnt & (1<<30));
    if (extpal) extpalslot = ((bgnum<2) && (bgcnt&0x2000)) ? (2+bgnum) : bgnum;
#ifdef LITEV_SOFT2D_BITMASK
    u16* const extbase = extpal ? GPU2D.GetBGExtPal(extpalslot, 0) : nullptr;
#endif

    u8* bgvram;
    u32 bgvrammask;
    GPU2D.GetBGVRAM(bgvram, bgvrammask);
    if (GPU2D.Num)
    {
        tilesetaddr = ((bgcnt & 0x003C) << 12);
        tilemapaddr = ((bgcnt & 0x1F00) << 3);

        pal = (u16*)&CurPalette[0x400];
    }
    else
    {
        tilesetaddr = ((GPU2D.DispCnt & 0x07000000) >> 8) + ((bgcnt & 0x003C) << 12);
        tilemapaddr = ((GPU2D.DispCnt & 0x38000000) >> 11) + ((bgcnt & 0x1F00) << 3);

        pal = (u16*)&CurPalette[0];
    }

    // adjust Y position in tilemap
    if (bgcnt & (1<<15))
    {
        tilemapaddr += ((yoff & 0x1F8) << 3);
        if (bgcnt & (1<<14))
            tilemapaddr += ((yoff & 0x100) << 3);
    }
    else
        tilemapaddr += ((yoff & 0xF8) << 3);

#ifdef LITEV_SOFT2D_BGSPAN
    if constexpr (!mosaic)
    {
        // A tile at a time: one tilemap entry + one tile-row load per 8 pixels, and a blank tile
        // row (all colour 0) skipped without touching its pixels. Same pixels as the loop below:
        // the row is 4/8-byte aligned and the VRAM mask is a power of two minus one, so the
        // per-pixel masked address is the masked row address + the byte offset.
        // Inspired by DSperate's text BG row gather with blank-tile skip (https://github.com/
        // beebono/DSperate, GPL-3.0-or-later, commit f5fa35e, src/core/gpu/engine2d.cpp).
        const u32 bgbit = 1u << bgnum, flag = 0x01000000u << bgnum, yrow = yoff & 7;
        const bool c256 = bgcnt & (1<<7);
        const bool nowin = !(GPU2D.DispCnt & 0xE000);   // WindowMask is all 0xFF (DrawScanline_BGOBJ)
#ifdef LITEV_SOFT2D_PALCONV
        const u32* const cv = CurPalConv ? CurPalConv + (GPU2D.Num ? 0x200 : 0) : nullptr;
#endif
        for (u32 i = 0; i < 256; )
        {
            const u32 xpos = (u16)(xoff + i);
            const u32 tx = xpos & 7;
            const u32 n = std::min<u32>(8 - tx, 256 - i);
            const u16 tile = *(u16*)&bgvram[(tilemapaddr + ((xpos & 0xF8) >> 2) + ((xpos & widexmask) << 3)) & bgvrammask];
            const u32 row = (tile & (1<<11)) ? (7 - yrow) : yrow;
            const u32 hx = (tile & (1<<10)) ? 7 : 0;   // tilexoff = x ^ 7 when flipped
            if (c256)
            {
                u64 px; memcpy(&px, &bgvram[(tilesetaddr + ((tile & 0x03FF) << 6) + (row << 3)) & bgvrammask], 8);
                if (px)
                {
#ifdef LITEV_SOFT2D_PALCONV
                    if (!extpal && cv)
                    {
                        if (n == 8 && nowin && !((px - 0x0101010101010101ull) & ~px & 0x8080808080808080ull))
                        {
                            // a whole opaque tile row: no per-pixel tests
                            u32* d = &BGOBJLine[i];
                            for (u32 k = 0; k < 8; k++) { d[256+k] = d[k]; d[k] = cv[(px >> ((k ^ hx) * 8)) & 0xFF] | flag; }
                            i += 8;
                            continue;
                        }
                        for (u32 k = 0; k < n; k++)
                        {
                            const u32 color = (px >> (((tx + k) ^ hx) * 8)) & 0xFF;
                            if (color && (nowin || (WindowMask[i+k] & bgbit)))
                                DrawPixelConv(&BGOBJLine[i+k], cv[color] | flag);
                        }
                        i += n;
                        continue;
                    }
#endif
                    const u16* cp = extpal ? S2D_EXTPAL(extpalslot, tile>>12) : pal;
                    for (u32 k = 0; k < n; k++)
                    {
                        const u32 color = (px >> (((tx + k) ^ hx) * 8)) & 0xFF;
                        if (color && (nowin || (WindowMask[i+k] & bgbit)))
                            DrawPixel(&BGOBJLine[i+k], cp[color], flag);
                    }
                }
            }
            else
            {
                u32 px; memcpy(&px, &bgvram[(tilesetaddr + ((tile & 0x03FF) << 5) + (row << 2)) & bgvrammask], 4);
                if (px)
                {
#ifdef LITEV_SOFT2D_PALCONV
                    if (cv)
                    {
                        const u32* const cc = cv + ((tile & 0xF000) >> 8);
                        // no zero nibble: a whole opaque tile row, no per-pixel tests
                        const u32 lo = px & 0x0F0F0F0F, hi = (px >> 4) & 0x0F0F0F0F;
                        if (n == 8 && nowin && !((lo - 0x01010101) & ~lo & 0x80808080) && !((hi - 0x01010101) & ~hi & 0x80808080))
                        {
                            u32* d = &BGOBJLine[i];
                            for (u32 k = 0; k < 8; k++) { d[256+k] = d[k]; d[k] = cc[(px >> ((k ^ hx) * 4)) & 0xF] | flag; }
                            i += 8;
                            continue;
                        }
                        for (u32 k = 0; k < n; k++)
                        {
                            const u32 color = (px >> (((tx + k) ^ hx) * 4)) & 0xF;
                            if (color && (nowin || (WindowMask[i+k] & bgbit)))
                                DrawPixelConv(&BGOBJLine[i+k], cc[color] | flag);
                        }
                        i += n;
                        continue;
                    }
#endif
                    const u16* cp = pal + ((tile & 0xF000) >> 8);
                    for (u32 k = 0; k < n; k++)
                    {
                        const u32 color = (px >> (((tx + k) ^ hx) * 4)) & 0xF;
                        if (color && (nowin || (WindowMask[i+k] & bgbit)))
                            DrawPixel(&BGOBJLine[i+k], cp[color], flag);
                    }
                }
            }
            i += n;
        }
        return;
    }

#endif
    u16 curtile;
    u16* curpal;
    u32 pixelsaddr;
    u8 color;
    u32 lastxpos;

    if (bgcnt & (1<<7))
    {
        // 256-color

        // preload shit as needed
        if ((xoff & 0x7) || mosaic)
        {
            curtile = *(u16*)&bgvram[(tilemapaddr + ((xoff & 0xF8) >> 2) + ((xoff & widexmask) << 3)) & bgvrammask];

            if (extpal) curpal = S2D_EXTPAL(extpalslot, curtile>>12);
            else        curpal = pal;

            pixelsaddr = tilesetaddr + ((curtile & 0x03FF) << 6)
                                     + (((curtile & (1<<11)) ? (7-(yoff&0x7)) : (yoff&0x7)) << 3);
        }

        if (mosaic) lastxpos = xoff;

        for (int i = 0; i < 256; i++)
        {
            u32 xpos;
            if (mosaic) xpos = xoff - CurBGXMosaicTable[i];
            else        xpos = xoff;

            if ((!mosaic && (!(xpos & 0x7))) ||
                (mosaic && ((xpos >> 3) != (lastxpos >> 3))))
            {
                // load a new tile
                curtile = *(u16*)&bgvram[(tilemapaddr + ((xpos & 0xF8) >> 2) + ((xpos & widexmask) << 3)) & bgvrammask];

                if (extpal) curpal = S2D_EXTPAL(extpalslot, curtile>>12);
                else        curpal = pal;

                pixelsaddr = tilesetaddr + ((curtile & 0x03FF) << 6)
                                         + (((curtile & (1<<11)) ? (7-(yoff&0x7)) : (yoff&0x7)) << 3);

                if (mosaic) lastxpos = xpos;
            }

            // draw pixel
            if (WindowMask[i] & (1<<bgnum))
            {
                u32 tilexoff = (curtile & (1<<10)) ? (7-(xpos&0x7)) : (xpos&0x7);
                color = bgvram[(pixelsaddr + tilexoff) & bgvrammask];

                if (color)
                    DrawPixel(&BGOBJLine[i], curpal[color], 0x01000000<<bgnum);
            }

            xoff++;
        }
    }
    else
    {
        // 16-color

        // preload shit as needed
        if ((xoff & 0x7) || mosaic)
        {
            curtile = *(u16*)&bgvram[((tilemapaddr + ((xoff & 0xF8) >> 2) + ((xoff & widexmask) << 3))) & bgvrammask];
            curpal = pal + ((curtile & 0xF000) >> 8);
            pixelsaddr = tilesetaddr + ((curtile & 0x03FF) << 5)
                                     + (((curtile & (1<<11)) ? (7-(yoff&0x7)) : (yoff&0x7)) << 2);
        }

        if (mosaic) lastxpos = xoff;

        for (int i = 0; i < 256; i++)
        {
            u32 xpos;
            if (mosaic) xpos = xoff - CurBGXMosaicTable[i];
            else        xpos = xoff;

            if ((!mosaic && (!(xpos & 0x7))) ||
                (mosaic && ((xpos >> 3) != (lastxpos >> 3))))
            {
                // load a new tile
                curtile = *(u16*)&bgvram[(tilemapaddr + ((xpos & 0xF8) >> 2) + ((xpos & widexmask) << 3)) & bgvrammask];
                curpal = pal + ((curtile & 0xF000) >> 8);
                pixelsaddr = tilesetaddr + ((curtile & 0x03FF) << 5)
                                         + (((curtile & (1<<11)) ? (7-(yoff&0x7)) : (yoff&0x7)) << 2);

                if (mosaic) lastxpos = xpos;
            }

            // draw pixel
            if (WindowMask[i] & (1<<bgnum))
            {
                u32 tilexoff = (curtile & (1<<10)) ? (7-(xpos&0x7)) : (xpos&0x7);
                if (tilexoff & 0x1)
                {
                    color = bgvram[(pixelsaddr + (tilexoff >> 1)) & bgvrammask] >> 4;
                }
                else
                {
                    color = bgvram[(pixelsaddr + (tilexoff >> 1)) & bgvrammask] & 0x0F;
                }

                if (color)
                    DrawPixel(&BGOBJLine[i], curpal[color], 0x01000000<<bgnum);
            }

            xoff++;
        }
    }
}

template<bool mosaic>
void SoftRenderer2D::DrawBG_Affine(u32 line, u32 bgnum)
{
    u16 bgcnt = GPU2D.BGCnt[bgnum];

    u32 tilesetaddr, tilemapaddr;
    u16* pal;

    u32 coordmask;
    u32 yshift;
    switch ((bgcnt >> 14) & 0x3)
    {
        case 0: coordmask = 0x07800; yshift = 7; break;
        case 1: coordmask = 0x0F800; yshift = 8; break;
        case 2: coordmask = 0x1F800; yshift = 9; break;
        case 3: coordmask = 0x3F800; yshift = 10; break;
    }

    u32 overflowmask;
    if (bgcnt & (1<<13)) overflowmask = 0;
    else                 overflowmask = ~(coordmask | 0x7FF);

    s16 rotA = GPU2D.BGRotA[bgnum-2];
    s16 rotC = GPU2D.BGRotC[bgnum-2];

    s32 rotX = GPU2D.BGXRefInternal[bgnum-2];
    s32 rotY = GPU2D.BGYRefInternal[bgnum-2];

    u8* bgvram;
    u32 bgvrammask;
    GPU2D.GetBGVRAM(bgvram, bgvrammask);

    if (GPU2D.Num)
    {
        tilesetaddr = ((bgcnt & 0x003C) << 12);
        tilemapaddr = ((bgcnt & 0x1F00) << 3);

        pal = (u16*)&CurPalette[0x400];
    }
    else
    {
        tilesetaddr = ((GPU2D.DispCnt & 0x07000000) >> 8) + ((bgcnt & 0x003C) << 12);
        tilemapaddr = ((GPU2D.DispCnt & 0x38000000) >> 11) + ((bgcnt & 0x1F00) << 3);

        pal = (u16*)&CurPalette[0];
    }

    u16 curtile;
    u8 color;

    yshift -= 3;

    for (int i = 0; i < 256; i++)
    {
        if (WindowMask[i] & (1<<bgnum))
        {
            s32 finalX, finalY;
            if (mosaic)
            {
                int im = CurBGXMosaicTable[i];
                finalX = rotX - (im * rotA);
                finalY = rotY - (im * rotC);
            }
            else
            {
                finalX = rotX;
                finalY = rotY;
            }

            if ((!((finalX|finalY) & overflowmask)))
            {
                curtile = bgvram[(tilemapaddr + ((((finalY & coordmask) >> 11) << yshift) + ((finalX & coordmask) >> 11))) & bgvrammask];

                // draw pixel
                u32 tilexoff = (finalX >> 8) & 0x7;
                u32 tileyoff = (finalY >> 8) & 0x7;

                color = bgvram[(tilesetaddr + (curtile << 6) + (tileyoff << 3) + tilexoff) & bgvrammask];

                if (color)
                    DrawPixel(&BGOBJLine[i], pal[color], 0x01000000<<bgnum);
            }
        }

        rotX += rotA;
        rotY += rotC;
    }
}

template<bool mosaic>
void SoftRenderer2D::DrawBG_Extended(u32 line, u32 bgnum)
{
    u16 bgcnt = GPU2D.BGCnt[bgnum];

    u32 tilesetaddr, tilemapaddr;
    u16* pal;
    u32 extpal;

    u8* bgvram;
    u32 bgvrammask;
    GPU2D.GetBGVRAM(bgvram, bgvrammask);

    extpal = (GPU2D.DispCnt & (1<<30));
#ifdef LITEV_SOFT2D_BITMASK
    u16* const extbase = extpal ? GPU2D.GetBGExtPal(bgnum, 0) : nullptr;
#endif

    s16 rotA = GPU2D.BGRotA[bgnum-2];
    s16 rotC = GPU2D.BGRotC[bgnum-2];

    s32 rotX = GPU2D.BGXRefInternal[bgnum-2];
    s32 rotY = GPU2D.BGYRefInternal[bgnum-2];

    if (bgcnt & (1<<7))
    {
        // bitmap modes

        u32 xmask, ymask;
        u32 yshift;
        switch ((bgcnt >> 14) & 0x3)
        {
            case 0: xmask = 0x07FFF; ymask = 0x07FFF; yshift = 7; break;
            case 1: xmask = 0x0FFFF; ymask = 0x0FFFF; yshift = 8; break;
            case 2: xmask = 0x1FFFF; ymask = 0x0FFFF; yshift = 9; break;
            case 3: xmask = 0x1FFFF; ymask = 0x1FFFF; yshift = 9; break;
        }

        u32 ofxmask, ofymask;
        if (bgcnt & (1<<13))
        {
            ofxmask = 0;
            ofymask = 0;
        }
        else
        {
            ofxmask = ~xmask;
            ofymask = ~ymask;
        }

        tilemapaddr = ((bgcnt & 0x1F00) << 6);

        if (bgcnt & (1<<2))
        {
            // direct color bitmap

            u16 color;

            for (int i = 0; i < 256; i++)
            {
                if (WindowMask[i] & (1<<bgnum))
                {
                    s32 finalX, finalY;
                    if (mosaic)
                    {
                        int im = CurBGXMosaicTable[i];
                        finalX = rotX - (im * rotA);
                        finalY = rotY - (im * rotC);
                    }
                    else
                    {
                        finalX = rotX;
                        finalY = rotY;
                    }

                    if (!(finalX & ofxmask) && !(finalY & ofymask))
                    {
                        color = *(u16*)&bgvram[(tilemapaddr + (((((finalY & ymask) >> 8) << yshift) + ((finalX & xmask) >> 8)) << 1)) & bgvrammask];

                        if (color & 0x8000)
                            DrawPixel(&BGOBJLine[i], color & 0x7FFF, 0x01000000<<bgnum);
                    }
                }

                rotX += rotA;
                rotY += rotC;
            }
        }
        else
        {
            // 256-color bitmap

            if (GPU2D.Num) pal = (u16*)&CurPalette[0x400];
            else           pal = (u16*)&CurPalette[0];

            u8 color;

            for (int i = 0; i < 256; i++)
            {
                if (WindowMask[i] & (1<<bgnum))
                {
                    s32 finalX, finalY;
                    if (mosaic)
                    {
                        int im = CurBGXMosaicTable[i];
                        finalX = rotX - (im * rotA);
                        finalY = rotY - (im * rotC);
                    }
                    else
                    {
                        finalX = rotX;
                        finalY = rotY;
                    }

                    if (!(finalX & ofxmask) && !(finalY & ofymask))
                    {
                        color = bgvram[(tilemapaddr + (((finalY & ymask) >> 8) << yshift) + ((finalX & xmask) >> 8)) & bgvrammask];

                        if (color)
                            DrawPixel(&BGOBJLine[i], pal[color], 0x01000000<<bgnum);
                    }
                }

                rotX += rotA;
                rotY += rotC;
            }
        }
    }
    else
    {
        // mixed affine/text mode

        u32 coordmask;
        u32 yshift;
        switch ((bgcnt >> 14) & 0x3)
        {
            case 0: coordmask = 0x07800; yshift = 7; break;
            case 1: coordmask = 0x0F800; yshift = 8; break;
            case 2: coordmask = 0x1F800; yshift = 9; break;
            case 3: coordmask = 0x3F800; yshift = 10; break;
        }

        u32 overflowmask;
        if (bgcnt & (1<<13)) overflowmask = 0;
        else                 overflowmask = ~(coordmask | 0x7FF);

        if (GPU2D.Num)
        {
            tilesetaddr = ((bgcnt & 0x003C) << 12);
            tilemapaddr = ((bgcnt & 0x1F00) << 3);

            pal = (u16*)&CurPalette[0x400];
        }
        else
        {
            tilesetaddr = ((GPU2D.DispCnt & 0x07000000) >> 8) + ((bgcnt & 0x003C) << 12);
            tilemapaddr = ((GPU2D.DispCnt & 0x38000000) >> 11) + ((bgcnt & 0x1F00) << 3);

            pal = (u16*)&CurPalette[0];
        }

        u16 curtile;
        u16* curpal;
        u8 color;

        yshift -= 3;
#ifdef LITEV_SOFT2D_BGSPAN
        u32 lastmapoff = ~0u;
#endif

        for (int i = 0; i < 256; i++)
        {
            if (WindowMask[i] & (1<<bgnum))
            {
                s32 finalX, finalY;
                if (mosaic)
                {
                    int im = CurBGXMosaicTable[i];
                    finalX = rotX - (im * rotA);
                    finalY = rotY - (im * rotC);
                }
                else
                {
                    finalX = rotX;
                    finalY = rotY;
                }

                if ((!((finalX|finalY) & overflowmask)))
                {
#ifdef LITEV_SOFT2D_BGSPAN
                    // consecutive pixels on the same map entry (scaled/rotated maps) reuse it
                    const u32 mapoff = (((((finalY & coordmask) >> 11) << yshift) + ((finalX & coordmask) >> 11)) << 1);
                    if (mapoff != lastmapoff)
                    {
                        lastmapoff = mapoff;
                        curtile = *(u16*)&bgvram[(tilemapaddr + mapoff) & bgvrammask];
                        if (extpal) curpal = S2D_EXTPAL(bgnum, curtile>>12);
                        else        curpal = pal;
                    }
#else
                    curtile = *(u16*)&bgvram[(tilemapaddr + (((((finalY & coordmask) >> 11) << yshift) + ((finalX & coordmask) >> 11)) << 1)) & bgvrammask];

                    if (extpal) curpal = S2D_EXTPAL(bgnum, curtile>>12);
                    else        curpal = pal;
#endif

                    // draw pixel
                    u32 tilexoff = (finalX >> 8) & 0x7;
                    u32 tileyoff = (finalY >> 8) & 0x7;

                    if (curtile & (1<<10)) tilexoff = 7-tilexoff;
                    if (curtile & (1<<11)) tileyoff = 7-tileyoff;

                    color = bgvram[(tilesetaddr + ((curtile & 0x03FF) << 6) + (tileyoff << 3) + tilexoff) & bgvrammask];

                    if (color)
                        DrawPixel(&BGOBJLine[i], curpal[color], 0x01000000<<bgnum);
                }
            }

            rotX += rotA;
            rotY += rotC;
        }
    }
}

template<bool mosaic>
void SoftRenderer2D::DrawBG_Large(u32 line) // BG is always BG2
{
    u16 bgcnt = GPU2D.BGCnt[2];

    u16* pal;

    // large BG sizes:
    // 0: 512x1024
    // 1: 1024x512
    // 2: 512x256
    // 3: 512x512
    u32 xmask, ymask;
    u32 yshift;
    switch ((bgcnt >> 14) & 0x3)
    {
        case 0: xmask = 0x1FFFF; ymask = 0x3FFFF; yshift = 9; break;
        case 1: xmask = 0x3FFFF; ymask = 0x1FFFF; yshift = 10; break;
        case 2: xmask = 0x1FFFF; ymask = 0x0FFFF; yshift = 9; break;
        case 3: xmask = 0x1FFFF; ymask = 0x1FFFF; yshift = 9; break;
    }

    u32 ofxmask, ofymask;
    if (bgcnt & (1<<13))
    {
        ofxmask = 0;
        ofymask = 0;
    }
    else
    {
        ofxmask = ~xmask;
        ofymask = ~ymask;
    }

    s16 rotA = GPU2D.BGRotA[0];
    s16 rotC = GPU2D.BGRotC[0];

    s32 rotX = GPU2D.BGXRefInternal[0];
    s32 rotY = GPU2D.BGYRefInternal[0];

    u8* bgvram;
    u32 bgvrammask;
    GPU2D.GetBGVRAM(bgvram, bgvrammask);

    // 256-color bitmap

    if (GPU2D.Num) pal = (u16*)&CurPalette[0x400];
    else           pal = (u16*)&CurPalette[0];

    u8 color;

    for (int i = 0; i < 256; i++)
    {
        if (WindowMask[i] & (1<<2))
        {
            s32 finalX, finalY;
            if (mosaic)
            {
                int im = CurBGXMosaicTable[i];
                finalX = rotX - (im * rotA);
                finalY = rotY - (im * rotC);
            }
            else
            {
                finalX = rotX;
                finalY = rotY;
            }

            if (!(finalX & ofxmask) && !(finalY & ofymask))
            {
                color = bgvram[((((finalY & ymask) >> 8) << yshift) + ((finalX & xmask) >> 8)) & bgvrammask];

                if (color)
                    DrawPixel(&BGOBJLine[i], pal[color], 0x01000000<<2);
            }
        }

        rotX += rotA;
        rotY += rotC;
    }
}


void SoftRenderer2D::ApplySpriteMosaicX()
{
    /*
     * apply X mosaic if needed
     * X mosaic for sprites is applied after all sprites are rendered
     *
     * rules:
     * pixels are processed from left to right
     * current pixel value is latched if:
     * - the X mosaic counter is 0
     * - the current pixel doesn't receive sprite mosaic
     * - the current pixel receives sprite mosaic and the previous one didn't, or vice versa
     * - the current BG-relative priority value is lower than the previous one
     */

    u8 mosw = GPU2D.OBJMosaicSize[0];
    if (mosw == 0) return;

    u8 mosx = 0;
    u32 latchcolor;
    for (int i = 0; i < 256; i++)
    {
        u32 curcolor = OBJLine[i];
        bool latch = false;

        if (mosx == 0)
            latch = true;
        else if (!(curcolor & OBJ_Mosaic))
            latch = true;
        else if (!(latchcolor & OBJ_Mosaic))
            latch = true;
        else if ((curcolor & OBJ_BGPrioMask) < (latchcolor & OBJ_BGPrioMask))
            latch = true;

        if (latch)
            latchcolor = curcolor;

        OBJLine[i] = latchcolor;

        if (mosx == mosw)
            mosx = 0;
        else
            mosx++;
    }
}

void SoftRenderer2D::InterleaveSprites(u32 prio)
{
    u32 attrmask = (prio << 16) | OBJ_IsOpaque;
    u16* pal = (u16*)&CurPalette[GPU2D.Num ? 0x600 : 0x200];
    u16* extpal = GPU2D.GetOBJExtPal();

#if defined(LITEV_SOFT2D_OBJNEON) && defined(__aarch64__)
    // NEON branchless reject-scan (LITEV_SOFT2D_OBJNEON). InterleaveSprites runs
    // once per BG priority level (up to 4x per scanline); at any given priority
    // most of the 256 pixels do NOT carry a matching opaque sprite, so the scalar
    // loop above burns the bulk of its time on the (OBJLine[i] & OpaPrioMask) !=
    // attrmask reject path. Here we test the OBJ-priority match 16 lanes at a time
    // and skip whole 16-pixel chunks that contain zero matches. Chunks that DO
    // contain a match fall through to the identical scalar body below (re-testing
    // both the prio mask and the window bit), so every drawn pixel — and the
    // per-pixel BGOBJLine/BGOBJLine+256 read-modify-write in DrawPixel — is
    // byte-identical to the scalar path. The palette gather (pal[]/extpal[]) has
    // no NEON equivalent on AArch64 (no gather), so only the reject scan is
    // vectorized. 256 is an exact multiple of 16 (no tail).
#ifdef LITEV_SOFT2D_PALCONV
    const u32* const cvo = CurPalConv ? CurPalConv + (GPU2D.Num ? 0x300 : 0x100) : nullptr;
#endif
    const uint32x4_t vOpaPrio = vdupq_n_u32(OBJ_OpaPrioMask);
    const uint32x4_t vAttr    = vdupq_n_u32(attrmask);
    for (u32 base = 0; base < 256; base += 16)
    {
        uint32x4_t o0 = vld1q_u32(&OBJLine[base + 0]);
        uint32x4_t o1 = vld1q_u32(&OBJLine[base + 4]);
        uint32x4_t o2 = vld1q_u32(&OBJLine[base + 8]);
        uint32x4_t o3 = vld1q_u32(&OBJLine[base + 12]);
        uint32x4_t m0 = vceqq_u32(vandq_u32(o0, vOpaPrio), vAttr);
        uint32x4_t m1 = vceqq_u32(vandq_u32(o1, vOpaPrio), vAttr);
        uint32x4_t m2 = vceqq_u32(vandq_u32(o2, vOpaPrio), vAttr);
        uint32x4_t m3 = vceqq_u32(vandq_u32(o3, vOpaPrio), vAttr);
        uint32x4_t any = vorrq_u32(vorrq_u32(m0, m1), vorrq_u32(m2, m3));
        if (vmaxvq_u32(any) == 0)
            continue; // no matching opaque sprite pixel in these 16 -> skip

        for (u32 i = base; i < base + 16; i++)
        {
            if ((OBJLine[i] & OBJ_OpaPrioMask) != attrmask)
                continue;
            if (!(WindowMask[i] & 0x10))
                continue;

            u16 color;
            u32 pixel = OBJLine[i];

#ifdef LITEV_SOFT2D_PALCONV
            if (cvo && (pixel & (OBJ_DirectColor | OBJ_StandardPal)) == OBJ_StandardPal)
            {
                DrawPixelConv(&BGOBJLine[i], cvo[pixel & 0xFF] | (pixel & 0xFF000000));
                continue;
            }
#endif
            if (pixel & OBJ_DirectColor)
                color = pixel & 0x7FFF;
            else if (pixel & OBJ_StandardPal)
                color = pal[pixel & 0xFF];
            else
                color = extpal[pixel & 0xFFF];

            DrawPixel(&BGOBJLine[i], color, pixel & 0xFF000000);
        }
    }
    return;
#endif

    for (u32 i = 0; i < 256; i++)
    {
        if ((OBJLine[i] & OBJ_OpaPrioMask) != attrmask)
            continue;
        if (!(WindowMask[i] & 0x10))
            continue;

        u16 color;
        u32 pixel = OBJLine[i];

        if (pixel & OBJ_DirectColor)
            color = pixel & 0x7FFF;
        else if (pixel & OBJ_StandardPal)
            color = pal[pixel & 0xFF];
        else
            color = extpal[pixel & 0xFFF];

        DrawPixel(&BGOBJLine[i], color, pixel & 0xFF000000);
    }
}

#define DoDrawSprite(type, ...) \
    do \
    { \
        if (iswin) \
        { \
            DrawSprite_##type<true>(__VA_ARGS__); \
        } \
        else \
        { \
            DrawSprite_##type<false>(__VA_ARGS__); \
        } \
    } while (0)

void SoftRenderer2D::DrawSprites(u32 line)
{
    // the OBJ buffers don't get updated at all if the 2D engine is disabled
    if (!GPU2D.Enabled)
        return;

#ifdef LITEV_SOFT2D_THREADED
    // A band unit (private GPU2D copy) rasters on the async 2D thread: the emu thread already
    // made the flat OBJ VRAM coherent at VBlank (SyncVRAM_OBJ). Doing it here raced the emu
    // thread's VRAM writes/dirty bits and pulled a later frame's OBJ VRAM into this raster.
    if (&GPU2D == &GPU.GPU2D_A || &GPU2D == &GPU.GPU2D_B)
#endif
    {
        if (GPU2D.Num == 0)
        {
            auto objDirty = GPU.VRAMDirty_AOBJ.DeriveState(GPU.VRAMMap_AOBJ, GPU);
            GPU.MakeVRAMFlat_AOBJCoherent(objDirty);
        }
        else
        {
            auto objDirty = GPU.VRAMDirty_BOBJ.DeriveState(GPU.VRAMMap_BOBJ, GPU);
            GPU.MakeVRAMFlat_BOBJCoherent(objDirty);
        }
    }

#ifdef S2D_OBJROW
    // the OBJ buffers are only written by a drawn sprite (counted in NumSprites): a line after one
    // with none is still clear
    if (NumSprites)
    {
        memset(OBJLine, 0, sizeof(OBJLine));
        memset(OBJWindow, 0, sizeof(OBJWindow));
    }
    NumSprites = 0;
#ifdef LITEV_SOFT2D_SPRLIST
    OBJPrioMask = 0;
#endif
#else
    NumSprites = 0;
#ifdef LITEV_SOFT2D_SPRLIST
    OBJPrioMask = 0;
#endif
    memset(OBJLine, 0, sizeof(OBJLine));
    memset(OBJWindow, 0, sizeof(OBJWindow));
#endif

    if (!GPU2D.OBJEnable)
        return;

#ifdef LITEV_SOFT2D_THREADED
    u16* oam = (u16*)&CurOAM[GPU2D.Num ? 0x400 : 0];
#else
    u16* oam = (u16*)&GPU.OAM[GPU2D.Num ? 0x400 : 0];
#endif

    const s32 spritewidth[16] =
    {
        8, 16, 8, 8,
        16, 32, 8, 8,
        32, 32, 16, 8,
        64, 64, 32, 8
    };
    const s32 spriteheight[16] =
    {
        8, 8, 16, 8,
        16, 8, 32, 8,
        32, 16, 32, 8,
        64, 32, 64, 8
    };

#ifdef LITEV_SOFT2D_SPRLIST
    // only the sprites whose box covers this line (lists rebuilt per OAM generation), in OAM order
    u64 cand[2] = {~0ull, ~0ull};
    if (CurOAMGen && line < 192)
    {
        if (SprListGen != *CurOAMGen) BuildSprLists(oam);
        cand[0] = SprLines[line][0]; cand[1] = SprLines[line][1];
    }
    for (int w = 0; w < 2; w++)
    for (u64 m = cand[w]; m; m &= m - 1)
    {
        const int sprnum = w * 64 + __builtin_ctzll(m);
#else
    for (int sprnum = 0; sprnum < 128; sprnum++)
    {
#endif
        u16* attrib = &oam[sprnum*4];

        u16 sprtype = (attrib[0] >> 8) & 0x3;
        if (sprtype == 2) // disabled
            continue;

        bool iswin = (((attrib[0] >> 10) & 0x3) == 2);

        u32 sizeparam = (attrib[0] >> 14) | ((attrib[1] & 0xC000) >> 12);
        s32 width = spritewidth[sizeparam];
        s32 height = spriteheight[sizeparam];
        s32 boundwidth = width;
        s32 boundheight = height;

        if (sprtype == 3) // double-size rotscale sprite
        {
            boundwidth <<= 1;
            boundheight <<= 1;
        }

        // TODO checkme (128-tall sprite overflow thing)
        s32 ypos = attrib[0] & 0xFF;
        if (((line - ypos) & 0xFF) >= boundheight)
            continue;

        s32 xpos = (s32)(attrib[1] << 23) >> 23;
        if (xpos <= -boundwidth)
            continue;

        if ((attrib[0] & (1<<12)) && (!iswin))
        {
            // adjust Y position for sprite mosaic
            // (sprite mosaic does not apply to OBJ-window sprites)
            // a ypos greater than the sprite height means we underflowed, due to OBJMosaicLine being
            // latched before the sprite's top, so we clamp it to 0
            ypos = (GPU2D.OBJMosaicLine - ypos) & 0xFF;
            if (ypos >= boundheight) ypos = 0;
        }
        else
            ypos = (line - ypos) & 0xFF;

        if (sprtype & 1)
            DoDrawSprite(Rotscale, sprnum, boundwidth, boundheight, width, height, xpos, ypos);
        else
            DoDrawSprite(Normal, sprnum, width, height, xpos, ypos);

        NumSprites++;
#ifdef LITEV_SOFT2D_SPRLIST
        if (!iswin) OBJPrioMask |= 1u << ((attrib[2] >> 10) & 3);
#endif
    }
}

#ifdef LITEV_SOFT2D_SPRLIST
// Per line, the sprites whose (bound) box covers it: the DrawSprites y test, done once per OAM
// generation instead of for 128 sprites on every line. Inspired by DSperate's sprite candidate
// lists (https://github.com/beebono/DSperate, GPL-3.0-or-later, commit f5fa35e,
// src/core/gpu/engine2d.cpp rebuild_sprite_lists).
void SoftRenderer2D::BuildSprLists(const u16* oam)
{
    static constexpr u8 sprheight[16] = { 8, 8, 16, 8, 16, 8, 32, 8, 32, 16, 32, 8, 64, 32, 64, 8 };
    memset(SprLines, 0, sizeof(SprLines));
    for (int sprnum = 0; sprnum < 128; sprnum++)
    {
        const u16* attrib = &oam[sprnum*4];
        const u32 sprtype = (attrib[0] >> 8) & 0x3;
        if (sprtype == 2) continue;
        u32 bh = sprheight[(attrib[0] >> 14) | ((attrib[1] & 0xC000) >> 12)];
        if (sprtype == 3) bh <<= 1;
        const u32 ypos = attrib[0] & 0xFF;
        for (u32 h = 0; h < bh; h++)
        {
            const u32 y = (ypos + h) & 0xFF;
            if (y < 192) SprLines[y][sprnum >> 6] |= 1ull << (sprnum & 63);
        }
    }
    SprListGen = *CurOAMGen;
}

#endif
template<bool window>
void SoftRenderer2D::DrawSpritePixel(int color, u32 pixelattr, s32 xpos)
{
    if (window)
    {
        if (color != -1)
            OBJWindow[xpos] = 1;
    }
    else
    {
        u32 oldpixel = OBJLine[xpos];
        bool oldisopaque = !!(oldpixel & OBJ_IsOpaque);
        bool newisopaque = (color != -1);
        bool priocheck = (pixelattr & OBJ_BGPrioMask) < (oldpixel & OBJ_BGPrioMask);

        if (newisopaque && (!oldisopaque || priocheck))
        {
            OBJLine[xpos] = color | pixelattr;
        }
        else if (!newisopaque && !oldisopaque)
        {
            OBJLine[xpos] &= ~(OBJ_Mosaic | OBJ_BGPrioMask);
            OBJLine[xpos] |= (pixelattr & (OBJ_IsSprite | OBJ_Mosaic | OBJ_BGPrioMask));
        }
    }
}

template<bool window>
void SoftRenderer2D::DrawSprite_Rotscale(u32 num, u32 boundwidth, u32 boundheight, u32 width, u32 height, s32 xpos, s32 ypos)
{
#ifdef LITEV_SOFT2D_THREADED
    u16* oam = (u16*)&CurOAM[GPU2D.Num ? 0x400 : 0];
#else
    u16* oam = (u16*)&GPU.OAM[GPU2D.Num ? 0x400 : 0];
#endif
    u16* attrib = &oam[num * 4];
    u16* rotparams = &oam[(((attrib[1] >> 9) & 0x1F) * 16) + 3];

    u32 pixelattr = ((attrib[2] & 0x0C00) << 6) | OBJ_IsSprite | OBJ_IsOpaque;
    u32 tilenum = attrib[2] & 0x03FF;
    u32 spritemode = window ? 0 : ((attrib[0] >> 10) & 0x3);

    u32 ytilefactor;

    u8* objvram;
    u32 objvrammask;
    GPU2D.GetOBJVRAM(objvram, objvrammask);

    s32 centerX = boundwidth >> 1;
    s32 centerY = boundheight >> 1;

    if ((attrib[0] & (1<<12)) && !window)
    {
        // apply Y mosaic
        pixelattr |= OBJ_Mosaic;
    }

    u32 xoff;
    if (xpos >= 0)
    {
        xoff = 0;
        if ((xpos+boundwidth) > 256)
            boundwidth = 256-xpos;
    }
    else
    {
        xoff = -xpos;
        xpos = 0;
    }

    s16 rotA = (s16)rotparams[0];
    s16 rotB = (s16)rotparams[4];
    s16 rotC = (s16)rotparams[8];
    s16 rotD = (s16)rotparams[12];

    s32 rotX = ((xoff-centerX) * rotA) + ((ypos-centerY) * rotB) + (width << 7);
    s32 rotY = ((xoff-centerX) * rotC) + ((ypos-centerY) * rotD) + (height << 7);

    width <<= 8;
    height <<= 8;

    u16 color = 0; // transparent in all cases

    if (spritemode == 3)
    {
        u32 alpha = attrib[2] >> 12;
        if (!alpha) return;
        alpha++;

        pixelattr |= (0xC0000000 | (alpha << 24));

        u32 pixelsaddr;
        if (GPU2D.DispCnt & 0x40)
        {
            if (GPU2D.DispCnt & 0x20)
            {
                // 'reserved'
                // draws nothing

                return;
            }
            else
            {
                pixelsaddr = tilenum << (7 + ((GPU2D.DispCnt >> 22) & 0x1));
                ytilefactor = ((width >> 8) * 2);
            }
        }
        else
        {
            if (GPU2D.DispCnt & 0x20)
            {
                pixelsaddr = ((tilenum & 0x01F) << 4) + ((tilenum & 0x3E0) << 7);
                ytilefactor = (256 * 2);
            }
            else
            {
                pixelsaddr = ((tilenum & 0x00F) << 4) + ((tilenum & 0x3F0) << 7);
                ytilefactor = (128 * 2);
            }
        }

        for (; xoff < boundwidth;)
        {
            if ((u32)rotX < width && (u32)rotY < height)
            {
                color = *(u16*)&objvram[(pixelsaddr + ((rotY >> 8) * ytilefactor) + ((rotX >> 8) << 1)) & objvrammask];

                DrawSpritePixel<window>((color&0x8000) ? color : -1, pixelattr, xpos);
            }

            rotX += rotA;
            rotY += rotC;
            xoff++;
            xpos++;
        }
    }
    else
    {
        u32 pixelsaddr = tilenum;
        if (GPU2D.DispCnt & (1<<4))
        {
            pixelsaddr <<= ((GPU2D.DispCnt >> 20) & 0x3);
            ytilefactor = (width >> 11) << ((attrib[0] & 0x2000) ? 1:0);
        }
        else
        {
            ytilefactor = 0x20;
        }

        if (spritemode == 1) pixelattr |= 0x80000000;
        else                 pixelattr |= 0x10000000;

        ytilefactor <<= 5;
        pixelsaddr <<= 5;

        if (attrib[0] & (1<<13))
        {
            // 256-color

            if (!window)
            {
                if (!(GPU2D.DispCnt & (1<<31)))
                    pixelattr |= OBJ_StandardPal;
                else
                    pixelattr |= ((attrib[2] & 0xF000) >> 4);
            }

            for (; xoff < boundwidth;)
            {
                if ((u32)rotX < width && (u32)rotY < height)
                {
                    color = objvram[(pixelsaddr + ((rotY>>11)*ytilefactor) + ((rotY&0x700)>>5) + ((rotX>>11)*64) + ((rotX&0x700)>>8)) & objvrammask];

                    DrawSpritePixel<window>(color ? color : -1, pixelattr, xpos);
                }

                rotX += rotA;
                rotY += rotC;
                xoff++;
                xpos++;
            }
        }
        else
        {
            // 16-color
            if (!window)
            {
                pixelattr |= OBJ_StandardPal;
                pixelattr |= ((attrib[2] & 0xF000) >> 8);
            }

            for (; xoff < boundwidth;)
            {
                if ((u32)rotX < width && (u32)rotY < height)
                {
                    color = objvram[(pixelsaddr + ((rotY>>11)*ytilefactor) + ((rotY&0x700)>>6) + ((rotX>>11)*32) + ((rotX&0x700)>>9)) & objvrammask];
                    if (rotX & 0x100)
                        color >>= 4;
                    else
                        color &= 0x0F;

                    DrawSpritePixel<window>(color ? color : -1, pixelattr, xpos);
                }

                rotX += rotA;
                rotY += rotC;
                xoff++;
                xpos++;
            }
        }
    }
}

#ifdef S2D_OBJROW
// texels [xoff, xend) of a decoded sprite row (mirrored when hflip) plotted from screen x xpos
void SoftRenderer2D::S2DPlotSpriteRow(const u8* tex, u8* row, u32 width, bool hflip, u32 xoff, u32 xend, s32 xpos, u32 pixelattr)
{
    const u32 cnt = xend - xoff;
    const u8* r = tex + xoff;
    if (hflip)
    {
        for (u32 k = 0; k < cnt; k++) row[k] = tex[width - 1 - (xoff + k)];
        r = row;
    }
    for (u32 j = S2DPlanes::PlotObjRow(r, cnt, pixelattr, &OBJLine[xpos]); j < cnt; j++)
        DrawSpritePixel<false>(r[j] ? r[j] : -1, pixelattr, xpos + j);
}
#endif

template<bool window>
void SoftRenderer2D::DrawSprite_Normal(u32 num, u32 width, u32 height, s32 xpos, s32 ypos)
{
#ifdef LITEV_SOFT2D_THREADED
    u16* oam = (u16*)&CurOAM[GPU2D.Num ? 0x400 : 0];
#else
    u16* oam = (u16*)&GPU.OAM[GPU2D.Num ? 0x400 : 0];
#endif
    u16* attrib = &oam[num * 4];

    u32 pixelattr = ((attrib[2] & 0x0C00) << 6) | OBJ_IsSprite | OBJ_IsOpaque;
    u32 tilenum = attrib[2] & 0x03FF;
    u32 spritemode = window ? 0 : ((attrib[0] >> 10) & 0x3);

    u32 wmask = width - 8; // really ((width - 1) & ~0x7)

    if ((attrib[0] & (1<<12)) && !window)
    {
        // apply Y mosaic
        pixelattr |= OBJ_Mosaic;
    }

    u8* objvram;
    u32 objvrammask;
    GPU2D.GetOBJVRAM(objvram, objvrammask);

    // yflip
    if (attrib[1] & (1<<13))
        ypos = height-1 - ypos;

    u32 xoff;
    u32 xend = width;
    if (xpos >= 0)
    {
        xoff = 0;
        if ((xpos+xend) > 256)
            xend = 256-xpos;
    }
    else
    {
        xoff = -xpos;
        xpos = 0;
    }

    u16 color = 0; // transparent in all cases

    if (spritemode == 3)
    {
        // bitmap sprite

        u32 alpha = attrib[2] >> 12;
        if (!alpha) return;
        alpha++;

        pixelattr |= (0xC0000000 | (alpha << 24));

        u32 pixelsaddr = tilenum;
        if (GPU2D.DispCnt & 0x40)
        {
            if (GPU2D.DispCnt & 0x20)
            {
                // 'reserved'
                // draws nothing

                return;
            }
            else
            {
                pixelsaddr <<= (7 + ((GPU2D.DispCnt >> 22) & 0x1));
                pixelsaddr += (ypos * width * 2);
            }
        }
        else
        {
            if (GPU2D.DispCnt & 0x20)
            {
                pixelsaddr = ((tilenum & 0x01F) << 4) + ((tilenum & 0x3E0) << 7);
                pixelsaddr += (ypos * 256 * 2);
            }
            else
            {
                pixelsaddr = ((tilenum & 0x00F) << 4) + ((tilenum & 0x3F0) << 7);
                pixelsaddr += (ypos * 128 * 2);
            }
        }

        s32 pixelstride;

        if (attrib[1] & (1<<12)) // xflip
        {
            pixelsaddr += ((width-1) << 1);
            pixelsaddr -= (xoff << 1);
            pixelstride = -2;
        }
        else
        {
            pixelsaddr += (xoff << 1);
            pixelstride = 2;
        }

        for (; xoff < xend;)
        {
            color = *(u16*)&objvram[pixelsaddr & objvrammask];

            pixelsaddr += pixelstride;

            DrawSpritePixel<window>((color&0x8000) ? color : -1, pixelattr, xpos);

            xoff++;
            xpos++;
        }
    }
    else
    {
        u32 pixelsaddr = tilenum;
        if (GPU2D.DispCnt & (1<<4))
        {
            pixelsaddr <<= ((GPU2D.DispCnt >> 20) & 0x3);
            pixelsaddr += ((ypos >> 3) * (width >> 3)) << ((attrib[0] & 0x2000) ? 1:0);
        }
        else
        {
            pixelsaddr += ((ypos >> 3) * 0x20);
        }

        if (spritemode == 1) pixelattr |= 0x80000000;
        else                 pixelattr |= 0x10000000;

        if (attrib[0] & (1<<13))
        {
            // 256-color
            pixelsaddr <<= 5;
            pixelsaddr += ((ypos & 0x7) << 3);
            s32 pixelstride;

            if (!window)
            {
                if (!(GPU2D.DispCnt & (1<<31)))
                    pixelattr |= OBJ_StandardPal;
                else
                    pixelattr |= ((attrib[2] & 0xF000) >> 4);
            }
#ifdef S2D_OBJROW
            if (!window)
            {
                // the whole texel row (tile rows of 8 bytes, 64 apart), then the visible part
                alignas(16) u8 tex[64], row[64];
                for (u32 t = 0; t < width; t += 8) memcpy(tex + t, &objvram[(pixelsaddr + (t << 3)) & objvrammask], 8);
                S2DPlotSpriteRow(tex, row, width, attrib[1] & (1<<12), xoff, xend, xpos, pixelattr);
                return;
            }
#endif

            if (attrib[1] & (1<<12)) // xflip
            {
                pixelsaddr += (((width-1) & wmask) << 3);
                pixelsaddr += ((width-1) & 0x7);
                pixelsaddr -= ((xoff & wmask) << 3);
                pixelsaddr -= (xoff & 0x7);
                pixelstride = -1;
            }
            else
            {
                pixelsaddr += ((xoff & wmask) << 3);
                pixelsaddr += (xoff & 0x7);
                pixelstride = 1;
            }

            for (; xoff < xend;)
            {
                color = objvram[pixelsaddr & objvrammask];

                pixelsaddr += pixelstride;

                DrawSpritePixel<window>(color ? color : -1, pixelattr, xpos);

                xoff++;
                xpos++;
                if (!(xoff & 0x7)) pixelsaddr += (56 * pixelstride);
            }
        }
        else
        {
            // 16-color
            pixelsaddr <<= 5;
            pixelsaddr += ((ypos & 0x7) << 2);
            s32 pixelstride;

            if (!window)
            {
                pixelattr |= OBJ_StandardPal;
                pixelattr |= ((attrib[2] & 0xF000) >> 8);
            }
#ifdef S2D_OBJROW
            if (!window)
            {
                // the whole texel row (tile rows of 4 bytes, 32 apart, low nibble first)
                alignas(16) u8 tex[64], row[64];
                for (u32 t = 0; t < width; t += 8)
                {
                    u32 w; memcpy(&w, &objvram[(pixelsaddr + (t << 2)) & objvrammask], 4);
                    const uint8x8_t raw = vreinterpret_u8_u32(vdup_n_u32(w));
                    vst1_u8(tex + t, vzip_u8(vand_u8(raw, vdup_n_u8(0xF)), vshr_n_u8(raw, 4)).val[0]);
                }
                S2DPlotSpriteRow(tex, row, width, attrib[1] & (1<<12), xoff, xend, xpos, pixelattr);
                return;
            }
#endif

            // TODO: optimize VRAM access!!
            // TODO: do xflip better? the 'two pixels per byte' thing makes it a bit shitty

            if (attrib[1] & (1<<12)) // xflip
            {
                pixelsaddr += (((width-1) & wmask) << 2);
                pixelsaddr += (((width-1) & 0x7) >> 1);
                pixelsaddr -= ((xoff & wmask) << 2);
                pixelsaddr -= ((xoff & 0x7) >> 1);
                pixelstride = -1;
            }
            else
            {
                pixelsaddr += ((xoff & wmask) << 2);
                pixelsaddr += ((xoff & 0x7) >> 1);
                pixelstride = 1;
            }

            for (; xoff < xend;)
            {
                if (attrib[1] & (1<<12))
                {
                    if (xoff & 0x1) { color = objvram[pixelsaddr & objvrammask] & 0x0F; pixelsaddr--; }
                    else              color = objvram[pixelsaddr & objvrammask] >> 4;
                }
                else
                {
                    if (xoff & 0x1) { color = objvram[pixelsaddr & objvrammask] >> 4; pixelsaddr++; }
                    else              color = objvram[pixelsaddr & objvrammask] & 0x0F;
                }

                DrawSpritePixel<window>(color ? color : -1, pixelattr, xpos);

                xoff++;
                xpos++;
                if (!(xoff & 0x7)) pixelsaddr += ((attrib[1] & 0x1000) ? -28 : 28);
            }
        }
    }
}

}
