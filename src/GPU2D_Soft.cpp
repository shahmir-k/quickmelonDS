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
        if ((GPU2D.LayerEnable & (1<<4)) && NumSprites)
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
        if ((GPU2D.LayerEnable & (1<<4)) && NumSprites)
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
        if ((GPU2D.LayerEnable & (1<<4)) && NumSprites)
        {
            InterleaveSprites(i);
        }
    }
}

void SoftRenderer2D::DrawScanline_BGOBJ(u32 line, u32* dst)
{
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
    for (int i = 0; i < 256; i++)
    {
        u32 val1 = BGOBJLine[i];
        u32 val2 = BGOBJLine[256+i];
        u32 flag1 = val1 >> 24;
        u32 flag2 = val2 >> 24;

        if ((flag1 & 0xC0) == 0x40)
        {
            // 3D on top. Under it: Under3D[0] (blend partner), then Under3D[1].
            u32 u1 = Under3D[0][i], u2 = Under3D[1][i];
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
            dst[256+i] = ColorComposite(i, u1, u2) & 0xFFFFFF;
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
            dst[i]     = ((blend ? val1 : ColorComposite(i, val1, val2)) & 0xFFFFFF) | (1u << 29) | ((eva & 0x1F) << 24);
            dst[256+i] = (ColorComposite(i, val1, under) & 0xFFFFFF) | ((evb & 0x1F) << 24);
        }
        else
        {
            dst[i]     = (ColorComposite(i, val1, val2) & 0xFFFFFF) | (7u << 29);
        }
    }
}

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

            if (extpal) curpal = GPU2D.GetBGExtPal(extpalslot, curtile>>12);
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

                if (extpal) curpal = GPU2D.GetBGExtPal(extpalslot, curtile>>12);
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
                    curtile = *(u16*)&bgvram[(tilemapaddr + (((((finalY & coordmask) >> 11) << yshift) + ((finalX & coordmask) >> 11)) << 1)) & bgvrammask];

                    if (extpal) curpal = GPU2D.GetBGExtPal(bgnum, curtile>>12);
                    else        curpal = pal;

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

    NumSprites = 0;
    memset(OBJLine, 0, sizeof(OBJLine));
    memset(OBJWindow, 0, sizeof(OBJWindow));

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

    for (int sprnum = 0; sprnum < 128; sprnum++)
    {
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
    }
}

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
