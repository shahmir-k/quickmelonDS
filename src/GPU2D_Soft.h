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

#include "GPU2D.h"

namespace melonDS
{
class SoftRenderer;

class SoftRenderer2D : public Renderer2D
{
public:
    SoftRenderer2D(melonDS::GPU2D& gpu2D, SoftRenderer& parent);
    ~SoftRenderer2D() override;
    bool Init() override { return true; }
    void Reset() override;

    void DrawScanline(u32 line) override;
    void DrawSprites(u32 line) override;
    void VBlank() override {}
    void VBlankEnd() override {};

#ifdef LITEV_SOFT2D_THREADED
    // ---- liteV banded deferred software 2D ----
    // Per-scanline register snapshot mirrored from the GL renderer's
    // sScanlineConfig set (the authoritative list of per-scanline-mutable state)
    // plus the extra fields the software raster reads live. Captured on the emu
    // thread at each LCD scanline event; replayed off the critical path at VBlank.
    struct S2DLineState
    {
        // engine gating
        bool Enabled;
        u8   ForcedBlank;
        // DISPCNT + BG control
        u32  DispCnt;
        u8   LayerEnable;
        u16  BGCnt[4];
        // BG scroll (text)
        u16  BGXPos[4];
        u16  BGYPos[4];
        // BG affine ref points (internal, per-scanline-advanced) + rotscale
        s32  BGXRefInternal[2];
        s32  BGYRefInternal[2];
        s16  BGRotA[2];
        s16  BGRotC[2];
        // mosaic
        u8   BGMosaicSize[2];
        u8   OBJMosaicSize[2];
        u32  BGMosaicLine;
        // blend
        u16  BlendCnt;
        u8   EVA, EVB, EVY;
        // windows (WININ/WINOUT + positions + active carry state)
        u8   WinCnt[4];
        u8   Win0Coords[4];
        u8   Win1Coords[4];
        u8   Win0Active;
        u8   Win1Active;
    };
    // Sprite state captured at DrawSprites time (one scanline earlier than the
    // BG state; OBJ mosaic + OBJ VRAM mapping are read at that distinct moment).
    struct S2DSprState
    {
        bool Enabled;
        u8   OBJEnable;
        u32  DispCnt;
        u32  OBJMosaicLine;
        u8   OBJMosaicSize[2];
    };

    void SnapshotLineState(u32 line);   // emu thread; captures into LineSnap[line]
    void SnapshotSprState(u32 line);    // emu thread; captures into SprSnap[line]
    void LoadLineState(const S2DLineState& s);
    void LoadSprState(const S2DSprState& s);

    // Async depth-1 pipeline: at VBlank the emu thread copies the just-completed
    // frame's per-line snapshots into the *R (render-owned) arrays, then the async
    // render thread reads ONLY the *R copies while the emu overwrites LineSnap/
    // SprSnap for the next frame. Avoids a per-scanline read/write race.
#if defined(LITEV_SOFT2D_DEPTH2)
    void CopyLineSnaps(int slot);
#else
    void CopyLineSnaps();
#endif

    // Deferred (no per-scanline VRAM coherence; that runs once/frame at VBlank).
    void SyncVRAM_BG();
    void SyncVRAM_OBJ();
    // Snapshot passed by ref so a per-band private renderer can read the shared
    // main snapshot arrays and load into its OWN GPU2D unit.
    void DrawSpritesDeferred(const S2DSprState& s, u32 line);
    void DrawScanlineDeferred(const S2DLineState& s, u32 line, u32* dst);
    u64 PipeTraceSpriteHash() const;

    u32* Cur3DLine = nullptr;                  // per-line 3D output (replaces Parent.Output3D)
    const u8* CurOAM = nullptr;                // per-line OAM base (replaces GPU.OAM)

    // Per-scanline snapshots, filled on the emu thread at each LCD scanline event,
    // consumed by the deferred (batched, later banded-threaded) render at VBlank.
    S2DLineState LineSnap[192];
    S2DSprState  SprSnap[192];
    // Render-owned copies (see CopyLineSnaps): read by the async render thread.
#if defined(LITEV_SOFT2D_DEPTH2)
    // Part 2: parity double-buffer (see SoftRenderer::SnapParity/AsyncSnapSlot). CopyLineSnaps
    // takes the slot; the deferred draws read LineSnapR[slot]/SprSnapR[slot]. Byte-identical
    // at depth-1.
    S2DLineState LineSnapR[2][192];
    S2DSprState  SprSnapR[2][192];
#else
    S2DLineState LineSnapR[192];
    S2DSprState  SprSnapR[192];
#endif
#endif

    // Palette base the deferred/banded draws read. Defaults to the live GPU.Palette
    // (non-threaded/inline path); the async render points it at a per-frame snapshot
    // (PaletteSnap) so frame N's render doesn't race the emu's palette writes in N+1.
    const u8* CurPalette = nullptr;

    // Hybrid renderer (3D on the GPU): engine A emits, per pixel, a 2-word descriptor
    // instead of a final colour whenever DISPCNT bit 3 (BG0 = 3D) is set:
    //   dst[i]     val1  RGB: blend partner (layer under the 3D, or the layer above it)
    //                    bits 29-31: merge mode (7 = no 3D, 4 = 3D on top with 3D-alpha
    //                    blend, 0/2/3 = 3D on top with no effect / brightness up / down,
    //                    1 = 3D under val1); bits 24-28: EVY (modes 2/3) or EVA (mode 1)
    //   dst[256+i] val2  RGB: final colour if the 3D pixel is transparent; bits 24-28: EVB
    // LastLineHas3D says whether the last DrawScanlineDeferred emitted descriptors (else
    // dst[0..255] is plain colour). Ported from classic melonDS's "accelerated" software 2D
    // (removed upstream in ba317e2e), packed into 2 planes instead of 3.
    bool HybridDesc = false;
    bool LastLineHas3D = false;
    static u32 HybridResolvePixel(u32 val1, u32 val2, u32 c3d);

private:
    SoftRenderer& Parent;

    enum
    {
        OBJ_StandardPal = (1<<12),
        OBJ_DirectColor = (1<<15),
        OBJ_BGPrioMask = (0x3<<16),
        OBJ_IsOpaque = (1<<18),
        OBJ_OpaPrioMask = (OBJ_BGPrioMask | OBJ_IsOpaque),
        OBJ_IsSprite = (1<<19),
        OBJ_Mosaic = (1<<20),
    };

    alignas(8) u32 BGOBJLine[256*2];

    alignas(8) u8 WindowMask[256];
    alignas(8) u32 Under3D[2][256];   // HybridDesc: the two layers under the 3D placeholder

    alignas(8) u32 OBJLine[256];
    alignas(8) u8 OBJWindow[256];

    u32 NumSprites;

    u8* CurBGXMosaicTable;
    array2d<u8, 16, 256> MosaicTable = []() constexpr
    {
        array2d<u8, 16, 256> table {};
        // initialize mosaic table
        for (int m = 0; m < 16; m++)
        {
            for (int x = 0; x < 256; x++)
            {
                int offset = x % (m+1);
                table[m][x] = offset;
            }
        }

        return table;
    }();

    u32 ColorComposite(int i, u32 val1, u32 val2) const;

    template<u32 bgmode> void DrawScanlineBGMode(u32 line);
    void DrawScanlineBGMode6(u32 line);
    void DrawScanlineBGMode7(u32 line);
    void DrawScanline_BGOBJ(u32 line, u32* dst);

    static void DrawPixel(u32* dst, u16 color, u32 flag);

    void DrawBG_3D();
    void HybridCompositeLine(u32* dst);
    template<bool mosaic> void DrawBG_Text(u32 line, u32 bgnum);
    template<bool mosaic> void DrawBG_Affine(u32 line, u32 bgnum);
    template<bool mosaic> void DrawBG_Extended(u32 line, u32 bgnum);
    template<bool mosaic> void DrawBG_Large(u32 line);

    void ApplySpriteMosaicX();
    void InterleaveSprites(u32 prio);
    template<bool window> void DrawSpritePixel(int color, u32 pixelattr, s32 xpos);
    template<bool window> void DrawSprite_Rotscale(u32 num, u32 boundwidth, u32 boundheight, u32 width, u32 height, s32 xpos, s32 ypos);
    template<bool window> void DrawSprite_Normal(u32 num, u32 width, u32 height, s32 xpos, s32 ypos);
};

}
