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

#ifdef LITEV_SOFT2D_PALCONV
// A BGR555 palette entry in the BGOBJLine colour format (6-bit channels, bit 15 = green LSB), as
// SoftRenderer2D::DrawPixel converts it. Inspired by DSperate's palette converted once per
// generation (https://github.com/beebono/DSperate, GPL-3.0-or-later, commit f5fa35e,
// src/core/gpu/engine2d.cpp std_pal18).
static inline u32 S2DConvColor(u16 c)
{
    const u32 r = (c & 0x001F) << 1, g = ((c & 0x03E0) >> 4) | ((c & 0x8000) >> 15), b = (c & 0x7C00) >> 9;
    return r | (g << 8) | (b << 16);
}

#endif
#if defined(LITEV_SOFT2D_PLANES) && defined(LITEV_SOFT2D_THREADED) && defined(LITEV_SOFT2D_PALCONV) && defined(LITEV_SOFT2D_NEON) && defined(__aarch64__)
// layer planes + one select/resolve per line (needs the converted palette and the NEON compositor)
#define S2D_PLANES 1
#endif
#if defined(LITEV_SOFT2D_OUTNEON) && defined(LITEV_SOFT2D_THREADED) && defined(__aarch64__)
// one-pass NEON output stage: brightness + 6->8 bit expansion, display capture
#define S2D_OUTNEON 1
#endif
#if defined(LITEV_SOFT2D_OBJROW) && defined(__aarch64__)
// sprite rows decoded a tile row at a time and plotted 4 pixels at a time
#define S2D_OBJROW 1
#endif
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
    bool LineSnapsEqual(int slot) const   // skip-repeat: this frame's snapshots == slot's
    { return !memcmp(LineSnap, LineSnapR[slot], sizeof(LineSnap)) && !memcmp(SprSnap, SprSnapR[slot], sizeof(SprSnap)); }
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
#ifdef LITEV_SOFT2D_SPRLIST
    const u32* CurOAMGen = nullptr;   // CurOAM content generation (null: unknown, scan all 128 sprites)
#endif

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
#ifdef LITEV_SOFT2D_PALCONV
    // CurPalette's 1024 entries already in the BGOBJLine colour format (S2DPalConv), or null
    const u32* CurPalConv = nullptr;
    static void DrawPixelConv(u32* dst, u32 c) { *(dst+256) = *dst; *dst = c; }
#endif

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
#ifdef S2D_PLANES
    // a new frame: the extended palettes are revalidated on first use
    void PlanesBeginFrame() { BGExtChecked = 0; OBJExtChecked = 0; }
#endif

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
#ifdef LITEV_SOFT2D_SPRLIST
    u32 OBJPrioMask = 0;        // priorities of this line's non-window sprites (InterleaveSprites skip)
    u32 SprListGen = 0;         // generation SprLines was built for
    u64 SprLines[192][2];       // per line: sprites whose box covers it
    void BuildSprLists(const u16* oam);
#endif

#ifdef S2D_PLANES
    // LITEV_SOFT2D_PLANES: each BG draws into its own plane of u16 values (bit 15 opaque, bits 0-14
    // the index into the layer's colour table), then one priority select per line picks the top
    // (and, when a colour effect can read it, the second) layer per pixel and resolves only those
    // through the tables. Inspired by DSperate's layer planes and select pass
    // (https://github.com/beebono/DSperate, GPL-3.0-or-later, commit f5fa35e,
    // src/core/gpu/engine2d.cpp render_line / select_layers).
    enum { TID_OBJ = 4, TID_BACKDROP = 5, TID_3D = 6, TID_NONE = 7 };
    alignas(16) u16 PlaneBuf[4][8 + 264 + 8];   // text rows land at -scroll, 33 tiles wide
    u16* PlaneV(int n) { return &PlaneBuf[n][8]; }
    const u32* PlaneTab[4] {};
    alignas(16) u32 DirCol[4][256];   // direct-colour BG: colours by position
    alignas(16) u32 ObjCol[256];      // opaque sprite pixels' colour records by position
    bool PlaneAny[4] {};
    alignas(16) u16 Top16[256], Sec16[256];
    alignas(16) u8 TopTid[256], SecTid[256];
    // extended palettes in the line colour format, revalidated (compared with the copy they were
    // converted from) on first use in a frame
    alignas(16) u32 BGExtConv[4][4096];
    u16 BGExtCopy[4][4096];
    u64 BGExtChecked = 0, BGExtHave = 0;   // bit slot * 16 + palette
    alignas(16) u32 OBJExtConv[4096];
    u16 OBJExtCopy[4096];
    u32 OBJExtChecked = 0, OBJExtHave = 0;
    void ValidateBGExt(u32 slot, u32 palmask);
    void ValidateOBJExt(u32 pal);
    void DrawScanlinePlanes(u32 line, u32* dst);
    template<bool mosaic> void PlaneBG_Text(u32 line, u32 bgnum);
    template<bool mosaic> void PlaneBG_Affine(u32 bgnum);
    template<bool mosaic> void PlaneBG_Extended(u32 bgnum);
    template<bool mosaic> void PlaneBG_Large();
#endif

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
#ifdef S2D_OBJROW
    void S2DPlotSpriteRow(const u8* tex, u8* row, u32 width, bool hflip, u32 xoff, u32 xend, s32 xpos, u32 pixelattr);
#endif
};

}
