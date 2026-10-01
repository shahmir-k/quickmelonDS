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

#include <thread>
#include "NDS.h"
#include "GPU_Soft.h"
#include "GPU_ColorOp.h"
#include "Platform.h"
#include "LitevSoftProf.h"

#ifdef LITEV_SOFT3D_DRASTIC
#include "GPU3D_TileSoft.h"   // alternate tile-based 3D renderer (P1 skeleton)
#endif

#if defined(LITEV_NEON_RENDERER) && defined(__aarch64__)
#include "GPU2D_NEON.h"
#endif

#if defined(LITEV_SOFT2D_DEPTH2) && defined(__ANDROID__)
#include <sys/system_properties.h>
#include <stdlib.h>
#endif

namespace melonDS
{

#ifdef LITEV_SOFT2D_DEPTH2
// Pipeline depth control (Part 3). 1 = steps 1a/1b/2 active but still depth-1 (byte-identical
// -- the second/third banks + snapshot slots just alternate, one live per frame). 2 = the
// depth-2 flip (up to 2 renders in flight, emu runs a frame ahead, VBlank barrier bubble gone).
// Cached per-VBlank read (cheap; like debug.litev.software).
static int litevReadPipeDepth()
{
    // LITEV_SOFT2D_DEPTH2 (the tile-renderer 2D decouple) DEFAULTS to depth-2 — its whole
    // purpose is to run the emu a frame ahead of the 2D compositor and remove the VBlank bar2D
    // barrier.
#if defined(LITEV_SOFT2D_DEPTH2)
  #if defined(__ANDROID__)
    char b[PROP_VALUE_MAX] = {0};
    int v = (__system_property_get("debug.litev.pipedepth", b) > 0) ? atoi(b) : 2;  // default depth-2
    return (v >= 2) ? 2 : 1;
  #else
    if (const char* e = getenv("LITEV_PIPEDEPTH")) return (atoi(e) >= 2) ? 2 : 1;   // headless override
    return 2;   // headless default under SOFT2D_DEPTH2: depth-2
  #endif
#elif defined(__ANDROID__)
    char b[PROP_VALUE_MAX] = {0};
    // DEFAULT = depth-1 (byte-identical scaffolding); opt into depth-2 by prop.
    int v = (__system_property_get("debug.litev.pipedepth", b) > 0) ? atoi(b) : 1;
    return (v >= 2) ? 2 : 1;
#else
    return 1;   // non-Android (headless) default: depth-1 (safe scaffolding; depth-2 is opt-in)
#endif
}

// RENDER-OUTPUT trace (diagnostic; OFF unless debug.litev.pipetrace=1 -> byte-identical when off).
// The app's FBHASH gate (MelonInstance litevFbHashRam) hashes GetFramebuffers() == the PRESENTED
// buffer (Framebuffer[AsyncPresentBuf], 1 frame delayed), so it cannot tell "the RENDER produced a
// stale bottom" from "the PRESENT served a stale bottom". This hashes the RENDER OUTPUT the instant
// the async raster finishes (Framebuffer[AsyncTargetBuf], before any present/swap), tagged with the
// render counter, the 2D snapshot slot it consumed, and the target buffer. Diff the RENDER seq vs
// the FBHASH PRESENT seq: if RENDER bot repeats ~natural (17%) but PRESENT bot repeats ~50%, the
// divergence is in the present/framebuffer path (NOT the 2D snapshot); if RENDER bot itself repeats
// ~50%, the raster is stale and ss/buf localize which generation it read.
static int litevReadPipeTrace()
{
#if defined(__ANDROID__)
    char b[PROP_VALUE_MAX] = {0};
    return (__system_property_get("debug.litev.pipetrace", b) > 0) ? atoi(b) : 0;
#else
    return 0;
#endif
}
static inline u64 litevPipeFnv1a(const void* p, size_t n, u64 h = 1469598103934665603ULL)
{
    const u8* d = (const u8*)p;
    for (size_t i = 0; i < n; i++) { h ^= d[i]; h *= 1099511628211ULL; }
    return h;
}
#endif

SoftRenderer::SoftRenderer(melonDS::NDS& nds)
    : Renderer(nds.GPU)
{
    const size_t len = 256 * 192;
    Framebuffer[0][0] = new u32[len];
    Framebuffer[0][1] = new u32[len];
    Framebuffer[1][0] = new u32[len];
    Framebuffer[1][1] = new u32[len];
#ifdef LITEV_SOFT2D_DEPTH2
    Framebuffer[2][0] = new u32[len];   // Part 3: 3rd framebuffer for depth-2
    Framebuffer[2][1] = new u32[len];
#endif
    BackBuffer = 0;

    Rend2D_A = std::make_unique<SoftRenderer2D>(GPU.GPU2D_A, *this);
    Rend2D_B = std::make_unique<SoftRenderer2D>(GPU.GPU2D_B, *this);
#ifdef LITEV_SOFT3D_DRASTIC
    // Alternate DraStic-style tile renderer (P1 skeleton). Implements the same Renderer3D
    // interface, so everything above (GetLine consumption, frame driving) is unchanged.
    Rend3D = std::make_unique<TileRenderer3D>(GPU.GPU3D, *this);
#else
    Rend3D = std::make_unique<SoftRenderer3D>(GPU.GPU3D, *this);
#endif

#ifdef LITEV_SOFT2D_THREADED
    AsyncStart = Platform::Semaphore_Create();
    AsyncDone  = Platform::Semaphore_Create();
#endif
}

SoftRenderer::~SoftRenderer()
{
#ifdef LITEV_SOFT2D_THREADED
    StopAsyncThread();
    Platform::Semaphore_Free(AsyncStart);
    Platform::Semaphore_Free(AsyncDone);
#endif
    delete[] Framebuffer[0][0];
    delete[] Framebuffer[0][1];
    delete[] Framebuffer[1][0];
    delete[] Framebuffer[1][1];
#ifdef LITEV_SOFT2D_DEPTH2
    delete[] Framebuffer[2][0];
    delete[] Framebuffer[2][1];
#endif
}

void SoftRenderer::Reset()
{
#ifdef LITEV_SOFT2D_THREADED
    FlushAsyncRender();
    AsyncEverProduced = false;
    AsyncPresentBuf = 1;
#endif
    const size_t len = 256 * 192 * sizeof(u32);
    memset(Framebuffer[0][0], 0, len);
    memset(Framebuffer[0][1], 0, len);
    memset(Framebuffer[1][0], 0, len);
    memset(Framebuffer[1][1], 0, len);
#ifdef LITEV_SOFT2D_DEPTH2
    memset(Framebuffer[2][0], 0, len);
    memset(Framebuffer[2][1], 0, len);
#endif

    Rend2D_A->Reset();
    Rend2D_B->Reset();
    Rend3D->Reset();
}

void SoftRenderer::Stop()
{
#ifdef LITEV_SOFT2D_THREADED
    // flush any in-flight async render so we don't clear a buffer mid-write
    FlushAsyncRender();
#endif
    // clear framebuffers to black
    const size_t len = 256 * 192 * sizeof(u32);
    memset(Framebuffer[0][0], 0, len);
    memset(Framebuffer[0][1], 0, len);
    memset(Framebuffer[1][0], 0, len);
    memset(Framebuffer[1][1], 0, len);
#ifdef LITEV_SOFT2D_DEPTH2
    memset(Framebuffer[2][0], 0, len);
    memset(Framebuffer[2][1], 0, len);
#endif
}


void SoftRenderer::PreSavestate()
{
#ifdef LITEV_SOFT2D_THREADED
    // ensure no async 2D render is reading emu state while it is (de)serialized
    FlushAsyncRender();
#endif
    auto rend3d = dynamic_cast<SoftRenderer3D*>(Rend3D.get());
#ifdef LITEV_SOFT3D_DRASTIC
    if (!rend3d) return;   // TileRenderer3D is synchronous -- no render thread to set up
#endif
    if (rend3d->IsThreaded())
        rend3d->SetupRenderThread();
}

void SoftRenderer::PostSavestate()
{
    auto rend3d = dynamic_cast<SoftRenderer3D*>(Rend3D.get());
#ifdef LITEV_SOFT3D_DRASTIC
    if (!rend3d) return;   // TileRenderer3D is synchronous -- no render thread to enable
#endif
    if (rend3d->IsThreaded())
        rend3d->EnableRenderThread();
}


void SoftRenderer::SetRenderSettings(RendererSettings& settings)
{
    auto rend3d = dynamic_cast<SoftRenderer3D*>(Rend3D.get());
#ifdef LITEV_SOFT3D_DRASTIC
    if (!rend3d) return;   // TileRenderer3D is synchronous -- threaded setting does not apply
#endif
    rend3d->SetThreaded(settings.Threaded);
}


void SoftRenderer::DrawScanline(u32 line)
{
#ifdef LITEV_SOFT2D_THREADED
    // Deferred (DraStic-model): snapshot this line's per-scanline state and defer the
    // whole raster+composite to RenderDeferredFrame() at VBlank. Index by the dst line
    // (fb position); the snapshot captures the current (VCount) register state, which
    // matches the inline path when VCount==line (the normal case).
    if (line < 192)
    {
        // DraStic model: the emu thread NEVER touches the 3D. It only snapshots the
        // 2D per-scanline register state. The whole raster (incl. consuming the 3D via
        // GetLine, paced by the 3D render thread) runs on the async render thread, so
        // the emu is never blocked per-scanline waiting on the 3D. (Requires the async
        // render to be single-threaded — S2D_NBANDS=1 — so GetLine is consumed in order.)
        static_cast<SoftRenderer2D*>(Rend2D_A.get())->SnapshotLineState(line);
        static_cast<SoftRenderer2D*>(Rend2D_B.get())->SnapshotLineState(line);
        SnapshotCompositeLine(line);
        S2DDeferActive = true;
        return;
    }
    // line >= 192 falls through to the original inline out-of-range path.
#endif
    u32 *dstA, *dstB;
    u32 dstoffset = 256 * line;
    if (GPU.ScreenSwap)
    {
        dstA = &Framebuffer[BackBuffer][0][dstoffset];
        dstB = &Framebuffer[BackBuffer][1][dstoffset];
    }
    else
    {
        dstA = &Framebuffer[BackBuffer][1][dstoffset];
        dstB = &Framebuffer[BackBuffer][0][dstoffset];
    }

    // the position used for drawing operations is based on VCOUNT
    line = GPU.VCount;
    if (line < 192)
    {
        // retrieve 3D output
        Output3D = Rend3D->GetLine(line);

        // draw BG/OBJ layers
        Rend2D_A->DrawScanline(line);
        Rend2D_B->DrawScanline(line);

        // draw the final screen output
        DrawScanlineA(line, dstA, Output2D[0], GPU.GPU2D_A.DispCnt, GPU.MasterBrightnessA);
        DrawScanlineB(line, dstB, Output2D[1], GPU.GPU2D_B.DispCnt, GPU.MasterBrightnessB);

        // perform display capture if enabled
        if (GPU.CaptureEnable)
            DoCapture(line, Output2D[0], Output3D);
    }
    else
    {
        // if scanlines outside VCOUNT range 0..191 were to be visible, fill them white
        // this may happen if VCOUNT is written to during active display
        // the actual hardware behavior depends on the screen model, and suggests that
        // no video signal is output for such scanlines

        for (int i = 0; i < 256; i++)
        {
            dstA[i] = 0x3F3F3F;
            dstB[i] = 0x3F3F3F;
        }
    }

    if (GPU.ScreensEnabled)
    {
        // expand the color from 6-bit to 8-bit
        ExpandColor(dstA);
        ExpandColor(dstB);
    }
    else
    {
        // if the screens are disabled: fill the framebuffer black
        for (int i = 0; i < 256; i++)
        {
            dstA[i] = 0xFF000000;
            dstB[i] = 0xFF000000;
        }
    }
}

void SoftRenderer::DrawSprites(u32 line)
{
#ifdef LITEV_SOFT2D_THREADED
    // Deferred: snapshot the OBJ state at this (one-line-ahead) moment; the actual
    // sprite raster runs in RenderDeferredFrame at VBlank.
    if (line < 192)
    {
        static_cast<SoftRenderer2D*>(Rend2D_A.get())->SnapshotSprState(line);
        static_cast<SoftRenderer2D*>(Rend2D_B.get())->SnapshotSprState(line);
        return;
    }
#endif
    Rend2D_A->DrawSprites(line);
    Rend2D_B->DrawSprites(line);
}

#ifdef LITEV_SOFT2D_THREADED
void SoftRenderer::SnapshotCompositeLine(u32 line)
{
    FrameLineSnap& f = FrameSnap[line];
    f.DispCntA = GPU.GPU2D_A.DispCnt;
    f.DispCntB = GPU.GPU2D_B.DispCnt;
    f.MasterBrightnessA = GPU.MasterBrightnessA;
    f.MasterBrightnessB = GPU.MasterBrightnessB;
    f.ScreenSwap = GPU.ScreenSwap;
    f.ScreensEnabled = GPU.ScreensEnabled;
    f.CaptureEnable = GPU.CaptureEnable;
    f.Valid = 1;
}

// The whole frame's 2D raster + final composite, run once at VBlank off the
// per-scanline critical path. Milestone 1: single-thread, reusing the inline
// draw functions after restoring each line's snapshot. Bit-exact-gated vs the
// per-scanline inline path before band-threading.
// Lazily create the per-band private render contexts (once).
void SoftRenderer::InitBands()
{
#ifdef LITEV_SOFT2D_DEPTH2
    // Part 2/3: one private band-unit set per parity slot (depth-2 needs both live).
    for (int p = 0; p < 2; p++)
        for (int b = 0; b < S2D_NBANDS; b++)
            for (int e = 0; e < 2; e++)
            {
                S2DBands[p][b].unit[e] = std::make_unique<GPU2D>((u32)e, GPU);
                S2DBands[p][b].rend[e] = std::make_unique<SoftRenderer2D>(*S2DBands[p][b].unit[e], *this);
                static_cast<SoftRenderer2D*>(S2DBands[p][b].rend[e].get())->Reset();
            }
#else
    for (int b = 0; b < S2D_NBANDS; b++)
    {
        for (int e = 0; e < 2; e++)
        {
            S2DBands[b].unit[e] = std::make_unique<GPU2D>((u32)e, GPU);
            S2DBands[b].rend[e] = std::make_unique<SoftRenderer2D>(*S2DBands[b].unit[e], *this);
            static_cast<SoftRenderer2D*>(S2DBands[b].rend[e].get())->Reset();
        }
    }
#endif
    S2DBandsInit = true;
}

// Render a disjoint line range [y0,y1) for BOTH engines into BandOut2D, using this
// band's PRIVATE units/renderers, reading the shared main snapshots. Runs on a
// helper thread; no shared mutable render state with the other bands.
void SoftRenderer::RenderBand(int bi, u32 y0, u32 y1)
{
    auto* mainA = static_cast<SoftRenderer2D*>(Rend2D_A.get());
    auto* mainB = static_cast<SoftRenderer2D*>(Rend2D_B.get());
#ifdef LITEV_SOFT2D_DEPTH2
    // Part 2/3: read this render's captured snapshot slot (depth-1: == the slot just written)
    // AND use that slot's private band-unit set (seeded by the emu for this frame's parity).
    const int _ss = AsyncSnapSlot;
    auto* rA = static_cast<SoftRenderer2D*>(S2DBands[_ss][bi].rend[0].get());
    auto* rB = static_cast<SoftRenderer2D*>(S2DBands[_ss][bi].rend[1].get());
    auto& fsR = FrameSnapR[_ss];
    const u8* const palSnap = PaletteSnap[_ss];
    const u8* const oamSnap = OAMSnap[_ss];
#else
    auto* rA = static_cast<SoftRenderer2D*>(S2DBands[bi].rend[0].get());
    auto* rB = static_cast<SoftRenderer2D*>(S2DBands[bi].rend[1].get());
    auto& fsR = FrameSnapR;
    const u8* const palSnap = PaletteSnap;
    const u8* const oamSnap = OAMSnap;
#endif

    const double _lspB0 = LSP_NOW();
#ifdef LITEV_SOFT2D_DEPTH2
    // Diagnostic-only: establish whether a render-output mismatch originated
    // in the TileRenderer3D source lines or in the 2D compositor consuming them.
    static int pipeTraceOn = -1;
    if (pipeTraceOn < 0) pipeTraceOn = litevReadPipeTrace();
    u64 pipe3dHash = 1469598103934665603ULL;
    u64 pipeSpriteTopHash = 1469598103934665603ULL;
    u64 pipe2dTopHash = 1469598103934665603ULL;
#endif
    for (u32 line = y0; line < y1; line++)
    {
        // DraStic model: consume the 3D line HERE, on the async render thread, paced by
        // the 3D render thread's per-scanline semaphore — NOT on the emu thread. Must
        // GetLine EVERY line (0..191) to keep the semaphore count balanced even for
        // skipped lines. (Single-threaded async render — S2D_NBANDS=1 — so in-order.)
        const double _lspG0 = LSP_NOW();
        u32* l3d = Rend3D->GetLine(line);
        LSP_ADD(S2DBlock[bi], LSP_NOW() - _lspG0);
#ifdef LITEV_SOFT2D_DEPTH2
        if (pipeTraceOn)
            pipe3dHash = litevPipeFnv1a(l3d, 256 * sizeof(u32), pipe3dHash);
#endif

        // Read the render-owned snapshot copies (frame N) — the emu thread is
        // concurrently overwriting the live FrameSnap/LineSnap/SprSnap for frame N+1,
        // so we must NOT touch those here.
        FrameLineSnap& f = fsR[line];
        if (!f.Valid) continue;

        // --- BG/OBJ raster into this band's per-engine line buffers ---
        rA->Cur3DLine = l3d;
        rA->CurOAM = oamSnap;
        rA->CurPalette = palSnap;
#ifdef LITEV_SOFT2D_DEPTH2
        rA->DrawSpritesDeferred(mainA->SprSnapR[_ss][line], line);
        if (pipeTraceOn)
        {
            const u64 sh = rA->PipeTraceSpriteHash();
            pipeSpriteTopHash = litevPipeFnv1a(&sh, sizeof(sh), pipeSpriteTopHash);
        }
        rA->DrawScanlineDeferred(mainA->LineSnapR[_ss][line], line, BandOut2D[0][line]);
#else
        rA->DrawSpritesDeferred(mainA->SprSnapR[line], line);
        rA->DrawScanlineDeferred(mainA->LineSnapR[line], line, BandOut2D[0][line]);
#endif
#ifdef LITEV_SOFT2D_DEPTH2
        if (pipeTraceOn)
            pipe2dTopHash = litevPipeFnv1a(BandOut2D[0][line], 256 * sizeof(u32), pipe2dTopHash);
#endif

        rB->Cur3DLine = l3d;
        rB->CurOAM = oamSnap;
        rB->CurPalette = palSnap;
#ifdef LITEV_SOFT2D_DEPTH2
        rB->DrawSpritesDeferred(mainB->SprSnapR[_ss][line], line);
        rB->DrawScanlineDeferred(mainB->LineSnapR[_ss][line], line, BandOut2D[1][line]);
#else
        rB->DrawSpritesDeferred(mainB->SprSnapR[line], line);
        rB->DrawScanlineDeferred(mainB->LineSnapR[line], line, BandOut2D[1][line]);
#endif

        // --- final composite + capture + expand into the framebuffer (banded too) ---
        u32 dstoffset = 256 * line;
        u32 *dstA, *dstB;
        // Write the buffer captured at signal time (NOT the live BackBuffer, which
        // FinishFrame swaps while this async render is still running).
        if (f.ScreenSwap)
        {
            dstA = &Framebuffer[AsyncTargetBuf][0][dstoffset];
            dstB = &Framebuffer[AsyncTargetBuf][1][dstoffset];
        }
        else
        {
            dstA = &Framebuffer[AsyncTargetBuf][1][dstoffset];
            dstB = &Framebuffer[AsyncTargetBuf][0][dstoffset];
        }

        DrawScanlineA(line, dstA, BandOut2D[0][line], f.DispCntA, f.MasterBrightnessA);
        DrawScanlineB(line, dstB, BandOut2D[1][line], f.DispCntB, f.MasterBrightnessB);

        if (f.CaptureEnable)
            DoCapture(line, BandOut2D[0][line], l3d);

        if (f.ScreensEnabled)
        {
            ExpandColor(dstA);
            ExpandColor(dstB);
        }
        else
        {
            for (int i = 0; i < 256; i++) { dstA[i] = 0xFF000000; dstB[i] = 0xFF000000; }
        }

        f.Valid = 0;
    }
#ifdef LITEV_SOFT2D_DEPTH2
    if (pipeTraceOn && bi == 0 && y0 == 0 && y1 == 192)
    {
        PipeTrace3DHash = pipe3dHash;
        PipeTraceSpriteTopHash = pipeSpriteTopHash;
        PipeTrace2DTopHash = pipe2dTopHash;
    }
#endif
    LSP_ADD(S2DBand[bi], LSP_NOW() - _lspB0);
}

// The banded raster+composite+capture+expand for one frame. Runs entirely on the
// persistent async render thread (spawning short-lived band helpers for the other
// cores), reading ONLY the render-owned snapshots the emu thread published at the
// signalling VBlank. The emu thread is emulating frame N+1 concurrently.
void SoftRenderer::AsyncRenderFrame()
{
    if (S2D_NBANDS == 1)
    {
        // Single-band: the whole 2D on this one persistent thread. No per-frame
        // std::thread spawn, and the 3D bands keep all 3 render cores.
        RenderBand(0, 0, 192);
        return;
    }

    const u32 rows = 192 / S2D_NBANDS;
    std::thread helpers[S2D_NBANDS > 1 ? S2D_NBANDS - 1 : 1];
    for (int b = 1; b < S2D_NBANDS; b++)
    {
        u32 y0 = (u32)b * rows;
        u32 y1 = (b == S2D_NBANDS - 1) ? 192 : y0 + rows;
        helpers[b-1] = std::thread([this, b, y0, y1]{ LSP_NAME("s2d-help"); RenderBand(b, y0, y1); });
    }
    RenderBand(0, 0, rows);
    for (int b = 0; b < S2D_NBANDS - 1; b++) helpers[b].join();
}

static void litevPinRenderThread() {}

void SoftRenderer::AsyncRenderThreadFunc()
{
    litevPinRenderThread();
    LSP_NAME("s2d-async");
    for (;;)
    {
        Platform::Semaphore_Wait(AsyncStart);
        if (!AsyncThreadRunning.load(std::memory_order_acquire))
            break;
#ifdef LITEV_SOFT2D_DEPTH2
        if (PipeDepth == 2)
        {
            // Depth-2: pop THIS frame's captured keys from the ring (published by the emu's
            // AsyncStart post we just acquired -- acquire edge). Set the per-frame resources
            // the render reads: its framebuffer (AsyncTargetBuf), its 2D-snapshot slot
            // (AsyncSnapSlot), and its 3D ColorBuffer consume bank. The 2D render thread is
            // single, so these members are set + used within this one frame -> no race with
            // another render. (At depth-1 the emu set them at VBlank; no pop.)
            P2InFlightFrame& fr = P2Ring[P2PopIdx];
            P2PopIdx = (P2PopIdx + 1) % 3;
            AsyncTargetBuf = fr.targetBuf;
            AsyncSnapSlot = fr.snapSlot;
        }
        // Part 1b: point the 2D flat BG/OBJ/ext-pal readers (GetBGVRAM/GetOBJVRAM/Get*ExtPal)
        // at THIS frame's snapshot bank before the raster reads them (AsyncSnapSlot: emu-set
        // at depth-1, ring-popped at depth-2). Independent of the 3D texture read pointer
        // (set on the render thread from the 3D render parity).
        GPU.SetBGOBJReadShadow(true, AsyncSnapSlot);
        // Hash the render-owned top-screen snapshot set before any raster work.
        // This separates a reused/corrupted input bank from a race inside the
        // 2D compositor itself. Diagnostic-only (debug.litev.pipetrace).
        if (litevReadPipeTrace())
        {
            auto* traceA = static_cast<SoftRenderer2D*>(Rend2D_A.get());
            u64 h = 1469598103934665603ULL;
            h = litevPipeFnv1a(&FrameSnapR[AsyncSnapSlot], sizeof(FrameSnapR[AsyncSnapSlot]), h);
            h = litevPipeFnv1a(traceA->LineSnapR[AsyncSnapSlot], sizeof(traceA->LineSnapR[AsyncSnapSlot]), h);
            h = litevPipeFnv1a(traceA->SprSnapR[AsyncSnapSlot], sizeof(traceA->SprSnapR[AsyncSnapSlot]), h);
            h = litevPipeFnv1a(PaletteSnap[AsyncSnapSlot], sizeof(PaletteSnap[AsyncSnapSlot]), h);
            h = litevPipeFnv1a(OAMSnap[AsyncSnapSlot], sizeof(OAMSnap[AsyncSnapSlot]), h);
            PipeTraceTopInputHash = h;
        }
#endif
        const double _t0 = LSP_NOW();
        AsyncRenderFrame();
        const double _t1 = LSP_NOW();
        LSP_ADD(S2DWall, _t1 - _t0);
#ifdef LITEV_SOFT2D_DEPTH2
        // RENDER-OUTPUT trace (see litevReadPipeTrace): hash the just-rastered framebuffer BEFORE
        // any present/swap, tagged with the render index + the 2D-snapshot slot + the target buffer.
        {
            static int traceOn = -1;
            static int rndCtr = 0;
            if (traceOn < 0) traceOn = litevReadPipeTrace();
            if (traceOn)
            {
                const size_t _sz = (size_t)256 * 192 * sizeof(u32);
                u64 _ht = litevPipeFnv1a(Framebuffer[AsyncTargetBuf][0], _sz);
                u64 _hb = litevPipeFnv1a(Framebuffer[AsyncTargetBuf][1], _sz);
                Platform::Log(Platform::Info,
                    "LITEV_PIPETRACE render=%d ss=%d buf=%d rin2dtop=0x%016llx r3d=0x%016llx rsprtop=0x%016llx r2dtop=0x%016llx rtop=0x%016llx rbot=0x%016llx\n",
                    rndCtr, AsyncSnapSlot, AsyncTargetBuf,
                    (unsigned long long)PipeTraceTopInputHash,
                    (unsigned long long)PipeTrace3DHash,
                    (unsigned long long)PipeTraceSpriteTopHash,
                    (unsigned long long)PipeTrace2DTopHash,
                    (unsigned long long)_ht, (unsigned long long)_hb);
            }
            rndCtr++;
        }
#endif
#ifdef LITEV_SOFTPROF
        // whole render critical path: 3D render thread wake -> 2D done
        {
            double t3d = LitevSP::S.T3DStart.load(std::memory_order_relaxed);
            if (t3d > 0.0) LSP_ADD(RenderWall, _t1 - t3d);
        }
#endif
        Platform::Semaphore_Post(AsyncDone);
    }
}

void SoftRenderer::StartAsyncThread()
{
    if (!AsyncThreadRunning.load(std::memory_order_relaxed))
    {
        AsyncThreadRunning.store(true, std::memory_order_release);
        AsyncThread = Platform::Thread_Create([this]() { AsyncRenderThreadFunc(); });
    }
}

void SoftRenderer::StopAsyncThread()
{
    FlushAsyncRender();
    if (AsyncThreadRunning.load(std::memory_order_relaxed))
    {
        AsyncThreadRunning.store(false, std::memory_order_release);
        Platform::Semaphore_Post(AsyncStart);   // wake the loop so it can exit
        Platform::Thread_Wait(AsyncThread);
        Platform::Thread_Free(AsyncThread);
        AsyncThread = nullptr;
    }
}

// Barrier: wait for the in-flight render (frame N-1) to finish, then publish the
// buffer it wrote as the present buffer. Safe to call when nothing is in flight.
void SoftRenderer::FlushAsyncRender()
{
#ifdef LITEV_SOFT2D_DEPTH2
    // Depth-2: drain every in-flight ring frame (Stop/Reset/PreSavestate must leave no render
    // touching emu state). Each AsyncDone corresponds to the oldest (ring[PresentIdx]).
    while (P2InFlight > 0)
    {
        const double _t0 = LSP_NOW();
        Platform::Semaphore_Wait(AsyncDone);
        LSP_ADD(EmuBarrier, LSP_NOW() - _t0);
        AsyncPresentBuf = P2Ring[P2PresentIdx].targetBuf;
        P2PresentIdx = (P2PresentIdx + 1) % 3;
        P2InFlight--;
        AsyncEverProduced = true;
    }
#endif
    if (AsyncInFlight)
    {
        const double _t0 = LSP_NOW();
        Platform::Semaphore_Wait(AsyncDone);
        LSP_ADD(EmuBarrier, LSP_NOW() - _t0);
        AsyncInFlight = false;
        AsyncPresentBuf = AsyncTargetBuf;
        AsyncEverProduced = true;
    }
}

// Depth-1 async VBlank: barrier the previous frame, snapshot the emu-mutable render
// inputs for THIS frame, then signal the render thread and return immediately so the
// emu can start frame N+1. emu frame time -> max(emu, render) instead of emu+render.
void SoftRenderer::VBlank()
{
    if (!S2DDeferActive)
        return;   // nothing was snapshotted this frame (e.g. frameskip)

#ifdef LITEV_SOFT2D_DEPTH2
    // (a) BARRIER. pipedepth cached here (per-VBlank read, cheap). Depth-1: wait the single
    // in-flight render every VBlank (byte-identical). Depth-2: block ONLY when 2 are already
    // in flight -- wait AsyncDone (drain the OLDEST render = ring[PresentIdx]) and present it.
    // Wait(AsyncDone) acquires that render's framebuffer writes before we publish it. This is
    // the frame-ahead: on a balanced frame the emu rarely reaches 2-in-flight-and-blocked.
    if (!PipeDepth)   // read ONCE, cached for the session
    {
        PipeDepth = litevReadPipeDepth();
        // Self-report the resolved depth so a gate run is never ambiguous about which
        // mode it exercised (the prop is read at the first VBlank and depth-1 is the
        // default; a setprop after launch is ignored).
        Platform::Log(Platform::Info, "LITEV_PIPEDEPTH resolved=%d\n", PipeDepth);
    }
    if (PipeDepth == 2)
    {
        if (P2InFlight >= 2)
        {
            const double _t0 = LSP_NOW();
            Platform::Semaphore_Wait(AsyncDone);
            LSP_ADD(EmuBarrier, LSP_NOW() - _t0);
            AsyncPresentBuf = P2Ring[P2PresentIdx].targetBuf;
            P2PresentIdx = (P2PresentIdx + 1) % 3;
            P2InFlight--;
            AsyncEverProduced = true;
        }
    }
    else
        FlushAsyncRender();
#else
    // (a) BARRIER: wait for the previous frame's render, publish it as present.
    FlushAsyncRender();
#endif

    const double _lspSnap0 = LSP_NOW();

    StartAsyncThread();

    if (!S2DBandsInit) InitBands();

    // (b) Snapshot the emu-mutable state the async render reads. The per-scanline
    // regs (LineSnap/SprSnap/FrameSnap) are copied into render-owned buffers; palette
    // + OAM are snapshotted; VRAMFlat is built here (on the emu thread, coherent for
    // frame N) and band units are seeded with frame-N regs. The 3D is NOT snapshotted
    // here — the async render consumes it directly via GetLine (off the emu thread).
#ifdef LITEV_SOFT2D_DEPTH2
    // Part 2: pick this frame's parity slot (toggle every VBlank -- the 2D always renders),
    // write all 2D snapshots into it, and (below, after SyncVRAM) snapshot the flat BG/OBJ
    // VRAM into the SAME parity. Capture AsyncSnapSlot for the render further down.
    SnapParity ^= 1;
    memcpy(FrameSnapR[SnapParity], FrameSnap, sizeof(FrameSnap));
    static_cast<SoftRenderer2D*>(Rend2D_A.get())->CopyLineSnaps(SnapParity);
    static_cast<SoftRenderer2D*>(Rend2D_B.get())->CopyLineSnaps(SnapParity);
    memcpy(PaletteSnap[SnapParity], GPU.Palette, sizeof(PaletteSnap[SnapParity]));
    memcpy(OAMSnap[SnapParity], GPU.OAM, sizeof(OAMSnap[SnapParity]));
#else
    memcpy(FrameSnapR, FrameSnap, sizeof(FrameSnap));
    static_cast<SoftRenderer2D*>(Rend2D_A.get())->CopyLineSnaps();
    static_cast<SoftRenderer2D*>(Rend2D_B.get())->CopyLineSnaps();
    memcpy(PaletteSnap, GPU.Palette, sizeof(PaletteSnap));
    memcpy(OAMSnap, GPU.OAM, sizeof(OAMSnap));
#endif

    auto* r2a = static_cast<SoftRenderer2D*>(Rend2D_A.get());
    auto* r2b = static_cast<SoftRenderer2D*>(Rend2D_B.get());
    r2a->SyncVRAM_BG(); r2a->SyncVRAM_OBJ();
    r2b->SyncVRAM_BG(); r2b->SyncVRAM_OBJ();
#ifdef LITEV_SOFT2D_DEPTH2
    // Part 1b: snapshot the now-coherent flat BG/OBJ/ext-pal VRAM into this frame's parity
    // bank (same SnapParity as the 2D snapshots above). The async 2D render reads it via the
    // *Read pointers redirected on the async thread (SetBGOBJReadShadow, keyed on AsyncSnapSlot).
    GPU.SnapshotBGOBJShadow(SnapParity);
#endif
    for (int b = 0; b < S2D_NBANDS; b++)
    {
#ifdef LITEV_SOFT2D_DEPTH2
        // Seed THIS frame's parity band-unit set (the render uses S2DBands[AsyncSnapSlot]).
        S2DBands[SnapParity][b].unit[0]->CopyRenderState(GPU.GPU2D_A);
        S2DBands[SnapParity][b].unit[1]->CopyRenderState(GPU.GPU2D_B);
#else
        S2DBands[b].unit[0]->CopyRenderState(GPU.GPU2D_A);
        S2DBands[b].unit[1]->CopyRenderState(GPU.GPU2D_B);
#endif
    }

    // (c) SIGNAL the render thread to raster frame N into the current back buffer.
    // Capture the buffer index now: FinishFrame will swap BackBuffer while the render
    // runs, but the render must keep writing the buffer we chose here.
#ifdef LITEV_SOFT2D_DEPTH2
    if (PipeDepth == 2)
    {
        // Depth-2 kick: capture frame N's per-frame resource keys into the ring, then kick.
        // The 2D thread pops the ring and sets AsyncTargetBuf/AsyncSnapSlot/3D-consume-bank
        // from it (NOT here -- there may be 2 in flight, each needing its own generation).
        // Framebuffer picked round-robin over the 3: with <=2 in flight + 1 presenting, the
        // RR target 3 frames back is free (its UI upload finished 2 presents ago). The
        // Semaphore_Post(AsyncStart) below is the RELEASE that publishes P2Ring[PushIdx]
        // (+ this frame's SnapParity/P2KickParity, written above) to the 2D thread's ACQUIRE.
        // ORDERING per frame: [barrier drains N-2] -> parity toggles + snapshot writes (b) ->
        // ring capture -> AsyncStart post -> (2D thread) ring pop + set keys + reads.
        int freshBuf = P2FrameBufRR;
        P2FrameBufRR = (P2FrameBufRR + 1) % 3;
        // consume3DParity = the P2 bank the 2D-N consumer must read = the parity 3D-N was
        // rastered into. 3D-N was kicked at frame N's VCount-215 RenderFrame (which toggled
        // P2KickParity mod-3); the next toggle is N+1's VCount 215, so at THIS VBlank(N)
        // P2KickParity still equals 3D-N's parity -- exactly the value Pipeline2LatchConsume
        // uses at depth-1. Fetched cross-class via the public getter.
        int consume3DParity = 0;   // tile renderer: 3D comes from GetLine, no plane parity
        P2Ring[P2PushIdx] = { freshBuf, SnapParity, consume3DParity };
        P2PushIdx = (P2PushIdx + 1) % 3;
        P2InFlight++;
        S2DDeferActive = false;
        LSP_ADD(EmuSnap, LSP_NOW() - _lspSnap0);
        Platform::Semaphore_Post(AsyncStart);
#ifdef LITEV_SOFTPROF
        LitevSP::Tick();
#endif
        return;   // emu emulates frame N+1 with up to 2 renders in flight
    }
#endif
    AsyncTargetBuf = BackBuffer;
    AsyncInFlight = true;
    S2DDeferActive = false;
#ifdef LITEV_SOFT2D_DEPTH2
    // Depth-1: the render reads the slot we just wrote (== SnapParity).
    AsyncSnapSlot = SnapParity;
#endif
    LSP_ADD(EmuSnap, LSP_NOW() - _lspSnap0);
    Platform::Semaphore_Post(AsyncStart);
#ifdef LITEV_SOFTPROF
    LitevSP::Tick();
#endif
    // (d) return immediately — emu emulates frame N+1 while the render thread runs.
}
#endif

void SoftRenderer::DrawScanlineA(u32 line, u32* dst, const u32* src2d, u32 dispcnt, u16 mbright)
{
    switch ((dispcnt >> 16) & 0x3)
    {
    case 0: // screen off
        {
            for (int i = 0; i < 256; i++)
                dst[i] = 0x3F3F3F;
        }
        return;

    case 1: // regular display
        {
            for (int i = 0; i < 256; i+=2)
                *(u64*)&dst[i] = *(u64*)&src2d[i];
        }
        break;

    case 2: // VRAM display
        {
            u32 vrambank = (dispcnt >> 18) & 0x3;
            if (GPU.VRAMMap_LCDC & (1<<vrambank))
            {
                u16* vram = (u16*)GPU.VRAM[vrambank];
                vram = &vram[line * 256];

                for (int i = 0; i < 256; i++)
                {
                    u16 color = vram[i];
                    u8 r = (color & 0x001F) << 1;
                    u8 g = (color & 0x03E0) >> 4;
                    u8 b = (color & 0x7C00) >> 9;

                    dst[i] = r | (g << 8) | (b << 16);
                }
            }
            else
            {
                for (int i = 0; i < 256; i++)
                    dst[i] = 0;
            }
        }
        break;

    case 3: // FIFO display
        {
            for (int i = 0; i < 256; i++)
            {
                u16 color = GPU.DispFIFOBuffer[i];
                u8 r = (color & 0x001F) << 1;
                u8 g = (color & 0x03E0) >> 4;
                u8 b = (color & 0x7C00) >> 9;

                dst[i] = r | (g << 8) | (b << 16);
            }
        }
        break;
    }

    ApplyMasterBrightness(mbright, dst);
}

void SoftRenderer::DrawScanlineB(u32 line, u32* dst, const u32* src2d, u32 dispcnt, u16 mbright)
{
    switch ((dispcnt >> 16) & 0x1)
    {
    case 0: // screen off
        {
            for (int i = 0; i < 256; i++)
                dst[i] = 0xFF3F3F3F;
        }
        return;

    case 1: // regular display
        {
            for (int i = 0; i < 256; i+=2)
                *(u64*)&dst[i] = *(u64*)&src2d[i];
        }
        break;
    }

    ApplyMasterBrightness(mbright, dst);
}

void SoftRenderer::DoCapture(u32 line, const u32* srcA2d, const u32* src3d)
{
    u32 captureCnt = GPU.CaptureCnt;

    u32 width, height;
    u32 sz = (captureCnt >> 20) & 0x3;
    if (sz == 0)
    {
        width = 128;
        height = 128;
    }
    else
    {
        width = 256;
        height = 64 * sz;
    }

    if (line >= height)
        return;

    u32 dstvram = (captureCnt >> 16) & 0x3;
    if (!(GPU.VRAMMap_LCDC & (1<<dstvram)))
        return;

    u16* dst = (u16*)GPU.VRAM[dstvram];
    u32 dstaddr = (((captureCnt >> 18) & 0x3) << 14) + (line * width);
    dst += (dstaddr & 0xFFFF);

    const u32* srcA;
    if (captureCnt & (1<<24))
        srcA = src3d;
    else
        srcA = srcA2d;

    u16* srcB = nullptr;
    if (captureCnt & (1<<25))
        srcB = GPU.DispFIFOBuffer;
    else
    {
        u32 dispcnt = GPU.GPU2D_A.DispCnt;
        u32 srcvram = (dispcnt >> 18) & 0x3;
        if (GPU.VRAMMap_LCDC & (1<<srcvram))
        {
            srcB = (u16*)GPU.VRAM[srcvram];

            u32 offset = line * 256;
            if (((dispcnt >> 16) & 0x3) != 2)
                offset += (((captureCnt >> 26) & 0x3) << 14);

            srcB += (offset & 0xFFFF);
        }
    }

    static_assert(VRAMDirtyGranularity == 512);
    GPU.VRAMDirty[dstvram][(dstaddr * 2) / VRAMDirtyGranularity] = true;

    switch ((captureCnt >> 29) & 0x3)
    {
    case 0: // source A
        {
            for (u32 i = 0; i < width; i++)
            {
                u32 val = srcA[i];

                u32 r = (val >> 1) & 0x1F;
                u32 g = (val >> 9) & 0x1F;
                u32 b = (val >> 17) & 0x1F;
                u32 a = ((val >> 24) != 0) ? 0x8000 : 0;

                dst[i] = r | (g << 5) | (b << 10) | a;
            }
        }
        break;

    case 1: // source B
        {
            if (srcB)
            {
                for (u32 i = 0; i < width; i++)
                    dst[i] = srcB[i];
            }
            else
            {
                for (u32 i = 0; i < width; i++)
                    dst[i] = 0;
            }
        }
        break;

    case 2: // sources A+B
    case 3:
        {
            u32 eva = captureCnt & 0x1F;
            u32 evb = (captureCnt >> 8) & 0x1F;

            // checkme
            if (eva > 16) eva = 16;
            if (evb > 16) evb = 16;

            if (srcB)
            {
                for (u32 i = 0; i < width; i++)
                {
                    u32 val = srcA[i];

                    u32 rA = (val >> 1) & 0x1F;
                    u32 gA = (val >> 9) & 0x1F;
                    u32 bA = (val >> 17) & 0x1F;
                    u32 aA = ((val >> 24) != 0) ? 1 : 0;

                    val = srcB[i];

                    u32 rB = val & 0x1F;
                    u32 gB = (val >> 5) & 0x1F;
                    u32 bB = (val >> 10) & 0x1F;
                    u32 aB = val >> 15;

                    u32 rD = ((rA * aA * eva) + (rB * aB * evb) + 8) >> 4;
                    u32 gD = ((gA * aA * eva) + (gB * aB * evb) + 8) >> 4;
                    u32 bD = ((bA * aA * eva) + (bB * aB * evb) + 8) >> 4;
                    u32 aD = (eva>0 ? aA : 0) | (evb>0 ? aB : 0);

                    if (rD > 0x1F) rD = 0x1F;
                    if (gD > 0x1F) gD = 0x1F;
                    if (bD > 0x1F) bD = 0x1F;

                    dst[i] = rD | (gD << 5) | (bD << 10) | (aD << 15);
                }
            }
            else
            {
                for (u32 i = 0; i < width; i++)
                {
                    u32 val = srcA[i];

                    u32 rA = (val >> 1) & 0x1F;
                    u32 gA = (val >> 9) & 0x1F;
                    u32 bA = (val >> 17) & 0x1F;
                    u32 aA = ((val >> 24) != 0) ? 1 : 0;

                    u32 rD = ((rA * aA * eva) + 8) >> 4;
                    u32 gD = ((gA * aA * eva) + 8) >> 4;
                    u32 bD = ((bA * aA * eva) + 8) >> 4;
                    u32 aD = (eva>0 ? aA : 0);

                    dst[i] = rD | (gD << 5) | (bD << 10) | (aD << 15);
                }
            }
        }
        break;
    }
}

void SoftRenderer::ApplyMasterBrightness(u16 regval, u32* dst)
{
    u16 mode = regval >> 14;
    if (mode == 1)
    {
        // up
        u32 factor = regval & 0x1F;
        if (factor > 16) factor = 16;

#if defined(LITEV_NEON_RENDERER) && defined(__aarch64__)
        GPU2DNeon::BrightnessUp(dst, 256, factor);
#else
        for (int i = 0; i < 256; i++)
            dst[i] = ColorBrightnessUp(dst[i], factor, 0x0);
#endif
    }
    else if (mode == 2)
    {
        // down
        u32 factor = regval & 0x1F;
        if (factor > 16) factor = 16;

#if defined(LITEV_NEON_RENDERER) && defined(__aarch64__)
        GPU2DNeon::BrightnessDown(dst, 256, factor);
#else
        for (int i = 0; i < 256; i++)
            dst[i] = ColorBrightnessDown(dst[i], factor, 0xF);
#endif
    }
}

void SoftRenderer::ExpandColor(u32* dst)
{
    // convert to 32-bit BGRA
    // note: 32-bit RGBA would be more straightforward, but
    // BGRA seems to be more compatible (Direct2D soft, cairo...)
#if defined(LITEV_NEON_RENDERER) && defined(__aarch64__)
    GPU2DNeon::ConvertToBGRA(dst, 256);
#else
    for (int i = 0; i < 256; i+=2)
    {
        u64 c = *(u64*)&dst[i];

        u64 r = (c << 18) & 0xFC000000FC0000;
        u64 g = (c << 2) & 0xFC000000FC00;
        u64 b = (c >> 14) & 0xFC000000FC;
        c = r | g | b;

        *(u64*)&dst[i] = c | ((c & 0x00C0C0C000C0C0C0) >> 6) | 0xFF000000FF000000;
    }
#endif
}


bool SoftRenderer::GetFramebuffers(void** top, void** bottom)
{
    int frontbuf = BackBuffer ^ 1;
#ifdef LITEV_SOFT2D_THREADED
    // In async mode the swapped-in "front" buffer may still be mid-render; return
    // the last buffer the render thread actually COMPLETED (published at the last
    // VBlank barrier). Depth-1: this is frame N-1 while frame N renders.
    frontbuf = AsyncPresentBuf;
#endif
    *top = Framebuffer[frontbuf][0];
    *bottom = Framebuffer[frontbuf][1];
    return true;
}

}
