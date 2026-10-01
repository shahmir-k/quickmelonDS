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

#if defined(LITEV_NEON_RENDERER) && defined(__aarch64__)
#include "GPU2D_NEON.h"
#endif


namespace melonDS
{

SoftRenderer::SoftRenderer(melonDS::NDS& nds)
    : Renderer(nds.GPU)
{
    const size_t len = 256 * 192;
    Framebuffer[0][0] = new u32[len];
    Framebuffer[0][1] = new u32[len];
    Framebuffer[1][0] = new u32[len];
    Framebuffer[1][1] = new u32[len];
    BackBuffer = 0;

    Rend2D_A = std::make_unique<SoftRenderer2D>(GPU.GPU2D_A, *this);
    Rend2D_B = std::make_unique<SoftRenderer2D>(GPU.GPU2D_B, *this);
    Rend3D = std::make_unique<SoftRenderer3D>(GPU.GPU3D, *this);

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
}


void SoftRenderer::PreSavestate()
{
#ifdef LITEV_SOFT2D_THREADED
    // ensure no async 2D render is reading emu state while it is (de)serialized
    FlushAsyncRender();
#endif
    auto rend3d = dynamic_cast<SoftRenderer3D*>(Rend3D.get());
    if (rend3d->IsThreaded())
        rend3d->SetupRenderThread();
}

void SoftRenderer::PostSavestate()
{
    auto rend3d = dynamic_cast<SoftRenderer3D*>(Rend3D.get());
    if (rend3d->IsThreaded())
        rend3d->EnableRenderThread();
}


void SoftRenderer::SetRenderSettings(RendererSettings& settings)
{
    auto rend3d = dynamic_cast<SoftRenderer3D*>(Rend3D.get());
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
    for (int b = 0; b < S2D_NBANDS; b++)
    {
        for (int e = 0; e < 2; e++)
        {
            S2DBands[b].unit[e] = std::make_unique<GPU2D>((u32)e, GPU);
            S2DBands[b].rend[e] = std::make_unique<SoftRenderer2D>(*S2DBands[b].unit[e], *this);
            static_cast<SoftRenderer2D*>(S2DBands[b].rend[e].get())->Reset();
        }
    }
    S2DBandsInit = true;
}

// Render a disjoint line range [y0,y1) for BOTH engines into BandOut2D, using this
// band's PRIVATE units/renderers, reading the shared main snapshots. Runs on a
// helper thread; no shared mutable render state with the other bands.
void SoftRenderer::RenderBand(int bi, u32 y0, u32 y1)
{
    auto* mainA = static_cast<SoftRenderer2D*>(Rend2D_A.get());
    auto* mainB = static_cast<SoftRenderer2D*>(Rend2D_B.get());
    auto* rA = static_cast<SoftRenderer2D*>(S2DBands[bi].rend[0].get());
    auto* rB = static_cast<SoftRenderer2D*>(S2DBands[bi].rend[1].get());
    auto& fsR = FrameSnapR;
    const u8* const palSnap = PaletteSnap;
    const u8* const oamSnap = OAMSnap;

    const double _lspB0 = LSP_NOW();
    for (u32 line = y0; line < y1; line++)
    {
        // DraStic model: consume the 3D line HERE, on the async render thread, paced by
        // the 3D render thread's per-scanline semaphore — NOT on the emu thread. Must
        // GetLine EVERY line (0..191) to keep the semaphore count balanced even for
        // skipped lines. (Single-threaded async render — S2D_NBANDS=1 — so in-order.)
        const double _lspG0 = LSP_NOW();
        u32* l3d = Rend3D->GetLine(line);
        LSP_ADD(S2DBlock[bi], LSP_NOW() - _lspG0);

        // Read the render-owned snapshot copies (frame N) — the emu thread is
        // concurrently overwriting the live FrameSnap/LineSnap/SprSnap for frame N+1,
        // so we must NOT touch those here.
        FrameLineSnap& f = fsR[line];
        if (!f.Valid) continue;

        // --- BG/OBJ raster into this band's per-engine line buffers ---
        rA->Cur3DLine = l3d;
        rA->CurOAM = oamSnap;
        rA->CurPalette = palSnap;
        rA->DrawSpritesDeferred(mainA->SprSnapR[line], line);
        rA->DrawScanlineDeferred(mainA->LineSnapR[line], line, BandOut2D[0][line]);

        rB->Cur3DLine = l3d;
        rB->CurOAM = oamSnap;
        rB->CurPalette = palSnap;
        rB->DrawSpritesDeferred(mainB->SprSnapR[line], line);
        rB->DrawScanlineDeferred(mainB->LineSnapR[line], line, BandOut2D[1][line]);

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
        const double _t0 = LSP_NOW();
        AsyncRenderFrame();
        const double _t1 = LSP_NOW();
        LSP_ADD(S2DWall, _t1 - _t0);
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

    // (a) BARRIER: wait for the previous frame's render, publish it as present.
    FlushAsyncRender();

    const double _lspSnap0 = LSP_NOW();

    StartAsyncThread();

    if (!S2DBandsInit) InitBands();

    // (b) Snapshot the emu-mutable state the async render reads. The per-scanline
    // regs (LineSnap/SprSnap/FrameSnap) are copied into render-owned buffers; palette
    // + OAM are snapshotted; VRAMFlat is built here (on the emu thread, coherent for
    // frame N) and band units are seeded with frame-N regs. The 3D is NOT snapshotted
    // here — the async render consumes it directly via GetLine (off the emu thread).
    memcpy(FrameSnapR, FrameSnap, sizeof(FrameSnap));
    static_cast<SoftRenderer2D*>(Rend2D_A.get())->CopyLineSnaps();
    static_cast<SoftRenderer2D*>(Rend2D_B.get())->CopyLineSnaps();
    memcpy(PaletteSnap, GPU.Palette, sizeof(PaletteSnap));
    memcpy(OAMSnap, GPU.OAM, sizeof(OAMSnap));

    auto* r2a = static_cast<SoftRenderer2D*>(Rend2D_A.get());
    auto* r2b = static_cast<SoftRenderer2D*>(Rend2D_B.get());
    r2a->SyncVRAM_BG(); r2a->SyncVRAM_OBJ();
    r2b->SyncVRAM_BG(); r2b->SyncVRAM_OBJ();
    for (int b = 0; b < S2D_NBANDS; b++)
    {
        S2DBands[b].unit[0]->CopyRenderState(GPU.GPU2D_A);
        S2DBands[b].unit[1]->CopyRenderState(GPU.GPU2D_B);
    }

    // (c) SIGNAL the render thread to raster frame N into the current back buffer.
    // Capture the buffer index now: FinishFrame will swap BackBuffer while the render
    // runs, but the render must keep writing the buffer we chose here.
    AsyncTargetBuf = BackBuffer;
    AsyncInFlight = true;
    S2DDeferActive = false;
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
