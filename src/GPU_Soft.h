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

#ifndef GPU_SOFT_H
#define GPU_SOFT_H

#include <atomic>

#include "GPU.h"
#include "Platform.h"
#include "GPU2D_Soft.h"
#include "GPU3D_Soft.h"

namespace melonDS
{

class SoftRenderer : public Renderer
{
public:
    explicit SoftRenderer(melonDS::NDS& nds);
    ~SoftRenderer() override;
    bool Init() override { return true; }
    void Reset() override;
    void Stop() override;

    void PreSavestate() override;
    void PostSavestate() override;

    void SetRenderSettings(RendererSettings& settings) override;

    void DrawScanline(u32 line) override;
    void DrawSprites(u32 line) override;

#ifdef LITEV_SOFT2D_THREADED
    void VBlank() override;
#else
    void VBlank() override {};
#endif
    void VBlankEnd() override {};

    void AllocCapture(u32 bank, u32 start, u32 len) override {};
    void SyncVRAMCapture(u32 bank, u32 start, u32 len, bool complete) override {};

    bool GetFramebuffers(void** top, void** bottom) override;

private:
    friend class SoftRenderer2D;
    friend class SoftRenderer3D;

    u32* Framebuffer[2][2];

    u32* Output3D;
    alignas(8) u32 Output2D[2][256];

#ifdef LITEV_SOFT2D_THREADED
    // Deferred (DraStic-model) software 2D: snapshot the final-composite per-scanline
    // state on the emu thread; the whole frame's raster+composite runs at VBlank, off
    // the per-scanline critical path (later banded across helper threads).
    struct FrameLineSnap
    {
        u32 DispCntA, DispCntB;
        u16 MasterBrightnessA, MasterBrightnessB;
        u8  ScreenSwap;
        u8  ScreensEnabled;
        u8  CaptureEnable;
        u8  Valid;
    };
    FrameLineSnap FrameSnap[192];
    // Render-owned copy (see async pipeline): the emu thread copies FrameSnap ->
    // FrameSnapR at VBlank, and the async render thread reads only FrameSnapR.
    FrameLineSnap FrameSnapR[192];
    // 3D output copied per line DURING the visible period, keeping the threaded-3D
    // GetLine semaphore consumption in lockstep with the render thread (the deferred
    // 2D batch at VBlank then reads these copies instead of re-calling GetLine, which
    // would race the 3D render thread's frame schedule).
    alignas(8) u32 Snap3D[192][256];
    // Render-owned copy of Snap3D (emu copies at VBlank; async render reads this).
    alignas(8) u32 Snap3DR[192][256];
    // Full-frame per-engine 2D output, so engine A and engine B (independent GPU2D
    // units + SoftRenderer2D instances + buffers) can render in parallel before the
    // sequential composite reads both. (M2 step 1: 2-way A||B; later: line bands.)
    alignas(8) u32 BandOut2D[2][192][256];
    bool S2DDeferActive = false;   // set per-frame: no capture/edge → safe to defer
    void SnapshotCompositeLine(u32 line);

    // N-way banded raster (DraStic model): each band renders a disjoint line range
    // for BOTH engines using PRIVATE GPU2D units (seeded from the main frame state
    // via CopyRenderState, then per-line snapshot overrides) + private scanline temp
    // buffers, so all bands run concurrently on idle cores with no shared mutable
    // render state. They read the shared read-only snapshots + shared VRAM (emu is
    // blocked during the batch).
    // The 2D raster is small and is NOT on the critical path: it gets a whole emu
    // frame of slack (measured emu 2D barrier = 0.00 ms) and needs only ~11 ms of CPU.
    // The 3D raster IS the critical path (measured: ~26 ms wall against a ~15 ms
    // budget, emu blocks ~9 ms/frame at Finish3DRendering). Every 2D helper thread
    // therefore STEALS a core from the 3D bands during the raster phase — and the old
    // NBANDS=2 path also spawned+joined a std::thread EVERY frame.
    // NBANDS=1: one persistent thread, no per-frame spawn, 3 full cores for the 3D.
    static constexpr int S2D_NBANDS = 1;
    struct S2DBand
    {
        std::unique_ptr<GPU2D> unit[2];
        std::unique_ptr<Renderer2D> rend[2];   // SoftRenderer2D bound to unit[]
    };
    S2DBand S2DBands[S2D_NBANDS];
    bool S2DBandsInit = false;
    void InitBands();
    void RenderBand(int bi, u32 y0, u32 y1);

    // ---- Depth-1 async pipeline (DraStic model) ----
    // At VBlank the emu thread snapshots the frame's remaining emu-mutable render
    // inputs (palette/OAM into these buffers; VRAM coherence + band-unit register
    // seeding done there too), then SIGNALS a persistent render thread to raster
    // frame N off the critical path while the emu immediately emulates frame N+1.
    // A barrier at the NEXT VBlank waits for frame N's render before reusing state.
    alignas(8) u8 PaletteSnap[2*1024];   // GPU.Palette snapshot (read by async draws)
    alignas(8) u8 OAMSnap[2*1024];       // GPU.OAM snapshot (read by async draws)

    Platform::Thread* AsyncThread = nullptr;
    Platform::Semaphore* AsyncStart = nullptr;   // emu -> render: "render this frame"
    Platform::Semaphore* AsyncDone  = nullptr;   // render -> emu: "frame complete"
    std::atomic<bool> AsyncThreadRunning { false };
    bool AsyncInFlight = false;           // a render was signaled and not yet barriered
    bool AsyncEverProduced = false;       // at least one async frame completed
    int  AsyncTargetBuf = 0;              // framebuffer index the render thread writes
    // last COMPLETED framebuffer (returned by GetFramebuffers). Inits to 1 so the
    // first present (before any async frame finishes) returns the untouched buffer,
    // not buffer 0 which the first async frame is concurrently rendering into.
    int  AsyncPresentBuf = 1;


    void StartAsyncThread();              // lazy-create the persistent render thread
    void StopAsyncThread();               // flush in-flight + join (Stop/dtor)
    void AsyncRenderThreadFunc();         // the persistent loop
    void AsyncRenderFrame();              // banded raster+composite+capture (off-thread)
    void FlushAsyncRender();              // barrier: wait in-flight done + publish present
#endif

    void DrawScanlineA(u32 line, u32* dst, const u32* src2d, u32 dispcnt, u16 mbright);
    void DrawScanlineB(u32 line, u32* dst, const u32* src2d, u32 dispcnt, u16 mbright);

    void DoCapture(u32 line, const u32* srcA2d, const u32* src3d);

    void ApplyMasterBrightness(u16 regval, u32* dst);
    void ExpandColor(u32* dst);
};

}

#endif // GPU_SOFT_H
