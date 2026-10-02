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

protected:
    friend class SoftRenderer2D;
    friend class SoftRenderer3D;

    // true while the 3D backend is the accurate SoftRenderer3D (runtime choice,
    // RendererSettings::Accurate3D). It renders into ONE colour buffer, which the async
    // 2D thread must not read while the next frame renders into it: in that mode the
    // emu thread copies the 3D lines into Snap3D at VBlank (see VBlank) and the 2D
    // thread reads the copy. The tile renderer has its own frame ring.
    bool AccurateBackend = false;
    void SetAccurateBackend(bool accurate);

#ifdef LITEV_SOFT2D_DEPTH2
    // Part 3: THREE framebuffers for the depth-2 flip (one rendering, one queued, one
    // presenting). Depth-1 (pipedepth==1) uses only [0]/[1] exactly as before -> byte-
    // identical. Allocated/freed/cleared in the ctor/dtor/Reset/Stop under the flag.
    u32* Framebuffer[3][2];
#else
    u32* Framebuffer[2][2];
#endif

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
        u16 XPos3D;          // BG0HOFS as seen by the 3D layer (hybrid merge)
    };
    FrameLineSnap FrameSnap[192];
    // Render-owned copy (see async pipeline): the emu thread copies FrameSnap ->
    // FrameSnapR at VBlank, and the async render thread reads only FrameSnapR.
#ifdef LITEV_SOFT2D_DEPTH2
    // Part 2: parity double-buffer the render-owned 2D snapshots so a depth-2 pipeline can
    // have the emu write frame N+1's inputs while the render reads frame N's. The emu writes
    // slot[SnapParity] at VBlank; the render reads slot[AsyncSnapSlot] (captured at VBlank
    // like AsyncTargetBuf). Byte-identical at depth-1 (one slot live; SnapParity==AsyncSnapSlot
    // per frame). SnapParity/AsyncSnapSlot also key the Part-1b flat BG/OBJ shadow bank.
    FrameLineSnap FrameSnapR[2][192];
    int SnapParity = 0;      // emu, toggled every VBlank (2D always renders)
    int AsyncSnapSlot = 0;   // captured at VBlank; the render/2D consumer reads this slot
#else
    FrameLineSnap FrameSnapR[192];
#endif
    // Accurate 3D backend only: the frame's 3D lines, copied by the emu thread at
    // VBlank (GetLine, which also keeps the threaded renderer's per-scanline semaphore
    // balanced) into the 2D snapshot slot the async render reads.
    alignas(8) u32 Snap3D[2][192][256];
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
#ifdef LITEV_SOFT2D_DEPTH2
    // Part 2/3: parity double-buffer the per-band private GPU2D units. CopyRenderState seeds
    // the emu-mutable 2D registers into them at VBlank; under depth-2 the render for frame N
    // uses S2DBands[AsyncSnapSlot] while the emu seeds S2DBands[SnapParity] for N+1. Byte-
    // identical at depth-1 (SnapParity==AsyncSnapSlot per frame, one set live).
    S2DBand S2DBands[2][S2D_NBANDS];
#else
    S2DBand S2DBands[S2D_NBANDS];
#endif
    bool S2DBandsInit = false;
    void InitBands();
    void RenderBand(int bi, u32 y0, u32 y1);

    // ---- Depth-1 async pipeline (DraStic model) ----
    // At VBlank the emu thread snapshots the frame's remaining emu-mutable render
    // inputs (palette/OAM into these buffers; VRAM coherence + band-unit register
    // seeding done there too), then SIGNALS a persistent render thread to raster
    // frame N off the critical path while the emu immediately emulates frame N+1.
    // A barrier at the NEXT VBlank waits for frame N's render before reusing state.
#ifdef LITEV_SOFT2D_DEPTH2
    alignas(8) u8 PaletteSnap[2][2*1024];   // [SnapParity] GPU.Palette snapshot
    alignas(8) u8 OAMSnap[2][2*1024];       // [SnapParity] GPU.OAM snapshot
#else
    alignas(8) u8 PaletteSnap[2*1024];   // GPU.Palette snapshot (read by async draws)
    alignas(8) u8 OAMSnap[2*1024];       // GPU.OAM snapshot (read by async draws)
#endif

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

#ifdef LITEV_SOFT2D_DEPTH2
    // Part 3: depth-2 in-flight ring. pipedepth==1 keeps the depth-1 barrier (byte-identical);
    // pipedepth==2 lets up to 2 renders be in flight so the emu runs a frame ahead, removing
    // the ~4.5 ms VBlank barrier bubble. Each in-flight frame captures ALL its per-frame
    // resource keys so the render (and later present) reference the right generation:
    //   targetBuf       -- the Framebuffer[] the render writes and the present reads
    //   snapSlot        -- 2D snapshot + BG/OBJ-flat parity (mod-2)
    //   consume3DParity -- 3D ColorBuffer/Depth/Attr bank (mod-3)
    // FIFO indexed by a monotonic frame counter mod 3 (<=2 in flight + 1 draining -> 3 slots
    // never collide). The emu writes ring[PushIdx] then posts AsyncStart (release); the 2D
    // thread pops ring[PopIdx] after Wait(AsyncStart) (acquire) -> the write is published. The
    // emu presents ring[PresentIdx] after Wait(AsyncDone) (the drained render is complete).
    struct P2InFlightFrame { int targetBuf; int snapSlot; int consume3DParity; };
    P2InFlightFrame P2Ring[3] {};
    int P2PushIdx = 0;        // emu: next ring slot to fill (mod 3)
    int P2PresentIdx = 0;     // emu: next ring slot to present/drain (mod 3)
    int P2PopIdx = 0;         // 2D thread: next ring slot to consume (mod 3)
    int P2InFlight = 0;       // emu: pushed-but-not-presented count (0..2)
    int P2FrameBufRR = 0;     // emu: round-robin framebuffer picker (mod 3)
    int PipeDepth = 0;        // debug.litev.pipedepth (1 or 2); 0=unread. Read ONCE at the
                              // first VBlank and cached for the session -- a mid-run 1<->2
                              // switch would leave the depth-1 in-flight state inconsistent
                              // with the depth-2 ring (mixing AsyncInFlight and P2InFlight).
    u64 PipeTrace3DHash = 0;  // async 2D thread: 3D lines consumed for the current render
    u64 PipeTraceSpriteTopHash = 0; // top sprite intermediates before BG composite
    u64 PipeTrace2DTopHash = 0; // async 2D thread: top BG/OBJ composite before final output
    u64 PipeTraceTopInputHash = 0; // render-owned top 2D snapshots consumed for the render
#endif

    // ---- Hybrid renderer support (HybridRenderer derives from this class) ----
    // Hybrid: engine A emits descriptors (SoftRenderer2D::HybridDesc), the 3D is merged
    // on the GPU. Per framebuffer slot b, HybFB[b] holds both screens (top, then bottom),
    // each 192 lines of HybStride words: 2 descriptor planes of 256 + 1 control word
    // (master brightness | display mode << 16 | line has descriptors << 18 | 3D x-scroll
    // in bits 24-31 and 23). HybHas3D[b][screen]: some line of that screen has descriptors
    // (else only plane 1 + the control column need uploading).
    static constexpr int HybStride = 256*2 + 1;
    bool HybHas3D[3][2] {};
    bool Hybrid = false;
    u32* HybFB[3] {};
    int HybTag[3] {};              // 3D colour-ring index the slot's frame pairs with
    u32* Hyb3D[2] {};              // capture frames: the 1x 3D read back (per snap slot)
    bool Hyb3DValid[2] {};
    virtual int HybridCurrentTag() { return 0; }
    // emu thread, at the 2D kick for framebuffer slot b: the hybrid maps a staging buffer
    // into HybMap[b]; the 2D thread copies the finished frame's descriptors into it, so
    // the emu thread's upload is just a GPU-side copy
    virtual void HybridKick(int b) {}
    u8* HybMap[3] {};
    void HybridStage(int b);
    virtual void HybridReadback3D(u32* dst) {}
    void HybridLine(u32 line, const FrameLineSnap& f, u32* descA, u32* descB, const u32* l3d, bool has3D);
    // LITEV_HYBRID_CHECK (headless self-test): the software path also builds engine A's
    // descriptor line, resolves it on the CPU with the same 3D line and counts pixels
    // that differ from the real composite. Must stay 0.
    bool HybridCheck = false;
    std::atomic<u64> HybCheckBad { 0 }, HybCheckLines { 0 };

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
