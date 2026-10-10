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

#ifndef GPU_HYBRID_H
#define GPU_HYBRID_H

#include <atomic>
#include <functional>
#include "GPU_Soft.h"
#include "GPU3D_OpenGL.h"

namespace melonDS
{

class GLThread3D;
class GLWorker;

// Hybrid renderer: 3D on the GPU at Nx (GLRenderer3D), 2D on the CPU at native
// resolution (the threaded software 2D in descriptor mode), one GPU merge pass at Nx.
// This is classic melonDS's OpenGL mode (removed upstream in ba317e2e) on top of the
// unified Renderer API. All GL work runs on the thread that owns the context (the emu
// thread): the 3D at VCount 215, the merge in GetFramebuffers.
//
// Display capture is done on the CPU at 1x into emulated VRAM (the 3D is read back on
// capture frames only), so every capture consumer sees plain VRAM. A captured image
// shown again (display mode 2, BG bitmap) is therefore 1x; "OpenGL (hi-res 2D)"
// (GLRenderer) keeps captures at Nx.
class HybridRenderer : public SoftRenderer
{
public:
    explicit HybridRenderer(melonDS::NDS& nds);
    ~HybridRenderer() override;
    bool Init() override;
    void Reset() override;

    void PreSavestate() override;
    void PostSavestate() override;

    void SetRenderSettings(RendererSettings& settings) override;

    bool GetFramebuffers(void** top, void** bottom) override;
    void Start3DRendering() override;
    // Present directly into the app's frame texture (instead of GetFramebuffers + a blit):
    // top screen at row 0, bottom screen at row bottomY.
    void PresentInto(GLuint dstTex, int bottomY);
    // Same, but the upload + merge run on a present thread with its own shared context
    // (GPU-side wait on the 3D fence): pre() runs there first, post() after the merge commands are issued (fence +
    // hand-off). Falls back to PresentInto when there is no GL 3D thread or the PBO path is on.
    // debug.litev.hybasync=0: always synchronous.
    void PresentIntoAsync(GLuint dstTex, int bottomY, std::function<void()> pre, std::function<void()> post);
    void WaitPresent();   // every queued async present has run
    void Finish3DRendering() override;
    void Restart3DRendering() override;


protected:
    int HybridCurrentTag() override;
    void HybridKick(int b) override;
    void HybridReadback3D(u32* dst) override;

private:
    GLThread3D* Thread3D();
    GLRenderer3D* GL3D();
    void Wait3D();             // the last 3D job is done (its colour buffer + fence are valid)
    void Sync3D(int colorIdx); // make this context's GPU commands wait for that 3D render
    void SetScale(int scale);

    int Scale = 0;
    GLuint MergeShader = 0;
    GLint ScaleULoc = -1, SingleULoc = -1, OriginULoc = -1, FastULoc = -1, FastEvyULoc = -1;
    GLuint PresentFB = 0;
    void Merge(GLuint fbo, int single, int bottomY);
    // fb: framebuffer slot, tag: its 3D colour ring index, vao: this context's VAO,
    // sync: wait (GPU-side) for that 3D render first (not needed on the GL thread)
    void MergeSlot(GLuint fbo, int single, int bottomY, int fb, int tag, GLuint vao, bool sync);
    GLuint EmptyVAO = 0;
#ifdef LITEV_HYB_MERGE_1X_2D
    // a screen without 3D merges at 1x here, then is blitted up to Nx (made by the merging context)
    GLuint Merge1xFB = 0, Merge1xTex = 0;
#endif
    std::unique_ptr<GLWorker> Present;        // async present thread (shared EGL context)
    GLuint GLThreadFB = 0, GLThreadVAO = 0;   // its objects
    u64 MergeSeq = 0, SlotMergeSeq[NFB] {};
    std::atomic<u64> MergeDone { 0 };
    // 513x192x2 RGBA8UI + staging buffer per framebuffer slot (a slot is reused 3 frames
    // later, so an upload never targets a texture an earlier merge may still be reading)
    GLuint DescTex[NFB] {};
    GLuint DescPBO[NFB] {};
    GLuint OutTex[2] {};           // Nx, 2 layers (top, bottom), like GLRenderer's FPOutputTex
    GLuint OutFB[2] {};
    int OutIdx = 0;
    GLuint ReadFB = 0, DownFB = 0, DownTex = 0;   // capture readback at 1x
    u8 ReadBuf[256 * 192 * 4];
#ifdef LITEV_HYB_CAPTURE_ASYNC
    // capture readback without a GPU stall: each capture frame starts its 3D's read into a
    // PBO and uses the one started on the previous frame (capture 3D one frame late)
    // a ring of reads: a capture uses the one started debug.litev.caplag (default 2) frames ago
    GLuint CapPBO[4] {};   // [3]: LITEV_FF_CAP_PREFETCH
    GLsync CapFence[4] {};
    u32 CapFrameOf[4] {};            // NumFrames each read was started on
#ifdef LITEV_HYB_CAPTURE_OFFTHREAD
    std::atomic<bool> CapBusy[4] {}; // its read is still being finished on the GL 3D thread
    u32 CapOut[4][256 * 192];        // the finished reads, converted
    int CapPrevSlot = -1;            // read started by the last capture job, not finished yet
    GLuint CapGLReadFB = 0, CapGLDownFB = 0;   // the GL 3D thread's own (FBOs aren't shared)
    void CapKickGL(int k, int color);
#ifdef LITEV_FF_CAP_PREFETCH
    int PrefColor = -1;        // the 3D colour buffer read into slot 3 (-1: none valid)
    u32 LastCapFrame = 0;      // NumFrames of the last capture readback
#endif
    void CapFinishGL(int k);
#endif
    int CapNext = 0;
    void CapKick(int pbo);
    void CapConvert(const u8* src, u32* dst);
#endif
    // emu-thread CPU time per 60 frames (logged as LITEV_HYB)
    double ProfGL3D = 0, ProfWait = 0, ProfMerge = 0, ProfUpload = 0, ProfSync = 0, ProfDraw = 0, ProfReadback = 0;
    int ProfFrames = 0;
};

}

#endif // GPU_HYBRID_H
