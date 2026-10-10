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

#include <string.h>
#include <algorithm>
#include "NDS.h"
#include "GPU_Hybrid.h"
#include "GLWorker.h"
#include "GLThread3D.h"
#include "LitevSoftProf.h"
#include <chrono>
#include <condition_variable>
#include <atomic>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#include <time.h>
#ifdef __ANDROID__
#include <EGL/egl.h>
#include <sched.h>
#include <sys/system_properties.h>
#include <sys/resource.h>
#include <pthread.h>
#endif

namespace melonDS
{

#include "OpenGL_shaders/HybridMergeVS.h"
#include "OpenGL_shaders/HybridMergeFS.h"

// Upload the descriptors straight from RAM on the emu thread (default). The PBO path (the
// 2D thread fills a mapped buffer, debug.litev.hybdirect=0) costs less emu CPU in the
// upload itself but more frame time on Mali: ABBA 72.5/73.2 fps vs 75.0/75.5 direct.
static bool HybDirect()
{
#ifdef __ANDROID__
    static const bool on = [] { char b[92] = {}; return !(__system_property_get("debug.litev.hybdirect", b) > 0 && atoi(b) == 0); }();
    return on;
#else
    return true;
#endif
}

static double HybNowMs()
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

#ifndef LITEV_SOFT2D_THREADED
#error "HybridRenderer needs the threaded software 2D (LITEV_SOFT2D_THREADED)"
#endif


GLThread3D* HybridRenderer::Thread3D() { return static_cast<GLThread3D*>(Rend3D.get()); }
GLRenderer3D* HybridRenderer::GL3D() { return Thread3D()->GL.get(); }
void HybridRenderer::Wait3D() { Thread3D()->Wait(); }
void HybridRenderer::Sync3D(int colorIdx)
{
    double t0 = HybNowMs();
    Thread3D()->WaitColor(colorIdx);
    ProfWait += HybNowMs() - t0;
    GLsync f = Thread3D()->Fence[colorIdx];
    if (f) glWaitSync(f, 0, GL_TIMEOUT_IGNORED);
}

HybridRenderer::HybridRenderer(melonDS::NDS& nds)
    : SoftRenderer(nds)
{
    // 3D on the GPU, on its own GL thread. No GLRenderer parent: captures stay in
    // emulated VRAM.
    Rend3D = std::make_unique<GLThread3D>(GPU.GPU3D);
    Hybrid = true;
    HybridCheck = false;
    for (int i = 0; i < NFB; i++) HybFB[i] = new u32[2 * 192 * HybStride]();
    for (int i = 0; i < 2; i++) Hyb3D[i] = new u32[256 * 192]();
}

HybridRenderer::~HybridRenderer()
{
    WaitPresent();       // an async present may still use the merge objects below
    if (GLThreadFB) Present->Run([this] { glDeleteFramebuffers(1, &GLThreadFB); glDeleteVertexArrays(1, &GLThreadVAO); }, true);
    Present.reset();
    StopAsyncThread();   // the 2D thread reads HybFB; stop it before GL teardown below
    glDeleteProgram(MergeShader);
    glDeleteVertexArrays(1, &EmptyVAO);
    glDeleteTextures(NFB, DescTex);
    glDeleteBuffers(NFB, DescPBO);
    glDeleteTextures(2, OutTex);
    glDeleteFramebuffers(2, OutFB);
    if (PresentFB) glDeleteFramebuffers(1, &PresentFB);
    glDeleteFramebuffers(1, &ReadFB);
#ifdef LITEV_HYB_CAPTURE_ASYNC
    glDeleteBuffers(4, CapPBO);
    for (GLsync& f : CapFence) if (f) { glDeleteSync(f); f = nullptr; }
#endif
    glDeleteFramebuffers(1, &DownFB);
    glDeleteTextures(1, &DownTex);
#ifdef LITEV_HYB_MERGE_1X_2D
    glDeleteFramebuffers(1, &Merge1xFB);
    glDeleteTextures(1, &Merge1xTex);
#endif
}

bool HybridRenderer::Init()
{
    if (!Rend3D->Init()) return false;
    // 4 colour buffers: the 3D a presented frame pairs with is up to 3 renders old
    // (async 2D pipeline at depth 2), and one more is being rendered.
    // glFinish: the new texture storage must be complete before the emu context uses it
    Thread3D()->Run([this] { GL3D()->SetColorRing(GLRenderer3D::MaxColorRing); glFinish(); }, true);

    // debug.litev.hybmp (default on): the merge in mediump (all values fit: colours <= 63,
    // products <= 63*32, coordinates < 2^15; 8-bit unorm fetches round exactly in fp16)
    std::string mergeFS = kHybridMergeFS;
    if (OpenGL::Prop("hybmp", 1))
        mergeFS.insert(mergeFS.find('\n') + 1, "precision mediump float;\nprecision mediump int;\n");
#ifdef LITEV_HYB_MERGE_FASTCOPY
    // fast lines without brightness copy the 3D texel instead of re-quantising it to 6 bits and
    // back: PW town 3x 400 MHz ~1 ms of GPU per frame. debug.litev.hybfastcopy=0 turns it off.
    if (OpenGL::Prop("hybfastcopy", 1))
        mergeFS.insert(mergeFS.find('\n') + 1, "#define FAST_COPY\n");
#endif
    if (OpenGL::Prop("hybdiv", 1))
        mergeFS.insert(mergeFS.find('\n') + 1, "#define NATIVE_VARYING\n");
    if (!OpenGL::CompileVertexFragmentProgram(MergeShader,
            kHybridMergeVS, mergeFS, "HybridMergeShader",
            {}, {{"oTopColor", 0}, {"oBottomColor", 1}}))
        return false;

    glUseProgram(MergeShader);
    glUniform1i(glGetUniformLocation(MergeShader, "DescTex"), 0);
    glUniform1i(glGetUniformLocation(MergeShader, "Tex3D"), 1);
    glUniform1i(glGetUniformLocation(MergeShader, "EdgeTex"), 2);
    EdgeULoc = glGetUniformLocation(MergeShader, "uEdge");
    ScaleULoc = glGetUniformLocation(MergeShader, "uScale");
    SingleULoc = glGetUniformLocation(MergeShader, "uSingle");
    OriginULoc = glGetUniformLocation(MergeShader, "uOrigin");
    FastULoc = glGetUniformLocation(MergeShader, "uFast");
    FastEvyULoc = glGetUniformLocation(MergeShader, "uFastEvy");

    glGenVertexArrays(1, &EmptyVAO);

    auto texParams = [](GLenum target)
    {
        glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    };

    glGenBuffers(NFB, DescPBO);
    for (GLuint b : DescPBO)
    {
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, b);
        glBufferData(GL_PIXEL_UNPACK_BUFFER, 2 * 192 * HybStride * 4, nullptr, GL_STREAM_DRAW);
    }
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    glGenTextures(NFB, DescTex);
    for (GLuint t : DescTex)
    {
        glBindTexture(GL_TEXTURE_2D_ARRAY, t);
        texParams(GL_TEXTURE_2D_ARRAY);
        glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8UI, HybStride, 192, 2, 0, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, HybFB[0]);
    }

    glGenTextures(2, OutTex);
    glGenFramebuffers(2, OutFB);
    for (int i = 0; i < 2; i++)
    {
        glBindTexture(GL_TEXTURE_2D_ARRAY, OutTex[i]);
        texParams(GL_TEXTURE_2D_ARRAY);
    }

    glGenTextures(1, &DownTex);
    glBindTexture(GL_TEXTURE_2D, DownTex);
    texParams(GL_TEXTURE_2D);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, 256, 192, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glGenFramebuffers(1, &DownFB);
    glBindFramebuffer(GL_FRAMEBUFFER, DownFB);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, DownTex, 0);
    glGenFramebuffers(1, &ReadFB);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    SetScale(1);
    Thread3D()->Run([this] { GL3D()->SetRenderSettings(1, false); glFinish(); }, true);
    return true;
}

void HybridRenderer::Reset()
{
    SoftRenderer::Reset();   // drains the 2D, resets the 2D renderers and the GL 3D
    for (int i = 0; i < NFB; i++) memset(HybFB[i], 0, 2 * 192 * HybStride * sizeof(u32));
}

void HybridRenderer::PreSavestate()
{
    FlushAsyncRender();
    Wait3D();
}

void HybridRenderer::PostSavestate()
{
    Rend3D->Reset();   // texture cache
#ifdef LITEV_HYB_CAPTURE_ASYNC
    // The capture readback ring holds 3D reads from before this savestate (NumFrames-tagged).
    // After a load (same or nearby NumFrames: netplay resync, a quick reload) a capture would
    // write that pre-load 3D into the restored VRAM. Drop them: the next capture starts a fresh
    // run (reads its own 3D). Done on save too, so the saving console and the one loading its
    // state capture the same pixels from here on. PreSavestate's Wait3D finished the GL jobs;
    // a read still pending in CapPrevSlot is finished by the next job and ignored.
    for (u32& f : CapFrameOf) f = GPU.NDS.NumFrames - 3;   // older than any caplag (<= 2)
#ifdef LITEV_FF_CAP_PREFETCH
    PrefColor = -1;
#endif
#endif
}

void HybridRenderer::SetRenderSettings(RendererSettings& settings)
{
    SetScale(std::max(1, settings.ScaleFactor));
    const bool better = settings.BetterPolygons;
    Thread3D()->Run([this, better] { GL3D()->SetRenderSettings(Scale, better); glFinish(); }, true);
}

void HybridRenderer::SetScale(int scale)
{
    if (scale == Scale) return;
    Scale = scale;

    const GLenum bufs[2] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
    for (int i = 0; i < 2; i++)
    {
        glBindTexture(GL_TEXTURE_2D_ARRAY, OutTex[i]);
        glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8, 256 * Scale, 192 * Scale, 2, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        glBindFramebuffer(GL_FRAMEBUFFER, OutFB[i]);
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, OutTex[i], 0, 0);
        glFramebufferTextureLayer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT1, OutTex[i], 0, 1);
        glDrawBuffers(2, bufs);
    }
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
}


void HybridRenderer::Start3DRendering()
{
    double t0 = HybNowMs();
    Rend3D->RenderFrame();   // emu: VRAM coherence + post the job
#ifdef LITEV_FF_CAP_PREFETCH
    // Frameskip (fast-forward) on a screen that uses display capture: the next drawn frame's
    // capture follows skipped frames (no reads in flight), so it read this 3D render back
    // synchronously on the emu thread (~8% of the PW title's fast-forward time). Read it now on
    // the GL 3D thread; HybridReadback3D takes it while it is still the current 3D (same colour
    // buffer, same conversion: same pixels). Not in a recording (KeepCaptures: every frame drawn,
    // reads are already in flight). debug.litev.capprefetch=0 turns it off.
    {
        static const bool on = OpenGL::Prop("capprefetch", 1) != 0;
        PrefColor = -1;
        if (on && GPU.FrameskipTarget > 0 && !GPU.KeepCapturesSeen && Thread3D()->Threaded
            && GPU.NDS.NumFrames - LastCapFrame < 16)
        {
            const int color = GL3D()->GetCurColor();
            LSP_WAIT("cap-pref-reuse", GPU.NDS.NumFrames, while (CapBusy[3].load(std::memory_order_acquire)) std::this_thread::yield());
            CapBusy[3].store(true, std::memory_order_relaxed);
            Thread3D()->Run([this, color] { CapKickGL(3, color); CapFinishGL(3); }, false);
            PrefColor = color;
        }
    }
#endif
    ProfGL3D += HybNowMs() - t0;
}

void HybridRenderer::Finish3DRendering()
{
    Rend3D->FinishRendering();   // LITEV_GL_NOSNAPCOPY: polygon-bank barrier; else a no-op
}

void HybridRenderer::Restart3DRendering()
{
    Rend3D->RestartFrame();
}

// emu thread, 2D kick for slot b: map its staging buffer for the 2D thread to fill
void HybridRenderer::HybridKick(int b)
{
    // an async present may still be reading this slot on the GL thread
    if (Present && MergeDone.load(std::memory_order_acquire) < SlotMergeSeq[b]) { LSP_EV(EV_KWAIT_BEG); LSP_WAIT("kick-present", GPU.NDS.NumFrames, Present->Wait()); LSP_EV(EV_KWAIT_END); }
    if (HybDirect()) return;
    if (HybMap[b]) return;   // still mapped (that frame was never presented): reuse
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, DescPBO[b]);
    HybMap[b] = (u8*)glMapBufferRange(GL_PIXEL_UNPACK_BUFFER, 0, 2 * 192 * HybStride * 4,
        GL_MAP_WRITE_BIT | GL_MAP_INVALIDATE_BUFFER_BIT | GL_MAP_UNSYNCHRONIZED_BIT);
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
}

int HybridRenderer::HybridCurrentTag()
{
    // the 2D frame being kicked pairs with the 3D prepared at the last VCount 215 (its
    // colour buffer is known at once; GetFramebuffers waits for that render if needed)
    return GL3D()->GetCurColor();
}

// Emu thread, VBlank of a capture frame: the frame's 3D (latest ring entry) at 1x in the
// software 3D line format (6-bit RGB, 5-bit alpha in bits 24-28). Synchronous; the 3D was
// submitted at the previous VCount 215, so the wait is short.
#ifdef LITEV_HYB_CAPTURE_ASYNC
void HybridRenderer::CapKick(int pbo)
{
    Sync3D(GL3D()->GetCurColor());   // GPU-side wait only, once the 3D thread submitted it
    GLuint tex = GL3D()->GetColorTex(GL3D()->GetCurColor());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, ReadFB);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    glDisable(GL_SCISSOR_TEST);
    if (Scale > 1)
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, DownFB);
        glBlitFramebuffer(0, 0, 256 * Scale, 192 * Scale, 0, 0, 256, 192, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, DownFB);
    }
    if (!CapPBO[pbo])
    {
        glGenBuffers(1, &CapPBO[pbo]);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, CapPBO[pbo]);
        glBufferData(GL_PIXEL_PACK_BUFFER, 256 * 192 * 4, nullptr, GL_STREAM_READ);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, CapPBO[pbo]);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadPixels(0, 0, 256, 192, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (CapFence[pbo]) glDeleteSync(CapFence[pbo]);
    CapFence[pbo] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();
}
#endif

#ifdef LITEV_HYB_CAPTURE_OFFTHREAD
// GL 3D thread: start reading 3D colour buffer `color` (at 1x) into CapPBO[k]
void HybridRenderer::CapKickGL(int k, int color)
{
    if (!CapGLReadFB) { glGenFramebuffers(1, &CapGLReadFB); glGenFramebuffers(1, &CapGLDownFB); }
    glBindFramebuffer(GL_READ_FRAMEBUFFER, CapGLReadFB);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, GL3D()->GetColorTex(color), 0);
    glDisable(GL_SCISSOR_TEST);
    if (Scale > 1)
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, CapGLDownFB);
        glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, DownTex, 0);
        glBlitFramebuffer(0, 0, 256 * Scale, 192 * Scale, 0, 0, 256, 192, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, CapGLDownFB);
    }
    if (!CapPBO[k])
    {
        glGenBuffers(1, &CapPBO[k]);
        glBindBuffer(GL_PIXEL_PACK_BUFFER, CapPBO[k]);
        glBufferData(GL_PIXEL_PACK_BUFFER, 256 * 192 * 4, nullptr, GL_STREAM_READ);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, CapPBO[k]);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadPixels(0, 0, 256, 192, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    if (CapFence[k]) glDeleteSync(CapFence[k]);
    CapFence[k] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
    glFlush();
}

// GL 3D thread: finish the read in CapPBO[k] into CapOut[k]
void HybridRenderer::CapFinishGL(int k)
{
    glClientWaitSync(CapFence[k], GL_SYNC_FLUSH_COMMANDS_BIT, 100000000);
    glBindBuffer(GL_PIXEL_PACK_BUFFER, CapPBO[k]);
    if (const u8* p = (const u8*)glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, 256 * 192 * 4, GL_MAP_READ_BIT))
    {
        memcpy(ReadBuf, p, sizeof(ReadBuf));   // the mapping is uncached on Mali: one bulk copy
        glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
        CapConvert(ReadBuf, CapOut[k]);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    CapBusy[k].store(false, std::memory_order_release);
}
#endif

void HybridRenderer::HybridReadback3D(u32* dst)
{
#ifdef LITEV_HYB_CAPTURE_ASYNC
    {
        double t0 = HybNowMs();
        const u32 frame = GPU.NDS.NumFrames;
        // the read started `lag` frames ago, or the newest one at most that old while a capture
        // run is starting (else this frame's own, once). One frame back was not enough: with
        // the GPU over a frame behind (PW title screen, 3x) the wait still cost ~11 ms a frame.
        static const int lag = std::clamp(OpenGL::Prop("caplag", 2), 1, 2);
        int use = -1;
        for (int k = 0; k < 3; k++)
            if (k != CapNext && CapFence[k] && frame - CapFrameOf[k] <= (u32)lag && frame != CapFrameOf[k]
                && (use < 0 || CapFrameOf[k] < CapFrameOf[use]))
                use = k;
#ifdef LITEV_HYB_CAPTURE_OFFTHREAD
        // the whole read on the GL 3D thread, queued behind this frame's 3D render (so no wait
        // for it here): each job starts this frame's read and finishes the previous one (wait,
        // map, copy out of the uncached mapping, convert: ~7 ms of the emu thread's frame on the
        // PW title screen at 3x). The emu thread takes the result `lag` frames later.
        // debug.litev.capoff=0 turns it off.
        static const bool off = OpenGL::Prop("capoff", 1) != 0 && Thread3D()->Threaded;
        if (off)
        {
            const int k = CapNext, color = GL3D()->GetCurColor(), prev = CapPrevSlot;
            LSP_WAIT("cap-reuse", frame, while (CapBusy[k].load(std::memory_order_acquire)) std::this_thread::yield());   // slot reuse
            CapBusy[k].store(true, std::memory_order_relaxed);
            CapFrameOf[k] = frame;
            Thread3D()->Run([this, k, color, prev] { CapKickGL(k, color); if (prev >= 0) CapFinishGL(prev); }, false);
            CapPrevSlot = k;
#ifdef LITEV_FF_CAP_PREFETCH
            LastCapFrame = frame;
            if (use < 0 && PrefColor >= 0 && PrefColor == color)   // read ahead at VCount 215
            {
                LSP_WAIT("cap-pref", frame, while (CapBusy[3].load(std::memory_order_acquire)) std::this_thread::yield());
                memcpy(dst, CapOut[3], sizeof(CapOut[3]));
                CapNext = (k + 1) % 3;
                ProfReadback += HybNowMs() - t0;
                return;
            }
#endif
            if (use < 0)   // a capture run starts: this frame's own read, now
            {
                LSP_WAIT("cap-start", frame, Thread3D()->Run([this, k] { CapFinishGL(k); }, true));
                CapPrevSlot = -1;
                use = k;
            }
            LSP_WAIT("cap-busy", frame, while (CapBusy[use].load(std::memory_order_acquire)) std::this_thread::yield());   // done by now on a run
            memcpy(dst, CapOut[use], sizeof(CapOut[use]));
            CapNext = (k + 1) % 3;
            ProfReadback += HybNowMs() - t0;
            return;
        }
#endif

        CapKick(CapNext);
        CapFrameOf[CapNext] = frame;
        if (use < 0) use = CapNext;

        glClientWaitSync(CapFence[use], GL_SYNC_FLUSH_COMMANDS_BIT, 100000000);   // done by now on a run
        glBindBuffer(GL_PIXEL_PACK_BUFFER, CapPBO[use]);
        if (const u8* p = (const u8*)glMapBufferRange(GL_PIXEL_PACK_BUFFER, 0, 256 * 192 * 4, GL_MAP_READ_BIT))
        {
            // the mapping is uncached on Mali: one bulk copy, not 3 byte loads per pixel from it
            // (PW title screen: ~10 ms a frame on the emu thread -> the copy)
            memcpy(ReadBuf, p, sizeof(ReadBuf));
            glUnmapBuffer(GL_PIXEL_PACK_BUFFER);
            CapConvert(ReadBuf, dst);
        }
        glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
        CapNext = (CapNext + 1) % 3;
        ProfReadback += HybNowMs() - t0;
        return;
    }
#endif

    double t0 = HybNowMs();
    Sync3D(GL3D()->GetCurColor());
    GLuint tex = GL3D()->GetColorTex(GL3D()->GetCurColor());
    glBindFramebuffer(GL_READ_FRAMEBUFFER, ReadFB);
    glFramebufferTexture2D(GL_READ_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, tex, 0);
    glDisable(GL_SCISSOR_TEST);
    if (Scale > 1)
    {
        glBindFramebuffer(GL_DRAW_FRAMEBUFFER, DownFB);
        glBlitFramebuffer(0, 0, 256 * Scale, 192 * Scale, 0, 0, 256, 192, GL_COLOR_BUFFER_BIT, GL_NEAREST);
        glBindFramebuffer(GL_READ_FRAMEBUFFER, DownFB);
    }
    glBindBuffer(GL_PIXEL_PACK_BUFFER, 0);
    glPixelStorei(GL_PACK_ALIGNMENT, 4);
    glReadPixels(0, 0, 256, 192, GL_RGBA, GL_UNSIGNED_BYTE, ReadBuf);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

#ifdef LITEV_HYB_CAPTURE_ASYNC
    CapConvert(ReadBuf, dst);
    ProfReadback += HybNowMs() - t0;
}

void HybridRenderer::CapConvert(const u8* src, u32* dst)
{
#endif
    for (int i = 0; i < 256 * 192; i++)
    {
#ifdef LITEV_HYB_CAPTURE_ASYNC
        const u8* p = &src[i * 4];
#else
        const u8* p = &ReadBuf[i * 4];
#endif
#ifdef __ANDROID__
        u32 r = p[2], g = p[1], b = p[0];   // the GLES 3D pass emits BGRA (3DRenderFS)
#else
        u32 r = p[0], g = p[1], b = p[2];
#endif
        u32 a = p[3];
        dst[i] = ((r * 63 + 127) / 255) | (((g * 63 + 127) / 255) << 8) |
                 (((b * 63 + 127) / 255) << 16) | (((a * 31 + 127) / 255) << 24);
    }
#ifndef LITEV_HYB_CAPTURE_ASYNC
    ProfReadback += HybNowMs() - t0;
#endif
}

// Emu thread, after RunFrame: upload the last completed 2D frame's descriptors and
// merge them with the 3D that frame pairs with, at Nx, into a 2-layer array texture.
bool HybridRenderer::GetFramebuffers(void** top, void** bottom)
{
    // merge into the 2-layer output array (the app blits it into its frame texture)
    OutIdx ^= 1;
    Merge(OutFB[OutIdx], -1, 0);
    *top = &OutTex[OutIdx];
    *bottom = nullptr;
    return false;
}

// Merge straight into the app's frame texture: top screen at row 0, bottom screen at row
// bottomY (both 256N x 192N), saving the blit of the output array.
void HybridRenderer::PresentInto(GLuint dstTex, int bottomY)
{
    if (!PresentFB) glGenFramebuffers(1, &PresentFB);
    glBindFramebuffer(GL_FRAMEBUFFER, PresentFB);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dstTex, 0);
    const GLenum bufs[2] = {GL_COLOR_ATTACHMENT0, GL_NONE};
    glDrawBuffers(2, bufs);
    Merge(PresentFB, 0, bottomY);
}

// Emu thread, after RunFrame: upload the last completed 2D frame's descriptors and
// merge them with the 3D that frame pairs with, at Nx.
//   single < 0: one draw, both screens into layers 0/1 of fbo's colour attachments 0/1
//   single = 0: fbo has one target; top screen at row 0, bottom screen at row bottomY
static bool HybAsync()
{
#ifdef __ANDROID__
    static const bool on = [] { char b[92] = {}; return !(__system_property_get("debug.litev.hybasync", b) > 0 && atoi(b) == 0); }();
    return on;
#else
    return false;
#endif
}

void HybridRenderer::PresentIntoAsync(GLuint dstTex, int bottomY, std::function<void()> pre, std::function<void()> post)
{
    if (!Present)
    {
        Present = std::make_unique<GLWorker>();
        if (HybDirect() && HybAsync() && Thread3D()->Threaded) Present->StartThread("hyb-present");
    }
    if (!Present->Threaded)
    {
        pre(); PresentInto(dstTex, bottomY); post();
        return;
    }
    const int fb = AsyncPresentBuf, tag = HybTag[fb];
    const u64 seq = ++MergeSeq;
    SlotMergeSeq[fb] = seq;   // HybridKick(fb) waits for this before the 2D thread rewrites the slot
    glFlush();                // the frame texture may have just been (re)allocated on this context
    LSP_EV(EV_PRES_Q);
    Present->Run([this, fb, tag, dstTex, bottomY, seq, pre = std::move(pre), post = std::move(post)] {
        LSP_EV(EV_PRES_BEG);
        pre();
        if (!GLThreadFB) { glGenFramebuffers(1, &GLThreadFB); glGenVertexArrays(1, &GLThreadVAO); }
        glBindFramebuffer(GL_FRAMEBUFFER, GLThreadFB);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dstTex, 0);
        const GLenum bufs[2] = {GL_COLOR_ATTACHMENT0, GL_NONE};
        glDrawBuffers(2, bufs);
        MergeSlot(GLThreadFB, 0, bottomY, fb, tag, GLThreadVAO, true);
        post();
        MergeDone.store(seq, std::memory_order_release);
        LSP_EV(EV_PRES_END);
    }, false);
}

void HybridRenderer::WaitPresent()
{
    if (Present && MergeDone.load(std::memory_order_acquire) < MergeSeq) LSP_WAIT("present", GPU.NDS.NumFrames, Present->Wait());
}

void HybridRenderer::Merge(GLuint fbo, int single, int bottomY)
{
    MergeSlot(fbo, single, bottomY, AsyncPresentBuf, HybTag[AsyncPresentBuf], EmptyVAO, true);
}

// ponytail: the Prof* counters are shared by the emu-thread and GL-thread merge paths
// (diagnostic only; one path is active per session)
void HybridRenderer::MergeSlot(GLuint fbo, int single, int bottomY, int fb, int tag, GLuint vao, bool sync)
{
    const double t0 = HybNowMs();

    // the descriptors were copied into DescPBO[fb] by the 2D thread (HybridStage):
    // unmap and let the GPU copy them into this slot's texture
    if (HybMap[fb])
    {
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, DescPBO[fb]);
        glUnmapBuffer(GL_PIXEL_UNPACK_BUFFER);
        HybMap[fb] = nullptr;
        glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex[fb]);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, 0, HybStride, 192, 2,
                        GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, nullptr);
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    }
    else if (HybDirect())
    {
        // per screen: everything if a line carries 3D descriptors, else plane 1 + control column
        glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex[fb]);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
        glPixelStorei(GL_UNPACK_ROW_LENGTH, HybStride);
        for (int s = 0; s < 2; s++)
        {
            const u32* src = HybFB[fb] + s * 192 * HybStride;
            if (HybHas3D[fb][s])
                glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, s, HybStride, 192, 1, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, src);
            else
            {
                glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, s, 256, 192, 1, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, src);
                glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 512, 0, s, 1, 192, 1, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, src + 512);
            }
        }
        glPixelStorei(GL_UNPACK_ROW_LENGTH, 0);
    }
    ProfUpload += HybNowMs() - t0;

    glBindFramebuffer(GL_FRAMEBUFFER, fbo);
    glDisable(GL_SCISSOR_TEST);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_STENCIL_TEST);
    glDisable(GL_BLEND);
    glColorMaski(0, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glColorMaski(1, GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_FALSE);
    glUseProgram(MergeShader);
    glUniform1i(ScaleULoc, Scale);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex[fb]);
    const double ts = HybNowMs();
    if (sync) Sync3D(tag);
    glActiveTexture(GL_TEXTURE1);
    {
        const GLuint edge = GL3D()->GetEdgeTex(tag);   // LITEV_GL_EDGE_MARK
        glUniform1i(EdgeULoc, edge != 0);
        if (edge) { glActiveTexture(GL_TEXTURE2); glBindTexture(GL_TEXTURE_2D, edge); glActiveTexture(GL_TEXTURE1); }
    }
    glBindTexture(GL_TEXTURE_2D, GL3D()->GetColorTex(tag));
    glBindVertexArray(vao);
    const double td = HybNowMs();
    if (single < 0)
    {
        glUniform1i(SingleULoc, -1);
        glUniform2i(OriginULoc, 0, 0);
        glViewport(0, 0, 256 * Scale, 192 * Scale);
        if (!(OpenGL::GLSkip() & 64)) glDrawArrays(GL_TRIANGLES, 0, 3);
    }
    else
    {
        for (int sc = 0; sc < 2; sc++)
        {
            const int y = sc ? bottomY : 0;
            glUniform1i(SingleULoc, sc);
            glUniform2i(OriginULoc, 0, y);
            glViewport(0, y, 256 * Scale, 192 * Scale);
#ifdef LITEV_HYB_MERGE_1X_2D
            // a screen without 3D has no Nx detail: merge its 256x192 pixels once, not Nx*Nx
            // times (the 2D-only screen's merge alone was ~2 ms of GPU per frame at 3x), then
            // copy them up. Same pixels. debug.litev.hybmerge1x=0 turns it off.
            static const bool merge1x = OpenGL::Prop("hybmerge1x", 1) != 0;
            if (merge1x && Scale > 1 && !HybHas3D[fb][sc])
            {
                // ponytail: made on the first merging context; one merge path (emu thread or
                // hyb-present) is used per session
                if (!Merge1xFB)
                {
                    glGenTextures(1, &Merge1xTex);
                    glBindTexture(GL_TEXTURE_2D, Merge1xTex);
                    glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, 256, 192);
                    glBindTexture(GL_TEXTURE_2D, GL3D()->GetColorTex(tag));   // TEXTURE1 stays the 3D colour
                    glGenFramebuffers(1, &Merge1xFB);
                    glBindFramebuffer(GL_FRAMEBUFFER, Merge1xFB);
                    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, Merge1xTex, 0);
                }
                glBindFramebuffer(GL_FRAMEBUFFER, Merge1xFB);
                glUniform1i(ScaleULoc, 1);
                glUniform2i(OriginULoc, 0, 0);
                glViewport(0, 0, 256, 192);
                if (!(OpenGL::GLSkip() & 64)) glDrawArrays(GL_TRIANGLES, 0, 3);
                glUniform1i(ScaleULoc, Scale);
                glBindFramebuffer(GL_READ_FRAMEBUFFER, Merge1xFB);
                glBindFramebuffer(GL_DRAW_FRAMEBUFFER, fbo);
                glBlitFramebuffer(0, 0, 256, 192, 0, y, 256 * Scale, y + 192 * Scale, GL_COLOR_BUFFER_BIT, GL_NEAREST);
                glBindFramebuffer(GL_FRAMEBUFFER, fbo);
                continue;
            }
#endif
#ifdef LITEV_HYB_MERGE_FASTLINES
            // lines that show the 3D straight through (most of a 3D scene) merge with a copy
            // shader; the others (2D on top, blends, brightness) with the full one. Each run of
            // lines is the full-screen triangle scissored to it (thin quads cost more on Mali).
            // PW town intro at 3x. Same pixels. debug.litev.hybfastlines=0 turns it off.
            static const bool fastLines = OpenGL::Prop("hybfastlines", 1) != 0;
            if (fastLines && Scale > 1 && HybHas3D[fb][sc] && !(OpenGL::GLSkip() & 64))
            {
                int fast[192];   // 0: full merge; else 1 + bright mode, with evy in bits 8+
                const u32* scr = HybFB[fb] + sc * 192 * HybStride;
                for (int l = 0; l < 192; l++)
                {
                    const u32* ln = scr + l * HybStride;
                    const u32 c = ln[512];
                    u32 bright = (c >> 14) & 3;
                    const u32 evy = std::min<u32>(c & 0x1F, 16);
                    if (bright == 3 || evy == 0) bright = 0;
                    bool f = ((c >> 16) & 3) == 1 && (c & (1 << 18)) && !(c >> 23);
                    for (int x = 0; f && x < 256; x++)
                    {
                        const u32 mode = ln[x] >> 29;
                        f = mode == 0 || mode == 5 || mode == 6;
                    }
                    fast[l] = f ? int(1 + bright + (bright ? evy << 8 : 0)) : 0;
                }
                glEnable(GL_SCISSOR_TEST);
                for (int l0 = 0; l0 < 192; )
                {
                    int l1 = l0 + 1;
                    while (l1 < 192 && fast[l1] == fast[l0]) l1++;
                    glScissor(0, y + l0 * Scale, 256 * Scale, (l1 - l0) * Scale);
                    glUniform1i(FastULoc, fast[l0] & 0xFF);
                    glUniform1i(FastEvyULoc, fast[l0] >> 8);
                    glDrawArrays(GL_TRIANGLES, 0, 3);
                    l0 = l1;
                }
                glDisable(GL_SCISSOR_TEST);
                glUniform1i(FastULoc, 0);
                continue;
            }
#endif
            if (!(OpenGL::GLSkip() & 64)) glDrawArrays(GL_TRIANGLES, 0, 3);
        }
    }
    ProfSync += td - ts; ProfDraw += HybNowMs() - td;

    glActiveTexture(GL_TEXTURE0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    ProfMerge += HybNowMs() - t0;
    if (++ProfFrames == 60)
    {
        ProfFrames = 0;
        static const bool on = [] {
#ifdef __ANDROID__
            char b[PROP_VALUE_MAX] = {0};
            return __system_property_get("debug.litev.prof", b) > 0 && atoi(b) != 0;
#else
            return getenv("LITEV_PROF") != nullptr;
#endif
        }();
        if (!on) { ProfGL3D = ProfWait = ProfMerge = ProfUpload = ProfSync = ProfDraw = ProfReadback = 0; return; }
        Platform::Log(Platform::Info, "LITEV_HYB 60f: gl3d=%.2f wait3d=%.2f merge=%.2f (upload=%.2f sync=%.2f draw=%.2f) readback=%.2f ms/frame (emu thread, scale %d)\n",
                      ProfGL3D / 60, ProfWait / 60, ProfMerge / 60, ProfUpload / 60, ProfSync / 60, ProfDraw / 60, ProfReadback / 60, Scale);
        ProfGL3D = ProfWait = ProfMerge = ProfUpload = ProfSync = ProfDraw = ProfReadback = 0;
        GLThread3D* t = Thread3D();
        if (t->JobN)
            Platform::Log(Platform::Info, "LITEV_HYB 3d-job: n=%d queued=%.2f wall=%.2f cpu=%.2f incl-flush=%.2f | emu: prev-job wait=%.2f prepare=%.2f kick-gap=%.2f job-end-from-kick=%.2f polyram-wait=%.2f ms/job (snapcopy %d, bank waits %d)\n",
                          t->JobN, t->JobQueued / t->JobN, t->JobWall / t->JobN, t->JobCpu / t->JobN, t->JobTail / t->JobN, t->PrepWait / t->JobN, t->PrepMs / t->JobN, t->KickGap / t->JobN, t->JobEndFromKick / t->JobN, t->FinishWait / t->JobN, t->CopyN, t->BankWaitN);
        if (t->JobN)
            Platform::Log(Platform::Info, "LITEV_HYB 3d-draws: %.1f draws %.1f polys per job\n",
                          (double)GL3D()->StatDraws / t->JobN, (double)GL3D()->StatPolys / t->JobN);
        GL3D()->StatDraws = GL3D()->StatPolys = 0;
        OpenGL::GLStatLog(60);
        t->JobQueued = t->JobWall = t->JobCpu = t->PrepWait = t->PrepMs = t->JobTail = t->KickGap = t->JobEndFromKick = t->FinishWait = 0; t->JobN = t->CopyN = t->BankWaitN = 0;
    }
}

}
