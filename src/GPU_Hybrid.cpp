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
    for (int i = 0; i < 3; i++) HybFB[i] = new u32[2 * 192 * HybStride]();
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
    glDeleteTextures(3, DescTex);
    glDeleteBuffers(3, DescPBO);
    glDeleteTextures(2, OutTex);
    glDeleteFramebuffers(2, OutFB);
    if (PresentFB) glDeleteFramebuffers(1, &PresentFB);
    glDeleteFramebuffers(1, &ReadFB);
    glDeleteFramebuffers(1, &DownFB);
    glDeleteTextures(1, &DownTex);
}

bool HybridRenderer::Init()
{
    if (!Rend3D->Init()) return false;
    // 4 colour buffers: the 3D a presented frame pairs with is up to 3 renders old
    // (async 2D pipeline at depth 2), and one more is being rendered.
    // glFinish: the new texture storage must be complete before the emu context uses it
    Thread3D()->Run([this] { GL3D()->SetColorRing(GLRenderer3D::MaxColorRing); glFinish(); }, true);

    if (!OpenGL::CompileVertexFragmentProgram(MergeShader,
            kHybridMergeVS, kHybridMergeFS, "HybridMergeShader",
            {}, {{"oTopColor", 0}, {"oBottomColor", 1}}))
        return false;

    glUseProgram(MergeShader);
    glUniform1i(glGetUniformLocation(MergeShader, "DescTex"), 0);
    glUniform1i(glGetUniformLocation(MergeShader, "Tex3D"), 1);
    ScaleULoc = glGetUniformLocation(MergeShader, "uScale");
    SingleULoc = glGetUniformLocation(MergeShader, "uSingle");
    OriginULoc = glGetUniformLocation(MergeShader, "uOrigin");

    glGenVertexArrays(1, &EmptyVAO);

    auto texParams = [](GLenum target)
    {
        glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    };

    glGenBuffers(3, DescPBO);
    for (GLuint b : DescPBO)
    {
        glBindBuffer(GL_PIXEL_UNPACK_BUFFER, b);
        glBufferData(GL_PIXEL_UNPACK_BUFFER, 2 * 192 * HybStride * 4, nullptr, GL_STREAM_DRAW);
    }
    glBindBuffer(GL_PIXEL_UNPACK_BUFFER, 0);
    glGenTextures(3, DescTex);
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
    for (int i = 0; i < 3; i++) memset(HybFB[i], 0, 2 * 192 * HybStride * sizeof(u32));
}

void HybridRenderer::PreSavestate()
{
    FlushAsyncRender();
    Wait3D();
}

void HybridRenderer::PostSavestate()
{
    Rend3D->Reset();   // texture cache
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
    ProfGL3D += HybNowMs() - t0;
}

void HybridRenderer::Finish3DRendering()
{
    Rend3D->FinishRendering();   // no-op: the GL thread renders from a snapshot
}

void HybridRenderer::Restart3DRendering()
{
    Rend3D->RestartFrame();
}

// emu thread, 2D kick for slot b: map its staging buffer for the 2D thread to fill
void HybridRenderer::HybridKick(int b)
{
    // an async present may still be reading this slot on the GL thread
    if (Present && MergeDone.load(std::memory_order_acquire) < SlotMergeSeq[b]) Present->Wait();
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
void HybridRenderer::HybridReadback3D(u32* dst)
{
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

    for (int i = 0; i < 256 * 192; i++)
    {
        const u8* p = &ReadBuf[i * 4];
#ifdef __ANDROID__
        u32 r = p[2], g = p[1], b = p[0];   // the GLES 3D pass emits BGRA (3DRenderFS)
#else
        u32 r = p[0], g = p[1], b = p[2];
#endif
        u32 a = p[3];
        dst[i] = ((r * 63 + 127) / 255) | (((g * 63 + 127) / 255) << 8) |
                 (((b * 63 + 127) / 255) << 16) | (((a * 31 + 127) / 255) << 24);
    }
    ProfReadback += HybNowMs() - t0;
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
    Present->Run([this, fb, tag, dstTex, bottomY, seq, pre = std::move(pre), post = std::move(post)] {
        pre();
        if (!GLThreadFB) { glGenFramebuffers(1, &GLThreadFB); glGenVertexArrays(1, &GLThreadVAO); }
        glBindFramebuffer(GL_FRAMEBUFFER, GLThreadFB);
        glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, dstTex, 0);
        const GLenum bufs[2] = {GL_COLOR_ATTACHMENT0, GL_NONE};
        glDrawBuffers(2, bufs);
        MergeSlot(GLThreadFB, 0, bottomY, fb, tag, GLThreadVAO, true);
        post();
        MergeDone.store(seq, std::memory_order_release);
    }, false);
}

void HybridRenderer::WaitPresent()
{
    if (Present && MergeDone.load(std::memory_order_acquire) < MergeSeq) Present->Wait();
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
            Platform::Log(Platform::Info, "LITEV_HYB 3d-job: n=%d queued=%.2f wall=%.2f cpu=%.2f incl-flush=%.2f | emu: prev-job wait=%.2f prepare=%.2f kick-gap=%.2f job-end-from-kick=%.2f ms/job\n",
                          t->JobN, t->JobQueued / t->JobN, t->JobWall / t->JobN, t->JobCpu / t->JobN, t->JobTail / t->JobN, t->PrepWait / t->JobN, t->PrepMs / t->JobN, t->KickGap / t->JobN, t->JobEndFromKick / t->JobN);
        if (t->JobN)
            Platform::Log(Platform::Info, "LITEV_HYB 3d-draws: %.1f draws %.1f polys per job\n",
                          (double)GL3D()->StatDraws / t->JobN, (double)GL3D()->StatPolys / t->JobN);
        GL3D()->StatDraws = GL3D()->StatPolys = 0;
        OpenGL::GLStatLog(60);
        t->JobQueued = t->JobWall = t->JobCpu = t->PrepWait = t->PrepMs = t->JobTail = t->KickGap = t->JobEndFromKick = 0; t->JobN = 0;
    }
}

}
