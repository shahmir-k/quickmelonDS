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
#endif

namespace melonDS
{

#include "OpenGL_shaders/HybridMergeVS.h"
#include "OpenGL_shaders/HybridMergeFS.h"

static double HybNowMs()
{
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

#ifndef LITEV_SOFT2D_THREADED
#error "HybridRenderer needs the threaded software 2D (LITEV_SOFT2D_THREADED)"
#endif

// GLRenderer3D on its own GL thread (shared EGL context), so its ~10 ms/frame of CPU
// work (texture cache, polygon setup, vertex build, draw submission, driver) leaves the
// emu thread. Wraps the renderer as a Renderer3D, so SoftRenderer drives it unchanged:
//   RenderFrame        emu: PrepareFrame (VRAM coherence) -> GL thread: RenderPreparedFrame
//   FinishRendering    waits for the job: GPU::VBlank calls it before it mutates any state
//                      the job reads (polygon RAM, Render* registers), the same barrier
//                      the threaded software 3D uses
//   everything else    runs synchronously on the GL thread (GL objects such as FBOs/VAOs
//                      are per-context, so the renderer is only ever touched there)
// Each finished render leaves a fence; the emu context waits on it (GPU-side) before
// sampling that colour buffer. Without EGL (desktop builds) or if the shared context
// cannot be made, everything runs inline on the caller's thread as before.
class GLThread3D : public Renderer3D
{
public:
    GLThread3D(melonDS::GPU3D& gpu3D) : Renderer3D(gpu3D)
    {
        StartThread();
        Run([this, &gpu3D] { GL = std::make_unique<GLRenderer3D>(gpu3D, nullptr); }, true);
    }
    ~GLThread3D() override
    {
        Run([this] {
            for (GLsync& f : Fence) if (f) { glDeleteSync(f); f = nullptr; }
            GL.reset();
        }, true);
        StopThread();
    }

    bool Init() override { bool ok = false; Run([&] { ok = GL->Init(); }, true); return ok; }
    void Reset() override { Run([this] { GL->Reset(); }, true); }
    void RenderFrame() override
    {
        // Up to two frames outstanding: the previous one may still render while this one
        // is prepared and queued. Wait only for a frame that has not STARTED yet (its slot
        // is about to be reused), and for the running one if this frame changes the flat
        // texture VRAM it may be decoding from.
        const double tw = NowMs();
        if (LastKick > 0) KickGap += tw - LastKick;
        LastKick = tw;
        WaitQueued();
        const double tp = NowMs();
        const int slot = Slot;
        Slot ^= 1;
        GL->PrepareFrame(slot, [this] {
            const double t = NowMs();
            Wait();
            PrepWait += NowMs() - t;
        });
        PrepWait += tp - tw; PrepMs += NowMs() - tp;
        const u64 seq = ++Seq;
        const int c = GL->GetCurColor();
        ColorSeq[c] = seq;   // (unchanged colour if the frame is skipped: still waits right)
        const double tq = NowMs();
        Run([this, tq, seq, c, slot] {
            const double t0 = NowMs(), c0 = CpuMs();
            JobQueued += t0 - tq;
            GL->RenderPreparedFrame(slot);
            JobWall += NowMs() - t0; JobCpu += CpuMs() - c0; JobN++;
            if (Threaded)
            {
                if (Fence[c]) glDeleteSync(Fence[c]);
                Fence[c] = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0);
                glFlush();
            }
            JobTail += NowMs() - t0;
            JobEndFromKick += NowMs() - tq;
            DoneSeq.store(seq, std::memory_order_release);
        }, false);
    }
    // the render state is snapshotted in PrepareFrame: VBlank never has to wait for the job
    void FinishRendering() override {}
    // block until the job that renders colour buffer c has been issued (its fence exists)
    void WaitColor(int c)
    {
        if (DoneSeq.load(std::memory_order_acquire) < ColorSeq[c]) Wait();
    }
    void RestartFrame() override { RenderFrame(); }
    u32* GetLine(int line) override { return nullptr; }
    // GLRenderer3D compiles its shaders in Init (no incremental compile); answering here
    // keeps the app's per-frame query from waiting on the in-flight render job
    bool NeedsShaderCompile() override { return false; }

    // run f on the GL thread (inline when there is none); wait = block until done
    void Run(std::function<void()> f, bool wait)
    {
        if (!Threaded) { f(); return; }
        {
            std::lock_guard<std::mutex> l(M);
            Q.push_back(std::move(f));
        }
        CV.notify_one();
        if (wait) Wait();
    }
    void Wait()
    {
        if (!Threaded) return;
        std::unique_lock<std::mutex> l(M);
        DoneCV.wait(l, [this] { return Q.empty() && !Busy; });
    }
    void WaitQueued()   // every queued job has started (the running one may continue)
    {
        if (!Threaded) return;
        std::unique_lock<std::mutex> l(M);
        DoneCV.wait(l, [this] { return Q.empty(); });
    }

    std::unique_ptr<GLRenderer3D> GL;
    GLsync Fence[GLRenderer3D::MaxColorRing] {};
    u64 Seq = 0, ColorSeq[GLRenderer3D::MaxColorRing] {};
    int Slot = 0;
    std::atomic<u64> DoneSeq { 0 };
    // job timing (GL thread), read/reset by HybridRenderer's log while the job is idle
    double JobQueued = 0, JobWall = 0, JobCpu = 0, PrepWait = 0, PrepMs = 0, JobTail = 0, KickGap = 0, LastKick = 0, JobEndFromKick = 0; int JobN = 0;
    static double NowMs() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
    static double CpuMs() { timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts); return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6; }
    bool Threaded = false;

private:
    std::thread T;
    std::mutex M;
    std::condition_variable CV, DoneCV;
    std::deque<std::function<void()>> Q;
    bool Busy = false, Quit = false;
#ifdef __ANDROID__
    EGLDisplay Dpy = EGL_NO_DISPLAY;
    EGLContext Ctx = EGL_NO_CONTEXT;
    EGLSurface Surf = EGL_NO_SURFACE;
#endif

    void StartThread()
    {
#ifdef __ANDROID__
        Dpy = eglGetCurrentDisplay();
        EGLContext share = eglGetCurrentContext();
        if (Dpy == EGL_NO_DISPLAY || share == EGL_NO_CONTEXT) return;
        EGLint cfgId = 0, n = 0;
        EGLConfig cfg = nullptr;
        eglQueryContext(Dpy, share, EGL_CONFIG_ID, &cfgId);
        const EGLint cfgAttr[] = {EGL_CONFIG_ID, cfgId, EGL_NONE};
        if (!eglChooseConfig(Dpy, cfgAttr, &cfg, 1, &n) || n < 1) return;
        const EGLint ctxAttr[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
        Ctx = eglCreateContext(Dpy, cfg, share, ctxAttr);
        if (Ctx == EGL_NO_CONTEXT) return;

        bool ok = false, started = false;
        T = std::thread([this, cfg, &ok, &started] {
            // surfaceless if supported, else a 1x1 pbuffer
            bool cur = eglMakeCurrent(Dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, Ctx);
            if (!cur)
            {
                const EGLint pb[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
                Surf = eglCreatePbufferSurface(Dpy, cfg, pb);
                cur = Surf != EGL_NO_SURFACE && eglMakeCurrent(Dpy, Surf, Surf, Ctx);
            }
            // off the emu thread's core 3 (it is pinned there by the app)
            cpu_set_t set; CPU_ZERO(&set);
            CPU_SET(0, &set); CPU_SET(1, &set); CPU_SET(2, &set);
            sched_setaffinity(0, sizeof(set), &set);
            {
                std::lock_guard<std::mutex> l(M);
                ok = cur; started = true;
            }
            DoneCV.notify_all();
            if (!cur) return;
            Loop();
            eglMakeCurrent(Dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        });
        {
            std::unique_lock<std::mutex> l(M);
            DoneCV.wait(l, [&] { return started; });
        }
        Threaded = ok;
        Platform::Log(Platform::Info, "LITEV_HYB GL 3D thread: %s\n", ok ? "on" : "FAILED (inline)");
        if (!ok)
        {
            T.join();
            if (Surf != EGL_NO_SURFACE) eglDestroySurface(Dpy, Surf);
            eglDestroyContext(Dpy, Ctx);
            Ctx = EGL_NO_CONTEXT; Surf = EGL_NO_SURFACE;
        }
#endif
    }
    void StopThread()
    {
        if (!Threaded) return;
        {
            std::lock_guard<std::mutex> l(M);
            Quit = true;
        }
        CV.notify_one();
        T.join();
        Threaded = false;
#ifdef __ANDROID__
        if (Surf != EGL_NO_SURFACE) eglDestroySurface(Dpy, Surf);
        eglDestroyContext(Dpy, Ctx);
#endif
    }
    void Loop()
    {
        for (;;)
        {
            std::function<void()> f;
            {
                std::unique_lock<std::mutex> l(M);
                CV.wait(l, [this] { return Quit || !Q.empty(); });
                if (Q.empty()) return;   // Quit with nothing left
                f = std::move(Q.front());
                Q.pop_front();
                Busy = true;
            }
            DoneCV.notify_all();   // WaitQueued
            f();
            {
                std::lock_guard<std::mutex> l(M);
                Busy = false;
            }
            DoneCV.notify_all();
        }
    }
};

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
    StopAsyncThread();   // the 2D thread reads HybFB; stop it before GL teardown below
    glDeleteProgram(MergeShader);
    glDeleteVertexArrays(1, &EmptyVAO);
    glDeleteTextures(1, &DescTex);
    glDeleteTextures(2, OutTex);
    glDeleteFramebuffers(2, OutFB);
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

    glGenVertexArrays(1, &EmptyVAO);

    auto texParams = [](GLenum target)
    {
        glTexParameteri(target, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(target, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(target, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    };

    glGenTextures(1, &DescTex);
    glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex);
    texParams(GL_TEXTURE_2D_ARRAY);
    glTexImage3D(GL_TEXTURE_2D_ARRAY, 0, GL_RGBA8UI, HybStride, 192, 2, 0, GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, nullptr);

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
    const double t0 = HybNowMs();
    const int fb = AsyncPresentBuf;

    glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex);
    glPixelStorei(GL_UNPACK_ALIGNMENT, 4);
    glTexSubImage3D(GL_TEXTURE_2D_ARRAY, 0, 0, 0, 0, HybStride, 192, 2,
                    GL_RGBA_INTEGER, GL_UNSIGNED_BYTE, HybFB[fb]);
    ProfUpload += HybNowMs() - t0;

    OutIdx ^= 1;
    glBindFramebuffer(GL_FRAMEBUFFER, OutFB[OutIdx]);
    glViewport(0, 0, 256 * Scale, 192 * Scale);
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
    glBindTexture(GL_TEXTURE_2D_ARRAY, DescTex);
    Sync3D(HybTag[fb]);
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(GL_TEXTURE_2D, GL3D()->GetColorTex(HybTag[fb]));
    glBindVertexArray(EmptyVAO);
    glDrawArrays(GL_TRIANGLES, 0, 3);

    glActiveTexture(GL_TEXTURE0);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);

    *top = &OutTex[OutIdx];
    *bottom = nullptr;

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
        if (!on) { ProfGL3D = ProfWait = ProfMerge = ProfUpload = ProfReadback = 0; return false; }
        Platform::Log(Platform::Info, "LITEV_HYB 60f: gl3d=%.2f wait3d=%.2f merge=%.2f (upload=%.2f) readback=%.2f ms/frame (emu thread, scale %d)\n",
                      ProfGL3D / 60, ProfWait / 60, ProfMerge / 60, ProfUpload / 60, ProfReadback / 60, Scale);
        ProfGL3D = ProfWait = ProfMerge = ProfUpload = ProfReadback = 0;
        GLThread3D* t = Thread3D();
        if (t->JobN)
            Platform::Log(Platform::Info, "LITEV_HYB 3d-job: n=%d queued=%.2f wall=%.2f cpu=%.2f incl-flush=%.2f | emu: prev-job wait=%.2f prepare=%.2f kick-gap=%.2f job-end-from-kick=%.2f ms/job\n",
                          t->JobN, t->JobQueued / t->JobN, t->JobWall / t->JobN, t->JobCpu / t->JobN, t->JobTail / t->JobN, t->PrepWait / t->JobN, t->PrepMs / t->JobN, t->KickGap / t->JobN, t->JobEndFromKick / t->JobN);
        t->JobQueued = t->JobWall = t->JobCpu = t->PrepWait = t->PrepMs = t->JobTail = t->KickGap = t->JobEndFromKick = 0; t->JobN = 0;
        ProfFrames = 0;
    }
    return false;
}

}
