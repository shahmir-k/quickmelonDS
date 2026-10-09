// GLThread3D: GLRenderer3D on its own GL thread (GLWorker), driven as a Renderer3D.
#ifndef GLTHREAD3D_H
#define GLTHREAD3D_H

#include <atomic>
#include <chrono>
#include <memory>
#include <time.h>
#include "GPU3D.h"
#include "GPU3D_OpenGL.h"
#include "GLWorker.h"

namespace melonDS
{

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
class GLThread3D : public Renderer3D, public GLWorker
{
public:
    // parent: GLRenderer (hi-res 2D), whose capture textures the 3D samples; nullptr: hybrid
    GLThread3D(melonDS::GPU3D& gpu3D, GLRenderer* parent = nullptr, const char* name = "hyb-gl3d")
        : Renderer3D(gpu3D), Parent(parent)
    {
        StartThread(name);
        Run([this, &gpu3D] { GL = std::make_unique<GLRenderer3D>(gpu3D, Parent); }, true);
#ifdef LITEV_HYB_TEXSTAGE
        if (Threaded && !Parent) GL->EnableTexStaging();
#endif
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
        // GLRenderer: its capture textures were written on the caller's context this frame
        GLsync inFence = nullptr;
        if (Parent && Threaded) { inFence = glFenceSync(GL_SYNC_GPU_COMMANDS_COMPLETE, 0); glFlush(); }
        const double tq = NowMs();
        Run([this, tq, seq, c, slot, inFence] {
            const double t0 = NowMs(), c0 = CpuMs();
            JobQueued += t0 - tq;
            if (inFence) { glWaitSync(inFence, 0, GL_TIMEOUT_IGNORED); glDeleteSync(inFence); }
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
    // The render state AND the polygons/vertices are snapshotted in PrepareFrame (copies the
    // job owns), so VBlank never has to wait for the job.
    void FinishRendering() override {}
    // block until the job that renders colour buffer c has been issued (its fence exists)
    void WaitColor(int c)
    {
        if (DoneSeq.load(std::memory_order_acquire) < ColorSeq[c]) Wait();
    }
    // caller's context: wait (GPU-side) for colour buffer c's render, return its texture
    GLuint SyncColor(int c)
    {
        WaitColor(c);
        if (Fence[c]) glWaitSync(Fence[c], 0, GL_TIMEOUT_IGNORED);
        return GL->GetColorTex(c);
    }
    void RestartFrame() override { RenderFrame(); }
    u32* GetLine(int line) override { return nullptr; }
    // GLRenderer3D compiles its shaders in Init (no incremental compile); answering here
    // keeps the app's per-frame query from waiting on the in-flight render job
    bool NeedsShaderCompile() override { return false; }

    GLRenderer* Parent;
    std::unique_ptr<GLRenderer3D> GL;
    GLsync Fence[GLRenderer3D::MaxColorRing] {};
    u64 Seq = 0, ColorSeq[GLRenderer3D::MaxColorRing] {};
    int Slot = 0;
    std::atomic<u64> DoneSeq { 0 };
    // job timing (GL thread), read/reset by HybridRenderer's log while the job is idle
    double JobQueued = 0, JobWall = 0, JobCpu = 0, PrepWait = 0, PrepMs = 0, JobTail = 0, KickGap = 0, LastKick = 0, JobEndFromKick = 0, FinishWait = 0; int JobN = 0;
    static double NowMs() { return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
    static double CpuMs() { timespec ts; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts); return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6; }
};

}

#endif // GLTHREAD3D_H
