/*
    liteDS: software-render phase profiler (LITEV_SOFTPROF, default OFF).

    Decomposes the software render pipeline into its real phases so the critical
    path can be identified instead of guessed:

      emu thread   : barrier block at VBlank (waiting on frame N-1's render),
                     snapshot/prep cost after the barrier
      3D rt thread : ClearBuffers, raster wall, final-pass wall
      3D bands     : per-band raster ms + per-band final-pass ms (load balance)
      2D async     : wall, and the time BLOCKED in GetLine (== 2D<-3D serialization)

    All accumulators are single-writer (one owning thread each); the emu thread
    reads+resets them every 60 frames. Relaxed atomics so the read is not UB.
*/
#ifndef LITEV_SOFTPROF_H
#define LITEV_SOFTPROF_H

#ifdef LITEV_SOFTPROF

#include <atomic>
#include <chrono>
#include "Platform.h"

namespace melonDS
{
namespace LitevSP
{

inline double NowMs()
{
    using namespace std::chrono;
    return duration<double, std::milli>(steady_clock::now().time_since_epoch()).count();
}

// single-writer accumulator
struct Acc
{
    std::atomic<double> v { 0.0 };
    inline void Add(double d) { v.store(v.load(std::memory_order_relaxed) + d, std::memory_order_relaxed); }
    inline double Take() { double d = v.load(std::memory_order_relaxed); v.store(0.0, std::memory_order_relaxed); return d; }
};

constexpr int MAXB = 8;

struct State
{
    Acc EmuBarrier;      // emu: Semaphore_Wait(AsyncDone) at VBlank  (2D barrier)
    Acc Emu3DBarrier;    // emu: Semaphore_Wait(Sema_RenderDone)      (3D barrier!)
    Acc EmuTexBarrier;   // emu: drain tile coordinator before texture input mutation
    std::atomic<uint64_t> CompositeCE[5]{}; // selected 2D color-effect lanes (debug.litev.softprof)
    Acc EmuSnap;         // emu: snapshot/prep after the barrier
    Acc S3DClear;        // 3D rt: ClearBuffers
    Acc S3DRaster;       // 3D rt: banded raster phase (post+wait all bands)
    Acc S3DFinalWall;    // 3D rt: banded final phase
    Acc S3DBand[MAXB];   // band i: its own raster work
    Acc S3DFinal[MAXB];  // band i: its own final-pass work
    Acc S2DWall;         // 2D async: whole AsyncRenderFrame
    Acc S2DBlock[MAXB];  // 2D band i: time BLOCKED inside GetLine
    Acc S2DBand[MAXB];   // 2D band i: total (incl. block)
    Acc RenderWall;      // 3D-thread-wake -> 2D-done  (whole render critical path)
    Acc S3DWake;         // Sema_RenderStart post (emu, VCount 215) -> 3D rt actually running
    Acc S3DTotal;        // Sema_RenderStart post -> Sema_RenderDone post (full 3D latency)
    Acc EmuWindow;       // Sema_RenderStart post -> emu enters Finish3DRendering (the budget)

    std::atomic<double> T3DPost  { 0.0 };   // when the emu posted Sema_RenderStart
    std::atomic<double> T3DStart { 0.0 };   // when the 3D rt woke for this frame
    std::atomic<int> Frames { 0 };
    std::atomic<int> NB3D { 0 };

    // event counters (per 60-frame window) — catch dilution / skipped renders
    std::atomic<int> NPost { 0 };       // Start3DRendering -> Sema_RenderStart posted
    std::atomic<int> NRender { 0 };     // 3D rt actually rasterized (!FrameIdentical)
    std::atomic<int> NIdent { 0 };      // 3D rt short-circuited (FrameIdentical)
    std::atomic<int> NFinish { 0 };     // emu entered Finish3DRendering and waited
};

extern State S;

// Called on the emu thread once per VBlank. Logs a decomposition every 60 frames.
void Tick();

void NameThread(const char* n);

// Per-frame pipeline event ring (debug.litev.pipering=N: record N events, then write them once
// to /sdcard/Android/data/com.sereneds.app/files/pipering.csv). Each event: wall ms, the calling
// thread's CPU ms, event id. ~100 ns per event; off unless the prop is set.
enum PipeEv { EV_VBL_IN, EV_VBL_BAR, EV_KICK, EV_S2D_BEG, EV_S2D_END, EV_PRES_BEG, EV_PRES_END,
              EV_KWAIT_BEG, EV_KWAIT_END, EV_HELP_BEG, EV_HELP_END, EV_GL3D_BEG, EV_GL3D_END,
              EV_PRES_Q };
void Ev(int ev);

// Stall log (debug.litev.stalllog=1): emu-thread waits over 3 ms and GL 3D jobs over 10 ms are
// logged (LITEV_STALL lines) with the frame and where the GL job's time went, to classify
// one-off slow frames. The GL counters are written by the GL 3D thread only.
bool StallLogOn();
struct GLJobStat { double TexNew, TexUp, Draw, DrawMax; int TexNewN, TexUpN, DrawN, DrawMaxKey; };
extern GLJobStat GLJ;
void StallWait(const char* site, double ms, unsigned frame);
void StallGLJob(unsigned frame, double wall, double cpu);
void StallFrame(unsigned frame);   // emu thread, each VBlank kick: logs gaps over 20 ms

} // namespace LitevSP
} // namespace melonDS

#define LSP_NOW()          ::melonDS::LitevSP::NowMs()
#define LSP_ADD(f, d)      ::melonDS::LitevSP::S.f.Add(d)
#define LSP_NAME(n)        ::melonDS::LitevSP::NameThread(n)
#define LSP_EV(e)          ::melonDS::LitevSP::Ev(::melonDS::LitevSP::e)
// time stmt into GLJ.<f> (+ count <f>N) when the stall log is on
#define LSP_GLT(f, stmt)   do { if (::melonDS::LitevSP::StallLogOn()) { const double t_ = LSP_NOW(); stmt; \
                               ::melonDS::LitevSP::GLJ.f += LSP_NOW() - t_; ::melonDS::LitevSP::GLJ.f##N++; } else { stmt; } } while (0)
// a draw call: also keep the slowest one and its key
#define LSP_GLDRAW(key, stmt) do { if (::melonDS::LitevSP::StallLogOn()) { const double t_ = LSP_NOW(); stmt; \
                               const double d_ = LSP_NOW() - t_; auto& j_ = ::melonDS::LitevSP::GLJ; j_.Draw += d_; j_.DrawN++; \
                               if (d_ > j_.DrawMax) { j_.DrawMax = d_; j_.DrawMaxKey = (int)(key); } } else { stmt; } } while (0)
// an emu-thread wait
#define LSP_WAIT(site, frame, stmt) do { if (::melonDS::LitevSP::StallLogOn()) { const double t_ = LSP_NOW(); stmt; \
                               ::melonDS::LitevSP::StallWait(site, LSP_NOW() - t_, frame); } else { stmt; } } while (0)

#else   // !LITEV_SOFTPROF

#define LSP_NOW()          0.0
#define LSP_ADD(f, d)      ((void)0)
#define LSP_NAME(n)        ((void)0)
#define LSP_EV(e)          ((void)0)
#define LSP_GLT(f, stmt)   do { stmt; } while (0)
#define LSP_GLDRAW(key, stmt) do { stmt; } while (0)
#define LSP_WAIT(site, frame, stmt) do { stmt; } while (0)

#endif  // LITEV_SOFTPROF
#endif  // LITEV_SOFTPROF_H
