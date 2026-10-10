/*
    LiteProfile - lightweight frame-breakdown profiler for melonDS (liteDS-v2).

    This is a TRIMMED port of the v1 fork's LiteProfile.h. Only the macro
    skeleton and a small per-frame counter struct are kept; the fork-specific
    hundreds-of-counters taxonomy is intentionally NOT ported.

    Everything compiles to nothing unless the build sets LITEV_PROFILE=1.
    Core call sites are deferred to later liteDS-v2 units; for Unit 0 the
    headless runner uses the FrameCounters struct + macros directly.
*/

#pragma once

#ifndef LITEV_PROFILE
#define LITEV_PROFILE 0
#endif

#if LITEV_PROFILE

#include <atomic>
#include <chrono>
#include <cstdint>

namespace melonDS::LiteProfile
{
using Clock = std::chrono::steady_clock;

// Per-frame counters. Deliberately small: the categories the v2 dispatch-core
// work (Milestones 1-2) needs from day one. Extend as call sites land.
struct FrameCounters
{
    // Scheduler
    std::atomic<uint64_t> SchedulerIterations{0};
    std::atomic<uint64_t> SchedulerEventsFired{0};

    // Per-event-type dispatch counts (indexed by NDS Event_* id). Sizing the
    // scheduler bucket: which event floods RunSystem's 2935 dispatches/frame
    // (Div/Sqrt hardware units? LCD scanline? timers?). 32 >= Event_MAX.
    std::atomic<uint64_t> SchedEventByType[32]{};

    // CPU execution time (nanoseconds within the frame)
    std::atomic<uint64_t> ARM9ExecNs{0};
    std::atomic<uint64_t> ARM7ExecNs{0};
    std::atomic<uint64_t> ARM7WaitNs{0};

    // Frame-decomposition timers (Unit 6 measurement): GPU3D geometry engine
    // run + the per-slice RunSystem event drain, so a frame decomposes into
    // ARM9 / ARM7 / GPU3D / system.
    std::atomic<uint64_t> GPU3DNs{0};
    std::atomic<uint64_t> RunSystemNs{0};

    // SPU mixer wall time (nanoseconds within the frame). SPU::Mix is a
    // scheduler event, so this is a CHILD of RunSystemNs, carved out to answer
    // "how big is the SPU chunk of the SPU+events bucket?" (liteDS-v2 Stage 1
    // SPU front). Purely diagnostic; compiled out unless LITEV_PROFILE=1.
    std::atomic<uint64_t> SPUMixNs{0};

    // M6.11 RunFrame decomposition (step 1): aim the NEON geometry work by
    // splitting the ~20ms in-race RunFrame bucket. RunFrameNs is the whole
    // NDS::RunFrame() wall time (the parent); the child buckets above plus the
    // DMA buckets below carve it up, and the residual (parent minus children)
    // is scheduler/event-dispatch + slice loop overhead.
    //
    // Nesting note: in this melonDS core the GXFIFO command engine does NOT run
    // synchronously inside ARM9 MMIO writes — CmdFIFOWrite only enqueues; the
    // geometry commands drain later in GPU3D::Run() (timed by GPU3DNs, called
    // once per scheduler slice OUTSIDE the ARM9 scope). So ARM9ExecNs already
    // excludes geometry math and there is no parent/child double-count between
    // ARM9 and GPU3D. What ARM9ExecNs does keep is the cheap FIFO-enqueue cost
    // of the thousands of GXFIFO MMIO writes per frame, which is correctly
    // ARM9-side work, not geometry. DMA is the one bucket that WAS nested: the
    // GXFIFO/geometry-adjacent DMA9 and DMA7 runs used to sit inside the ARM9 /
    // ARM7 scopes, so they are now timed separately and the CPU scopes wrap
    // only ARM9.Execute / ARM7.Execute.
    std::atomic<uint64_t> RunFrameNs{0};
    std::atomic<uint64_t> DMA9Ns{0};
    std::atomic<uint64_t> DMA7Ns{0};

    // GXFIFO command throughput. ExecuteCommand() is called thousands of times
    // per frame, so per-command clock_gettime would distort GPU3DNs badly — we
    // therefore time GPU3D at the Run()/FIFO-drain batch level (GPU3DNs) and
    // only COUNT commands here. GPU3DNs / GXCommands gives an ns/command figure
    // whose sanity (vs. a known-cheap command) exposes timing distortion.
    std::atomic<uint64_t> GXCommands{0};

    // M6.11 step 3 (NEON geometry) scope-sizing: how often CalculateLighting()
    // runs in the measured window. Used to decide whether the per-light
    // normal-transform/dot-product math is worth vectorizing for a given
    // workload (Shrek's race scene) vs. leaving it scalar. A cheap ++.
    std::atomic<uint64_t> LightingCalls{0};

    // Idle-loop fast-forward hits (Unit 6): how often the existing branch-to-self
    // IdleLoop detection (ARM.cpp Execute) fast-forwards each CPU to its slice
    // target.
    std::atomic<uint64_t> ARM9IdleHits{0};
    std::atomic<uint64_t> ARM7IdleHits{0};

    // IsIdleLoop compile-time reason histogram (target A investigation): counts
    // WHY backward-conditional loops are/aren't recognized as idle. Compile-time
    // (per block-compile), so these are static totals over the run, not per-frame
    // hotness. IdleCandidates = backward cond loops fed to IsIdleLoop; Accepted =
    // flagged idle; the Rej* buckets split the rejections by first-hit reason.
    // IdleRejRegDep is the cross-iteration register-recurrence reject that
    // LITEV_IDLE_AGGRESSIVE relaxes.
    std::atomic<uint64_t> IdleCandidates{0};
    std::atomic<uint64_t> IdleAccepted{0};
    std::atomic<uint64_t> IdleRejWriteMem{0};
    std::atomic<uint64_t> IdleRejCoproc{0};
    std::atomic<uint64_t> IdleRejBranch{0};
    std::atomic<uint64_t> IdleRejRegDep{0};

    // Block-transition taxonomy (populated once M1 dispatcher/linking lands)
    std::atomic<uint64_t> LinkedTransitions{0};
    std::atomic<uint64_t> DispatcherHits{0};
    std::atomic<uint64_t> DispatcherMisses{0};
    std::atomic<uint64_t> CppReentries{0};

    // Unit 4 direct-linking bookkeeping counters (C++-side; the per-hop asm
    // LinkedHops counter is intentionally not implemented — see the unit report).
    std::atomic<uint64_t> LinksPatched{0};   // outgoing sites patched site->target
    std::atomic<uint64_t> LinksUnlinked{0};  // sites rewritten target->dispatcher
    std::atomic<uint64_t> PendingPeak{0};    // high-water mark of pending-link maps
    std::atomic<uint64_t> LinkSitesEmitted{0};   // eligible exit sites (got a link slot)
    std::atomic<uint64_t> CommitStubEntries{0};  // commit-stub entries (runtime, emitted in the stub)
    std::atomic<uint64_t> DispatchOnlyExits{0};  // ineligible exit sites (plain dispatcher)
    std::atomic<uint64_t> ICacheHits{0};         // per-site inline-cache hits (runtime, LITEV_JIT_ICACHE)
    std::atomic<uint64_t> RasHits{0};            // dispatcher hits via the return-address stack (LITEV_JIT_RAS)
#ifdef LITEV_JIT_DIRECTPATCH
    // --- DIRECTPATCH: monomorphic exit-site promotion (LITEV_JIT_DIRECTPATCH) ---
    std::atomic<uint64_t> DirectGuardHits{0};    // promoted guard stub hits -> DIRECT B to target (runtime)
    std::atomic<uint64_t> DirectGuardMisses{0};  // promoted guard misses (target changed) -> fell to dispatcher
    std::atomic<uint64_t> DirectPromotions{0};   // sites promoted (guard stub emitted + exit-B patched)
    std::atomic<uint64_t> DirectDemotions{0};    // sites demoted (guard permanently missed -> reverted)
    std::atomic<uint64_t> DirectReverts{0};      // patches reverted wholesale on block invalidation
#endif

    // --- DraStic tile renderer async attribution (LITEV_SOFT3D_DRASTIC) ---
    std::atomic<uint64_t> TileGetLineCalls{0};   // 2D-consumer GetLine calls
    std::atomic<uint64_t> TileGetLineStalls{0};  // GetLine calls that hit RowReady==0 (row not ready)
    std::atomic<uint64_t> TileGetLineStallNs{0}; // wall time the consumer stalled in the RowReady fallback

    // Time-in-JIT vs time-in-C++ (nanoseconds within the frame)
    std::atomic<uint64_t> TimeInJitNs{0};
    std::atomic<uint64_t> TimeInCppNs{0};

    // M3 memory-fast-path accounting: how many times the ARM9 C++ memory helpers
    // are actually entered. With the LITEV_MEM_FAST tiers off these count every
    // block transfer / u32 load; with them on they count only guard-miss
    // fallbacks, so the OFF-vs-ON delta is the number of helper calls eliminated.
    std::atomic<uint64_t> MemBlock9HelperCalls{0};   // SlowBlockTransfer9 entries
    std::atomic<uint64_t> MemRead9U32HelperCalls{0}; // SlowRead9<u32> entries

    void Reset()
    {
        SchedulerIterations.store(0, std::memory_order_relaxed);
        SchedulerEventsFired.store(0, std::memory_order_relaxed);
        for (auto& c : SchedEventByType) c.store(0, std::memory_order_relaxed);
        ARM9ExecNs.store(0, std::memory_order_relaxed);
        ARM7ExecNs.store(0, std::memory_order_relaxed);
        ARM7WaitNs.store(0, std::memory_order_relaxed);
        GPU3DNs.store(0, std::memory_order_relaxed);
        RunSystemNs.store(0, std::memory_order_relaxed);
        SPUMixNs.store(0, std::memory_order_relaxed);
        RunFrameNs.store(0, std::memory_order_relaxed);
        DMA9Ns.store(0, std::memory_order_relaxed);
        DMA7Ns.store(0, std::memory_order_relaxed);
        GXCommands.store(0, std::memory_order_relaxed);
        LightingCalls.store(0, std::memory_order_relaxed);
        ARM9IdleHits.store(0, std::memory_order_relaxed);
        ARM7IdleHits.store(0, std::memory_order_relaxed);
        IdleCandidates.store(0, std::memory_order_relaxed);
        IdleAccepted.store(0, std::memory_order_relaxed);
        IdleRejWriteMem.store(0, std::memory_order_relaxed);
        IdleRejCoproc.store(0, std::memory_order_relaxed);
        IdleRejBranch.store(0, std::memory_order_relaxed);
        IdleRejRegDep.store(0, std::memory_order_relaxed);
        LinkedTransitions.store(0, std::memory_order_relaxed);
        DispatcherHits.store(0, std::memory_order_relaxed);
        DispatcherMisses.store(0, std::memory_order_relaxed);
        CppReentries.store(0, std::memory_order_relaxed);
        LinksPatched.store(0, std::memory_order_relaxed);
        LinksUnlinked.store(0, std::memory_order_relaxed);
        PendingPeak.store(0, std::memory_order_relaxed);
        LinkSitesEmitted.store(0, std::memory_order_relaxed);
        CommitStubEntries.store(0, std::memory_order_relaxed);
        DispatchOnlyExits.store(0, std::memory_order_relaxed);
        ICacheHits.store(0, std::memory_order_relaxed);
        RasHits.store(0, std::memory_order_relaxed);
#ifdef LITEV_JIT_DIRECTPATCH
        DirectGuardHits.store(0, std::memory_order_relaxed);
        DirectGuardMisses.store(0, std::memory_order_relaxed);
        DirectPromotions.store(0, std::memory_order_relaxed);
        DirectDemotions.store(0, std::memory_order_relaxed);
        DirectReverts.store(0, std::memory_order_relaxed);
#endif
        TimeInJitNs.store(0, std::memory_order_relaxed);
        TimeInCppNs.store(0, std::memory_order_relaxed);
        MemBlock9HelperCalls.store(0, std::memory_order_relaxed);
        MemRead9U32HelperCalls.store(0, std::memory_order_relaxed);
    }
};

// Global frame counters (one active frame at a time). The headless runner and
// later core call sites both reference this instance.
inline FrameCounters g_Frame;

inline uint64_t NowNs()
{
    return (uint64_t)std::chrono::duration_cast<std::chrono::nanoseconds>(
        Clock::now().time_since_epoch()).count();
}

inline void AddAtomic(std::atomic<uint64_t>& counter, uint64_t value = 1)
{
    counter.fetch_add(value, std::memory_order_relaxed);
}

// RAII scope timer: accumulates elapsed nanoseconds into an atomic counter.
struct ScopeTimer
{
    std::atomic<uint64_t>& Counter;
    uint64_t Start;
    explicit ScopeTimer(std::atomic<uint64_t>& counter)
        : Counter(counter), Start(NowNs()) {}
    ~ScopeTimer() { Counter.fetch_add(NowNs() - Start, std::memory_order_relaxed); }
};

inline void ResetFrame() { g_Frame.Reset(); }

} // namespace melonDS::LiteProfile

#define LITE_PROFILE_SCOPE(var_name, counter) melonDS::LiteProfile::ScopeTimer var_name(counter)
#define LITE_PROFILE_ADD(counter) melonDS::LiteProfile::AddAtomic(counter)
#define LITE_PROFILE_ADD_VALUE(counter, value) melonDS::LiteProfile::AddAtomic((counter), (value))
#define LITE_PROFILE_RESET_FRAME() melonDS::LiteProfile::ResetFrame()
#define LITE_PROFILE_NOW_NS() melonDS::LiteProfile::NowNs()

#else // !LITEV_PROFILE

#define LITE_PROFILE_SCOPE(var_name, counter) do { } while (0)
#define LITE_PROFILE_ADD(counter) do { } while (0)
#define LITE_PROFILE_ADD_VALUE(counter, value) do { } while (0)
#define LITE_PROFILE_RESET_FRAME() do { } while (0)
#define LITE_PROFILE_NOW_NS() 0ULL

#endif // LITEV_PROFILE
