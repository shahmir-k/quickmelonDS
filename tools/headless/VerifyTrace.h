/*
    liteDS-v2 headless harness - deterministic per-frame state trace,
    golden-trace verification, and JIT-vs-interp convergence check.

    See docs/liteDS-v2-plan.md Appendix B "Unit 1" and the orchestrator's
    revised oracle design: the authoritative oracle for later JIT-rewrite units
    is JIT-vs-JIT golden-trace comparison (record with the current unmodified
    JIT; future JIT changes must reproduce the trace bit-exactly). Interp
    comparison is kept only as a loose convergence sanity check because the JIT
    and interpreter legitimately diverge in mid-boot timing.
*/

#pragma once

#include <string>
#include "types.h"

namespace liteds
{

// Default fixed RTC epoch used in headless trace modes for determinism.
// 946684800 == 2000-01-01 00:00:00 UTC, which matches the DS RTC power-on
// default, so a recorded trace also matches a run that never touched the RTC.
constexpr long long kDefaultRtcEpoch = 946684800LL;

struct TraceRunConfig
{
    std::string rom;                          // path to the DS ROM (required)
    std::string dataDir = "./headless-data";  // firmware/save scratch dir
    bool jit = true;                          // JIT vs interpreter
    long long fixedRtcEpoch = kDefaultRtcEpoch;
    std::string instanceTag = "headless";     // save-file stem (isolation)
    std::string inputScript;                  // optional --input-script path
};

// --record-trace: run `frames` frames and write a fixed-size binary trace
// (header + one record per frame) to `outPath`. Returns a process exit code.
int RecordTrace(const TraceRunConfig& cfg, int frames, const std::string& outPath);

// --verify-trace: re-run the ROM and compare each frame's record against the
// golden trace at `tracePath`. Prints the first mismatch (frame + every
// differing field, expected vs actual) and returns nonzero; prints a summary
// and returns 0 on a full match.
int VerifyTrace(const TraceRunConfig& cfg, const std::string& tracePath);

// --verify-interp-converge: run a JIT instance and an interpreter instance side
// by side (both heap-allocated in one process), compare framebuffer hashes each
// frame, and report the convergence pattern. Returns 0 iff the final 60 frames
// are all identical. `cfg.jit` is ignored (both modes are built internally).
int VerifyInterpConverge(const TraceRunConfig& cfg, int frames);

// --mp-test: two-instance local-multiplayer harness. Phase 0 runs two NDS
// instances concurrently on two threads (no MP wired yet) to prove concurrent
// two-instance execution is safe under the LITEV stack. Later phases wire the
// shared LocalMP + health metrics. Returns 0 iff both instances complete all
// frames without a crash.
int MPTest(const TraceRunConfig& cfg, int frames,
           const std::string& script0, const std::string& script1);

} // namespace liteds
