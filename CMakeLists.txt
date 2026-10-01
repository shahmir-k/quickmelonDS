cmake_minimum_required(VERSION 3.16)

cmake_policy(VERSION 3.15)
if (POLICY CMP0076)
    cmake_policy(SET CMP0076 NEW)
endif()

set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake" ${CMAKE_MODULE_PATH})
set(CMAKE_USER_MAKE_RULES_OVERRIDE "${CMAKE_CURRENT_SOURCE_DIR}/cmake/DefaultBuildFlags.cmake")
set(CMAKE_OSX_DEPLOYMENT_TARGET "10.15" CACHE STRING "Minimum OS X deployment version")

option(USE_VCPKG "Use vcpkg for dependency packages" OFF)
if (USE_VCPKG)
	include(ConfigureVcpkg)
endif()

project(melonDS
    VERSION 1.1
    DESCRIPTION "DS emulator, sorta"
    HOMEPAGE_URL "https://melonds.kuribo64.net"
    LANGUAGES C CXX)

include(CheckSymbolExists)
include(CheckLibraryExists)
include(CMakeDependentOption)
include(CheckIPOSupported)

include(SetupCCache)
include(Sanitizers)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

function(detect_architecture symbol arch)
    if (NOT DEFINED ARCHITECTURE)
        set(CMAKE_REQUIRED_QUIET 1)
        check_symbol_exists("${symbol}" "" ARCHITECTURE_${arch})
        unset(CMAKE_REQUIRED_QUIET)

        # The output variable needs to be unique across invocations otherwise
        # CMake's crazy scope rules will keep it defined
        if (ARCHITECTURE_${arch})
            set(ARCHITECTURE "${arch}" PARENT_SCOPE)
            set(ARCHITECTURE_${arch} 1 PARENT_SCOPE)
            add_definitions(-DARCHITECTURE_${arch}=1)
        endif()
    endif()
endfunction()

detect_architecture("__x86_64__" x86_64)
detect_architecture("__i386__" x86)
detect_architecture("__arm__" ARM)
detect_architecture("__aarch64__" ARM64)

cmake_dependent_option(ENABLE_JIT "Enable JIT recompiler" ON
    "ARCHITECTURE STREQUAL x86_64 OR ARCHITECTURE STREQUAL ARM64;NOT ARCHITECTURE STREQUAL x86_64 OR NOT APPLE" OFF)
cmake_dependent_option(ENABLE_JIT_PROFILING "Enable JIT profiling with VTune" OFF "ENABLE_JIT" OFF)
option(ENABLE_OGLRENDERER "Enable OpenGL renderer" ON)

check_ipo_supported(RESULT IPO_SUPPORTED)
cmake_dependent_option(ENABLE_LTO_RELEASE "Enable link-time optimizations for release builds" ON "IPO_SUPPORTED" OFF)
cmake_dependent_option(ENABLE_LTO "Enable link-time optimizations" OFF "IPO_SUPPORTED" OFF)

if (ENABLE_LTO_RELEASE)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
endif()

if (ENABLE_LTO)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
endif()

if (NOT APPLE)
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} -s")
endif()

if (WIN32)
    option(BUILD_STATIC "Statically link dependencies" OFF)
    add_definitions(-DWIN32_LEAN_AND_MEAN -DNOMINMAX)
    if (NOT MINGW)
        add_definitions(-D__WIN32__ -D_CRT_SECURE_NO_WARNINGS -D_USE_MATH_DEFINES)
        if (BUILD_STATIC)
            set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
        endif()
    elseif (BUILD_STATIC AND MINGW)
        set(CMAKE_FIND_LIBRARY_SUFFIXES .a)
    endif()
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

option(ENABLE_GDBSTUB "Enable GDB stub" ON)
if (ENABLE_GDBSTUB)
    add_definitions(-DGDBSTUB_ENABLED)
endif()

option(BUILD_QT_SDL "Build Qt/SDL frontend" ON)

# liteDS-v2 additions (all default OFF -> upstream-identical build with no flags).
option(LITEV_HEADLESS "Build the liteDS-headless benchmark harness" OFF)
option(LITEV_PROFILE "Enable LiteProfile frame-breakdown counters" OFF)
if (LITEV_PROFILE)
    # Reach the core too (not just the headless harness) so in-core call sites
    # (e.g. the Unit 3 dispatcher re-entry counters in ARM.cpp) are compiled in.
    add_compile_definitions(LITEV_PROFILE=1)
endif()
# Software-render phase profiler: names every render thread and logs a per-60-frame
# decomposition (emu barrier / snapshot, 3D clear/raster/final + per-band, 2D wall +
# per-band GetLine block, whole render critical path). Diagnostic only; default OFF.
option(LITEV_SOFTPROF "liteV: software-render phase profiler (thread names + phase ms)" OFF)
if (LITEV_SOFTPROF)
    add_compile_definitions(LITEV_SOFTPROF=1)
endif()
# O(1) offset-indexed dispatch for the ARM9 memory-mapped I/O 32-bit path
# (NDS::ARM9IORead32 / ARM9IOWrite32). Stock melonDS routes these through a
# switch(addr) over the FULL sparse address (0x04000004 .. 0x04FFFA24) which the
# compiler lowers to a data-dependent compare/binary-search tree (~6 mispredict-
# prone branches per access on the in-order A55). This flag adds a fast path that,
# for word-aligned accesses in the primary 8 KB I/O window (0x04000000-0x04001FFF),
# dispatches on the dense word index (addr & 0x1FFF) >> 2 -- the compiler lowers
# that to an indexed jump. Out-of-window regs (0x041xxxxx / 0x04004xxx / 0x04FFFAxx)
# and misaligned accesses fall through to the ORIGINAL switch, which is preserved
# verbatim. Each fast case keeps its original body byte-for-byte and its label is a
# mechanical compile-time reduction (LABEL & 0x1FFF) >> 2 of the original case, so
# the mapping is 1:1 by inspection. Default OFF => flag-OFF build is byte-identical;
# flag-ON must be bit-exact (state + framebuffer hashes over a 2000-frame trace).
option(LITEV_IO_DISPATCH_TABLE "liteV: O(1) offset-indexed dispatch for ARM9 32-bit I/O reads/writes" OFF)
if (LITEV_IO_DISPATCH_TABLE)
    add_compile_definitions(LITEV_IO_DISPATCH_TABLE=1)
endif()
# Unit 2: shadow-mode equivalence assert for the CyclesBudget slot. Debug-only gate;
# implies nothing else. Aborts if the budget and timestamp/target mechanisms disagree.
option(LITEV_SHADOW_ASSERT "Assert CyclesBudget == timestamp/target equivalence" OFF)
if (LITEV_SHADOW_ASSERT)
    add_compile_definitions(LITEV_SHADOW_ASSERT=1)
endif()

# Unit 3: AArch64-only emitted JIT dispatcher. When ON, JIT block exits jump to a
# generated dispatcher stub that does the fast block lookup inline and branches
# straight into the next block, staying in JIT context for the whole slice; it
# only returns to C++ (ARM_Ret) on budget-expiry, StopExecution, region miss, or
# lookup miss. Default OFF -> the build is bit-identical to the shadow-budget
# tree. The x64 JIT ALWAYS keeps the unmodified upstream C++ dispatch loop; this
# option therefore requires an AArch64 target and errors at configure time on x86.
option(LITEV_JIT_DISPATCH "AArch64 emitted JIT block dispatcher (chained block exits)" OFF)
if (LITEV_JIT_DISPATCH)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_DISPATCH=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_DISPATCH requires an AArch64 target "
                            "(the x64 JIT keeps the upstream C++ dispatch loop).")
    endif()
endif()

# Stage 2 (liteDS-v2 core-speed / DraStic parity): native-NZCV condition guard.
# EXACTNESS-PRESERVING. For guest ARM instructions with a COMPOUND condition
# (cond >= 8: HI/LS/GE/LT/GT/LE), the upstream CheckCondition emits a 5-instruction
# flag-table lookup out of RCPSR. Since this backend keeps guest CPSR in a GP reg
# (not host NZCV), we can instead transiently `MSR NZCV, X<cpsr>` and use a native
# `B.<inverted-cond>` skip (2 instructions). Same architectural branch decision,
# fewer host ops / smaller blocks. Single-flag conditions (cond < 8, already a
# 1-instruction TBZ/TBNZ) are left untouched. No register-cache / cycle-model
# interaction, so ON must stay bit-exact on the exact-timing goldens. AArch64 only.
option(LITEV_JIT_CONDFOLD "Stage 2: native-NZCV guard for compound conditions (exactness-preserving, AArch64)" OFF)
if (LITEV_JIT_CONDFOLD)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_CONDFOLD=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_CONDFOLD requires an AArch64 target.")
    endif()
endif()

# Stage 1 of the fixed-register / CPSR-in-NZCV JIT rearchitecture (DraStic teardown
# 01 §6.1). An UNCONDITIONAL flag-setting ADD/SUB/RSB/CMP/CMN leaves its guest NZCV
# resident in the host PSTATE instead of extracting it into the emulated RCPSR flag
# word; the word is reconstructed lazily (bit-exact CSET/BFI) only at consumers /
# host-NZCV clobbers / block boundaries, and CheckCondition then evaluates the guest
# condition natively on the resident host flags. Pure codegen change: cycle counts,
# timing and the register cache are untouched. Default OFF; flag-off is byte-identical.
option(LITEV_JIT_FIXEDREG "Stage 1: keep guest CPSR flags in host NZCV on the ALU hot path (AArch64)" OFF)
if (LITEV_JIT_FIXEDREG)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_FIXEDREG=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_FIXEDREG requires an AArch64 target.")
    endif()
endif()

# GLOBALREG (DraStic teardown 01 §6.1 — global cross-block fixed register
# allocation). Builds ON TOP of LITEV_JIT_FIXEDREG + LITEV_JIT_DISPATCH. Pins a
# SUBSET of the never-banked guest GP registers (r0..r7) to fixed callee-saved
# host registers (W19..) GLOBALLY: they are loaded ONCE at ARM_Dispatch (JIT
# slice entry), spilled ONCE at ARM_Ret (slice exit), and kept LIVE across block
# boundaries / the emitted dispatcher / linked chains (callee-saved => preserved
# for free across the dispatcher and any C helper). The per-block RegisterCache
# no longer reload-on-first-use or spill-at-block-end the pinned regs, removing
# the block-boundary register traffic DraStic eliminates. Pinning only r0..r7
# (never mode-banked) keeps mode/exception switches transparent; the sole
# file-coherence point is the interpreter fallback, which is bracketed with an
# explicit spill-before / reload-after. Codegen-only: cycles/timing/register-file
# semantics unchanged, so flag-ON must stay bit-exact on the exact-timing +
# torture goldens. Default OFF -> emission-identical to the FIXEDREG tree.
# AArch64 only (the x64 JIT keeps the upstream dynamic per-block register cache).
option(LITEV_JIT_GLOBALREG "GLOBALREG: globally pin never-banked guest regs to fixed host regs across block boundaries (AArch64)" OFF)
if (LITEV_JIT_GLOBALREG)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_GLOBALREG=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_GLOBALREG requires an AArch64 target.")
    endif()
endif()

# FULL LAZY-FLAGS (DraStic teardown 01 §6.1 — the terminal step of the fixed-register /
# CPSR-in-NZCV rearchitecture). LITEV_JIT_FIXEDREG kept a partial lazy-NZCV but still
# carried the ENTIRE guest CPSR (flags + control bits) live in a permanently-reserved
# host register (RCPSR = W27), loaded/spilled per JIT slice and committed at every block
# boundary. This flag COMPLETES the transition: the guest CONTROL word becomes always-
# canonical in ARM::CPSR MEMORY (every control-bit RMW writes memory directly, no cross-
# instruction caching), the guest NZCV lives EITHER in host PSTATE (deferred, the hot
# path) OR in ARM::CPSR memory (flushed) tracked by the existing NZCVDeferred/NZCVCondValid
# residency, and RCPSR (W27) is DELETED as a CPSR carrier. The freed callee-saved W27 is
# then globally pinned to guest r7 (the 8th GLOBALREG pin), matching DraStic's wider
# fixed-register file. Pure codegen: guest cycle counts, event-check points and every
# architecturally-visible guest observation point are byte-identical (only host register
# allocation / instruction layout changes) -> MP-safe / bit-exact. Builds ON TOP of and
# REQUIRES LITEV_JIT_DISPATCH + LITEV_JIT_FIXEDREG + LITEV_JIT_GLOBALREG + LITEV_JIT_CONDFOLD
# (the shipping .dev config). AArch64 only. Default OFF -> flag-off codegen is byte-identical
# (every edit is inside #ifdef LITEV_JIT_LAZYFLAGS or provably no-op when OFF).
option(LITEV_JIT_LAZYFLAGS "Full lazy-flags: control word in memory, NZCV in host PSTATE/memory, delete RCPSR + pin r7->W27 (AArch64)" OFF)
if (LITEV_JIT_LAZYFLAGS)
    if (NOT CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        message(FATAL_ERROR "LITEV_JIT_LAZYFLAGS requires an AArch64 target.")
    endif()
    if (NOT (LITEV_JIT_DISPATCH AND LITEV_JIT_FIXEDREG AND LITEV_JIT_GLOBALREG AND LITEV_JIT_CONDFOLD))
        message(FATAL_ERROR "LITEV_JIT_LAZYFLAGS requires LITEV_JIT_DISPATCH, "
                            "LITEV_JIT_FIXEDREG, LITEV_JIT_GLOBALREG and LITEV_JIT_CONDFOLD all ON "
                            "(the shipping .dev config).")
    endif()
    add_compile_definitions(LITEV_JIT_LAZYFLAGS=1)
endif()

# ---------------------------------------------------------------------------
# Fetch-stall lever A: LITEV_JIT_PERFMAP (attribution aid, NOT a perf change).
# The AArch64 JIT emits its blocks/stubs into one opaque anonymous-rwx blob, so
# simpleperf attributes ~39.5% of L1I refills to a single unsplittable symbol.
# With this option ON the JIT appends a linux-perf / simpleperf "perf-<pid>.map"
# (one `<hexaddr> <hexsize> <name>` line per emitted region: jit_a9_/jit_a7_<guest>
# blocks, jit_dispatcher9/7, jit_commit9/7, jit_jumpto9/7_<k>, jit_readbanked/
# jit_writebanked, jit_pstore_/jit_pload_ patched load/store funcs) so a later
# `simpleperf report` can split the blob per block. The emitter is PURELY
# OBSERVATIONAL (reads GetRXPtr()/GetCodeOffset(), writes a text file, emits ZERO
# host instructions and never advances m_code) -> flag-ON JIT output is byte-
# identical to flag-OFF and emulation stays bit-exact / MP-timing-exact. It is
# ALSO runtime-gated: nothing is written unless an output directory is named at
# run time (env LITEV_PERFMAP_DIR, or on Android the `debug.litev.perfmap`
# system property), so the flag can ship ON in a .dev build at zero file cost
# until armed. AArch64 only (the map machinery lives in the A64 backend). Default
# OFF -> the code is #ifdef'd out entirely (zero overhead, byte-identical binary).
option(LITEV_JIT_PERFMAP "Emit a simpleperf perf-<pid>.map so the JIT blob can be split per block (attribution aid; runtime-gated; bit-exact)" OFF)
if (LITEV_JIT_PERFMAP)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_PERFMAP=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_PERFMAP requires an AArch64 target (the map machinery lives in the A64 JIT backend).")
    endif()
endif()

# Unit 4: direct block linking (patch, don't dispatch). Builds ON TOP of the
# Unit 3 dispatcher: an eligible block exit whose next PC is a compile-time
# constant emits a per-hop-commit guard followed by a patchable `B` (initially to
# the dispatcher). When the target block exists the site is patched to branch
# straight into it, bypassing the dispatcher's inline lookup. Three independent
# kill switches gate the three eligible exit CLASSES (unconditional static
# branch / conditional-branch edge / block-end fall-through) so a regression can
# be bisected to a class. All default ON, but each only takes effect when
# LITEV_JIT_DISPATCH is ON (linking has no meaning without the dispatcher). With
# all three OFF the build is bit-identical to the pure Unit-3 dispatcher.
option(LITEV_LINK_UNCOND "Unit 4: link unconditional static branch exits" ON)
option(LITEV_LINK_COND "Unit 4: link conditional-branch edge exits" ON)
option(LITEV_LINK_FALLTHROUGH "Unit 4: link block-end fall-through exits" ON)
if (LITEV_JIT_DISPATCH AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
    set(_litev_any_link OFF)
    if (LITEV_LINK_UNCOND)
        add_compile_definitions(LITEV_LINK_UNCOND=1)
        set(_litev_any_link ON)
    endif()
    if (LITEV_LINK_COND)
        add_compile_definitions(LITEV_LINK_COND=1)
        set(_litev_any_link ON)
    endif()
    if (LITEV_LINK_FALLTHROUGH)
        add_compile_definitions(LITEV_LINK_FALLTHROUGH=1)
        set(_litev_any_link ON)
    endif()
    # Umbrella: the link registry / JitBlock link fields / patch machinery are
    # compiled in whenever at least one exit class links. All three OFF -> the
    # bookkeeping vanishes and the tree is the pure Unit-3 dispatcher.
    if (_litev_any_link)
        add_compile_definitions(LITEV_JIT_LINK=1)
    endif()
endif()

# Per-site block inline cache (ICACHE). An earlier single-entry return-target cache
# caught only ~6.5% because it keyed on a GLOBAL last-target; the dispatcher target stream is instead
# strongly PER-SITE monomorphic/bimodal (offline replay of the shrek-race 3400f
# stream: 76.7% of lookups at sites with <=2 distinct targets; per-site N=2 covers
# 93.6% vs global-N1 15%). ICACHE gives each dynamic exit site its own 2-way
# (guest-PC -> host-block-ptr) cache, checked in the dispatcher AFTER instrAddr is
# computed but BEFORE the region-bounds + cache-cold FastBlockLookup gather -> a hit
# skips the multi-MB indexed load. Entries are filled ONLY from a dispatcher
# region-hit resolve (Option A; the C++-reentry fill added <2%) and EPOCH-GUARDED:
# every block invalidation / cache reset bumps ARM::ICacheEpoch, so all entries go
# stale in O(1) (single-block invalidations are ~4/3400f in steady state). Pure
# predictor (same block runs, same cycle accounting as the plain hit path) -> BIT-EXACT /
# MP-safe. Default OFF -> flag-off byte-identical (all code behind #ifdef).
option(LITEV_JIT_ICACHE "Per-site 2-way block inline cache in the dispatcher (guest-PC->host-ptr), epoch-guarded" OFF)
if (LITEV_JIT_ICACHE)
    if (LITEV_JIT_DISPATCH AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_ICACHE=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_ICACHE requires LITEV_JIT_DISPATCH and an AArch64 target.")
    endif()
endif()

# DIRECTPATCH: monomorphic indirect-branch exit-site promotion. Builds ON TOP of
# LITEV_JIT_ICACHE (piggybacks its per-site slot + the dispatcher's inline-cache
# hit path). When a dynamic exit site resolves to the SAME target for N consecutive
# ICACHE hits (measured MONOMORPHIC), the site's exit `B dispatcher` is runtime-
# patched to a tiny per-site GUARD STUB that reproduces the dispatcher's per-hop
# commit (a-d) + a single instrAddr==expected guard, then a DIRECT B into the target
# block's host entry -- turning the dispatcher's shared, poorly-predicted indirect BR
# into a perfectly-predicted direct branch and skipping the 2-way load/epoch chain.
# The guard keeps correctness identical to the ICACHE key check (a wrong target falls
# back to the dispatcher); every block invalidation reverts all live patches (RX is
# append-only within an epoch, so a stale direct edge can never survive). Pure host
# dispatch mechanics with byte-identical guest cycle accounting -> BIT-EXACT / MP-safe.
# Default OFF -> flag-off byte-identical (all code behind #ifdef LITEV_JIT_DIRECTPATCH,
# incl. the ICACHE entry stays 32B / LSL#5 when OFF).
option(LITEV_JIT_DIRECTPATCH "Promote MONOMORPHIC dynamic exit sites to direct guard-stub branches (needs ICACHE)" OFF)
if (LITEV_JIT_DIRECTPATCH)
    if (LITEV_JIT_DISPATCH AND LITEV_JIT_ICACHE AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_DIRECTPATCH=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_DIRECTPATCH requires LITEV_JIT_DISPATCH, LITEV_JIT_ICACHE and an AArch64 target.")
    endif()
endif()

# Unit 5 (M2.1): event-true scheduler slices. When ON, each scheduler iteration
# runs the CPUs to the TRUE next deadline (next scheduled event / frame end /
# next timer overflow) instead of the fixed kMaxIterationCycles cap, so the CPUs
# stay in generated code for whole event intervals. Independent of, but designed
# to combine with, LITEV_JIT_DISPATCH. Default OFF -> NextTarget() keeps the fixed
# cap and the build is bit-identical to the pre-Unit-5 tree.
#
# SEMANTIC NOTE: longer slices coarsen the ARM9/ARM7 interleave, an OBSERVABLE
# timing change (cross-CPU shared-memory/IPC visibility shifts). The flag-ON build
# is therefore NOT bit-exact vs the flag-OFF golden by design; verify it against a
# trace recorded with the flag ON.
option(LITEV_EVENT_SLICES "Unit 5: event-true scheduler slices (remove fixed iteration cap)" OFF)
if (LITEV_EVENT_SLICES)
    add_compile_definitions(LITEV_EVENT_SLICES=1)
endif()

# Milestone 3 (M3): tiered memory fast paths. AArch64-only (the A64 JIT emits the
# guarded raw accesses); the x64 JIT keeps the unmodified upstream helper paths.
# These claim ZERO semantic change: every fast path adds EXACTLY the cycles the
# slow helper would (cycles are accounted at compile time by Comp_AddCycles_*,
# independent of the runtime memory path), reads/writes the SAME bytes, and falls
# back to the existing helper on any runtime-guard miss. Default OFF -> the build
# is emission-identical to the pre-M3 tree (every fast path is #if'd out).
#
#   LITEV_MEM_FAST         umbrella; ON enables the two sub-flags below by default.
#   LITEV_MEM_DTCM_BLOCK   Tier A: DTCM-hit inline path for LDM/STM block xfers
#                          (plan order 1; the single biggest memory-helper cost).
#   LITEV_MEM_MAINRAM_LOAD Tier A: MainRAM-hit inline path for single u32 loads
#                          (plan order 4; SlowRead9<u32>'s dominant case). Stores
#                          stay on the helper (no MainRAM raw-store thunk: v1
#                          rejection, preserves JIT invalidation semantics).
# Each sub-flag is independently toggleable for per-shape bisection.
option(LITEV_MEM_FAST "M3: tiered memory fast paths (DTCM block + MainRAM load)" OFF)
option(LITEV_MEM_DTCM_BLOCK "M3 Tier A: DTCM inline path for LDM/STM block transfers" ${LITEV_MEM_FAST})
option(LITEV_MEM_MAINRAM_LOAD "M3 Tier A: MainRAM inline path for single u32 loads" ${LITEV_MEM_FAST})
if (LITEV_MEM_DTCM_BLOCK OR LITEV_MEM_MAINRAM_LOAD)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        if (LITEV_MEM_DTCM_BLOCK)
            add_compile_definitions(LITEV_MEM_DTCM_BLOCK=1)
        endif()
        if (LITEV_MEM_MAINRAM_LOAD)
            add_compile_definitions(LITEV_MEM_MAINRAM_LOAD=1)
        endif()
    else()
        message(FATAL_ERROR "LITEV_MEM_FAST requires an AArch64 target "
                            "(the x64 JIT keeps the upstream helper paths).")
    endif()
endif()

# 2026-09-16 (DTCM-engagement drill): map the DTCM region into the fastmem window so DTCM guest
# accesses inline via fastmem instead of faulting -> slow helper. Upstream's MapAtAddress hole-punch
# is not guarded against the DTCM region itself, so it maps NOTHING for DTCM (marks MappedRW over an
# unmapped hole) -> every DTCM fastmem access SIGSEGVs and is patched to the slow helper. DTCM is
# 84-99% of all slow-mem calls (the stack). Byte+cycle-exact: DTCM backing = MemoryBase+DTCMOffset =
# exactly what SlowRead9/Write9 use; DTCM is non-executable so writes need no JIT invalidation.
# Runtime prop debug.litev.dtcmfastmem (read once at init; when unset it defaults ON/1 -- see
# ARMJIT_Memory.cpp DTCMFastmem init) for a single-binary A/B.
option(LITEV_MEM_DTCM_FASTMEM "DTCM-engagement drill: map DTCM into the fastmem window (AArch64)" OFF)
if (LITEV_MEM_DTCM_FASTMEM)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_MEM_DTCM_FASTMEM=1)
    else()
        message(FATAL_ERROR "LITEV_MEM_DTCM_FASTMEM requires an AArch64 target.")
    endif()
endif()

# DraStic LDM/STM block-transfer match (docs/drastic-teardown/01-cpu-recompiler.md
# §7.1): DraStic translates every LDM/STM to a single BL into a hand-written
# straight-line stub, one per register count (arm64_load_block1..16 /
# arm64_store_block1..16), that moves the guest words with ldp/stp PAIRS (two
# words per instruction) instead of an emitted per-word loop. melonDS on this
# build already pairs the stack marshalling, but (a) its inline DTCM/MainRAM
# block tiers copy the contiguous memory side with SCALAR ldr/str per word, and
# (b) the MainRAM-hit inline block LOAD tier (the dominant ~86%-of-slow-reads
# case) is left disabled, so those LDMs fall through to the per-word
# SlowBlockTransfer9 helper. This flag ports the DraStic win: it enables the
# MainRAM inline block-LOAD tier AND pairs the contiguous memory side of the
# DTCM+MainRAM inline copies with ldp/stp. Same exactness class as
# LITEV_MEM_DTCM_BLOCK: cycles baked at compile time by
# Comp_AddCycles_*, the paired copy moves the SAME bytes in the same order, and
# any runtime guard miss falls back to the exact helper. Default OFF ->
# emission-identical to the pre-LDMSTM tree.
option(LITEV_JIT_LDMSTM "DraStic-style ldp/stp-paired inline LDM/STM block transfers + MainRAM block-load tier (AArch64)" OFF)
if (LITEV_JIT_LDMSTM)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_LDMSTM=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_LDMSTM requires an AArch64 target "
                            "(the x64 JIT keeps the upstream helper paths).")
    endif()
endif()

# Hoist the per-element region dispatch out of SlowBlockTransfer9 for blocks that lie
# wholly inside one no-side-effect TCM (ITCM/DTCM) — classify once and loop directly
# instead of re-running the ITCM/DTCM/region classification per word (SlowBlockTransfer9
# is ~2.6% of the emu thread; LDM/STM stack ops hit non-fastmem TCM). BIT-EXACT (same
# values/addresses, ITCM writes still CheckAndInvalidate per word), MP-safe (no timing
# change), straddling/MMIO blocks fall through to the exact per-element path. Default OFF.
option(LITEV_JIT_BLOCKXFER_FAST "Hoist SlowBlockTransfer9's region dispatch for whole-in-TCM blocks (bit-exact)" OFF)
if (LITEV_JIT_BLOCKXFER_FAST)
    add_compile_definitions(LITEV_JIT_BLOCKXFER_FAST=1)
endif()

# DraStic JIT emit-quality (teardown 01, FUN_001912f0): merge the host NZCV flags into
# guest CPSR with one MRS + masked AND/ORR (3 host instrs) instead of a per-flag CSET+BFI
# pair (8 instrs) for full-flag arithmetic S-ops (CMP/SUBS/ADDS...). Guest CPSR N/Z/C/V
# == host NZCV MRS layout -> bit-exact. AArch64-only; the x64 path keeps CSET/BFI.
option(LITEV_JIT_FLAGMERGE "DraStic: one-shot MRS-NZCV flag merge vs per-flag CSET+BFI (AArch64)" OFF)
if (LITEV_JIT_FLAGMERGE)
    if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_FLAGMERGE=1)
    endif()
endif()
add_subdirectory(src)

# --- liteDS-v2 core-fix: export LITEV_* configuration as a PUBLIC usage requirement ---
# The LITEV_* feature macros above are set with add_compile_definitions(), which is
# DIRECTORY-scoped: it configures core's own sources (and core's in-tree subdirectory
# consumers: the Qt/SDL frontend and the headless harness) but is NOT part of `core`'s
# exported usage requirements. Several of these macros change the LAYOUT of core public
# types that downstream code allocates and constructs:
#   * LITEV_JIT_DISPATCH -> Compiler::DispatcherEntry[]  (Compiler is embedded in ARMJIT)
#   * LITEV_JIT_LINK     -> ARMJIT::PendingLinks9/7 + Compiler link fields
# and `melonDS::ARMJIT JIT` is the FIRST member of NDS, so any of these shifts every
# later NDS member and grows sizeof(NDS).
#
# An OUT-OF-TREE dependent that links `core` and includes its public headers (e.g. the
# Android JNI frontend target `melonDS-android-frontend`, whose MelonInstance.cpp does
# `new NDS(...)`) lives in a DIFFERENT directory scope and therefore does NOT inherit
# these directory definitions. It would then size `operator new(sizeof(NDS))` for a
# SMALLER layout than NDS::NDS (compiled in core with the flags) constructs, so a late
# member write (a FIFO past the JIT member) runs off the end of the undersized heap
# block -> SIGSEGV/SEGV_ACCERR into the allocator guard page during construction.
#
# Re-export every LITEV_* directory definition as a PUBLIC compile definition of `core`
# so ALL dependents (in-tree and out-of-tree) compile against one identical ABI. This
# adds nothing new to core's own sources (they already have the macros via the
# directory scope, so the in-tree/headless build stays byte-identical); it only makes
# the same macros flow across the `target_link_libraries(... core ...)` boundary.
get_directory_property(_litev_dir_defs COMPILE_DEFINITIONS)
foreach(_litev_def IN LISTS _litev_dir_defs)
    if(_litev_def MATCHES "^LITEV_")
        target_compile_definitions(core PUBLIC ${_litev_def})
    endif()
endforeach()

if (BUILD_QT_SDL)
    add_subdirectory(src/frontend/qt_sdl)
endif()

if (LITEV_HEADLESS)
    add_subdirectory(tools/headless)
endif()
