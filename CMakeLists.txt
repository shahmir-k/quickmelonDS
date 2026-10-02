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
# liteV soft-2D threading: banded deferred software-2D raster. Snapshots the
# per-scanline 2D register state on the emu thread, then rasterizes all 192
# scanlines off the LCD critical path at VBlank, split into N contiguous bands
# across a small helper-thread pool (mirrors GPU3D_Soft's threading). Default OFF
# => flag-OFF build is byte-identical; flag-ON must be bit-exact (top/bottom/audio
# framebuffer hashes identical to flag-OFF).
option(LITEV_SOFT2D_THREADED "liteV: banded deferred software 2D raster off the LCD critical path" OFF)
if (LITEV_SOFT2D_THREADED)
    add_compile_definitions(LITEV_SOFT2D_THREADED=1)
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
# Async software 3D: let the banded raster run PAST VBlank into the next frame.
# Stock melonDS starts the 3D raster at VCount 215 and hard-barriers the emu thread on
# it at the next VBlank (GPU::VBlank -> Finish3DRendering), so the raster gets only
# ~1/2 a frame of wall. MEASURED on the Shrek race (A55, software renderer): the raster
# needs ~26 ms but is given ~15 ms, so the emu stalls ~9.4 ms EVERY frame -- that stall
# IS the 40fps wall. The game flushes 3D geometry only every OTHER frame, so the raster
# actually has two frames of headroom. This flag barriers the emu ONLY when VBlank is
# about to mutate state the raster still reads (geometry re-sort / Render* regs), and
# only flattens texture VRAM behind that same barrier. Output is unchanged (the 2D
# compositor already paces against the raster per-scanline via Sema_ScanlineCount);
# FBHASH-gated. Default OFF.
option(LITEV_SOFT3D_ASYNC "liteV: soft 3D raster may span into the next frame (no VBlank barrier unless state is mutated)" OFF)
if (LITEV_SOFT3D_ASYNC)
    add_compile_definitions(LITEV_SOFT3D_ASYNC=1)
endif()
# DRASTIC: DraStic-style TILE-BASED software 3D renderer (TileRenderer3D, GPU3D_TileSoft.*).
# Two planes per pixel (colour + packed depth|id) in 32 KB tiles that fit one core's L1D,
# polygons bucketed per 16-line block, AA/edge/fog recomputed in a post-pass. Selected at
# GPU_Soft.cpp construction instead of SoftRenderer3D; the new files compile to nothing when
# OFF. Render-only: emulation state is unchanged. Default OFF.
option(LITEV_SOFT3D_DRASTIC "liteV: DraStic-style tile-based software 3D renderer (TileRenderer3D)" OFF)
if (LITEV_SOFT3D_DRASTIC)
    add_compile_definitions(LITEV_SOFT3D_DRASTIC=1)
endif()
option(LITEV_SOFT2D_DEPTH2 "liteV: 2D depth-2 async (double-buffered 2D snapshots + emu runs a frame ahead of the 2D compositor; removes the VBlank bar2D barrier). For the DraStic tile renderer." OFF)
if (LITEV_SOFT2D_DEPTH2)
    add_compile_definitions(LITEV_SOFT2D_DEPTH2=1)
endif()
option(LITEV_TILE_COORD "liteV: DraStic-style 3D decouple — double-buffered geometry so the emu builds ahead without blocking on the raster" OFF)
if (LITEV_TILE_COORD)
    add_compile_definitions(LITEV_TILE_COORD=1)
endif()
option(LITEV_POLY_RADIX "liteV: allocation-free stable radix Y-sort for the VBlank polygon sort" OFF)
if (LITEV_POLY_RADIX)
    add_compile_definitions(LITEV_POLY_RADIX=1)
endif()
option(LITEV_SNAP_DIRTY "liteV: dirty-incremental shadow snapshots (tex + BGOBJ flat VRAM) instead of blind full memcpy" OFF)
if (LITEV_SNAP_DIRTY)
    add_compile_definitions(LITEV_SNAP_DIRTY=1)
endif()
# 4-wide NEON port of the per-pixel 2D BG/OBJ colour-special-effects compositor
# (SoftRenderer2D::ColorComposite, called 256×/scanline from DrawScanline_BGOBJ).
# The scalar version is a ~134-instruction out-of-line CALL PER PIXEL (branchy
# blend/window/priority logic that clang does NOT auto-vectorize). This lever
# replaces it with a branchless NEON kernel (arm masks/selects) doing 4 px/iter,
# mirroring DraStic's NEON-vectorized 2D composite (teardown FUN_001494a4, 927
# NEON ops). Bit-exact with the scalar path (top/bottom/audio hashes identical);
# all changes behind #ifdef LITEV_SOFT2D_NEON; flag-OFF is byte-identical.
# AArch64 only (scalar fallback elsewhere).
option(LITEV_SOFT2D_NEON "liteV: 4-wide NEON 2D BG/OBJ colour-effect compositor" OFF)
if (LITEV_SOFT2D_NEON)
    add_compile_definitions(LITEV_SOFT2D_NEON=1)
endif()
# NEON branchless reject-scan for SoftRenderer2D::InterleaveSprites (the sprite
# priority resolver, called once per BG priority level = up to 4×/scanline). At a
# given priority most of the 256 pixels carry no matching opaque sprite, so the
# scalar loop is dominated by the (OBJLine[i]&OpaPrioMask)!=attrmask reject path.
# This lever tests the prio match 16 lanes at a time and skips whole 16-pixel
# chunks with zero matches; matching chunks fall through to the identical scalar
# body (palette gather has no AArch64 NEON equivalent). Bit-exact: every drawn
# pixel + DrawPixel RMW is byte-identical (final_top/bot hashes ON==OFF).
# AArch64 only; flag-OFF is byte-identical. Default OFF.
option(LITEV_SOFT2D_OBJNEON "liteV: NEON reject-scan for the 2D sprite interleaver" OFF)
if (LITEV_SOFT2D_OBJNEON)
    add_compile_definitions(LITEV_SOFT2D_OBJNEON=1)
endif()
# 4-wide NEON port of the per-pixel 3D-layer 2D compositor (SoftRenderer2D::
# DrawBG_3D, inlined into DrawScanlineBGMode<0> and called once per scanline in a
# 3D game where BG0 is the 3D layer). The scalar loop is 1 px/iter with two
# per-pixel branches (opaque-alpha test + window test) and a shift-down RMW into
# BGOBJLine. This lever replaces it with a branchless NEON mask/select kernel
# doing 4 px/iter. Profiled as the top scalar non-vectorized 2D loop once
# LITEV_SOFT2D_NEON (ColorComposite) + LITEV_SOFT2D_OBJNEON (sprite reject-scan)
# are ON. Bit-exact (final_top/bot/audio hashes ON==OFF); all changes behind
# #ifdef LITEV_SOFT2D_BG3DNEON; flag-OFF is byte-identical. AArch64 only.
option(LITEV_SOFT2D_BG3DNEON "liteV: 4-wide NEON 2D 3D-layer compositor (DrawBG_3D)" OFF)
if (LITEV_SOFT2D_BG3DNEON)
    add_compile_definitions(LITEV_SOFT2D_BG3DNEON=1)
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

# ---------------------------------------------------------------------------
# Fetch-stall lever B: LITEV_LINKOPT — cut the link-time L1I share (measured
# ~6% of L1I refills: @plt trampolines 3.48%, linker64 1.23%, tlsdesc 1.23%).
#   * -fno-plt (compile, arm64 clang): call external functions straight through
#     the GOT instead of executing a lazy `bl func@plt` stub -> removes the @plt
#     trampoline instructions from the I-stream. Applied at DIRECTORY scope here
#     so every `core` TU (added at add_subdirectory(src) below) picks it up; the
#     final SHARED .so (melonDS-android-frontend, linked in the app's
#     app/CMakeLists.txt) mirrors -fno-plt + the -Wl,-Bsymbolic-functions LINK
#     flag there (link flags on this STATIC `core` are inert -- see that file).
# (Initial-exec TLS on NDS::Current was tried here too and removed: bionic rejects
# the IE model for a dlopen'd library's own TLS.) Link flags cannot change guest
# semantics -> emulation stays bit-exact. Default OFF.
option(LITEV_LINKOPT "Kill the @plt/linker L1I share: -fno-plt (compile) + -Wl,-Bsymbolic-functions (final .so link, in app/CMakeLists.txt)" OFF)
if (LITEV_LINKOPT)
    add_compile_definitions(LITEV_LINKOPT=1)
    add_compile_options(-fno-plt)
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
# Region cache: the dispatcher (and the C++ re-entry path) keep the last 4 executable
# code regions per CPU, so a dynamic branch into another region (ITCM <-> main RAM <->
# BIOS, WRAM7 <-> main RAM) switches FastBlockLookup in emitted code instead of returning
# to C++ for SetupExecutableRegion. Entries are only cached when they equal what
# SetupExecutableRegion would return for every address in the window, and are cleared on
# every mapping change (ITCM/DTCM, WRAMCNT, savestate, reset) -> guest-identical.
# Budget register (probe): the slice budget lives in W15 across block hops instead of a
# per-hop Timestamp + CyclesBudget read-modify-write. Timestamp is materialized from it
# (JitTsBase - W15) before every C++ helper call and at every exit to C++, so every C++
# reader still sees the exact block-start time. Exact (category A).
option(LITEV_JIT_BUDGET_REG "Keep the slice budget in a register across JIT hops (exact)" OFF)
if (LITEV_JIT_BUDGET_REG)
    if (LITEV_JIT_DISPATCH AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_BUDGET_REG=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_BUDGET_REG requires LITEV_JIT_DISPATCH and an AArch64 target.")
    endif()
endif()

option(LITEV_JIT_REGION_CACHE "Dispatcher-side multi-region block lookup (exact)" OFF)
if (LITEV_JIT_REGION_CACHE)
    if (LITEV_JIT_DISPATCH AND CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64|ARM64")
        add_compile_definitions(LITEV_JIT_REGION_CACHE=1)
    else()
        message(FATAL_ERROR "LITEV_JIT_REGION_CACHE requires LITEV_JIT_DISPATCH and an AArch64 target.")
    endif()
endif()

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

# Scheduler drain: run consecutive DMA9 / GXFIFO-stall steps in one scheduler
# iteration while ARM7 is halted (the skipped iteration tails are no-ops; see
# NDS::SchedDrainContinue). Guest- and cycle-identical. Needs LITEV_EVENT_SLICES.
option(LITEV_SCHED_DRAIN "Drain DMA9/GXFIFO-stall steps in one scheduler iteration while ARM7 is halted (exact)" OFF)
if (LITEV_SCHED_DRAIN)
    add_compile_definitions(LITEV_SCHED_DRAIN=1)
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

# Diagnostic only (not a lever): tally each ARM9 slow-helper call by ClassifyAddress9 region and dump
# to stderr at exit (SLOWHIST lines). Tells us whether residual slow-mem cost is fastmem-recoverable
# (MainRAM/DTCM mispredicts) or genuinely unmapped (IO/VRAM/Palette/OAM). Never ship ON.
option(LITEV_SLOWMEM_HIST "Diagnostic: histogram slow-helper calls by memory region" OFF)
if (LITEV_SLOWMEM_HIST)
    add_compile_definitions(LITEV_SLOWMEM_HIST=1)
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

option(LITEV_GXFIFO_UNIFIED "GX command PIPE and FIFO in one ring: FIFO->PIPE moves become counter updates (exact)" OFF)
if (LITEV_GXFIFO_UNIFIED)
    add_compile_definitions(LITEV_GXFIFO_UNIFIED=1)
endif()
option(LITEV_JIT_CYCLE_BATCH "Batch the per-instruction cycle adds of unconditional non-branch instructions into one add before the next cycle read (AArch64, exact)" OFF)
if (LITEV_JIT_CYCLE_BATCH)
    add_compile_definitions(LITEV_JIT_CYCLE_BATCH=1)
endif()
option(LITEV_JIT_LDM_FASTMEM "Fault-backed fastmem path for LDM/POP block loads, not only stores (AArch64)" OFF)
if (LITEV_JIT_LDM_FASTMEM)
    add_compile_definitions(LITEV_JIT_LDM_FASTMEM=1)
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

# Milestone 4 (renderer track, software-renderer scope). All default OFF.
option(LITEV_NEON_RENDERER "Enable NEON SIMD 2D output paths (AArch64 only)" OFF)
option(LITEV_SPU_FAST_INTERP "Replace SPU channel interpolation with cheap linear interp" OFF)
option(LITEV_AGGRESSIVE_SKIP "Enable per-frame 2D/3D rasterization skip gate" OFF)

# Adaptive frameskip that holds REAL-TIME pace on scenes that cannot sustain 60 fps. Without
# it the limiter has no headroom to sleep, so the game runs in slow motion; with it the
# frontend raises the rasterisation-skip target (CPU/DMA/timers keep running) just enough to
# keep pace, and lowers it when headroom returns. A host-side UX feature, runtime-gated by a
# user setting and OFF by default; it must stay off for performance measurements. Drives
# GPU::SetFrameskipTarget, so it requires LITEV_AGGRESSIVE_SKIP. add_compile_definitions so
# the LITEV_* re-export loop below carries it PUBLIC to the out-of-tree frontend.
option(LITEV_AUTO_FRAMESKIP "Adaptive frameskip to hold real-time speed (requires LITEV_AGGRESSIVE_SKIP)" OFF)
if (LITEV_AUTO_FRAMESKIP)
    if (NOT LITEV_AGGRESSIVE_SKIP)
        message(FATAL_ERROR "LITEV_AUTO_FRAMESKIP requires LITEV_AGGRESSIVE_SKIP.")
    endif()
    add_compile_definitions(LITEV_AUTO_FRAMESKIP=1)
endif()

# M6.11 step 3: flag-gated integer-NEON of the GPU3D geometry engine's hot
# per-vertex / per-command fixed-point math (SubmitVertex position + texcoord
# transforms, MatrixMult4x4). Default OFF -> byte-identical scalar behaviour.
# The option is host-agnostic: the source guards every NEON block with
# `#if defined(__ARM_NEON)` and falls back to the scalar reference elsewhere, so
# all configs build on both aarch64 (NEON active) and x86_64 (scalar). Set via
# add_compile_definitions so the LITEV_* re-export loop below carries it PUBLIC.
option(LITEV_NEON_GEOMETRY "M6.11: integer-NEON GPU3D geometry math (AArch64 active, scalar fallback elsewhere)" OFF)
if (LITEV_NEON_GEOMETRY)
    add_compile_definitions(LITEV_NEON_GEOMETRY=1)
endif()

# liteV wave-2 integer-NEON GPU3D geometry (the residual scalar-s64 tail left after
# LITEV_NEON_GEOMETRY vectorized the per-vertex clip-matrix transform): per-polygon
# backface-cull cross/dot in SubmitPolygon, MatrixMult4x3/3x3, and the BuildFrameGeom
# vertex pack. Bit-exact by construction (integer NEON, scalar #else fallback,
# flag-OFF byte-identical); AArch64-only (each block also guarded by __ARM_NEON).
option(LITEV_GEOM_NEON2 "liteV wave-2 integer-NEON GPU3D geometry: per-poly cull cross/dot, MatrixMult4x3/3x3, BuildFrameGeom vertex pack (bit-exact, AArch64)" OFF)
if (LITEV_GEOM_NEON2)
    add_compile_definitions(LITEV_GEOM_NEON2=1)
endif()

# Staged deep prefetch for the pointer-chased Vertex structs in BuildFrameGeom (the LANDED
# lever; a superseded shallow 1-poly variant was removed 2026-09-17). The shallow 1-poly prefetch
# only recovered ~-1pp because it landed after the ~180-cyc DRAM miss already stalled the
# in-order A55. PREFETCH2 warms the Polygon struct pfPoly polys ahead (all 4 lines) and the
# Vertex lines pfVtx polys ahead (pfPoly>pfVtx so the poly is warm before we chase Vertices[]),
# deep enough to overlap the latency. Pure prefetch -> bit-exact. Distances swept via props.
option(LITEV_GEOM_PREFETCH2 "Staged deep prefetch for BuildFrameGeom scatter (bit-exact)" OFF)
if (LITEV_GEOM_PREFETCH2)
    add_compile_definitions(LITEV_GEOM_PREFETCH2=1)
endif()

# liteV wave-3 integer-NEON GPU3D geometry: completes NEON coverage of the remaining
# per-command clip/vector-matrix transforms in the GXFIFO command path that waves 1/2
# left scalar — the BoxTest 8-vertex cube transform (0x70), PosTest (0x71), VecTest
# (0x72), and MatrixScale (0x1B) / MatrixTranslate (0x1C). BoxTest/PosTest/Translate
# reuse the SAME proven NeonMat4Vec4_s64<12> kernel as the shipping per-vertex clip
# transform; VecTest mirrors CalculateLighting's shipping 32-bit >>N NEON dot; Scale is
# a widening per-row scale. Bit-exact by construction (integer NEON, scalar #else
# fallback, flag-OFF byte-identical); no AddCycles/timing change so MP-safe. AArch64-only
# (each block also guarded by __ARM_NEON).
option(LITEV_GEOM_NEON3 "liteV wave-3 integer-NEON GPU3D geometry: BoxTest/PosTest/VecTest + MatrixScale/MatrixTranslate command-path transforms (bit-exact, AArch64)" OFF)
if (LITEV_GEOM_NEON3)
    add_compile_definitions(LITEV_GEOM_NEON3=1)
endif()

# Instant ARM9 hardware divider/sqrt: compute the DIV/SQRT result at register-
# write time and skip the scheduled completion event. In-race profiling showed
# Event_Div is the single largest scheduler load (~1233 dispatches/frame, 42% of
# all events) plus Sqrt (~80/frame); eliminating them cuts scheduler/event-
# dispatch overhead and coarsens slicing. The busy bit is never observed set
# (result ready same-cycle) — a deliberate ARM9 timing relaxation (FPS-first).
option(LITEV_INSTANT_DIVSQRT "Instant ARM9 divider/sqrt: compute on write, skip Event_Div/Event_Sqrt" OFF)
if (LITEV_INSTANT_DIVSQRT)
    add_compile_definitions(LITEV_INSTANT_DIVSQRT=1)
endif()

# DraStic batched audio ring (teardown doc 11): SPU::Mix generates N samples per
# scheduled Event_SPU instead of one, cutting the SPU scheduler-event flood
# (547/frame) by ~N x. Sample generation is bit-identical per iteration; only
# scheduler granularity + SPU-IRQ/capture latency coarsen (FPS-first). N is the
# batch factor (default 8 -> 547/frame -> ~68/frame). Default OFF.
option(LITEV_SPU_BATCH "DraStic #11: batch N audio samples per Event_SPU (coarsen SPU event flood)" OFF)
set(LITEV_SPU_BATCH_N "8" CACHE STRING "SPU batch factor (samples generated per scheduled Event_SPU)")
if (LITEV_SPU_BATCH)
    add_compile_definitions(LITEV_SPU_BATCH=1 LITEV_SPU_BATCH_N=${LITEV_SPU_BATCH_N})
endif()

# Integer-NEON vectorization of the SPU 16-channel pan-accumulate in SPU::Mix.
# BIT-EXACT with the scalar path: each PanOutput term is ((s64)in*(128-pan))>>10;
# |in|<=2^26, |term|<=2^23, 16-term sum <=2^27 (<< 2^31) so s64-accumulate-then-
# narrow == the scalar per-term s32 accumulation with no overflow for ANY DS game.
# Emu-core lever (does NOT move work off-core -> unaffected by the 4-core wall).
# Default OFF -> byte-identical scalar behaviour. ARM64-only (guarded by include).
option(LITEV_SPU_MIX_NEON "Vectorize the SPU 16-channel pan-accumulate with integer NEON (bit-exact)" OFF)
if (LITEV_SPU_MIX_NEON)
    add_compile_definitions(LITEV_SPU_MIX_NEON=1)
endif()

# DraStic-style inert-tick RTC coarsening: the 32768Hz RTC oscillator schedules
# ~548 events/frame just to bump a counter. When no periodic INT1 frequency /
# INT2 alarm is armed (the common gameplay case), the periodic ProcessIRQ(1) is a
# no-op, so ClockTimer hops straight to the next 1024-tick boundary in one event
# instead of ~1024 per-tick events -- removing essentially the whole RTC flood.
# Reverts to the exact per-tick path the moment a game arms an RTC IRQ. FPS-first,
# default OFF. (Known: HANGS at --frameskip 99; real gameplay runs frameskip 0.)
option(LITEV_COARSE_RTC "DraStic: skip inert 32768Hz RTC ticks when no periodic RTC IRQ is armed" OFF)
if (LITEV_COARSE_RTC)
    add_compile_definitions(LITEV_COARSE_RTC=1)
endif()

# DraStic-style DMA streamlining for the GXFIFO geometry-DMA path (the dominant
# in-race DMA: MainRAM display-list -> 0x04000400, fixed dst, incrementing src).
# The stock loop routes every word through NDS::ARM9Read32 (region decode) and
# NDS::ARM9Write32 -> ARM9IOWrite32 -> GPU3D::Write32 (I/O address dispatch) before
# reaching GPU3D::WriteToGXFIFO. Since IsGXFIFODMA already proves src==MainRAM /
# dst==0x04000400 / DstAddrInc==0, this fast path reads the source word directly
# from MainRAM (exactly what ARM9Read32's 0x02 case does) and calls WriteToGXFIFO
# directly (exactly what the 0x400 dispatch resolves to, incl. the GeometryEnabled
# guard). Per-word DMA timing (UnitTimings9_32) and stall handling are unchanged,
# so this is BIT-EXACT dispatch elision, not an accuracy tradeoff. Profiled on the
# Shrek race: DMA::Run9 is ~10.5% of the emu thread and ~60% of that is this
# GXFIFO chain. Default OFF.
option(LITEV_DMA_GXFIFO_FAST "DraStic: direct MainRAM->WriteToGXFIFO fast path for geometry DMA (dispatch elision)" OFF)
if (LITEV_DMA_GXFIFO_FAST)
    add_compile_definitions(LITEV_DMA_GXFIFO_FAST=1)
endif()

option(LITEV_GXFIFO_DMA_INLINE "liteV: inline the GXFIFO producer path (WriteToGXFIFO+CmdFIFOWrite) into DMA::Run9's per-word geometry-DMA loop; byte/cycle-exact, no per-word cross-TU call (needs LTO otherwise off for .dev). Runtime toggle debug.litev.gxinline (default ON)" OFF)
if (LITEV_GXFIFO_DMA_INLINE)
    add_compile_definitions(LITEV_GXFIFO_DMA_INLINE=1)
endif()

# Lazy-load the ARM9 32-bit DMA-unit memory timings (DMA::UnitTimings9_32, ~1.5% of the emu
# thread): defer the 4 ARM9MemTimings[][6/7] loads into the branch that reads them so the hot
# geometry-DMA burst path (which uses none on a non-refresh unit) skips them. BIT-EXACT (same
# timing value; ARM9MemTimings is invariant within one call), MP-safe. Default OFF.
option(LITEV_DMA_TIMING_LAZY "Lazy-load ARM9 32-bit DMA unit-timing region lookups (bit-exact)" OFF)
if (LITEV_DMA_TIMING_LAZY)
    add_compile_definitions(LITEV_DMA_TIMING_LAZY=1)
endif()

# DraStic thread-affinity (teardown 07): pin the software render threads (async 2D, 3D
# render, tile workers) to cores {0,1,2}, off the emu's core (3, pinned by the app glue). Without this the lib-created render threads land on core 3
# and preempt the critical emu thread (~13ms/frame descheduling: app runFrame 33.5ms vs
# 20.4ms headless for identical code). Android-only (Renderer thread cores).
option(LITEV_PIN_RENDER "Pin software render threads to cores {0,1,2}, off the emu core" OFF)
if (LITEV_PIN_RENDER)
    add_compile_definitions(LITEV_PIN_RENDER=1)
endif()

# DraStic-style aggressive idle-loop recognition (teardown doc 01 item 6). Stock
# melonDS's IsIdleLoop (ARMJIT.cpp) rejects any loop with a cross-iteration
# register recurrence (a register read before it is rewritten in the iteration --
# e.g. a spin/timeout counter or a poll that also bumps a tick). DraStic
# fast-forwards such loops anyway: the recurrent register is a dead
# timeout/accumulator and the observable wait outcome is driven by the memory/IO
# the loop polls, not by that register's exact value. With this flag IsIdleLoop
# accepts the loop as long as it does NO store and NO coprocessor access (both
# still hard-rejected), so it only reads mem/IO, computes in regs, and branches
# back. Fast-forwards the CPU to the next scheduled event on each such taken
# back-branch (same mechanism as stock idle loops). Accuracy tradeoff (FPS-first)
# -- a genuine register-recurrent compute loop that also polls would advance one
# iteration per event; verify no desync. Default OFF.
option(LITEV_IDLE_AGGRESSIVE "DraStic: recognize register-recurrent poll loops as idle (fast-forward)" OFF)
if (LITEV_IDLE_AGGRESSIVE)
    add_compile_definitions(LITEV_IDLE_AGGRESSIVE=1)
endif()

# DraStic-style "baked" timer deadline (attacks the run_system bucket). Under
# LITEV_EVENT_SLICES, NextTarget() recomputes NextTimerDeadline() on every one of
# the ~1957 scheduler iterations/frame -- an 8-timer scan whose result is provably
# INVARIANT between timer overflows/reloads and control writes (RunTimer advances
# Counter by cycles<<shift while TimerTimestamp advances by cycles, leaving the
# absolute soonest-overflow SysTimestamp unchanged). This caches the deadline and
# recomputes only when a timer actually overflows/reloads (HandleTimerOverflow) or
# its control/prescaler/start state is written (TimerStart); savestate-load and
# reset invalidate it. EXACT (no accuracy tradeoff) -- pure recompute elision.
# Default OFF.
option(LITEV_TIMER_FAST "DraStic: cache NextTimerDeadline, recompute only on timer overflow/control write" OFF)
if (LITEV_TIMER_FAST)
    add_compile_definitions(LITEV_TIMER_FAST=1)
endif()

# --- DraStic backlog #3: batched GXFIFO threaded-code interpreter ------------------------
# Hoists GPU3D::Run()'s per-command drain loop INTO GPU3D::ExecuteCommand() as a threaded
# internal loop: each command re-dispatches through the computed-goto jump table and falls
# through to the next command with NO bl/ret + NO function
# prologue/epilogue per command (DraStic docs/drastic-teardown/04-gpu3d-geometry.md §3).
# CmdFIFORead() (and its CheckFIFODMA/CheckFIFOIRQ side effects) still runs once per command
# in the identical order, so geometry OUTPUT and audio are byte-identical to flag-OFF.
# Applied on `core` in src/CMakeLists.txt. Default OFF.
option(LITEV_GXFIFO_THREADED "DraStic #3: batched GXFIFO threaded-code interpreter (no call/ret per GX command)" OFF)

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
