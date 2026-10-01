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
