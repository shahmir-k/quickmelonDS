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
