# liteDS-v2 headless benchmark harness (Unit 0).
# Builds the "liteDS-headless" CLI runner, linking the melonDS core library
# ONLY (no Qt, no SDL, no frontend).

# LITEV_HEADLESS_LAN: build the real MPInterface + LAN backend (ENet) instead of the
# local-only stand-in, so one headless instance can join a LAN session hosted elsewhere
# (e.g. the SereneDS app on a device): --lan-join <host>. ENet comes from a source tree
# (default: the copy bundled with the Android app), not a system package.
option(LITEV_HEADLESS_LAN "liteDS-headless: LAN multiplayer client (--lan-join)" OFF)
set(LITEV_ENET_SOURCE_DIR "${CMAKE_SOURCE_DIR}/../melonDS-android-app-clean/app/src/main/cpp/enet"
    CACHE PATH "ENet source tree for LITEV_HEADLESS_LAN")

if (LITEV_HEADLESS_LAN)
    set(LITEV_HEADLESS_MP_SOURCES "${CMAKE_SOURCE_DIR}/src/net/MPInterface.cpp" "${CMAKE_SOURCE_DIR}/src/net/LAN.cpp")
else()
    set(LITEV_HEADLESS_MP_SOURCES MPLocal.cpp)
endif()

add_executable(liteDS-headless
    main.cpp
    PlatformHeadless.cpp
    VerifyTrace.cpp
    InputScript.cpp
    # Local multiplayer: compile LocalMP + a Local-only MPInterface directly into
    # the harness (NOT the net-utils lib, which drags in ENet/Slirp for LAN/Netplay).
    # LocalMP only needs Platform mutex/semaphore from core.
    "${CMAKE_SOURCE_DIR}/src/net/LocalMP.cpp"
    "${CMAKE_SOURCE_DIR}/src/net/LockstepMP.cpp"
    "${CMAKE_SOURCE_DIR}/src/net/NetplayInput.cpp"
    ${LITEV_HEADLESS_MP_SOURCES}
)
if (LITEV_HOSTED_NETPLAY)
    target_sources(liteDS-headless PRIVATE "${CMAKE_SOURCE_DIR}/src/net/HostedMP.cpp")
endif()

target_link_libraries(liteDS-headless PRIVATE core)

if (LITEV_HEADLESS_LAN)
    add_subdirectory("${LITEV_ENET_SOURCE_DIR}" enet-build EXCLUDE_FROM_ALL)
    target_link_libraries(liteDS-headless PRIVATE enet)
    target_include_directories(liteDS-headless PRIVATE "${LITEV_ENET_SOURCE_DIR}/include")
    target_compile_definitions(liteDS-headless PRIVATE LITEV_HEADLESS_LAN=1)
endif()

# core's INTERFACE include dir already points at src/, so "types.h",
# "xxhash/xxhash.h", etc. resolve. Add our own dir + src/net for the MP headers.
target_include_directories(liteDS-headless PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/src/net")

# Profiler compile-out discipline: LITE_PROFILE_* macros vanish unless
# LITEV_PROFILE=1 is set (default OFF, upstream-identical).
if (LITEV_PROFILE)
    target_compile_definitions(liteDS-headless PRIVATE LITEV_PROFILE=1)
endif()

# dlopen/dlsym for the Platform dynamic-library backend.
if (NOT WIN32)
    target_link_libraries(liteDS-headless PRIVATE ${CMAKE_DL_LIBS})
endif()

set_target_properties(liteDS-headless PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
