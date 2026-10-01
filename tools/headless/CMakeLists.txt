# liteDS-v2 headless benchmark harness (Unit 0).
# Builds the "liteDS-headless" CLI runner, linking the melonDS core library
# ONLY (no Qt, no SDL, no frontend).

add_executable(liteDS-headless
    main.cpp
    PlatformHeadless.cpp
    VerifyTrace.cpp
    InputScript.cpp
    # Local multiplayer: compile LocalMP + a Local-only MPInterface directly into
    # the harness (NOT the net-utils lib, which drags in ENet/Slirp for LAN/Netplay).
    # LocalMP only needs Platform mutex/semaphore from core.
    "${CMAKE_SOURCE_DIR}/src/net/LocalMP.cpp"
    MPLocal.cpp
)

target_link_libraries(liteDS-headless PRIVATE core)

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
