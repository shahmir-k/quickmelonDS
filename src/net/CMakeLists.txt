include(FixInterfaceIncludes)

add_library(net-utils STATIC
    Net.cpp
    Net_PCap.cpp
    Net_Slirp.cpp
    PacketDispatcher.cpp
    LocalMP.cpp
    LockstepMP.cpp
    Fiber.cpp
    NetplayInput.cpp
    ../teakra/src/makedsp1/sha256.cpp
    LAN.cpp
    Netplay.cpp
    MPInterface.cpp
)

if (LITEV_HOSTED_NETPLAY)
    target_sources(net-utils PRIVATE HostedMP.cpp)
endif()

target_include_directories(net-utils PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}"
    "${CMAKE_CURRENT_SOURCE_DIR}/..")

option(USE_SYSTEM_LIBSLIRP "Use system libslirp instead of the bundled version" OFF)
if (USE_SYSTEM_LIBSLIRP)
    pkg_check_modules(Slirp REQUIRED IMPORTED_TARGET slirp)
    fix_interface_includes(PkgConfig::Slirp)
    target_link_libraries(net-utils PUBLIC PkgConfig::Slirp)
else()
    add_subdirectory(libslirp EXCLUDE_FROM_ALL)
    target_link_libraries(net-utils PUBLIC slirp)
endif()

if (USE_VCPKG)
    find_package(unofficial-enet CONFIG REQUIRED)
    target_link_libraries(net-utils PRIVATE unofficial::enet::enet)
elseif (ANDROID)
    # liteDS-v2-android: the melonDS-android frontend provides enet as a CMake
    # subdirectory target (no pkg-config on Android). Link it directly.
    target_link_libraries(net-utils PUBLIC enet)
else()
    pkg_check_modules(ENet REQUIRED IMPORTED_TARGET libenet)
    fix_interface_includes(PkgConfig::ENet)
    target_link_libraries(net-utils PUBLIC PkgConfig::ENet)
endif()

# The Android app adds net-utils from its own directory scope and net-utils does not link core,
# so it missed every LITEV_* directory definition (shipping LockstepMP ran kHostDelay 0, LAN.cpp
# without LITEV_LAN_RX_SPIN). Take the same definitions core exports to its dependents.
if (TARGET core)
    target_compile_definitions(net-utils PRIVATE $<TARGET_PROPERTY:core,INTERFACE_COMPILE_DEFINITIONS>)
endif()
