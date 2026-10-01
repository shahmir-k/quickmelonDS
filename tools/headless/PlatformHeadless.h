/*
    liteDS-v2 headless harness - Platform backend helpers.
*/

#pragma once

#include <string>

namespace HeadlessHost
{
// Sets the directory used to resolve "local" files (firmware/save/wifi settings).
// Created if it does not exist. Default is "./headless-data".
void SetDataDir(const std::string& dir);
const std::string& GetDataDir();

// Per-NDS-instance platform data, passed as the NDS "userdata" pointer and
// handed back to the Platform save/firmware callbacks. Lets two NDS instances
// coexist in one process (verify-interp-converge) without clobbering each
// other's save files. When userdata is null the callbacks fall back to the
// "headless" prefix (single-instance benchmark path).
struct InstanceUserData
{
    // Filename stem for this instance's NDS/GBA saves, e.g. "headless-jit".
    std::string savePrefix = "headless";

    // Local-multiplayer instance id (0..15), routed to MPInterface by the
    // headless MP_* callbacks. Distinct per NDS instance so two instances
    // associate as different wireless players. 0 for the single-instance path.
    int instanceID = 0;
};
}
