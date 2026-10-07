/*
    Copyright 2016-2025 melonDS team

    This file is part of melonDS.

    melonDS is free software: you can redistribute it and/or modify it under
    the terms of the GNU General Public License as published by the Free
    Software Foundation, either version 3 of the License, or (at your option)
    any later version.

    melonDS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with melonDS. If not, see http://www.gnu.org/licenses/.
*/

#ifndef LITEVCORES_H
#define LITEVCORES_H

#if defined(__ANDROID__) || defined(__linux__)

#include <sched.h>
#include <unistd.h>
#include <algorithm>
#include <cstdio>
#include <vector>

namespace melonDS
{

// Which CPU cores the emulator thread and everything else use, from the device's own topology.
// The emulator thread gets the fastest core (highest cpuinfo_max_freq; ties go to the highest
// index), everything else may use every other core, fastest first. On the RG DS's four identical
// Cortex-A55s that is core 3 for the emulator and 0-2 for the rest; on a Snapdragon 8 Gen 2
// (AYN Thor) the prime core 7 for the emulator, then the big cores 3-6, then the little 0-2.
struct LitevCores
{
    int Emu = 0;
    std::vector<int> Others; // fastest first
    cpu_set_t EmuSet, OtherSet;

    static const LitevCores& Get()
    {
        static const LitevCores cores;
        return cores;
    }

private:
    LitevCores()
    {
        int n = (int)sysconf(_SC_NPROCESSORS_CONF);
        n = std::clamp(n, 1, (int)CPU_SETSIZE);
        std::vector<long> freq(n, 0);
        for (int i = 0; i < n; i++)
        {
            char path[96];
            snprintf(path, sizeof(path), "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", i);
            if (FILE* f = fopen(path, "r"))
            {
                if (fscanf(f, "%ld", &freq[i]) != 1) freq[i] = 0;
                fclose(f);
            }
        }
        for (int i = 0; i < n; i++)
            if (freq[i] >= freq[Emu]) Emu = i;
        for (int i = 0; i < n; i++)
            if (i != Emu) Others.push_back(i);
        std::stable_sort(Others.begin(), Others.end(), [&](int a, int b) { return freq[a] > freq[b]; });

        CPU_ZERO(&EmuSet);
        CPU_SET(Emu, &EmuSet);
        CPU_ZERO(&OtherSet);
        for (int c : Others) CPU_SET(c, &OtherSet);
        if (Others.empty()) OtherSet = EmuSet;
    }
};

}

#endif

#endif
