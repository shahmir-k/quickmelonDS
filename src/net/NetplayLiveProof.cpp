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

// The live handoff's reversible pre-commit proof (NetplayInput.h). Its own file: NDS.h and the
// SHA-256 code in NetplayInput.cpp define conflicting WORD types.

#include <chrono>
#include <thread>

#include "NetplayInput.h"
#include "MPInterface.h"
#include "../NDS.h"
#include "../xxhash/xxhash.h"

namespace melonDS
{

u64 NetplayLiveProofHash(NDS& nds)
{
    u64 parts[8] = {
        nds.GetSysTimestamp(),
        XXH3_64bits(nds.ARM9.R, sizeof(nds.ARM9.R)),
        XXH3_64bits(nds.ARM7.R, sizeof(nds.ARM7.R)),
        XXH3_64bits(nds.MainRAM, nds.MainRAMMask + 1),
        XXH3_64bits(nds.SharedWRAM, nds.SharedWRAMSize),
        XXH3_64bits(nds.ARM7WRAM, nds.ARM7WRAMSize),
        0, 0
    };
    for (int bank = 0; bank < 9; ++bank)
        parts[6] ^= XXH3_64bits(nds.GPU.VRAM[bank], nds.GPU.VRAMMask[bank] + 1) * (bank + 1);
    parts[7] = nds.ARM9.R[15] | (u64)nds.ARM7.R[15] << 32;
    return XXH3_64bits(parts, sizeof(parts));
}

bool NetplayRunLiveProof(const std::array<NDS*, 2>& consoles, const std::array<u64, 2>& rawClocks,
                         const std::function<void(int)>& runFrame,
                         std::array<u64, 4>& hashes, std::string& error)
{
    hashes = {};
    error.clear();
    if (!consoles[0] || !consoles[1] || !runFrame) { error = "live proof needs both consoles"; return false; }
    bool timedOut = false, runFailed = false;
    {
        // Only the two proof consoles execute against this temporary backend; the installed one
        // (Ambient, with the frozen source) comes back even when a proof frame fails.
        struct Restore
        {
            int Type = MPInterface::GetType();
            std::unique_ptr<MPInterface> Saved = MPInterface::Take();
            ~Restore() { MPInterface::Take().reset(); MPInterface::Set(std::move(Saved), (MPInterfaceType)Type); }
        } restore;
        auto trial = std::make_unique<LockstepMP>();
        LockstepMP* link = trial.get();
        for (int p = 0; p < 2; ++p)
        {
            NDS* nds = consoles[p];
            link->SetClockOrigin(p, rawClocks[p], 1000000000ull);
#ifdef LITEV_MP_FASTPOLL
            link->SetClockSource(p, nds->SysTimestampPtr());
#else
            link->SetClock(p, [nds] { return nds->GetSysTimestamp(); });
#endif
            link->SetWake(p, *nds);
            nds->SetKeyMask(0xFFF); // identical, neutral inputs on every device
            nds->ReleaseScreen();
        }
        MPInterface::Set(std::move(trial), MPInterface_Netplay);
        for (int p = 0; p < 2; ++p)
            if (consoles[p]->Wifi.IsPoweredOn()) link->Begin(p);
        std::atomic<bool> finished{false}, expired{false}, workerFailed{false};
        std::thread worker, watchdog;
        try
        {
            watchdog = std::thread([&] {
                const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(8);
                while (!finished.load(std::memory_order_acquire) && std::chrono::steady_clock::now() < deadline)
                    std::this_thread::sleep_for(std::chrono::milliseconds(5));
                if (!finished.load(std::memory_order_acquire))
                { expired.store(true, std::memory_order_release); link->Stop(); }
            });
            worker = std::thread([&] {
                try
                {
                    for (int f = 0; f < 2; ++f)
                    { runFrame(1); hashes[f * 2 + 1] = NetplayLiveProofHash(*consoles[1]); }
                }
                catch (...) { workerFailed.store(true, std::memory_order_release); link->Stop(); }
            });
            for (int f = 0; f < 2; ++f)
            { runFrame(0); hashes[f * 2] = NetplayLiveProofHash(*consoles[0]); }
        }
        catch (const std::exception& exception)
        {
            runFailed = true;
            error = std::string("cannot execute reversible live proof: ") + exception.what();
            link->Stop();
        }
        if (worker.joinable()) worker.join();
        finished.store(true, std::memory_order_release);
        if (watchdog.joinable()) watchdog.join();
        timedOut = expired.load(std::memory_order_acquire);
        runFailed |= workerFailed.load(std::memory_order_acquire);
    }
    if (timedOut || runFailed)
    {
        if (error.empty()) error = timedOut ? "reversible live proof timed out" : "reversible live proof frame failed";
        return false;
    }
    return true;
}

}
