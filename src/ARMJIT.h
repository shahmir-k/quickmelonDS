/*
    Copyright 2016-2026 melonDS team

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

#ifndef ARMJIT_H
#define ARMJIT_H

#include <algorithm>
#include <optional>
#include <memory>
#include <deque>
#include <vector>
#include <thread>
#include <condition_variable>
#include "types.h"
#include "MemConstants.h"
#include "Args.h"
#include "ARMJIT_Memory.h"

#ifdef JIT_ENABLED
#include "JitBlock.h"
#ifdef LITEV_JIT_FLATMAPS
#include "FlatBlockMap.h"
#endif

#if defined(__APPLE__) && defined(__aarch64__)
    #include <pthread.h>
#endif

#include "ARMJIT_Compiler.h"

namespace melonDS
{
class ARM;

class JitBlock;
class ARMJIT
{
public:
    ARMJIT(melonDS::NDS& nds, std::optional<JITArgs> jit) noexcept;
    ~ARMJIT() noexcept;
    void InvalidateByAddr(u32) noexcept;
    void CheckAndInvalidateWVRAM(int) noexcept;
    void CheckAndInvalidateITCM() noexcept;
    void Reset() noexcept;
    void JitEnableWrite() noexcept;
    void JitEnableExecute() noexcept;
    void CompileBlock(ARM* cpu) noexcept;
    void ResetBlockCache() noexcept;

    template <u32 num, int region>
    void CheckAndInvalidate(u32 addr) noexcept
    {
        u32 localAddr = Memory.LocaliseAddress(region, num, addr);
        if (CodeMemRegions[region][(localAddr & 0x7FFFFFF) / 512].Code & (1 << ((localAddr & 0x1FF) / 16)))
        {
#ifdef LITEV_JIT_SILENT_STATS
            SilentStats[2]++;
#endif
            InvalidateByAddr(localAddr);
        }
    }
    // Store variant: dst is the host location the caller writes val to right after this call.
    // LITEV_JIT_SILENT_STORE: a store of the value already there leaves the code (and its blocks,
    // page protection and patches) untouched. Inspired by DSperate's silent-store elimination
    // (mem::store_code, src/core/mem/page_table.h, https://github.com/beebono/DSperate, f5fa35e).
    // Not exact here (OFF): the guest bytes are the same, but keeping the blocks changes what gets
    // recompiled later, and this JIT's cycle charges depend on block shape (e.g. a block-final
    // conditional branch charges its not-taken cycles on both edges), so The Sims 3 diverges at f899.
    // Census (LITEV_JIT_SILENT_STATS): 24 silent of 182 code-hit stores in the 19297-frame PW
    // recording, 0 of 8 in the 8P MKDS race; the PW transition invalidations come from A9HLE natives.
    template <u32 num, int region, typename T>
    void CheckAndInvalidate(u32 addr, const T* dst, T val) noexcept
    {
        u32 localAddr = Memory.LocaliseAddress(region, num, addr);
        if (CodeMemRegions[region][(localAddr & 0x7FFFFFF) / 512].Code & (1 << ((localAddr & 0x1FF) / 16)))
        {
            const bool silent = *dst == val;
#ifdef LITEV_JIT_SILENT_STATS
            SilentStats[silent]++;
#endif
#ifdef LITEV_JIT_SILENT_STORE
            if (silent && SilentStoreOn)
                return;
#endif
            InvalidateByAddr(localAddr);
        }
    }
#ifdef LITEV_JIT_SILENT_STORE
    bool SilentStoreOn = true;   // headless A/B in one binary: LITEV_JIT_SILENT_STORE_OFF=1
#endif
#ifdef LITEV_JIT_SILENT_STATS
    u64 SilentStats[3] {};       // code-hit stores: [0] changed the value, [1] same value, [2] value unknown (other callers)
#endif
    JitBlockEntry LookUpBlock(u32 num, u64* entries, u32 offset, u32 addr) noexcept;
    bool SetupExecutableRegion(u32 num, u32 blockAddr, u64*& entry, u32& start, u32& size, int* regionOut = nullptr) noexcept;
#ifdef LITEV_JIT_REGION_CACHE
    bool RegionCacheable(u32 num, int region, u32 start, u32 size) const noexcept;
#endif
    u32 LocaliseCodeAddress(u32 num, u32 addr) const noexcept;

    ARMJIT_Memory Memory;
private:
    int MaxBlockSize {};
    bool LiteralOptimizations = false;
    bool BranchOptimizations = false;
    bool FastMemory = false;

public:
    melonDS::NDS& NDS;
    TinyVector<u32> InvalidLiterals {};
    friend class ARMJIT_Memory;
    void blockSanityCheck(u32 num, u32 blockAddr, JitBlockEntry entry) noexcept;
    void RetireJitBlock(JitBlock* block) noexcept;

    int GetMaxBlockSize() const noexcept { return MaxBlockSize; }
    bool LiteralOptimizationsEnabled() const noexcept { return LiteralOptimizations; }
    bool BranchOptimizationsEnabled() const noexcept { return BranchOptimizations; }
    bool FastMemoryEnabled() const noexcept { return FastMemory; }

    void SetJITArgs(JITArgs args) noexcept;
    void SetMaxBlockSize(int size) noexcept;
    void SetLiteralOptimizations(bool enabled) noexcept;
    void SetBranchOptimizations(bool enabled) noexcept;
    void SetFastMemory(bool enabled) noexcept;

    Compiler JITCompiler;
#ifdef LITEV_JIT_POOL_ALLOC
    template <typename K, typename V> using BlockMap = std::unordered_map<K, V, std::hash<K>, std::equal_to<K>, JitPool::Allocator<std::pair<const K, V>>>;
    template <typename K, typename V> using BlockMultiMap = std::unordered_multimap<K, V, std::hash<K>, std::equal_to<K>, JitPool::Allocator<std::pair<const K, V>>>;
#else
    template <typename K, typename V> using BlockMap = std::unordered_map<K, V>;
    template <typename K, typename V> using BlockMultiMap = std::unordered_multimap<K, V>;
#endif
#ifdef LITEV_JIT_FLATMAPS
    FlatBlockMap<JitBlock*> JitBlocks9 {};
    FlatBlockMap<JitBlock*> JitBlocks7 {};

    FlatBlockMap<JitBlock*> RestoreCandidates {};
#else
    BlockMap<u32, JitBlock*> JitBlocks9 {};
    BlockMap<u32, JitBlock*> JitBlocks7 {};

    BlockMap<u32, JitBlock*> RestoreCandidates {};
#endif

#ifdef LITEV_JIT_LINK
    // liteDS-v2 Unit 4 (direct block linking). Per-CPU pending links keyed by the
    // targetAddr a source site is waiting to be compiled. All mutation happens on
    // the emulator thread while it owns execution -> no locking.
#ifdef LITEV_JIT_FLATMAPS
    struct PendingAlloc
    {
#ifdef LITEV_JIT_POOL_ALLOC
        static void* Alloc(size_t n) { return JitPool::Alloc(n); }
        static void Free(void* p, size_t n) { JitPool::Free(p, n); }
#else
        static void* Alloc(size_t n) { return ::operator new(n); }
        static void Free(void* p, size_t) { ::operator delete(p); }
#endif
    };
    PendingSiteMap<LinkSite, PendingAlloc> PendingLinks9 {};
    PendingSiteMap<LinkSite, PendingAlloc> PendingLinks7 {};
#else
    BlockMultiMap<u32, LinkSite> PendingLinks9 {};
    BlockMultiMap<u32, LinkSite> PendingLinks7 {};
#endif

    // Resolve a freshly-compiled-or-restored block's outgoing links + drain any
    // pending links waiting on its StartAddr. Block must already be in JitBlocks +
    // FastBlockLookup.
    void LinkBlock(JitBlock* block) noexcept;
    // A dying block: rewrite its incoming sites back to the dispatcher (and re-pend
    // them), and purge its outgoing sites from targets' Incoming / from pending.
    // Must run before the block leaves JitBlocks / RetireJitBlock.
    void UnlinkBlock(JitBlock* block) noexcept;
    void LinkBlockOutgoing(JitBlock* block) noexcept;
  #if defined(LITEV_SHADOW_ASSERT)
    // Walk every live block's incoming/outgoing sites and assert each currently
    // holds a `B` to either the dispatcher or a live block entry. Shadow-only.
    void ValidateLinkSites() noexcept;
  #endif
#endif


#ifdef LITEV_JIT_BGEMIT
    // Background emission (see JitEmitJob). Jobs live for one block-cache epoch, by id.
    int BgMode = 0;
    std::vector<std::unique_ptr<JitEmitJob>> JobStore;   // every job ever allocated (reused)
    std::vector<JitEmitJob*> FreeJobs;
    u32 NextJobId = 0;                        // = the job's entry stub index
    // The queue: Ring[Head..Tail) waits for emission (Ring[k] is the k-th job queued since the
    // reset, = Jobs[k]). The emulator thread alone moves Tail, the emitter (EmitLock holder) Head.
    static constexpr u32 RingSize = 2 * 1024 * 1024 / 16;   // = the entry stub arena's capacity
    std::unique_ptr<JitEmitJob*[]> Ring;
    std::atomic<u32> Head {0}, Tail {0};
    u32 Finalized = 0;                        // jobs [0, Finalized) are linked in (emulator thread)
    u64 EnqBound = 0, EnqFarBound = 0;        // sum of the code bounds queued since the reset
    std::atomic<s64> PublishedUsed {0}, PublishedFar {0};   // the stream's end after job Head-1
    std::thread Worker;
    std::mutex SleepLock;
    std::condition_variable SleepCv, DoneCv;
    std::atomic<bool> EmuWaiting {false};
    std::atomic<bool> WorkerStop {false}, WorkerSleeping {false};
    bool EmitFront(bool onWorker);
    u64 BgBoundMisses = 0, StatResolves = 0, StatWaits = 0, StatSteals = 0;
    void EmitJob(JitEmitJob& j, bool onWorker);   // EmitLock held
    void WaitJob(JitEmitJob& j);
    void DrainEmits();
    void FinalizeJob(JitEmitJob& j);
    void ProcessDone();
    void CheckCodeSpace(int count);
    void WorkerLoop();
    u32 EntryOff(JitBlock* block) noexcept;
public:
    JitBlockEntry PendingResolve(u32 id) noexcept;
#endif

    AddressRange CodeIndexITCM[ITCMPhysicalSize / 512] {};
    AddressRange CodeIndexMainRAM[MainRAMMaxSize / 512] {};
    AddressRange CodeIndexSWRAM[SharedWRAMSize / 512] {};
    AddressRange CodeIndexVRAM[0x100000 / 512] {};
    AddressRange CodeIndexARM9BIOS[ARM9BIOSSize / 512] {};
    AddressRange CodeIndexARM7BIOS[ARM7BIOSSize / 512] {};
    AddressRange CodeIndexARM7WRAM[ARM7WRAMSize / 512] {};
    AddressRange CodeIndexARM7WVRAM[0x40000 / 512] {};
    AddressRange CodeIndexBIOS9DSi[0x10000 / 512] {};
    AddressRange CodeIndexBIOS7DSi[0x10000 / 512] {};
    AddressRange CodeIndexNWRAM_A[NWRAMSize / 512] {};
    AddressRange CodeIndexNWRAM_B[NWRAMSize / 512] {};
    AddressRange CodeIndexNWRAM_C[NWRAMSize / 512] {};

    u64 FastBlockLookupITCM[ITCMPhysicalSize / 2] {};
    u64 FastBlockLookupMainRAM[MainRAMMaxSize / 2] {};
    u64 FastBlockLookupSWRAM[SharedWRAMSize / 2] {};
    u64 FastBlockLookupVRAM[0x100000 / 2] {};
    u64 FastBlockLookupARM9BIOS[ARM9BIOSSize / 2] {};
    u64 FastBlockLookupARM7BIOS[ARM7BIOSSize / 2] {};
    u64 FastBlockLookupARM7WRAM[ARM7WRAMSize / 2] {};
    u64 FastBlockLookupARM7WVRAM[0x40000 / 2] {};
    u64 FastBlockLookupBIOS9DSi[0x10000 / 2] {};
    u64 FastBlockLookupBIOS7DSi[0x10000 / 2] {};
    u64 FastBlockLookupNWRAM_A[NWRAMSize / 2] {};
    u64 FastBlockLookupNWRAM_B[NWRAMSize / 2] {};
    u64 FastBlockLookupNWRAM_C[NWRAMSize / 2] {};

    AddressRange* const CodeMemRegions[ARMJIT_Memory::memregions_Count] =
    {
        NULL,
        CodeIndexITCM,
        NULL,
        CodeIndexARM9BIOS,
        CodeIndexMainRAM,
        CodeIndexSWRAM,
        NULL,
        CodeIndexVRAM,
        CodeIndexARM7BIOS,
        CodeIndexARM7WRAM,
        NULL,
        NULL,
        CodeIndexARM7WVRAM,
        CodeIndexBIOS9DSi,
        CodeIndexBIOS7DSi,
        CodeIndexNWRAM_A,
        CodeIndexNWRAM_B,
        CodeIndexNWRAM_C
    };

    u64* const FastBlockLookupRegions[ARMJIT_Memory::memregions_Count] =
    {
        NULL,
        FastBlockLookupITCM,
        NULL,
        FastBlockLookupARM9BIOS,
        FastBlockLookupMainRAM,
        FastBlockLookupSWRAM,
        NULL,
        FastBlockLookupVRAM,
        FastBlockLookupARM7BIOS,
        FastBlockLookupARM7WRAM,
        NULL,
        NULL,
        FastBlockLookupARM7WVRAM,
        FastBlockLookupBIOS9DSi,
        FastBlockLookupBIOS7DSi,
        FastBlockLookupNWRAM_A,
        FastBlockLookupNWRAM_B,
        FastBlockLookupNWRAM_C
    };
};
}

// Defined in assembly
#ifdef LITEV_JIT_MEMBASE_PIN
// memBase: this CPU's fastmem base, kept in x26 for the whole slice (blocks no longer load it)
extern "C" void ARM_Dispatch(melonDS::ARM* cpu, melonDS::JitBlockEntry entry, void* memBase);
#else
extern "C" void ARM_Dispatch(melonDS::ARM* cpu, melonDS::JitBlockEntry entry);
#endif
#else
namespace melonDS
{
class ARM;

// This version is a stub; the methods all do nothing,
// but there's still a Memory member.
class ARMJIT
{
public:
    ARMJIT(melonDS::NDS& nds, std::optional<JITArgs>) noexcept : Memory(nds) {}
    ~ARMJIT() noexcept {}
    void InvalidateByAddr(u32) noexcept {}
    void CheckAndInvalidateWVRAM(int) noexcept {}
    void CheckAndInvalidateITCM() noexcept {}
    void Reset() noexcept {}
    void JitEnableWrite() noexcept {}
    void JitEnableExecute() noexcept {}
    void CompileBlock(ARM*) noexcept {}
    void ResetBlockCache() noexcept {}
    template <u32, int>
    void CheckAndInvalidate(u32 addr) noexcept {}
    template <u32, int, typename T>
    void CheckAndInvalidate(u32, const T*, T) noexcept {}

    ARMJIT_Memory Memory;
};
}
#endif // JIT_ENABLED

#endif // ARMJIT_H

