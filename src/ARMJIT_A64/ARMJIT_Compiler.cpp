/*
    Copyright 2016-2026 melonDS team, RSDuck

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

#include "ARMJIT_Compiler.h"

#include "../ARMJIT_Internal.h"
#include "../ARMInterpreter.h"
#include "../ARMJIT.h"
#include "../NDS.h"
#include "../ARMJIT_Global.h"
#include "../ARMJIT_x64/ARMJIT_Offsets.h"
#include "../LiteProfile.h"

#ifdef LITEV_BLOCKPROF
// Profiling only: per-compiled-block entry counter + dump at exit (LITEV_BLOCKPROF_OUT).
#include <deque>
#include <cstdio>
#include <mutex>
#include <map>
#include <cstring>
namespace { struct BPEnt { uint32_t addr, last; uint8_t num, thumb, idle; int8_t slot = -1; uint16_t n; uint64_t count; std::vector<uint32_t> instr, iaddr; std::vector<uint8_t> host; };   // (every instruction: a block can be longer than 64)
std::deque<BPEnt>& BP() { static std::deque<BPEnt> d; return d; }
std::mutex BPMu; thread_local BPEnt* BPCur;   // --mp-test compiles on a thread per console
void BPDump() { const char* o = getenv("LITEV_BLOCKPROF_OUT"); FILE* f = fopen(o ? o : "/tmp/blockprof.txt", "w"); if (!f) return;
  for (auto& e : BP()) if (e.count) { fprintf(f, "%d %d %08x %08x %d %d %llu", e.num, e.thumb, e.addr, e.last, e.n, e.idle, (unsigned long long)e.count);
    for (int i = 0; i < e.n; i++) fprintf(f, " %08x:%08x", e.iaddr[i], e.instr[i]);
    fprintf(f, " |"); for (uint8_t b : e.host) fprintf(f, "%02x", b); fprintf(f, "\n"); } fclose(f); }
// measurement only (LITEV_BLOCKTRACE env): every block entry appends {cntvct, emission index} to its console's ring
constexpr uint64_t BTSize = 1ull << 24;   // bytes per console ring (1M records)
struct BTRing { uint8_t* base = nullptr; uint8_t* cur = nullptr; };
BTRing BTR[16]; std::map<const void*, int> BTSlot;
void BTDump() { const char* o = getenv("LITEV_BLOCKTRACE"); FILE* f = fopen(o, "wb"); if (!f) return;
  for (int k = 0; k < 16; k++) if (BTR[k].base) { uint32_t h[2] = { (uint32_t)k, (uint32_t)((BTR[k].cur - BTR[k].base) / 16) }; fwrite(h, 4, 2, f); fwrite(BTR[k].base, 1, BTSize, f); }
  fclose(f);
  std::string bn = std::string(o) + ".blocks"; f = fopen(bn.c_str(), "w"); if (!f) return; size_t i = 0;
  for (auto& e : BP()) { fprintf(f, "%zu %d %d %08x %zu\n", i++, e.slot, e.num, e.addr, e.host.size()); } fclose(f); }
uint8_t** BTCursor(const void* nds) { std::lock_guard<std::mutex> lk(BPMu);
  auto it = BTSlot.find(nds); if (it == BTSlot.end()) { int k = (int)BTSlot.size(); it = BTSlot.emplace(nds, k).first;
    BTR[k].base = (uint8_t*)aligned_alloc(2 * BTSize, 2 * BTSize); memset(BTR[k].base, 0, BTSize); BTR[k].cur = BTR[k].base; if (k == 0) atexit(BTDump); }
  return &BTR[it->second].cur; } }
#endif

#include <stdlib.h>
#ifdef LITEV_JIT_SHARED_CODE
#include <sys/mman.h>
#include <string>
#include <unordered_map>
#endif
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif
#include <cstddef>

using namespace Arm64Gen;

extern "C" void ARM_Ret();

namespace melonDS
{

#ifdef LITEV_JIT_PERFMAP
// ============================================================================
// LITEV_JIT_PERFMAP — emit a simpleperf / linux-perf "perf-<pid>.map" so the
// opaque anonymous-rwx JIT blob can be split per emitted region.
//
// PURELY OBSERVATIONAL: every function here only reads the emitter's already-
// computed RX pointers (GetRXPtr()) and writes a text file. It emits NO host
// instructions and never advances m_code, so flag-ON JIT codegen is byte-for-
// byte identical to flag-OFF -> guest emulation stays bit-exact / MP-timing-
// exact. RUNTIME-GATED: nothing is written unless an output directory is named
// at run time, so the flag can ship ON in a .dev build at zero file cost.
//
// Map line format (hex, absolute runtime addresses):  <addr> <size> <name>
// Output path resolution (first match wins; else the feature stays dormant):
//   1. env  LITEV_PERFMAP_DIR=<dir>              -> <dir>/perf-<pid>.map   (headless)
//   2. Android prop debug.litev.perfmap=/abs/dir -> /abs/dir/perf-<pid>.map
//   3. Android prop debug.litev.perfmap=1|true   -> /data/data/<pkg>/perf-<pid>.map
//      (the app's own writable data dir, read from /proc/self/cmdline; matches
//       simpleperf's app convention -> found automatically, NO root needed)
//
// ResetBlockCache wholesale-frees every block, so the file is TRUNCATED and the
// process-lifetime "static" stub regions (recorded once at Compiler construction)
// are rewritten at each epoch, after which the regenerated dispatchers/commit
// stubs and freshly compiled blocks append to it.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>
#include <string>
#include <mutex>
#include <vector>
#include <unistd.h>
#ifdef __ANDROID__
#include <sys/system_properties.h>
#endif

namespace litev_perfmap
{
    struct Region { unsigned long long addr; unsigned long long size; std::string name; };

    static std::mutex         g_lock;    // several consoles (Netplay) compile concurrently
    static std::string        g_path;
    static FILE*              g_file  = nullptr;
    static int                g_state = 0;   // 0=unresolved, 1=armed, -1=disabled
    static std::vector<Region> g_static;     // process-lifetime regions, re-emitted each epoch
    static std::vector<Region> g_dynamic;    // per-block regions, ACCUMULATED across epochs
                                             // so a cache reset (BeginEpoch) does not wipe the
                                             // live blocks a concurrent profile is sampling.
                                             // Reused host addrs may collide (minor mis-attrib
                                             // noise) but the JIT blob becomes resolvable.

    // Resolve g_path once. Returns true iff an output location was named (armed).
    static bool ResolvePath()
    {
        const char* dir = getenv("LITEV_PERFMAP_DIR");
        std::string dirbuf;
#ifdef __ANDROID__
        char prop[PROP_VALUE_MAX] = {0};
        if ((!dir || !dir[0]) && __system_property_get("debug.litev.perfmap", prop) > 0 && prop[0] && strcmp(prop, "0") != 0)
        {
            if (prop[0] == '/') { dirbuf = prop; dir = dirbuf.c_str(); }
            else
            {
                // truthy-but-not-a-path -> derive the app's own writable data dir
                // from the process name (/proc/self/cmdline == package name).
                FILE* cf = fopen("/proc/self/cmdline", "rb");
                if (cf)
                {
                    char nm[256] = {0};
                    size_t n = fread(nm, 1, sizeof(nm) - 1, cf);
                    fclose(cf);
                    for (size_t i = 0; i < n; i++) { if (nm[i] == 0 || nm[i] == ':') { nm[i] = 0; break; } }
                    if (nm[0]) { dirbuf = std::string("/data/data/") + nm; dir = dirbuf.c_str(); }
                }
            }
        }
#endif
        if (!dir || !dir[0]) return false;   // not armed -> stay dormant

        char path[600];
        snprintf(path, sizeof(path), "%s/perf-%d.map", dir, (int)getpid());
        g_path = path;
        return true;
    }

    // Record a process-lifetime region (the Compiler-construction stubs). Buffered
    // only; flushed to the file by BeginEpoch (their RX addresses never change).
    static void AddStatic(const char* name, void* rxstart, void* rxend)
    {
        std::lock_guard<std::mutex> lk(g_lock);
        if ((u8*)rxend <= (u8*)rxstart) return;
        g_static.push_back({ (unsigned long long)(uintptr_t)rxstart,
                             (unsigned long long)((u8*)rxend - (u8*)rxstart),
                             std::string(name) });
    }

    // Start a fresh cache epoch: (re)open the file truncated, rewrite the static
    // regions. Called at the top of Compiler::Reset() (== every ResetBlockCache).
    static void BeginEpoch()
    {
        std::lock_guard<std::mutex> lk(g_lock);
        if (g_state == 0) g_state = ResolvePath() ? 1 : -1;
        if (g_state < 0) return;
        if (g_file) fclose(g_file);
        g_file = fopen(g_path.c_str(), "w");
        if (!g_file)
        {
            fprintf(stderr, "[litev-perfmap] failed to open %s\n", g_path.c_str());
            g_state = -1;
            return;
        }
        fprintf(stderr, "[litev-perfmap] writing %s (%zu static + %zu dynamic regions)\n",
                g_path.c_str(), g_static.size(), g_dynamic.size());
        for (const Region& r : g_static)
            fprintf(g_file, "%llx %llx %s\n", r.addr, r.size, r.name.c_str());
        // Re-emit every per-block region compiled so far so a mid-race cache reset
        // does not blank the map out from under a running profile.
        for (const Region& r : g_dynamic)
            fprintf(g_file, "%llx %llx %s\n", r.addr, r.size, r.name.c_str());
        fflush(g_file);
    }

    // Append one epoch region (dispatchers, commit stubs, compiled blocks).
    static void Add(const char* name, void* rxstart, void* rxend)
    {
        // Only accumulate/emit when ARMED (prop set) — otherwise a normal play
        // session would grow g_dynamic unbounded per block-compile. Unarmed
        // (g_state != 1) this is a cheap early-out.
        if (g_state != 1) return;
        std::lock_guard<std::mutex> lk(g_lock);
        if (!g_file) return;
        if ((u8*)rxend <= (u8*)rxstart) return;
        g_dynamic.push_back({ (unsigned long long)(uintptr_t)rxstart,
                              (unsigned long long)((u8*)rxend - (u8*)rxstart),
                              std::string(name) });
        fprintf(g_file, "%llx %llx %s\n",
                (unsigned long long)(uintptr_t)rxstart,
                (unsigned long long)((u8*)rxend - (u8*)rxstart), name);
        fflush(g_file);
    }
}
void LitevPerfmapAdd(const char* name, void* a, void* b) { litev_perfmap::Add(name, a, b); }   // (in namespace melonDS)
#endif  // LITEV_JIT_PERFMAP

// liteDS-v2 Unit 2: prove the hand-maintained ARMJIT_Offsets.h values against the
// real ARM struct layout. These offsets are baked as immediates in the A64
// linkage/dispatch code, so a silent layout shift (e.g. a field inserted before
// CPSR) must fail the build, not the emulation. The layout is host-arch-
// independent, but the assert is kept under the A64 guard to mirror where the
// offsets are consumed.
#ifdef __aarch64__
#ifdef LITEV_JIT_RAS
static_assert(offsetof(ARM, RasRing) + sizeof(ARM::RasRing) < 16384, "RAS fields need a scaled 12-bit LDR/STR offset");
#endif
static_assert(offsetof(ARM, Cycles) == ARM_Cycles_offset,
    "ARM_Cycles_offset out of sync with ARM::Cycles");
static_assert(offsetof(ARM, StopExecution) == ARM_StopExecution_offset,
    "ARM_StopExecution_offset out of sync with ARM::StopExecution");
static_assert(offsetof(ARM, CPSR) == ARM_CPSR_offset,
    "ARM_CPSR_offset out of sync with ARM::CPSR");
static_assert(offsetof(ARM, CyclesBudget) == ARM_CyclesBudget_offset,
    "ARM_CyclesBudget_offset out of sync with ARM::CyclesBudget");
#ifdef LITEV_JIT_LAZYFLAGS
static_assert(offsetof(ARM, JitNZCV) == ARM_JitNZCV_offset,
    "ARM_JitNZCV_offset out of sync with ARM::JitNZCV (V2 NZCV mirror slot)");
#endif
// Unit 3 dispatcher lookup fields (see ARMJIT_Offsets.h). Proven here so a layout
// shift fails the build rather than corrupting the emitted inline block lookup.
static_assert(offsetof(ARM, FastBlockLookupStart) == ARM_FastBlockLookupStart_offset,
    "ARM_FastBlockLookupStart_offset out of sync with ARM::FastBlockLookupStart");
static_assert(offsetof(ARM, FastBlockLookupSize) == ARM_FastBlockLookupSize_offset,
    "ARM_FastBlockLookupSize_offset out of sync with ARM::FastBlockLookupSize");
static_assert(offsetof(ARM, FastBlockLookup) == ARM_FastBlockLookup_offset,
    "ARM_FastBlockLookup_offset out of sync with ARM::FastBlockLookup");
#ifdef LITEV_JIT_GLOBALREG
// GLOBALREG: the ARM_Dispatch/ARM_Ret linkage loads/spills pinned guest regs at
// ARM_R_offset + reg*4; prove that base against the real ARM::R[] layout.
static_assert(offsetof(ARM, R) == ARM_R_offset,
    "ARM_R_offset out of sync with ARM::R");
#endif
#endif

#ifdef LITEV_JIT_GLOBALREG
// GLOBALREG global fixed register map. STEP 1 pins the single hottest never-banked
// guest reg (r0) to a callee-saved host reg (W19). Only r0..r7 are eligible: they
// are never mode-banked, so a mode/exception switch (which reorders R_FIQ/R_SVC/...
// via the register file) never relocates them and needs no sync. The host regs are
// the leading entries of NativeRegAllocOrder (callee-saved => preserved for free
// across the emitted dispatcher, linked chains, and any C helper BL). Widening =
// append pairs here AND the matching `ldr/str wNN` in ARMJIT_Linkage.S.
static const struct { int GuestReg; Arm64Gen::ARM64Reg HostReg; } GlobalRegPins[] =
{
    // STEP 2: the full free callee-saved pool. AArch64 reserves x26/x27/x28/x29
    // (RMemBase/RCPSR/RCycles/RCPU), so W19..W25 (7 regs) are the architectural
    // maximum for a global pin here — a full DraStic r0..r14 (15-reg) pin is not
    // representable on this host without evicting the fastmem/CPSR/cycle regs.
    // Guest r0..r6 (never mode-banked) map to the leading NativeRegAllocOrder
    // entries the dynamic cache used to hand out.
    { 0, Arm64Gen::W19 },
    { 1, Arm64Gen::W20 },
    { 2, Arm64Gen::W21 },
    { 3, Arm64Gen::W22 },
    { 4, Arm64Gen::W23 },
    { 5, Arm64Gen::W24 },
    { 6, Arm64Gen::W25 },
#if defined(LITEV_JIT_LAZYFLAGS)
    // FULL LAZY-FLAGS frees W27 (formerly RCPSR, the whole-CPSR carrier) as the 8th
    // global pin: guest r7 -> W27. Only when SWTABLE is OFF (else r7->W26 above and W27
    // is left as scratch-pool relief). ARM_Dispatch/Ret load/spill w27 in lockstep.
    { 7, Arm64Gen::W27 },
#endif
};
static constexpr int NumGlobalRegPins = sizeof(GlobalRegPins) / sizeof(GlobalRegPins[0]);
// r0..r5 are always pinned; r6 unless it is repurposed as the down-counter's Ran
// accumulator; r7 when the sw-table (W26) or full lazy-flags (W27) frees a host reg.
// Byte-identical to the previous #if-ladder for every pre-LAZYFLAGS combo.
static constexpr u16 GlobalRegPinnedMask =
    0x003F
    | 0x0040   // r6
#if defined(LITEV_JIT_LAZYFLAGS)
    | 0x0080   // r7 (W26 via sw-table, or W27 freed by lazy-flags)
#endif
    ;
#endif

/*

    Recompiling classic ARM to ARMv8 code is at the same time
    easier and trickier than compiling to a less related architecture
    like x64. At one hand you can translate a lot of instructions directly.
    But at the same time, there are a ton of exceptions, like for
    example ADD and SUB can't have a RORed second operand on ARMv8.
 
    While writing a JIT when an instruction is recompiled into multiple ones
    not to write back until you've read all the other operands!
*/

#ifdef LITEV_JIT_SHARED_CODE
// LITEV_JIT_SHARED_CODE: the process's shared code arena. One set of static stubs and dispatchers
// (generated by the first compiler, console-independent: per-console state through RCPU), then
// blocks appended under Lock. A block's code is a function of its captured compile inputs
// (JitEmitJob), so the same inputs reuse the code another console emitted (Objs). Append-only:
// code lives until the last sharing compiler is gone. ponytail: one 120 MB arena, abort when it is
// full; chain more arenas (one reservation, a common base for the 32-bit lookup offsets) if a
// session ever gets there.
//
// Different ROMs / diverged consoles (cross-game Netplay, e.g. Pokemon Black trading with White;
// overlays, self-modifying code, save-driven paths): sharing is keyed by the block's guest bytes,
// never by ROM or address alone. SharedKey holds every instruction word the console itself fetched
// for the block (with its addresses, literal values, compile-time profile and memory map), so a
// console only ever reuses code emitted for exactly the bytes in its own memory; blocks of two
// different games share code only where the bytes are identical (common SDK functions), which is
// the same code either would emit. Each console keeps its own block maps and invalidation (a write
// to its code drops its block, the shared code stays for the others) and its own links: a shared
// block's link exits branch through the console's table (ARM::JitLinks), which only that console's
// LinkBlock / UnlinkBlock write, exactly where the unshared JIT patches and unpatches its link sites.
SharedArena& JitArena() { static SharedArena a; return a; }
namespace
{

constexpr size_t SharedArenaSize = (size_t)120 << 20, SharedFarSize = (size_t)16 << 20;
static_assert(offsetof(ARM, JitLinks) % 8 == 0 && offsetof(ARM, JitIdle2Enable) + 8 <= 32760, "RCPU-relative 64-bit loads");
static_assert(offsetof(ARM, JitICacheTable) % 8 == 0 && offsetof(ARM, JitMainRAM) % 8 == 0 && offsetof(ARM, JitVRAMPtr7) % 8 == 0
              && offsetof(ARM, JitNDS) % 8 == 0 && offsetof(ARM, JitIdle2Enable) % 8 == 0, "RCPU-relative 64-bit loads");

void SharedKey(std::string& k, const JitEmitJob& j)
{
    auto put = [&](const void* p, size_t n) { k.append((const char*)p, n); };
    auto put32 = [&](u32 v) { put(&v, 4); };
    put32(j.Num | (j.Thumb << 8) | (j.HasMem << 9) | ((j.CPSR & 0x1F) << 16));
    put32(j.Count);
    put32(j.Ctx.ITCMSize); put32(j.Ctx.DTCMMask); put32(j.Ctx.DTCMBase);
    put32(j.Ctx.SWRAM9 | (j.Ctx.SWRAM7 << 1)); put32((u32)j.Ctx.ConsoleType); put32(j.Ctx.SCFG_BIOS);
    put(j.Ctx.NWRAMStart, sizeof(j.Ctx.NWRAMStart)); put(j.Ctx.NWRAMEnd, sizeof(j.Ctx.NWRAMEnd));
    for (int i = 0; i < j.Count; i++)
    {
        const FetchedInstr& f = j.Instrs[i];
        put32(f.Instr); put32(f.Addr); put32(f.DataRegion);
        put32(f.BranchFlags | (f.SetFlags << 8) | (f.DataCycles << 16)
#ifdef LITEV_JIT_COND_MEMGUESS
              | (f.DataExecuted << 24)
#endif
              );
        put32(f.CodeCycles);
        put32(f.Info.DstRegs | (f.Info.SrcRegs << 16)); put32(f.Info.NotStrictlyNeeded | (f.Info.Kind << 16));
        put32(f.Info.SpecialKind | (f.Info.ReadFlags << 8) | (f.Info.WriteFlags << 16) | (f.Info.EndBlock << 24));
    }
    const ARM* cpu = j.Cpu;
    for (int i = 0; i < j.NumSnaps; i++)
    {
        const JitEmitJob::Snap& sn = j.Snaps[i];
        put32(sn.Kind | (sn.State << 8)); put32(sn.Addr); put32(sn.A); put32(sn.B); put32(sn.A2);
        put32(sn.P ? (u32)(sn.P - cpu->JitIdle2Enable) + 1 : 0);   // IDLE2: the site's index
    }
    for (int i = 0; i < j.NumA7; i++) { put32(j.A7[i].Row); put(j.A7[i].V, 4); }
}
}

Compiler::PatchMap& Compiler::Patches() { return Shared ? JitArena().Patches : LoadStorePatches; }

u32 Compiler::ICacheAssignSite()
{
    if (Shared)
    {
        SharedArena& a = JitArena();
        return a.NextSite < SharedICacheSites ? a.NextSite++ : 0;
    }
    return (ICacheNextSite < ICacheSites) ? ICacheNextSite++ : 0;
}

void Compiler::SharedSetCpuFields()
{
    NDS.ARM9.JitICacheTable = ICacheTable[0];
    NDS.ARM7.JitICacheTable = ICacheTable[1];
    NDS.ARM9.JitNDS = NDS.ARM7.JitNDS = &NDS;
    NDS.ARM9.JitMainRAM = NDS.ARM7.JitMainRAM = NDS.MainRAM;
    NDS.ARM9.JitVRAMPtr7 = NDS.ARM7.JitVRAMPtr7 = &NDS.GPU.VRAMPtr_ARM7[0];
}

JitBlockEntry Compiler::SharedCompile(JitEmitJob& job, u32& id)
{
    std::string key;
    key.reserve(64 + 48 * job.Count);
    SharedKey(key, job);
    SharedArena& a = JitArena();
    std::lock_guard<std::recursive_mutex> lk(a.Lock);
    auto it = a.Objs.find(key);
    if (it != a.Objs.end())
    {
        a.Hits++;
        NumLinkExits = it->second.NumLinkExits;
        for (int k = 0; k < NumLinkExits; k++)
            LinkExits[k] = it->second.LinkExits[k];
        id = it->second.Id;
#if defined(__aarch64__)
        asm volatile("isb" ::: "memory");   // code another core wrote
#endif
        return AddEntryOffset(it->second.Off);
    }
    a.Misses++;
    if ((ptrdiff_t)JitMemMainSize - a.Near < 256 * 1024 || (ptrdiff_t)(JitMemMainSize + JitMemSecondarySize) - a.Far < 128 * 1024)
    {
        fprintf(stderr, "LITEV_JIT_SHARED_CODE: shared code arena full (near %td far %td)\n", a.Near, a.Far - (ptrdiff_t)JitMemMainSize);
        abort();
    }
    SetCodePtr(a.Near);
    OtherCodeRegion = a.Far;
    CurSharedId = a.NextId++;
    CurJob = &job;
    NDS.JIT.JitEnableWrite();
    JitBlockEntry entry = CompileBlock(job.Cpu, job.Thumb, job.Instrs, job.Count, job.HasMem);
    NDS.JIT.JitEnableExecute();
    CurJob = nullptr;
    a.Near = GetCodeOffset();
    a.Far = OtherCodeRegion;
    SharedObj& o = a.Objs[std::move(key)];
    o.Off = SubEntryOffset(entry);
    o.Id = id = CurSharedId;
    o.NumLinkExits = NumLinkExits;
    for (int k = 0; k < NumLinkExits; k++)
        o.LinkExits[k] = LinkExits[k];
    CurSharedId = 0;
    static const bool stats = getenv("LITEV_JITSHARE_STATS") != nullptr;
    if (stats && (a.Misses & 4095) == 0)
        fprintf(stderr, "JITSHARE objs %zu hits %llu misses %llu near %.1f MB far %.1f MB sites %u\n", a.Objs.size(),
                (unsigned long long)a.Hits, (unsigned long long)a.Misses, a.Near / 1048576.0,
                (a.Far - (ptrdiff_t)JitMemMainSize) / 1048576.0, a.NextSite);
    return entry;
}
#endif

template <>
const ARM64Reg RegisterCache<Compiler, ARM64Reg>::NativeRegAllocOrder[] =
{
    W19, W20, W21, W22, W23, W24, W25,
#ifdef LITEV_JIT_BUDGET_REG
    W8, W9, W10, W11, W12, W13, W14     // W15 = slice budget (RBudget)
};
template <>
const int RegisterCache<Compiler, ARM64Reg>::NativeRegsAvailable = 14;

const BitSet32 CallerSavedPushRegs({W8, W9, W10, W11, W12, W13, W14});
#else
    W8, W9, W10, W11, W12, W13, W14, W15
};
template <>
const int RegisterCache<Compiler, ARM64Reg>::NativeRegsAvailable = 15;

const BitSet32 CallerSavedPushRegs({W8, W9, W10, W11, W12, W13, W14, W15});
#endif

#ifdef LITEV_JIT_BUDGET_REG
// Every C++ helper call: spill the budget, materialize Timestamp = JitTsBase - budget
// (the exact block-start time the per-hop commit used to leave in memory), call, then
// reload the budget (a helper may have forced an exit = zeroed it). X16/X17 are never
// allocated; X15 is free once spilled. Args in X0-X7 and `scratchreg` are untouched.
void Compiler::QuickCallFunction(ARM64Reg scratchreg, const void* func)
{
    EmitBudgetSpill(X16, X17, X15);
    ARM64XEmitter::QuickCallFunction(scratchreg, func);
    LDR(INDEX_UNSIGNED, RBudget, RCPU, offsetof(ARM, CyclesBudget));
}

// STR budget; Timestamp = JitTsBase - (s64)budget. Clobbers t0, t1, t2 (64-bit regs).
void Compiler::EmitBudgetSpill(ARM64Reg t0, ARM64Reg t1, ARM64Reg t2)
{
    STR(INDEX_UNSIGNED, RBudget, RCPU, offsetof(ARM, CyclesBudget));
    SXTW(t1, RBudget);
    LDR(INDEX_UNSIGNED, t0, RCPU, offsetof(ARM, JitTsBase));
    SUB(t0, t0, t1);
    LDR(INDEX_UNSIGNED, t2, RCPU, offsetof(ARM, JitTsPtr));
    STR(INDEX_UNSIGNED, t0, t2, 0);
}
#endif

#ifdef LITEV_EXIT_PROTO_PC
// A linked hop may skip the R[15] store only if nothing can read the stale value before
// the chain leaves to C++ (every path to the dispatcher stores it). C++ reads ARM::R[15]
// mid-slice only in: exception entry and the C++ loop (at exits), ARM7 BIOS read
// protection (NDS::ARM7Read*: R[15] vs 0x4000 / ARM7BIOSProt), the DSi ARM9 loader check
// and DSi ARM7 BIOS protection, and log messages. So: DS mode only; ARM9 always; ARM7
// only between blocks outside the BIOS, where a stale R[15] (the start of some earlier
// block of the same chain, also >= 0x4000) gives the same protection answer.
bool Compiler::ExitPCElidable(u32 target)
{
    if (NDS.ConsoleType != 0)
        return false;
    if (Num == 0)
        return true;
    return BlockStartAddr >= 0x4000 && target >= 0x4000;
}
#endif

void Compiler::MovePC()
{
    ADD(MapReg(15), MapReg(15), Thumb ? 2 : 4);
}

void Compiler::A_Comp_MRS()
{
    Comp_AddCycles_C();

    ARM64Reg rd = MapReg(CurInstr.A_Reg(12));

    if (CurInstr.Instr & (1 << 22))
    {
#ifdef LITEV_JIT_LAZYFLAGS
        LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));   // control-only (mode) read
        ANDI2R(W5, W5, 0x1F);
#else
        ANDI2R(W5, RCPSR, 0x1F);
#endif
        MOVI2R(W3, 0);
        MOVI2R(W1, 15 - 8);
        BL(ReadBanked);
        MOV(rd, W3);
    }
    else
    {
#ifdef LITEV_JIT_LAZYFLAGS
        // V2 full CPSR read: the guest CPSR word = ARM::CPSR control bits [27:0] (always
        // canonical) merged with the JitNZCV slot's NZCV nibble [31:28]. Flush any host-
        // resident guest flags to the slot first (Comp_MaterializeFlags -> slot; for an
        // unconditional MRS ReconcileFlags already did this via ReadFlags, so it is a no-op
        // then). Then rd = (CPSR & 0x0FFFFFFF) | (slot & 0xF0000000). rd is a mapped guest
        // reg (never W0/W1), so W0/W1 are safe scratch.
        Comp_MaterializeFlags();
        LDR(INDEX_UNSIGNED, rd, RCPU, offsetof(ARM, CPSR));
        LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
        ANDI2R(rd, rd, 0x0FFFFFFF, W1);
        ORR(rd, rd, W0);
#else
        MOV(rd, RCPSR);
#endif
    }
}

#ifdef LITEV_JIT_CP15_INLINE
// ARM9 coprocessor ops the JIT used to hand to the interpreter (spill every register,
// call A_MCR/A_MRC, reload) although they do nothing to emulated state. PW's overworld
// runs ~830 data-cache maintenance MCRs (c7: 761/7A4/7E1) and ~280 DTCM-setting reads
// (MRC c9,c1,0, in the IRQ path) per frame. Same cycles as A_MCR/A_MRC (CodeCycles + 2 / + 3).
// A block compiled in user mode keeps the fallback (there both ops are undefined instructions).
bool Compiler::CP15InlineOK()
{
    if (!Cp15InlineOn || Num != 0 || Thumb || NDS.ConsoleType != 0)
        return false;
#ifdef LITEV_JIT_BGEMIT
    if (((CurJob ? CurJob->CPSR : CurCPU->CPSR) & 0x1F) == 0x10)
#else
    if ((CurCPU->CPSR & 0x1F) == 0x10)
#endif
        return false;
    u32 instr = CurInstr.Instr;
    if (((instr >> 8) & 0xF) != 15)
        return false;
    u32 id = (((instr >> 16) & 0xF) << 8) | ((instr & 0xF) << 4) | ((instr >> 5) & 0x7);
    if (CurInstr.Info.Kind == ARMInstrInfo::ak_MCR)
        // every c7 write but wait-for-interrupt and the icache invalidates is a no-op in CP15Write
        return (id & 0xF00) == 0x700 && id != 0x704 && id != 0x782
            && id != 0x750 && id != 0x751 && id != 0x752;
    // CP15Read(0x910) == DTCMSetting
    return id == 0x910 && ((instr >> 12) & 0xF) != 15;
}

void Compiler::A_Comp_MCR_CacheOp()
{
    Comp_AddCycles_CI(1 + 1);
}

void Compiler::A_Comp_MRC_DTCM()
{
    Comp_AddCycles_CI(2 + 1);
    ARM64Reg rd = MapReg(CurInstr.A_Reg(12));
    LDR(INDEX_UNSIGNED, rd, RCPU, offsetof(ARMv5, DTCMSetting));
}
#endif

void UpdateModeTrampoline(ARM* arm, u32 oldmode, u32 newmode)
{
    arm->UpdateMode(oldmode, newmode);
}

void Compiler::A_Comp_MSR()
{
    Comp_AddCycles_C();

    ARM64Reg val;
    if (CurInstr.Instr & (1 << 25))
    {
        val = W0;
        MOVI2R(val, melonDS::ROR((CurInstr.Instr & 0xFF), ((CurInstr.Instr >> 7) & 0x1E)));
    }
    else
    {
        val = MapReg(CurInstr.A_Reg(0));
    }

    u32 mask = 0;
    if (CurInstr.Instr & (1<<16)) mask |= 0x000000FF;
    if (CurInstr.Instr & (1<<17)) mask |= 0x0000FF00;
    if (CurInstr.Instr & (1<<18)) mask |= 0x00FF0000;
    if (CurInstr.Instr & (1<<19)) mask |= 0xFF000000;

    if (CurInstr.Instr & (1 << 22))
    {
#ifdef LITEV_JIT_LAZYFLAGS
        LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));   // mode (control) read
        ANDI2R(W5, W5, 0x1F);
#else
        ANDI2R(W5, RCPSR, 0x1F);
#endif
        MOVI2R(W3, 0);
        MOVI2R(W1, 15 - 8);
        BL(ReadBanked);

        MOVI2R(W1, mask);
        MOVI2R(W2, mask & 0xFFFFFF00);
#ifdef LITEV_JIT_LAZYFLAGS
        LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, CPSR));   // mode (control) read
        ANDI2R(W5, W5, 0x1F);
#else
        ANDI2R(W5, RCPSR, 0x1F);
#endif
        CMP(W5, 0x10);
        CSEL(W1, W2, W1, CC_EQ);

        BIC(W3, W3, W1);
        AND(W0, val, W1);
        ORR(W3, W3, W0);

        MOVI2R(W1, 15 - 8);

        BL(WriteBanked);
    }
    else
    {
        mask &= 0xFFFFFFDF;
        CPSRDirty = true;

#ifdef LITEV_JIT_LAZYFLAGS
        // MSR can write the flag byte; its InstrInfo WriteFlags==0xF makes ReconcileFlags's
        // full-producer fast path SKIP the flush, yet MSR does NOT set host NZCV. Flush any
        // host-resident guest flags to memory NOW so the following memory RMW is not later
        // clobbered by a stale materialize (and NZCVDeferred is cleared).
        Comp_MaterializeFlags();
        if ((mask & 0xFF) == 0)
        {
            LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
            ANDI2R(W1, W1, ~mask);
            ANDI2R(W0, val, mask);
            ORR(W1, W1, W0);
            STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
        }
        else
        {
            LDR(INDEX_UNSIGNED, W4, RCPU, offsetof(ARM, CPSR));   // W4 = CPSR word
            MOVI2R(W2, mask);
            MOVI2R(W3, mask & 0xFFFFFF00);
            ANDI2R(W1, W4, 0x1F);
            // W1 = first argument (old mode)
            CMP(W1, 0x10);
            CSEL(W2, W3, W2, CC_EQ);

            BIC(W4, W4, W2);
            AND(W0, val, W2);
            ORR(W4, W4, W0);
            STR(INDEX_UNSIGNED, W4, RCPU, offsetof(ARM, CPSR));

            MOV(W2, W4);   // newmode arg
            MOV(X0, RCPU);

            PushRegs(true, true);
            QuickCallFunction(X3, UpdateModeTrampoline);
            PopRegs(true, true);
        }

        // V2: if this MSR wrote the guest flag byte, ARM::CPSR[31:28] now holds the NEW
        // guest NZCV; refresh the JitNZCV slot (the in-slice NZCV home) from it so the
        // resuming JIT observes the new flags. If the flag byte was NOT written, the slot
        // still holds the canonical NZCV (untouched here) and CPSR[31:28] is stale-but-
        // harmless (re-synced at the next escape / the slice merge), so the slot is left
        // alone. NZCVDeferred is 0 (Comp_MaterializeFlags above); W0/W1 are free.
        if (mask & 0xF0000000)
        {
            LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CPSR));
            ANDI2R(W0, W0, 0xF0000000, W1);
            STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
        }
    }
#else
        if ((mask & 0xFF) == 0)
        {
            ANDI2R(RCPSR, RCPSR, ~mask);
            ANDI2R(W0, val, mask);
            ORR(RCPSR, RCPSR, W0);
        }
        else
        {
            MOVI2R(W2, mask);
            MOVI2R(W3, mask & 0xFFFFFF00);
            ANDI2R(W1, RCPSR, 0x1F);
            // W1 = first argument
            CMP(W1, 0x10);
            CSEL(W2, W3, W2, CC_EQ);

            BIC(RCPSR, RCPSR, W2);
            AND(W0, val, W2);
            ORR(RCPSR, RCPSR, W0);

            MOV(W2, RCPSR);
            MOV(X0, RCPU);

            PushRegs(true, true);
            QuickCallFunction(X3, UpdateModeTrampoline);
            PopRegs(true, true);
        }
    }
#endif
}


void Compiler::PushRegs(bool saveHiRegs, bool saveRegsToBeChanged, bool allowUnload)
{
    BitSet32 loadedRegs(RegCache.LoadedRegs);

    if (saveHiRegs)
    {
        BitSet32 hiRegsLoaded(RegCache.LoadedRegs & 0x7F00);
        for (int reg : hiRegsLoaded)
        {
            if (Thumb || CurInstr.Cond() == 0xE)
                RegCache.UnloadRegister(reg);
            else
                SaveReg(reg, RegCache.Mapping[reg]);
            // prevent saving the register twice
            loadedRegs[reg] = false;
        }
    }

    for (int reg : loadedRegs)
    {
        if (CallerSavedPushRegs[RegCache.Mapping[reg]]
            && (saveRegsToBeChanged || !((1<<reg) & CurInstr.Info.DstRegs && !((1<<reg) & CurInstr.Info.SrcRegs))))
        {
            if ((Thumb || CurInstr.Cond() == 0xE) && !((1 << reg) & (CurInstr.Info.DstRegs|CurInstr.Info.SrcRegs)) && allowUnload)
                RegCache.UnloadRegister(reg);
            else
                SaveReg(reg, RegCache.Mapping[reg]);
        }
    }
}

void Compiler::PopRegs(bool saveHiRegs, bool saveRegsToBeChanged)
{
    BitSet32 loadedRegs(RegCache.LoadedRegs);
    for (int reg : loadedRegs)
    {
        if ((saveHiRegs && reg >= 8 && reg < 15)
            || (CallerSavedPushRegs[RegCache.Mapping[reg]]
                && (saveRegsToBeChanged || !((1<<reg) & CurInstr.Info.DstRegs && !((1<<reg) & CurInstr.Info.SrcRegs)))))
        {
            LoadReg(reg, RegCache.Mapping[reg]);
        }
    }
}

Compiler::Compiler(melonDS::NDS& nds) : Arm64Gen::ARM64XEmitter(), NDS(nds)
{
#ifdef __SWITCH__
    JitRWBase = aligned_alloc(0x1000, JitMemSize);

    JitRXStart = (u8*)&__start__ - JitMemSize - 0x1000;
    virtmemLock();
    JitRWStart = virtmemFindAslr(JitMemSize, 0x1000);
    MemoryInfo info = {0};
    u32 pageInfo = {0};
    int i = 0;
    while (JitRXStart != NULL)
    {
        svcQueryMemory(&info, &pageInfo, (u64)JitRXStart);
        if (info.type != MemType_Unmapped)
            JitRXStart = (void*)((u8*)info.addr - JitMemSize - 0x1000);
        else
            break;
        if (i++ > 8)
        {
            Log(LogLevel::Error, "couldn't find unmapped place for jit memory\n");
            JitRXStart = NULL;
        }
    }

    assert(JitRXStart != NULL);

    bool succeded = R_SUCCEEDED(svcMapProcessCodeMemory(envGetOwnProcessHandle(), (u64)JitRXStart, (u64)JitRWBase, JitMemSize));
    assert(succeded);
    succeded = R_SUCCEEDED(svcSetProcessMemoryPermission(envGetOwnProcessHandle(), (u64)JitRXStart, JitMemSize, Perm_Rx));
    assert(succeded);
    succeded = R_SUCCEEDED(svcMapProcessMemory(JitRWStart, envGetOwnProcessHandle(), (u64)JitRXStart, JitMemSize));
    assert(succeded);

    virtmemUnlock();

    SetCodeBase((u8*)JitRWStart, (u8*)JitRXStart);
    JitMemMainSize = JitMemSize;
#else
    ARMJIT_Global::Init();

#ifdef LITEV_JIT_SHARED_CODE
    Shared = ARMJIT_Global::SharedCode();
    if (Shared)
    {
        SharedArena& a = JitArena();
        std::lock_guard<std::recursive_mutex> lk(a.Lock);
        if (a.Users++ > 0)
        {
            // the first sharing compiler built the stubs (console-independent): use them
            CodeMemBase = a.Mem;
            SetCodeBase(a.Base, a.Base);
            JitMemMainSize = a.MainSize;
            JitMemSecondarySize = a.SecondarySize;
            memcpy(JumpToFuncs9, a.JumpTo9, sizeof(JumpToFuncs9));
            memcpy(JumpToFuncs7, a.JumpTo7, sizeof(JumpToFuncs7));
            ReadBanked = a.ReadBanked;
            WriteBanked = a.WriteBanked;
            memcpy(PatchedLoadFuncs, a.PLoad, sizeof(PatchedLoadFuncs));
            memcpy(PatchedStoreFuncs, a.PStore, sizeof(PatchedStoreFuncs));
#ifdef LITEV_JIT_BGEMIT
            BgLayout = false;
#endif
            return;
        }
        a.Mem = CodeMemBase = ARMJIT_Global::AllocateSharedCodeMem(SharedArenaSize);
        if (!a.Mem || a.Mem == (void*)-1) { fprintf(stderr, "LITEV_JIT_SHARED_CODE: no code memory\n"); abort(); }
        nds.JIT.JitEnableWrite();
        SetCodeBase(reinterpret_cast<u8*>(CodeMemBase), reinterpret_cast<u8*>(CodeMemBase));
        JitMemMainSize = SharedArenaSize;
    }
    else
    {
#endif
    CodeMemBase = ARMJIT_Global::AllocateCodeMem();
    nds.JIT.JitEnableWrite();

    SetCodeBase(reinterpret_cast<u8*>(CodeMemBase), reinterpret_cast<u8*>(CodeMemBase));
    JitMemMainSize = ARMJIT_Global::CodeMemorySliceSize;
#ifdef LITEV_JIT_SHARED_CODE
    }
#endif
#endif
    SetCodePtr(0);

    for (int i = 0; i < 3; i++)
    {
        JumpToFuncs9[i] = Gen_JumpTo9(i);
#ifdef LITEV_JIT_PERFMAP
        { char nm[24]; snprintf(nm, sizeof(nm), "jit_jumpto9_%d", i);
          litev_perfmap::AddStatic(nm, JumpToFuncs9[i], GetRXPtr()); }
#endif
        JumpToFuncs7[i] = Gen_JumpTo7(i);
#ifdef LITEV_JIT_PERFMAP
        { char nm[24]; snprintf(nm, sizeof(nm), "jit_jumpto7_%d", i);
          litev_perfmap::AddStatic(nm, JumpToFuncs7[i], GetRXPtr()); }
#endif
    }

    /*
        W4 - whether the register was written to
        W5 - mode
        W1 - reg num
        W3 - in/out value of reg
    */
    {
        ReadBanked = GetRXPtr();

        ADD(X2, RCPU, X1, ArithOption(X2, ST_LSL, 2));
        CMP(W5, 0x11);
        FixupBranch fiq = B(CC_EQ);
        SUBS(W1, W1, 13 - 8);
        ADD(X2, RCPU, X1, ArithOption(X2, ST_LSL, 2));
        FixupBranch notEverything = B(CC_LT);
        CMP(W5, 0x12);
        FixupBranch irq = B(CC_EQ);
        CMP(W5, 0x13);
        FixupBranch svc = B(CC_EQ);
        CMP(W5, 0x17);
        FixupBranch abt = B(CC_EQ);
        CMP(W5, 0x1B);
        FixupBranch und = B(CC_EQ);
        SetJumpTarget(notEverything);
        RET();

        SetJumpTarget(fiq);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_FIQ));
        RET();
        SetJumpTarget(irq);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_IRQ));
        RET();
        SetJumpTarget(svc);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_SVC));
        RET();
        SetJumpTarget(abt);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_ABT));
        RET();
        SetJumpTarget(und);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_UND));
        RET();
    }
#ifdef LITEV_JIT_PERFMAP
    litev_perfmap::AddStatic("jit_readbanked", ReadBanked, GetRXPtr());
#endif
    {
        WriteBanked = GetRXPtr();

        ADD(X2, RCPU, X1, ArithOption(X2, ST_LSL, 2));
        CMP(W5, 0x11);
        FixupBranch fiq = B(CC_EQ);
        SUBS(W1, W1, 13 - 8);
        ADD(X2, RCPU, X1, ArithOption(X2, ST_LSL, 2));
        FixupBranch notEverything = B(CC_LT);
        CMP(W5, 0x12);
        FixupBranch irq = B(CC_EQ);
        CMP(W5, 0x13);
        FixupBranch svc = B(CC_EQ);
        CMP(W5, 0x17);
        FixupBranch abt = B(CC_EQ);
        CMP(W5, 0x1B);
        FixupBranch und = B(CC_EQ);
        SetJumpTarget(notEverything);
        MOVI2R(W4, 0);
        RET();

        SetJumpTarget(fiq);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_FIQ));
        MOVI2R(W4, 1);
        RET();
        SetJumpTarget(irq);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_IRQ));
        MOVI2R(W4, 1);
        RET();
        SetJumpTarget(svc);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_SVC));
        MOVI2R(W4, 1);
        RET();
        SetJumpTarget(abt);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_ABT));
        MOVI2R(W4, 1);
        RET();
        SetJumpTarget(und);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ARM, R_UND));
        MOVI2R(W4, 1);
        RET();
    }
#ifdef LITEV_JIT_PERFMAP
    litev_perfmap::AddStatic("jit_writebanked", WriteBanked, GetRXPtr());
#endif

    for (int consoleType = 0; consoleType < 2; consoleType++)
    {
        for (int num = 0; num < 2; num++)
        {
            for (int size = 0; size < 3; size++)
            {
                for (int reg = 0; reg < 32; reg++)
                {
                    // FULL LAZY-FLAGS pins guest r7 -> W27 (the freed RCPSR reg). A guest
                    // LDR/STR r7 through the fault-based fastmem path indexes these tables by
                    // the mapped host reg, so W27 needs its own patched load/store funcs (the
                    // shipping LAZYFLAGS config has SWTABLE OFF, so fault-fastmem is the memory
                    // mechanism). Missing -> a NULL PatchFunc -> wild BL -> SIGILL at boot.
#ifdef LITEV_JIT_LAZYFLAGS
                    if (!(reg == W4 || (reg >= W8 && reg <= W15) || (reg >= W19 && reg <= W25) || reg == W27))
                        continue;
#else
                    if (!(reg == W4 || (reg >= W8 && reg <= W15) || (reg >= W19 && reg <= W25)))
                        continue;
#endif
                    ARM64Reg rdMapped = (ARM64Reg)reg;
                    PatchedStoreFuncs[consoleType][num][size][reg] = GetRXPtr();
#ifdef LITEV_JIT_ARM7_CPUARG
                    // ARM9 and ARM7 slow helpers share the layout: W0=addr, X1=cpu, W2=val
                    MOV(X1, RCPU);
                    MOV(W2, rdMapped);
#else
                    if (num == 0)
                    {
                        MOV(X1, RCPU);
                        MOV(W2, rdMapped);
                    }
                    else
                    {
                        MOV(W1, rdMapped);
                    }
#endif
                    ABI_PushRegisters(BitSet32({30}) | CallerSavedPushRegs);
                    if (consoleType == 0)
                    {
                        switch ((8 << size) |  num)
                        {
                        case 32: QuickCallFunction(X3, SlowWrite9<u32, 0>); break;
                        case 33: QuickCallFunction(X3, SlowWrite7<u32, 0>); break;
                        case 16: QuickCallFunction(X3, SlowWrite9<u16, 0>); break;
                        case 17: QuickCallFunction(X3, SlowWrite7<u16, 0>); break;
                        case 8: QuickCallFunction(X3, SlowWrite9<u8, 0>); break;
                        case 9: QuickCallFunction(X3, SlowWrite7<u8, 0>); break;
                        }
                    }
                    else
                    {
                        switch ((8 << size) |  num)
                        {
                        case 32: QuickCallFunction(X3, SlowWrite9<u32, 1>); break;
                        case 33: QuickCallFunction(X3, SlowWrite7<u32, 1>); break;
                        case 16: QuickCallFunction(X3, SlowWrite9<u16, 1>); break;
                        case 17: QuickCallFunction(X3, SlowWrite7<u16, 1>); break;
                        case 8: QuickCallFunction(X3, SlowWrite9<u8, 1>); break;
                        case 9: QuickCallFunction(X3, SlowWrite7<u8, 1>); break;
                        }
                    }
                    
                    ABI_PopRegisters(BitSet32({30}) | CallerSavedPushRegs);
                    RET();
#ifdef LITEV_JIT_PERFMAP
                    { char nm[40]; snprintf(nm, sizeof(nm), "jit_pstore_c%dn%ds%dr%d",
                        consoleType, num, size, reg);
                      litev_perfmap::AddStatic(nm, PatchedStoreFuncs[consoleType][num][size][reg], GetRXPtr()); }
#endif

                    for (int signextend = 0; signextend < 2; signextend++)
                    {
                        PatchedLoadFuncs[consoleType][num][size][signextend][reg] = GetRXPtr();
#ifdef LITEV_JIT_ARM7_CPUARG
                        MOV(X1, RCPU);
#else
                        if (num == 0)
                            MOV(X1, RCPU);
#endif
                        ABI_PushRegisters(BitSet32({30}) | CallerSavedPushRegs);
                        if (consoleType == 0)
                        {
                            switch ((8 << size) |  num)
                            {
                            case 32: QuickCallFunction(X3, SlowRead9<u32, 0>); break;
                            case 33: QuickCallFunction(X3, SlowRead7<u32, 0>); break;
                            case 16: QuickCallFunction(X3, SlowRead9<u16, 0>); break;
                            case 17: QuickCallFunction(X3, SlowRead7<u16, 0>); break;
                            case 8: QuickCallFunction(X3, SlowRead9<u8, 0>); break;
                            case 9: QuickCallFunction(X3, SlowRead7<u8, 0>); break;
                            }
                        }
                        else
                        {
                            switch ((8 << size) |  num)
                            {
                            case 32: QuickCallFunction(X3, SlowRead9<u32, 1>); break;
                            case 33: QuickCallFunction(X3, SlowRead7<u32, 1>); break;
                            case 16: QuickCallFunction(X3, SlowRead9<u16, 1>); break;
                            case 17: QuickCallFunction(X3, SlowRead7<u16, 1>); break;
                            case 8: QuickCallFunction(X3, SlowRead9<u8, 1>); break;
                            case 9: QuickCallFunction(X3, SlowRead7<u8, 1>); break;
                            }
                        }
                        ABI_PopRegisters(BitSet32({30}) | CallerSavedPushRegs);
                        if (size == 32)
                            MOV(rdMapped, W0);
                        else if (signextend)
                            SBFX(rdMapped, W0, 0, 8 << size);
                        else
                            UBFX(rdMapped, W0, 0, 8 << size);
                        RET();
#ifdef LITEV_JIT_PERFMAP
                        { char nm[44]; snprintf(nm, sizeof(nm), "jit_pload_c%dn%ds%dse%dr%d",
                            consoleType, num, size, signextend, reg);
                          litev_perfmap::AddStatic(nm, PatchedLoadFuncs[consoleType][num][size][signextend][reg], GetRXPtr()); }
#endif
                    }
                }
            }
        }
    }

    FlushIcache();

    JitMemSecondarySize = 1024*1024*4;
#ifdef LITEV_JIT_SHARED_CODE
    if (Shared)
        JitMemSecondarySize = SharedFarSize;
#endif

    JitMemMainSize -= GetCodeOffset();
    JitMemMainSize -= JitMemSecondarySize;
#ifdef LITEV_JIT_BGEMIT
    // background emission: two arenas after the far region, written only by the emulator
    // thread: block entry stubs and DirectPatch guard stubs (they used to be appended to the
    // block stream; the stream is now the emitter's alone, so its layout is deterministic)
    BgLayout = JitBgMode() != 0;
#ifdef LITEV_JIT_SHARED_CODE
    if (Shared)
        BgLayout = false;   // inline emission into the shared arena (ARMJIT::SharedEmit)
#endif
    if (BgLayout)
    {
        const u32 stubArena = 2 * 1024 * 1024, dpArena = 2 * 1024 * 1024;
        JitMemMainSize -= stubArena + dpArena;
        StubArenaStart = JitMemMainSize + JitMemSecondarySize;
        StubArenaEnd = StubArenaStart + stubArena;
        DPArenaStart = StubArenaEnd;
        DPArenaEnd = DPArenaStart + dpArena;
    }
#endif

    SetCodeBase((u8*)GetRWPtr(), (u8*)GetRXPtr());
#ifdef LITEV_JIT_SHARED_CODE
    if (Shared)
    {
        SharedArena& a = JitArena();
        std::lock_guard<std::recursive_mutex> lk(a.Lock);
        a.Base = (u8*)GetRXPtr();
        a.MainSize = JitMemMainSize;
        a.SecondarySize = JitMemSecondarySize;
        memcpy(a.JumpTo9, JumpToFuncs9, sizeof(JumpToFuncs9));
        memcpy(a.JumpTo7, JumpToFuncs7, sizeof(JumpToFuncs7));
        a.ReadBanked = ReadBanked;
        a.WriteBanked = WriteBanked;
        memcpy(a.PLoad, PatchedLoadFuncs, sizeof(PatchedLoadFuncs));
        memcpy(a.PStore, PatchedStoreFuncs, sizeof(PatchedStoreFuncs));
    }
#endif
}

Compiler::~Compiler()
{
#ifdef LITEV_JIT_SHARED_CODE
    if (Shared)
    {
        SharedArena& a = JitArena();
        std::lock_guard<std::recursive_mutex> lk(a.Lock);
        for (int c = 0; c < 2; c++)
            if (ICacheTable[c])
                munmap(ICacheTable[c], (size_t)SharedICacheSites * sizeof(ICacheEntry));
        if (--a.Users == 0)
        {
            if (getenv("LITEV_JITSHARE_STATS"))
                fprintf(stderr, "JITSHARE end: objs %zu hits %llu misses %llu near %.1f MB far %.1f MB sites %u ids %u\n", a.Objs.size(),
                        (unsigned long long)a.Hits, (unsigned long long)a.Misses, a.Near / 1048576.0,
                        (a.Far - (ptrdiff_t)JitMemMainSize) / 1048576.0, a.NextSite, a.NextId);
            ARMJIT_Global::FreeSharedCodeMem(a.Mem, SharedArenaSize);
            a.Mem = nullptr; a.Base = nullptr;
            a.Dispatchers = false;
            a.Near = a.Far = 0;
            a.NextId = a.NextSite = a.NextLinkSite = 1;
            a.Patches.clear();
            a.Objs.clear();
            a.Hits = a.Misses = 0;
        }
        ARMJIT_Global::DeInit();
        return;
    }
#endif
#ifdef __SWITCH__
    if (JitRWStart != NULL)
    {
        bool succeded = R_SUCCEEDED(svcUnmapProcessMemory(JitRWStart, envGetOwnProcessHandle(), (u64)JitRXStart, JitMemSize));
        assert(succeded);
        succeded = R_SUCCEEDED(svcUnmapProcessCodeMemory(envGetOwnProcessHandle(), (u64)JitRXStart, (u64)JitRWBase, JitMemSize));
        assert(succeded);
        free(JitRWBase);
    }
#endif

    ARMJIT_Global::FreeCodeMem(CodeMemBase);
    ARMJIT_Global::DeInit();
}

void Compiler::LoadCycles()
{
    LDR(INDEX_UNSIGNED, RCycles, RCPU, offsetof(ARM, Cycles));
}

void Compiler::SaveCycles()
{
#ifdef LITEV_JIT_CYCLE_BATCH
    FlushPendingCycles();
#endif
    STR(INDEX_UNSIGNED, RCycles, RCPU, offsetof(ARM, Cycles));
}

void Compiler::LoadReg(int reg, ARM64Reg nativeReg)
{
    if (reg == 15)
        MOVI2R(nativeReg, R15);
    else
        LDR(INDEX_UNSIGNED, nativeReg, RCPU, offsetof(ARM, R) + reg*4);
}

void Compiler::SaveReg(int reg, ARM64Reg nativeReg)
{
    STR(INDEX_UNSIGNED, nativeReg, RCPU, offsetof(ARM, R) + reg*4);
}

void Compiler::LoadCPSR()
{
    assert(!CPSRDirty);
#ifdef LITEV_JIT_LAZYFLAGS
    // V2: no RCPSR register to reload — but the within-slice C++ escape we are returning
    // from (interpreter fallback / restoreCPSR trampoline) may have written the guest NZCV
    // straight into ARM::CPSR[31:28]. Refresh the JitNZCV slot (the in-slice NZCV home)
    // from ARM::CPSR so the resuming JIT reads the up-to-date flags. NZCVDeferred is 0 at
    // every LoadCPSR site (the matching SaveCPSR cleared it). W0/W1 are free.
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CPSR));
    ANDI2R(W0, W0, 0xF0000000, W1);
    STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));
#else
    LDR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
#endif
}

void Compiler::SaveCPSR(bool markClean)
{
#ifdef LITEV_JIT_FIXEDREG
    // Reconcile any host-NZCV-resident guest flags into the canonical CPSR store before a
    // C escape / block boundary (mode switch, exception, interpreter fallback). Under
    // LAZYFLAGS this flushes host NZCV into the JitNZCV slot.
    Comp_MaterializeFlags();
#endif
#ifdef LITEV_JIT_LAZYFLAGS
    // V2: the materialize above put the guest NZCV in the JitNZCV slot and the control bits
    // are always canonical in ARM::CPSR — but ARM::CPSR[31:28] itself is now STALE (V2 does
    // NOT keep the full word current mid-slice). Merge the slot's NZCV nibble into the
    // ARM::CPSR word so the imminent within-slice C++/interpreter/trampoline reader observes
    // a fully-canonical CPSR (LoadCPSR refreshes the slot back afterwards). W0/W1/W2 free.
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, JitNZCV));   // W0 = NZCV<<28 (low bits 0)
    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));      // control (+ stale NZCV)
    ANDI2R(W1, W1, 0x0FFFFFFF, W2);                          // clear the stale NZCV nibble
    ORR(W1, W1, W0);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CPSR));
    if (markClean)
        CPSRDirty = false;
#else
    if (CPSRDirty)
    {
        STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
        CPSRDirty = CPSRDirty && !markClean;
    }
#endif
}

FixupBranch Compiler::CheckCondition(u32 cond)
{
#ifdef LITEV_EXIT_PROTO_NZCV
    const bool deferredAtEntry = NZCVDeferred != 0;
#endif
#ifdef LITEV_JIT_FIXEDREG
    if (NZCVDeferred)
    {
        // Guest flags are resident in host NZCV. Reconcile them into RCPSR (so the
        // conditionally-skipped body still observes a canonical CPSR word).
        // Comp_MaterializeFlags leaves PSTATE untouched.
        bool condValid = NZCVCondValid;
#ifdef LITEV_JIT_NZ_BRANCH
        // LITEV_JIT_NZ_BRANCH: EQ/NE read only Z, MI/PL only N. When that flag is one of the
        // host-resident ones (a logical producer's N,Z), branch on host NZCV like a full
        // producer instead of reloading the slot just written.
        const u8 needed = (cond <= 1) ? 0x4 : (cond == 4 || cond == 5) ? 0x8 : 0;
        if (JitQOn(jitq_NZBranch) && !condValid && needed && (NZCVDeferred & needed))
            condValid = true;
#endif
        Comp_MaterializeFlags();
        if (condValid)
        {
            // Full guest NZCV resident (arithmetic producer): evaluate the guest
            // condition NATIVELY on the still-live host NZCV. CCFlags == the ARM
            // cond field and (cond ^ 1) is the AArch64 inversion, so we branch
            // (skip the body) exactly when the guest condition is false. Valid for
            // every cond 0..13 (CheckCondition is only called for cond < 0xE).
            return B((CCFlags)(cond ^ 1));
        }
        // Only N,Z were host-resident (logical producer); host C,V are invalid.
        // The materialize above wrote N,Z into the canonical CPSR store (C,V were
        // already there), so fall through to the standard word path — which under
        // LAZYFLAGS reloads the full guest CPSR from ARM::CPSR memory.
    }
#endif
#ifdef LITEV_JIT_LAZYFLAGS
    // V2: RCPSR is gone and the guest NZCV is homed in the JitNZCV slot (NZCV in [31:28],
    // low bits 0). Read the condition from the slot: if the FIXEDREG block above hit the
    // FR_HOST_NZ path it just materialised N,Z into the slot (C,V were already there); under
    // FR_MEMORY the slot is canonical. Load it into a scratch (W1) and evaluate exactly as
    // the RCPSR path did — MSR NZCV consumes [31:28] and TBNZ/TBZ test a single bit in
    // [28..31], so the zero low bits are irrelevant. LAZYFLAGS requires CONDFOLD.
    {
#ifdef LITEV_EXIT_PROTO_NZCV
        // Host NZCV == the slot (and nothing was deferred, so the slot is canonical):
        // evaluate natively, exactly like the FR_HOST_FULL path above.
        if (NZCVHostSynced && !deferredAtEntry)
            return B((CCFlags)(cond ^ 1));
#endif
        LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, JitNZCV));
        if (cond >= 0x8)
        {
            _MSR(FIELD_NZCV, EncodeRegTo64(W1));
            return B((CCFlags)(cond ^ 1));
        }
        else
        {
            u8 bit = (28 + ((~(cond >> 1) & 1) << 1 | (cond >> 2 & 1) ^ (cond >> 1 & 1)));
            if (cond & 1)
                return TBNZ(W1, bit);
            else
                return TBZ(W1, bit);
        }
    }
#else
    if (cond >= 0x8)
    {
#ifdef LITEV_JIT_CONDFOLD
        // Compound condition (HI/LS/GE/LT/GT/LE). Guest CPSR lives in RCPSR with
        // NZCV in bits [31:28] — exactly the layout MSR NZCV consumes. Push the
        // guest flags into the host PSTATE and let the hardware evaluate the
        // condition natively. CCFlags is encoded identically to the ARM cond
        // field (CC_EQ..CC_LE == 0..13), and inverting an AArch64 condition is a
        // low-bit flip (cond ^ 1). We want to SKIP when the condition is FALSE,
        // so branch on the inverted condition. Bit-for-bit the same decision as
        // the flag-table path below, in 2 host instructions instead of 5.
        _MSR(FIELD_NZCV, EncodeRegTo64(RCPSR));
        return B((CCFlags)(cond ^ 1));
#else
        LSR(W1, RCPSR, 28);
        MOVI2R(W2, 1);
        LSLV(W2, W2, W1);
        ANDI2R(W2, W2, ARM::ConditionTable[cond], W3);

        return CBZ(W2);
#endif
    }
    else
    {
        u8 bit = (28 + ((~(cond >> 1) & 1) << 1 | (cond >> 2 & 1) ^ (cond >> 1 & 1)));

        if (cond & 1)
            return TBNZ(RCPSR, bit);
        else
            return TBZ(RCPSR, bit);
    }
#endif
}

#define F(x) &Compiler::A_Comp_##x
const Compiler::CompileFunc A_Comp[ARMInstrInfo::ak_Count] =
{
    // AND
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // EOR
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // SUB
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // RSB
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // ADD
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // ADC
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // SBC
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // RSC
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // ORR
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // MOV
    F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp),
    F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp),
    // BIC
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp), F(ALUTriOp),
    // MVN
    F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp),
    F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp), F(ALUMovOp),
    // TST
    F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp),
    // TEQ
    F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp),
    // CMP
    F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp),
    // CMN
    F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp), F(ALUCmpOp),
    // Mul
    F(Mul), F(Mul), F(Mul_Long), F(Mul_Long), F(Mul_Long), F(Mul_Long), F(Mul_Short), F(Mul_Short), F(Mul_Short), F(Mul_Short), F(Mul_Short),
    // ARMv5 exclusives
    F(Clz), NULL, NULL, NULL, NULL, 
    
    // STR
    F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB),
    // STRB
    F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB),
    // LDR
    F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB),
    // LDRB
    F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB), F(MemWB),
    // STRH
    F(MemHD), F(MemHD), F(MemHD), F(MemHD),
    // LDRD
    NULL, NULL, NULL, NULL,
    // STRD
    NULL, NULL, NULL, NULL,
    // LDRH
    F(MemHD), F(MemHD), F(MemHD), F(MemHD),
    // LDRSB
    F(MemHD), F(MemHD), F(MemHD), F(MemHD),
    // LDRSH
    F(MemHD), F(MemHD), F(MemHD), F(MemHD),
    // Swap
    NULL, NULL,
    // LDM, STM
    F(LDM_STM), F(LDM_STM),
    // Branch
    F(BranchImm), F(BranchImm), F(BranchImm), F(BranchXchangeReg), F(BranchXchangeReg),
    // Special
    NULL, F(MSR), F(MSR), F(MRS), NULL, NULL, NULL,
    &Compiler::Nop
};
#undef F
#define F(x) &Compiler::T_Comp_##x
const Compiler::CompileFunc T_Comp[ARMInstrInfo::tk_Count] =
{
    // Shift imm
    F(ShiftImm), F(ShiftImm), F(ShiftImm),
    // Add/sub tri operand
    F(AddSub_), F(AddSub_), F(AddSub_), F(AddSub_),
    // 8 bit imm
    F(ALUImm8), F(ALUImm8), F(ALUImm8), F(ALUImm8),
    // ALU
    F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU),
    F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU), F(ALU),
    // ALU hi reg
    F(ALU_HiReg), F(ALU_HiReg), F(ALU_HiReg),
    // PC/SP relative ops
    F(RelAddr), F(RelAddr), F(AddSP),
    // LDR PC rel
    F(LoadPCRel),
    // LDR/STR reg offset
    F(MemReg), F(MemReg), F(MemReg), F(MemReg),
    // LDR/STR sign extended, half
    F(MemRegHalf), F(MemRegHalf), F(MemRegHalf), F(MemRegHalf),
    // LDR/STR imm offset
    F(MemImm), F(MemImm), F(MemImm), F(MemImm),
    // LDR/STR half imm offset
    F(MemImmHalf), F(MemImmHalf),
    // LDR/STR sp rel
    F(MemSPRel), F(MemSPRel),
    // PUSH/POP
    F(PUSH_POP), F(PUSH_POP),
    // LDMIA, STMIA
    F(LDMIA_STMIA), F(LDMIA_STMIA),
    // Branch
    F(BCOND), F(BranchXchangeReg), F(BranchXchangeReg), F(B), F(BL_LONG_1), F(BL_LONG_2),
    // Unk, SVC
    NULL, NULL,
    F(BL_Merged)
};

bool Compiler::CanCompile(bool thumb, u16 kind)
{
    return (thumb ? T_Comp[kind] : A_Comp[kind]) != NULL;
}

#ifdef LITEV_JIT_DISPATCH
// liteDS-v2 Unit 3 — emitted per-CPU block dispatcher.
//
// Entered by an unconditional `B` from a block's exit tail. The exiting block has
// flushed all guest registers to memory and committed R[15] (the invariant upstream's
// C++ loop already relies on to recompute instrAddr after every ARM_Ret). RCPU(x29)/
// RCPSR(w27)/RCycles(w28) are live; RMemBase(x26) is reloaded by each block's own
// prologue; scratch w0-w7/x2-x7 are freely clobbered (the next block reloads its guest
// state from memory).
//
// It reproduces, bit-for-bit, the inter-block work of ARM::Execute<JIT>. Upstream runs:
//     run block K
//     if (StopExecution) { if (IRQ) TriggerIRQ(); if (Halted||IdleLoop) {..} }
//     Timestamp += Cycles; Cycles = 0;
// The order is load-bearing: the StopExecution handling (IRQ delivery, idle/halt skip)
// must observe Timestamp *without* block K's cycles, and only same-slice work that
// continues (chains) may advance Timestamp. Crucially `NDS::ScheduleEvent` schedules
// relative to ARMxTimestamp (not Timestamp+Cycles), so a stale mid-slice Timestamp
// would move every event a chained block schedules — hence the per-hop Timestamp update.
//
//   (a) StopExecution set  -> exit to C++ WITHOUT touching Timestamp/Cycles/budget, so
//       C++ runs its StopExecution handling (old Timestamp) then does Timestamp += Cycles.
//   (b) otherwise commit block K: Timestamp += Cycles; budget -= Cycles; Cycles = 0.
//   (c) budget <= 0 (natural slice end, or ForceExecutionExit which zeroes it) -> exit.
//   (d) region miss / lookup miss -> exit (Timestamp already advanced, Cycles 0).
//   (e) hit -> commit CPSR, br straight into the next block.
// All exit edges go to ARM_Ret (commits Cycles/CPSR, pops the callee frame).
#ifdef LITEV_JIT_DIRECTPATCH
// C-ABI trampolines the dispatcher BLs into (baked Compiler* as arg0). Defined after
// the Compiler methods below; forward-declared so Gen_Dispatcher can take their address.
extern u64 JitRecompileSame[8];
static void LiteV_DirectPatchPromote(Compiler* c, u32 num, u32 site, u32 guestTarget, u64 hostEntry);
static void LiteV_DirectPatchDemote(Compiler* c, u32 num, u32 site);
#endif
#ifdef LITEV_JIT_NATIVE_LOOP
// LITEV_JIT_NATIVE_LOOP (see NDS::NativeSliceNext): the ARM9 dispatcher's budget exit calls this
// and, given a new budget, goes on with the next block instead of returning to C++.
// Headless A/B in one binary: LITEV_JIT_NATIVE_LOOP_OFF=1; Android: debug.litev.nativeloop=0.
static bool NativeLoopOn()
{
    static const bool on = [] {
#if defined(__ANDROID__)
        char b[8] = {0};
        return __system_property_get("debug.litev.nativeloop", b) > 0 ? atoi(b) != 0 : true;
#else
        return !getenv("LITEV_JIT_NATIVE_LOOP_OFF");
#endif
    }();
    return on;
}
static s32 LiteV_NativeSlice9(melonDS::NDS* nds)
{
    return nds->NativeSliceNext();
}
#endif
#ifdef LITEV_JIT_SHARED_IC32
// LITEV_JIT_SHARED_IC32: shared code (no DIRECTPATCH) needs only the first 32 bytes of an ICACHE
// entry (keys, epoch, pointers), so its per-console tables are indexed 32-byte: half the lines and
// pages the dispatcher touches for the process-wide site ids. Exact (host-only).
// Headless A/B: LITEV_JIT_SHARED_IC32=0; Android: debug.litev.jitic32=0.
static bool IC32On()
{
    static const bool on = [] {
#if defined(__ANDROID__)
        char b[8] = {0};
        return !(__system_property_get("debug.litev.jitic32", b) > 0 && atoi(b) == 0);
#else
        const char* e = getenv("LITEV_JIT_SHARED_IC32");
        return !(e && atoi(e) == 0);
#endif
    }();
    return on;
}
#endif
#ifdef LITEV_JIT_SHARED_IC16
// LITEV_JIT_SHARED_IC16: shared code's per-console ICACHE entries are 16 bytes, {~key0, off0, ~key1,
// off1} (keys stored inverted, so a zero entry matches nothing; offsets into the code arena as in
// FastBlockLookup), with no epoch: when a block leaves FastBlockLookup the console clears its used
// entries (ICacheClearShared) before its JIT code runs again. One LDP per probe, a quarter of the
// 64-byte layout's lines. Exact (host-only). Headless A/B: LITEV_JIT_SHARED_IC16=0; Android:
// debug.litev.jitic16=0 (then IC32 / 64-byte entries as before).
static bool IC16On()
{
    static const bool on = [] {
#if defined(__ANDROID__)
        char b[8] = {0};
        return !(__system_property_get("debug.litev.jitic16", b) > 0 && atoi(b) == 0);
#else
        const char* e = getenv("LITEV_JIT_SHARED_IC16");
        return !(e && atoi(e) == 0);
#endif
    }();
    return on;
}
bool Compiler::IC16() const { return Shared && IC16On(); }
void Compiler::ICacheClearShared()
{
    if (!IC16()) return;
    const size_t bytes = (size_t)JitArena().NextSite << 4;
    for (int c = 0; c < 2; c++)
        if (ICacheTable[c]) memset(ICacheTable[c], 0, bytes);
}
#endif
#if defined(LITEV_JIT_SHARED_IC32) || defined(LITEV_JIT_SHARED_IC16)
int Compiler::ICShift() const
{
#ifdef LITEV_JIT_SHARED_IC16
    if (IC16()) return 4;
#endif
#ifdef LITEV_JIT_SHARED_IC32
    if (Shared && IC32On()) return 5;
#endif
    return ICacheEntryShift;
}
#ifndef LITEV_JIT_SHARED_IC16
bool Compiler::IC16() const { return false; }
void Compiler::ICacheClearShared() {}
#endif
#endif

void Compiler::LoadICacheTable(u32 num)
{
#ifdef LITEV_JIT_SHARED_CODE
    if (Shared)
    {
        LDR(INDEX_UNSIGNED, X2, RCPU, offsetof(ARM, JitICacheTable));
        return;
    }
#endif
    MOVP2R(X2, (void*)ICacheTable[num]);
}

void* Compiler::Gen_Dispatcher(u32 num)
{
    AlignCode16();
    void* res = GetRXPtr();

    // (Compiler has a member named NDS, so the bare name resolves to it; force the type.)
    // Bake the exact address of this CPU's Timestamp field. offsetof(NDS, ARMxTimestamp)
    // is multiple MB (NDS embeds the two ARM cores' huge PU/MemTimings arrays), which
    // overflows the scaled 12-bit immediate of a 64-bit LDR/STR — so address it at offset 0.
    void* tsPtr = (num == 0) ? (void*)&NDS.ARM9Timestamp : (void*)&NDS.ARM7Timestamp;

    // (a) StopExecution first — bounce out leaving RCycles = block K's cycles and Timestamp
    //     untouched (C++ handles IRQ/halt/idle with the pre-block Timestamp, then adds Cycles).
    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, StopExecution));
#ifdef LITEV_JIT_IRQMASK_CONT
    // (a') A pending IRQ the guest has masked (CPSR.I) is the only stop reason: C++ would
    //      call TriggerIRQ, which returns at once on CPSR.I, and re-dispatch with the same
    //      Timestamp/budget arithmetic as (b)-(c) below. So continue here instead of the
    //      round trip. StopExecution == 0x100 <=> IRQ = 1, Halted = IdleLoop = 0. ARM::CPSR
    //      memory is canonical at every exit under LAZYFLAGS (see (d)). The exit's site
    //      register W9 is not set on the stop paths of linked exits / guard stubs, so this
    //      hop opts out of the per-site cache.
    FixupBranch noStop = CBZ(W2);
    CMP(W2, 0x100);
    FixupBranch exitStop = B(CC_NEQ);
  #ifdef LITEV_JIT_LAZYFLAGS
    LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
    FixupBranch exitStopUnmasked = TBZ(W3, 7);
  #else
    FixupBranch exitStopUnmasked = TBZ(RCPSR, 7);
  #endif
  #if defined(LITEV_JIT_ICACHE) && !defined(LITEV_JIT_EXIT_PROTO)
    // Without EXIT_PROTO a linked exit's stop path reaches here with W9 unset. Under
    // EXIT_PROTO every way in sets W9 (block exit: its site; link stub: 0; guard stub:
    // its site), so the hop keeps its site's inline cache.
    MOVI2R(W9, 0);
  #endif
    SetJumpTarget(noStop);
#else
    FixupBranch exitStop = CBNZ(W2);
#endif

#ifdef LITEV_JIT_BUDGET_REG
    // (b) budget register: consume block K (Timestamp stays JitTsBase - budget; it is
    //     written only at the C++ exit tail below and before helper calls).
    (void)tsPtr;
    SUBS(RBudget, RBudget, RCycles);
    MOVI2R(RCycles, 0);
    FixupBranch exitBudget = B(CC_LE);
#else
    // (b) commit block K to the timeline: Timestamp += Cycles (64-bit, unshifted, exactly as
    //     `NDS.ARMxTimestamp += Cycles`), keep budget == Target-Timestamp, restart the
    //     accumulator at 0 for the next block. Cycles is always >= 0 so uxtw == the s32 value.
    MOVP2R(X4, tsPtr);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, RCycles, ArithOption(RCycles));      // x5 += (u32)RCycles
    STR(INDEX_UNSIGNED, X5, X4, 0);
    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    SUB(W1, W1, RCycles);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    MOVI2R(RCycles, 0);

    // (c) slice over or forced exit? budget <= 0
    CMP(W1, 0);
    FixupBranch exitBudget = B(CC_LE);
#endif

#ifdef LITEV_JIT_NATIVE_LOOP
    const void* lookupEntry = GetRXPtr();   // the native slice loop's re-entry
#endif
    // (d) instrAddr = R[15] - ((CPSR&0x20)?2:4) == (R[15] - 4) + (thumb << 1), as in the loop
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
#ifdef LITEV_JIT_LAZYFLAGS
    // RCPSR(W27) is now guest r7; read the Thumb bit from the canonical ARM::CPSR memory
    // (the exiting block materialized NZCV + control is always current, so it is canonical).
    LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
    UBFX(W3, W3, 5, 1);                               // W3 = Thumb bit
#else
    UBFX(W3, RCPSR, 5, 1);                            // W3 = Thumb bit
#endif
    SUB(W0, W0, 4);
    ADD(W0, W0, W3, ArithOption(W3, ST_LSL, 1));      // W0 = clean instrAddr

#ifdef LITEV_JIT_ICACHE
    // (d.5-ICACHE) per-site 2-way inline cache. W9 = site index (set by the exiting
    // block; 0 == "no cache", e.g. an unlinked EmitLinkExit hop). W0 = instrAddr is
    // PRESERVED (steps e/f/writeback below also rely on it). A hit requires the entry
    // epoch to match the live ARM::ICacheEpoch (O(1) invalidation) AND key0/key1 ==
    // instrAddr, then branches straight to the cached host block — skipping the region
    // bounds + the cache-cold FastBlockLookup gather. The commit (a)(b)(c) is already
    // done and the CPSR store is byte-identical to step (g), so a hit is timing-exact.
#if defined(LITEV_JIT_SHARED_IC32) || defined(LITEV_JIT_SHARED_IC16)
    const bool ic16 = IC16();
#else
    constexpr bool ic16 = false;
#endif
    // IC16 hit tail (inline): W4 = the matching way's code offset
    auto ic16Tail = [&]() {
  #ifndef LITEV_JIT_LAZYFLAGS
        STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
  #endif
        {   // the dispatchers sit at the start of the code arena: ADR reaches its base
            const s64 d = (s64)((u8*)GetRXBase() - (u8*)GetRXPtr());
            if (d >= -(1 << 20) && d < (1 << 20)) ADR(X3, (s32)d);
            else MOVI2R(X3, (u64)GetRXBase());
        }
        ADD(X5, X3, W4, ArithOption(W4));
  #if LITEV_PROFILE
        MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.ICacheHits);
        LDR(INDEX_UNSIGNED, X3, X4, 0);
        ADD(X3, X3, 1);
        STR(INDEX_UNSIGNED, X3, X4, 0);
  #endif
        BR(X5);
    };
    FixupBranch icHit0, icHit1;
    if (ic16)
    {
        FixupBranch noCache16 = CBZ(W9);
        LoadICacheTable(num);
        ADD(X2, X2, X9, ArithOption(X9, ST_LSL, 4));
        LDP(INDEX_SIGNED, W3, W4, X2, 0);            // ~key0, off0
        EON(W3, W3, W0);                             // 0 <=> key0 == instrAddr
        FixupBranch hit16 = CBZ(W3);
        LDP(INDEX_SIGNED, W3, W4, X2, 8);            // ~key1, off1
        EON(W3, W3, W0);
        FixupBranch miss16 = CBNZ(W3);
        SetJumpTarget(hit16);
        ic16Tail();
        SetJumpTarget(miss16);
        SetJumpTarget(noCache16);
    }
    else
    {
    FixupBranch icNoCache = CBZ(W9);
    LoadICacheTable(num);
    ADD(X2, X2, X9, ArithOption(X9, ST_LSL, ICShift())); // &ICacheTable[num][W9]
    LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, epoch));
    LDR(INDEX_UNSIGNED, W4, RCPU, offsetof(ARM, ICacheEpoch));
    CMP(W3, W4);
    FixupBranch icEpochMiss = B(CC_NEQ);
    LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key0));
    CMP(W3, W0);
    icHit0 = B(CC_EQ);
    LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key1));
    CMP(W3, W0);
    icHit1 = B(CC_EQ);
    SetJumpTarget(icEpochMiss);
    SetJumpTarget(icNoCache);
    }
    // miss -> fall through to the region-bounds + tag lookup, which WRITES BACK below.
#endif
#ifdef LITEV_JIT_RAS
    // (d.6-RAS) second chance: W10 = the caller's call-site slot popped by a guest
    // return (0 = none). Same epoch + key check as above, 1-way (key0/ptr0).
    FixupBranch rasHit;
    if (RasOn && ic16)
    {
        FixupBranch rasNone16 = CBZ(W10);
        LoadICacheTable(num);
        ADD(X2, X2, X10, ArithOption(X10, ST_LSL, 4));
        LDP(INDEX_SIGNED, W3, W4, X2, 0);            // ~key0, off0 (1-way)
        EON(W3, W3, W0);
        FixupBranch rasMiss16 = CBNZ(W3);
        ic16Tail();
        SetJumpTarget(rasMiss16);
        SetJumpTarget(rasNone16);
    }
    else if (RasOn)
    {
        FixupBranch rasNone = CBZ(W10);
        LoadICacheTable(num);
        ADD(X2, X2, X10, ArithOption(X10, ST_LSL, ICShift()));
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, epoch));
        LDR(INDEX_UNSIGNED, W4, RCPU, offsetof(ARM, ICacheEpoch));
        CMP(W3, W4);
        FixupBranch rasEpochMiss = B(CC_NEQ);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key0));
        CMP(W3, W0);
        rasHit = B(CC_EQ);
        SetJumpTarget(rasEpochMiss);
        SetJumpTarget(rasNone);
    }
#endif

    // (e) region bounds: offset = instrAddr - FastBlockLookupStart, exit if >= Size (unsigned;
    //     a single compare also catches instrAddr < Start via wraparound)
    LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, FastBlockLookupStart));
    SUB(W4, W0, W3);
    LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, FastBlockLookupSize));
    CMP(W4, W5);
#ifdef LITEV_JIT_REGION_CACHE
    // (e') window miss: try the cached regions (ARM::JitRegions, exact by construction)
    //      before giving up to C++. A hit makes it the current window, exactly as the
    //      C++ re-entry's JitSetupRegion would, then continues with the tag lookup.
    FixupBranch inRegion = B(CC_LO);
    FixupBranch rcHit[ARM::JitRegionCount];
    const u32 rcBase = offsetof(ARM, JitRegions);
    const u32 rcSize = sizeof(ARM::JitRegionEntry);
    for (int k = 0; k < ARM::JitRegionCount; k++)
    {
        LDR(INDEX_UNSIGNED, W3, RCPU, rcBase + k * rcSize + offsetof(ARM::JitRegionEntry, Start));
        SUB(W4, W0, W3);
        LDR(INDEX_UNSIGNED, W5, RCPU, rcBase + k * rcSize + offsetof(ARM::JitRegionEntry, Size));
        CMP(W4, W5);
        rcHit[k] = B(CC_LO);
    }
    FixupBranch exitRegion = B();
    FixupBranch rcToLookup[ARM::JitRegionCount];
    for (int k = 0; k < ARM::JitRegionCount; k++)
    {
        SetJumpTarget(rcHit[k]);
        LDR(INDEX_UNSIGNED, X6, RCPU, rcBase + k * rcSize + offsetof(ARM::JitRegionEntry, Lookup));
        STR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, FastBlockLookupStart));
        STR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, FastBlockLookupSize));
        STR(INDEX_UNSIGNED, X6, RCPU, offsetof(ARM, FastBlockLookup));
        rcToLookup[k] = B();
    }
    SetJumpTarget(inRegion);
#else
    FixupBranch exitRegion = B(CC_HS);
#endif

    // (f) inline tag lookup: entry = FastBlockLookup[offset/2]; tag = entry>>32 == (instrAddr|num)
    LDR(INDEX_UNSIGNED, X6, RCPU, offsetof(ARM, FastBlockLookup));
#ifdef LITEV_JIT_REGION_CACHE
    for (int k = 0; k < ARM::JitRegionCount; k++)
        SetJumpTarget(rcToLookup[k]);
#endif
    LSR(W4, W4, 1);
    LDR(X7, X6, ArithOption(W4, true));               // ldr x7, [x6, w4, uxtw #3]
    LSR(X2, X7, 32);                                  // W2 = tag (high word)
    FixupBranch miss;
    if (num == 0)
    {
        CMP(W2, W0);
        miss = B(CC_NEQ);
    }
    else
    {
        ORRI2R(W3, W0, 1);                            // ARM7 tag carries num bit
        CMP(W2, W3);
        miss = B(CC_NEQ);
    }

    // (g) hit: commit live CPSR to memory before entering the next block. Blocks are compiled
    //     assuming CPSR-in-memory is current at their entry (per-block CPSRDirty starts false,
    //     so an interpreter-fallback first instruction skips its SaveCPSR); upstream upholds
    //     this via ARM_Ret's CPSR store, which chaining bypasses.
#ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
#endif
    // LAZYFLAGS: ARM::CPSR is already canonical in memory at every block exit (the exiting
    // block materialized host NZCV and control bits are never register-cached), so the
    // per-hop commit is unnecessary; W27 now holds guest r7 and must not be stored here.
    // low 32 bits of the entry (W7) are the sub-entry offset into the RX region; baked RXBase
    // == GetRXBase() (stable rebased base, the same one SubEntryOffset subtracts).
    MOVI2R(X3, (u64)GetRXBase());
#ifdef LITEV_JIT_ICACHE
    // Compute host addr into X6 (keep W0 = instrAddr for the cache key), fill this
    // site's 2-way entry (LRU: demote slot0->slot1, new -> slot0) stamped with the
    // live epoch, then branch. W9 == 0 means the exiting site opted out of caching.
    ADD(X6, X3, W7, ArithOption(W7));                 // X6 = absolute host block entry
    if (ic16)
    {
        // IC16: slot0 -> slot1, then {~instrAddr, offset} -> slot0 (no epoch: dead entries were cleared)
        FixupBranch wbSkip = CBZ(W9);
        LoadICacheTable(num);
        ADD(X2, X2, X9, ArithOption(X9, ST_LSL, 4));
        LDP(INDEX_SIGNED, W3, W4, X2, 0);
        STP(INDEX_SIGNED, W3, W4, X2, 8);
        ORN(W3, WZR, W0);
        STP(INDEX_SIGNED, W3, W7, X2, 0);
        SetJumpTarget(wbSkip);
    }
    else
    {
        FixupBranch wbSkip = CBZ(W9);
        LoadICacheTable(num);
        ADD(X2, X2, X9, ArithOption(X9, ST_LSL, ICShift()));
        // The epoch covers both ways: demote slot0 only if it is from the live epoch. A stale
        // slot0 (its block may be invalidated since) would otherwise come back to life under
        // the new stamp, and the guest would run a dead block's code (another JIT state than
        // the one a savestate rebuilds, and the old code if the guest rewrote it). Stale: slot1
        // gets key ~0, which no (even) instruction address matches.
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, epoch));
        LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, ICacheEpoch));
        CMP(W3, W5);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key0));
        CSINV(W3, W3, WZR, CC_EQ);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key1));   // key1 = old key0 (or none)
        LDR(INDEX_UNSIGNED, X4, X2, offsetof(ICacheEntry, ptr0));
        STR(INDEX_UNSIGNED, X4, X2, offsetof(ICacheEntry, ptr1));   // ptr1 = old ptr0
        STR(INDEX_UNSIGNED, W0, X2, offsetof(ICacheEntry, key0));   // key0 = instrAddr
        STR(INDEX_UNSIGNED, X6, X2, offsetof(ICacheEntry, ptr0));   // ptr0 = host entry
        STR(INDEX_UNSIGNED, W5, X2, offsetof(ICacheEntry, epoch));
        SetJumpTarget(wbSkip);
    }
#ifdef LITEV_JIT_RAS
    if (RasOn && ic16)
    {
        FixupBranch rasSkip16 = CBZ(W10);
        LoadICacheTable(num);
        ADD(X2, X2, X10, ArithOption(X10, ST_LSL, 4));
        ORN(W3, WZR, W0);
        STP(INDEX_SIGNED, W3, W7, X2, 0);            // slot0 only (slot1 kept), as below
        SetJumpTarget(rasSkip16);
    }
    else if (RasOn)
    {
        FixupBranch rasSkip = CBZ(W10);
        LoadICacheTable(num);
        ADD(X2, X2, X10, ArithOption(X10, ST_LSL, ICShift()));
        // (same rule: re-stamping a stale entry must not revive its slot1)
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, epoch));
        LDR(INDEX_UNSIGNED, W5, RCPU, offsetof(ARM, ICacheEpoch));
        CMP(W3, W5);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key1));
        CSINV(W3, W3, WZR, CC_EQ);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, key1));
        STR(INDEX_UNSIGNED, W0, X2, offsetof(ICacheEntry, key0));
        STR(INDEX_UNSIGNED, X6, X2, offsetof(ICacheEntry, ptr0));
        STR(INDEX_UNSIGNED, W5, X2, offsetof(ICacheEntry, epoch));
        SetJumpTarget(rasSkip);
    }
#endif
  #if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DispatcherHits);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
  #endif
#ifdef LITEV_JIT_DIRECTPATCH
#ifdef LITEV_JIT_SHARED_CODE
    if (!Shared)   // (shared code is never patched per console)
#endif
    // A writeback == this site did NOT hit slot0 -> break its consecutive-slot0 streak.
    // If it is nonetheless PROMOTED, its guard permanently missed (primary target
    // changed) and we are re-resolving in the dispatcher -> DEMOTE (revert the exit-B).
    // W9 = site (0 = no cache); X6 = host entry (preserved across the demote call).
    {
        FixupBranch dpNoCache = CBZ(W9);
        LoadICacheTable(num);
        ADD(X2, X2, X9, ArithOption(X9, ST_LSL, ICShift()));
        STR(INDEX_UNSIGNED, WZR, X2, offsetof(ICacheEntry, hitCount));   // streak reset
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, promoted));
        FixupBranch dpNotPromoted = CBZ(W3);
        ABI_PushRegisters({6});                       // preserve X6 (host entry)
        MOV(W2, W9);                                  // arg2 = site
        MOVI2R(W1, num);                              // arg1 = num
        MOVP2R(X0, (void*)this);                      // arg0 = Compiler*
        QuickCallFunction(X16, (void*)&LiteV_DirectPatchDemote);
        ABI_PopRegisters({6});
        SetJumpTarget(dpNotPromoted);
        SetJumpTarget(dpNoCache);
    }
#endif
    BR(X6);
#else
    ADD(X0, X3, W7, ArithOption(W7));                 // add x0, x3, w7, uxtw
#if LITEV_PROFILE
    // Runtime dispatcher-resolved chain hops (compare to CommitStubEntries = linked hops -> the
    // ACTUAL hot-loop link coverage). X4/X5 are dead here; X0 (the branch target) is preserved.
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DispatcherHits);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
#endif
    BR(X0);
#endif

#ifdef LITEV_JIT_RAS
    if (RasOn && !ic16)
    {
        // RAS hit (X2 = the call-site entry). CPSR is canonical in memory (LAZYFLAGS).
        SetJumpTarget(rasHit);
  #ifndef LITEV_JIT_LAZYFLAGS
        STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
  #endif
        LDR(INDEX_UNSIGNED, X5, X2, offsetof(ICacheEntry, ptr0));
  #if LITEV_PROFILE
        MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.RasHits);
        LDR(INDEX_UNSIGNED, X3, X4, 0);
        ADD(X3, X3, 1);
        STR(INDEX_UNSIGNED, X3, X4, 0);
  #endif
        BR(X5);
    }
#endif
#ifdef LITEV_JIT_ICACHE
    // ICACHE hit tails (reached from (d.5-ICACHE); X2 still holds the entry pointer).
    // Under LAZYFLAGS ARM::CPSR is already canonical (as at step (g)); otherwise commit
    // the live RCPSR exactly as the plain hit path does.
    if (!ic16)
    {
    SetJumpTarget(icHit0);
  #ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
  #endif
    LDR(INDEX_UNSIGNED, X5, X2, offsetof(ICacheEntry, ptr0));
  #if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.ICacheHits);
    LDR(INDEX_UNSIGNED, X3, X4, 0);
    ADD(X3, X3, 1);
    STR(INDEX_UNSIGNED, X3, X4, 0);
  #endif
#ifdef LITEV_JIT_DIRECTPATCH
#ifdef LITEV_JIT_SHARED_CODE
    if (!Shared)
#endif
    // A slot0 hit is a consecutive-same-target (monomorphic) resolve. Bump this site's
    // streak; at DirectPatchThreshold, PROMOTE (emit a per-site guard stub + patch the
    // exit-B to it) via a C++ helper. A promoted site never re-enters here (its guard
    // takes the target directly), so this counter only runs for not-yet-promoted sites.
    // Live: X2=&entry, W0=instrAddr(guest target), W9=site, X5=host entry (ptr0).
    {
        // An ALREADY-promoted site can land here on a guard-miss fallback whose
        // target matches key0 (bimodal site). Skip the streak/promote work entirely:
        // the C++ promote call would be a useless round-trip (idempotent early-out).
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, promoted));
        FixupBranch dpProm = CBNZ(W3);
        LDR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, hitCount));
        ADD(W3, W3, 1);
        STR(INDEX_UNSIGNED, W3, X2, offsetof(ICacheEntry, hitCount));
        CMP(W3, DirectPatchThreshold);
        FixupBranch dpBelow = B(CC_LT);
        ABI_PushRegisters({5});                       // preserve X5 (host entry) across the call
        MOV(W3, W0);                                  // arg3 = guestTarget (from W0=instrAddr)
        MOV(X4, X5);                                  // arg4 = hostEntry
        MOV(W2, W9);                                  // arg2 = site
        MOVI2R(W1, num);                              // arg1 = num
        MOVP2R(X0, (void*)this);                      // arg0 = Compiler*
        QuickCallFunction(X16, (void*)&LiteV_DirectPatchPromote);
        ABI_PopRegisters({5});
        SetJumpTarget(dpBelow);
        SetJumpTarget(dpProm);
    }
#endif
    BR(X5);
    SetJumpTarget(icHit1);
  #ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
  #endif
    LDR(INDEX_UNSIGNED, X5, X2, offsetof(ICacheEntry, ptr1));
  #if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.ICacheHits);
    LDR(INDEX_UNSIGNED, X3, X4, 0);
    ADD(X3, X3, 1);
    STR(INDEX_UNSIGNED, X3, X4, 0);
  #endif
#ifdef LITEV_JIT_DIRECTPATCH
    // A slot1 hit breaks the consecutive-slot0 streak (site is bimodal, not monomorphic).
#ifdef LITEV_JIT_SHARED_CODE
    if (!Shared)   // (shared code: no DIRECTPATCH, and LITEV_JIT_SHARED_IC32 entries have no such field)
#endif
    STR(INDEX_UNSIGNED, WZR, X2, offsetof(ICacheEntry, hitCount));
#endif
    BR(X5);
    }
#endif

#if defined(LITEV_JIT_NATIVE_LOOP) && defined(LITEV_JIT_BUDGET_REG)
    if (num == 0 && NativeLoopOn())
    {
        // The slice is over: spill as for C++ (Timestamp, CyclesBudget), then let the native
        // slice loop run the scheduler up to the next ARM9 slice. W0 = its budget (> 0: go on
        // with the block at R[15], skipping the per-site caches as a C++ entry does), 0 = C++.
        // The guest registers pinned in callee-saved host registers stay live across the call.
        SetJumpTarget(exitBudget);
        EmitBudgetSpill(X1, X2, X3);
#ifdef LITEV_JIT_SHARED_CODE
        if (Shared)
            LDR(INDEX_UNSIGNED, X0, RCPU, offsetof(ARM, JitNDS));
        else
#endif
        MOVP2R(X0, (void*)&NDS);
        ARM64XEmitter::QuickCallFunction(X16, (void*)&LiteV_NativeSlice9);
        FixupBranch toCpp = CBZ(W0);
        MOV(RBudget, W0);
  #ifdef LITEV_JIT_ICACHE
        MOVI2R(W9, 0);
  #endif
  #ifdef LITEV_JIT_RAS
        if (RasOn)
            MOVI2R(W10, 0);
  #endif
        B(lookupEntry);
        SetJumpTarget(toCpp);
        QuickTailCall(X0, ARM_Ret);
    }
    else
        SetJumpTarget(exitBudget);
#else
    SetJumpTarget(exitBudget);
#endif
    SetJumpTarget(exitStop);
#ifdef LITEV_JIT_IRQMASK_CONT
    SetJumpTarget(exitStopUnmasked);
#endif
    SetJumpTarget(exitRegion);
    SetJumpTarget(miss);
#ifdef LITEV_JIT_BUDGET_REG
    // Back to C++: leave CyclesBudget / Timestamp exactly as the per-hop commit did
    // (on the StopExecution path block K's cycles are still in RCycles, uncommitted).
    EmitBudgetSpill(X1, X2, X3);
#endif
#if LITEV_PROFILE
    // Dispatcher exits to C++ (block-lookup miss / recompile churn + the rare slice-end).
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DispatcherMisses);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
#endif
    QuickTailCall(X0, ARM_Ret);

    return res;
}

void Compiler::EmitBlockExit()
{
#ifdef LITEV_JIT_LAZYFLAGS
    // The dispatcher no longer stores RCPSR, so ARM::CPSR memory MUST be canonical here.
    // In every path this exit is reached the flags were already flushed (block-end 1419 /
    // Comp_JumpTo / CheckCondition), so this is a compile-time no-op (NZCVDeferred==0 ->
    // emits nothing); it is the "flags-dirty-guarded" flush the design specifies, and it
    // makes the invariant explicit / robust rather than implicit.
    Comp_MaterializeFlags();
#endif
#ifdef LITEV_EXIT_PROTO_PC
    // The dispatcher reads R[15]: an elided PC store must have been for a link exit.
    if (PCElided)
    {
        Log(LogLevel::Error, "JIT: PC store elided for a dispatcher exit (block %08x)\n", BlockStartAddr);
        abort();
    }
#endif
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.DispatchOnlyExits);
#ifdef LITEV_JIT_ICACHE
    // Hand this dynamic exit its own per-site cache slot; the dispatcher (d.5-ICACHE)
    // reads W9 to index / fill the 2-way entry. Index 0 (assigned on table overflow)
    // disables caching for this site. Compile-time constant -> baked immediate.
#ifdef LITEV_JIT_DIRECTPATCH
    u32 dpSite = ICacheAssignSite();
    MOVI2R(W9, dpSite);
#ifdef LITEV_JIT_RAS
    // W10 = the call-site slot this path's taken return popped (0 = none), the
    // dispatcher's second chance after a W9 miss. Cleared so a not-taken path of a
    // conditional return never sees a stale slot.
    if (RasRetBlock)
    {
        LDR(INDEX_UNSIGNED, W10, RCPU, offsetof(ARM, RasSite));
        STR(INDEX_UNSIGNED, WZR, RCPU, offsetof(ARM, RasSite));
    }
    else if (RasOn)
        MOVI2R(W10, 0);
#endif
    // Record the RX offset of the patchable exit `B` so DIRECTPATCH can later rewrite
    // it to a guard stub. Compile-time; survives until the next ICacheReset (which
    // zeroes the whole table, incl. patchOff/hitCount/promoted). Site 0 = no cache.
    if (dpSite != 0 && ICacheTable[Num]
#ifdef LITEV_JIT_SHARED_CODE
        && !Shared
#endif
        )
        ICacheTable[Num][dpSite].patchOff = (u32)((u8*)GetRXPtr() - GetRXBase());
#else
    MOVI2R(W9, ICacheAssignSite());
#endif
#endif
    B(DispatcherEntry[Num]);
}
#endif

#if defined(LITEV_JIT_EXIT_TAIL) || defined(LITEV_JIT_LDR_ALIGNCHK) || defined(LITEV_JIT_COLD_EXITS)
void Compiler::EmitTailStubs()
{
    // index loop: a cold exit emitted here appends its own link stub to TailStubs
    for (size_t k = 0; k < TailStubs.size(); k++)
    {
        TailStub t = TailStubs[k];
#ifdef LITEV_JIT_COLD_EXITS
        if (t.Kind == 2)
        {
            SetJumpTarget(t.A);
            EmitColdExit(ColdExits[t.Index]);
            continue;
        }
#endif
        if (t.Kind == 0)
        {
            // link exit: budget spent -> undo the subtraction, then the unlinked stub
            SetJumpTarget(t.A);
            ADD(RBudget, RBudget, RCycles);
            SetJumpTarget(t.B);
  #ifdef LITEV_JIT_ICACHE
            MOVI2R(W9, 0);   // a linkable exit opts out of the per-site cache (0 = no cache)
  #endif
  #ifdef LITEV_JIT_RAS
            if (RasOn)
                MOVI2R(W10, 0);
  #endif
            if (t.PCElided)
            {
                MOVI2R(W0, t.NewPC);
                STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
            }
            B(DispatcherEntry[Num]);
        }
        else
        {
            // misaligned load: the slow-path thunk (W0 = address) rotates exactly like the
            // fastmem sequence it replaces did
            SetJumpTarget(t.A);
            BL(t.Func);
            B(t.Back);
        }
    }
    TailStubs.clear();
#ifdef LITEV_JIT_COLD_EXITS
    ColdExits.clear();
#endif
}
#endif

#ifdef LITEV_JIT_COLD_EXITS
void Compiler::SaveCompState(CompState& s)
{
    s.RegCache = RegCache;
    s.CurInstr = CurInstr; s.CurIdx = CurIdx; s.R15 = R15; s.CodeRegion = CodeRegion; s.ConstantCycles = ConstantCycles;
    s.Exit = Exit; s.IrregularCycles = IrregularCycles; s.CPSRDirty = CPSRDirty;
#ifdef LITEV_JIT_CYCLE_BATCH
    s.PendingCycles = PendingCycles; s.DeferCycles = DeferCycles;
#endif
#ifdef LITEV_JIT_FIXEDREG
    s.NZCVDeferred = NZCVDeferred; s.NZCVCondValid = NZCVCondValid;
#endif
#ifdef LITEV_JIT_LAZYFLAGS
    s.FlagsLiveInCur = FlagsLiveInCur; s.CarryInHostResident = CarryInHostResident;
#ifdef LITEV_EXIT_PROTO_NZCV
    s.NZCVHostSynced = NZCVHostSynced; s.LastFlushFull = LastFlushFull;
#endif
#endif
#ifdef LITEV_JIT_LINK
    s.HasStaticExit = HasStaticExit; s.StaticExitCond = StaticExitCond;
    s.StaticExitTarget = StaticExitTarget; s.StaticExitNewPC = StaticExitNewPC;
#endif
#ifdef LITEV_EXIT_PROTO_PC
    s.PCElided = PCElided;
#endif
#ifdef LITEV_JIT_RAS
    s.RasRetBlock = RasRetBlock;
#endif
}

void Compiler::LoadCompState(const CompState& s)
{
    RegCache = s.RegCache;
    CurInstr = s.CurInstr; CurIdx = s.CurIdx; R15 = s.R15; CodeRegion = s.CodeRegion; ConstantCycles = s.ConstantCycles;
    Exit = s.Exit; IrregularCycles = s.IrregularCycles; CPSRDirty = s.CPSRDirty;
#ifdef LITEV_JIT_CYCLE_BATCH
    PendingCycles = s.PendingCycles; DeferCycles = s.DeferCycles;
#endif
#ifdef LITEV_JIT_FIXEDREG
    NZCVDeferred = s.NZCVDeferred; NZCVCondValid = s.NZCVCondValid;
#endif
#ifdef LITEV_JIT_LAZYFLAGS
    FlagsLiveInCur = s.FlagsLiveInCur; CarryInHostResident = s.CarryInHostResident;
#ifdef LITEV_EXIT_PROTO_NZCV
    NZCVHostSynced = s.NZCVHostSynced; LastFlushFull = s.LastFlushFull;
#endif
#endif
#ifdef LITEV_JIT_LINK
    HasStaticExit = s.HasStaticExit; StaticExitCond = s.StaticExitCond;
    StaticExitTarget = s.StaticExitTarget; StaticExitNewPC = s.StaticExitNewPC;
#endif
#ifdef LITEV_EXIT_PROTO_PC
    PCElided = s.PCElided;
#endif
#ifdef LITEV_JIT_RAS
    RasRetBlock = s.RasRetBlock;
#endif
}

void Compiler::SaveCpuJumpFields(CpuJumpFields& f)
{
#ifdef LITEV_JIT_BGEMIT
    if (CurJob) return;   // a captured block's emission never touches the CPU
#endif
    f.CodeRegion = CurCPU->CodeRegion; f.CodeCycles = CurCPU->CodeCycles;
    f.RegionCodeCycles = Num == 0 ? ((ARMv5*)CurCPU)->RegionCodeCycles : 0;
    f.CodeMem = CurCPU->CodeMem; f.R15 = CurCPU->R[15];
}

void Compiler::LoadCpuJumpFields(const CpuJumpFields& f)
{
#ifdef LITEV_JIT_BGEMIT
    if (CurJob) return;
#endif
    CurCPU->CodeRegion = f.CodeRegion; CurCPU->CodeCycles = f.CodeCycles;
    if (Num == 0)
        ((ARMv5*)CurCPU)->RegionCodeCycles = f.RegionCodeCycles;
    CurCPU->CodeMem = f.CodeMem; CurCPU->R[15] = f.R15;
}

// Idle-loop branches stay in place: their taken edge registers an IDLE2 site at compile time,
// in compile order.
bool Compiler::ColdExitOK() const
{
    return JitQOn(jitq_ColdExits)
        && (CurInstr.BranchFlags & (branch_FollowCondTaken | branch_FollowCondNotTaken))
        && !(CurInstr.BranchFlags & branch_IdleBranch)
#ifdef LITEV_JIT_IDLE2
        && !(CurInstr.BranchFlags & branch_Idle2Cand)
#endif
        ;
}

// `from` jumps to the exit; the current compiler state is what that edge starts with
void Compiler::DeferColdExit(FixupBranch from, u8 kind)
{
    ColdExit e;
    SaveCompState(e.State);
    e.Kind = kind;
    ColdExits.push_back(e);
    TailStub t{};
    t.Kind = 2;
    t.A = from;
    t.Index = (u32)(ColdExits.size() - 1);
    TailStubs.push_back(t);
    if (kind != 1)
    {
        // The taken edge's Comp_JumpTo changes CPU fields at compile time; whatever compiles
        // last leaves them for C++. Make that change here, in program order, by compiling it
        // into scratch space (the far region past its live end, which the next far-code
        // emission overwrites); the tail compile of it restores the fields it changes.
        ptrdiff_t at = GetCodeOffset();
        SetCodePtrUnsafe(OtherCodeRegion);
  #ifdef LITEV_JIT_ICACHE
        u32 sites = ICacheNextSite;
  #endif
        if (kind == 0)
            A_Comp_BranchImm();
        else
            Comp_JumpTo(R15 + ((s32)(CurInstr.Instr << 24) >> 23) + 1, true);
  #ifdef LITEV_JIT_ICACHE
        ICacheNextSite = sites;
  #endif
        SetCodePtrUnsafe(at);
        LoadCompState(e.State);
    }
}

// The same calls, in the same order and state, as the in-place edge (CompileBlock's ARM
// conditional path / T_Comp_BCOND). Everything they change in the compiler is local to the
// exit, except what the caller applies to the continuing path itself (see the callers).
void Compiler::EmitColdExit(const ColdExit& e)
{
    CompState cur;
    SaveCompState(cur);
    CpuJumpFields cpu;
    SaveCpuJumpFields(cpu);
    LoadCompState(e.State);
    if (e.Kind == 0)
    {
        A_Comp_BranchImm();
        Comp_BranchSpecialBehaviour(true);
    }
    else if (e.Kind == 1)
        Comp_BranchSpecialBehaviour(false);
    else
    {
        s32 offset = (s32)(CurInstr.Instr << 24) >> 23;
        Comp_JumpTo(R15 + offset + 1, true);
        Comp_BranchSpecialBehaviour(true);
    }
    LoadCpuJumpFields(cpu);
    LoadCompState(cur);
}
#endif

#ifdef LITEV_JIT_LINK
// liteDS-v2 Unit 4 — a linkable static exit.
//
// A linked branch bypasses the dispatcher, so it must reproduce, at the SOURCE
// site, exactly the per-hop work the dispatcher does before entering the next
// block — otherwise a chained hop diverges from the pure-dispatch build. The
// order here mirrors Gen_Dispatcher bit-for-bit:
//   (a) StopExecution set  -> dispatcher (which, still holding this block's Cycles
//       and an un-advanced Timestamp, bounces to C++ identically to a plain exit);
//   (b) commit this block: Timestamp += Cycles; budget -= Cycles; Cycles = 0;
//   (c) budget <= 0        -> dispatcher (re-commits as a no-op, then exits);
//   (d) hit: commit CPSR, then the patchable `B`. UNLINKED it targets the
//       dispatcher (whose inline lookup finds the — possibly not-yet-compiled —
//       target); LINKED it is rewritten to branch straight into the target block.
// Because the slot only ever holds an unconditional `B`, it can be re-patched
// (link / unlink) concurrently with execution on ARMv8 without synchronisation.
void Compiler::EmitLinkExit(u32 targetAddr, u32 newPC)
{
#ifdef LITEV_JIT_LAZYFLAGS
    // A linked hop bypasses the dispatcher AND (under LAZYFLAGS) neither the stub nor the
    // inline (d) stores RCPSR, so ARM::CPSR memory MUST be canonical before the exit. It
    // always is (the mid-block conditional edges reach here after Comp_JumpTo / CheckCondition
    // flushed, and the block-end after 1419), so this guarded flush is a compile-time no-op.
    Comp_MaterializeFlags();
#endif
#ifdef LITEV_JIT_EXIT_PROTO
    // EXIT_PROTO hop. Hot path (linked, with the STOP part):
    //     subs w15, w15, w28 ; b.le cold ; mov w28, #0 ; b <target>
    // The slot's own stub follows it; it is the unlinked target (UnlinkedSiteTarget):
    //     [mov w9, #0] ; [store R[15]] ; b dispatcher
    // and the cold path undoes the subtraction and joins the stub, so the dispatcher
    // sees exactly what an unlinked hop always gave it and runs (a)-(c) itself: a pending
    // stop still exits with block K's cycles uncommitted, a spent budget still commits.
  #ifndef LITEV_EXIT_PROTO_STOP
    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, StopExecution));
    FixupBranch toStop = CBNZ(W2);
  #endif
    // STOP: a stop the dispatcher would exit on has already forced the budget to 0
    // (ARM::JitStopToBudget), so this one compare covers it.
    SUBS(RBudget, RBudget, RCycles);
    FixupBranch toCold = B(CC_LE);
    MOVI2R(RCycles, 0);
    u32 patchOffset = (u32)((u8*)GetRXPtr() - GetRXBase());
#ifdef LITEV_JIT_SHARED_CODE
    // Shared code: no patch slot. The hop branches through this console's link table entry for
    // the site (0 = unlinked: the stub below, as the unlinked slot does), so each console's links
    // are exactly the ones its own block cache made (UnlinkBlock clears them where the unshared
    // JIT unpatches). The site's table index takes the place of the patch offset.
    FixupBranch sharedUnlinked;
    const bool sharedSite = Shared;
    if (sharedSite)
    {
        SharedArena& a = JitArena();
        if (a.NextLinkSite >= LinkSites) { fprintf(stderr, "LITEV_JIT_SHARED_CODE: out of link sites\n"); abort(); }
        const u32 site = a.NextLinkSite++;
        patchOffset = site;
        const u32 off = site * 8;
        LDR(INDEX_UNSIGNED, X16, RCPU, offsetof(ARM, JitLinks));
        ADD(X16, X16, off >> 12, true);
        LDR(INDEX_UNSIGNED, X16, X16, off & 0xFFF);
        sharedUnlinked = CBZ(X16);
        BR(X16);
    }
    else
#endif
    {
    FixupBranch slot = B();   // the patch slot; unlinked it branches to the stub below
    SetJumpTarget(slot);
    }
#ifdef LITEV_JIT_EXIT_TAIL
    if (JitQOn(jitq_ExitTail))
    {
        // The stub and the budget-expired path go to the block tail; the word after the
        // slot (still its unlinked target, UnlinkedSiteTarget) only branches there. Same
        // instructions run on every path, two more taken branches on the cold ones.
        TailStub t{};
        t.Kind = 0;
        t.A = toCold;
#ifdef LITEV_JIT_SHARED_CODE
        if (sharedSite)
            t.B = sharedUnlinked;   // (the table entry is 0: to the stub)
        else
#endif
        t.B = B();
  #ifdef LITEV_EXIT_PROTO_PC
        t.PCElided = PCElided;
        t.NewPC = newPC;
        PCElided = false;
  #endif
        TailStubs.push_back(t);
        if (NumLinkExits < MaxOutgoingLinks)
        {
            LinkExits[NumLinkExits].PatchOffset = patchOffset;
            LinkExits[NumLinkExits].TargetAddr = targetAddr;
            NumLinkExits++;
        }
        LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.LinkSitesEmitted);
        return;
    }
#endif
    const void* stub = GetRXPtr();
#ifdef LITEV_JIT_SHARED_CODE
    if (sharedSite) SetJumpTarget(sharedUnlinked);
#endif
  #ifdef LITEV_JIT_ICACHE
    MOVI2R(W9, 0);   // a linkable exit opts out of the per-site cache (0 = no cache)
  #endif
  #ifdef LITEV_JIT_RAS
    if (RasOn)
        MOVI2R(W10, 0);
  #endif
  #ifdef LITEV_EXIT_PROTO_PC
    if (PCElided)
    {
        MOVI2R(W0, newPC);
        STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
        PCElided = false;
    }
  #else
    (void)newPC;
  #endif
    B(DispatcherEntry[Num]);
    SetJumpTarget(toCold);
    ADD(RBudget, RBudget, RCycles);
  #ifndef LITEV_EXIT_PROTO_STOP
    SetJumpTarget(toStop);
  #endif
    B(stub);
#else
    (void)newPC;
    void* tsPtr = (Num == 0) ? (void*)&NDS.ARM9Timestamp : (void*)&NDS.ARM7Timestamp;

    // (a)
    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, StopExecution));
    FixupBranch toDispatchStop = CBNZ(W2);

#ifdef LITEV_JIT_BUDGET_REG
    // (b)+(c) in the budget register. On the slice end, undo the subtraction and let
    // the dispatcher do the exit (it re-runs (a)-(c) with the canonical state).
    (void)tsPtr;
    SUBS(RBudget, RBudget, RCycles);
    FixupBranch toBudgetEnd = B(CC_LE);
    MOVI2R(RCycles, 0);
#else
    // (b)
    MOVP2R(X4, tsPtr);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, RCycles, ArithOption(RCycles));      // x5 += (u32)RCycles
    STR(INDEX_UNSIGNED, X5, X4, 0);
    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    SUB(W1, W1, RCycles);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    MOVI2R(RCycles, 0);

    // (c)
    CMP(W1, 0);
    FixupBranch toDispatchBudget = B(CC_LE);
#endif

    // (d)
#ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
#endif
    // LAZYFLAGS: ARM::CPSR already canonical in memory at the linkable exit; W27 = guest r7.
#ifdef LITEV_JIT_ICACHE
    MOVI2R(W9, 0);   // a linkable exit opts out of the per-site cache (0 = no cache)
#endif
#ifdef LITEV_JIT_RAS
    if (RasOn)
        MOVI2R(W10, 0);
#endif
    u32 patchOffset = (u32)((u8*)GetRXPtr() - GetRXBase());
    B(DispatcherEntry[Num]);   // the patch slot (unlinked state)

#ifdef LITEV_JIT_BUDGET_REG
    SetJumpTarget(toBudgetEnd);
    ADD(RBudget, RBudget, RCycles);
    SetJumpTarget(toDispatchStop);
#else
    SetJumpTarget(toDispatchStop);
    SetJumpTarget(toDispatchBudget);
#endif
    B(DispatcherEntry[Num]);
#endif  // LITEV_JIT_EXIT_PROTO

    if (NumLinkExits < MaxOutgoingLinks)
    {
        LinkExits[NumLinkExits].PatchOffset = patchOffset;
        LinkExits[NumLinkExits].TargetAddr = targetAddr;
        NumLinkExits++;
    }
    LITE_PROFILE_ADD(melonDS::LiteProfile::g_Frame.LinkSitesEmitted);
}

void Compiler::PatchLinkSite(u32 rxOffset, u32 targetRxOffset)
{
#ifdef LITEV_JIT_SHARED_CODE
    if (Shared)
    {
        // (rxOffset: the site's index in this console's link table)
        SharedLinks[rxOffset] = targetRxOffset == SharedUnlinked ? 0 : (u64)(GetRXBase() + targetRxOffset);
        return;
    }
#endif
    ptrdiff_t delta = (ptrdiff_t)targetRxOffset - (ptrdiff_t)rxOffset;
    // A64 unconditional B reaches +-128MB; the whole block cache is a few MB.
    assert(delta >= -(1 << 27) && delta < (1 << 27));
    u32 instr = 0x14000000u | ((u32)((delta >> 2)) & 0x03FFFFFFu);

#ifdef LITEV_JIT_BGEMIT
    u8* rwbase = BgLayout ? RWBase : GetWriteableRWPtr() - GetCodeOffset();   // (the cursor may be moving)
#else
    u8* rwbase = GetWriteableRWPtr() - GetCodeOffset();   // m_rwbase
#endif
    *(u32*)(rwbase + rxOffset) = instr;

    u8* rxptr = GetRXBase() + rxOffset;
#ifdef LITEV_JIT_LINK_FLUSHBATCH
    if (PatchBatchDepth)
    {
        if (NumPatchAddrs == MaxPatchAddrs)
            FlushPatchBatch();
        PatchAddrs[NumPatchAddrs++] = rxptr;
        return;
    }
#endif
    __builtin___clear_cache((char*)rxptr, (char*)rxptr + 4);
}
#endif

#ifdef LITEV_JIT_LINK_FLUSHBATCH
// One cache-maintenance sequence for all the link sites a link/unlink patched: a line clean
// and invalidate per site, but the barriers (DSB, DSB, ISB) once, not per 4-byte patch.
// Nothing runs the patched code before the batch ends (the emulator thread is in C++).
void Compiler::FlushPatchBatch()
{
    if (!NumPatchAddrs)
        return;
#if defined(__aarch64__) && !defined(__APPLE__) && !defined(_WIN32)
    u64 ctr;
    __asm__ volatile("mrs %0, ctr_el0" : "=r"(ctr));
    const u64 dsize = 4 << ((ctr >> 16) & 0xf), isize = 4 << (ctr & 0xf);
    if (!((ctr >> 28) & 1))   // IDC clear: clean the data lines
        for (int i = 0; i < NumPatchAddrs; i++)
            __asm__ volatile("dc civac, %0" : : "r"((u64)PatchAddrs[i] & ~(dsize - 1)) : "memory");
    __asm__ volatile("dsb ish" : : : "memory");
    if (!((ctr >> 29) & 1))   // DIC clear: invalidate the instruction lines
    {
        for (int i = 0; i < NumPatchAddrs; i++)
            __asm__ volatile("ic ivau, %0" : : "r"((u64)PatchAddrs[i] & ~(isize - 1)) : "memory");
        __asm__ volatile("dsb ish" : : : "memory");
    }
    __asm__ volatile("isb" : : : "memory");
#else
    for (int i = 0; i < NumPatchAddrs; i++)
        __builtin___clear_cache((char*)PatchAddrs[i], (char*)PatchAddrs[i] + 4);
#endif
    NumPatchAddrs = 0;
}
#endif

#ifdef LITEV_JIT_DIRECTPATCH
void Compiler::DirectPatchWriteBranch(u32 rxOffset, u32 targetRxOffset)
{
    ptrdiff_t delta = (ptrdiff_t)targetRxOffset - (ptrdiff_t)rxOffset;
    // A64 unconditional B reaches +-128MB; a code slice is 32MB, so exit-site->stub,
    // stub->target and stub->dispatcher are all trivially in range.
    assert(delta >= -(1 << 27) && delta < (1 << 27));
    u32 instr = 0x14000000u | ((u32)((delta >> 2)) & 0x03FFFFFFu);

#ifdef LITEV_JIT_BGEMIT
    u8* rwbase = BgLayout ? RWBase : GetWriteableRWPtr() - GetCodeOffset();   // (the cursor may be moving)
#else
    u8* rwbase = GetWriteableRWPtr() - GetCodeOffset();   // m_rwbase
#endif
    *(u32*)(rwbase + rxOffset) = instr;

    u8* rxptr = GetRXBase() + rxOffset;
    __builtin___clear_cache((char*)rxptr, (char*)rxptr + 4);
}

// Promote a monomorphic exit site. Emits a per-site GUARD STUB and rewrites the exit
// `B dispatcher` to it. The stub reproduces the dispatcher's per-hop commit (a-d) EXACTLY
// (same ops/order -> identical Timestamp/budget/event schedule -> trace bit-exact), then
// a single guard `instrAddr == guestTarget`:
//   match  -> a DIRECT (statically-predicted) B into hostEntry -- skips the 2-way
//             epoch/key load chain AND the shared, mispredicted indirect BR;
//   miss / StopExecution / slice-end -> fall to the dispatcher, which re-commits as a
//             no-op (RCycles was zeroed by the stub) and resolves via ICACHE exactly as
//             before. W9 = site is still live at stub entry (the exit site set it and the
//             stub's (a-d) never touch W9), so the dispatcher gets it for its bookkeeping.
// Runs on the emu thread only (single JIT thread); the JitEnableWrite/Execute bracket
// keeps the whole write window in C++ text (Apple W^X-safe), a no-op on Android RWX.
void Compiler::DirectPatchPromote(u32 num, u32 site, u32 guestTarget, u64 hostEntry)
{
    if (site == 0 || site >= ICacheSites || !ICacheTable[num])
        return;
    ICacheEntry& e = ICacheTable[num][site];
    if (e.promoted) return;                              // idempotent
    if (e.patchOff == 0) { e.hitCount = 0; return; }     // no exit-site recorded
    // Never emit past the main region (CompileBlock resets at <16KB; keep a stub's
    // headroom). Back off (reset streak, retry later) rather than promote when tight.
    // One fixed-size stub slot per site, reused by every later promotion of it. Appending a new
    // stub per promotion leaked: an invalidation reverts every promotion and the sites promote
    // again, so the stubs filled the code memory (Sims 3: 20 of 29 MB by 3.4 min) and reset the
    // block cache at a moment that depended on this host history (a savestate starts it over).
    constexpr u32 kStubBytes = 128;
    const bool reuse = e.stubOff != 0;
#ifdef LITEV_JIT_BGEMIT
    // its own arena (the block stream belongs to the emitter, which may be another thread)
    // host-only: when the emitter is busy, promote later
    std::unique_lock<std::recursive_mutex> lk(EmitLock, std::try_to_lock);
    if (!lk.owns_lock()) { e.hitCount = 0; return; }
    if (BgLayout && !reuse && DPArenaEnd - DPCursor < (ptrdiff_t)kStubBytes) { e.hitCount = 0; return; }
#endif
    const ptrdiff_t high = GetCodeOffset();
    if (!reuse && (ptrdiff_t)JitMemMainSize - high < 4096
#ifdef LITEV_JIT_BGEMIT
        && !BgLayout
#endif
        ) { e.hitCount = 0; return; }

    NDS.JIT.JitEnableWrite();

    if (reuse)
        SetCodePtrUnsafe(e.stubOff);
    else
    {
#ifdef LITEV_JIT_BGEMIT
        if (BgLayout)
            SetCodePtrUnsafe(DPCursor);
#endif
        AlignCode16();
    }
    u32 stubOff = (u32)((u8*)GetRXPtr() - GetRXBase());

    void* dispatcher = DispatcherEntry[num];
    void* tsPtr = (num == 0) ? (void*)&NDS.ARM9Timestamp : (void*)&NDS.ARM7Timestamp;

#ifdef LITEV_EXIT_PROTO_STOP
    // (a)-(c) as one budget compare: a stop the dispatcher would exit on has already
    // zeroed the budget (ARM::JitStopToBudget). The cold path below adds block K back, so
    // the dispatcher still sees it uncommitted and checks the stop first.
    (void)tsPtr;
    SUBS(RBudget, RBudget, RCycles);
    FixupBranch toCold = B(CC_LE);
    MOVI2R(RCycles, 0);
#else
    // (a) StopExecution -> dispatcher (bounce to C++, block K's Cycles intact).
    LDR(INDEX_UNSIGNED, W2, RCPU, offsetof(ARM, StopExecution));
    FixupBranch toDispStop = CBNZ(W2);

#ifdef LITEV_JIT_BUDGET_REG
    // (b)+(c) in the budget register; on the slice end the dispatcher (a)-(c) re-runs as a
    // no-op subtraction (RCycles = 0) and exits through its spill tail.
    (void)tsPtr;
    SUBS(RBudget, RBudget, RCycles);
    MOVI2R(RCycles, 0);
    FixupBranch toDispBudget = B(CC_LE);
#else
    // (b) commit block K: Timestamp += Cycles; budget -= Cycles; Cycles = 0.
    MOVP2R(X4, tsPtr);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, RCycles, ArithOption(RCycles));
    STR(INDEX_UNSIGNED, X5, X4, 0);
    LDR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    SUB(W1, W1, RCycles);
    STR(INDEX_UNSIGNED, W1, RCPU, offsetof(ARM, CyclesBudget));
    MOVI2R(RCycles, 0);

    // (c) slice over? budget <= 0 -> dispatcher (re-commit no-op, then exits).
    CMP(W1, 0);
    FixupBranch toDispBudget = B(CC_LE);
#endif
#endif  // LITEV_EXIT_PROTO_STOP

    // (d) instrAddr = R[15] - ((CPSR&0x20)?2:4)  (== dispatcher step (d), byte for byte).
    LDR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
#ifdef LITEV_JIT_LAZYFLAGS
    LDR(INDEX_UNSIGNED, W3, RCPU, offsetof(ARM, CPSR));
    UBFX(W3, W3, 5, 1);
#else
    UBFX(W3, RCPSR, 5, 1);
#endif
    SUB(W0, W0, 4);
    ADD(W0, W0, W3, ArithOption(W3, ST_LSL, 1));         // W0 = instrAddr

    // guard: does the ACTUAL guest target still equal the promoted target?
    MOVI2R(W3, guestTarget);
    CMP(W0, W3);
    FixupBranch guardMiss = B(CC_NEQ);

    // hit: under LAZYFLAGS ARM::CPSR is already canonical (dispatcher step (g) no-op);
    // otherwise commit the live RCPSR exactly as the plain hit path does.
#ifndef LITEV_JIT_LAZYFLAGS
    STR(INDEX_UNSIGNED, RCPSR, RCPU, offsetof(ARM, CPSR));
#endif
#if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DirectGuardHits);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
#endif
    B((const void*)hostEntry);            // DIRECT, statically-predicted hop into the target

    // guard miss: the guest target changed -> count (PROFILE only), fall to the
    // dispatcher, whose writeback path will DEMOTE this site. X4/X5 are dispatcher
    // scratch; W9 = site stays live (the counter bump must not touch it). The stop /
    // budget exits land PAST the counter (they are not guard misses).
    SetJumpTarget(guardMiss);
#if LITEV_PROFILE
    MOVP2R(X4, (void*)&melonDS::LiteProfile::g_Frame.DirectGuardMisses);
    LDR(INDEX_UNSIGNED, X5, X4, 0);
    ADD(X5, X5, 1);
    STR(INDEX_UNSIGNED, X5, X4, 0);
#endif
#ifdef LITEV_EXIT_PROTO_STOP
    B(dispatcher);
    SetJumpTarget(toCold);
    ADD(RBudget, RBudget, RCycles);
#else
    SetJumpTarget(toDispStop);
    SetJumpTarget(toDispBudget);
#endif
    B(dispatcher);

    assert(GetCodeOffset() - (ptrdiff_t)stubOff <= (ptrdiff_t)kStubBytes);
    FlushIcacheSection(GetRXBase() + stubOff, GetRXBase() + stubOff + kStubBytes);
#ifdef LITEV_JIT_PERFMAP
    if (litev_perfmap::g_state == 1)   // armed: DirectPatch guard stubs (outside the block's range)
        litev_perfmap::Add(num == 0 ? "jit_dpstub_a9" : "jit_dpstub_a7", GetRXBase() + stubOff, GetRXPtr());
#endif
#ifdef LITEV_JIT_BGEMIT
    if (BgLayout && !reuse)
        DPCursor = (ptrdiff_t)stubOff + kStubBytes;
    if (BgLayout)
        SetCodePtrUnsafe(high);
    else
#endif
    SetCodePtrUnsafe(reuse ? high : (ptrdiff_t)stubOff + kStubBytes);

    // --- patch the exit-site B -> the stub, then flip back to executable ---
    DirectPatchWriteBranch(e.patchOff, stubOff);

    NDS.JIT.JitEnableExecute();

    e.stubOff = stubOff;
    e.promoted = 1;
#ifdef LITEV_JIT_DP_TARGETED
    if (!((e._dp0 & 2) && (u32)e._dp1 == (guestTarget & ~1u)))
    {
        e._dp0 |= 2; e._dp1 = guestTarget & ~1u;
        DPByTarget[num].emplace(guestTarget & ~1u, site);
    }
#endif
#ifdef LITEV_JIT_DP_REVERTLIST
    // (bit 0 = listed; with DP_TARGETED bit 1 = indexed by target: RevertAll must still find every site)
    if (!(e._dp0 & 1)) { e._dp0 |= 1; DirectPatchListed[num].push_back(site); }
#endif
#if LITEV_PROFILE
    melonDS::LiteProfile::g_Frame.DirectPromotions.fetch_add(1, std::memory_order_relaxed);
#endif
}

// Demote a promoted site: revert its exit `B stub` back to `B dispatcher`. Called from
// the dispatcher writeback when a promoted site re-resolves (its guard permanently
// missed). The leaked stub is reclaimed at the next ResetBlockCache.
void Compiler::DirectPatchDemote(u32 num, u32 site)
{
    if (site == 0 || site >= ICacheSites || !ICacheTable[num]) return;
    ICacheEntry& e = ICacheTable[num][site];
    if (!e.promoted) return;

    NDS.JIT.JitEnableWrite();
    u32 dispOff = (u32)((u8*)DispatcherEntry[num] - GetRXBase());
    DirectPatchWriteBranch(e.patchOff, dispOff);
    NDS.JIT.JitEnableExecute();

    e.promoted = 0;
    e.hitCount = 0;
#if LITEV_PROFILE
    melonDS::LiteProfile::g_Frame.DirectDemotions.fetch_add(1, std::memory_order_relaxed);
#endif
}

// Revert EVERY live promotion back to the dispatcher. The sound backstop for block
// invalidation: RX is append-only within a cache epoch, so a promoted site's direct B
// could otherwise outlive its (retired-but-RX-preserved) target block. Cheap and rare
// (single-block invalidations ~4/3400f); over-reverting is harmless (sites re-promote).
void Compiler::DirectPatchRevertAll()
{
    JitRecompileSame[7]++;   // (EmitLock taken by an invalidation; headless jit_compiles line)
#ifdef LITEV_JIT_BGEMIT
    std::lock_guard<std::recursive_mutex> lk(EmitLock);
#endif
    bool any = false;
    NDS.JIT.JitEnableWrite();
    for (u32 num = 0; num < 2; num++)
    {
        if (!ICacheTable[num]) continue;
        u32 dispOff = (u32)((u8*)DispatcherEntry[num] - GetRXBase());
#ifdef LITEV_JIT_DP_REVERTLIST
        // Every promoted site is listed (Promote lists it, only this clears the list), so
        // this reverts exactly the sites the full scan below would.
        for (u32 s : DirectPatchListed[num])
        {
            ICacheEntry& e = ICacheTable[num][s];
            e._dp0 = 0;
            if (e.promoted)
            {
                DirectPatchWriteBranch(e.patchOff, dispOff);
                e.promoted = 0;
                e.hitCount = 0;
                any = true;
            }
        }
        DirectPatchListed[num].clear();
#ifdef LITEV_JIT_DP_TARGETED
        DPByTarget[num].clear();   // (_dp0 = 0 above unindexed every site)
#endif
        continue;
#endif
        u32 n = ICacheNextSite;   // only [1, ICacheNextSite) were ever handed out
        if (n > ICacheSites) n = ICacheSites;
        for (u32 s = 1; s < n; s++)
        {
            ICacheEntry& e = ICacheTable[num][s];
            if (e.promoted)
            {
                DirectPatchWriteBranch(e.patchOff, dispOff);
                e.promoted = 0;
                e.hitCount = 0;
                any = true;
            }
        }
    }
    NDS.JIT.JitEnableExecute();
#if LITEV_PROFILE
    if (any) melonDS::LiteProfile::g_Frame.DirectReverts.fetch_add(1, std::memory_order_relaxed);
#else
    (void)any;
#endif
}

#ifdef LITEV_JIT_DP_TARGETED
// Revert the promoted sites that jump to a block at one of `addrs` (dead blocks of CPU num): the
// sound part of DirectPatchRevertAll (a direct edge must not outlive its target block). Every other
// promotion stays: its target is alive, and a promoted site behaves as the dispatcher resolve it
// replaced (the guard compares the guest target).
void Compiler::DirectPatchRevertTargets(u32 num, const u32* addrs, int n)
{
    if (!ICacheTable[num] || DPByTarget[num].empty()) return;
    // DPByTarget and ICacheEntry::promoted change only on this (the emulator) thread, so the lookup
    // needs no lock; EmitLock (the background emitter holds it for a whole block) only around a patch.
    // Taking it per invalidation was ~20% of the Golden Sun emu thread on the Mac (mutex wait).
#ifdef LITEV_JIT_BGEMIT
    std::unique_lock<std::recursive_mutex> lk(EmitLock, std::defer_lock);
#endif
    bool write = false;
    const u32 dispOff = (u32)((u8*)DispatcherEntry[num] - GetRXBase());
    for (int k = 0; k < n; k++)
    {
        const u32 a = addrs[k] & ~1u;
        auto r = DPByTarget[num].equal_range(a);
        for (auto it = r.first; it != r.second; ++it)
        {
            ICacheEntry& e = ICacheTable[num][it->second];
            if (!(e._dp0 & 2) || (u32)e._dp1 != a) continue;   // indexed under another target since
            e._dp0 &= ~2ull;
            if (!e.promoted) continue;
            if (!write)
            {
#ifdef LITEV_JIT_BGEMIT
                lk.lock();
#endif
                JitRecompileSame[7]++;
                NDS.JIT.JitEnableWrite(); write = true;
            }
            DirectPatchWriteBranch(e.patchOff, dispOff);
            e.promoted = 0;
            e.hitCount = 0;
        }
        DPByTarget[num].erase(r.first, r.second);
    }
    if (write) NDS.JIT.JitEnableExecute();
}
#endif

static void LiteV_DirectPatchPromote(Compiler* c, u32 num, u32 site, u32 guestTarget, u64 hostEntry)
{
    c->DirectPatchPromote(num, site, guestTarget, hostEntry);
}

static void LiteV_DirectPatchDemote(Compiler* c, u32 num, u32 site)
{
    c->DirectPatchDemote(num, site);
}
#endif

void Compiler::Comp_BranchSpecialBehaviour(bool taken)
{
#ifdef LITEV_JIT_IDLE2
    if (taken && (CurInstr.BranchFlags & branch_Idle2Cand) && Num == 0)
    {
        // Candidate wait-loop back-edge: while the site's enable byte is set, exit to C++
        // with IdleLoop = 2 (ARMv5::Idle2Handle proves/skips the loop), exactly like the
        // classic idle branch below.
#ifdef LITEV_JIT_BGEMIT
        // registered when the block was captured (program order, like the base build's emission)
        u8* en;
        if (CurJob)
        {
            JitEmitJob::Snap* sn = EmitSnap(JitEmitJob::SnapIdle2, CurIdx);
            if (!sn) EmitSnapMissing("idle2", CurInstr.Addr);
            en = sn->P;
        }
        else
        {
            en = ((ARMv5*)CurCPU)->Idle2EnableByte(CurInstr.Addr | (Thumb ? 1 : 0));
            if (VerifyJob)
            {
                JitEmitJob::Snap* sn = VerifyJob->Find(JitEmitJob::SnapIdle2, CurIdx);
                if (!sn || sn->P != en) VerifyFail("idle2", CurInstr.Addr, 0, 0);
                sn->Used = 1;
            }
        }
        if (en)
#else
        if (u8* en = ((ARMv5*)CurCPU)->Idle2EnableByte(CurInstr.Addr | (Thumb ? 1 : 0)))
#endif
        {
#ifdef LITEV_JIT_SHARED_CODE
            if (Shared)
            {
                // this console's enable byte for the site: same index on every console that shares the code
                const u32 idx = (u32)(en - (CurJob ? CurJob->Cpu : CurCPU)->JitIdle2Enable);
                LDR(INDEX_UNSIGNED, X16, RCPU, offsetof(ARM, JitIdle2Enable));
                if (idx > 0xFFF)
                {
                    ADD(X16, X16, idx >> 12, true);
                    LDRB(INDEX_UNSIGNED, W17, X16, idx & 0xFFF);
                }
                else
                    LDRB(INDEX_UNSIGNED, W17, X16, idx);
            }
            else
#endif
            {
            MOVP2R(X16, en);
            LDRB(INDEX_UNSIGNED, W17, X16, 0);
            }
            FixupBranch off = CBZ(W17);
            MOVI2R(W17, CurInstr.Addr | (Thumb ? 1 : 0));
            STR(INDEX_UNSIGNED, W17, RCPU, offsetof(ARM, Idle2Site));
            MOVI2R(W0, 2);
            STRB(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, IdleLoop));
#ifdef LITEV_JIT_DISPATCH
#ifdef LITEV_JIT_BUDGET_REG
            SXTW(X17, RBudget);
            LDR(INDEX_UNSIGNED, X16, RCPU, offsetof(ARM, JitTsBase));
            SUB(X16, X16, X17);
            STR(INDEX_UNSIGNED, X16, RCPU, offsetof(ARM, JitTsBase));
            MOVI2R(RBudget, 0);
#endif
            STR(INDEX_UNSIGNED, WZR, RCPU, offsetof(ARM, CyclesBudget));
#endif
            SetJumpTarget(off);
        }
    }
#endif
    if (taken && CurInstr.BranchFlags & branch_IdleBranch)
    {
        MOVI2R(W0, 1);
        STRB(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, IdleLoop));
#ifdef LITEV_JIT_DISPATCH
        // The dispatcher checks the budget, not IdleLoop; zero it so the exit bounces
        // to C++ (which performs the idle-skip). Keeps the ForceExecutionExit invariant:
        // every setter of StopExecution/Halted/IdleLoop also zeroes CyclesBudget.
#ifdef LITEV_JIT_BUDGET_REG
        // Register form of ForceExecutionExit: re-base JitTsBase so Timestamp stays the
        // block-start time with a zero budget.
        SXTW(X17, RBudget);
        LDR(INDEX_UNSIGNED, X16, RCPU, offsetof(ARM, JitTsBase));
        SUB(X16, X16, X17);
        STR(INDEX_UNSIGNED, X16, RCPU, offsetof(ARM, JitTsBase));
        MOVI2R(RBudget, 0);
#endif
        STR(INDEX_UNSIGNED, WZR, RCPU, offsetof(ARM, CyclesBudget));
#endif
    }

    if ((CurInstr.BranchFlags & branch_FollowCondNotTaken && taken)
        || (CurInstr.BranchFlags & branch_FollowCondTaken && !taken))
    {
        RegCache.PrepareExit();

#ifdef LITEV_JIT_CYCLE_BATCH
        // Mid-block exit: add the pending cycles on this path only; the path that
        // continues in the block keeps them deferred.
        u32 exitCycles = ConstantCycles + PendingCycles;
        for (; exitCycles > 0xFFF; exitCycles -= 0xFFF)
            ADD(RCycles, RCycles, 0xFFF);
        if (exitCycles)
            ADD(RCycles, RCycles, exitCycles);
#else
        if (ConstantCycles)
            ADD(RCycles, RCycles, ConstantCycles);
#endif
#ifdef LITEV_JIT_DISPATCH
  #if defined(LITEV_JIT_LINK) && defined(LITEV_LINK_COND)
        // Conditional-branch edges each have a single compile-time-constant next PC:
        //   taken edge     -> the static branch target (Comp_JumpTo(u32) just ran);
        //   not-taken edge -> the fall-through address after this instruction.
        // A dynamic conditional branch (e.g. conditional BX) leaves HasStaticExit
        // false on the taken edge -> fall back to the dispatcher for that edge.
        if (taken && HasStaticExit)
        {
            EmitLinkExit(StaticExitTarget, StaticExitNewPC);
        }
        else if (!taken)
        {
            EmitLinkExit(CurInstr.Addr + (Thumb ? 2 : 4), CurInstr.Addr + (Thumb ? 4 : 8));
        }
        else
        {
            EmitBlockExit();
        }
  #else
        EmitBlockExit();
  #endif
#else
        QuickTailCall(X0, ARM_Ret);
#endif
    }
}

#ifdef LITEV_JIT_R15_ELIDE
// Will Comp_MemAccess take the literal path (Comp_MemLoadLiteral succeeds) for this ARM
// instruction? Mirrors its conditions exactly: LDR/LDRB/LDRH/LDRSB/LDRSH, pre-indexed,
// immediate offset, no writeback, rn = pc, rd != pc, literal not invalidated.
bool Compiler::R15LiteralLoadFolds(CompileFunc comp)
{
    const u32 in = CurInstr.Instr;
    bool wb = comp == &Compiler::A_Comp_MemWB, hd = comp == &Compiler::A_Comp_MemHD;
    if (!(wb || hd) || !NDS.JIT.LiteralOptimizationsEnabled())
        return false;
    if (!(in & (1 << 20)) || ((in >> 16) & 0xF) != 15 || ((in >> 12) & 0xF) == 15
        || !(in & (1 << 24)) || (in & (1 << 21)))
        return false;
    u32 imm;
    if (wb)
    {
        if (in & (1 << 25)) return false;
        imm = in & 0xFFF;
    }
    else
    {
        if (!(in & (1 << 22))) return false;
        imm = (in & 0xF) | ((in >> 4) & 0xF0);
    }
    u32 addr = R15 + imm * ((in & (1 << 23)) ? 1 : -1);
#ifdef LITEV_JIT_BGEMIT
    if (CurJob)
    {
        JitEmitJob::Snap* sn = CurJob->Find(JitEmitJob::SnapLit, CurIdx, addr);
        if (!sn) EmitSnapMissing("r15 literal", addr);
        return sn->State == 1;
    }
#endif
    return NDS.JIT.InvalidLiterals.Find(NDS.JIT.LocaliseCodeAddress(Num, addr)) == -1;
}
#endif

#ifdef LITEV_JIT_WAITLOOP
// headless A/B in one binary: LITEV_JIT_WAITLOOP_OFF=1 compiles the loop as before
static bool WaitLoopOn()
{
    static const bool on = !getenv("LITEV_JIT_WAITLOOP_OFF");
    return on;
}
#endif

#ifdef LITEV_JIT_BGEMIT
// A block's entry stub (16 bytes in the stub arena): B +4 | MOVZ W17,id | MOVK W17,id>>16 | B thunk.
// The first word becomes B <code> once the code exists (the emulator thread patches it, then runs
// it); until then the thunk asks ARMJIT::PendingResolve for the code, waiting for it if needed.
u32 Compiler::WriteEntryStub(u32 jobId, u32 num)
{
    // written for every id at the cache reset (WriteAllEntryStubs): stub k belongs to job k
    (void)num;
    if (StubArenaEnd - StubCursor < 16)
        return 0;
    u32 off = (u32)StubCursor;
    assert(off == StubArenaStart + 16 * jobId);
    StubCursor += 16;
    return off;
}

void Compiler::WriteAllEntryStubs()
{
    u32 thunkOff = (u32)((u8*)PendingThunk[0] - GetRXBase());
    for (u32 off = StubArenaStart, id = 0; off + 16 <= StubArenaEnd; off += 16, id++)
    {
        u32* w = (u32*)(RWBase + off);
        w[0] = 0x14000001u;
        w[1] = 0x52800000u | ((id & 0xFFFF) << 5) | 17;
        w[2] = 0x72A00000u | ((id >> 16) << 5) | 17;
        w[3] = 0x14000000u | ((u32)((s32)(thunkOff - (off + 12)) >> 2) & 0x03FFFFFFu);
    }
    FlushIcacheSection(GetRXBase() + StubArenaStart, GetRXBase() + StubArenaEnd);
}

void Compiler::PatchEntryStub(u32 stubOff, u32 targetOff)
{
    *(u32*)(RWBase + stubOff) = 0x14000000u | ((u32)((s32)(targetOff - stubOff) >> 2) & 0x03FFFFFFu);
    FlushIcacheSection(GetRXBase() + stubOff, GetRXBase() + stubOff + 4);
}

static JitBlockEntry LiteV_PendingResolve(ARMJIT* jit, u32 id)
{
    return jit->PendingResolve(id);
}

// Entered from an entry stub (W17 = job id) wherever a block entry can be: everything a block
// entry may find live is kept (x0-x18, x30, NZCV; the rest is callee-saved).
void* Compiler::Gen_PendingThunk(u32 num)
{
    (void)num;
    AlignCode16();
    void* res = GetRXPtr();
    SUB(SP, SP, 176);
    for (int r = 0; r < 18; r += 2)
        STP(INDEX_SIGNED, (ARM64Reg)(X0 + r), (ARM64Reg)(X0 + r + 1), SP, r * 8);
    STP(INDEX_SIGNED, X18, X30, SP, 144);
    MRS(X0, FIELD_NZCV);
    STR(INDEX_UNSIGNED, X0, SP, 160);
    MOVP2R(X0, &NDS.JIT);
    MOV(W1, W17);
    QuickCallFunction(X2, LiteV_PendingResolve);
    STR(INDEX_UNSIGNED, X0, SP, 128);     // into X16's slot: X16 = the code
    LDR(INDEX_UNSIGNED, X0, SP, 160);
    _MSR(FIELD_NZCV, X0);
    for (int r = 0; r < 18; r += 2)
        LDP(INDEX_SIGNED, (ARM64Reg)(X0 + r), (ARM64Reg)(X0 + r + 1), SP, r * 8);
    LDP(INDEX_SIGNED, X18, X30, SP, 144);
    ADD(SP, SP, 176);
    BR(X16);
    return res;
}
#endif

#ifdef LITEV_JIT_BGEMIT
// Everything Compiler::CompileBlock would read from the live emulator for this block, read now
// (right after the block's first, interpreted run: where the base build emits it), plus the CPU
// field changes that emission makes, made now in the same program order: Comp_JumpTo(u32) on every
// static branch (JumpToCpuEffects) and the literal reads of Comp_MemLoadLiteral. The selection
// below mirrors CompileBlock's dispatch and those comps' conditions; LITEV_JITBG_VERIFY=1 checks it
// against a real emission for every block.
void Compiler::CaptureEmitInputs(ARM* cpu, JitEmitJob& job)
{
    const bool thumb = job.Thumb;
    const u32 num = cpu->Num;
    job.Cpu = cpu;
    job.Num = num;
    job.CPSR = cpu->CPSR;
    job.Ctx = NDS.JIT.Memory.MemCtx();
    job.NumSnaps = 0;
    job.NumA7 = 0;
    const bool litOpt = NDS.JIT.LiteralOptimizationsEnabled();
    u32 farParts = 0;

    auto literal = [&](u32 iaddr, int size, bool sx, u32 addr, u32 r15)
    {
        JitEmitJob::Snap& sn = job.Add(JitEmitJob::SnapLit, iaddr);
        sn.A2 = addr; sn.B = size;
        if (NDS.JIT.InvalidLiterals.Find(NDS.JIT.LocaliseCodeAddress(num, addr)) != -1)
            sn.State = 2;
        else
        {
            sn.State = 1;
            sn.A = LiteralRead(cpu, size, sx, addr, r15);
        }
        return sn.State == 1;
    };
    auto jump = [&](u32 iaddr, u32 target, u32 r15)
    {
        JitEmitJob::Snap& sn = job.Add(JitEmitJob::SnapJump, iaddr);
        sn.A2 = target;
        JumpToCpuEffects(cpu, target, r15, sn.A, sn.B);
    };

    for (int i = 0; i < job.Count; i++)
    {
        const FetchedInstr& in = job.Instrs[i];
        const u32 instr = in.Instr;
        const u32 r15 = in.Addr + (thumb ? 4 : 8);
        if (num == 1)
        {
            int k = 0;
            for (; k < job.NumA7; k++) if (job.A7[k].Row == in.CodeCycles) break;
            if (k == job.NumA7)
            {
                assert(job.NumA7 < 40);
                job.A7[k].Row = in.CodeCycles;
                memcpy(job.A7[k].V, NDS.ARM7MemTimings[in.CodeCycles], 4);
                job.NumA7++;
            }
        }
        CompileFunc comp = thumb ? T_Comp[in.Info.Kind] : A_Comp[in.Info.Kind];
        if (!thumb && in.Info.Kind != ARMInstrInfo::ak_BLX_IMM && (instr >> 28) == 0xF)
            comp = nullptr;   // CompileBlock: Comp_AddCycles_C only
        if (comp == &Compiler::A_Comp_LDM_STM || comp == &Compiler::T_Comp_PUSH_POP || comp == &Compiler::T_Comp_LDMIA_STMIA)
            farParts++;
        if (comp == &Compiler::A_Comp_BranchImm)
        {
            u32 target = r15 + ((s32)(instr << 8) >> 6);
            if ((instr >> 28) == 0xF)
                target += (((instr >> 24) & 1) << 1) + 1;
            jump(i, target, r15);
        }
        else if (comp == &Compiler::T_Comp_BCOND)
            jump(i, r15 + ((s32)(instr << 24) >> 23) + 1, r15);
        else if (comp == &Compiler::T_Comp_B)
            jump(i, r15 + ((s32)((instr & 0x7FF) << 21) >> 20) + 1, r15);
        else if (comp == &Compiler::T_Comp_BL_Merged)
        {
            u32 r = r15 + 2;
            u32 upperPart = instr >> 16;
            u32 target = (r - 2) + ((s32)((instr & 0x7FF) << 21) >> 9);
            target += (upperPart & 0x7FF) << 1;
            if (num == 1 || upperPart & (1 << 12))
                target |= 1;
            jump(i, target, r);
        }
        else if (comp == &Compiler::A_Comp_MemWB || comp == &Compiler::A_Comp_MemHD)
        {
            // Comp_MemAccess: rn = pc, rd != pc, immediate offset, pre-indexed, no writeback, load
            bool wb = comp == &Compiler::A_Comp_MemWB;
            bool imm = wb ? !(instr & (1 << 25)) : (instr & (1 << 22));
            if (litOpt && (instr & (1 << 20)) && ((instr >> 16) & 0xF) == 15 && ((instr >> 12) & 0xF) != 15
                && (instr & (1 << 24)) && !(instr & (1 << 21)) && imm)
            {
                u32 off = wb ? (instr & 0xFFF) : ((instr & 0xF) | ((instr >> 4) & 0xF0));
                u32 addr = r15 + off * ((instr & (1 << 23)) ? 1 : -1);
                int size; bool sx = false;
                if (wb)
                    size = (instr & (1 << 22)) ? 8 : 32;
                else
                {
                    int op = (instr >> 5) & 0x3;
                    sx = op >= 2;
                    size = op == 2 ? 8 : 16;
                }
                literal(i, size, sx, addr, r15);
            }
        }
        else if (comp == &Compiler::T_Comp_LoadPCRel && litOpt)
        {
            u32 offset = (instr & 0xFF) << 2;
            // an invalidated literal: Comp_MemAccess tries R15 + offset itself (the same address
            // unless R15 & 2, which then finds the same capture)
            if (!literal(i, 32, false, (r15 & ~0x2) + offset, r15) && (r15 & 0x2))
                literal(i, 32, false, r15 + offset, r15);
        }
#ifdef LITEV_JIT_IDLE2
        if ((in.BranchFlags & branch_Idle2Cand) && num == 0
            && (thumb ? comp == &Compiler::T_Comp_BCOND : in.Info.Kind != ARMInstrInfo::ak_BLX_IMM && (instr >> 28) != 0xF))
            job.Add(JitEmitJob::SnapIdle2, i).P = ((ARMv5*)cpu)->Idle2EnableByte(in.Addr | (thumb ? 1 : 0));
#endif
    }
    // code bounds (CheckCodeSpace): the block, and the far-region slow path of each LDM/STM
    job.Bound = 1024 + 512 * job.Count;
    job.FarBound = 1536 * farParts;
}

JitEmitJob::Snap* Compiler::EmitSnap(u8 kind, u32 addr)
{
    JitEmitJob::Snap* sn = CurJob->Find(kind, addr);
    return sn;
}

void Compiler::EmitSnapMissing(const char* what, u32 addr)
{
    Log(LogLevel::Error, "LITEV_JIT_BGEMIT: %s at %08x not captured (block %08x)\n", what, addr, CurJob->Instrs[0].Addr);
    fprintf(stderr, "LITEV_JIT_BGEMIT: %s at %08x not captured (block %08x)\n", what, addr, CurJob->Instrs[0].Addr);
    abort();
}

void Compiler::VerifyFail(const char* what, u32 addr, u32 captured, u32 live)
{
    fprintf(stderr, "LITEV_JITBG_VERIFY: %s mismatch at %08x (block %08x num %d): captured %08x, emission %08x\n",
            what, addr, VerifyJob->Instrs[0].Addr, VerifyJob->Num, captured, live);
    abort();
}

u8 Compiler::A7T(u32 row, int k)
{
    JitEmitJob* j = CurJob ? CurJob : VerifyJob;
    if (j)
    {
        for (int i = 0; i < j->NumA7; i++)
            if (j->A7[i].Row == row)
            {
                if (!CurJob && j->A7[i].V[k] != NDS.ARM7MemTimings[row][k])
                    VerifyFail("arm7 timing", row, j->A7[i].V[k], NDS.ARM7MemTimings[row][k]);
                return j->A7[i].V[k];
            }
        if (CurJob) EmitSnapMissing("arm7 timing row", row);
        VerifyFail("arm7 timing row", row, 0, 0);
    }
    return NDS.ARM7MemTimings[row][k];
}
#else
u8 Compiler::A7T(u32 row, int k) { return NDS.ARM7MemTimings[row][k]; }
#endif

JitBlockEntry Compiler::CompileBlock(ARM* cpu, bool thumb, FetchedInstr instrs[], int instrsCount, bool hasMemInstr)
{
    // (the cache-full reset happens in ARMJIT::CompileBlock, see NearlyFull)

    JitBlockEntry res = (JitBlockEntry)GetRXPtr();
#ifdef LITEV_BLOCKPROF
    {
        static bool reg = (BP(), atexit(BPDump), true); (void)reg;
        u32 bpId; { std::lock_guard<std::mutex> lk(BPMu); BPCur = &BP().emplace_back(); bpId = (u32)BP().size() - 1; }
        BPEnt& e = *BPCur;
        e.addr = instrs[0].Addr; e.last = instrs[instrsCount-1].Addr; e.num = cpu->Num; e.thumb = thumb; e.n = instrsCount; e.count = 0; e.idle = 0;
        for (int i = 0; i < instrsCount; i++) { e.instr.push_back(instrs[i].Instr); e.iaddr.push_back(instrs[i].Addr); if (instrs[i].BranchFlags & branch_IdleBranch) e.idle = 1; }
        MOVP2R(X16, &e.count);
        LDR(INDEX_UNSIGNED, X17, X16, 0);
        ADD(X17, X17, 1);
        STR(INDEX_UNSIGNED, X17, X16, 0);
        static const bool bt = getenv("LITEV_BLOCKTRACE") != nullptr;
        if (bt)
        {
            u32 id = bpId;
            STP(INDEX_PRE, X0, X1, SP, -16);
            uint8_t** btc = BTCursor(&NDS);
            { std::lock_guard<std::mutex> lk(BPMu); for (int k = 0; k < 16; k++) if (&BTR[k].cur == btc) e.slot = k; }
            MOVP2R(X16, btc);
            LDR(INDEX_UNSIGNED, X17, X16, 0);
            CNTVCT(X0);
            MOVI2R(W1, id);
            STP(INDEX_SIGNED, X0, X1, X17, 0);
            ADD(X0, X17, 16);
            ANDI2R(X1, X0, ~(2 * BTSize - 1));
            ANDI2R(X0, X0, BTSize - 1);
            ORR(X17, X1, X0);
            STR(INDEX_UNSIGNED, X17, X16, 0);
            LDP(INDEX_POST, X0, X1, SP, 16);
        }
    }
#endif

    Thumb = thumb;
    Num = cpu->Num;
    CurCPU = cpu;
    ConstantCycles = 0;
#ifdef LITEV_JIT_CYCLE_BATCH
    PendingCycles = 0;
    DeferCycles = false;
#endif
#ifdef LITEV_JIT_REGALLOC_USEMASK
    RegCache.Reset(this, instrs, instrsCount, true);
#else
    RegCache = RegisterCache<Compiler, ARM64Reg>(this, instrs, instrsCount, true);
#endif
#ifdef LITEV_JIT_GLOBALREG
    // GLOBALREG: install the fixed guest->host map. These regs are already resident
    // in their host regs (loaded by ARM_Dispatch at slice entry, preserved across
    // block boundaries / the dispatcher / linked chains), so the cache marks them
    // loaded WITHOUT emitting any per-block reload, and never spills them at a
    // block boundary. That removes the block-entry first-load + block-exit
    // writeback traffic for the pinned subset.
    for (int i = 0; i < NumGlobalRegPins; i++)
        RegCache.PinRegister(GlobalRegPins[i].GuestReg, GlobalRegPins[i].HostReg);
#endif
    CPSRDirty = false;
#ifdef LITEV_JIT_FIXEDREG
    NZCVDeferred = 0;
    NZCVCondValid = false;
#endif
#ifdef LITEV_EXIT_PROTO_NZCV
    NZCVHostSynced = false;   // entered from anywhere: dispatcher, C++, links
#endif
#ifdef LITEV_JIT_LAZYFLAGS
    // V3 item 1: precompute the block's per-instruction flag LiveIn (barrier-conservative,
    // block exit all-live). Used by Comp_MaterializeFlags' item-1b upgrade to prove the
    // C,V it would preserve are dead. CompileBlock INTERPRETS the block on first compile
    // elsewhere, but this pass is pure analysis over CurInstr[] and emits nothing.
    Comp_ComputeFlagLiveness(instrs, instrsCount);
    FlagsLiveInCur = 0xF;
    CarryInHostResident = false;
#endif

#ifdef LITEV_JIT_LINK
    NumLinkExits = 0;
    HasStaticExit = false;
    LastInstrCompiledNonBranch = false;
#endif
#ifdef LITEV_JIT_RAS
    RasRetBlock = false;
#endif
#ifdef LITEV_EXIT_PROTO_PC
    PCElided = false;
    BlockStartAddr = instrs[0].Addr;
#endif
#if defined(LITEV_JIT_EXIT_TAIL) || defined(LITEV_JIT_LDR_ALIGNCHK) || defined(LITEV_JIT_COLD_EXITS)
    TailStubs.clear();
#endif
#ifdef LITEV_JIT_COLD_EXITS
    ColdExits.clear();
#endif

    if (hasMemInstr && !JitQOn(jitq_MemBasePin))   // pinned: ARM_Dispatch set x26 for the slice
        MOVP2R(RMemBase, Num == 0 ? NDS.JIT.Memory.FastMem9Start : NDS.JIT.Memory.FastMem7Start);

#if defined(LITEV_JIT_WAITLOOP) && defined(LITEV_JIT_GLOBALREG) && defined(LITEV_JIT_BUDGET_REG) && defined(LITEV_JIT_EXIT_PROTO)
    // ARM7 BIOS WaitByLoop (SWI 3) fast-forward. Based on DSperate's wait-loop fast-forward
    // (src/core/cpu/wait_loop.cpp, src/core/cpu/jit/a64/translate.cpp; https://github.com/beebono/DSperate,
    // GPL-3.0-or-later, f5fa35e). FreeBIOS's loop `subs r0, r0, #1; bgt self` (ARM, 0x115C) compiles to a
    // block of its own. An iteration touches only r0 and the flags and costs the same c cycles, and the
    // next one runs only if r0 stays > 0 and the budget stays > 0 after the hop (EmitLinkExit: subs RBudget,
    // RBudget, RCycles; b.le). So at the head, the k iterations that would certainly continue
    // (k <= r0 - 1 and k*c < RBudget - RCycles) are skipped in one step: r0 -= k, RBudget -= k*c. The block
    // then runs the rest itself, so the state, the cycle count and the slice end are exactly what running
    // them leaves. The flags are dead (the subs rewrites NZCV). The retail BIOS's Thumb loop (0x2F08) is
    // not handled: the headless gates run FreeBIOS only, so its cost could not be checked.
    if (Num == 1 && !Thumb && instrsCount == 2 && WaitLoopOn()
        && instrs[0].Addr == 0x115C && instrs[0].Instr == 0xE2500001 && instrs[1].Instr == 0xCAFFFFFD
        && !(instrs[1].BranchFlags & (branch_FollowCondTaken | branch_FollowCondNotTaken)))
    {
        // c = what the taken path charges, as this compiler emits it: the subs (Comp_AddCycles_C), the
        // refill (Comp_JumpTo, N32 + S32), and the block-final conditional branch's not-taken cycles,
        // which the in-place path adds to ConstantCycles and so charges on both edges (S32).
        const u32 cc = instrs[0].CodeCycles;
        // (A7T: the timing rows LITEV_JIT_BGEMIT captured for this block, as Comp_AddCycles_C/Comp_JumpTo read them)
        const u32 c = A7T(cc, 3) + A7T(instrs[1].Addr >> 15, 2) + A7T(instrs[1].Addr >> 15, 3) + A7T(instrs[1].CodeCycles, 3);
        const ARM64Reg r0 = W19;   // GlobalRegPins: guest r0
        CMP(r0, 2);
        FixupBranch few = B(CC_LT);                 // signed: bgt needs r0 - 1 > 0
        SUB(W16, RBudget, RCycles);
        SUB(W16, W16, 1);                           // what the hops may spend and stay > 0
        MOVI2R(W17, c);
        CMP(W16, W17);
        FixupBranch poor = B(CC_LT);
        UDIV(W16, W16, W17);                        // iterations the budget covers
        SUB(W17, r0, 1);                            // iterations that branch back
        CMP(W16, W17);
        CSEL(W16, W16, W17, CC_LO);                 // k
        SUB(r0, r0, W16);
        MOVI2R(W17, c);
        MSUB(RBudget, W16, W17, RBudget);
        SetJumpTarget(few);
        SetJumpTarget(poor);
    }
#endif

    for (int i = 0; i < instrsCount; i++)
    {
        CurInstr = instrs[i];
        CurIdx = i;
        R15 = CurInstr.Addr + (Thumb ? 4 : 8);
        CodeRegion = R15 >> 24;
#ifdef LITEV_JIT_LAZYFLAGS
        // V3: carry this instruction's flag LiveIn into the flush helpers, and clear the
        // per-instruction native-carry signal (set only by Comp_ReconcileFlags for this
        // instruction's ADC/SBC body).
        FlagsLiveInCur = FlagsLiveIn[i];
        CarryInHostResident = false;
#endif

#ifdef LITEV_JIT_LINK
        // Reset per instruction so a mid-block followed branch's static target does
        // not leak into the block-end exit decision.
        HasStaticExit = false;
#endif

        CompileFunc comp = Thumb
            ? T_Comp[CurInstr.Info.Kind]
            : A_Comp[CurInstr.Info.Kind];
#ifdef LITEV_JIT_CP15_INLINE
        if (!comp && !Thumb && (CurInstr.Info.Kind == ARMInstrInfo::ak_MCR
                || CurInstr.Info.Kind == ARMInstrInfo::ak_MRC) && CP15InlineOK())
            comp = CurInstr.Info.Kind == ARMInstrInfo::ak_MCR
                ? &Compiler::A_Comp_MCR_CacheOp : &Compiler::A_Comp_MRC_DTCM;
#endif

        Exit = i == (instrsCount - 1) || (CurInstr.BranchFlags & branch_FollowCondNotTaken);

        //printf("%x instr %x regs: r%x w%x n%x flags: %x %x %x\n", R15, CurInstr.Instr, CurInstr.Info.SrcRegs, CurInstr.Info.DstRegs, CurInstr.Info.ReadFlags, CurInstr.Info.NotStrictlyNeeded, CurInstr.Info.WriteFlags, CurInstr.SetFlags);

        bool isConditional = Thumb ? CurInstr.Info.Kind == ARMInstrInfo::tk_BCOND : CurInstr.Cond() < 0xE;
#ifdef LITEV_JIT_CYCLE_BATCH
        // RCycles is read by SaveCycles (which flushes first) and at exits (which add the
        // pending cycles on the exit path). Flush up front for interpreter fallbacks and
        // conditional branches that may SaveCycles inside their skippable body (PC-writing ALU/LDM/LDR,
        // which can restore the CPSR). Plain conditional B/BL/BX/BLX and every other
        // conditional instruction leave the pending adds alone (nothing in the body reads
        // RCycles) but keep their own adds inline, since those run on one path only.
        // Unconditional instructions defer; the block-end add then carries everything.
        bool plainBranch = Thumb || CurInstr.Info.Kind == ARMInstrInfo::ak_B
            || CurInstr.Info.Kind == ARMInstrInfo::ak_BL || CurInstr.Info.Kind == ARMInstrInfo::ak_BX
            || CurInstr.Info.Kind == ARMInstrInfo::ak_BLX_REG;
        bool readsCycles = comp == NULL || (isConditional && CurInstr.Info.Branches() && !plainBranch);
        if (readsCycles)
            FlushPendingCycles();
        DeferCycles = !readsCycles && !isConditional;
#endif
        if (comp == NULL || (CurInstr.BranchFlags & branch_FollowCondTaken) || (i == instrsCount - 1 && (!CurInstr.Info.Branches() || isConditional)))
        {
#ifdef LITEV_EXIT_PROTO_PC
            // This R15 is only read by the not-taken edge of a followed-taken branch or the
            // block-end fall-through. Both are link exits for a compiled instruction (see
            // Comp_BranchSpecialBehaviour and the block-end tail), whose cold path stores it.
            bool fallthroughLinks = comp != NULL
                && ((CurInstr.BranchFlags & branch_FollowCondTaken)
                    || (i == instrsCount - 1 && !CurInstr.Info.Branches()))
                && ExitPCElidable(CurInstr.Addr + (Thumb ? 2 : 4));
            if (fallthroughLinks)
                PCElided = true;
            else
#endif
            {
                MOVI2R(W0, R15);
                STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, R[15]));
            }
            if (comp == NULL)
            {
                MOVI2R(W0, CurInstr.Instr);
                STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CurInstr));
            }
            // ARMv5::CodeCycles is read only by interpreter instructions (AddCycles_*), and
            // every C++ path that runs one sets it first (CodeRead32 / the compile pass);
            // JIT code never reads it. So only an interpreter fallback here needs the store.
            if (Num == 0 && (comp == NULL || !JitQOn(jitq_CodeCyclesDead)))
            {
                MOVI2R(W0, (s32)CurInstr.CodeCycles);
                STR(INDEX_UNSIGNED, W0, RCPU, offsetof(ARM, CodeCycles));
            }
        }

        if (comp == NULL)
        {
            SaveCycles();
            SaveCPSR();
            RegCache.Flush();
#ifdef LITEV_JIT_GLOBALREG
            // GLOBALREG: the interpreter reads/writes the guest register FILE
            // directly, but pinned regs live in host regs and Flush() deliberately
            // did NOT spill them. Spill them to the file now so the interpreter
            // sees the authoritative value; the matching reload runs after the
            // interpreter call (the sole file-coherence point for pinned r0..r7).
            for (int p = 0; p < NumGlobalRegPins; p++)
                SaveReg(GlobalRegPins[p].GuestReg, GlobalRegPins[p].HostReg);
#endif
        }
        else
        {
#ifdef LITEV_JIT_R15_ELIDE
            // An ARM literal load (LDR rd, [pc, #imm]) the literal optimisation will turn into
            // a constant never reads r15: don't materialise it into a host register.
            const u16 srcRegs = instrs[i].Info.SrcRegs;
            RegCache.ElideR15Value = JitQOn(jitq_R15Elide);
            if (RegCache.ElideR15Value && !Thumb && R15LiteralLoadFolds(comp))
                instrs[i].Info.SrcRegs &= ~(1 << 15);
#endif
            RegCache.Prepare(Thumb, i);
#ifdef LITEV_JIT_R15_ELIDE
            instrs[i].Info.SrcRegs = srcRegs;
#endif
        }

        if (Thumb)
        {
#ifdef LITEV_JIT_FIXEDREG
            // Thumb instructions carry no per-instruction main-loop CheckCondition, so
            // reconcile any deferred host flags before the body. Stage 2b: spill ONLY
            // when this body forces it (reads a deferred flag, or clobbers host NZCV
            // without being a full producer); a transparent Thumb body keeps the flags
            // resident in host PSTATE across it.
            Comp_ReconcileFlags();
#endif
            if (comp == NULL)
            {
                MOV(X0, RCPU);
                QuickCallFunction(X1, InterpretTHUMB[CurInstr.Info.Kind]);
            }
            else
            {
                (this->*comp)();
            }
        }
        else
        {
            u32 cond = CurInstr.Cond();
            if (CurInstr.Info.Kind == ARMInstrInfo::ak_BLX_IMM)
            {
#ifdef LITEV_JIT_FIXEDREG
                Comp_MaterializeFlags();
#endif
                if (comp)
                    (this->*comp)();
                else
                {
                    MOV(X0, RCPU);
                    QuickCallFunction(X1, ARMInterpreter::A_BLX_IMM);
                }
            }
            else if (cond == 0xF)
            {
#ifdef LITEV_JIT_FIXEDREG
                Comp_MaterializeFlags();
#endif
                Comp_AddCycles_C();
            }
            else
            {
                IrregularCycles = comp == NULL;

#ifdef LITEV_JIT_COLD_EXITS
                if (cond < 0xE && comp == &Compiler::A_Comp_BranchImm && ColdExitOK())
                {
                    if (CurInstr.BranchFlags & branch_FollowCondNotTaken)
                    {
                        // taken edge = exit, in the tail; then the not-taken edge's compile-time
                        // effects, exactly as the in-place code: the body (Comp_JumpTo) made the
                        // cycles irregular and the not-taken path counts this instruction's cycles
                        DeferColdExit(CheckCondition(cond ^ 1), 0);
                        IrregularCycles = true;
                        Comp_AddCycles_C(true);
                    }
                    else
                    {
                        // not-taken edge = exit, in the tail; the taken edge (the body) runs on
                        FixupBranch notTaken = CheckCondition(cond);
                        (this->*comp)();
                        Comp_BranchSpecialBehaviour(true);
                        if (IrregularCycles)
                            Comp_AddCycles_C(true);
                        DeferColdExit(notTaken, 1);
  #ifdef LITEV_EXIT_PROTO_PC
                        PCElided = false;   // the exit's EmitLinkExit consumes it
  #endif
                    }
                    goto instrDone;
                }
#endif
                FixupBranch skipExecute;
                if (cond < 0xE)
                    skipExecute = CheckCondition(cond);
#ifdef LITEV_JIT_FIXEDREG
                else
                    // Unconditional (AL) body: no CheckCondition ran. Stage 2b: keep
                    // the resident flags alive across the body unless it reads a
                    // deferred flag or clobbers host NZCV without being a full producer.
                    Comp_ReconcileFlags();
#endif

                if (comp == NULL)
                {
                    MOV(X0, RCPU);
                    QuickCallFunction(X1, InterpretARM[CurInstr.Info.Kind]);
                }
                else
                {
                    (this->*comp)();
                }

                Comp_BranchSpecialBehaviour(true);

                if (cond < 0xE)
                {
                    if (IrregularCycles || (CurInstr.BranchFlags & branch_FollowCondTaken))
                    {
                        FixupBranch skipNop = B();
                        SetJumpTarget(skipExecute);

                        if (IrregularCycles)
                            Comp_AddCycles_C(true);

                        Comp_BranchSpecialBehaviour(false);

                        SetJumpTarget(skipNop);
                    }
                    else
                    {
                        SetJumpTarget(skipExecute);
                    }
                }

            }
        }

#ifdef LITEV_JIT_COLD_EXITS
    instrDone:
#endif
        if (comp == NULL)
        {
            LoadCycles();
            LoadCPSR();
#ifdef LITEV_JIT_GLOBALREG
            // GLOBALREG: the interpreter may have written the pinned guest regs in
            // the file (e.g. an ALU/LDM/SWI fallback targeting r0). Reload them into
            // their fixed host regs so the pinned invariant holds for the rest of
            // the block / the chained successor.
            for (int p = 0; p < NumGlobalRegPins; p++)
                LoadReg(GlobalRegPins[p].GuestReg, GlobalRegPins[p].HostReg);
#endif
        }

#ifdef LITEV_JIT_LINK
        // Only a JIT-compiled non-branch has a statically-known single next PC (an
        // interpreter fallback might mutate R15). Captured for the block-end tail.
        LastInstrCompiledNonBranch = (comp != NULL) && !CurInstr.Info.Branches();
        LastInstrFallthroughAddr = CurInstr.Addr + (Thumb ? 2 : 4);
#endif
#ifdef LITEV_JIT_LAZYFLAGS
        // CORRECTNESS FIX (carry/overflow deferral is unsafe): never carry the guest C/V
        // flags deferred-resident in the host PSTATE across an instruction boundary.
        // The deferral tracking failed to materialize host C,V before a later host op
        // clobbered them, so a subsequent guest conditional read stale C/V and took the
        // WRONG branch — e.g. Pokémon White's intro mis-branched in its 3D geometry
        // submission and dropped the professor's polygons (RenderNumPolygons 23->0 =>
        // solid-white professor). Shrek never hit the pattern, and the CPU/RAM state
        // re-converged, so both the Shrek gate and the state-trace oracle missed it;
        // only a framebuffer diff vs the interpreter catches it.
        //
        // Fix: materialize eagerly whenever C or V is deferred (flushes the arithmetic
        // producer's NZCV to the canonical JitNZCV slot now). The common logical-producer
        // N/Z-only deferral is retained, so most of the lazy-flags win is kept. Validated
        // byte-identical to the interpreter on Pokémon White (professor renders) and Shrek
        // (no regression). The slot structure and the r7->W27 global pin are untouched.
#ifdef LITEV_EXIT_PROTO_NZCV
        LastFlushFull = false;
#endif
        if (NZCVDeferred & 0x3)
            Comp_MaterializeFlags();
#ifdef LITEV_EXIT_PROTO_NZCV
        // Host NZCV equals the slot after this instruction if it just ended with a full
        // MRS+STR flush (the deferral contract already requires nothing between the producer
        // and that flush to touch host NZCV), or if it already did before and this
        // instruction is an unconditional, compiled, non-branching body that the Stage 2b
        // classifier proves never writes host NZCV nor the slot.
        {
            bool uncond = Thumb ? CurInstr.Info.Kind != ARMInstrInfo::tk_BCOND : CurInstr.Cond() == 0xE;
            bool neutral = comp != NULL && uncond && !CurInstr.Info.Branches()
                && Comp_BodyIsNZCVTransparent(CurInstr.Info.Kind, CurInstr.Info.WriteFlags, CurInstr.Info.ReadFlags);
            NZCVHostSynced = LastFlushFull || (NZCVHostSynced && neutral);
        }
#endif
#endif
    }

    RegCache.Flush();

#ifdef LITEV_JIT_LAZYFLAGS
    // V3: block boundary == all-live (slot must be fully canonical for ARM_Ret /
    // dispatcher). Force FlagsLiveInCur=0xF so the block-end flush NEVER takes the
    // item-1b C,V-clobber upgrade.
    FlagsLiveInCur = 0xF;
#endif
#ifdef LITEV_JIT_FIXEDREG
    // Block ends with the last instruction's flags possibly still resident in host
    // NZCV (e.g. a trailing unconditional CMP). The dispatcher / ARM_Ret stores the
    // live RCPSR at runtime, so reconcile now.
    Comp_MaterializeFlags();
#endif

#ifdef LITEV_JIT_CYCLE_BATCH
    DeferCycles = false;
    if (ConstantCycles + PendingCycles <= 0xFFF)
    {
        ConstantCycles += PendingCycles;   // both are added at this one point
        PendingCycles = 0;
    }
    else
        FlushPendingCycles();
#endif
    if (ConstantCycles)
        ADD(RCycles, RCycles, ConstantCycles);
#ifdef LITEV_JIT_DISPATCH
  #ifdef LITEV_JIT_LINK
    // Block-end exit eligibility:
    //   - an UNCONDITIONAL static branch (Comp_JumpTo(u32), not a BCOND) has one
    //     target and no converging not-taken path here -> LINK_UNCOND;
    //   - a JIT-compiled non-branch falls through to a single known PC -> LINK_FALLTHROUGH.
    // A conditional branch at block end converges taken+not-taken onto this one
    // exit (two possible PCs) and a dynamic branch has no compile-time target;
    // both keep the plain dispatcher exit.
    bool linked = false;
    if (HasStaticExit && !StaticExitCond)
    {
    #ifdef LITEV_LINK_UNCOND
        EmitLinkExit(StaticExitTarget, StaticExitNewPC);
        linked = true;
    #endif
    }
    else if (!HasStaticExit && LastInstrCompiledNonBranch)
    {
    #ifdef LITEV_LINK_FALLTHROUGH
        EmitLinkExit(LastInstrFallthroughAddr, LastInstrFallthroughAddr + (Thumb ? 2 : 4));
        linked = true;
    #endif
    }
    if (!linked)
    {
        EmitBlockExit();
    }
  #else
    EmitBlockExit();
  #endif
#else
    QuickTailCall(X0, ARM_Ret);
#endif

#if defined(LITEV_JIT_EXIT_TAIL) || defined(LITEV_JIT_LDR_ALIGNCHK) || defined(LITEV_JIT_COLD_EXITS)
    EmitTailStubs();
#endif
#ifdef LITEV_BLOCKPROF
    BPCur->host.assign((const uint8_t*)res, (const uint8_t*)GetRXPtr());
#endif
#ifdef LITEV_JIT_COMPILE_STATS
    const u64 flushT0 = JitTicks();
#endif
    FlushIcache();
#ifdef LITEV_JIT_COMPILE_STATS
    JitCompileTicks[JitPhase_Flush] += JitTicks() - flushT0;
#endif


#ifdef LITEV_JIT_PERFMAP
    if (litev_perfmap::g_state == 1)   // armed: name the block (formatting it costs ~2% of a compile)
    {
        char nm[24];
        snprintf(nm, sizeof(nm), "jit_a%d_%x", Num == 0 ? 9 : 7, (unsigned)instrs[0].Addr);
        litev_perfmap::Add(nm, (void*)res, GetRXPtr());
    }
    // LITEV_JIT_DUMP=<file> (with the perf map armed): every block's guest range and host code bytes,
    // appended: u32 cpu, u32 guest addr, u32 guest instr count, u32 thumb, u32 host bytes, bytes
    {
        static FILE* df = [] { const char* f = getenv("LITEV_JIT_DUMP"); return f ? fopen(f, "ab") : nullptr; }();
        if (df && litev_perfmap::g_state == 1)
        {
            static std::mutex dl;
            std::lock_guard<std::mutex> lk(dl);
            const u8* a = (const u8*)res; const u8* b = (const u8*)GetRXPtr();
            u32 h[5] = { (u32)Num, (u32)instrs[0].Addr, (u32)instrsCount, (u32)thumb, (u32)(b - a) };
            fwrite(h, 4, 5, df); fwrite(a, 1, b - a, df); fflush(df);
        }
    }
#endif

    return res;
}

bool Compiler::NearlyFull()
{
    // Host room (as before): a block needs at most a few KB near and far. (Background emission:
    // the stream is the emitter's, ARMJIT::CheckCodeSpace checks it with the queued blocks.)
    if (
#ifdef LITEV_JIT_BGEMIT
        !BgLayout &&
#endif
#ifdef LITEV_JIT_SHARED_CODE
        !Shared &&   // (the arena's room is not this console's: SharedCompile checks it)
#endif
        (JitMemMainSize - GetCodeOffset() < 1024 * 16 || (JitMemMainSize + JitMemSecondarySize) - OtherCodeRegion < 1024 * 8))
    {
        Log(LogLevel::Debug, "JIT memory full, resetting...\n");
        return true;
    }
    // The deterministic limit (LogicalSize: ~1.6x the real near code on Shrek; the near region
    // also holds up to 4 MB of DIRECTPATCH stub slots and, after a savestate load, the blocks
    // compiled from its profiles), reached well before the host one. ponytail: a game whose
    // blocks run larger than the estimate on average could still reach the host check first
    // (a reset at a host-dependent moment); raise the per-instruction estimate if one does.
#ifdef LITEV_JIT_LIMIT
    return LogicalSize > LogicalLimit;
#else
    return LogicalSize > (u64)JitMemMainSize - 6 * 1024 * 1024;
#endif
}

#ifdef LITEV_JIT_ICACHE
void Compiler::ICacheAllocOnce()
{
    for (int c = 0; c < 2; c++)
        if (!ICacheTable[c])
        {
#ifdef LITEV_JIT_SHARED_CODE
            if (Shared)   // process-wide site ids: a bigger table, touched only where sites run
            {
                void* m = mmap(nullptr, (size_t)SharedICacheSites * sizeof(ICacheEntry), PROT_READ | PROT_WRITE,
                               MAP_PRIVATE | MAP_ANONYMOUS | MAP_NORESERVE, -1, 0);
                if (m == MAP_FAILED) { fprintf(stderr, "LITEV_JIT_SHARED_CODE: no ICACHE table\n"); abort(); }
                ICacheTable[c] = (ICacheEntry*)m;
                continue;
            }
#endif
            ICacheTable[c] = (ICacheEntry*)calloc(ICacheSites, sizeof(ICacheEntry));
        }
}

void Compiler::ICacheReset()
{
    // Wholesale block-cache reset: every host block is gone, so every cached
    // (guest-PC -> host-ptr) pair is stale. Zeroing the tables makes key0/key1 == 0
    // (an impossible even instrAddr only if 0 is never a real block start; the epoch
    // guard is the real safety net regardless). Reset the site counter and bump the
    // per-CPU epoch so any in-flight entry (there are none post-reset, but be robust)
    // is rejected. NextSite starts at 1 (0 reserved for "no cache").
#ifdef LITEV_JIT_SHARED_CODE
    // shared: the site ids stay (they are baked into the shared code); only the used part is cleared
    const size_t icSites = Shared ? JitArena().NextSite : ICacheSites;
#else
    const size_t icSites = ICacheSites;
#endif
    for (int c = 0; c < 2; c++)
    {
        if (ICacheTable[c]) memset(ICacheTable[c], 0, icSites << ICShift());
#if defined(LITEV_JIT_DIRECTPATCH) && defined(LITEV_JIT_DP_REVERTLIST)
        DirectPatchListed[c].clear();
#endif
#if defined(LITEV_JIT_DIRECTPATCH) && defined(LITEV_JIT_DP_TARGETED)
        DPByTarget[c].clear();
#endif
    }
    ICacheNextSite = 1;
    NDS.ARM9.ICacheEpoch++;
    NDS.ARM7.ICacheEpoch++;
}
#endif

void Compiler::Reset()
{
    LogicalSize = 0;
#ifdef LITEV_JIT_SHARED_CODE
    if (!Shared)   // (shared: the arena's, for code that stays)
#endif
    LoadStorePatches.clear();
#ifdef LITEV_JIT_STORE_REPROMOTE
    SlowStoreSites.clear();   // the code they point into is gone
    SlowStoreNextRetry = ~0u;
#endif
#ifdef LITEV_JIT_PATCHMAP_RESERVE
    // one entry per compiled memory access: sized up front so compiling never rehashes it
    LoadStorePatches.reserve(1 << 15);
#endif

#ifdef LITEV_JIT_ICACHE
    // Allocate the per-CPU inline-cache tables BEFORE Gen_Dispatcher bakes their base
    // address, then clear them + bump the epoch for this new cache epoch.
    ICacheAllocOnce();
    ICacheReset();
#endif
#ifdef LITEV_JIT_CP15_INLINE
    {
#if defined(__ANDROID__)
        char b[8] = {0}; int n = __system_property_get("debug.litev.cp15inline", b);
        Cp15InlineOn = (n > 0) ? (atoi(b) != 0) : true;
#else
        const char* e = getenv("debug.litev.cp15inline"); Cp15InlineOn = e ? (atoi(e) != 0) : true;
#endif
    }
#endif
#ifdef LITEV_JIT_RAS
    {
#if defined(__ANDROID__)
        char b[8] = {0}; int n = __system_property_get("debug.litev.jitras", b);
        RasOn = (n > 0) ? (atoi(b) != 0) : true;
#else
        const char* e = getenv("debug.litev.jitras"); RasOn = e ? (atoi(e) != 0) : true;
#endif
    }
#endif

    {
        // debug.litev.jitq: JIT code-quality lever bits (see Compiler::jitq_*), default all
        // compiled-in levers on; 0 = the code the build emitted before them
        u32 on = 0
#ifdef LITEV_JIT_EXIT_TAIL
            | jitq_ExitTail
#endif
#ifdef LITEV_JIT_MEMBASE_PIN
            | jitq_MemBasePin
#endif
#ifdef LITEV_JIT_FLAGS_BFXIL
            | jitq_FlagsBfxil
#endif
#ifdef LITEV_JIT_R15_ELIDE
            | jitq_R15Elide
#endif
#ifdef LITEV_JIT_LDR_ALIGNCHK
            | jitq_LdrAlignChk
#endif
#ifdef LITEV_JIT_CODECYCLES_DEAD
            | jitq_CodeCyclesDead
#endif
#ifdef LITEV_JIT_COLD_EXITS
            | jitq_ColdExits
#endif
#ifdef LITEV_JIT_NZ_BRANCH
            | jitq_NZBranch
#endif
#ifdef LITEV_JIT_MOV_ELIDE
            | jitq_MovElide
#endif
            ;
#if defined(__ANDROID__)
        char b[16] = {0}; int n = __system_property_get("debug.litev.jitq", b);
        JitQ = on & ((n > 0) ? (u32)strtoul(b, nullptr, 0) : ~0u);
#else
        const char* e = getenv("debug.litev.jitq"); JitQ = on & (e ? (u32)strtoul(e, nullptr, 0) : ~0u);
#endif
    }

#ifdef LITEV_JIT_PERFMAP
    // Wholesale block-cache reset: truncate the perf map and rewrite the
    // process-lifetime stub regions before the dispatchers/blocks re-populate it.
    litev_perfmap::BeginEpoch();
#endif

#ifdef LITEV_JIT_SHARED_CODE
    if (Shared)
    {
        // the code stays (other consoles run it); the dispatchers are built once, console-independent
        SharedArena& a = JitArena();
        std::lock_guard<std::recursive_mutex> lk(a.Lock);
        if (!a.Dispatchers)
        {
            NDS.JIT.JitEnableWrite();
            SetCodePtr(0);   // (fresh zero memory: UDF traps like the BRK fill)
            a.DispatcherEntry[0] = Gen_Dispatcher(0);
#ifdef LITEV_JIT_PERFMAP
            litev_perfmap::Add("jit_dispatcher9", a.DispatcherEntry[0], GetRXPtr());
#endif
            a.DispatcherEntry[1] = Gen_Dispatcher(1);
#ifdef LITEV_JIT_PERFMAP
            litev_perfmap::Add("jit_dispatcher7", a.DispatcherEntry[1], GetRXPtr());
#endif
            FlushIcache();
            NDS.JIT.JitEnableExecute();
            if (const char* dd = getenv("LITEV_JIT_DUMPDISP"))   // measurement: the dispatchers' code (raw A64)
                if (FILE* f = fopen(dd, "wb")) { fwrite(a.DispatcherEntry[0], 1, (u8*)GetRXPtr() - (u8*)a.DispatcherEntry[0], f); fclose(f); }
            a.Near = GetCodeOffset();
            a.Far = JitMemMainSize;
            a.Dispatchers = true;
        }
        DispatcherEntry[0] = a.DispatcherEntry[0];
        DispatcherEntry[1] = a.DispatcherEntry[1];
        SharedSetCpuFields();
        return;
    }
#endif
    const u32 brk_0 = 0xD4200000;
#ifdef LITEV_JIT_LIMIT
    // the block memory starts zero (fresh / madvised slice; zero = UDF, traps like BRK): refill only
    // what was written, which keeps the untouched rest of the 64 MB slice non-resident.
    // (first Reset: GetCodeOffset() is 0 and OtherCodeRegion not yet set)
    NearHigh = std::max(NearHigh, (ptrdiff_t)GetCodeOffset());
    if (OtherCodeRegion > (ptrdiff_t)JitMemMainSize && OtherCodeRegion <= (ptrdiff_t)(JitMemMainSize + JitMemSecondarySize))
        FarHigh = std::max(FarHigh, OtherCodeRegion);
    SetCodePtr(0);
    OtherCodeRegion = JitMemMainSize;
    std::fill_n((u32*)GetRWPtr(), NearHigh / 4, brk_0);
    if (FarHigh)
        std::fill_n((u32*)GetRWPtr() + JitMemMainSize / 4, (FarHigh - JitMemMainSize) / 4, brk_0);
#else
    SetCodePtr(0);
    OtherCodeRegion = JitMemMainSize;

    for (int i = 0; i < (JitMemMainSize + JitMemSecondarySize) / 4; i++)
        *(((u32*)GetRWPtr()) + i) = brk_0;
#endif

#ifdef LITEV_JIT_DISPATCH
    // Generate the per-CPU dispatchers at the very start of the (now-brk-filled)
    // block-cache region. Doing it here — not in the constructor — means GetRXBase()
    // is already the stable, rebased block-cache base (== the base SubEntryOffset uses),
    // so the RXBase baked into the hit path is always correct. Blocks compiled after
    // this Reset() start just past the dispatcher code. Regenerated on every
    // ResetBlockCache(), which is cheap and keeps the base bit-for-bit consistent.
    SetCodePtr(0);
    DispatcherEntry[0] = Gen_Dispatcher(0);
#ifdef LITEV_JIT_PERFMAP
    litev_perfmap::Add("jit_dispatcher9", DispatcherEntry[0], GetRXPtr());
#endif
    DispatcherEntry[1] = Gen_Dispatcher(1);
#ifdef LITEV_JIT_PERFMAP
    litev_perfmap::Add("jit_dispatcher7", DispatcherEntry[1], GetRXPtr());
#endif
#ifdef LITEV_JIT_BGEMIT
    if (BgLayout)
    {
        PendingThunk[0] = PendingThunk[1] = Gen_PendingThunk(0);
        StubCursor = StubArenaStart;
        DPCursor = DPArenaStart;
        RWBase = (u8*)GetRWPtr() - GetCodeOffset();
        WriteAllEntryStubs();
    }
#endif
    FlushIcache();
#endif
}

void Compiler::Comp_AddCycles_C(bool forceNonConstant)
{
    s32 cycles = Num ?
        A7T(CurInstr.CodeCycles, Thumb ? 1 : 3)
        : ((R15 & 0x2) ? 0 : CurInstr.CodeCycles);

    if (forceNonConstant)
        ConstantCycles += cycles;
#ifdef LITEV_JIT_CYCLE_BATCH
    else if (DeferCycles)
        PendingCycles += cycles;
#endif
    else
        ADD(RCycles, RCycles, cycles);
}

void Compiler::Comp_AddCycles_CI(u32 numI)
{
    IrregularCycles = true;

    s32 cycles = (Num ?
        A7T(CurInstr.CodeCycles, Thumb ? 0 : 2)
        : ((R15 & 0x2) ? 0 : CurInstr.CodeCycles)) + numI;

    if (Thumb || CurInstr.Cond() == 0xE)
        ConstantCycles += cycles;
    else
        ADD(RCycles, RCycles, cycles);
}

void Compiler::Comp_AddCycles_CI(u32 c, ARM64Reg numI, ArithOption shift)
{
    IrregularCycles = true;

    s32 cycles = (Num ?
        A7T(CurInstr.CodeCycles, Thumb ? 0 : 2)
        : ((R15 & 0x2) ? 0 : CurInstr.CodeCycles)) + c;

#ifdef LITEV_JIT_CYCLE_BATCH
    if (DeferCycles)
        PendingCycles += cycles;
    else
#endif
    ADD(RCycles, RCycles, cycles);
    if (Thumb || CurInstr.Cond() >= 0xE)
        ConstantCycles += cycles;
    else
        ADD(RCycles, RCycles, cycles);
}

void Compiler::Comp_AddCycles_CDI()
{
    if (Num == 0)
        Comp_AddCycles_CD();
    else
    {
        IrregularCycles = true;

        s32 cycles;

        s32 numC = A7T(CurInstr.CodeCycles, Thumb ? 0 : 2);
        s32 numD = CurInstr.DataCycles;

        if (LITEV_FT) cycles = numC + numD;   // fast timing: no overlap, no region rules
        else if ((CurInstr.DataRegion >> 24) == 0x02) // mainRAM
        {
            if (CodeRegion == 0x02)
                cycles = numC + numD;
            else
            {
                numC++;
                cycles = std::max(numC + numD - 3, std::max(numC, numD));
            }
        }
        else if (CodeRegion == 0x02)
        {
            numD++;
            cycles = std::max(numC + numD - 3, std::max(numC, numD));
        }
        else
        {
            cycles = numC + numD + 1;
        }
        
        if (!Thumb && CurInstr.Cond() < 0xE)
            ADD(RCycles, RCycles, cycles);
        else
            ConstantCycles += cycles;
    }
}

void Compiler::Comp_AddCycles_CD()
{
    u32 cycles = 0;
    if (Num == 0)
    {
        s32 numC = (R15 & 0x2) ? 0 : CurInstr.CodeCycles;
        s32 numD = CurInstr.DataCycles;

        //if (DataRegion != CodeRegion)
            cycles = LITEV_FT ? numC + numD : std::max(numC + numD - 6, std::max(numC, numD));

        IrregularCycles = cycles != numC;
    }
    else
    {
        s32 numC = A7T(CurInstr.CodeCycles, Thumb ? 0 : 2);
        s32 numD = CurInstr.DataCycles;

        if (LITEV_FT) cycles += numC + numD;   // fast timing
        else if ((CurInstr.DataRegion >> 24) == 0x02)
        {
            if (CodeRegion == 0x02)
                cycles += numC + numD;
            else
                cycles += std::max(numC + numD - 3, std::max(numC, numD));
        }
        else if (CodeRegion == 0x02)
        {
            cycles += std::max(numC + numD - 3, std::max(numC, numD));
        }
        else
        {
            cycles += numC + numD;
        }

        IrregularCycles = true;
    }

    if ((!Thumb && CurInstr.Cond() < 0xE) && IrregularCycles)
        ADD(RCycles, RCycles, cycles);
    else
        ConstantCycles += cycles;
}

}
