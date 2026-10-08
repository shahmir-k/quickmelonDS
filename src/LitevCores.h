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

#endif // __ANDROID__ || __linux__

#ifdef LITEV_TOPO_PIN
// ---- Role-based placement (docs/THREAD-PINNING.md in the profiler repo) --------------------
// Code asks for a role, never a core number. Place() maps roles onto whatever cores exist; it
// is pure (no sysfs) so tools/topotest feeds it fake topologies. On the RG DS's 4 equal A55s
// it reproduces the measured placement: emu alone on 3, tile workers one each on 0/1/2, every
// other render/present/netplay thread on 0-2.

#include <cstdlib>
#include <cstring>
#include <string>
#include <thread>
#include <vector>
#include <algorithm>
#include <cstdio>
#ifdef __ANDROID__
#include <sys/system_properties.h>
#endif
#include "Platform.h"

namespace melonDS::LitevTopo
{

enum class CoreRole : int
{
    Emu,            // the serial critical path: the fastest core, nothing else on it
    RenderParallel, // tile band workers: one distinct core each (idx = worker index)
    RenderCritical, // light latency-sensitive (2D, GL/hyb-present, tile coordinator, present);
                    // shared mask over the fast non-emu cores. idx >= 0 = that one core (s2dcpu)
    RemoteConsole,  // Netplay remote-console threads (NetplayRemote): same mask as RenderCritical
    Background,     // everything else
};
inline const char* RoleName(CoreRole r)
{
    static const char* n[] = {"Emu", "RenderParallel", "RenderCritical", "RemoteConsole", "Background"};
    return n[(int)r];
}

enum class CoreType : int { Unknown, InOrder, OoO };

// From MIDR (implementer, part). Missing rows only mean "rank by capacity".
inline CoreType TypeFromId(unsigned impl, unsigned part)
{
    if (impl == 0x41)
    {
        switch (part)
        {
        case 0xd03: case 0xd04: case 0xd05: case 0xd46: case 0xd80:
            return CoreType::InOrder; // A53, A35, A55, A510, A520
        case 0xd07: case 0xd08: case 0xd09: case 0xd0a: case 0xd0b: case 0xd0d: case 0xd41:
        case 0xd4b: case 0xd47: case 0xd4d: case 0xd81: case 0xd87:
        case 0xd44: case 0xd48: case 0xd4e: case 0xd82: case 0xd85:
            return CoreType::OoO;     // A57..A78(C), A710/715/720/725, X1..X4, X925
        }
    }
    if (impl == 0x51 && part >= 0x800 && part <= 0x805)
        return (part & 1) ? CoreType::InOrder : CoreType::OoO; // Kryo 2xx/3xx/4xx Silver / Gold
    return CoreType::Unknown;
}

struct CoreInfo { int id; long capacity; long maxFreq; CoreType type; };

struct Placement
{
    bool pin = false;            // false: every role gets every core (unknown speed, or <= 2 cores)
    int emu = -1;
    std::vector<int> workers;    // tile worker cores, one per worker, fastest first
    int workerCount = 3;         // preferred tile worker count (the renderer rounds to its split)
    std::vector<int> critical;   // RenderCritical / RemoteConsole
    std::vector<int> background;
    std::vector<int> all;
    std::vector<CoreInfo> cores;

    std::vector<int> CoresFor(CoreRole r, int idx = -1) const
    {
        if (!pin) return all;
        switch (r)
        {
        case CoreRole::Emu: return {emu};
        case CoreRole::RenderParallel:
            if (idx >= 0 && !workers.empty()) return {workers[idx % workers.size()]};
            return critical;
        case CoreRole::RenderCritical:
            if (idx >= 0 && std::find(critical.begin(), critical.end(), idx) != critical.end()) return {idx};
            return critical;
        case CoreRole::RemoteConsole: return critical;
        case CoreRole::Background: return background;
        }
        return all;
    }

    std::string Describe() const
    {
        auto list = [](const std::vector<int>& v) {
            std::string s;
            for (size_t i = 0; i < v.size(); i++) s += (i ? "," : "") + std::to_string(v[i]);
            return s.empty() ? std::string("-") : s;
        };
        static const char* tn[] = {"?", "in-order", "OoO"};
        std::string s;
        for (const CoreInfo& c : cores)
            s += "cpu" + std::to_string(c.id) + " cap=" + std::to_string(c.capacity) + " freq=" +
                 std::to_string(c.maxFreq) + " " + tn[(int)c.type] + "\n";
        s += std::string("pin=") + (pin ? "on" : "off (all roles share every core)") + "\n";
        s += "Emu            -> " + (pin ? std::to_string(emu) : list(all)) + "\n";
        s += "RenderParallel -> " + std::to_string(workerCount) + " workers, " + (pin ? "one core each: " + list(workers) : "unpinned: " + list(all)) + "\n";
        s += "RenderCritical -> " + list(CoresFor(CoreRole::RenderCritical)) + "\n";
        s += "RemoteConsole  -> " + list(CoresFor(CoreRole::RemoteConsole)) + "\n";
        s += "Background     -> " + list(CoresFor(CoreRole::Background)) + "\n";
        return s;
    }
};

// cores: every allowed core. Speed = cpu_capacity if every core has one, else cpuinfo_max_freq,
// else no pinning. defWorkers = today's tile worker count (3); maxWorkers = the renderer's cap.
inline Placement Place(std::vector<CoreInfo> cores, int defWorkers = 3, int maxWorkers = 6)
{
    Placement p;
    std::sort(cores.begin(), cores.end(), [](const CoreInfo& a, const CoreInfo& b) { return a.id < b.id; });
    p.cores = cores;
    for (const CoreInfo& c : cores) p.all.push_back(c.id);
    const int n = (int)cores.size();
    p.workerCount = std::clamp(n - 1, 1, defWorkers);

    bool cap = n > 0, freq = n > 0;
    for (const CoreInfo& c : cores) { cap &= c.capacity > 0; freq &= c.maxFreq > 0; }
    // <= 2 cores: isolation is impossible, share everything (doc §5 step 1).
    if (n <= 2 || !(cap || freq)) return p;
    std::vector<long> s(n);
    for (int i = 0; i < n; i++) s[i] = cap ? cores[i].capacity : cores[i].maxFreq;
    auto faster = [&](int a, int b) { return s[a] != s[b] ? s[a] > s[b] : cores[a].id > cores[b].id; };

    // Emu: the fastest core, ties to the highest index (today's RG DS core 3).
    int emu = 0;
    for (int i = 1; i < n; i++) if (faster(i, emu)) emu = i;
    // §5a: with known, mixed in-order/OoO types prefer the best OoO core unless the best
    // in-order core clocks more than 1.5x higher (a rough IPC-gap estimate; tunable).
    bool known = true, anyIn = false, anyOoO = false;
    int bestIn = -1, bestOoO = -1;
    for (int i = 0; i < n; i++)
    {
        known &= cores[i].type != CoreType::Unknown;
        if (cores[i].type == CoreType::InOrder) { anyIn = true; if (bestIn < 0 || faster(i, bestIn)) bestIn = i; }
        if (cores[i].type == CoreType::OoO) { anyOoO = true; if (bestOoO < 0 || faster(i, bestOoO)) bestOoO = i; }
    }
    const bool mixed = known && anyIn && anyOoO;
    if (mixed)
        emu = (cores[bestOoO].maxFreq > 0 && cores[bestIn].maxFreq * 2 > cores[bestOoO].maxFreq * 3) ? bestIn : bestOoO;
    p.pin = true;
    p.emu = cores[emu].id;

    // The rest, split big / little: little = under half the top speed, or in-order on a chip
    // that also has OoO cores. Little cores never get render-critical work while big ones exist.
    long top = *std::max_element(s.begin(), s.end());
    std::vector<int> big, little;
    for (int i = 0; i < n; i++)
    {
        if (i == emu) continue;
        bool isLittle = s[i] * 2 < top || (mixed && cores[i].type == CoreType::InOrder && cores[emu].type == CoreType::OoO);
        (isLittle ? little : big).push_back(i);
    }
    auto order = [&](std::vector<int>& v) {
        std::sort(v.begin(), v.end(), [&](int a, int b) { return s[a] != s[b] ? s[a] > s[b] : cores[a].id < cores[b].id; });
    };
    order(big); order(little);
    if (big.empty()) std::swap(big, little);

    for (int i : big) p.critical.push_back(cores[i].id);
    for (int i : (little.empty() ? big : little)) p.background.push_back(cores[i].id);

    // Workers: the fastest remaining equal-speed cluster (the barrier waits for the slowest
    // worker); if that is a single core, take up to defWorkers of the big cores instead.
    std::vector<int> w;
    for (int i : big) if (s[i] == s[big[0]]) w.push_back(i);
    if (w.size() < 2) w.assign(big.begin(), big.begin() + std::min<size_t>(big.size(), defWorkers));
    if ((int)w.size() > maxWorkers) w.resize(maxWorkers);
    for (int i : w) p.workers.push_back(cores[i].id);
    p.workerCount = (int)p.workers.size();
    return p;
}

// Runtime knob: an Android property, or (headless/desktop) an environment variable of the same name.
inline int Prop(const char* name, int def)
{
#ifdef __ANDROID__
    char b[PROP_VALUE_MAX] = {0};
    if (__system_property_get(name, b) > 0) return atoi(b);
#else
    if (const char* v = getenv(name)) return atoi(v);
#endif
    return def;
}

inline long ReadLong(const char* path, int base = 10)
{
    long v = 0;
    if (FILE* f = fopen(path, "r"))
    {
        char b[64] = {0};
        if (fgets(b, sizeof b, f)) v = (long)strtoull(b, nullptr, base);
        fclose(f);
    }
    return v;
}

// One log line per placement line (logcat in the app, stderr in headless).
inline void Log(const std::string& s)
{
    for (size_t a = 0, b; a < s.size(); a = b + 1)
    {
        b = s.find('\n', a);
        if (b == std::string::npos) b = s.size();
        Platform::Log(Platform::Info, "LITEV_TOPO %s\n", s.substr(a, b - a).c_str());
    }
}

// This device's placement (computed once). debug.litev.placement=1 logs it at startup.
inline const Placement& Get()
{
    static const Placement p = [] {
        std::vector<CoreInfo> cores;
#if defined(__ANDROID__) || defined(__linux__)
        int n = std::clamp((int)sysconf(_SC_NPROCESSORS_CONF), 1, (int)CPU_SETSIZE);
        // CPU type from MIDR, else /proc/cpuinfo "CPU implementer"/"CPU part" per processor.
        std::vector<unsigned> impl(n, 0), part(n, 0);
        if (FILE* f = fopen("/proc/cpuinfo", "r"))
        {
            char line[256]; int cur = -1; unsigned v;
            while (fgets(line, sizeof line, f))
            {
                if (sscanf(line, "processor : %d", &cur) == 1) continue;
                if (cur < 0 || cur >= n) continue;
                if (sscanf(line, "CPU implementer : %x", &v) == 1) impl[cur] = v;
                else if (sscanf(line, "CPU part : %x", &v) == 1) part[cur] = v;
            }
            fclose(f);
        }
        for (int i = 0; i < n; i++)
        {
            char path[96];
            CoreInfo c{i, 0, 0, CoreType::Unknown};
            snprintf(path, sizeof path, "/sys/devices/system/cpu/cpu%d/cpu_capacity", i);
            c.capacity = ReadLong(path);
            snprintf(path, sizeof path, "/sys/devices/system/cpu/cpu%d/cpufreq/cpuinfo_max_freq", i);
            c.maxFreq = ReadLong(path);
            snprintf(path, sizeof path, "/sys/devices/system/cpu/cpu%d/regs/identification/midr_el1", i);
            if (unsigned long long midr = (unsigned long long)ReadLong(path, 16))
                impl[i] = (midr >> 24) & 0xff, part[i] = (midr >> 4) & 0xfff;
            c.type = TypeFromId(impl[i], part[i]);
            cores.push_back(c);
        }
#else
        int n = (int)std::thread::hardware_concurrency();
        for (int i = 0; i < n; i++) cores.push_back({i, 0, 0, CoreType::Unknown});
#endif
        Placement r = Place(cores);
        if (Prop("debug.litev.placement", 0)) Log(r.Describe());
        return r;
    }();
    return p;
}

// Tile worker count: debug.litev.tileworkers (1..maxWorkers) or the topology's, rounded down to
// a divisor of the renderer's block count so every worker gets an equal contiguous range.
inline int TileWorkers(int blocks, int maxWorkers)
{
    int want = std::clamp(Prop("debug.litev.tileworkers", Get().workerCount), 1, maxWorkers);
    while (blocks % want) want--;
    return want;
}

} // namespace melonDS::LitevTopo

#if defined(__ANDROID__) || defined(__linux__)
#include <dirent.h>
#include <mutex>

namespace melonDS::LitevTopo
{

// The cores this process may use now: its cpuset (Android moves apps between top-app,
// foreground and background), else every online core. Re-read on each sweep.
inline cpu_set_t Allowed()
{
    cpu_set_t set; CPU_ZERO(&set);
    char cg[128] = {0}, path[192], list[256] = {0};
    bool ok = false;
    if (FILE* f = fopen("/proc/self/cpuset", "r")) { ok = fgets(cg, sizeof cg, f) != nullptr; fclose(f); }
    if (ok)
    {
        cg[strcspn(cg, "\n")] = 0;
        snprintf(path, sizeof path, "/dev/cpuset%s/cpus", strcmp(cg, "/") ? cg : "");
        ok = false;
        if (FILE* f = fopen(path, "r")) { ok = fgets(list, sizeof list, f) != nullptr; fclose(f); }
    }
    if (!ok)
        if (FILE* f = fopen("/sys/devices/system/cpu/online", "r")) { ok = fgets(list, sizeof list, f) != nullptr; fclose(f); }
    for (char* p = list; ok && *p;)
    {
        char* e; int a = (int)strtol(p, &e, 10), b = a;
        if (e == p) break;
        if (*e == '-') b = (int)strtol(e + 1, &e, 10);
        for (int c = a; c <= b && c < CPU_SETSIZE; c++) CPU_SET(c, &set);
        p = (*e == ',') ? e + 1 : e;
        if (*p == '\n') break;
    }
    if (CPU_COUNT(&set) == 0)
        for (int c : Get().all) CPU_SET(c, &set);
    return set;
}

// The role's cores, intersected with the allowed set (never empty).
inline cpu_set_t MaskFor(CoreRole r, int idx, const cpu_set_t& allowed)
{
    cpu_set_t set; CPU_ZERO(&set);
    for (int c : Get().CoresFor(r, idx)) if (CPU_ISSET(c, &allowed)) CPU_SET(c, &set);
    return CPU_COUNT(&set) ? set : allowed;
}

struct Entry { pid_t tid; CoreRole role; int idx; };
inline std::mutex& RegMutex() { static std::mutex m; return m; }
inline std::vector<Entry>& Registry() { static std::vector<Entry> r; return r; }

// Remember tid's role so the app's periodic sweep re-asserts it (not one blanket mask).
inline void Register(pid_t tid, CoreRole r, int idx = -1)
{
    std::lock_guard<std::mutex> l(RegMutex());
    for (Entry& e : Registry()) if (e.tid == tid) { e = {tid, r, idx}; return; }
    Registry().push_back({tid, r, idx});
}

// Register the calling thread and apply its role mask now.
inline void PinSelf(CoreRole r, int idx = -1)
{
    Register(gettid(), r, idx);
    cpu_set_t set = MaskFor(r, idx, Allowed());
    sched_setaffinity(0, sizeof set, &set);
}

// The app's periodic sweep: emuTid -> Emu, each registered thread -> its own role mask, every
// other thread of the process -> Background. Re-reads the allowed set; drops exited tids.
inline void Reassert(pid_t emuTid)
{
    static const bool distinct = Prop("debug.litev.tilepin", 0) != 0;
    const cpu_set_t allowed = Allowed();
    cpu_set_t set = MaskFor(CoreRole::Emu, -1, allowed);
    sched_setaffinity(emuTid, sizeof set, &set);
    std::vector<Entry> reg, live;
    { std::lock_guard<std::mutex> l(RegMutex()); reg = Registry(); }
    DIR* d = opendir("/proc/self/task");
    if (!d) return;
    while (dirent* e = readdir(d))
    {
        pid_t tid = atoi(e->d_name);
        if (tid <= 0 || tid == emuTid) continue;
        Entry en{tid, CoreRole::Background, -1};
        for (const Entry& x : reg) if (x.tid == tid) { en = x; live.push_back(x); break; }
        // Tile workers start on distinct cores (PinSelf); the sweep re-masks them to the shared
        // render mask unless debug.litev.tilepin=1 (= the old blanket sweep's steady state).
        if (en.role == CoreRole::RenderParallel && !distinct) en.idx = -1;
        set = MaskFor(en.role, en.idx, allowed);
        sched_setaffinity(tid, sizeof set, &set);
    }
    closedir(d);
    // ponytail: prune by "not in /proc/self/task now"; a tid reused between sweeps keeps the
    // old role for <= 300 frames. Unregister on thread exit if that ever matters.
    std::lock_guard<std::mutex> l(RegMutex());
    std::vector<Entry>& r = Registry();
    auto in = [](const std::vector<Entry>& v, pid_t t) {
        return std::any_of(v.begin(), v.end(), [t](const Entry& y) { return y.tid == t; });
    };
    // drop what was registered at the snapshot but has exited; keep anything registered since
    r.erase(std::remove_if(r.begin(), r.end(), [&](const Entry& x) { return in(reg, x.tid) && !in(live, x.tid); }), r.end());
}

} // namespace melonDS::LitevTopo

#endif

#endif // LITEV_TOPO_PIN

#endif
