// LITEV_A7_NATIVE_WM: see WMNative.h.
#ifdef LITEV_A7_NATIVE_WM
#include "WMNative.h"
#include "NDS.h"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <map>
#include <vector>
#include <deque>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace melonDS::WMNative
{
namespace { int InstOf(NDS& nds); }
#ifdef LITEV_HLE_DIAG
namespace
{
std::mutex g_TrM;
std::map<const NDS*, FILE*> g_Tr;
struct TrSt { u32 status = 0; u32 last[0x7bc / 4] = {}; };
std::map<const NDS*, TrSt> g_TrSt;
void StatusDiff(FILE* f, NDS& nds)
{
    TrSt* t;
    { std::lock_guard<std::mutex> l(g_TrM); t = &g_TrSt[&nds]; }
    if (!t->status) return;
    for (u32 i = 0; i < 0x7bc / 4; i++)
    {
        u32 v; memcpy(&v, &nds.MainRAM[(t->status + 4 * i) & nds.MainRAMMask], 4);
        if (v != t->last[i]) { fprintf(f, "  S+%03x %08x -> %08x\n", 4 * i, t->last[i], v); t->last[i] = v; }
    }
}
FILE* Tr(NDS& nds)
{
    static const char* pre = getenv("LITEV_WMTRACE");
    if (!pre) return nullptr;
    std::lock_guard<std::mutex> l(g_TrM);
    auto it = g_Tr.find(&nds);
    if (it != g_Tr.end()) return it->second;
    char fn[512];
    if (InstOf(nds) >= 0) snprintf(fn, sizeof fn, "%s.%d.txt", pre, InstOf(nds));
    else snprintf(fn, sizeof fn, "%s.x%zu.txt", pre, g_Tr.size());
    FILE* f = fopen(fn, "w");
    g_Tr[&nds] = f;
    return f;
}
void Dump(FILE* f, NDS& nds, u32 a, int n)
{
    if ((a >> 24) != 2) return;
    fprintf(f, "  @%08x", a);
    for (int i = 0; i < n; i += 4) { u32 v; memcpy(&v, &nds.MainRAM[(a + i) & nds.MainRAMMask], 4); fprintf(f, "%s%08x", i % 32 ? " " : "\n   ", v); }
    fprintf(f, "\n");
}
}
void TraceFifo(NDS& nds, int fromArm7, u32 val)
{
    FILE* f = Tr(nds);
    if (!f) return;
    const u32 tag = val & 31, data = val >> 6;
    fprintf(f, "%s f%u t%llu tag%u err%u data %07x\n", fromArm7 ? "F7" : "F9", nds.NumFrames, (unsigned long long)nds.ARM7Timestamp, tag, (val >> 5) & 1, data);
    if (tag == 10 && (data >> 24) == 2)
    {
        Dump(f, nds, data, fromArm7 ? 0x100 : 0x80);
        u32 w0; memcpy(&w0, &nds.MainRAM[data & nds.MainRAMMask], 4);
        if (!fromArm7 && (w0 & 0x7fff) == 0) { u32 st; memcpy(&st, &nds.MainRAM[(data + 8) & nds.MainRAMMask], 4); std::lock_guard<std::mutex> l(g_TrM); g_TrSt[&nds].status = st; }
        StatusDiff(f, nds);
        // LITEV_WMTRACE_MEM=<from>:<to>: at each SET_MP_DATA request, the ARM7 WRAM / ARM7 VRAM words changed since the last one
        static const char* e = getenv("LITEV_WMTRACE_MEM");
        static const u32 from = e ? (u32)atoi(e) : 0, to = e && strchr(e, ':') ? (u32)atoi(strchr(e, ':') + 1) : 0;
        if (e && !fromArm7 && (w0 & 0x7fff) == 0xf && nds.NumFrames >= from && nds.NumFrames <= to)
        {
            static thread_local std::vector<u32> last;
            std::vector<u32> cur;
            for (u32 a = 0x037F8000; a < 0x03810000; a += 4) cur.push_back(nds.ARM7Read32(a));
            for (u32 a = 0x06000000; a < 0x06040000; a += 4) cur.push_back(nds.ARM7Read32(a));
            if (last.size() == cur.size())
                for (size_t i = 0; i < cur.size(); i++)
                    if (cur[i] != last[i]) fprintf(f, "  M %08x %08x -> %08x\n", i < 0x6000 ? 0x037F8000 + 4 * (u32)i : 0x06000000 + 4 * (u32)(i - 0x6000), last[i], cur[i]);
            last = cur;
        }
    }
}
void TraceFrame(NDS& nds, int tx, const u8* buf, int len, u64 us)
{
    FILE* f = Tr(nds);
    if (!f) return;
    fprintf(f, "%s f%u us%llu len%d", tx ? "TX" : "RX", nds.NumFrames, (unsigned long long)us, len);
    for (int i = 0; i < len; i++) fprintf(f, "%s%02x", i % 32 ? "" : "\n   ", buf[i]);
    fprintf(f, "\n");
}
void TraceChip(NDS& nds, int wr, u32 addr, u16 val, u64 us)
{
    static const char* e = getenv("LITEV_WMTRACE_CHIP");
    static const u32 from = e ? (u32)atoi(e) : 0, to = e && strchr(e, ':') ? (u32)atoi(strchr(e, ':') + 1) : ~0u;
    if (!e || nds.NumFrames < from || nds.NumFrames > to) return;
    FILE* f = Tr(nds);
    if (!f) return;
    const u32 a = addr & 0x7FFE;
    if (a >= 0x4000 && a < 0x6000) fprintf(f, "CM %04x %04x\n", a - 0x4000, val);
    else fprintf(f, "C%c %03x %04x us%llu pc %08x\n", wr ? 'W' : 'R', a, val, (unsigned long long)us, nds.ARM7.R[15]);
}
void TraceA7Write(NDS& nds, u32 addr, u32 val, int size)
{
    // LITEV_WMTRACE_A7W=<from>:<to> (frames)
    static const char* e = getenv("LITEV_WMTRACE_A7W");
    static const u32 from = e ? (u32)atoi(e) : 0, to = e && strchr(e, ':') ? (u32)atoi(strchr(e, ':') + 1) : ~0u;
    if (!e || nds.NumFrames < from || nds.NumFrames > to) return;
    if (addr >= 0x027E0000 && addr < 0x027FF000) return;     // (ARM7 private main RAM: stacks / heap)
    FILE* f = Tr(nds);
    if (!f) return;
    fprintf(f, "W%d %08x %0*x pc %08x f%u\n", size, addr, size * 2, val, nds.ARM7.R[15], nds.NumFrames);
}
#endif

// ==== the native driver ===============================================================================================
namespace
{
constexpr u32 Event_WMN = Event_DSi_NWifi;     // (only a DSi uses this slot; the native driver runs on a DS console only)
constexpr u32 kTagWM = 10;
constexpr u32 kCbFlag = 0x027FFF96;            // bit 0: the ARM7->ARM9 callback slot is in use (the ARM9 clears it)

struct Ctx
{
    explicit Ctx(NDS& n) : nds(n) {}
    NDS& nds;
    int inst = -1;
    int allow = -1;                             // -1: debug.litev.a7wm / env decides at WM INITIALIZE
    bool active = false;
    u32 wm7buf = 0, status = 0, cbuf = 0;       // ARM9 structures (from INITIALIZE)
    std::deque<u32> reqs;                       // request buffers not processed yet
    std::deque<std::vector<u8>> cbs;            // callbacks waiting for the slot
    u64 wake = ~0ull;                           // the scheduled event time (sys cycles), ~0 none
    u64 now = 0;                                // the time of the current entry (sys cycles)
    bool inside = false, irqPending = false;
    u64 stReq = 0, stCb = 0, stIrq = 0, stEv = 0;
    ~Ctx();
};
u32 Rd32(Ctx& c, u32 a) { return c.nds.ARM7Read32(a); }
u16 Rd16(Ctx& c, u32 a) { return c.nds.ARM7Read16(a); }
void Wr32(Ctx& c, u32 a, u32 v) { c.nds.ARM7Write32(a, v); }
void Wr16(Ctx& c, u32 a, u16 v) { c.nds.ARM7Write16(a, v); }
u64 CpuNow(NDS& nds) { return nds.CurCPU == 0 ? nds.ARM9Timestamp >> nds.ARM9ClockShift : nds.ARM7Timestamp; }

Ctx* Get(NDS& nds) { return (Ctx*)nds.Wifi.WMN.get(); }
int InstOf(NDS& nds) { Ctx* c = Get(nds); return c ? c->inst : -1; }
Ctx& GetOrMake(NDS& nds)
{
    if (!nds.Wifi.WMN) nds.Wifi.WMN = {new Ctx(nds), [](void* p) { delete (Ctx*)p; }};
    return *Get(nds);
}

const bool g_Stats = getenv("LITEV_A7WM_STATS") && atoi(getenv("LITEV_A7WM_STATS"));
Ctx::~Ctx()
{
    if (g_Stats && active) fprintf(stderr, "A7WM inst%d: requests=%llu callbacks=%llu chip_irqs=%llu events=%llu\n", inst, (unsigned long long)stReq, (unsigned long long)stCb, (unsigned long long)stIrq, (unsigned long long)stEv);
}

// debug.litev.a7wm (prop on Android, env elsewhere): 0 off, 1 on; unset: the build default
bool Allowed(Ctx& c)
{
    if (c.allow >= 0) return c.allow == 1;
#if defined(__ANDROID__)
    char b[16] = {0};
    const char* v = __system_property_get("debug.litev.a7wm", b) > 0 ? b : nullptr;
#else
    const char* v = getenv("debug.litev.a7wm");
#endif
    return v ? atoi(v) == 1 : false;        // ponytail: default off until the gates pass
}


// ---- WM build recognition ----
// The WMSP request dispatch table (apiid -> handler; 46+ entries into one module) is found by a scan of the ARM7's memory;
// the build id is an FNV-1a hash of the first 96 words of every handler, branch offsets / pc-relative load offsets /
// pointer-like literals masked (relocation-insensitive: the same build at another address matches). Unrecognised builds
// keep the emulated driver.
struct Build { u64 id; const char* name; };
constexpr Build kBuilds[] = {
    {0x408040ea75491a59ull, "NitroSDK 3.0 WM (Mario Kart DS, Animal Crossing WW)"},
};
u32 MaskWord(u32 w)
{
    if (((w >> 25) & 7) == 5) return w & 0xFF000000;                       // B / BL
    if ((w & 0x0F7F0000) == 0x051F0000) return w & 0xFFFFF000;              // LDR rX, [pc, #imm]
    if ((w >= 0x01FF8000 && w < 0x02800000) || (w >= 0x02FE0000 && w < 0x03000000) || (w >= 0x037F8000 && w < 0x03810000) || (w >= 0x06000000 && w < 0x06900000)) return 0;
    return w;
}
u64 Fnv(u64 h, u32 w) { for (int k = 0; k < 4; k++) h = (h ^ ((w >> (8 * k)) & 0xFF)) * 0x100000001b3ull; return h; }
const Build* FindBuild(Ctx& c, u32* tableOut)
{
    static const u32 kRegions[][2] = {{0x06000000, 0x06040000}, {0x027C0000, 0x02800000}, {0x02FE0000, 0x03000000}, {0x037F8000, 0x03810000}};
    for (const auto& r : kRegions)
    {
        u32 run = 0, base = 0;
        for (u32 a = r[0]; a < r[1]; a += 4)
        {
            const u32 w = Rd32(c, a);
            const bool ptr = !(w & 3) && w >= r[0] && w < r[1] && w != 0;
            if (!ptr) { run = 0; continue; }
            if (!run) base = a;
            if (++run < 46) continue;
            // a run of 46 code pointers into this region: the table candidate starts at base
            u32 distinct = 0, prev = 0;
            for (u32 i = 0; i < 46; i++) { const u32 e = Rd32(c, base + 4 * i); distinct += e != prev; prev = e; }
            if (distinct < 40) { run = 0; continue; }
            u64 h = 0xcbf29ce484222325ull;
            for (u32 i = 0; i < 46; i++)
            {
                const u32 e = Rd32(c, base + 4 * i);
                for (u32 k = 0; k < 96; k++) h = Fnv(h, MaskWord(Rd32(c, e + 4 * k)));
            }
            for (const Build& b : kBuilds) if (b.id == h) { if (tableOut) *tableOut = base; return &b; }
            if (g_Stats) fprintf(stderr, "A7WM inst%d: WM dispatch table at %08x, build %016llx not supported (emulated driver)\n", c.inst, base, (unsigned long long)h);
            return nullptr;
        }
    }
    if (g_Stats) fprintf(stderr, "A7WM inst%d: no WM dispatch table found (emulated driver)\n", c.inst);
    return nullptr;
}

void Process(Ctx& c);
void OnEvent(void* that, u32)
{
    Ctx& c = *(Ctx*)that;
    c.now = c.wake;
    c.wake = ~0ull;
    c.stEv++;
    Process(c);
}
// run Process at time t (sys cycles) or earlier
void WakeAt(Ctx& c, u64 t)
{
    if (t >= c.wake) return;
    NDS& nds = c.nds;
    nds.CancelEvent(Event_WMN);
    SchedEvent& e = nds.SchedList[Event_WMN];
    nds.ScheduleEvent(Event_WMN, true, (s32)(t - e.Timestamp), 0, 0);
    c.wake = t;
}

// ---- ARM9 interface ----
void SendCb(Ctx& c, const void* p, size_t n)
{
    c.cbs.emplace_back((const u8*)p, (const u8*)p + n);
}
// the next callback into the slot when the ARM9 has freed it (and the IPC FIFO has room)
void PumpCbs(Ctx& c)
{
    NDS& nds = c.nds;
    while (!c.cbs.empty())
    {
        if (Rd16(c, kCbFlag) & 1) { WakeAt(c, c.now + 256); return; }      // ponytail: poll; the ARM9 frees it in its PXI IRQ
        if (nds.ARM7IORead16(0x04000184) & 0x0002) { WakeAt(c, c.now + 256); return; }   // (send FIFO full)
        const std::vector<u8>& b = c.cbs.front();
        for (size_t i = 0; i + 4 <= b.size(); i += 4) { u32 v; memcpy(&v, &b[i], 4); Wr32(c, c.cbuf + (u32)i, v); }
        if (b.size() & 2) { u16 v; memcpy(&v, &b[b.size() & ~3], 2); Wr16(c, c.cbuf + (u32)(b.size() & ~3), v); }
        Wr16(c, kCbFlag, Rd16(c, kCbFlag) | 1);
        nds.ARM7IOWrite32(0x04000188, kTagWM | (c.cbuf << 6));
        c.cbs.pop_front();
        c.stCb++;
    }
}

struct Cb4 { u16 apiid, err; };
void Request(Ctx& c, u32 a)
{
    const u16 api = Rd16(c, a) & 0x7FFF;
    Wr16(c, a, Rd16(c, a) | 0x8000);        // (the request buffer is free again)
    c.stReq++;
    switch (api)
    {
    case 0x00:      // INITIALIZE   ponytail: stub (state IDLE, no chip) until the specs land
        Wr16(c, c.status, 2);
        { Cb4 r{0, 0}; SendCb(c, &r, sizeof r); }
        break;
    default:
        { Cb4 r{api, 3}; SendCb(c, &r, sizeof r); }     // WM_ERRCODE_ILLEGAL_STATE   ponytail: until each API is in
        break;
    }
}

void Process(Ctx& c)
{
    if (c.inside) return;
    c.inside = true;
    while (!c.reqs.empty()) { u32 a = c.reqs.front(); c.reqs.pop_front(); Request(c, a); }
    PumpCbs(c);
    c.inside = false;
}

bool Activate(Ctx& c, u32 req)
{
    NDS& nds = c.nds;
    if (nds.ConsoleType != 0 || !Allowed(c)) return false;
    u32 table = 0;
    const Build* b = FindBuild(c, &table);
    if (!b) return false;
    c.wm7buf = Rd32(c, req + 4); c.status = Rd32(c, req + 8); c.cbuf = Rd32(c, req + 12);
    nds.RegisterEventFuncs(Event_WMN, &c, {[](void* that, u32 p) { OnEvent(that, p); }});
    c.active = true;
    nds.Wifi.WMNOwns = true;
    if (g_Stats) fprintf(stderr, "A7WM inst%d: native WM driver active: %s (table %08x; WM7 %08x status %08x cb %08x)\n", c.inst, b->name, table, c.wm7buf, c.status, c.cbuf);
    return true;
}
}

void Reset(NDS& nds)
{
    Ctx* p = Get(nds);
    if (!p) return;
    const int inst = p->inst, allow = p->allow;
    if (p->active) nds.CancelEvent(Event_WMN);
    nds.Wifi.WMNOwns = false;
    nds.Wifi.WMN.reset();
    Ctx& c = GetOrMake(nds);
    c.inst = inst; c.allow = allow;
}

void SetInstance(NDS& nds, int inst, int allow)
{
    Ctx& c = GetOrMake(nds);
    c.inst = inst;
    c.allow = allow;
}

bool Fifo9(NDS& nds, u32 val)
{
    Ctx* p = Get(nds);
    const u32 a = val >> 6;
    if (!p || !p->active)
    {
        // WM INITIALIZE (apiid 0) is where the native driver takes the WM over
        if ((a >> 24) != 2 || (nds.ARM7Read16(a) & 0x7FFF) != 0) return false;
        if (!Activate(GetOrMake(nds), a)) return false;
        p = Get(nds);
    }
#ifdef LITEV_HLE_DIAG
    TraceFifo(nds, 0, val);
#endif
    Ctx& c = *p;
    c.reqs.push_back(a);
    WakeAt(c, CpuNow(nds) + 64);        // (the ARM7 takes the request in its PXI IRQ)
    return true;
}

void ChipIRQ(NDS& nds)
{
    Ctx* p = Get(nds);
    if (!p || !p->active) return;
    p->stIrq++;
    p->irqPending = true;
    WakeAt(*p, CpuNow(nds) + 16);
}
}
#endif
