// LITEV_A7_NATIVE_WM: see WMNative.h.
#ifdef LITEV_A7_NATIVE_WM
#include "WMNative.h"
#include "NDS.h"
#include "Savestate.h"
#include "Platform.h"
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
thread_local u64 g_SendAt = 0;      // a native callback's send time (its event's; the CPUs can be past it)
std::map<const NDS*, FILE*> g_Tr;
struct TrSt { u32 status = 0; u32 last[0x7d0 / 4] = {}; };
std::map<const NDS*, TrSt> g_TrSt;
void StatusDiff(FILE* f, NDS& nds)
{
    TrSt* t;
    { std::lock_guard<std::mutex> l(g_TrM); t = &g_TrSt[&nds]; }
    if (!t->status) return;
    for (u32 i = 0; i < 0x7d0 / 4; i++)
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
    fprintf(f, "%s f%u t%llu tag%u err%u data %07x\n", fromArm7 ? "F7" : "F9", nds.NumFrames, (unsigned long long)(fromArm7 && g_SendAt ? g_SendAt : nds.CurCPU == 0 ? nds.ARM9Timestamp >> nds.ARM9ClockShift : nds.ARM7Timestamp), tag, (val >> 5) & 1, data);
    if (tag == 10 && (data >> 24) == 2)
    {
        Dump(f, nds, data, fromArm7 ? 0x100 : 0x80);
        u32 w0; memcpy(&w0, &nds.MainRAM[data & nds.MainRAMMask], 4);
        if (!fromArm7 && (w0 & 0x7fff) == 0) { u32 st; memcpy(&st, &nds.MainRAM[(data + 8) & nds.MainRAMMask], 4); std::lock_guard<std::mutex> l(g_TrM); g_TrSt[&nds].status = st; }
        StatusDiff(f, nds);
        // a START_SCAN_EX "found" callback: the BSS descriptors it points to
        if (fromArm7 && (w0 & 0xffff) == 0x26)
        {
            u32 st8; memcpy(&st8, &nds.MainRAM[(data + 8) & nds.MainRAMMask], 4);
            u32 n; memcpy(&n, &nds.MainRAM[(data + 0xc) & nds.MainRAMMask], 4);
            if ((st8 & 0xffff) == 5) for (u32 i = 0; i < ((n >> 16) & 0xff) && i < 4; i++) { u32 d; memcpy(&d, &nds.MainRAM[(data + 0x10 + 4 * i) & nds.MainRAMMask], 4); fprintf(f, "  DESC%u", i); Dump(f, nds, d, 0xc0); }
        }
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
    else fprintf(f, "C%c %03x %04x us%llu pc %08x v%u\n", wr ? 'W' : 'R', a, val, (unsigned long long)us, nds.ARM7.R[15], nds.GPU.VCount);
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
// Two layers, mirroring the guest's: WMSP (the PXI request handlers, WMStatus, the callbacks to the ARM9, the send
// queue, port packets, the MP V-count alarms) and WL (the MAC layer: chip bring-up, scan, beacon, auth / assoc, the MP
// CMD / reply / ACK exchange). WMSP keeps its state where the guest keeps it (WMStatus, ARM9 RAM: the ARM9 library and
// the game read parts of it); WL keeps its state here. The guest's WM / WL threads, their IRQ handler and alarms never
// run: the ARM9's WM requests never reach the ARM7 and the chip's IRQ comes here (Wifi::WMNOwns).
// Timing: requests / callbacks / chip IRQs are handled at once (the guest takes microseconds of ARM7 time); the WL
// waits (scan dwell, measure time, auth / assoc timeouts), the MP V-count alarms and the child's reply-window alarm are
// timed as the guest's. Category B: deterministic, version-locked; on air the frames are the guest's byte for byte.
namespace
{
constexpr u32 Event_WMN = Event_DSi_NWifi;     // (only a DSi uses this slot; the native driver runs on a DS console only)
constexpr u32 kTagWM = 10;
constexpr u32 kCbFlag = 0x027FFF96;            // bit 0: the ARM7->ARM9 callback slot is in use (the ARM9 clears it)
constexpr u32 kEntropy = 0x027FFF98;
inline u64 Us(u64 us) { return us * 33513982ull / 1000000ull; }
inline u64 Ms(u64 ms) { return ms * 33513982ull / 1000ull; }

enum : u16
{
    R_ModeReset = 0x004, R_ModeWEP = 0x006, R_TXStatCnt = 0x008, R_IF = 0x010, R_IE = 0x012,
    R_MAC0 = 0x018, R_BSSID0 = 0x020, R_AIDLow = 0x028, R_AIDFull = 0x02A, R_TXRetry = 0x02C, R_RXCnt = 0x030,
    R_WEPCnt = 0x032, R_PowerUS = 0x036, R_PowerTX = 0x038, R_PowerState = 0x03C, R_PowerForce = 0x040, R_PowerDown = 0x048,
    R_RXBegin = 0x050, R_RXEnd = 0x052, R_RXWrCursor = 0x054, R_RXWrAddr = 0x056, R_RXRdCursor = 0x05A, R_RXGap = 0x062,
    R_BeaconSlot = 0x080, R_BeaconTIM = 0x084, R_ListenCount = 0x088, R_BeaconInt = 0x08C, R_ListenInt = 0x08E,
    R_CmdSlot = 0x090, R_Reply1 = 0x094, R_Reply2 = 0x098, R_Loc2 = 0x0A4, R_TXReqReset = 0x0AC, R_TXReqSet = 0x0AE,
    R_TXSlotReset = 0x0B4, R_TXBusy = 0x0B6, R_TXStat = 0x0B8, R_Preamble = 0x0BC, R_CmdTotal = 0x0C0, R_CmdReply = 0x0C4,
    R_RXFilter = 0x0D0, R_RXFilter2 = 0x0E0, R_USCountCnt = 0x0E8, R_USCmpCnt = 0x0EA, R_USCmp0 = 0x0F0,
    R_US0 = 0x0F8, R_US1 = 0x0FA, R_US2 = 0x0FC, R_US3 = 0x0FE, R_CmdCount = 0x118, R_BeaconCount2 = 0x134,
    R_BBCnt = 0x158, R_BBWrite = 0x15A, R_BBMode = 0x160, R_BBPower = 0x168, R_RFData2 = 0x17C, R_RFData1 = 0x17E,
    R_RFCnt = 0x184, R_1AA = 0x1AA, R_1AE = 0x1AE,
};
// WM_STATE / WM_ERRCODE / WM_STATECODE (NitroSDK)
enum : u16 { S_STOP = 0, S_READY = 1, S_IDLE = 2, S_CLASS1 = 3, S_SCAN = 5, S_PARENT = 7, S_CHILD = 8, S_MP_PARENT = 9, S_MP_CHILD = 0xa };
enum : u16 { E_OK = 0, E_FAILED = 1, E_OPERATING = 2, E_ILLEGAL = 3, E_INVALID = 6, E_FIFO = 8, E_TIMEOUT = 9, E_QFULL = 0xa,
             E_NO_ENTRY = 0xb, E_OVER_MAX = 0xc, E_NO_DATA = 0xe, E_SEND_FAILED = 0xf };
enum : u16 { SC_PARENT_START = 0, SC_BEACON_SENT = 2, SC_NOT_FOUND = 4, SC_FOUND = 5, SC_CONNECT_START = 6, SC_CONNECTED = 7,
             SC_DISCONNECTED = 9, SC_MP_START = 0xa, SC_MPEND = 0xb, SC_MP_IND = 0xc, SC_MPACK = 0xd, SC_BEACON_RECV = 0x10,
             SC_AUTH = 0x13, SC_PORT_SEND = 0x14, SC_PORT_RECV = 0x15, SC_FIFO_ERROR = 0x16, SC_DISC_MYSELF = 0x1a };
// WMStatus offsets (NitroSDK 3.0 WM; spec-wmsp section 7)
enum : u32
{
    ST_state = 0x000, ST_busyApi = 0x002, ST_apiBusy = 0x004, ST_mpFlag = 0x00c, ST_dcf = 0x010, ST_ks = 0x014, ST_dcfSend = 0x018,
    ST_vsync = 0x01c, ST_wlVer = 0x020, ST_macVer = 0x028, ST_rfVer = 0x02a, ST_bbVer = 0x02c, ST_c030 = 0x030, ST_c032 = 0x032,
    ST_freq = 0x034, ST_count = 0x036, ST_limit = 0x038, ST_resend = 0x03a, ST_lastPoll = 0x03c, ST_lastHdr = 0x03e, ST_lastTmptt = 0x040,
    ST_lastLen = 0x042, ST_rbSel = 0x044, ST_rbSize = 0x046, ST_rb0 = 0x048, ST_rb1 = 0x04c, ST_sendBuf = 0x050, ST_sendSize = 0x054,
    ST_ackTs = 0x056, ST_waitAck = 0x084, ST_ready = 0x086, ST_newFrame = 0x088, ST_keyPend = 0x08a, ST_rxB = 0x08c, ST_rxA = 0x08e,
    ST_polled = 0x090, ST_minPoll = 0x092, ST_single = 0x094, ST_fixFreq = 0x096, ST_retry = 0x098, ST_keepAlive = 0x09a,
    ST_keepCnt = 0x09c, ST_ptgid = 0x0b6, ST_link = 0x0b8, ST_minRssi = 0x0ba, ST_rssiCnt = 0x0bc, ST_indOn = 0x0be, ST_pm = 0x0c2,
    ST_vaPend = 0x0ca, ST_sync = 0x0cc, ST_syncLast = 0x0d0, ST_vofs = 0x0d4, ST_vres = 0x0d8, ST_ignFatal = 0x0da, ST_mac = 0x0dc,
    ST_mode = 0x0e2, ST_pp = 0x0e4, ST_ppUgi = 0x0e4, ST_ppUgiLen = 0x0e8, ST_ppGgid = 0x0ec, ST_ppTgid = 0x0f0, ST_ppEntry = 0x0f2,
    ST_ppMaxEntry = 0x0f4, ST_ppMB = 0x0f6, ST_ppKS = 0x0f8, ST_ppCS = 0x0fa, ST_ppBeacon = 0x0fc, ST_ppChannel = 0x116,
    ST_ppPSize = 0x118, ST_ppCSize = 0x11a, ST_childMac = 0x124, ST_conn = 0x17e, ST_scanBuf = 0x180, ST_aid = 0x184, ST_pMac = 0x186,
    ST_scanCh = 0x18c, ST_maxSend = 0x18e, ST_maxRecv = 0x190, ST_wepOn = 0x194, ST_rate = 0x1e8, ST_preamble = 0x1ea,
    ST_enableCh = 0x1f0, ST_allowedCh = 0x1f2, ST_seq = 0x1f4, ST_q = 0x2f4, ST_free = 0x6f4, ST_readyQ = 0x6f8, ST_sendQ = 0x708,
    ST_inFlight = 0x730, ST_lastRx = 0x734, ST_life = 0x7b4,
};

enum Timer : u8 { T_OP, T_AIDCLR, T_ACKTO, T_TICK, T_POLL, kTimers };
enum Op : u8 { OpNone, OpScan, OpMeasure, OpConnect, OpDeauth, OpHold };

struct Rq { u32 addr; u16 api; u32 a1; };
struct Ind { u16 code; std::vector<u8> b; };      // b = the WL body (the guest's message + 0x10)      // addr 0: an internal request (api, a1)
struct Sta { u8 state = 0; u8 mac[6] = {}; u16 aid = 0; u16 life = 0xffff, lifeReload = 0xffff; u8 ssid[32] = {}; };
struct Mgmt { std::vector<u8> f; u8 kind; u8 sta; u16 status; };
struct Build;   // a queued management frame (parent: LOC2 in the TBTT window)

struct Ctx
{
    explicit Ctx(NDS& n) : nds(n) {}
    NDS& nds;
    int inst = -1;
    int allow = -1;                             // -1: debug.litev.a7wm / env decides at WM INITIALIZE
    bool active = false;
    u32 w7 = 0, st = 0, cbuf = 0;               // ARM9 structures (from INITIALIZE)
    std::deque<Rq> reqs;                        // requests not processed yet (ARM9's and internal)
    struct Cb { u8 d[0x100] = {}; u8 m[0x100] = {}; u16 n = 0; };
    std::deque<Cb> cbs;                         // callbacks waiting for the slot (only the written bytes are stored)
    u64 wake = ~0ull, now = 0;                  // the scheduled event (sys cycles), the time of this entry
    u64 tm[kTimers] = {};                       // WL / WMSP timers (0 off)
    bool inside = false, irqPending = false, lineDue = false;
    u16 lineDueV = 0;
    u64 stReq = 0, stCb = 0, stIrq = 0, stEv = 0, stMp = 0, stRx = 0;
    // the request the guest's request thread would be blocked on (a WL operation in progress)
    u8 op = OpNone; u32 opAddr = 0; u16 opApi = 0; int opStage = 0;
    u16 opCh = 0, opTime = 0; u32 opScanBuf = 0; u8 opBssid[6] = {};
    std::vector<u8> opDesc;                     // scan: the first BSS descriptor found
    u32 opArg = 0; u8 opMac[6] = {}; u16 opReason = 0; u16 opBitmap = 0, opDone = 0; bool opAuto = false, opWasMP = false;
    // WL
    u16 wl = 0;                                 // 0 off, 0x10 idle, 0x20 class1, 0x40 active
    u16 mode = 0;                               // 0 none, 1 parent, 2 child / scan, 4 measure
    u8 mac[6] = {}, bssid[6] = {};
    u16 channel = 0, rxBegin = 0;
    bool pm = false;                            // MLME power management mode (parent: the beacon's active zone)
    u16 activeZone = 0xffff;
    u16 vtsf = 0;                               // the guest's u16 0x0380fff0 (parent: V-count stamp; child: from the beacon)
    u32 entropy = 0;
    // parent
    Sta sta[16];                                // [aid 1..15]; [0] unused
    std::vector<Sta> pending;                   // authenticated, not associated yet (by MAC)
    u16 maxChild = 0;
    std::deque<Mgmt> mgmtQ; bool mgmtBusy = false; Mgmt mgmtCur;
    bool txWindow = false;
    u16 beaconLen = 0, gameInfoOff = 0;         // (chip RAM 0x0238: the beacon; gameInfoOff = Nintendo IE offset)
    std::vector<u8> pendingGameInfo; bool gameInfoPending = false;
    u32 seqNo = 0;
    // parent MP (WL MA_MP)
    bool mpBusy = false; u16 mpPoll = 0, mpGot = 0, mpN = 0, mpStride = 0, mpTxopUs = 0, mpLen = 0, mpHdr = 0, mpTmptt = 0;
    u16 mpRemain = 0; std::vector<u8> mpInd; u16 mpFailed = 0; std::vector<u8> mpBody;
    // child
    u16 aid = 0; u8 pmac[6] = {}; bool replyB = false; bool bufBusy[2] = {}; u16 lastCmdSeq = 0xffff; u16 cmdFlags = 0;
    u16 beaconPeriod = 0;
    // VAlarm (WMSP's MP V-count alarm)
    s32 vX = -1;
    // WMSP
    u8 rssi[32] = {}; u32 rssiIdx = 0;          // (3.x: the first 16)
    u16 camLife = 0x28;                         // WL station lifetime (100 ms units)
    Rq opRq{0, 0, 0};
    std::deque<struct Ind> inds;                // WL -> WMSP indications not handled yet
    std::deque<std::vector<u8>> autoDeAuth;     // MACs for internal AUTO_DEAUTH (0x22)
    u8 joinDesc[0x40] = {}; u16 joinTgid = 0, joinPSize = 0, joinCSize = 0; u8 joinAttr = 0, joinRssi = 0;
    u16 mpTxopOrig = 0;
    const u16* stMap = nullptr;                 // WMStatus offset map of this build (nullptr: the 3.0 layout)
    const struct Build* bd = nullptr; bool v31 = false, v4 = false, pw = false;
    bool nullKey = false;                       // WL NullKeyResponseMode (3.1 MP: the child's W_AIDLow stays set)
    u64 keyNext = 0;
    u64 reqAt = 0;                              // the front ARM9 request runs then (the guest's WL processing time)
    u64 vaNext = 0;                             // the MP V-alarm request (0x1c) is handled not before this                            // child: the next key data not before this (3.1 childInterval)
    // 3.1+: the next-MP / next-key-data requests the parent / child interval OS alarm posts when it fires (the guest posts
    // them then, so ARM9 requests arriving meanwhile - SET_MP_DATA - are processed first and go into that MP)
    u64 kickAt[2] = {}; u16 kickArg[2] = {};
    // START_SCAN_EX (4.x): channels left / requested, the result buffer, the SSID filter, the BSS descriptors found
    bool opEx = false; u16 opList = 0, opListAll = 0, opBufSize = 0, opSsidLen = 0, opMatch = 0, opType = 0; u8 opSsid[32] = {};
    std::vector<std::vector<u8>> opDescs;
    u64 mpNext = 0;                             // the next MP of a sequence not before this (the guest's MP end -> CMD path)
    // late takeover: the guest's WM handles INITIALIZE and everything up to the first local-play request (scan / measure /
    // parent / connect to a DS parent), and again from END on (Nintendo WFC's DCF mode stays emulated)
    s8 build = -1;                              // this WM session: -1 not checked, 0 emulated, k >= 1 kBuilds[k - 1] (may take over)
    u32 lastPassed = 0;                         // the last request left to the guest
    bool guestCbSince = true;                   // a guest callback went to the ARM9 since then
    bool release = false;                       // END done: give the WM back once its callback is out
    std::deque<Cb> held;                        // a request's callbacks held until the guest would have finished it
    ~Ctx();
};
u32 Rd32(Ctx& c, u32 a) { return c.nds.ARM7Read32(a); }
u16 Rd16(Ctx& c, u32 a) { return c.nds.ARM7Read16(a); }
u8 Rd8(Ctx& c, u32 a) { return c.nds.ARM7Read8(a); }
void Wr32(Ctx& c, u32 a, u32 v) { c.nds.ARM7Write32(a, v); }
void Wr16(Ctx& c, u32 a, u16 v) { c.nds.ARM7Write16(a, v); }
void Wr8(Ctx& c, u32 a, u8 v) { c.nds.ARM7Write8(a, v); }
// WMStatus address of a field given by its NitroSDK 3.0 (MKDS) offset: other builds map it (BuildDef::stMap)
u32 SA(Ctx& c, u32 o) { return c.st + (c.stMap ? c.stMap[o] : o); }
u16 S16(Ctx& c, u32 o) { return Rd16(c, SA(c, o)); }
u32 S32(Ctx& c, u32 o) { return Rd32(c, SA(c, o)); }
void SS16(Ctx& c, u32 o, u16 v) { Wr16(c, SA(c, o), v); }
void SS32(Ctx& c, u32 o, u32 v) { Wr32(c, SA(c, o), v); }
void SS64(Ctx& c, u32 o, u64 v) { SS32(c, o, (u32)v); SS32(c, o + 4, (u32)(v >> 32)); }
u64 S64(Ctx& c, u32 o) { return S32(c, o) | (u64)S32(c, o + 4) << 32; }
// NitroSDK 3.1 WMStatus fields with no 3.0 counterpart (3.1 offsets)
enum : u32 { N_pSize = 0x30, N_cSize = 0x32, N_pMax = 0x34, N_cMax = 0x36, N_sendSize = 0x38, N_recvSize = 0x3a, N_maxSend = 0x3c,
             N_maxRecv = 0x3e, N_pVC = 0x40, N_cVC = 0x42, N_pInt = 0x44, N_cInt = 0x46, N_pTick = 0x48, N_cTick = 0x50, N_minFreq = 0x58,
             N_freq = 0x5a, N_maxFreq = 0x5c, N_ignSize = 0x9c,
             // the persistent MP parameters (3.1 + 4.x) and 4.x's current ones (the MP engine runs on these)
             N_minPoll = 0x92, N_single = 0x94, N_retry = 0x98, N_ignFatal = 0x9a,
             A_minFreq = 0x7c0, A_freq = 0x7c2, A_maxFreq = 0x7c4, A_minPoll = 0x7c6, A_single = 0x7c8, A_retry = 0x7ca, A_ignFatal = 0x7cc };
u16 N16(Ctx& c, u32 o) { return Rd16(c, c.st + o); }
void NS16(Ctx& c, u32 o, u16 v) { Wr16(c, c.st + o, v); }
u64 N64(Ctx& c, u32 o) { return Rd32(c, c.st + o) | (u64)Rd32(c, c.st + o + 4) << 32; }
void NS64(Ctx& c, u32 o, u64 v) { Wr32(c, c.st + o, (u32)v); Wr32(c, c.st + o + 4, (u32)(v >> 32)); }
// 4.x: WEP off (wep flag, mode, the 0x50 key bytes)
void WepOff(Ctx& c) { if (!c.v4) return; Wr32(c, c.st + 0x198, 0); Wr16(c, c.st + 0x196, 0); for (u32 o = 0; o < 0x50; o += 4) Wr32(c, c.st + 0x19c + o, 0); }
// the key-data pending flag (TwlSDK 5.x WMSP never writes it)
void KeyPend(Ctx& c, u16 v) { if (!c.pw) SS16(c, ST_keyPend, v); }
// 3.1 size variables (0x027e0000..0x027e0100): "parent" = my aid 0
bool AmParent(Ctx& c) { return S16(c, ST_aid) == 0; }
void SetPSize(Ctx& c, u16 s) { NS16(c, N_pSize, s); NS16(c, AmParent(c) ? N_sendSize : N_recvSize, s + 4); }
void SetCSize(Ctx& c, u16 s) { NS16(c, N_cSize, s); NS16(c, AmParent(c) ? N_recvSize : N_sendSize, s + 2); }
void SetPMax(Ctx& c, u16 s)
{
    NS16(c, N_pSize, s); NS16(c, N_pMax, s);
    const bool p = AmParent(c);
    NS16(c, p ? N_maxSend : N_maxRecv, s + 4); NS16(c, p ? N_sendSize : N_recvSize, s + 4);
}
void SetCMax(Ctx& c, u16 s)
{
    NS16(c, N_cSize, s); NS16(c, N_cMax, s);
    const bool p = AmParent(c);
    NS16(c, p ? N_maxRecv : N_maxSend, s + 2); NS16(c, p ? N_recvSize : N_sendSize, s + 2);
}
// the sizes go to 0 (3.0: maxSend / maxRecv; 3.1: ResetSizeVars)
void SizesOff(Ctx& c)
{
    if (c.v31) for (u32 o = N_pSize; o < N_maxRecv + 2; o += 2) NS16(c, o, 0);
    else { SS16(c, ST_maxSend, 0); SS16(c, ST_maxRecv, 0); }
}
u64 CpuNow(NDS& nds) { return nds.CurCPU == 0 ? nds.ARM9Timestamp >> nds.ARM9ClockShift : nds.ARM7Timestamp; }
u64 Tick(Ctx& c) { return c.now >> 6; }
inline void B16v(u8* p, u32 o, u16 v) { p[o] = v & 0xFF; p[o + 1] = v >> 8; }
inline u16 G16v(const u8* p, u32 o) { return (u16)(p[o] | p[o + 1] << 8); }        // OS_GetTick (33.514 MHz / 64)

// chip
u16 CR(Ctx& c, u16 r) { return c.nds.Wifi.Read(0x04808000 | r); }
void CW(Ctx& c, u16 r, u16 v) { c.nds.Wifi.Write(0x04808000 | r, v); }
u8* RAM(Ctx& c) { return c.nds.Wifi.RAMPtr(); }
u16 M16(Ctx& c, u32 o) { u16 v; memcpy(&v, &RAM(c)[o & 0x1FFE], 2); return v; }
void MW16(Ctx& c, u32 o, u16 v) { memcpy(&RAM(c)[o & 0x1FFE], &v, 2); }
void MWB(Ctx& c, u32 o, const u8* p, u32 n) { for (u32 i = 0; i < n; i++) RAM(c)[(o + i) & 0x1FFF] = p[i]; }
u64 USCount(Ctx& c) { return CR(c, R_US0) | (u64)CR(c, R_US1) << 16 | (u64)CR(c, R_US2) << 32 | (u64)CR(c, R_US3) << 48; }
const u8* Fw(Ctx& c) { return c.nds.SPI.GetFirmware().Buffer(); }

Ctx* Get(NDS& nds) { return (Ctx*)nds.Wifi.WMN.get(); }
int InstOf(NDS& nds) { Ctx* c = Get(nds); return c ? c->inst : -1; }
Ctx& GetOrMake(NDS& nds)
{
    if (!nds.Wifi.WMN) nds.Wifi.WMN = {new Ctx(nds), [](void* p) { delete (Ctx*)p; }};
    return *Get(nds);
}

const bool g_Stats = getenv("LITEV_A7WM_STATS") && atoi(getenv("LITEV_A7WM_STATS"));
const int g_Dbg = getenv("LITEV_A7WM_DBG") ? atoi(getenv("LITEV_A7WM_DBG")) : 0;
Ctx::~Ctx()
{
    if (g_Stats && active) fprintf(stderr, "A7WM inst%d: requests=%llu callbacks=%llu chip_irqs=%llu events=%llu mp=%llu rx=%llu\n", inst, (unsigned long long)stReq,
                                   (unsigned long long)stCb, (unsigned long long)stIrq, (unsigned long long)stEv, (unsigned long long)stMp, (unsigned long long)stRx);
}

// debug.litev.a7wm (prop on Android, env elsewhere): 0 off; unset / 1: on (compiled in = on)
bool Allowed(Ctx& c)
{
    if (c.allow >= 0) return c.allow == 1;
#if defined(__ANDROID__)
    char b[16] = {0};
    const char* v = __system_property_get("debug.litev.a7wm", b) > 0 ? b : nullptr;
#else
    const char* v = getenv("debug.litev.a7wm");
#endif
    return v ? atoi(v) != 0 : true;
}
// debug.litev.a7wmtwl=0 (prop / env): the TWL 5.x build (PB/PW/B2W2/Sims 3) stays emulated (default on: its MP path matched
// the emulated driver through a PB<->PW Union Room trainer-card exchange)
bool TwlAllowed()
{
#if defined(__ANDROID__)
    char b[16] = {0};
    const char* v = __system_property_get("debug.litev.a7wmtwl", b) > 0 ? b : nullptr;
#else
    const char* v = getenv("debug.litev.a7wmtwl");
#endif
    return !v || atoi(v) != 0;
}

// ---- WM build recognition ----
// The WMSP request dispatch table (apiid -> handler; 46+ entries into one module) is found by a scan of the ARM7's memory;
// the build id is an FNV-1a hash of the first 96 words of every handler, branch offsets / pc-relative load offsets /
// pointer-like literals masked (relocation-insensitive: the same build at another address matches). Unrecognised builds
// keep the emulated driver.
// Per build: the WMSP / WL generation and the guest's latencies (us) for the waits the native driver times.
//  gen 30: NitroSDK 3.0 (spec-wmsp / spec-wl)
//  gen 31: NitroSDK 3.1 - 3.1 WMStatus layout, START_MP + WMMPParam, SET_MP_PARAMETER, size variables, null-key MP mode,
//          DISCONNECTED_FROM_MYSELF, interval alarms (spec-shrek-delta)
//  gen 40: NitroSDK 4.x (WL 2.83) - + current MP parameters (st+0x7c0..), WMMPTmpParam, 0x204 port limits, 32-entry
//          RSSI ring, V-alarm line 0xcb, START_SCAN_EX (spec-kirby-delta; NitroSDK 4.2 source)
//  gen 50: TwlSDK 5.x NITRO mode (WL 2.88) - + no key-pending flag, child empty flag rxFlags 0x0400, flush before the MP
//          clean-ups, PORT_RECV rssi (spec-pw-delta)
struct Build
{
    u64 id; const char* name; u8 gen; char wlVer[8];
    u32 parentHoldUs, scanSetupUs, scanExtraUs, scanDownUs, endScanUs, measPreUs, measExtraUs, measPostUs, mpGapUs, resetUs, endParentUs;
    // the guest's request-thread latency from the MP V-count alarm to the chip (parent: CMD armed, child: reply armed);
    // measured as V-count lines of the guest's W_TXSlotCmd / W_TXSlotReply1 write after the alarm line, x 63.56 us
    u32 vaParentUs, vaChildUs;
    u32 kickChildUs;    // child: MPACK -> next reply armed (the guest's KICK_MP_CHILD request latency)
};
constexpr Build kBuilds[] = {
    {0x408040ea75491a59ull, "NitroSDK 3.0 WM (Mario Kart DS, Animal Crossing WW)", 30, "2.64.00", 3000, 1500, 100, 800, 11200, 0, 0, 12600, 1100, 0, 0, 445, 190, 0},
    {0x29342b46397da78full, "NitroSDK 3.1 WM (Shrek SnCR, New Super Mario Bros.)", 31, "2.74.00", 4500, 3800, 112, 570, 11700, 3800, 120, 12050, 900, 0, 0, 1557, 1049, 510},
    {0x3e7897fa761e4bf8ull, "NitroSDK 4.x WM (Kirby SSU, Chrono Trigger, M&L BIS)", 40, "2.83.00", 4340, 2300, 100, 1300, 12000, 3800, 120, 12580, 1600, 12000, 12160, 1843, 1176, 600},
    {0x6d9f2d0d3ca0bd96ull, "NitroSDK 4.x WM (GTA CW, Sonic Chronicles)", 40, "2.83.00", 4340, 2300, 100, 1300, 12000, 3800, 120, 12580, 1600, 12000, 12160, 1843, 1176, 600},
    {0x32abc95715c0a7fdull, "NitroSDK 4.x WM (Dragon Quest IX)", 40, "2.83.00", 4340, 2300, 100, 1300, 12000, 3800, 120, 12580, 1600, 12000, 12160, 1843, 1176, 600},
    {0x44459275eff97ba4ull, "NitroSDK 4.x WM (Pokemon HeartGold / SoulSilver)", 40, "2.83.00", 4340, 2300, 100, 1300, 12000, 3800, 120, 12580, 1600, 12000, 12160, 1843, 1176, 600},
    {0x8f8c5c9ada28f3c0ull, "NitroSDK 4.x WM (Pokemon Platinum)", 40, "2.83.00", 4340, 2300, 100, 1300, 12000, 3800, 120, 12580, 1600, 12000, 12160, 1843, 1176, 600},
    {0x5c3f3bbdb2cacf4bull, "TwlSDK 5.x WM, NITRO mode (Pokemon Black / White / B2W2, The Sims 3)", 50, "2.88.00", 6070, 3570, 320, 180, 11780, 3800, 120, 12580, 1600, 12000, 12700, 1589, 1176, 700},
};
// NitroSDK 3.1 WMStatus: 3.0 offset -> 3.1 offset (spec-shrek-delta 1.2); 4.x: + the MP engine's current parameters
const u16* Map31(bool v4)
{
    static u16 tab[2][0x800];
    static const bool init = [] {
      for (int g = 0; g < 2; g++)
      {
        u16* m = tab[g];
        for (u32 o = 0; o < 0x800; o++) m[o] = (u16)(o >= 0x9a ? o + 4 : o);
        for (u32 o = 0x30; o < 0x34; o++) m[o] = (u16)(o + 0x2e);
        m[0x34] = 0x5a; m[0x35] = 0x5b;
        for (u32 o = 0x36; o < 0x58; o++) m[o] = (u16)(o + 0x2c);
        m[0xda] = 0x9a; m[0xdb] = 0x9b;
        for (u32 o = 0x18e; o < 0x192; o++) m[o] = (u16)(o - 0x152);
        if (g)              // ST_freq / ST_minPoll / ST_single / ST_retry / ST_ignFatal -> mp_current_* (u16)
        {
            const u16 cur[][2] = {{0x34, 0x7c2}, {0x92, 0x7c6}, {0x94, 0x7c8}, {0x98, 0x7ca}, {0xda, 0x7cc}};
            for (auto& k : cur) { m[k[0]] = k[1]; m[k[0] + 1] = k[1] + 1; }
        }
      }
        return true;
    }();
    (void)init;
    return tab[v4];
}
void SetBuild(Ctx& c, int b)
{
    c.build = (s8)b;
    c.bd = b >= 1 ? &kBuilds[b - 1] : nullptr;
    const u8 gen = c.bd ? c.bd->gen : 30;
    c.v31 = gen >= 31; c.v4 = gen >= 40; c.pw = gen >= 50;
    c.stMap = c.v31 ? Map31(c.v4) : nullptr;
}
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
            if (g_Stats) fprintf(stderr, "A7WM inst%d: pointer run at %08x, hash %016llx: not a supported WM build\n", c.inst, base, (unsigned long long)h);
            run = 0;                            // (another pointer run can precede the WM table: keep scanning)
        }
    }
    if (g_Stats) fprintf(stderr, "A7WM inst%d: no supported WM build found (emulated driver)\n", c.inst);
    return nullptr;
}

// ---- scheduling ----
void Process(Ctx& c);
void Release(Ctx& c);
void OnEvent(void* that, u32)
{
    Ctx& c = *(Ctx*)that;
    c.now = std::max(c.now, c.wake);         // (monotonic: a V-line runs at its line's start, which can be behind)
    c.wake = ~0ull;
    c.stEv++;
    Process(c);
}
void WakeAt(Ctx& c, u64 t)
{
    if (t >= c.wake) return;
    NDS& nds = c.nds;
    nds.CancelEvent(Event_WMN);
    nds.SchedList[Event_WMN].Timestamp = t;           // (absolute: as Wifi::EvtScheduleAt)
    nds.ScheduleEvent(Event_WMN, true, 0, 0, 0);
    c.wake = t;
}
void SetTimer(Ctx& c, Timer k, u64 at) { c.tm[k] = at ? at : 1; WakeAt(c, c.tm[k]); }
void StopTimer(Ctx& c, Timer k) { c.tm[k] = 0; }
// the WMSP V-count alarm (OSVAlarm at count X): the GPU calls VLine at line X
void ArmVLine(Ctx& c, s32 x)
{
    c.vX = x;
    c.nds.GPU.WMLine = x;
#ifdef LITEV_LCD_EVT
    if (c.nds.GPU.LcdEvt) c.nds.GPU.LcdPlan();
#endif
}

// ---- callbacks (ARM7 -> ARM9) ----
// Only the bytes the guest writes are written (the rest of the slot keeps the previous callback's bytes, as the guest).
struct CbB
{
    Ctx::Cb b;
    CbB(u16 api, u16 err) { P16(0, api); P16(2, err); }
    CbB& P8(u32 o, u8 v) { b.d[o] = v; b.m[o] = 1; if (o + 1 > b.n) b.n = o + 1; return *this; }
    CbB& P16(u32 o, u16 v) { P8(o, v & 0xFF); return P8(o + 1, v >> 8); }
    CbB& P32(u32 o, u32 v) { P16(o, v & 0xFFFF); return P16(o + 2, v >> 16); }
    CbB& PB(u32 o, const u8* p, u32 n) { for (u32 i = 0; i < n; i++) P8(o + i, p[i]); return *this; }
    // 3.1: the parent / child sizes at o, o + 2 (st+0x30 / +0x32)
    CbB& Sz(Ctx& c, u32 o) { if (c.v31) { P16(o, N16(c, N_pSize)); P16(o + 2, N16(c, N_cSize)); } return *this; }
    // 3.1 PORT_SEND / PORT_RECV: my size, the peer's size
    CbB& MySz(Ctx& c, u32 o)
    {
        if (c.v31) { const bool p = AmParent(c); P16(o, N16(c, p ? N_pSize : N_cSize)); P16(o + 2, N16(c, p ? N_cSize : N_pSize)); }
        return *this;
    }
};
// internal request ids and the dispatcher's bound (3.1: two more APIs before the internal ones)
u16 IdMP(Ctx& c) { return c.v31 ? 0x2B : 0x29; }
u16 IdKey(Ctx& c) { return c.v31 ? 0x2C : 0x2A; }
u16 IdResend(Ctx& c) { return c.v31 ? 0x2D : 0x2B; }
u16 ApiBound(Ctx& c) { return c.v31 ? 0x2E : 0x2C; }
void Send(Ctx& c, const CbB& cb) { c.cbs.push_back(cb.b); }
// the next callback into the slot when the ARM9 has freed it
void PumpCbs(Ctx& c)
{
    NDS& nds = c.nds;
    while (!c.cbs.empty())
    {
        if ((Rd16(c, kCbFlag) & 1) || (nds.ARM7IORead16(0x04000184) & 0x0002))   // (slot in use / send FIFO full)
        {
            SetTimer(c, T_POLL, c.now + 128);       // ponytail: poll; the ARM9 frees it in its PXI IRQ, microseconds later
            return;
        }
        const Ctx::Cb& b = c.cbs.front();
        for (u32 i = 0; i < b.n; )
        {
            if (!b.m[i]) { i++; continue; }
            if (!(i & 3) && i + 3 < b.n && b.m[i + 1] && b.m[i + 2] && b.m[i + 3]) { u32 v; memcpy(&v, &b.d[i], 4); Wr32(c, c.cbuf + i, v); i += 4; }
            else if (!(i & 1) && b.m[i + 1]) { u16 v; memcpy(&v, &b.d[i], 2); Wr16(c, c.cbuf + i, v); i += 2; }
            else { Wr8(c, c.cbuf + i, b.d[i]); i++; }
        }
        Wr16(c, kCbFlag, Rd16(c, kCbFlag) | 1);
#ifdef LITEV_HLE_DIAG
        g_SendAt = c.now;
#endif
        nds.ARM7IOWrite32(0x04000188, kTagWM | (c.cbuf << 6));
#ifdef LITEV_HLE_DIAG
        g_SendAt = 0;
#endif
        c.cbs.pop_front();
        c.stCb++;
    }
}

// ---- small helpers ----
inline int Pop(u32 v) { return __builtin_popcount(v); }
u16 Conv(u8 b) { return (b & 2) ? (b >> 2) : (b >> 2) + 25; }          // RSSI byte -> WMSP units
u16 LinkLevel(u16 r) { return r < 8 ? 0 : r < 14 ? 1 : r < 20 ? 2 : 3; }
// 4.x: a joined parent of platform 8 (w7 bssDesc gameInfo byte 3) uses other thresholds
u16 LinkLevelC(Ctx& c, u16 r) { return c.v4 && Rd8(c, c.w7 + 0x53) == 8 ? (r < 0x16 ? 0 : r < 0x1C ? 1 : r < 0x22 ? 2 : 3) : LinkLevel(r); }
void SeedUpdate(Ctx& c, u16 r)
{
    u32 x = r ^ ((u32)Rd16(c, kEntropy) << 1);
    x ^= x >> 16;
    Wr16(c, kEntropy, (u16)x);
}
bool MacEq(const u8* a, const u8* b) { return !memcmp(a, b, 6); }
const u8 kBcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
const u8 kMpCmd[6] = {0x03, 0x09, 0xbf, 0x00, 0x00, 0x00};
const u8 kMpReply[6] = {0x03, 0x09, 0xbf, 0x00, 0x00, 0x10};

// ======================================================================================================================
// WL: the MAC layer on the chip (spec-wl)
// ======================================================================================================================
void BBW(Ctx& c, u8 idx, u8 v) { CW(c, R_BBWrite, v); CW(c, R_BBCnt, 0x5000 | idx); }
void RFW(Ctx& c, u32 v) { CW(c, R_RFData1, v & 0xFFFF); CW(c, R_RFData2, v >> 16); }
u16 Fw16(Ctx& c, u32 o) { const u8* f = Fw(c); return f[o] | f[o + 1] << 8; }
// the firmware register block + RF init (0x06009B04)
void FwRegs(Ctx& c)
{
    static const u16 kTbl[16] = {0x146, 0x148, 0x14A, 0x14C, 0x120, 0x122, 0x154, 0x144, 0x132, 0x132, 0x140, 0x142, 0x038, 0x124, 0x128, 0x150};
    const u8* f = Fw(c);
    for (int i = 0; i < 16; i++) CW(c, kTbl[i], Fw16(c, 0x44 + 2 * i));
    CW(c, R_RFCnt, (f[0x41] & 0x7F) | ((f[0x41] >> 7) << 8));
    if (f[0x40] == 3)
        for (u32 i = 0; i < f[0x42]; i++) RFW(c, 0x050000 | i << 8 | f[0xCE + i]);
    else
    {
        const u32 nb = ((f[0x41] & 0x7F) + 7) / 8;
        for (u32 i = 0; i < f[0x42]; i++)
        {
            u32 v = 0;
            for (u32 k = 0; k < nb; k++) v |= (u32)f[0xCE + i * nb + k] << (8 * k);
            RFW(c, v);
        }
    }
}
void PowerUp(Ctx& c) { CW(c, R_PowerUS, 0); CW(c, R_BBPower, 0); FwRegs(c); }
void PowerDown(Ctx& c) { BBW(c, 0x1E, 0x3F); CW(c, R_BBPower, 0x800D); CW(c, R_PowerUS, 1); }
// MAC stop (0x06009ECC)
void MacStop(Ctx& c)
{
    StopTimer(c, T_TICK);
    if (c.wl > 0x20) c.wl = 0x20;
    CW(c, R_IE, 0); CW(c, R_ModeReset, 0); CW(c, R_USCmpCnt, 0); CW(c, R_USCountCnt, 0); CW(c, R_TXStatCnt, 0); CW(c, 0x00A, 0);
    if (c.mode == 1) CW(c, R_BeaconSlot, 0);
    CW(c, R_TXReqReset, 0xFFFF); CW(c, R_TXSlotReset, 0xFFFF);
    c.mgmtQ.clear(); c.mgmtBusy = false; c.mpBusy = false;
}
void SetBssid(Ctx& c, const u8* b)
{
    memcpy(c.bssid, b, 6);
    for (int i = 0; i < 3; i++) CW(c, R_BSSID0 + 2 * i, b[2 * i] | b[2 * i + 1] << 8);
    const u16 f = CR(c, R_RXFilter);
    CW(c, R_RXFilter, (b[0] & 1) ? (f & ~0x400) : (f | 0x400));
}
// channel switch (0x0600AF7C)
void SetChannel(Ctx& c, u16 ch)
{
    const u8* f = Fw(c);
    const u16 save = CR(c, R_PowerForce);
    CW(c, R_PowerForce, 0x8001);
    c.channel = ch;
    if (f[0x40] == 3)
    {
        u32 p = 0xCE + f[0x42] + 1;
        for (u32 i = 0; i < f[0xCE + f[0x42]]; i++, p += 15) BBW(c, f[p], f[p + ch]);
        for (u32 i = 0; i < f[0x43]; i++, p += 15) RFW(c, 0x050000 | f[p] << 8 | f[p + ch]);
    }
    else
    {
        auto rd24 = [&](u32 o) { return f[o] | f[o + 1] << 8 | f[o + 2] << 16; };
        RFW(c, rd24(0xF2 + 6 * (ch - 1)));
        RFW(c, rd24(0xF5 + 6 * (ch - 1)));
        BBW(c, 0x1E, f[0x146 + ch - 1]);
    }
    CW(c, R_PowerForce, save);
    CW(c, R_PowerDown, 3);
}
void RxSetup(Ctx& c, u16 begin)
{
    c.rxBegin = begin;
    CW(c, R_RXCnt, 0x8000); CW(c, R_RXBegin, 0x4000 + begin); CW(c, R_RXWrAddr, begin / 2); CW(c, R_RXEnd, 0x5F60);
    CW(c, R_RXRdCursor, begin / 2); CW(c, R_RXGap, 0x5F5E); CW(c, R_RXCnt, 0x8001); CW(c, 0x24C, 0xFFFF); CW(c, 0x24E, 0xFFFF);
    MW16(c, 0x1F70, 0xFFFF); MW16(c, 0x1F72, 0xFFFF); MW16(c, 0x1F7E, 0xFFFF); MW16(c, 0x1F76, 0xFFFF);
}
void Sentinel(Ctx& c, u32 end) { const u32 a = (end + 3) & ~3u; MW16(c, a, 0xB6B8); MW16(c, a + 2, 0x1D46); }
// MAC start for a mode (0x06009F84): 1 parent, 2 STA (scan / join / connected child), 4 measure
void MacStart(Ctx& c, u16 mode)
{
    MacStop(c);
    c.mode = mode;
    CW(c, R_WEPCnt, 0x8000); CW(c, R_BeaconCount2, 0xFFFF); CW(c, R_AIDFull, 0); CW(c, R_AIDLow, 0); CW(c, R_PowerTX, 0x000F);
    if (mode == 1)
    {
        static const u16 kG[] = {0x234, 0x330, 0x954, 0xA9C, 0x10C0};
        for (u16 g : kG) { MW16(c, g, 0xB6B8); MW16(c, g + 2, 0x1D46); }
        RxSetup(c, 0x10C4);
    }
    else
    {
        static const u16 kG[] = {0x230, 0x464, 0x48C, 0x5D4, 0xBF8};
        for (u16 g : kG) { MW16(c, g, 0xB6B8); MW16(c, g + 2, 0x1D46); }
        CW(c, R_TXReqSet, 0x000D);
        RxSetup(c, mode == 4 ? 0xBFC : 0xBFC);
    }
    CW(c, R_RXCnt, 0x8000); CW(c, R_IF, 0xFFFF); CW(c, R_1AE, 0x1FFF); CW(c, R_1AA, 0x0400); CW(c, R_TXStatCnt, 0); CW(c, 0x00A, 0);
    if (mode == 1)
    {
        CW(c, R_IE, 0x703F); CW(c, R_1AE, 0x1FFF); CW(c, R_RXFilter, 0x0301); CW(c, R_RXFilter2, 0x000D);
        CW(c, R_TXStatCnt, 0xE000); CW(c, R_ModeReset, 1);
        const u64 bi = (u64)c.beaconPeriod << 10, t = (USCount(c) / bi + 1) * bi;
        CW(c, R_USCmp0 + 6, t >> 48); CW(c, R_USCmp0 + 4, (t >> 32) & 0xFFFF); CW(c, R_USCmp0 + 2, (t >> 16) & 0xFFFF);
        CW(c, R_USCmp0, (t & 0xFFFF) | 1);
        CW(c, R_USCountCnt, 1); CW(c, R_USCmpCnt, 1);
        c.wl = 0x40;
        CW(c, R_BeaconSlot, 0x8000 | (0x238 >> 1));
        CW(c, R_TXReqSet, 2);
    }
    else if (mode == 2)
    {
        CW(c, R_IE, 0xE03F); CW(c, R_1AA, 0x0468);
        CW(c, R_RXFilter, (c.bssid[0] & 1) ? 0x0581 : 0x0181); CW(c, R_RXFilter2, 0x000B);
        CW(c, R_ModeReset, 1); CW(c, R_USCountCnt, 1); CW(c, R_USCmpCnt, 1);
        c.wl = 0x20;
    }
    else
    {
        CW(c, R_IE, 0); CW(c, R_1AE, 0); CW(c, R_ModeReset, 1);
        c.wl = 0x20;
    }
    CW(c, R_PowerDown, 0);
    CW(c, R_TXReqSet, 2);
    // ponytail: power save off (the transceiver stays on; nothing on air differs): PWRMGT mode 0
    CW(c, R_PowerForce, 0x8000); CW(c, R_PowerState, 2);
    SetTimer(c, T_TICK, c.now + Ms(100));
}

void DeauthFinish(Ctx& c);
void DevIdle(Ctx& c);
// ---- frames ----
// a 12-byte TX header + 24-byte 802.11 header at chip RAM a; returns the body offset
u32 Hdr(Ctx& c, u32 a, u16 fc, const u8* a1, const u8* a2, const u8* a3, u32 bodyLen)
{
    for (u32 i = 0; i < 12; i += 2) if (i != 6) MW16(c, a + i, 0);
    MW16(c, a + 8, 0x0014);
    MW16(c, a + 0xA, 24 + bodyLen + 4);
    MW16(c, a + 0xC, fc); MW16(c, a + 0xE, 0);
    MWB(c, a + 0x10, a1, 6); MWB(c, a + 0x16, a2, 6); MWB(c, a + 0x1C, a3, 6);
    MW16(c, a + 0x22, 0);
    return a + 0x24;
}
// parent: management frames go out through LOC2 in the beacon window (TBTT .. active zone end), one at a time
constexpr u32 kLoc2Parent = 0x0958, kLoc2Child = 0x0490;
void MgmtKick(Ctx& c)
{
    if (c.mgmtBusy || c.mgmtQ.empty() || (c.mode == 1 && !c.txWindow)) return;
    c.mgmtCur = c.mgmtQ.front();
    c.mgmtQ.pop_front();
    const u32 a = c.mode == 1 ? kLoc2Parent : kLoc2Child;
    MWB(c, a, c.mgmtCur.f.data(), (u32)c.mgmtCur.f.size());
    Sentinel(c, a + (u32)c.mgmtCur.f.size());
    c.mgmtBusy = true;
    CW(c, R_Loc2, 0x8000 | (a >> 1));
    CW(c, R_TXReqSet, 4);
}
std::vector<u8> Frame(u16 fc, const u8* a1, const u8* a2, const u8* a3, const u8* body, u32 n)
{
    std::vector<u8> f(12 + 24 + n, 0);
    f[8] = 0x14; f[0xA] = (24 + n + 4) & 0xFF; f[0xB] = (24 + n + 4) >> 8;
    f[0xC] = fc & 0xFF; f[0xD] = fc >> 8;
    memcpy(&f[0x10], a1, 6); memcpy(&f[0x16], a2, 6); memcpy(&f[0x1C], a3, 6);
    if (n) memcpy(&f[0x24], body, n);
    return f;
}
enum MgmtKind : u8 { MK_AUTH_RESP, MK_ASSOC_RESP, MK_DEAUTH, MK_AUTH_REQ, MK_ASSOC_REQ };
void QueueMgmt(Ctx& c, std::vector<u8>&& f, u8 kind, u8 sta, u16 status)
{
    c.mgmtQ.push_back({std::move(f), kind, sta, status});
    MgmtKick(c);
}

// ---- beacon (parent) ----
std::vector<u8> GameInfo(Ctx& c);
void BuildBeacon(Ctx& c)
{
    const u32 a = 0x238;
    const std::vector<u8> gi = GameInfo(c);
    std::vector<u8> b;
    b.insert(b.end(), 8, 0);                                        // timestamp (the chip inserts it)
    b.push_back(c.beaconPeriod & 0xFF); b.push_back(c.beaconPeriod >> 8);
    b.push_back(0x21); b.push_back(0x00);                           // capability: ESS | short preamble
    const u8 rates[] = {0x01, 0x02, 0x82, 0x84}; b.insert(b.end(), rates, rates + 4);
    b.push_back(0x03); b.push_back(0x01); b.push_back((u8)c.channel);
    const u8 tim[] = {0x05, 0x05, 0x00, 0x02, 0x00, 0x00, 0x00}; b.insert(b.end(), tim, tim + 7);
    c.gameInfoOff = (u16)b.size();
    b.push_back(0xDD); b.push_back((u8)(gi.size() + 8)); b.push_back(0x00); b.push_back(0x09); b.push_back(0xBF); b.push_back(0x00);
    const u16 az = c.pm ? c.activeZone : 0xFFFF;
    b.push_back(az & 0xFF); b.push_back(az >> 8);
    b.push_back(c.vtsf & 0xFF); b.push_back(c.vtsf >> 8);
    b.insert(b.end(), gi.begin(), gi.end());
    const u32 body = Hdr(c, a, 0x0080, kBcast, c.mac, c.mac, (u32)b.size());
    MWB(c, body, b.data(), (u32)b.size());
    Sentinel(c, body + (u32)b.size());
    c.beaconLen = (u16)b.size();
    CW(c, R_BeaconTIM, 0x13 + 2);
}
void BeaconVtsf(Ctx& c) { MW16(c, 0x238 + 0x24 + c.gameInfoOff + 8, c.vtsf); }
void BeaconGameInfo(Ctx& c)
{
    const u32 a = 0x238, body = a + 0x24;
    const std::vector<u8>& gi = c.pendingGameInfo;
    RAM(c)[body + c.gameInfoOff + 1] = (u8)(gi.size() + 8);
    MWB(c, body + c.gameInfoOff + 10, gi.data(), (u32)gi.size());
    const u32 n = c.gameInfoOff + 10 + (u32)gi.size();
    MW16(c, a + 0xA, 24 + n + 4);
    Sentinel(c, body + n);
    c.gameInfoPending = false;
}

// ======================================================================================================================
// indications from WL to WMSP (handled in WMSP order: the guest's indication thread)
// ======================================================================================================================
std::deque<Ind>& IndQ(Ctx& c);
void PostInd(Ctx& c, u16 code, std::vector<u8>&& b) { IndQ(c).push_back({code, std::move(b)}); }
inline void B16(std::vector<u8>& b, u32 o, u16 v) { if (b.size() < o + 2) b.resize(o + 2); b[o] = v & 0xFF; b[o + 1] = v >> 8; }
inline u16 G16(const std::vector<u8>& b, u32 o) { return o + 1 < b.size() ? (u16)(b[o] | b[o + 1] << 8) : 0; }
inline u16 G16(const u8* p, u32 o) { return (u16)(p[o] | p[o + 1] << 8); }

// ---- RX ----
// one received frame: h = the 12-byte RX header, f = the 802.11 frame (h[8] bytes)
void RxBeacon(Ctx& c, const u8* h, const u8* f, u32 len);
void RxMgmt(Ctx& c, const u8* h, const u8* f, u32 len);
void RxCmd(Ctx& c, const u8* h, const u8* f, u32 len, u16 ts);
void RxAck(Ctx& c, const u8* h, const u8* f, u32 len, u16 ts);
void RxReply(Ctx& c, const u8* h, const u8* f, u32 len);
void RxRing(Ctx& c)
{
    const u16 wr = CR(c, R_RXWrCursor);
    u16 cur = CR(c, R_RXRdCursor);
    const u32 ringSize = 0x1F60 - c.rxBegin;
    u8 buf[12 + 0x930];
    int guard = 0;
    while (cur != wr && guard++ < 512)
    {
        auto rd = [&](u32 byteAddr) { if (byteAddr >= 0x1F60) byteAddr -= ringSize; return RAM(c)[byteAddr & 0x1FFF]; };
        const u32 base = (u32)cur * 2;
        for (u32 i = 0; i < 12; i++) buf[i] = rd(base + i);
        const u32 len = G16(buf, 8);
        u32 next = ((len + base + 0xF) >> 2) << 1;
        if (next >= 0xFB0) next -= ringSize / 2;
        if (len > 0x92C) { CW(c, R_RXRdCursor, wr); break; }
        for (u32 i = 0; i < len; i++) buf[12 + i] = rd(base + 12 + i);
        CW(c, R_RXRdCursor, next);
        cur = next;
        c.stRx++;
        const u8* f = buf + 12;
        const u16 fc = G16(f, 0), type = buf[0] & 0xF;
        const u16 ts = (u16)(USCount(c) >> 4);
        if (type == 1 && fc == 0x0080) RxBeacon(c, buf, f, len);
        else if ((type == 0 && (fc & 0xF) == 0) || (type == 5)) RxMgmt(c, buf, f, len);
        else if (type == 0xC && (fc & 0xE7FF) == 0x0228) RxCmd(c, buf, f, len, ts);
        else if (type == 0xD && (fc & 0xE7FF) == 0x0218) RxAck(c, buf, f, len, ts);
        else if ((type == 0xE || type == 0xF) && (fc & 0xE7BF) == 0x0118) RxReply(c, buf, f, len);
    }
}

// ---- scan: BSS descriptor from a beacon (A.2.2 / A.2.3) ----
std::vector<u8> BssDesc(Ctx& c, const u8* h, const u8* f, u32 len)
{
    std::vector<u8> d(0x40, 0);
    const u8* body = f + 24;
    const u32 bl = len > 24 ? len - 24 : 0;
    B16(d, 2, h[0xA]);
    memcpy(&d[4], f + 16, 6);
    B16(d, 0x2C, G16(body, 10));
    B16(d, 0x32, G16(body, 8));
    B16(d, 0x36, c.channel);
    const u8* ni = nullptr; u32 nl = 0;
    static const u8 kRates[16] = {0x02, 0x04, 0x0B, 0x16, 0x0C, 0x12, 0x18, 0x24, 0x30, 0x48, 0x60, 0x6C, 0, 0, 0, 0};
    for (u32 o = 12; o + 2 <= bl; )
    {
        const u8 id = body[o], l = body[o + 1];
        if (o + 2 + l > bl) break;
        const u8* p = body + o + 2;
        if (id == 0 && l <= 32) { B16(d, 0xA, l); memcpy(&d[0xC], p, l); }
        else if (id == 1)
        {
            u16 basic = 0, sup = 0;
            for (u32 i = 0; i < l; i++) for (int k = 0; k < 16; k++) if (kRates[k] && kRates[k] == (p[i] & 0x7F)) { sup |= 1 << k; if (p[i] & 0x80) basic |= 1 << k; }
            B16(d, 0x2E, basic); B16(d, 0x30, sup);
        }
        else if (id == 3 && l >= 1) B16(d, 0x36, p[0]);
        else if (id == 4 && l >= 6) B16(d, 0x38, p[2]);
        else if (id == 5 && l >= 3) B16(d, 0x34, p[1]);
        else if (id == 0xDD && l >= 8 && p[0] == 0x00 && p[1] == 0x09 && p[2] == 0xBF && p[3] == 0x00) { ni = p; nl = l; }
        o += 2 + l;
    }
    if (ni)
    {
        const u32 gl = nl - 8;
        B16(d, 0x3C, gl);
        d.resize(0x40 + gl);
        memcpy(&d[0x40], ni + 8, gl);
        B16(d, 0, (u16)((gl + 0x41) / 2));
    }
    else B16(d, 0, 0x41 / 2);
    if (d.size() & 1) d.push_back(0);
    return d;
}
// the Nintendo IE's active zone / V-count stamp (child: from the parent's beacons)
bool NintendoIE(const u8* f, u32 len, const u8** ie, u32* il)
{
    const u8* body = f + 24;
    const u32 bl = len > 24 ? len - 24 : 0;
    for (u32 o = 12; o + 2 <= bl; )
    {
        const u8 id = body[o], l = body[o + 1];
        if (o + 2 + l > bl) break;
        if (id == 0xDD && l >= 8 && body[o + 2] == 0x00 && body[o + 3] == 0x09 && body[o + 4] == 0xBF && body[o + 5] == 0x00) { *ie = body + o + 2; *il = l; return true; }
        o += 2 + l;
    }
    return false;
}
void ScanFound(Ctx& c, std::vector<u8>&& d);
void JoinBeacon(Ctx& c, const u8* h, const u8* f, u32 len);
void RxBeacon(Ctx& c, const u8* h, const u8* f, u32 len)
{
    if (c.op == OpScan && c.opStage == 1)
    {
        // passive scan: SSID matches (WMSP's scan SSID is empty) and BSSID filter (broadcast here)
        if (!(c.opBssid[0] & 1) && !MacEq(c.opBssid, f + 16)) return;
        if (c.opEx)         // (every new BSSID whose SSID passes the filter)
        {
            for (const auto& d : c.opDescs) if (MacEq(&d[4], f + 16)) return;
            std::vector<u8> d = BssDesc(c, h, f, len);
            if (c.opSsidLen)
            {
                const u32 n = std::min<u32>(c.opType >= 2 ? c.opMatch : c.opSsidLen, 32);
                if (memcmp(&d[0xC], c.opSsid, n)) return;
            }
            c.opDescs.push_back(std::move(d));
        }
        else if (c.opDesc.empty()) ScanFound(c, BssDesc(c, h, f, len));
        return;
    }
    if (c.mode != 2 || !MacEq(f + 16, c.bssid)) return;
    if (c.op == OpConnect && c.opStage == 1) { JoinBeacon(c, h, f, len); return; }
    if (c.wl != 0x40) return;
    // connected child: TBTT from the parent's timestamp, the V-count stamp, the station lifetime
    const u8* ie; u32 il;
    if (NintendoIE(f, len, &ie, &il)) c.vtsf = G16(ie, 6);
    c.sta[1].life = c.sta[1].lifeReload;
    {
        u64 T; memcpy(&T, f + 24, 8);
        const u64 P = (u64)c.beaconPeriod << 10, nx = (T / P + 1) * P;
        CW(c, R_USCmp0 + 6, nx >> 48); CW(c, R_USCmp0 + 4, (nx >> 32) & 0xFFFF); CW(c, R_USCmp0 + 2, (nx >> 16) & 0xFFFF); CW(c, R_USCmp0, (nx & 0xFFFF) | 1);
    }
    // BEACON_RECV indication (PARAM 0x215 on)
    std::vector<u8> b(0x2C, 0);
    b[0xE] = h[6]; b[0xF] = h[0xA];
    memcpy(&b[0x1E], f + 10, 6);
    if (NintendoIE(f, len, &ie, &il))
    {
        const u32 gl = il - 8;
        B16(b, 6, gl);
        b.resize(0x2C + gl);
        memcpy(&b[0x2C], ie + 8, gl);
    }
    PostInd(c, 0x008D, std::move(b));
}

// ARM9 memory blocks (main RAM fast path)
void ReadA9(Ctx& c, u32 a, u8* d, u32 n)
{
    if ((a >> 24) == 2) { for (u32 i = 0; i < n; i++) d[i] = c.nds.MainRAM[(a + i) & c.nds.MainRAMMask]; return; }
    for (u32 i = 0; i < n; i++) d[i] = Rd8(c, a + i);
}
void WriteA9(Ctx& c, u32 a, const u8* s, u32 n)
{
    u32 i = 0;
    if (!(a & 3)) for (; i + 4 <= n; i += 4) { u32 v; memcpy(&v, s + i, 4); Wr32(c, a + i, v); }
    for (; i + 2 <= n; i += 2) { u16 v; memcpy(&v, s + i, 2); Wr16(c, a + i, v); }
    for (; i < n; i++) Wr8(c, a + i, s[i]);
}

// ---- parent: auth / assoc / deauth (B.5) ----
Sta* StaByMac(Ctx& c, const u8* m)
{
    for (int a = 1; a < 16; a++) if (c.sta[a].state >= 0x40 && MacEq(c.sta[a].mac, m)) return &c.sta[a];
    return nullptr;
}
Sta* PendingByMac(Ctx& c, const u8* m)
{
    for (Sta& s : c.pending) if (MacEq(s.mac, m)) return &s;
    return nullptr;
}
bool QueuedTo(Ctx& c, const u8* m, u16 fc)
{
    for (const Mgmt& g : c.mgmtQ) if (G16(g.f.data(), 0xC) == fc && MacEq(&g.f[0x10], m)) return true;
    return c.mgmtBusy && G16(c.mgmtCur.f.data(), 0xC) == fc && MacEq(&c.mgmtCur.f[0x10], m);
}
void IndMacReason(Ctx& c, u16 code, const u8* m, u16 v)
{
    std::vector<u8> b(8, 0);
    memcpy(&b[0], m, 6); B16(b, 6, v);
    PostInd(c, code, std::move(b));
}
void SendDeauth(Ctx& c, const u8* to, u16 reason)
{
    const u8 body[2] = {(u8)(reason & 0xFF), (u8)(reason >> 8)};
    QueueMgmt(c, Frame(0x00C0, to, c.mac, c.bssid, body, 2), MK_DEAUTH, 0, reason);
}
void ConnectStep(Ctx& c, int result, u16 status);
void RxMgmt(Ctx& c, const u8* h, const u8* f, u32 len)
{
    const u16 fc = G16(f, 0), sub = (fc >> 4) & 0xF;
    const u8* sa = f + 10;
    const u8* body = f + 24;
    const u32 bl = len > 24 ? len - 24 : 0;
    if (c.mode == 1)
    {
        if (!MacEq(f + 4, c.mac)) return;
        if (sub == 0xB && bl >= 6)          // auth request
        {
            if (QueuedTo(c, sa, 0x00B0)) return;
            const u16 alg = G16(body, 0), seq = G16(body, 2);
            if (Sta* s = StaByMac(c, sa))   // known and associated: back to authenticated, its link went
            {
                IndMacReason(c, 0x0085, s->mac, 1);
                *s = Sta{};
            }
            u16 status = 0;
            const bool known = PendingByMac(c, sa) != nullptr;
            int used = (int)c.pending.size();
            for (int a = 1; a < 16; a++) used += c.sta[a].state >= 0x40;
            if (!known && used >= c.maxChild) status = 0x13;
            else if (alg != 0 || seq != 1) status = 0xE;
            if (!status && !known) { Sta s; memcpy(s.mac, sa, 6); s.state = 0x20; c.pending.push_back(s); }
            const u8 rb[6] = {(u8)alg, (u8)(alg >> 8), 2, 0, (u8)status, (u8)(status >> 8)};
            QueueMgmt(c, Frame(0x00B0, sa, c.mac, c.bssid, rb, 6), MK_AUTH_RESP, 0, status);
        }
        else if (sub == 0x0 && bl > 4)      // assoc request
        {
            if (QueuedTo(c, sa, 0x0010)) return;
            Sta* p = PendingByMac(c, sa);
            if (!p || p->state < 0x30) { if (!QueuedTo(c, sa, 0x00C0)) SendDeauth(c, sa, 6); return; }
            const u16 cap = G16(body, 0);
            u16 status = 0;
            const u8* ssid = nullptr; u32 sl = 0;
            for (u32 o = 4; o + 2 <= bl; )
            {
                const u8 id = body[o], l = body[o + 1];
                if (o + 2 + l > bl) break;
                if (id == 0) { ssid = body + o + 2; sl = l; }
                o += 2 + l;
            }
            if ((cap & 0xFFC2) || !(cap & 1) || !(cap & 0x20)) status = 0x0A;
            else
            {
                // SSID (masked compare: WMSP's mask = bytes 8..31 ignored; own SSID = ggid, tgid, 0)
                u8 own[32] = {};
                const u32 gg = S32(c, ST_ppGgid); const u16 tg = S16(c, ST_ppTgid);
                own[0] = gg & 0xFF; own[1] = gg >> 8; own[2] = gg >> 16; own[3] = gg >> 24; own[4] = tg & 0xFF; own[5] = tg >> 8;
                if (!ssid || sl != 32) status = 1;
                else for (int i = 0; i < 8; i++) if (ssid[i] != own[i]) status = 1;
            }
            u16 aid = 0;
            if (!status)
            {
                for (u16 a = 1; a < 16; a++) if (!c.sta[a].state) { aid = a; break; }
                if (!aid) status = 0x13;
                else
                {
                    p->aid = aid; if (ssid) memcpy(p->ssid, ssid, 32);
                    c.sta[aid] = Sta{}; c.sta[aid].state = 0x10; memcpy(c.sta[aid].mac, sa, 6);   // (reserved until the response is out)
                }
            }
            const u16 a2 = aid ? (aid | 0xC000) : 0;
            const u8 rb[10] = {0x21, 0x00, (u8)status, (u8)(status >> 8), (u8)a2, (u8)(a2 >> 8), 0x01, 0x02, 0x82, 0x84};
            QueueMgmt(c, Frame(0x0010, sa, c.mac, c.bssid, rb, 10), MK_ASSOC_RESP, (u8)aid, status);
        }
        else if (sub == 0xC && bl >= 2)     // deauth
        {
            if (Sta* s = StaByMac(c, sa)) { IndMacReason(c, 0x0085, s->mac, G16(body, 0)); *s = Sta{}; }
            for (size_t i = 0; i < c.pending.size(); i++) if (MacEq(c.pending[i].mac, sa)) { c.pending.erase(c.pending.begin() + i); break; }
        }
        else if (sub == 0xA && bl >= 2)     // disassoc
        {
            if (Sta* s = StaByMac(c, sa)) { IndMacReason(c, 0x0088, s->mac, G16(body, 0)); }
        }
        return;
    }
    if (c.mode != 2 || !MacEq(f + 4, c.mac)) return;
    if (sub == 0xB && bl >= 6 && c.op == OpConnect && c.opStage == 2 && MacEq(sa, c.bssid) && G16(body, 2) == 2)
    {
        StopTimer(c, T_OP);
        const u16 status = G16(body, 4);
        ConnectStep(c, status ? 0xC : 0, status);
    }
    else if (sub == 0x1 && bl >= 6 && c.op == OpConnect && c.opStage == 3 && MacEq(sa, c.bssid))
    {
        StopTimer(c, T_OP);
        const u16 status = G16(body, 2);
        if (!status)
        {
            c.aid = G16(body, 4) & 0x0FFF;
            CW(c, R_AIDFull, c.aid);
            memcpy(c.pmac, sa, 6);
            c.wl = 0x40;
            CW(c, R_PowerTX, CR(c, R_PowerTX) | 2);
        }
        ConnectStep(c, status ? 0xC : 0, status);
    }
    else if (sub == 0xC && bl >= 2 && c.wl > 0x20 && MacEq(sa, c.pmac))
    {
        c.wl = 0x20; c.aid = 0; CW(c, R_AIDLow, 0); CW(c, R_AIDFull, 0);
        IndMacReason(c, 0x0085, sa, G16(body, 0));
    }
    else if (sub == 0xA && bl >= 2 && c.wl == 0x40 && MacEq(sa, c.pmac))
    {
        c.wl = 0x30; c.aid = 0; CW(c, R_AIDLow, 0); CW(c, R_AIDFull, 0);
        IndMacReason(c, 0x0088, sa, G16(body, 0));
    }
}
// a LOC2 management frame finished (TXStat 0x1001); RAM hdr[0] = 1: sent
void MgmtDone(Ctx& c)
{
    if (!c.mgmtBusy) return;
    c.mgmtBusy = false;
    const Mgmt& m = c.mgmtCur;
    const u8* to = &m.f[0x10];
    if (m.kind == MK_AUTH_RESP && m.status == 0)
    {
        if (Sta* p = PendingByMac(c, to)) { p->state = 0x30; IndMacReason(c, 0x0084, to, 0); }
    }
    else if (m.kind == MK_ASSOC_RESP && m.status == 0)
    {
        if (Sta* p = PendingByMac(c, to); p && m.sta)
        {
            Sta s = *p;
            s.state = 0x40; s.aid = m.sta; s.life = s.lifeReload = c.sta[0].lifeReload;
            c.sta[m.sta] = s;
            for (size_t i = 0; i < c.pending.size(); i++) if (&c.pending[i] == p) { c.pending.erase(c.pending.begin() + i); break; }
            std::vector<u8> b(0x2A, 0);
            memcpy(&b[0], s.mac, 6); B16(b, 6, s.aid & 0xFFF); B16(b, 8, 32); memcpy(&b[0xA], s.ssid, 32);
            PostInd(c, 0x0086, std::move(b));
        }
    }
    else if (m.kind == MK_ASSOC_RESP && m.sta && c.sta[m.sta].state == 0x10) c.sta[m.sta] = Sta{};
    else if (m.kind == MK_DEAUTH && c.op == OpDeauth && c.mgmtQ.empty()) { c.mgmtBusy = false; DeauthFinish(c); return; }
    else if (m.kind == MK_AUTH_REQ) { if (c.op == OpConnect && c.opStage == 2) SetTimer(c, T_OP, c.now + Ms(2000)); }
    else if (m.kind == MK_ASSOC_REQ) { if (c.op == OpConnect && c.opStage == 3) SetTimer(c, T_OP, c.now + Ms(2000)); }
    MgmtKick(c);
}

// ---- parent MP: MA_MP (B.6) ----
// WL request: replyBytes (WL "txop"), pollBitmap, deadline (10 us units, 0: none), usSample (W_USCount0 when WMSP decided),
// the body (wmHeader + data)
int MaMp(Ctx& c, u16 resume, u16 txop, u16 poll, u16 tmptt, u16 usSample, u16 len, u16 hdr, const u8* data)
{
    if (c.mode != 1) return 0xB;
    if (c.mpBusy) return 8;
    if (resume & 0x8000)
    {
        if (!(resume & 2)) txop = c.mpTxopOrig;
        if (!(resume & 4)) poll = c.mpFailed;
        if (!(resume & 8)) tmptt = c.mpTmptt;
        if (!(resume & 0x10)) len = c.mpLen;
    }
    c.mpTxopOrig = txop;
    // (txop bit 15: the reply slot in us given directly - 3.1's keep-alive MP)
    const u16 replyBytes = (txop & 0x8000) ? ((txop & 0x7FFF) - 0xD0) / 4 : txop + 1;
    const u16 txopUs = (txop & 0x8000) ? (txop & 0x7FFF) : replyBytes * 4 + 0xD0 + 2, durBase = (txop & 0x8000) ? 0xEA : 0xEA + 6;
    const u16 n = (u16)Pop(poll & 0xFFFE);
    const u16 stride = (replyBytes + 9) & ~1;
    c.mpTxopUs = txopUs; c.mpPoll = poll; c.mpGot = 0; c.mpN = n; c.mpStride = stride; c.mpTmptt = tmptt; c.mpLen = len;
    c.mpRemain = poll & 0xFFFE;
    // the result indication (0x184) with its records preset
    c.mpInd.assign(0xA + n * stride, 0);
    B16(c.mpInd, 0, c.mpRemain); B16(c.mpInd, 4, n); B16(c.mpInd, 6, stride);
    for (u16 a = 1, k = 0; a < 16; a++)
        if (poll & (1 << a)) { B16(c.mpInd, 0xA + k * stride, 0xFFFF); B16(c.mpInd, 0xA + k * stride + 4, a); k++; }
    // the CMD frame at chip RAM 0x0000
    const u16 dur = (txopUs + 10) * n + durBase;
    MW16(c, 0, 0); MW16(c, 2, poll); MW16(c, 4, 0); MW16(c, 8, 0x0014); MW16(c, 0xA, len + 0x22);
    MW16(c, 0xC, 0x0228); MW16(c, 0xE, dur);
    MWB(c, 0x10, kMpCmd, 6); MWB(c, 0x16, c.bssid, 6); MWB(c, 0x1C, c.mac, 6);
    const bool keepSeq = (resume & 0x8000) && M16(c, 0x22) != 0xFFFF;
    if (!keepSeq) MW16(c, 0x22, 0xFFFF);
    MW16(c, 0x24, txopUs); MW16(c, 0x26, poll); MW16(c, 0x28, hdr);
    if (!(resume & 0x8000) || (resume & 0x10)) MWB(c, 0x2A, data, len);
    Sentinel(c, 0x2A + len);
    CW(c, R_CmdReply, txopUs);
    CW(c, R_CmdTotal, dur);
    if (tmptt == 0)     // (WL 2.74: + 1000 us unless W_ID is 0x1440)
        CW(c, R_CmdCount, (u16)(((len + 0x22) * 4 + 0x60 + (c.v31 && CR(c, 0x000) != 0x1440 ? 1000 : 0) + txopUs * n + 0x388 + 0x32) / 10));
    else
    {
        const u16 el = (u16)(((CR(c, R_US0) + 0x10000 - usSample) & 0xFFFF) / 10);
        if (el + 3 >= tmptt) { c.mpInd.clear(); return 5; }
        CW(c, R_CmdCount, tmptt - el - 1);
    }
    c.mpBusy = true;
    c.stMp++;
    CW(c, R_CmdSlot, 0x8000 | (keepSeq ? 0x4000 : 0));
    return 0;
}
void RxReply(Ctx& c, const u8* h, const u8* f, u32 len)
{
    if (c.mode != 1 || !c.mpBusy || !MacEq(f + 4, c.bssid)) return;
    if (len < 0x18 || len - 0x18 > (u32)c.mpStride - 8) return;
    Sta* s = StaByMac(c, f + 10);
    if (!s) { if (!QueuedTo(c, f + 10, 0x00C0)) SendDeauth(c, f + 10, 7); return; }
    s->life = s->lifeReload;
    const u16 bit = 1 << s->aid;
    if ((c.mpGot & bit) || !(c.mpPoll & bit)) return;
    c.mpGot |= bit; c.mpRemain &= ~bit;
    B16(c.mpInd, 0, c.mpRemain);
    int k = 0;
    for (u16 a = 1; a < s->aid; a++) k += (c.mpPoll >> a) & 1;
    const u32 r = 0xA + k * c.mpStride;
    B16(c.mpInd, r, (u16)(len - 0x18));
    c.mpInd[r + 2] = h[6]; c.mpInd[r + 3] = h[0xA];
    memcpy(&c.mpInd[r + 8], f + 0x18, std::min<u32>(len - 0x18, c.mpStride - 8));
}
void MpEnd(Ctx& c)
{
    if (!c.mpBusy) return;
    c.mpNext = c.now + Us(c.bd ? c.bd->mpGapUs : 1100);
    RxRing(c);                                  // (late replies)
    c.mpBusy = false;
    B16(c.mpInd, 2, c.mpRemain ^ M16(c, 2));
    B16(c.mpInd, 8, 1);
    c.mpFailed = c.mpRemain;
    PostInd(c, 0x0184, std::move(c.mpInd));
    c.mpInd = {};
}

// ---- child MP (C.5) ----
void ReplyRelease(Ctx& c)
{
    for (int i = 0; i < 2; i++)
    {
        const u32 a = i ? 0x234 : 0x000;
        if (c.bufBusy[i] && (M16(c, a) & 1) && CR(c, R_Reply2) != (0x8000 | (a >> 1))) c.bufBusy[i] = false;
    }
}
int MaKeyData(Ctx& c, u16 len, u16 hdr, const u8* data)
{
    if (c.mode != 2) return 0xB;
    if (len > 0x204) return 5;
    const int i = c.bufBusy[0] ? 1 : 0;
    if (c.bufBusy[i] || (CR(c, R_Reply1) & 0x8000)) return 8;
    const u32 a = i ? 0x234 : 0x000;
    MW16(c, a + 0, 0); MW16(c, a + 4, 0); MW16(c, a + 8, 0x0014); MW16(c, a + 0xA, len + 0x1E);
    MW16(c, a + 0xC, 0x0118);
    MWB(c, a + 0x10, c.bssid, 6); MWB(c, a + 0x16, c.mac, 6); MWB(c, a + 0x1C, kMpReply, 6);
    MW16(c, a + 0x24, hdr);
    if (len) MWB(c, a + 0x26, data, len);
    Sentinel(c, a + 0x24 + len + 2);
    c.bufBusy[i] = true;
    CW(c, R_Reply1, 0x8000 | (a >> 1));
    if (!c.nullKey) CW(c, R_AIDLow, c.aid);
    return 0;
}
std::vector<u8> RxBody(Ctx& c, const u8* h, const u8* f, u32 len, u16 flags, u16 ts)
{
    std::vector<u8> b(0x14 + len, 0);
    B16(b, 2, 1);
    B16(b, 6, (u16)(len >= 0x1C ? len - 0x1C : 0));
    B16(b, 8, flags);
    B16(b, 0xC, ts);
    b[0xE] = h[6]; b[0xF] = h[0xA];
    B16(b, 0x10, (u16)len);
    b[0x12] = h[0xA]; b[0x13] = h[0xB];
    memcpy(&b[0x14], f, len);
    return b;
}
void RxCmd(Ctx& c, const u8* h, const u8* f, u32 len, u16 ts)
{
    if (c.mode != 2) return;
    const u16 seq = G16(f, 22), fc = G16(f, 0);
    if (seq == c.lastCmdSeq && (fc & 0x0800)) return;
    if (c.wl != 0x40) return;
    ReplyRelease(c);
    if (c.nullKey) {}                          // (null-key MP mode: W_AIDLow stays, every CMD is indicated)
    else if (CR(c, R_AIDLow) != 0 && (CR(c, R_Reply2) & 0x8000))
        SetTimer(c, T_AIDCLR, c.now + Us(G16(f, 2)));
    else
    {
        MW16(c, 0x1F7E, 0xFFFF); CW(c, 0x24C, 0xFFFF); CW(c, 0x24E, 0xFFFF);
        c.lastCmdSeq = 0xFFFF;
        return;
    }
    c.lastCmdSeq = seq;
    if (!MacEq(f + 10, c.bssid) || !MacEq(f + 16, c.pmac)) return;
    c.sta[1].life = c.sta[1].lifeReload;
    const u8* body = f + 24;
    const u16 poll = G16(body, 2);
    u16 flags = (poll & (1 << c.aid)) ? 0x2000 : 0;
    const u16 r2 = CR(c, R_Reply2);
    if ((r2 & 0x8000) && RAM(c)[((r2 & 0xFFF) << 1) + 4]) flags |= 0x4000;
    c.cmdFlags = flags;
    flags |= h[0] | h[1] << 8;
    flags |= (r2 & 0x8000) >> 4;
    flags |= (CR(c, R_Reply1) & 0x8000) >> 3;
    std::vector<u8> b = RxBody(c, h, f, len, flags, ts);
    B16(b, 0xA, (u16)(ts + (((G16(body, 0) + 10) * Pop(poll) + 0xFC) >> 4)));
    PostInd(c, 0x0182, std::move(b));
}
void RxAck(Ctx& c, const u8* h, const u8* f, u32 len, u16 ts)
{
    if (c.mode != 2 || c.wl != 0x40) return;
    if (!c.nullKey && (CR(c, R_AIDLow) == 0 || !(CR(c, R_Reply2) & 0x8000))) return;
    if (!MacEq(f + 10, c.bssid) || !MacEq(f + 16, c.pmac)) return;
    u16 flags = (h[0] | h[1] << 8) | c.cmdFlags | ((CR(c, R_Reply2) & 0x8000) >> 4) | ((CR(c, R_Reply1) & 0x8000) >> 3);
    PostInd(c, 0x0185, RxBody(c, h, f, len, flags, ts));
}

// ---- join / auth / assoc (child, C.1-C.3) ----
void JoinBeacon(Ctx& c, const u8* h, const u8* f, u32 len)
{
    StopTimer(c, T_OP);
    const u8* ie; u32 il;
    if (NintendoIE(f, len, &ie, &il)) { c.activeZone = G16(ie, 4); CW(c, R_BeaconCount2, c.activeZone); c.vtsf = G16(ie, 6); }
    else { c.activeZone = 0xFFFF; c.vtsf = 0; }
    CW(c, R_ListenInt, 2); CW(c, R_ListenCount, 0);
    CW(c, R_BeaconInt, c.beaconPeriod);
    CW(c, R_PowerDown, 3); CW(c, R_PowerTX, CR(c, R_PowerTX) | 1);
    ConnectStep(c, 0, 0);
}

// ======================================================================================================================
// chip IRQ (the guest's 0x0600D920)
// ======================================================================================================================
void HandleChip(Ctx& c)
{
    for (int guard = 0; guard < 64; guard++)
    {
        const u16 r = CR(c, R_IF) & CR(c, R_IE);
        if (!r) break;
        if (r & 0x8000) CW(c, R_IF, 0x8000);
        if (r & 0x4000)                         // TBTT
        {
            CW(c, R_IF, 0x4000);
            if (c.mode == 1)
            {
                BeaconVtsf(c);
                if (c.pm) CW(c, R_BeaconCount2, c.activeZone + CR(c, R_BeaconCount2) + 1);
                c.txWindow = true;
                MgmtKick(c);
            }
        }
        if (r & 0x2000) { CW(c, R_IF, 0x2000); if (c.mode == 1 && c.pm) c.txWindow = false; }
        if (r & 0x0800) CW(c, R_IF, 0x0800);
        if (r & 0x000C) CW(c, R_IF, r & 0x000C);
        if (r & 0x0001) { CW(c, R_IF, 0x0001); if (c.v31 && c.mode == 0) CW(c, R_RXRdCursor, CR(c, R_RXWrCursor)); else RxRing(c); }
        if (r & 0x0030) CW(c, R_IF, r & 0x0030);
        if (r & 0x0002)
        {
            CW(c, R_IF, 0x0002);
            const u16 s = CR(c, R_TXStat);
            if (s == 0x0800) CW(c, R_TXReqReset, 2);
            else if (s == 0x0301)
            {
                if (c.gameInfoPending) BeaconGameInfo(c);
                PostInd(c, 0x008C, {});
            }
            else if ((s & 0xFF) == 0x01 && (s >> 12) == 1) MgmtDone(c);
        }
        if (r & 0x1000)
        {
            CW(c, R_IF, 0x1000);
            if (c.mode == 1)
            {
                CW(c, R_TXSlotReset, 2); CW(c, R_PowerDown, 0); CW(c, R_TXReqSet, 2);
                MpEnd(c);
            }
        }
        if (r & 0x00C0) CW(c, R_IF, r & 0x00C0);
    }
}

// ======================================================================================================================
// WMSP (spec-wmsp)
// ======================================================================================================================

void PostInternal(Ctx& c, u16 api, u32 a1)
{
    if (c.reqs.size() >= 32) { Send(c, CbB(0x80, E_FIFO).P16(4, SC_FIFO_ERROR).P16(6, api)); return; }
    c.reqs.push_back({0, api, a1});
}
u16 AllowedChannel(u16 m)
{
    m &= 0x1FFF;
    if (!m) return 0;
    int lo = 0, hi = 15;
    while (lo < 16 && !(m & (1 << lo))) lo++;
    while (hi > 0 && !(m & (1 << hi))) hi--;
    if (hi - lo < 5) return 1 << lo;
    int mid = (hi + lo) / 2;
    for (int k = 0; k < hi - lo; k++) { mid += k * (2 * (k % 2) - 1); if (m & (1 << mid)) break; }
    if (hi - mid < 5 || mid - lo < 5) return (1 << lo) | (1 << hi);
    return (1 << lo) | (1 << mid) | (1 << hi);
}
// the WMGameInfo blob (CopyParentParam 0x0600044c), from the WMParentParam copy in WMStatus
std::vector<u8> GameInfo(Ctx& c)
{
    const u16 len = S16(c, ST_ppUgiLen);
    std::vector<u8> g(0x10 + ((len + 1) & ~1), 0);
    g[0] = 1;
    if (c.v4) g[2] = 1;                         // (4.x: gameinfo version 1)
    const u32 gg = S32(c, ST_ppGgid); const u16 tg = S16(c, ST_ppTgid);
    memcpy(&g[4], &gg, 4); memcpy(&g[8], &tg, 2);
    g[0xA] = (u8)len;
    g[0xB] = (S16(c, ST_ppEntry) ? 1 : 0) | (S16(c, ST_ppMB) ? 2 : 0) | (S16(c, ST_ppKS) ? 4 : 0) | (!c.v31 && S16(c, ST_ppCS) ? 8 : 0);
    const u16 ps = S16(c, ST_ppPSize), cs = c.v4 && S16(c, ST_ppMB) ? std::min<u16>(S16(c, ST_ppCSize), 8) : S16(c, ST_ppCSize);
    memcpy(&g[0xC], &ps, 2); memcpy(&g[0xE], &cs, 2);
    if (len) ReadA9(c, S32(c, ST_ppUgi), &g[0x10], (len + 1) & ~1);
    g.resize(0x10 + len);
    return g;
}
void PrioRestore(Ctx&) {}     // (thread priorities: no native meaning)
void CancelVAlarm(Ctx& c) { ArmVLine(c, -1); }
void FillSeqRow(Ctx& c, u16 aid) { for (int i = 0; i < 8; i++) SS16(c, ST_seq + aid * 0x10 + 2 * i, 0x0001); }

// ---- send queue (4.x) ----
u32 QE(u16 i) { return ST_q + 32u * i; }
void QAppend(Ctx& c, u32 list, u16 i)
{
    SS16(c, QE(i), 0xFFFF);
    const u16 tail = S16(c, list + 2);
    if (tail == 0xFFFF) SS16(c, list, i); else SS16(c, QE(tail), i);
    SS16(c, list + 2, i);
}
void InitSendQueue(Ctx& c)
{
    for (u32 o = 0; o < 0x400; o += 4) SS32(c, ST_q + o, 0);
    for (u16 i = 0; i < 32; i++) SS16(c, QE(i), i < 31 ? i + 1 : 0xFFFF);
    SS16(c, ST_free, 0); SS16(c, ST_free + 2, 31);
    for (u32 p = 0; p < 4; p++) { SS32(c, ST_sendQ + 4 * p, 0xFFFFFFFF); SS32(c, ST_readyQ + 4 * p, 0xFFFFFFFF); }
}
void PortSendCb(Ctx& c, u16 i, u16 err)
{
    const u32 e = QE(i);
    Send(c, CbB(0x81, err).P16(8, SC_PORT_SEND).P16(0xA, S16(c, e + 2)).P16(0xC, S16(c, e + 4)).P16(0xE, S16(c, e + 6))
                 .P16(0x10, S16(c, e + 8)).P32(0x14, S32(c, e + 0x14)).P16(0x18, S16(c, e + 0xE)).P16(0x1A, S16(c, e + 0x10))
                 .P32(0x1C, S32(c, e + 0x18)).P32(0x20, S32(c, e + 0x1C)).MySz(c, 0x24));
}
u16 PutSendQueue(Ctx& c, u16 conn, u16 prio, u16 port, u16 dest, u32 data, u16 size, u32 cb, u32 arg)
{
    if (!size || size + ((port & 8) ? 2 : 0) > (c.v4 ? 0x204 : S16(c, ST_maxSend))) return E_INVALID;
    if (c.pw && !S32(c, ST_mpFlag)) return E_SEND_FAILED;
    const u16 i = S16(c, ST_free);
    if (i == 0xFFFF) return E_QFULL;
    SS16(c, ST_free, S16(c, QE(i)));
    if (S16(c, ST_free + 2) == i) SS16(c, ST_free + 2, 0xFFFF);
    const u32 e = QE(i);
    SS16(c, e + 2, port); SS16(c, e + 4, dest); SS16(c, e + 6, dest & conn); SS16(c, e + 8, 0); SS16(c, e + 0xA, 0);
    SS32(c, e + 0x14, data); SS16(c, e + 0xE, size); SS32(c, e + 0x18, cb); SS32(c, e + 0x1C, arg);
    SS16(c, e + 0x10, 0xFFFF); SS16(c, e + 0x12, S16(c, ST_retry));
    QAppend(c, ST_sendQ + 4 * (prio & 3), i);
    return E_OPERATING;
}
void FreeEntry(Ctx& c, u16 i) { QAppend(c, ST_free, i); if (S16(c, ST_free) == 0xFFFF) SS16(c, ST_free, i); }
void CleanSendQueue(Ctx& c, u16 bitmap)
{
    const u16 keep = S16(c, ST_conn) & ~bitmap;
    for (int pass = 0; pass < 2; pass++)          // (this build walks the waiting queues twice)
        for (u32 p = 0; p < 4; p++)
        {
            const u32 list = ST_sendQ + 4 * p;
            u16 prev = 0xFFFF, i = S16(c, list);
            while (i != 0xFFFF)
            {
                const u32 e = QE(i);
                const u16 next = S16(c, e);
                SS16(c, e + 6, S16(c, e + 6) & keep); SS16(c, e + 0xA, S16(c, e + 0xA) & keep);
                if (!S16(c, e + 6))
                {
                    PortSendCb(c, i, E_OK);
                    if (prev == 0xFFFF) SS16(c, list, next); else SS16(c, QE(prev), next);
                    if (S16(c, list + 2) == i) SS16(c, list + 2, prev);
                    FreeEntry(c, i);
                }
                else prev = i;
                i = next;
            }
        }
}
// Pack (0x06007394): the MP body (sendBuf image) from the waiting queues; returns 1 if nothing may be packed now
int Pack(Ctx& c, u16 pollMask, u16 conn, u16 ready, u16* hdr, u16* sentDest, std::vector<u8>& out)
{
    s32 cap = c.v31 ? N16(c, N_sendSize) : S16(c, ST_maxSend);
    if (c.v31 && S16(c, ST_sendSize) < ((cap + 0x1F) & ~0x1F)) return 1;
    if (S16(c, ST_aid) >= 16) return 1;
    const u16 state = S16(c, ST_state);
    const bool parent = state == S_MP_PARENT;
    if (!parent && state != S_MP_CHILD) return 1;
    *hdr = 0; *sentDest = 0; out.clear();
    u16 filterSet = ready; bool filter = parent;
    if (parent && S16(c, ST_minPoll)) { filterSet = conn; filter = false; }
    const u16 rp = ready & pollMask;
    u16 blocked = 0; bool first = true;
    if (S32(c, ST_inFlight) == 1) return 1;
    SS32(c, ST_inFlight, 1);
    const u16 myAid = S16(c, ST_aid);
    for (u32 p = 0; p < 4 && cap > 2; p++)
    {
        const u32 list = ST_sendQ + 4 * p;
        u16 prev = 0xFFFF, i = S16(c, list);
        while (i != 0xFFFF && cap > 2)
        {
            const u32 e = QE(i);
            const u16 next = S16(c, e);
            const u16 port = S16(c, e + 2), bit = 1 << (port & 0xF);
            if (blocked & bit) { prev = i; i = next; continue; }
            blocked |= bit;
            const u16 d = S16(c, e + 6) & filterSet;
            if (filter && (d & ~rp)) { prev = i; i = next; continue; }
            const u16 dw = c.v4 ? S16(c, e + 6) : d;    // (4.x: the dest word is the entry's rest bitmap)
            const bool seqf = (port & 8) != 0, destf = parent && ((dw | 1) != 0xFFFF);
            u16 size = S16(c, e + 0xE);
            if (size & 1) { size++; SS16(c, e + 0xE, size); }
            const s32 need = size + (first ? 0 : 2) + (seqf ? 2 : 0) + (destf ? 2 : 0);
            if (need > cap) { prev = i; i = next; continue; }
            size_t hp;                          // (index of the header in out; ~0: the frame's wmHeader)
            if (!first) { hp = out.size(); out.push_back(0); out.push_back(0); cap -= 2; }
            else hp = ~(size_t)0;
            u16 h = ((port << 8) & 0xF00) | ((size >> 1) & 0xFF);
            const size_t o = out.size();
            out.resize(o + size);
            ReadA9(c, S32(c, e + 0x14), &out[o], size);
            cap -= size;
            if (seqf)
            {
                u16 s;
                if (S16(c, e + 0x10) & 1)
                {
                    const u32 so = ST_seq + myAid * 0x10 + 2 * (port & 7);
                    s = S16(c, so); SS16(c, so, s + 1);
                    SS16(c, e + 0x10, (u16)(s << 1));
                }
                else s = S16(c, e + 0x10) >> 1;
                out.push_back((s & 0x7FFF) & 0xFF); out.push_back((s & 0x7FFF) >> 8);
                cap -= 2;
            }
            if (destf) { h |= 0x1000; out.push_back(dw & 0xFF); out.push_back(dw >> 8); cap -= 2; }
            if (hp == ~(size_t)0) *hdr |= h; else { out[hp] = h & 0xFF; out[hp + 1] = h >> 8; }
            first = false;
            // unlink -> ready queue
            if (prev == 0xFFFF) SS16(c, list, next); else SS16(c, QE(prev), next);
            if (S16(c, list + 2) == i) SS16(c, list + 2, prev);
            QAppend(c, ST_readyQ + 4 * p, i);
            SS16(c, e + 0xA, d);
            blocked &= ~bit;
            *sentDest |= S16(c, e + 4);
            i = next;
            if (S16(c, ST_single) == 1) { p = 4; break; }
        }
    }
    return 0;
}
// SendComplete (0x06006f18): returns 1 if anything went back to the waiting queues
int SendComplete(Ctx& c, bool failAll, u16 failBitmap)
{
    const u16 state = S16(c, ST_state);
    const bool parent = state == S_MP_PARENT;
    if (!parent && state != S_MP_CHILD) return 0;
    if (S32(c, ST_inFlight) == 0) return 0;
    const u16 conn = parent ? S16(c, ST_conn) : 1;
    int requeued = 0;
    for (u32 p = 0; p < 4; p++)
    {
        const u32 rl = ST_readyQ + 4 * p;
        u16 th = 0xFFFF, tt = 0xFFFF;
        u16 i = S16(c, rl);
        SS32(c, rl, 0xFFFFFFFF);
        while (i != 0xFFFF)
        {
            const u32 e = QE(i);
            const u16 next = S16(c, e);
            if (!failAll) { SS16(c, e + 8, S16(c, e + 8) | (S16(c, e + 0xA) & ~failBitmap)); SS16(c, e + 6, S16(c, e + 6) & ~S16(c, e + 8)); }
            SS16(c, e + 6, S16(c, e + 6) & conn);
            SS16(c, e + 0xA, 0);
            const u16 rest = S16(c, e + 6), retries = S16(c, e + 0x12);
            if (rest && ((S16(c, e + 2) & 8) || retries))
            {
                if (retries) SS16(c, e + 0x12, retries - 1);
                SS16(c, e, 0xFFFF);
                if (tt == 0xFFFF) th = i; else SS16(c, QE(tt), i);
                tt = i;
            }
            else
            {
                PortSendCb(c, i, rest ? E_SEND_FAILED : E_OK);
                FreeEntry(c, i);
            }
            i = next;
        }
        if (th != 0xFFFF)
        {
            const u32 sl = ST_sendQ + 4 * p;
            SS16(c, QE(tt), S16(c, sl));
            if (S16(c, sl + 2) == 0xFFFF) SS16(c, sl + 2, tt);
            SS16(c, sl, th);
            requeued = 1;
        }
    }
    SS32(c, ST_inFlight, 0);
    return requeued;
}
// TimeLeft (0x06006a10): 10-us units until V-count 0x102, 0 if the MP would not fit
u16 TimeLeft(Ctx& c, u16 len, u16 tmptt, u16 poll)
{
    // (3.1: the reply slot's real time; the V-count target from parentVCount)
    const u32 per = !c.v31 ? tmptt : (tmptt & 0x8000) ? (tmptt & 0x7FFF) : (tmptt + 0x1C) * 4 + 0x66;
    const u32 need = (len + 0x22) * 4 + 0x60 + Pop(poll) * per + 0x388;
    s32 lines = (c.v31 ? (s32)(s16)N16(c, N_pVC) - 2 : 0x102) - (s32)c.nds.GPU.VCount;
    while (lines < 0) lines += 0x107;
    const s32 t = lines * 0x7F / 20;
    return (u16)((t * 10 < (s32)need) ? 0 : t);
}
void SendMaMP(Ctx& c, u16 poll)
{
    const u16 conn = S16(c, ST_conn), ready = S16(c, ST_ready);
    u16 hdr = 0, sentDest = 0, tmptt, txop;
    std::vector<u8>& body = c.mpBody;
    if (S16(c, ST_newFrame) == 1) { poll = 0xFFFF; SS16(c, ST_newFrame, 0); }
    if (S16(c, ST_keepAlive) == 0)
    {
        if (Pack(c, poll, conn, ready, &hdr, &sentDest, body) == 1)
        {
            if (c.v31) { SS16(c, ST_count, 0); SS16(c, ST_limit, 0); }
            return;
        }
        tmptt = (c.v31 ? N16(c, N_recvSize) : S16(c, ST_maxRecv)) + 2;
        if (S16(c, ST_minPoll) == 1) poll = sentDest;
        poll &= conn;
        if (c.v31 && S16(c, ST_rbSize) < (((N16(c, N_recvSize) + 0xC) * Pop(poll) + 0x29) & ~0x1F))
        {
            SendComplete(c, false, poll);
            SS16(c, ST_count, 0); SS16(c, ST_limit, 0);
            return;
        }
        if ((s16)S16(c, ST_count) == 1 || (s16)S16(c, ST_limit) == 1) { txop = TimeLeft(c, (u16)body.size(), tmptt, poll); hdr |= 0x8000; }
        else txop = 0;
    }
    else
    {
        SS16(c, ST_keepAlive, 0);
        poll = conn; txop = 0; body.clear(); hdr &= ~0x8000; tmptt = c.v31 ? 0x80D6 : 0;
        SS16(c, ST_count, S16(c, ST_count) + 1); SS16(c, ST_limit, S16(c, ST_limit) + 1);
    }
    MaMp(c, 0, tmptt, poll, txop, CR(c, R_US0), (u16)body.size(), hdr, body.data());
    SS16(c, ST_lastPoll, poll); SS16(c, ST_lastHdr, hdr); SS16(c, ST_lastTmptt, tmptt); SS16(c, ST_lastLen, (u16)body.size());
}
void ResendMaMP(Ctx& c, u16 arg)
{
    const u16 poll = arg & S16(c, ST_conn);
    if (c.v31 && S16(c, ST_rbSize) < (((N16(c, N_recvSize) + 0xC) * Pop(poll) + 0x29) & ~0x1F))
    {
        // (the guest sleeps 2 ms and asks again)
        c.mpNext = std::max(c.mpNext, c.now + Ms(2));
        PostInternal(c, IdResend(c), S16(c, ST_lastPoll));
        SS16(c, ST_resend, 1);
        return;
    }
    u16 hdr = S16(c, ST_lastHdr), txop;
    if ((s16)S16(c, ST_count) == 1 || (s16)S16(c, ST_limit) == 1) { txop = TimeLeft(c, S16(c, ST_lastLen), S16(c, ST_lastTmptt), poll); hdr |= 0x8000; }
    else { txop = 0; hdr &= ~0x8000; }
    MaMp(c, 0x800C, 0, poll, txop, CR(c, R_US0), 0, hdr, nullptr);
}
void SendMaKeyData(Ctx& c)
{
    if (S16(c, ST_conn) == 0) return;
    KeyPend(c, 1);
    u16 hdr = 0, sd = 0;
    std::vector<u8>& body = c.mpBody;
    if (Pack(c, 1, 1, 1, &hdr, &sd, body) == 1) { KeyPend(c, 0); return; }
    if (S32(c, ST_vsync) == 1) hdr |= 0x8000;
    const int r = MaKeyData(c, (u16)body.size(), hdr, body.data());
    if (r != 0 && r != 8) KeyPend(c, 0);
}
// ParsePortPacket (0x06006ab0)
void ParsePortPacket(Ctx& c, u16 aid, u16 wmHeader, u32 data, s32 len, u32 rb, u16 rssi)
{
    const u16 myAid = S16(c, ST_aid);
    if (len == 0 || len > (c.v4 ? 0x204 : (s32)S16(c, ST_maxRecv)) || aid == myAid || aid >= 16 || (len & 1)) return;
    bool first = true;
    while (len > 0)
    {
        u16 h;
        if (first) h = wmHeader; else { h = Rd16(c, data); data += 2; len -= 2; }
        first = false;
        u32 size = (h & 0xFF) * 2; if (!size) size = 0x200;
        const bool seqf = h & 0x800, destf = h & 0x1000;
        const u16 port = (h >> 8) & 0xF;
        len -= size + (seqf ? 2 : 0) + (destf ? 2 : 0);
        if (len < 0) return;
        const u32 p = data; u32 q = data + size;
        data += size + (seqf ? 2 : 0) + (destf ? 2 : 0);
        u16 seqOut = 0xFFFF;
        if (seqf)
        {
            const u16 s = Rd16(c, q); q += 2;
            const u32 so = ST_seq + aid * 0x10 + 2 * (port & 7);
            const u16 slot = S16(c, so);
            if (slot & 1) { SS16(c, so, (u16)(s << 1)); seqOut = s; }
            else
            {
                const u16 s2 = (u16)(s << 1);
                if ((u16)(slot - s2) < 0x100) continue;
                SS16(c, so, s2); seqOut = s2;
            }
        }
        if (destf && !(Rd16(c, q) & (1 << myAid))) continue;
        if (size > 0)
            Send(c, CbB(0x82, E_OK).P16(4, SC_PORT_RECV).P16(6, port).P32(8, rb).P32(0xC, p).P16(0x10, (u16)size).P16(0x12, aid)
                     .P16(0x20, myAid).P16(0x1A, seqOut).MySz(c, 0x40).P16(c.pw ? 0x3E : 0x20, c.pw ? rssi : myAid));
    }
}

// ---- MP indications (3.5) ----
void IndMpEnd(Ctx& c, const std::vector<u8>& B)
{
    if (!S32(c, ST_mpFlag)) return;
    if (G16(B, 2) != 0 || (S16(c, ST_resend) == 1 && G16(B, 0) != 0))
    {
        PostInternal(c, IdResend(c), S16(c, ST_lastPoll));
        SS16(c, ST_resend, 1);
        return;
    }
    if (S16(c, ST_resend)) SS16(c, ST_resend, 0);
    const u16 sel = S16(c, ST_rbSel);
    const u32 rb = S32(c, ST_rb0 + 4 * sel);
    const u16 count = G16(B, 4), size = G16(B, 6);
    const u32 n = std::min<u32>((u32)count * size + 0xA, S16(c, ST_rbSize));
    WriteA9(c, rb, B.data(), std::min<u32>(n, (u32)B.size()));
    // min RSSI of the period
    if (Rd16(c, rb) != 0) { if (!c.v31) SS16(c, ST_minRssi, std::min<u16>(S16(c, ST_minRssi), 4)); }
    else for (u32 i = 0; i < count; i++) SS16(c, ST_minRssi, std::min<u16>(S16(c, ST_minRssi), Conv(B[0xA + i * size + 3])));
    const u64 now = Tick(c) | 1;
    u16 fail = Rd16(c, rb);
    for (u32 i = 0; i < count; i++)
    {
        const u32 e = rb + 0xA + i * size;
        const u16 aid = G16(B, 0xA + i * size + 4);
        if (aid < 1 || aid > 15) continue;
        const u16 len = G16(B, 0xA + i * size);
        if (len >= 2 && len != 0xFFFF)
        {
            Wr16(c, e, len - 2);
            SS16(c, ST_ready, S16(c, ST_ready) | (1 << aid));
            SS64(c, ST_lastRx + 8 * aid, now);
            if (len - 2) ParsePortPacket(c, aid, G16(B, 0xA + i * size + 8), e + 0xA, len - 2, rb, Conv(B[0xA + i * size + 3]));
        }
        else if (len == 0)
        {
            if (c.v31) fail |= 1 << aid;        // (an empty reply: the child did not take the data)
            const u64 last = S64(c, ST_lastRx + 8 * aid), life = S64(c, ST_life);
            if (last && life && (s64)(now - last) > (s64)life)
            {
                if (g_Dbg) fprintf(stderr, "A7WM inst%d f%u parent lifetime aid %u: now %llx last %llx life %llx\n", c.inst, c.nds.NumFrames, aid, (unsigned long long)now, (unsigned long long)last, (unsigned long long)life);
                SS64(c, ST_lastRx + 8 * aid, 0); PostInternal(c, 0x25, (1u << aid) | (0x8001u << 16));
            }
        }
    }
    SendComplete(c, false, fail);
    Send(c, CbB(0x0E, E_OK).P16(4, SC_MPEND).P32(8, rb));
    SS16(c, ST_rbSel, sel ^ 1);
    if (Rd16(c, rb) == 0) SS16(c, ST_count, S16(c, ST_count) - 1);
    if ((s16)S16(c, ST_limit) > 0) SS16(c, ST_limit, S16(c, ST_limit) - 1);
    if ((s16)S16(c, ST_count) > 0 && (s16)S16(c, ST_limit) > 0)
    {
        const u16 arg = Rd16(c, rb) ? G16(B, 0) : 0xFFFF;
        if (c.v31 && N16(c, N_pInt)) { c.kickAt[0] = c.now + N64(c, N_pTick) * 64; c.kickArg[0] = arg; }   // (parentInterval alarm)
        else PostInternal(c, IdMP(c), arg);
    }
}
void IndMpInd(Ctx& c, const std::vector<u8>& B)
{
    const u16 r = Conv(B[0xF]);
    if (S16(c, ST_minRssi) > r) SS16(c, ST_minRssi, r);
    if (!S32(c, ST_mpFlag)) return;
    if (S16(c, ST_c032) == 1) SS16(c, ST_c032, 0);
    const u16 old8e = S16(c, ST_rxA);
    const u16 sel = S16(c, ST_rbSel) ^ 1;
    SS16(c, ST_rbSel, sel);
    const u32 rb = S32(c, ST_rb0 + 4 * sel);
    WriteA9(c, rb, B.data(), std::min<u32>(std::min<u32>(G16(B, 6) + 0x30, S16(c, ST_rbSize)), (u32)B.size()));
    const bool pending = S16(c, ST_waitAck) == 1;
    if (pending) StopTimer(c, T_ACKTO);
    SS16(c, ST_waitAck, 1);
    SS16(c, ST_ackTs, Rd16(c, rb + 0xA));
    const u16 f = G16(B, 8);
    const bool polled = (f & 0x2000) != 0;
    SS16(c, ST_polled, polled);
    {
        const s64 ticks = (s64)(s32)(((u32)(u16)(Rd16(c, rb + 0xA) - Rd16(c, rb + 0xC)) + 0x80) * 16 * 0x82EA / 64 / 1024);
        SetTimer(c, T_ACKTO, c.now + (u64)std::max<s64>(ticks, 1) * 64);
    }
    // (5.x: the "empty" flag is rxFlags 0x0400 - ponytail: WL 2.88 sets it for what 3.x/4.x flag as 0x0800, key data out;
    //  the MP path of this build is from its code only, no connected trace)
    const bool a = c.pw ? (f & 0x0800) != 0 : (f & 0x2800) == 0x2800, b = (f & 0x6000) == 0x6000;
    if (b) KeyPend(c, 0);
    SS16(c, ST_rxA, a); SS16(c, ST_rxB, b);
    if (c.v31 && polled)        // (the child's send size follows the parent's reply slot)
    {
        s32 n = ((s32)Rd16(c, rb + 0x2C) - 0x66) / 4 - 0x20;
        if (n >= 0) { n = std::min<s32>(n, c.v4 ? 0x200 : N16(c, N_cMax)); if (n != N16(c, N_cSize)) SetCSize(c, (u16)n); }
    }
    if (pending)
    {
        if (old8e == 1) SendComplete(c, true, 0);
        Send(c, CbB(0x0E, E_TIMEOUT).P16(4, SC_MPACK).P32(8, 0));
    }
    if (!polled) { if (Rd16(c, rb + 6) >= 2) SS16(c, ST_c030, (Rd16(c, rb + 0x30) & 0x8000) ? 1 : 0); return; }
    const u16 len = Rd16(c, rb + 6);
    if (len >= 2)
    {
        Wr16(c, rb + 6, len - 2);
        SS16(c, ST_c030, (Rd16(c, rb + 0x30) & 0x8000) ? 1 : 0);
        Send(c, CbB(0x0E, E_OK).P16(4, SC_MP_IND).P32(8, rb));
        if (len - 2) ParsePortPacket(c, 0, Rd16(c, rb + 0x30), rb + 0x32, len - 2, rb, Conv(B[0xF]));
    }
    else
    {
        Wr16(c, rb + 6, 0); SS16(c, ST_c030, 0);
        Send(c, CbB(0x0E, E_NO_DATA).P16(4, SC_MP_IND).P32(8, rb));
    }
    if (S64(c, ST_life)) SS64(c, ST_lastRx, Tick(c) | 1);
}
void IndMpAck(Ctx& c, const std::vector<u8>& B, bool fake)
{
    if (!S32(c, ST_mpFlag)) return;
    if (fake)
    {
        const u32 us = (u32)(USCount(c) >> 4);
        if ((s16)(u16)(us - S16(c, ST_ackTs)) <= 0) return;
    }
    if (S16(c, ST_waitAck) == 0) return;
    SS16(c, ST_waitAck, 0);
    const bool polled = S16(c, ST_polled);
    StopTimer(c, T_ACKTO);
    const u16 aid = S16(c, ST_aid);
    const u16 ackBmp = fake ? 0 : G16(B, 0x2E);
    bool fail = true;
    if (S16(c, ST_rxB)) fail = fake ? false : ((ackBmp & (1 << aid)) != 0);
    if (!c.v31 && polled && (fake || fail)) SS16(c, ST_minRssi, 4);
    if (S16(c, ST_rxB)) SS16(c, ST_rxB, 0);
    int requeued = 0;
    if (S16(c, ST_rxA)) { SS16(c, ST_rxA, 0); requeued = SendComplete(c, fake, fail ? 1 : 0); }
    if (!polled) return;
    CbB cb(0x0E, fake ? E_TIMEOUT : ((ackBmp & (1 << aid)) ? E_SEND_FAILED : E_OK));
    cb.P16(4, SC_MPACK).P32(8, 0);
    if (!fake) cb.P16(0xC, G16(B, 0xC)).P16(0xE, G16(B, 0xE)).PB(0x10, &B[0x18], 6).PB(0x16, &B[0x1E], 6).P16(0x1C, G16(B, 0x2A)).P16(0x1E, G16(B, 0x2C)).P16(0x20, G16(B, 0x2E));
    Send(c, cb);
    if (requeued == 1 || S16(c, ST_c030) == 0)
    {
        // ponytail: 3.1's childInterval alarm writes these flags when it fires; here at once (the post is what is timed)
        SS16(c, ST_c030, 0); SS16(c, ST_c032, 0); SS16(c, ST_newFrame, 0);
        if (c.v31 && N16(c, N_cInt)) c.kickAt[1] = c.now + N64(c, N_cTick) * 64;   // (childInterval alarm)
        else { c.keyNext = c.now + Us(c.bd->kickChildUs); PostInternal(c, IdKey(c), 0); }
    }
    else { SS16(c, ST_c030, 0); SS16(c, ST_c032, 1); SS16(c, ST_newFrame, 0); }
}

// ---- RSSI ring / link level (0.6) ----
void AddRssi(Ctx& c, u16 r) { c.rssi[c.rssiIdx] = (u8)r; c.rssiIdx = (c.rssiIdx + 1) & (c.v4 ? 31 : 15); SeedUpdate(c, r); }
u16 AvgLinkLevel(Ctx& c) { const int n = c.v4 ? 32 : 16; u32 s = 0; for (int i = 0; i < n; i++) s += c.rssi[i]; return LinkLevelC(c, (u16)(s / n)); }

// ---- V-count alarm (5.1) ----
void PostVAlarmMP(Ctx& c)
{
    if (S16(c, ST_vaPend) == 1) return;
    SS16(c, ST_vaPend, 1);
    c.vaNext = c.now + Us(S16(c, ST_mode) == 1 ? c.bd->vaParentUs : c.bd->vaChildUs);
    PostInternal(c, 0x1C, 0);
}
void SetVCount(Ctx& c, u16 v) { c.nds.ARM7IOWrite16(0x04000006, v); }
u32 US32(Ctx& c) { return CR(c, R_US0) | (u32)CR(c, R_US1) << 16; }
// the MP V-count lines (3.1: parentVCount / childVCount, defaults the 3.0 constants)
s32 PVC(Ctx& c) { return c.v31 ? (s16)N16(c, N_pVC) : 0x104; }
s32 CVC(Ctx& c) { return c.v31 ? (s16)N16(c, N_cVC) : 0xF0; }
// the parent's V-sync line and the end of both adjust windows (4.x: 0xcb, [.., 0xd2))
s32 SyncLine(Ctx& c) { return c.v4 ? 0xCB : 0xD1; }
s32 WinEnd(Ctx& c) { return c.v4 ? 0xD2 : 0xD4; }
void VAlarm(Ctx& c, u16 x)
{
    const u16 vc = c.nds.GPU.VCount;
    if (S16(c, ST_mode) == 1)
    {
        if (x == SyncLine(c))
        {
            if (S16(c, ST_vres) >= 0x3C) { if (vc >= SyncLine(c) && vc < WinEnd(c)) { SetVCount(c, vc); SS16(c, ST_vres, 0); } }
            else SS16(c, ST_vres, S16(c, ST_vres) + 1);
            c.vtsf = (u16)(((US32(c) << 1) - vc * 0x7F) >> 7);
            ArmVLine(c, PVC(c));
        }
        else
        {
            if (S32(c, ST_mpFlag) == 1) { ArmVLine(c, SyncLine(c)); PostVAlarmMP(c); }
            else ArmVLine(c, -1);
        }
        return;
    }
    if (x == 0xC8)
    {
        SS32(c, ST_sync, c.vtsf);
        if (S32(c, ST_sync) != S32(c, ST_syncLast))
        {
            SS32(c, ST_syncLast, S32(c, ST_sync));
            // CalcOffset (0x060064f0)
            u32 cc = S32(c, ST_sync) << 6;
            const u32 us = US32(c) & 0x3FFFC0;
            const u32 t = ((((0x107 - vc) * 0x7F) + 2 * us) >> 1) & 0x3FFFC0;
            u32 ofs = 0;
            if (cc <= t)
                for (int k = 1; k <= 0x1D; k++)
                {
                    cc += 0x414B;
                    if (cc > t) { const u32 d = cc - t; ofs = d > 0x400E ? 0 : d; break; }
                }
            SS32(c, ST_vofs, ofs);
        }
        if (S32(c, ST_vofs) > 0x7F) ArmVLine(c, 0xD0);
        else { SS32(c, ST_vsync, 1); ArmVLine(c, CVC(c)); }
    }
    else if (x == 0xD0)
    {
        u32 d = S32(c, ST_vofs);
        if (vc >= 0xD0 && vc < WinEnd(c) && d >= 0x7F)
        {
            u32 k = 1;
            while (k < 7 && !(d < 0x3F * k + 0x7F)) k++;
            SetVCount(c, vc + 1 - k);
            d -= 0x3F * k;
            SS32(c, ST_vofs, d);
        }
        if (S32(c, ST_vofs) >= 0x7F) SS32(c, ST_vsync, 0);
        ArmVLine(c, CVC(c));
    }
    else
    {
        if (S32(c, ST_mpFlag) == 1)
        {
            ArmVLine(c, 0xC8);
            const u64 life = S64(c, ST_life), last = S64(c, ST_lastRx);
            if (life && last && (s64)((Tick(c) | 1) - last) > (s64)life)
            {
                if (g_Dbg) fprintf(stderr, "A7WM inst%d f%u child lifetime: now %llx last %llx life %llx\n", c.inst, c.nds.NumFrames, (unsigned long long)Tick(c), (unsigned long long)last, (unsigned long long)life);
                SS64(c, ST_lastRx, 0); PostInternal(c, 0x25, 0 | (0x8001u << 16));
            }
            else PostVAlarmMP(c);
        }
        else ArmVLine(c, -1);
    }
}
void StartVAlarm(Ctx& c)
{
    if (S16(c, ST_mode) == 1) ArmVLine(c, SyncLine(c));
    else { SS32(c, ST_vsync, 0); ArmVLine(c, 0xC8); SS32(c, ST_vofs, 0); }
}
void VAlarmMP(Ctx& c)       // internal 0x1c (0x06006868)
{
    SS16(c, ST_vaPend, 0);
    SS16(c, ST_newFrame, 1);
    const u16 n = S16(c, ST_rssiCnt) - 1;
    SS16(c, ST_rssiCnt, n);
    if (n == 0)
    {
        if (c.v31 && S16(c, ST_minRssi) == 0xFFFF) SS16(c, ST_minRssi, 4);   // (3.1: a period without a sample counts 4)
        if (S16(c, ST_minRssi) != 0xFFFF) { AddRssi(c, S16(c, ST_minRssi) & 0xFF); SS16(c, ST_link, AvgLinkLevel(c)); }
        SS16(c, ST_minRssi, 0xFFFF); SS16(c, ST_rssiCnt, c.v31 ? 1 : 4);
    }
    const u16 state = S16(c, ST_state);
    if (state == S_MP_PARENT)
    {
        if (S16(c, ST_conn) == 0) { SS16(c, ST_count, 0); return; }
        bool idle = !((s16)S16(c, ST_count) != 0 && (s16)S16(c, ST_limit) != 0);
        if (c.v4) { idle = !((s16)S16(c, ST_count) > 0 && (s16)S16(c, ST_limit) > 0); if ((s16)S16(c, ST_count) < 0) SS16(c, ST_count, 0); }
        s32 cnt = (s16)S16(c, ST_count) + S16(c, ST_freq);
        if (cnt > 0x100) cnt = 0x100;
        SS16(c, ST_count, (u16)cnt);
        SS16(c, ST_limit, c.v4 ? N16(c, A_maxFreq) : c.v31 ? N16(c, N_maxFreq) : S16(c, ST_fixFreq) ? S16(c, ST_freq) : 6);
        if (idle && (s16)S16(c, ST_count) > 0 && (s16)S16(c, ST_limit) > 0) SendMaMP(c, 0xFFFF);
        if (S16(c, ST_minPoll) == 1) { const u16 k = S16(c, ST_keepCnt) - 1; SS16(c, ST_keepCnt, k); if (!k) { SS16(c, ST_keepAlive, 1); SS16(c, ST_keepCnt, 0x3C); } }
    }
    else if (state == S_MP_CHILD)
    {
        if (S32(c, ST_inFlight) != 1) { SS16(c, ST_c032, 0); SendMaKeyData(c); }
    }
}

// ---- disconnect (3.10) ----
// returns 0 if it stopped on a WL error (callback sent), 1 done (*disc = the AIDs disconnected)
int DisconnectCore(Ctx& c, u16 bitmap, bool autoMode, u16 reason, u16* disc)
{
    const u16 state = S16(c, ST_state);
    bool wasMP = false;
    u16 d = 0;
    if (state == S_PARENT || state == S_MP_PARENT) wasMP = S32(c, ST_mpFlag) == 1;
    else if (state == S_CHILD || state == S_MP_CHILD)
    {
        if (c.v31 && S16(c, ST_conn) == 0)
        {
            if (!autoMode) Send(c, CbB(0x0D, E_ILLEGAL).P16(4, 0).P16(6, 0).P16(8, bitmap).P16(0xA, 0));
            return 0;
        }
        if (S32(c, ST_mpFlag)) { SS32(c, ST_mpFlag, 0); wasMP = true; CancelVAlarm(c); if (c.pw) SendComplete(c, false, 0); }
        if (state == S_MP_CHILD) SS16(c, ST_state, S_CHILD);
        SS16(c, ST_conn, 0); SS16(c, ST_ready, 0); SS32(c, ST_ks, 0); SS32(c, ST_dcf, 0); SS32(c, ST_vsync, 0);
    }
    else
    {
        if (!autoMode) Send(c, CbB(0x0D, E_ILLEGAL).P16(4, 0).P16(6, 0).P16(8, bitmap).P16(0xA, 0));
        return 0;
    }
    if (state == S_CHILD || state == S_MP_CHILD)
    {
        SendDeauth(c, c.pmac, 3);           // (the rest when it is out: DeauthFinish)
        c.opWasMP = wasMP; c.opAuto = autoMode; c.opReason = reason; c.opBitmap = bitmap;
        d = 1;
    }
    else
    {
        for (u16 aid = 1; aid < 16; aid++)
        {
            if (!(S16(c, ST_conn) & bitmap & (1 << aid))) continue;
            u8 m[6];
            for (int k = 0; k < 6; k++) m[k] = Rd8(c, SA(c, ST_childMac + 6 * (aid - 1)) + k);
            SendDeauth(c, m, 3);
            c.sta[aid] = Sta{};
            d |= 1 << aid;
            SS16(c, ST_conn, S16(c, ST_conn) & ~(1 << aid)); SS16(c, ST_ready, S16(c, ST_ready) & ~(1 << aid));
            SS64(c, ST_lastRx + 8 * aid, 0);
            for (int k = 0; k < 6; k++) Wr8(c, SA(c, ST_childMac + 6 * (aid - 1)) + k, 0);
            if (autoMode) Send(c, CbB(0x08, E_OK).P16(8, SC_DISCONNECTED).P16(0x12, reason).P16(0x10, aid).PB(0xA, m, 6).Sz(c, 0x2C));
            else if (c.v31) Send(c, CbB(0x08, E_OK).P16(8, SC_DISC_MYSELF).P16(0x12, 0xF001).P16(0x10, aid).PB(0xA, m, 6).Sz(c, 0x2C));
            if (wasMP) CleanSendQueue(c, 1 << aid);
        }
    }
    if (disc) *disc = d;
    return 1;
}

// the guest's request -> callback time of the short requests (us; traces of each build's guest, medians): native runs
// them that long after taking them, so state changes and callbacks land when the guest's would
u32 QuickUs(Ctx& c, u16 api)
{
    //                         SET_GAMEINFO START_MP(parent/child) END_MP SET_P_PARAM END SET_ENTRY
    static const u16 k30[] = {232, 362, 283, 400, 163, 198, 228};      // (MKDS; END_MP estimated)
    static const u16 k31[] = {280, 396, 359, 450, 181, 200, 276};      // (Shrek; SET_GAMEINFO / END_MP / END estimated)
    static const u16 k40[] = {300, 440, 647, 590, 199, 246, 223};      // (Kirby; SET_GAMEINFO / END_MP / END estimated)
    static const u16 k50[] = {351, 469, 464, 585, 220, 226, 400};      // (PB / PW; SET_ENTRY estimated)
    if (!c.bd) return 0;
    const u16* t = c.bd->gen >= 50 ? k50 : c.bd->gen >= 40 ? k40 : c.bd->gen >= 31 ? k31 : k30;
    switch (api)
    {
    case 0x18: return t[0];
    case 0x0E: return AmParent(c) ? t[1] : t[2];
    case 0x10: return t[3];
    case 0x07: return t[4];
    case 0x02: return t[5];
    case 0x21: return t[6];
    }
    return 0;
}
// requests that send deauthentications first (DISCONNECT, AUTO_DISCONNECT, END_PARENT, RESET): the guest's WL confirms a
// deauth when it is on air (the parent's only in the beacon window), then the request goes on
void OpFinish(Ctx& c);
void DeauthFinish(Ctx& c)
{
    const u16 api = c.opApi;
    const bool child = c.mode == 2;
    if ((api == 0x0D || api == 0x25) && child)
    {
        SS16(c, ST_indOn, 0);
        SS16(c, ST_state, S_CLASS1);
        MacStop(c); c.mode = 0; c.aid = 0; c.wl = 0x10; PowerUp(c);
        SS16(c, ST_state, S_IDLE);
        if (c.v31)
        {
            WepOff(c); SizesOff(c);
            u8 pm[6]; ReadA9(c, SA(c, ST_pMac), pm, 6);
            if (c.opAuto) Send(c, CbB(0x0C, E_OK).P16(8, SC_DISCONNECTED).P16(0xC, c.opReason).P16(0xA, S16(c, ST_aid)).PB(0x10, pm, 6).Sz(c, 0x16));
            else Send(c, CbB(0x0C, E_OK).P16(8, SC_DISC_MYSELF).P16(0xC, 0xF001).P16(0xA, S16(c, ST_aid)).PB(0x10, pm, 6).Sz(c, 0x16));
        }
        else
        {
            if (c.opAuto) Send(c, CbB(0x0C, E_OK).P16(8, SC_DISCONNECTED).P16(0xC, c.opReason).P16(0xA, S16(c, ST_aid)));
            SizesOff(c);
        }
        if (c.opWasMP) CleanSendQueue(c, 1);
    }
    if (api == 0x0D) Send(c, CbB(0xD, E_OK).P16(8, c.opBitmap).P16(0xA, c.opDone));
    else if (api == 0x09)
    {
        for (Sta& t : c.sta) t = Sta{};
        c.pending.clear();
        SS16(c, ST_indOn, 0); SS16(c, ST_state, S_CLASS1);
        DevIdle(c);
        SS16(c, ST_state, S_IDLE); WepOff(c); SizesOff(c);
        Send(c, CbB(0x09, E_OK));
    }
    else if (api == 0x01)
    {
        for (Sta& t : c.sta) t = Sta{};
        c.pending.clear();
        DevIdle(c);
        SS16(c, ST_preamble, 1);
        SS16(c, ST_state, S_IDLE);
        if (c.v4) { WepOff(c); SizesOff(c); }
        Send(c, CbB(0x01, E_OK));
    }
    OpFinish(c);
}
// returns false: the request waits (op OpDeauth) until the queued deauths are out (or 2 s)
bool DeauthWait(Ctx& c, const Rq& q)
{
    c.op = OpDeauth; c.opApi = q.api; c.opRq = q;
    if (c.mgmtQ.empty() && !c.mgmtBusy) { DeauthFinish(c); return true; }
    SetTimer(c, T_OP, c.now + Ms(2000));
    return false;
}

// ---- WL indications in WMSP (0.2 HandleInd) ----
void HandleInd(Ctx& c, Ind& in)
{
    if (!c.w7 || S16(c, ST_state) == S_READY) return;
    const std::vector<u8>& B = in.b;
    const u16 state = S16(c, ST_state);
    switch (in.code)
    {
    case 0x0084: Send(c, CbB(0x80, E_OK).P16(4, SC_AUTH)); break;
    case 0x0088: Send(c, CbB(0x80, E_OK).P16(4, 0x11)); break;
    case 0x0086:            // assoc -> CONNECTED (parent)
    {
        const u16 aid = G16(B, 6);
        if (aid < 1 || aid > 15) break;
        if (S16(c, ST_ppEntry) == 0)
        {
            c.reqs.push_back({0, 0x22, 0});
            c.autoDeAuth.push_back(std::vector<u8>(B.begin(), B.begin() + 6));
            break;
        }
        for (int k = 0; k < 6; k++) Wr8(c, SA(c, ST_childMac + 6 * (aid - 1)) + k, B[k]);
        SS16(c, ST_conn, S16(c, ST_conn) | (1 << aid));
        SS16(c, ST_ready, S16(c, ST_ready) & ~(1 << aid));
        SS64(c, ST_lastRx + 8 * aid, Tick(c) | 1);
        FillSeqRow(c, aid);
        Send(c, CbB(0x08, E_OK).P16(8, SC_CONNECTED).PB(0xA, &B[0], 6).P16(0x10, aid).PB(0x14, &B[0x12], 24).Sz(c, 0x2C));
        break;
    }
    case 0x0085:            // deauth
    {
        if (state == S_PARENT || state == S_MP_PARENT)
        {
            for (u16 i = 0; i < 15; i++)
            {
                bool eq = true;
                for (int k = 0; k < 6 && eq; k++) eq = Rd8(c, SA(c, ST_childMac + 6 * i + k)) == B[k];
                if (!eq) continue;
                const u16 aid = i + 1;
                if (c.v31 && !(S16(c, ST_conn) & (1 << aid))) continue;
                SS16(c, ST_conn, S16(c, ST_conn) & ~(1 << aid)); SS16(c, ST_ready, S16(c, ST_ready) & ~(1 << aid));
                SS64(c, ST_lastRx + 8 * aid, 0);
                for (int k = 0; k < 6; k++) Wr8(c, SA(c, ST_childMac + 6 * i + k), 0);
                c.sta[aid] = Sta{};
                Send(c, CbB(0x08, E_OK).P16(8, SC_DISCONNECTED).P16(0x12, G16(B, 6)).P16(0x10, aid).PB(0xA, &B[0], 6).Sz(c, 0x2C));
                if (S32(c, ST_mpFlag)) CleanSendQueue(c, 1 << aid);
                break;
            }
        }
        else
        {
            if (c.v31 && S16(c, ST_conn) == 0) break;
            bool was = false;
            if (S32(c, ST_mpFlag)) { SS32(c, ST_mpFlag, 0); CancelVAlarm(c); was = true; if (c.pw) SendComplete(c, false, 0); }
            SS16(c, ST_conn, 0); SS16(c, ST_ready, 0); SS32(c, ST_ks, 0); SS32(c, ST_dcf, 0); SS32(c, ST_vsync, 0); SS16(c, ST_indOn, 0);
            SS16(c, ST_state, S_CLASS1);
            if (c.v31) { WepOff(c); SizesOff(c); }
            Send(c, CbB(0x0C, E_OK).P16(8, SC_DISCONNECTED).P16(0xC, G16(B, 6)).P16(0xA, S16(c, ST_aid)).Sz(c, 0x16)
                         .PB(0x10, &B[0], c.v31 ? 6 : 0));
            SizesOff(c);
            if (was) CleanSendQueue(c, 1);
        }
        break;
    }
    case 0x008C: if (S16(c, ST_indOn)) Send(c, CbB(0x08, E_OK).P16(8, SC_BEACON_SENT)); break;
    case 0x008D:            // beacon received (child)
    {
        const u16 r = Conv(B[0xF]);
        SeedUpdate(c, r);
        if (state != S_CHILD && state != S_MP_CHILD) break;
        const u16 gl = G16(B, 6);
        const u16 tgid = G16(B, 0x2C + 8);
        if (S16(c, ST_ptgid) != tgid) PostInternal(c, 0x25, 1 | (0x8002u << 16));
        else if (S16(c, ST_indOn))
        {
            CbB cb(0x80, E_OK);
            cb.P16(4, SC_BEACON_RECV).P16(6, tgid).P16(8, state).P16(0xA, gl);
            if (gl <= 0x80) cb.PB(0xC, &B[0x2C], std::min<u32>((gl + 1) & ~1, (u32)B.size() - 0x2C));
            Send(c, cb);
        }
        break;
    }
    case 0x0182: IndMpInd(c, B); break;
    case 0x0184: IndMpEnd(c, B); break;
    case 0x0185: IndMpAck(c, B, B.empty()); break;
    }
}

// ======================================================================================================================
// requests (section 1)
// ======================================================================================================================
void SetAllParams(Ctx& c)
{
    // (PARAM_SET_ALL: MAC, retry 7, mode, rate, WEP off, active zone 10, SSID mask bytes 8..31, preamble)
    c.activeZone = 0x000A;
    CW(c, R_TXRetry, 7);
}
void DevIdle(Ctx& c) { if (c.wl > 0x20) MacStop(c); c.wl = 0x10; CW(c, R_PowerForce, 0); MacStop(c); c.wl = 0x10; c.mode = 0; PowerUp(c); }
void PowerOnSeq(Ctx& c)
{
    // DEV_REBOOT: power down + the default registers (0x0600892C / 0x0600B218)
    PowerDown(c);
    static const u16 kDef[][2] = {{0x004, 0}, {0x008, 0}, {0x00A, 0}, {0x012, 0}, {0x010, 0xFFFF}, {0x254, 0}, {0x0B4, 0xFFFF}, {0x080, 0}, {0x08E, 1},
        {0x088, 0}, {0x02A, 0}, {0x028, 0}, {0x0E8, 0}, {0x0EA, 0}, {0x0EE, 1}, {0x0EC, 0x3F03}, {0x1A2, 1}, {0x1A0, 0}, {0x110, 0x0800}, {0x0BC, 1},
        {0x0D4, 3}, {0x0D8, 4}, {0x0DA, 0x0602}, {0x076, 0}, {0x130, 0x0146}};
    for (auto& d : kDef) CW(c, d[0], d[1]);
    CW(c, R_TXReqSet, 0x0008);
    const u8* f = Fw(c);
    memcpy(c.mac, f + 0x36, 6);
    for (int i = 0; i < 3; i++) CW(c, R_MAC0 + 2 * i, c.mac[2 * i] | c.mac[2 * i + 1] << 8);
    CW(c, R_TXRetry, 7); CW(c, R_ModeWEP, 0x000A);
    for (u32 o = 0x1F80; o < 0x2000; o += 0x20) for (u32 i = 0; i < 0x14; i += 2) MW16(c, o + i, 0);
    CW(c, R_BeaconInt, 0x01F4); CW(c, R_Preamble, 0x0007);
    BBW(c, 0x13, 0x00); BBW(c, 0x35, 0x1F);
    CW(c, 0x290, 0); CW(c, 0x290, 0);
    c.wl = 0;
    // DEV_IDLE
    DevIdle(c);
    CW(c, 0x124, 0x00C8); CW(c, 0x128, 0x07D0); CW(c, 0x150, 0x0202);
    for (Sta& s : c.sta) s.lifeReload = 0x28;
}
void InitStatus(Ctx& c)
{
    if (S32(c, ST_mpFlag)) { SS32(c, ST_mpFlag, 0); CancelVAlarm(c); }
    SS16(c, ST_conn, 0); SS16(c, ST_ready, 0); SS32(c, ST_ks, 0); SS32(c, ST_dcf, 0); SS32(c, ST_vsync, 0);
    SS16(c, ST_minPoll, 0); SS16(c, ST_single, 0);
    if (c.v31)
    {
        NS16(c, N_minFreq, 1); NS16(c, N_freq, 1); NS16(c, N_maxFreq, 6); NS16(c, N_retry, 0); NS16(c, N_ignFatal, 0); NS16(c, N_ignSize, 0);
        NS16(c, N_minPoll, 0); NS16(c, N_single, 0);
        SizesOff(c);
        NS16(c, N_pVC, 0x104); NS16(c, N_cVC, 0xF0); NS16(c, N_pInt, 1000); NS16(c, N_cInt, 0); NS64(c, N_pTick, 0x20B); NS64(c, N_cTick, 0);
        if (c.v4) { const u16 cur[] = {1, 1, 6, 0, 0, 0, 0}; for (int i = 0; i < 7; i++) NS16(c, A_minFreq + 2 * i, cur[i]); }
    }
    else { SS16(c, ST_fixFreq, 0); SizesOff(c); }
    SS16(c, ST_vaPend, 0); SS16(c, ST_indOn, 0); SS16(c, ST_pm, 0); SS16(c, ST_preamble, 1);
    for (int i = 0; i < 32; i++) Wr32(c, c.w7 + (c.v31 ? 0xD0 : 0xF8) + 16 * i, 0x8000);
    for (u32 o = 0; o < 0x100; o += 2) SS16(c, ST_seq + o, 0x0001);
}
void ReqDone(Ctx& c, const Rq& q) { if (q.addr) Wr16(c, q.addr, q.api | 0x8000); }
// an asynchronous WL operation finished: the request's continuation
void ScanDone(Ctx& c);
void MeasureDone(Ctx& c);
void ConnectStep(Ctx& c, int result, u16 status);

u32 RqArg(Ctx& c, const Rq& q, u32 o) { return Rd32(c, q.addr + o); }
// 3.1 SetMPParameterCore (0x027e8458): p = a WMMPParam (0x1c bytes); old (if any) gets the values before the update
u16 SetMPParam(Ctx& c, const u8* p, u8* old)
{
    u16 r = E_OK;
    u32 mask; memcpy(&mask, p, 4);
    const u16 state = S16(c, ST_state);
    if ((state == S_MP_PARENT || state == S_MP_CHILD) && (mask & 0x2C00)) { r = E_ILLEGAL; mask &= ~0x2C00u; }
    if (old)
    {
        const u32 all = 0x3FFF; memcpy(old, &all, 4);
        if (c.v4) { B16v(old, 4, N16(c, N_minFreq)); B16v(old, 6, N16(c, N_freq)); B16v(old, 8, N16(c, N_maxFreq)); }
        else { const u16 f = N16(c, N_freq); B16v(old, 4, f); B16v(old, 6, f); B16v(old, 8, f); }     // (3.1 reports freq thrice)
        B16v(old, 0xA, N16(c, N_pSize)); B16v(old, 0xC, N16(c, N_cSize)); B16v(old, 0xE, N16(c, N_pInt)); B16v(old, 0x10, N16(c, N_cInt));
        B16v(old, 0x12, N16(c, N_pVC)); B16v(old, 0x14, N16(c, N_cVC)); B16v(old, 0x16, N16(c, N_retry));
        old[0x18] = (u8)N16(c, N_minPoll); old[0x19] = (u8)N16(c, N_single); old[0x1A] = (u8)N16(c, N_ignFatal); old[0x1B] = (u8)N16(c, N_ignSize);
    }
    auto g = [&](u32 o) { return G16v(p, o); };
    auto vok = [](u16 v) { return v <= 0xBE || (v >= 0xDC && v <= 0x106); };
    auto both = [&](u32 o, u32 a, u16 v) { NS16(c, o, v); if (c.v4) NS16(c, a, v); };
    if (mask & 1) both(N_minFreq, A_minFreq, g(4) ? g(4) : 16);
    if (mask & 2) { const u16 f = g(6) ? g(6) : 16; both(N_freq, A_freq, f); if ((s16)S16(c, ST_count) > (s16)f) SS16(c, ST_count, f); }
    if (mask & 4) { const u16 f = g(8) ? g(8) : 16; both(N_maxFreq, A_maxFreq, f); if ((s16)S16(c, ST_count) > (s16)f) SS16(c, ST_count, f); }
    if (mask & 8) { const u16 v = (g(0xA) + 1) & ~1; if (v > N16(c, N_pMax)) r = E_INVALID; else SetPSize(c, v); }
    if (mask & 0x10) { const u16 v = (g(0xC) + 1) & ~1; if (v > (c.v4 ? 0x200 : N16(c, N_cMax))) r = E_INVALID; else SetCSize(c, v); }
    if (mask & 0x20) { const u16 us = g(0xE); if (us > 10000) r = E_INVALID; else { NS16(c, N_pInt, us); NS64(c, N_pTick, ((u64)us * 0x82EA) >> 16); } }
    if (mask & 0x40) { const u16 us = g(0x10); if (us > 10000) r = E_INVALID; else { NS16(c, N_cInt, us); NS64(c, N_cTick, ((u64)us * 0x82EA) >> 16); } }
    if (mask & 0x80) { if (vok(g(0x12))) NS16(c, N_pVC, g(0x12)); else r = E_INVALID; }
    if (mask & 0x100) { if (vok(g(0x14))) NS16(c, N_cVC, g(0x14)); else r = E_INVALID; }
    if (mask & 0x200) both(N_retry, A_retry, g(0x16));
    if (mask & 0x400) NS16(c, N_minPoll, p[0x18]);
    if (mask & 0x800) NS16(c, N_single, p[0x19]);
    if (mask & 0x1000) both(N_ignFatal, A_ignFatal, p[0x1A]);
    if (mask & 0x2000) NS16(c, N_ignSize, p[0x1B]);
    return r;
}
// 4.x WMMPTmpParam (START_MP +0x30): the current MP parameters for this MP session
void SetMPTmpParam(Ctx& c, const u8* t)
{
    u32 m; memcpy(&m, t, 4);
    auto g = [&](u32 o) { return G16v(t, o); };
    u16 maxF = (m & 4) ? g(8) : N16(c, N_maxFreq); if (!maxF) maxF = 16;
    u16 minF = (m & 1) ? g(4) : N16(c, N_minFreq); if (!minF) minF = 16; minF = std::min(minF, maxF);
    u16 f = (m & 2) ? g(6) : N16(c, N_freq); if (!f) f = 16; f = std::min(f, maxF);
    NS16(c, A_maxFreq, maxF); NS16(c, A_minFreq, minF); NS16(c, A_freq, f);
    if ((s16)S16(c, ST_count) > (s16)maxF) SS16(c, ST_count, maxF);
    NS16(c, A_retry, (m & 0x200) ? g(0xA) : N16(c, N_retry));
    NS16(c, A_minPoll, (m & 0x400) ? t[0xC] : N16(c, N_minPoll));
    NS16(c, A_single, (m & 0x800) ? t[0xD] : N16(c, N_single));
    NS16(c, A_ignFatal, (m & 0x1000) ? t[0xE] : N16(c, N_ignFatal));
}
// 3.1 START_MP (0x027e51a0): recvBuf, recvBufSize, sendBuf, sendBufSize, WMMPParam
void StartMP31(Ctx& c, const Rq& q, u16 state)
{
    const u32 rb = RqArg(c, q, 4), rbs = RqArg(c, q, 8), sb = RqArg(c, q, 0xC), sbs = RqArg(c, q, 0x10);
    u8 prm[0x1C]; ReadA9(c, q.addr + 0x14, prm, 0x1C);
    u16 err = E_OK;
    if (N16(c, N_ignSize) == 0)
    {
        const u32 mr = N16(c, N_maxRecv);
        const u32 need = AmParent(c) ? (((mr + 0xC) * S16(c, ST_ppMaxEntry) + 0x29) & ~0x1Fu) : ((mr + 0x51) & ~0x1Fu);
        if (sbs < ((N16(c, N_maxSend) + 0x1Fu) & ~0x1Fu) || rbs < need) err = E_INVALID;
    }
    if (!err && S16(c, ST_mode) == 2 && !(((1u << G16v(c.joinDesc, 0x36)) >> 1) & S16(c, ST_allowedCh))) err = E_INVALID;
    if (err) { Send(c, CbB(0xE, err).P16(4, SC_MP_START)); return; }
    if (S32(c, ST_mpFlag)) { SS32(c, ST_mpFlag, 0); CleanSendQueue(c, 0xFFFF); }
    InitSendQueue(c);
    SetMPParam(c, prm, nullptr);
    if (c.v4 && state != S_MP_PARENT && state != S_MP_CHILD) { u8 t[0x10]; ReadA9(c, q.addr + 0x30, t, 0x10); SetMPTmpParam(c, t); }
    if (state != S_PARENT && state != S_CHILD) { Send(c, CbB(0xE, E_ILLEGAL).P16(4, SC_MP_START)); return; }
    SS16(c, ST_waitAck, 0); SS16(c, ST_c030, 0); SS16(c, ST_c032, 1); SS16(c, ST_newFrame, 0); SS16(c, ST_keepAlive, 0);
    SS16(c, ST_keepCnt, 0x3C); SS32(c, ST_inFlight, 0); KeyPend(c, 0); SS16(c, ST_rxB, 0); SS16(c, ST_rxA, 0);
    SS16(c, ST_polled, 0); SS16(c, ST_resend, 0);
    SS32(c, ST_rb0, rb); SS16(c, ST_rbSize, (u16)rbs); SS32(c, ST_rb1, rb + rbs); SS16(c, ST_rbSel, 0);
    SS32(c, ST_sendBuf, sb); SS16(c, ST_sendSize, (u16)sbs);
    SS16(c, ST_count, 0); SS16(c, ST_limit, 0); SS16(c, ST_lastPoll, 0); SS16(c, ST_lastHdr, 0);
    SS16(c, ST_minRssi, 0xFFFF); SS16(c, ST_rssiCnt, 1);
    for (int a = 0; a < 16; a++) SS64(c, ST_lastRx + 8 * a, Tick(c) | 1);
    SS16(c, ST_vaPend, 0);
    StartVAlarm(c);
    SS16(c, ST_state, state == S_CHILD ? S_MP_CHILD : S_MP_PARENT);
    Send(c, CbB(0xE, E_OK).P16(4, SC_MP_START));
    SS32(c, ST_mpFlag, 1);
    // WL NullKeyResponseMode(1): W_AIDLow = W_AIDFull for the whole MP session
    c.nullKey = true;
    CW(c, R_AIDLow, CR(c, R_AIDFull));
}
// returns false if the request is still in progress (an async WL op)
bool Request(Ctx& c, const Rq& q)
{
    const u16 api = q.api;
    c.stReq++;
    auto cb0 = [&](u16 err) { Send(c, CbB(api, err)); };
    const u16 state = S16(c, ST_state);
    if (api < ApiBound(c)) { SS32(c, ST_apiBusy, 1); SS16(c, ST_busyApi, api); }
    bool done = true;
    const size_t cbs0 = c.cbs.size();
    u16 k = api;             // (3.1: internal ids 0x2b..0x2d are 3.0's 0x29..0x2b)
    if (c.v31) { if (api == 0x29 || api == 0x2A) k = 0xFFFF; else if (api >= 0x2B && api <= 0x2D) k = api - 2; }
    switch (k)
    {
    case 0x00:              // INITIALIZE
        Wr32(c, c.w7, c.st); Wr32(c, c.w7 + 8, c.cbuf);
        InitStatus(c);
        SS16(c, ST_state, S_READY);
        PowerOnSeq(c);
        SS16(c, ST_enableCh, Fw16(c, 0x3C) & 0x7FFE);
        SS16(c, ST_allowedCh, AllowedChannel((S16(c, ST_enableCh) >> 1) & 0x1FFF));
        SS64(c, ST_life, 0x001FF621);
        SS16(c, ST_rate, 2); SS16(c, ST_preamble, 1);
        {
            WriteA9(c, SA(c, ST_wlVer), (const u8*)(c.bd ? c.bd->wlVer : "2.64.00"), 8);
            SS16(c, ST_macVer, CR(c, 0x000)); SS16(c, ST_rfVer, Fw(c)[0x40]); SS16(c, ST_bbVer, 0x006D); SS16(c, ST_bbVer + 2, 0x5B1A);
            WriteA9(c, SA(c, ST_mac), c.mac, 6);
        }
        SS16(c, ST_state, S_IDLE);
        cb0(E_OK);
        break;
    case 0x02:              // END
        if (state != S_IDLE) { cb0(E_ILLEGAL); break; }
        if (c.wl > 0x10) { cb0(E_FAILED); break; }
        PowerDown(c); c.wl = 0;
        SS16(c, ST_state, S_READY); SS16(c, ST_state, S_STOP);
        cb0(E_OK);
        c.release = true;
        break;
    case 0x01:              // RESET
    {
        if (c.pw && S32(c, ST_mpFlag)) SendComplete(c, false, 0xFFFF);
        if (S32(c, ST_mpFlag)) { SS32(c, ST_mpFlag, 0); CancelVAlarm(c); if (state == S_MP_CHILD) SS16(c, ST_state, S_CHILD); if (state == S_MP_PARENT) SS16(c, ST_state, S_PARENT); CleanSendQueue(c, 0xFFFF); }
        if (c.v31)          // (3.1: DISCONNECTED_FROM_MYSELF for every connection, then the child MAC table is cleared)
        {
            const u16 conn = S16(c, ST_conn);
            const bool par = S16(c, ST_state) == S_PARENT;
            if (c.v4 && par) SS16(c, ST_ppEntry, 0);
            for (u16 aid = 0; aid < 16; aid++)
            {
                if (!(conn & (1 << aid))) continue;
                u8 m[6]; ReadA9(c, aid ? SA(c, ST_childMac + 6 * (aid - 1)) : SA(c, ST_pMac), m, 6);
                if (par) Send(c, CbB(0x08, E_OK).P16(8, SC_DISC_MYSELF).P16(0x12, 0xF001).P16(0x10, aid).PB(0xA, m, 6).Sz(c, 0x2C));
                else Send(c, CbB(0x0C, E_OK).P16(8, SC_DISC_MYSELF).P16(0xC, 0xF001).P16(0xA, S16(c, ST_aid)).PB(0x10, m, 6).Sz(c, 0x16));
            }
            for (u32 o = 0; o < 0x5A; o += 2) Wr16(c, SA(c, ST_childMac) + o, 0);
        }
        SS16(c, ST_conn, 0); SS16(c, ST_ready, 0); SS32(c, ST_ks, 0); SS32(c, ST_dcf, 0); SS32(c, ST_vsync, 0); SS16(c, ST_indOn, 0);
        if (c.mode == 1) for (int a = 1; a < 16; a++) if (c.sta[a].state) SendDeauth(c, c.sta[a].mac, 3);
        if (c.mode == 2 && c.wl >= 0x30) SendDeauth(c, c.pmac, 3);
        done = DeauthWait(c, q);
        break;
    }
    case 0x07:              // SET_P_PARAM
    {
        u8 pp[0x40];
        ReadA9(c, RqArg(c, q, 4), pp, 0x40);
        WriteA9(c, SA(c, ST_pp), pp, 0x40);
        if (!((1u << S16(c, ST_ppChannel)) & S16(c, ST_enableCh))) { cb0(E_INVALID); break; }
        c.maxChild = S16(c, ST_ppMaxEntry);
        cb0(E_OK);
        break;
    }
    case 0x08:              // START_PARENT
    {
        if (state != S_IDLE || (c.v4 && (Rd32(c, c.st + 0xC8) & 1))) { Send(c, CbB(8, E_ILLEGAL).P16(8, 0)); break; }
        const u16 ch = S16(c, ST_ppChannel);
        if (!(((1u << ch) >> 1) & S16(c, ST_allowedCh))) { Send(c, CbB(8, E_INVALID).P16(8, 0)); break; }
        SS16(c, ST_mode, 1); SS16(c, ST_aid, 0); SS16(c, ST_conn, 0); SS16(c, ST_ready, 0); SS16(c, ST_preamble, 1);
        SetAllParams(c);
        c.wl = 0x20;
        const u32 pm = RqArg(c, q, 4) != 0;
        c.pm = pm; SS16(c, ST_pm, pm);
        // MLME_START
        c.beaconPeriod = S16(c, ST_ppBeacon);
        SetBssid(c, c.mac);
        CW(c, R_BeaconInt, c.beaconPeriod);
        CW(c, R_ListenInt, 2); CW(c, R_ListenCount, 0);
        SetChannel(c, ch);
        for (Sta& s : c.sta) s = Sta{};
        for (Sta& s : c.sta) s.lifeReload = c.camLife;
        c.pending.clear();
        c.txWindow = false; c.gameInfoPending = false;
        BuildBeacon(c);
        MacStart(c, 1);
        if (c.v31) { SetPMax(c, S16(c, ST_ppPSize) + (S16(c, ST_ppKS) ? 0x2A : 0)); SetCMax(c, S16(c, ST_ppCSize) + (S16(c, ST_ppKS) ? 6 : 0)); }
        else
        {
            SS16(c, ST_maxSend, S16(c, ST_ppPSize) + 4 + (S16(c, ST_ppKS) ? 0x2A : 0));
            SS16(c, ST_maxRecv, S16(c, ST_ppCSize) + 2 + (S16(c, ST_ppKS) ? 6 : 0));
        }
        SS16(c, ST_state, S_PARENT);
        Send(c, CbB(8, E_OK).P16(8, SC_PARENT_START).Sz(c, 0x2C));
        SS16(c, ST_indOn, 1);
        break;
    }
    case 0x09:              // END_PARENT
    {
        if (state != S_PARENT) { cb0(E_ILLEGAL); break; }
        if (c.v4) SS16(c, ST_ppEntry, 0);
        for (u16 aid = 1; aid < 16; aid++)
            if (S16(c, ST_conn) & (1 << aid))
            {
                if (c.sta[aid].state) SendDeauth(c, c.sta[aid].mac, 3);
                SS16(c, ST_conn, S16(c, ST_conn) & ~(1 << aid)); SS16(c, ST_ready, S16(c, ST_ready) & ~(1 << aid));
                SS64(c, ST_lastRx + 8 * aid, 0);
                if (c.v31)
                {
                    u8 m[6]; ReadA9(c, SA(c, ST_childMac + 6 * (aid - 1)), m, 6);
                    Send(c, CbB(0x08, E_OK).P16(8, SC_DISC_MYSELF).P16(0x12, 0xF001).P16(0x10, aid).PB(0xA, m, 6).Sz(c, 0x2C));
                }
            }
        done = DeauthWait(c, q);
        break;
    }
    case 0x0A:              // START_SCAN
    {
        const u16 ch = Rd16(c, q.addr + 2);
        if (state != S_IDLE && state != S_CLASS1 && state != S_SCAN) { Send(c, CbB(0xA, E_ILLEGAL).P16(8, SC_NOT_FOUND)); break; }
        SS32(c, ST_scanBuf, RqArg(c, q, 4)); SS16(c, ST_scanCh, ch);
        for (int k = 0; k < 6; k++) c.opBssid[k] = Rd8(c, q.addr + 0xA + k);
        if (G16(c.opBssid, 0) != 0xFFFF && (c.opBssid[0] & 1)) c.opBssid[0] &= ~1;
        if (!ch || !((1u << ch) & S16(c, ST_enableCh))) { Send(c, CbB(0xA, E_INVALID).P16(8, SC_NOT_FOUND)); break; }
        SS16(c, ST_mode, 2);
        if (c.wl == 0x10) { SetAllParams(c); c.wl = 0x20; SS16(c, ST_state, S_CLASS1); SS16(c, ST_pm, 1); }
        SS16(c, ST_state, S_SCAN);
        // MLME_SCAN: passive, one channel, maxChannelTime ms
        c.op = OpScan; c.opStage = 0; c.opCh = ch; c.opTime = Rd16(c, q.addr + 8); c.opDesc.clear(); c.opEx = false;
        SetTimer(c, T_OP, c.now + Us(c.bd ? c.bd->scanSetupUs : 1500));     // (the guest's channel / power settling before the dwell)
        done = false;
        break;
    }
    case 0x26:              // START_SCAN_EX (4.x)
    {
        if (!c.v4) { cb0(E_ILLEGAL); break; }
        auto fail = [&](u16 e) { Send(c, CbB(0x26, e).P16(8, SC_NOT_FOUND)); };
        if (state != S_IDLE && state != S_CLASS1 && state != S_SCAN) { fail(E_ILLEGAL); break; }
        const u16 chl = Rd16(c, q.addr + 2), size = Rd16(c, q.addr + 8);
        const u32 buf = RqArg(c, q, 4);
        u16 type = Rd16(c, q.addr + 0x12);
        SS32(c, ST_scanBuf, buf); SS16(c, ST_scanCh, chl);
        for (int k = 0; k < 6; k++) c.opBssid[k] = Rd8(c, q.addr + 0xC + k);
        if (G16(c.opBssid, 0) != 0xFFFF && (c.opBssid[0] & 1)) c.opBssid[0] &= ~1;
        const u16 list = (u16)((chl << 1) & S16(c, ST_enableCh));
        if (!list || ((Rd32(c, c.st + 0xC8) & 1) && (type & 1) != 1) || !buf || (buf & 3) || size < 0x40) { fail(E_INVALID); break; }
        SS16(c, ST_mode, 2);
        if (c.wl == 0x10) { SetAllParams(c); c.wl = 0x20; SS16(c, ST_state, S_CLASS1); SS16(c, ST_pm, 1); }
        // (preamble: active scans long, passive short; ponytail: probe requests of an active scan are not sent)
        if (!(type & 1) && S16(c, ST_preamble) == 1) SS16(c, ST_preamble, 0);
        if ((type & 1) && S16(c, ST_preamble) == 0) SS16(c, ST_preamble, 1);
        SS16(c, ST_state, S_SCAN);
        c.op = OpScan; c.opStage = 0; c.opEx = true; c.opList = list; c.opListAll = list; c.opBufSize = size; c.opType = type;
        c.opSsidLen = std::min<u16>(Rd16(c, q.addr + 0x14), 32); c.opMatch = Rd16(c, q.addr + 0x36);
        ReadA9(c, q.addr + 0x16, c.opSsid, 32);
        c.opCh = 0; while (!(list & (1u << c.opCh))) c.opCh++;
        c.opTime = Rd16(c, q.addr + 0xA); c.opDescs.clear();
        SetTimer(c, T_OP, c.now + Us(c.bd->scanSetupUs));
        done = false;
        break;
    }
    case 0x0B:              // END_SCAN
        if (state != S_SCAN) { cb0(E_ILLEGAL); break; }
        DevIdle(c);
        SS16(c, ST_state, S_IDLE);
        if (S16(c, ST_preamble) == 0) SS16(c, ST_preamble, 1);
        cb0(E_OK);
        break;
    case 0x0C:              // START_CONNECT
    {
        if (state != S_IDLE || (c.v4 && (Rd32(c, c.st + 0xC8) & 1))) { Send(c, CbB(0xC, E_ILLEGAL).P16(8, SC_CONNECT_START)); break; }
        u8 desc[0xC0];
        ReadA9(c, RqArg(c, q, 4), desc, 0xC0);
        WriteA9(c, c.w7 + 0x10, desc, 0xC0);
        const u16 gil = G16(desc, 0x3C), ch = G16(desc, 0x36);
        if (gil >= 0x10 && !(desc[0x4B] & 1)) { Send(c, CbB(0xC, E_NO_ENTRY).P16(8, SC_CONNECT_START)); break; }
        const u32 bit = 1u << ch;
        if (!(bit & S16(c, ST_enableCh)) || !((bit >> 1) & 0x1FFF)) { Send(c, CbB(0xC, E_INVALID).P16(8, SC_CONNECT_START)); break; }
        Send(c, CbB(0xC, E_OK).P16(8, SC_CONNECT_START));
        c.nullKey = false;
        SS16(c, ST_rate, (G16(desc, 0x2E) & 2) ? 2 : 1);
        SS16(c, ST_preamble, (G16(desc, 0x2C) & 0x20) ? 1 : 0);
        if (!c.v4) SS32(c, ST_wepOn, (G16(desc, 0x2C) & 0x10) ? 1 : 0);
        SS16(c, ST_mode, gil == 0 ? 3 : 2);
        SetAllParams(c);
        c.wl = 0x20; SS16(c, ST_state, S_CLASS1);
        const u32 pm = Rd32(c, q.addr + 0x20) != 0;
        SS16(c, ST_pm, pm);
        // the join descriptor: SSID = ggid, tgid, 0, + the child's 24-byte user part
        memcpy(c.joinDesc, desc, 0x40);
        if (S16(c, ST_mode) == 2)
        {
            B16v(c.joinDesc, 0xA, 0x20);
            memcpy(&c.joinDesc[0xC], &desc[0x44], 4); memcpy(&c.joinDesc[0x10], &desc[0x48], 2);
            c.joinDesc[0x12] = c.joinDesc[0x13] = 0;
            ReadA9(c, q.addr + 8, &c.joinDesc[0x14], 24);
        }
        // MLME_JOIN
        c.beaconPeriod = G16(desc, 0x32);
        SetBssid(c, desc + 4);
        CW(c, R_Preamble, (G16(desc, 0x2C) & 0x20) ? (CR(c, R_Preamble) | 6) : (CR(c, R_Preamble) & ~6));
        CW(c, R_BeaconInt, c.beaconPeriod);
        SetChannel(c, ch);
        MacStart(c, 2);
        for (Sta& s : c.sta) s = Sta{};
        c.sta[1].lifeReload = c.camLife; c.sta[1].life = c.camLife;
        c.op = OpConnect; c.opStage = 1;
        c.joinTgid = G16(desc, 0x48); c.joinPSize = G16(desc, 0x4C); c.joinCSize = G16(desc, 0x4E); c.joinAttr = desc[0x4B];
        c.joinRssi = desc[2];
        SetTimer(c, T_OP, c.now + Ms(2000));
        done = false;
        break;
    }
    case 0x0D:              // DISCONNECT
    {
        u16 d = 0;
        if (DisconnectCore(c, (u16)RqArg(c, q, 4), false, 0, &d)) { c.opBitmap = (u16)RqArg(c, q, 4); c.opDone = d; done = DeauthWait(c, q); }
        break;
    }
    case 0x25:              // AUTO_DISCONNECT (internal)
        if (g_Dbg) fprintf(stderr, "A7WM inst%d f%u AUTO_DISCONNECT %x reason %x\n", c.inst, c.nds.NumFrames, q.a1 & 0xFFFF, q.a1 >> 16);
        if (DisconnectCore(c, q.a1 & 0xFFFF, true, q.a1 >> 16, nullptr)) done = DeauthWait(c, q);
        break;
    case 0x22:              // AUTO_DEAUTH (internal)
        if (!c.autoDeAuth.empty())
        {
            std::vector<u8> m = c.autoDeAuth.front();
            c.autoDeAuth.pop_front();
            SendDeauth(c, m.data(), 0x13);
            if (Sta* s = StaByMac(c, m.data())) *s = Sta{};
            cb0(E_OK);
        }
        break;
    case 0x0E:              // START_MP
    {
        if (c.v31) { StartMP31(c, q, state); break; }
        if (S16(c, ST_mode) == 2 && !(((1u << G16v(c.joinDesc, 0x36)) >> 1) & S16(c, ST_allowedCh))) { Send(c, CbB(0xE, E_INVALID).P16(4, SC_MP_START)); break; }
        if (S32(c, ST_mpFlag)) { SS32(c, ST_mpFlag, 0); CleanSendQueue(c, 0xFFFF); }
        InitSendQueue(c);
        if (state != S_PARENT && state != S_CHILD) { Send(c, CbB(0xE, E_ILLEGAL).P16(4, SC_MP_START)); break; }
        const u32 rb = RqArg(c, q, 4), rbs = RqArg(c, q, 8), freq = RqArg(c, q, 0x14);
        SS16(c, ST_waitAck, 0); SS16(c, ST_c030, 0); SS16(c, ST_c032, 1); SS16(c, ST_newFrame, 0); SS16(c, ST_keepAlive, 0);
        SS16(c, ST_keepCnt, 0x3C); SS32(c, ST_inFlight, 0); KeyPend(c, 0); SS16(c, ST_rxB, 0); SS16(c, ST_rxA, 0);
        SS16(c, ST_polled, 0); SS16(c, ST_resend, 0);
        SS32(c, ST_rb0, rb); SS16(c, ST_rbSize, (u16)rbs); SS32(c, ST_rb1, rb + rbs); SS16(c, ST_rbSel, 0);
        SS32(c, ST_sendBuf, RqArg(c, q, 0xC)); SS16(c, ST_sendSize, (u16)RqArg(c, q, 0x10));
        SS16(c, ST_freq, freq ? (u16)freq : 16); SS16(c, ST_retry, (u16)RqArg(c, q, 0x18));
        SS16(c, ST_minPoll, RqArg(c, q, 0x1C) != 0); SS16(c, ST_single, RqArg(c, q, 0x20) != 0); SS16(c, ST_fixFreq, RqArg(c, q, 0x24) != 0);
        SS16(c, ST_count, 0); SS16(c, ST_limit, 0); SS16(c, ST_lastPoll, 0); SS16(c, ST_lastHdr, 0);
        SS16(c, ST_minRssi, 0xFFFF); SS16(c, ST_rssiCnt, 4); SS16(c, ST_ignFatal, RqArg(c, q, 0x28) != 0);
        for (int a = 0; a < 16; a++) SS64(c, ST_lastRx + 8 * a, Tick(c) | 1);
        SS16(c, ST_vaPend, 0);
        StartVAlarm(c);
        SS16(c, ST_state, state == S_CHILD ? S_MP_CHILD : S_MP_PARENT);
        Send(c, CbB(0xE, E_OK).P16(4, SC_MP_START));
        SS32(c, ST_mpFlag, 1);
        break;
    }
    case 0x0F:              // SET_MP_DATA
    {
        const u32 data = RqArg(c, q, 4); const u16 size = (u16)RqArg(c, q, 8); u16 dest = (u16)RqArg(c, q, 0xC);
        const u16 port = (u16)RqArg(c, q, 0x10), prio = (u16)RqArg(c, q, 0x14);
        const u32 cbp = RqArg(c, q, 0x18), arg = RqArg(c, q, 0x1C);
        const u16 conn = S16(c, ST_conn);
        if (S16(c, ST_aid)) dest = 1;
        const u16 r = !S32(c, ST_mpFlag) ? E_ILLEGAL : ((dest & conn) == 0 ? E_OK : PutSendQueue(c, conn, prio, port, dest, data, size, cbp, arg));
        if (r != E_OPERATING)
            Send(c, CbB(0x81, r).P16(8, SC_PORT_SEND).P16(0xA, port).P16(0xC, dest).P16(0xE, r == E_QFULL ? (dest & conn) : 0).P16(0x10, 0)
                         .P32(0x14, data).P16(0x18, size).P16(0x1A, 0xFFFF).P32(0x1C, cbp).P32(0x20, arg).MySz(c, 0x24));
        break;
    }
    case 0x10:              // END_MP
    {
        if (state != S_MP_PARENT && state != S_MP_CHILD) { cb0(E_ILLEGAL); break; }
        const bool was = S32(c, ST_mpFlag) != 0;
        if (c.pw && was) SendComplete(c, false, 0xFFFF);
        SS32(c, ST_mpFlag, 0);
        CancelVAlarm(c);
        SS16(c, ST_state, state == S_MP_CHILD ? S_CHILD : S_PARENT);
        c.nullKey = false;
        if (c.mode == 2) { CW(c, R_Reply1, 0); CW(c, R_AIDLow, 0); c.bufBusy[0] = c.bufBusy[1] = false; }
        else if (c.v31) { CW(c, R_AIDLow, 0); CW(c, R_TXSlotReset, 0x00C0); CW(c, R_TXSlotReset, 2); CW(c, R_TXSlotReset, 9); }  // (WL MaClearData(7))
        KeyPend(c, 0);
        if (was) CleanSendQueue(c, 0xFFFF);
        cb0(E_OK);
        break;
    }
    case 0x18:              // SET_GAMEINFO
    case 0x21:              // SET_ENTRY
        if (api == 0x18)
        {
            const u32 attr = RqArg(c, q, 0x14) & 0xFF;
            SS32(c, ST_ppUgi, RqArg(c, q, 4)); SS16(c, ST_ppUgiLen, (u16)RqArg(c, q, 8)); SS32(c, ST_ppGgid, RqArg(c, q, 0xC));
            SS16(c, ST_ppTgid, (u16)RqArg(c, q, 0x10)); SS16(c, ST_ppEntry, attr & 1); SS16(c, ST_ppMB, (attr & 2) ? 1 : 0);
            SS16(c, ST_ppKS, (attr & 4) ? 1 : 0); SS16(c, ST_ppCS, (attr & 8) ? 1 : 0);
        }
        else SS16(c, ST_ppEntry, Rd16(c, q.addr + 4));
        if (c.mode != 1) { Send(c, CbB(api, E_FAILED).P16(4, 0x245).P16(6, 0xB)); break; }
        c.pendingGameInfo = GameInfo(c);
        c.gameInfoPending = true;
        cb0(E_OK);
        break;
    case 0x1D:              // SET_LIFETIME
    {
        const u16 cam = (u16)RqArg(c, q, 8), mp = (u16)RqArg(c, q, 0x10);
        c.camLife = cam;
        for (Sta& s : c.sta) { s.lifeReload = cam; if (s.state) s.life = cam; }
        SS64(c, ST_life, mp == 0xFFFF ? 0 : mp == 0 ? 1 : c.v31 ? ((u64)mp * 100 * 0x82EA) >> 6 : (u64)(s64)((s32)(mp * 0x332368) / 64));
        for (int a = 0; a < 16; a++) SS64(c, ST_lastRx + 8 * a, Tick(c) | 1);
        cb0(E_OK);
        break;
    }
    case 0x1E:              // MEASURE_CHANNEL
    {
        if (state != S_IDLE) { cb0(E_ILLEGAL); break; }
        SS16(c, ST_mode, 2);
        if (c.wl == 0x10) { SetAllParams(c); c.wl = 0x20; SS16(c, ST_state, S_CLASS1); SS16(c, ST_pm, 1); }
        c.op = OpMeasure; c.opCh = Rd16(c, q.addr + 6); c.opTime = Rd16(c, q.addr + 8);
        if (c.bd && c.bd->measPreUs) { c.opStage = 0; SetTimer(c, T_OP, c.now + Us(c.bd->measPreUs)); done = false; break; }
        c.opStage = 1;
        SetChannel(c, c.opCh);
        MacStart(c, 4);
        SetTimer(c, T_OP, c.now + Ms(c.opTime));
        done = false;
        break;
    }
    case 0x23:              // SET_MP_FREQUENCY (3.1: SET_MP_PARAMETER)
    {
        if (c.v31)
        {
            u8 p[0x1C], old[0x1C];
            ReadA9(c, q.addr + 4, p, 0x1C);
            const u16 r = SetMPParam(c, p, old);
            Send(c, CbB(0x23, r).P32(4, RqArg(c, q, 4)).PB(8, old, 0x1C));
            break;
        }
        const u16 f = (u16)RqArg(c, q, 4) ? (u16)RqArg(c, q, 4) : 16;
        SS16(c, ST_freq, f);
        if ((s16)S16(c, ST_count) > (s16)f) SS16(c, ST_count, f);
        cb0(E_OK);
        break;
    }
    case 0x19: case 0x24: cb0(E_OK); break;
    case 0x1C: VAlarmMP(c); break;
    case 0xFFFF: cb0(4); break;    // (3.1 test-RX mode APIs 0x29 / 0x2a: WM_ERRCODE_WM_DISABLE)
    case 0x29: if (S16(c, ST_state) == S_MP_PARENT) SendMaMP(c, (u16)q.a1); break;
    case 0x2A: if (S16(c, ST_state) == S_MP_CHILD) SendMaKeyData(c); break;
    case 0x2B: if (S16(c, ST_state) == S_MP_PARENT) ResendMaMP(c, (u16)q.a1); break;
    case 0x15: case 0x16: case 0x17: break;
    default:
        if (api < ApiBound(c)) cb0(E_ILLEGAL);     // ponytail: APIs this game family doesn't use (DCF, WEP, test modes, counters)
        break;
    }
    // the guest's WL takes this long for these (power-up waits, channel settling): the callback and the request's
    // consumed flag come then (scan / connect / measure time themselves)
    u32 holdUs = 0;
    if (done) holdUs = api == 0x00 ? 12700 : api == 0x0B ? c.bd->endScanUs : api == 0x08 ? c.bd->parentHoldUs : api == 0x01 ? c.bd->resetUs : api == 0x09 ? c.bd->endParentUs : 0;
    if (holdUs && c.cbs.size() > cbs0)
    {
        while (c.cbs.size() > cbs0) { c.held.push_front(c.cbs.back()); c.cbs.pop_back(); }
        c.op = OpHold; c.opRq = q;
        SetTimer(c, T_OP, c.now + Us(holdUs));
        return false;
    }
    if (done)
    {
        if (api < ApiBound(c)) SS32(c, ST_apiBusy, 0);
        ReqDone(c, q);
    }
    else c.opRq = q;
    return done;
}
void OpFinish(Ctx& c)
{
    SS32(c, ST_apiBusy, 0);
    ReqDone(c, c.opRq);
    c.op = OpNone; c.opStage = 0;
    StopTimer(c, T_OP);
}
void ScanFound(Ctx& c, std::vector<u8>&& d) { c.opDesc = std::move(d); }
// START_SCAN_EX result (4.x): the descriptors packed into the scan buffer (4-aligned), pointers + link levels in the callback
void ScanExDone(Ctx& c)
{
    const u32 sb = S32(c, ST_scanBuf);
    CbB cb(0x26, E_OK);
    cb.P16(0xA, c.opListAll >> 1);
    if (c.opDescs.empty()) cb.P16(8, SC_NOT_FOUND).P16(0xE, 0);
    else
    {
        for (u32 o = 0; o + 2 <= c.opBufSize; o += 2) Wr16(c, sb + o, 0);
        u32 dest = sb, n = 0;
        for (const auto& d : c.opDescs)
        {
            const u32 len = G16(d, 0) * 2u;
            if (n == 16 || dest + len > sb + c.opBufSize) break;   // ponytail: the WL's confirm-buffer bound, approximated
            WriteA9(c, dest, d.data(), (u32)std::min<size_t>(len, d.size()));
            const u16 sl = G16(d, 0xA);
            bool valid = sl && sl <= 32;
            if (valid) { valid = false; for (u32 i = 0; i < sl; i++) valid |= d[0xC + i] != 0; }
            if (c.opSsidLen && !valid) { Wr16(c, dest + 0xA, c.opSsidLen); WriteA9(c, dest + 0xC, c.opSsid, 32); }
            const u16 r = Conv(d[2]);
            cb.P32(0x10 + 4 * n, dest).P16(0x50 + 2 * n, LinkLevelC(c, r));
            SeedUpdate(c, r);
            dest += len; if (dest & 2) dest = (dest + 2) & ~3u;
            n++;
        }
        cb.P16(8, SC_FOUND).P16(0xE, (u16)n);
    }
    Send(c, cb);
    OpFinish(c);
}
void ScanDone(Ctx& c)
{
    if (c.opEx) { ScanExDone(c); return; }
    const u16 ch = c.opCh;
    if (c.opDesc.empty()) Send(c, CbB(0xA, E_OK).P16(8, SC_NOT_FOUND).P16(0x10, ch).P16(0x12, 0));
    else
    {
        const std::vector<u8>& d = c.opDesc;
        const u32 sb = S32(c, ST_scanBuf);
        for (u32 o = 0x40; o < 0xC0; o += 2) Wr16(c, sb + o, 0);
        WriteA9(c, sb, d.data(), (u32)std::min<size_t>(d.size(), G16(d, 0) * 2u));
        const u16 r = Conv(d[2] & 0xFF);
        SeedUpdate(c, r);
        const u16 gil = G16(d, 0x3C);
        CbB cb(0xA, E_OK);
        if (gil > 0x80) cb.P16(8, SC_NOT_FOUND).P16(0x10, ch).P16(0x12, 0);
        else
        {
            cb.P16(8, SC_FOUND).P16(0x10, G16(d, 0x36)).P16(0x12, LinkLevelC(c, r)).P16(0x14, G16(d, 0xA)).PB(0xA, &d[4], 6).PB(0x16, &d[0xC], 32).P16(0x36, gil);
            for (u32 o = 0x38; o < 0xB8; o++) cb.P8(o, 0);
            cb.PB(0x38, &d[0x40], std::min<u32>((gil + 1) & ~1, (u32)d.size() - 0x40));
        }
        Send(c, cb);
    }
    OpFinish(c);
}
void MeasureDone(Ctx& c)
{
    SS16(c, ST_state, S_IDLE);
    Send(c, CbB(0x1E, E_OK).P16(8, c.opCh & 0xFF).P16(0xA, 0));      // (CCA busy ratio: never busy in the chip model)
    OpFinish(c);
}
void ConnectFail(Ctx& c, u16 cmd, u16 res, u16 status)
{
    MacStop(c); c.mode = 0;
    if (res == 0xC && status == 0x13) Send(c, CbB(0xC, E_OVER_MAX).P16(8, SC_CONNECT_START));
    else Send(c, CbB(0xC, E_FAILED).P16(4, cmd).P16(6, res).P16(0xE, status));
    OpFinish(c);
}
// the child's connect sequence: 1 join (beacon) -> 2 auth -> 3 assoc -> connected
void ConnectStep(Ctx& c, int result, u16 status)
{
    if (c.op != OpConnect) return;
    if (c.opStage == 1)
    {
        if (result) { ConnectFail(c, 3, (u16)result, status); return; }
        WriteA9(c, SA(c, ST_pMac), c.bssid, 6);
        const u16 ks = (c.joinAttr & 4) != 0;
        if (!c.v31)         // (3.1: after the association)
        {
            SS16(c, ST_maxSend, c.joinCSize + 2 + (ks ? 6 : 0));
            SS16(c, ST_maxRecv, c.joinPSize + 4 + (ks ? 0x2A : 0));
        }
        c.opStage = 2;
        const u8 body[6] = {0, 0, 1, 0, 0, 0};
        QueueMgmt(c, Frame(0x00B0, c.bssid, c.mac, c.bssid, body, 6), MK_AUTH_REQ, 0, 0);
        SetTimer(c, T_OP, c.now + Ms(2000));
        return;
    }
    if (c.opStage == 2)
    {
        if (result) { ConnectFail(c, 4, (u16)result, status); return; }
        c.opStage = 3;
        u8 body[42];
        body[0] = 0x21; body[1] = 0; body[2] = 1; body[3] = 0; body[4] = 0x00; body[5] = 0x20;
        memcpy(&body[6], &c.joinDesc[0xC], 32);
        body[38] = 0x01; body[39] = 0x02; body[40] = 0x82; body[41] = 0x84;
        QueueMgmt(c, Frame(0x0000, c.bssid, c.mac, c.bssid, body, 42), MK_ASSOC_REQ, 0, 0);
        SetTimer(c, T_OP, c.now + Ms(2000));
        return;
    }
    if (c.opStage == 3)
    {
        if (result) { ConnectFail(c, 6, (u16)result, status); return; }
        SS16(c, ST_aid, c.aid);
        SS16(c, ST_ptgid, c.joinTgid);
        FillSeqRow(c, 0);
        const u16 r = Conv(c.joinRssi);
        SS16(c, ST_link, LinkLevelC(c, r));
        for (u8& v : c.rssi) v = (u8)r;
        c.rssiIdx = 0;
        SS16(c, ST_conn, 1); SS16(c, ST_ready, 1);
        if (S64(c, ST_life)) SS64(c, ST_lastRx, Tick(c) | 1);
        SS16(c, ST_state, S_CHILD);
        if (c.v31)
        {
            const bool ks = (c.joinAttr & 4) != 0;
            SetPMax(c, c.joinPSize + (ks ? 0x2A : 0)); SetCMax(c, c.joinCSize + (ks ? 6 : 0));
        }
        SS16(c, ST_indOn, 1);
        Send(c, CbB(0xC, E_OK).P16(8, SC_CONNECTED).P16(0xA, c.aid).PB(0x10, c.bssid, c.v31 ? 6 : 0).Sz(c, 0x16));
        c.sta[1].state = 0x40; memcpy(c.sta[1].mac, c.bssid, 6);
        OpFinish(c);
    }
}

// ---- timers ----
void Tick100(Ctx& c)
{
    SetTimer(c, T_TICK, c.now + Ms(100));
    if (c.mode == 1)
        for (u16 a = 1; a < 16; a++)
        {
            Sta& s = c.sta[a];
            if (!s.state || s.life == 0xFFFF) continue;
            if (s.life && --s.life) continue;
            u8 m[6]; memcpy(m, s.mac, 6);
            SendDeauth(c, m, 1);
            s = Sta{};
            IndMacReason(c, 0x0085, m, 1);
        }
    else if (c.mode == 2 && c.wl == 0x40)
    {
        Sta& s = c.sta[1];
        if (s.life != 0xFFFF && !(s.life && --s.life))
        {
            SendDeauth(c, c.pmac, 1);
            c.wl = 0x20; c.aid = 0; CW(c, R_AIDLow, 0); CW(c, R_AIDFull, 0);
            IndMacReason(c, 0x0085, c.pmac, 1);
            s.life = s.lifeReload;
        }
    }
}
void FireTimers(Ctx& c)
{
    for (int k = 0; k < kTimers; k++)
    {
        if (!c.tm[k] || c.tm[k] > c.now) continue;
        c.tm[k] = 0;
        switch (k)
        {
        case T_OP:
            if (c.op == OpScan && c.opStage == 0)
            {
                SetBssid(c, c.opBssid);
                SetChannel(c, c.opCh);
                MacStart(c, 2);
                c.opStage = 1;
                SetTimer(c, T_OP, c.now + Us(c.opTime * 1000u + c.bd->scanExtraUs));
            }
            else if (c.op == OpScan && c.opStage == 1 && c.opEx && (c.opList & ~((2u << c.opCh) - 1)))
            {
                // START_SCAN_EX: the next channel of the list (the guest's WL: flash checksum + channel switch, MAC kept)
                c.opStage = 3;
                SetTimer(c, T_OP, c.now + Us(3050));
            }
            else if (c.op == OpScan && c.opStage == 3)
            {
                do c.opCh++; while (!(c.opList & (1u << c.opCh)));
                SetChannel(c, c.opCh);
                c.opStage = 1;
                SetTimer(c, T_OP, c.now + Us(c.opTime * 1000u + c.bd->scanExtraUs));
            }
            else if (c.op == OpScan && c.opStage == 1) { MacStop(c); c.mode = 0; c.opStage = 2; SetTimer(c, T_OP, c.now + Us(c.bd->scanDownUs)); }
            else if (c.op == OpScan) ScanDone(c);
            else if (c.op == OpMeasure && c.opStage == 0)
            {
                c.opStage = 1;
                SetChannel(c, c.opCh);
                MacStart(c, 4);
                SetTimer(c, T_OP, c.now + Us(c.opTime * 1000u + c.bd->measExtraUs));
            }
            else if (c.op == OpMeasure && c.opStage == 1) { MacStop(c); c.mode = 0; DevIdle(c); c.opStage = 2; SetTimer(c, T_OP, c.now + Us(c.bd->measPostUs)); }
            else if (c.op == OpMeasure) MeasureDone(c);
            else if (c.op == OpDeauth) { c.mgmtQ.clear(); c.mgmtBusy = false; DeauthFinish(c); }
            else if (c.op == OpHold)
            {
                while (!c.held.empty()) { c.cbs.push_back(c.held.front()); c.held.pop_front(); }
                if (c.opRq.api < ApiBound(c)) SS32(c, ST_apiBusy, 0);
                ReqDone(c, c.opRq);
                c.op = OpNone;
            }
            else if (c.op == OpConnect) { const int st = c.opStage; ConnectFail(c, st == 1 ? 3 : st == 2 ? 4 : 6, 7, 0); }
            break;
        case T_AIDCLR: if (!(CR(c, R_Reply1) & 0x8000)) CW(c, R_AIDLow, 0); break;
        case T_ACKTO: PostInd(c, 0x0185, {}); break;
        case T_TICK: Tick100(c); break;
        case T_POLL: break;
        }
    }
}

std::deque<Ind>& IndQ(Ctx& c) { return c.inds; }

void Process(Ctx& c)
{
    if (c.inside) return;
    c.inside = true;
    for (int guard = 0; guard < 256; guard++)
    {
        if (c.irqPending) { c.irqPending = false; HandleChip(c); }
        FireTimers(c);
        for (int k = 0; k < 2; k++)
            if (c.kickAt[k] && c.now >= c.kickAt[k])
            {
                c.kickAt[k] = 0;
                if (k == 0) { PostInternal(c, IdMP(c), c.kickArg[0]); c.mpNext = c.now + Us(c.bd->mpGapUs); }
                else PostInternal(c, IdKey(c), 0);
            }
        if (c.lineDue) { c.lineDue = false; VAlarm(c, c.lineDueV); }
        PumpCbs(c);
        if (!c.cbs.empty()) break;              // (WMSP waits for the ARM9 to take the callback)
        if (!c.inds.empty()) { Ind in = std::move(c.inds.front()); c.inds.pop_front(); HandleInd(c, in); continue; }
        if (c.op == OpNone && !c.reqs.empty())
        {
            const Rq& f = c.reqs.front();
            // the guest's parent starts the next MP of a sequence >= ~0.84 ms (median 1.2) after the MP end (WL task ->
            // WMSP indication -> request thread -> MA_MP); emulated children need that time to arm their next reply
            if (!f.addr && (f.api == IdMP(c) || f.api == IdResend(c)) && c.now < c.mpNext) { SetTimer(c, T_POLL, c.mpNext); break; }
            if (!f.addr && f.api == IdKey(c) && c.now < c.keyNext) { SetTimer(c, T_POLL, c.keyNext); break; }
            if (!f.addr && f.api == 0x1C && c.now < c.vaNext) { SetTimer(c, T_POLL, c.vaNext); break; }
            if (f.addr && !c.reqAt)
                if (const u32 us = QuickUs(c, f.api)) { c.reqAt = c.now + Us(us); if (f.api < ApiBound(c)) { SS32(c, ST_apiBusy, 1); SS16(c, ST_busyApi, f.api); } }
            if (c.reqAt && c.now < c.reqAt) { SetTimer(c, T_POLL, c.reqAt); break; }
            c.reqAt = 0;
            Rq q = f; c.reqs.pop_front(); Request(c, q); continue;
        }
        break;
    }
    PumpCbs(c);
    if (c.release && c.cbs.empty() && c.op == OpNone && c.inds.empty() && c.reqs.empty()) { Release(c); c.inside = false; return; }
    u64 next = ~0ull;
    for (u64 t : c.tm) if (t && t < next) next = t;
    for (u64 t : c.kickAt) if (t && t < next) next = t;
    if (c.irqPending || c.lineDue) next = c.now + 1;
    if (next != ~0ull) WakeAt(c, std::max(next, c.now + 1));
    c.inside = false;
}

// ---- savestate (section "WMNA"; layout kSaveVer) ----
constexpr u32 kSaveVer = 6;          // (3: + nullKey, keyNext; 4: + interval kicks; 2 and 3 still load)
struct SS
{
    Savestate* f;
    template <class T> void V(T& v) { f->VarArray(&v, sizeof(T)); }
    void B(bool& v) { u8 x = v; f->Var8(&x); v = x; }
    void Bytes(std::vector<u8>& v) { u32 n = (u32)v.size(); f->Var32(&n); if (!f->Saving) v.assign(n, 0); if (n) f->VarArray(v.data(), n); }
    template <class C, class G> void Seq(C& q, G g) { u32 n = (u32)q.size(); f->Var32(&n); if (!f->Saving) { q.clear(); q.resize(n); } for (auto& e : q) g(e); }
};
void Visit(Ctx& c, SS& z, u32 ver)
{
    z.B(c.active); z.V(c.w7); z.V(c.st); z.V(c.cbuf);
    z.Seq(c.reqs, [&](Rq& q) { z.V(q.addr); z.V(q.api); z.V(q.a1); });
    auto cb = [&](Ctx::Cb& b) { z.V(b.d); z.V(b.m); z.V(b.n); };
    z.Seq(c.cbs, cb); z.Seq(c.held, cb);
    z.V(c.wake); z.V(c.tm); z.B(c.irqPending); z.B(c.lineDue); z.V(c.lineDueV);
    z.V(c.op); z.V(c.opAddr); z.V(c.opApi); z.V(c.opStage); z.V(c.opCh); z.V(c.opTime); z.V(c.opScanBuf); z.V(c.opBssid); z.Bytes(c.opDesc);
    z.V(c.opRq.addr); z.V(c.opRq.api); z.V(c.opRq.a1);
    z.V(c.wl); z.V(c.mode); z.V(c.mac); z.V(c.bssid); z.V(c.channel); z.V(c.rxBegin); z.B(c.pm); z.V(c.activeZone); z.V(c.vtsf);
    auto sta = [&](Sta& t) { z.V(t.state); z.V(t.mac); z.V(t.aid); z.V(t.life); z.V(t.lifeReload); z.V(t.ssid); };
    for (Sta& t : c.sta) sta(t);
    z.Seq(c.pending, sta);
    z.V(c.maxChild);
    auto mg = [&](Mgmt& m) { z.Bytes(m.f); z.V(m.kind); z.V(m.sta); z.V(m.status); };
    z.Seq(c.mgmtQ, mg); z.B(c.mgmtBusy); mg(c.mgmtCur); z.B(c.txWindow);
    z.V(c.beaconLen); z.V(c.gameInfoOff); z.Bytes(c.pendingGameInfo); z.B(c.gameInfoPending);
    z.B(c.mpBusy); z.V(c.mpPoll); z.V(c.mpGot); z.V(c.mpN); z.V(c.mpStride); z.V(c.mpTxopUs); z.V(c.mpLen); z.V(c.mpHdr); z.V(c.mpTmptt);
    z.V(c.mpRemain); z.Bytes(c.mpInd); z.V(c.mpFailed); z.V(c.mpTxopOrig);
    z.V(c.aid); z.V(c.pmac); z.V(c.bufBusy); z.V(c.lastCmdSeq); z.V(c.cmdFlags); z.V(c.beaconPeriod);
    z.V(c.vX); z.f->VarArray(c.rssi, ver >= 3 ? 32 : 16); z.V(c.rssiIdx); z.V(c.camLife);
    z.Seq(c.inds, [&](Ind& in) { z.V(in.code); z.Bytes(in.b); });
    z.Seq(c.autoDeAuth, [&](std::vector<u8>& v) { z.Bytes(v); });
    z.V(c.build); z.V(c.lastPassed); z.B(c.guestCbSince); z.B(c.release); z.V(c.mpNext);
    z.V(c.joinDesc); z.V(c.joinTgid); z.V(c.joinPSize); z.V(c.joinCSize); z.V(c.joinAttr); z.V(c.joinRssi);
    if (ver >= 3)
    {
        z.B(c.nullKey); z.V(c.keyNext);
        if (ver >= 4) { z.V(c.kickAt); z.V(c.kickArg); }
        if (ver >= 5) z.V(c.vaNext);
        if (ver >= 6) z.V(c.reqAt);
        z.B(c.opEx); z.V(c.opList); z.V(c.opListAll); z.V(c.opBufSize); z.V(c.opSsidLen); z.V(c.opMatch); z.V(c.opType); z.V(c.opSsid);
        z.Seq(c.opDescs, [&](std::vector<u8>& v) { z.Bytes(v); });
    }
    if (!z.f->Saving) SetBuild(c, c.build);
}

// the guest's WMSP / WL are idle (its last request called back); the native driver goes on from the WMStatus state it left
bool Activate(Ctx& c, u16 api)
{
    NDS& nds = c.nds;
    const u16 state = S16(c, ST_state);
    if (state != S_IDLE && state != S_CLASS1 && state != S_SCAN) return false;
    nds.RegisterEventFuncs(Event_WMN, &c, {[](void* that, u32 p) { OnEvent(that, p); }});
    c.active = true;
    nds.Wifi.WMNOwns = true;
    c.wl = state == S_IDLE ? 0x10 : 0x20;
    c.mode = 0;
    for (int k = 0; k < 6; k++) c.mac[k] = Rd8(c, SA(c, ST_mac + k));
    for (Sta& t : c.sta) { t = Sta{}; t.lifeReload = c.camLife; }
    c.pending.clear();
    Platform::Log(Platform::LogLevel::Info, "A7_NATIVE_WM: takeover inst %d frame %u build=%016llx (%s) on request %02x, WM state %u\n",
                  c.inst, nds.NumFrames, (unsigned long long)c.bd->id, c.bd->name, api, state);
    if (g_Stats) fprintf(stderr, "A7WM inst%d f%u: native WM driver takes over (state %u; WM7 %08x status %08x cb %08x)\n", c.inst, nds.NumFrames, state, c.w7, c.st, c.cbuf);
    return true;
}
// END done: the guest's WM gets the next session (its WMSP / WL are where they were at the takeover: idle)
void Release(Ctx& c)
{
    NDS& nds = c.nds;
    nds.CancelEvent(Event_WMN);
    nds.GPU.WMLine = -1;
    nds.Wifi.WMNOwns = false;
    c.active = false; c.release = false; c.lastPassed = 0; c.guestCbSince = true;
    c.reqs.clear(); c.cbs.clear(); c.held.clear(); c.inds.clear(); c.autoDeAuth.clear();
    for (u64& t : c.tm) t = 0;
    c.wake = ~0ull; c.irqPending = false; c.lineDue = false; c.vX = -1;
    c.op = OpNone; c.opStage = 0; c.opDesc.clear();
    c.wl = 0; c.mode = 0; c.mgmtQ.clear(); c.mgmtBusy = false; c.txWindow = false; c.gameInfoPending = false;
    c.mpBusy = false; c.mpInd.clear(); c.aid = 0; c.bufBusy[0] = c.bufBusy[1] = false; c.lastCmdSeq = 0xFFFF;
    c.nullKey = false; c.keyNext = 0; c.mpNext = 0; c.kickAt[0] = c.kickAt[1] = 0; c.vaNext = 0; c.reqAt = 0;
    Platform::Log(Platform::LogLevel::Info, "A7_NATIVE_WM: hand-back inst %d frame %u build=%016llx: WM END done, the emulated driver has the next session\n",
                  c.inst, nds.NumFrames, c.bd ? (unsigned long long)c.bd->id : 0ull);
    SetBuild(c, -1);
    for (Sta& t : c.sta) t = Sta{};
    c.pending.clear();
    if (g_Stats) fprintf(stderr, "A7WM inst%d f%u: WM back to the emulated driver\n", c.inst, nds.NumFrames);
}
}



void DoSavestate(NDS& nds, Savestate* f)
{
    Ctx& c = GetOrMake(nds);
    if (!f->Saving)
    {
        const int inst = c.inst, allow = c.allow;
        Reset(nds);
        Get(nds)->inst = inst; Get(nds)->allow = allow;
        if (!f->HasSection("WMNA")) return;
    }
    Ctx& d = *Get(nds);
    f->Section("WMNA");
    u32 ver = kSaveVer;
    f->Var32(&ver);
    if (ver < 2 || ver > kSaveVer) { f->Error = true; return; }
    SS z{f};
    Visit(d, z, ver);
    if (!f->Saving && d.active)
    {
        nds.RegisterEventFuncs(Event_WMN, &d, {[](void* that, u32 p) { OnEvent(that, p); }});
        nds.Wifi.WMNOwns = true;
        nds.GPU.WMLine = d.vX;
    }
}

void Reset(NDS& nds)
{
    Ctx* p = Get(nds);
    if (!p) return;
    const int inst = p->inst, allow = p->allow;
    if (p->active) { nds.CancelEvent(Event_WMN); nds.GPU.WMLine = -1; }
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
    const u32 a = val >> 6;
    if ((a >> 24) != 2) return false;
    Ctx* p = Get(nds);
    if (!p || !p->active)
    {
        Ctx& c = GetOrMake(nds);
        if ((val >> 5) & 1) return false;
        const u16 api = nds.ARM7Read16(a) & 0x7FFF;
        bool take = false;
        if (api == 0x00)            // the guest's INITIALIZE: is this WM build ours (for this session)?
        {
            u32 table = 0;
            const Build* b = nds.ConsoleType == 0 && Allowed(c) ? FindBuild(c, &table) : nullptr;
            if (b && b->gen >= 50 && !TwlAllowed())
            {
                if (g_Stats) fprintf(stderr, "A7WM inst%d: WM build %s: emulated (debug.litev.a7wmtwl=0)\n", c.inst, b->name);
                b = nullptr;
            }
            SetBuild(c, b ? (int)(b - kBuilds) + 1 : 0);
            c.w7 = nds.ARM7Read32(a + 4); c.st = nds.ARM7Read32(a + 8); c.cbuf = nds.ARM7Read32(a + 12);
            c.camLife = 0x28;
            if (b && g_Stats) fprintf(stderr, "A7WM inst%d: WM build %s (table %08x): native from the first local-play request\n", c.inst, b->name, table);
            Platform::Log(Platform::LogLevel::Info, "A7_NATIVE_WM: WM INITIALIZE inst %d: %s\n", c.inst, b ? b->name : !Allowed(c) ? "native WM off (debug.litev.a7wm=0): emulated" : "WM build not recognised (or TWL native off): emulated");
        }
        else if (api == 0x1D) c.camLife = (u16)nds.ARM7Read32(a + 8);
        else if (c.build >= 1 && (c.guestCbSince || !c.lastPassed || (nds.ARM7Read16(c.lastPassed) & 0x8000)))
        {
            if (api == 0x0A || api == 0x1E || api == 0x07 || api == 0x08) take = true;
            else if (api == 0x26) take = c.v4 && nds.ARM7Read16(a + 0x14) == 0;   // (4.x scan for any DS parent; an SSID = an AP: WFC)
            else if (api == 0x0C)   // connect: to a DS parent (game info), not an access point (WFC)
            {
                const u32 d = nds.ARM7Read32(a + 4);
                take = (d >> 24) == 2 && nds.ARM7Read16(d + 0x3C) >= 0x10;
            }
        }
        if (!take || !Activate(c, api)) { c.lastPassed = a; c.guestCbSince = false; return false; }
        p = &c;
    }
#ifdef LITEV_HLE_DIAG
    TraceFifo(nds, 0, val);
#endif
    Ctx& c = *p;
    if ((val >> 5) & 1) return true;            // (error bit: ignored)
    if (c.reqs.size() >= 32) { Send(c, CbB(nds.ARM7Read16(a) & 0x7FFF, E_FIFO).P16(4, 0).P16(6, 0)); }
    else c.reqs.push_back({a, (u16)(nds.ARM7Read16(a) & 0x7FFF), 0});
    if (c.reqAt) c.reqAt += Us(16);             // (the guest's WL request in progress yields to the PXI IRQ taking this one)
    WakeAt(c, CpuNow(nds) + 64);                // (the ARM7 takes the request in its PXI IRQ)
    return true;
}

void Fifo7(NDS& nds, u32 val)
{
    Ctx* p = Get(nds);
    if (p && !p->active && (val & 31) == kTagWM) p->guestCbSince = true;
}

void ChipIRQ(NDS& nds)
{
    Ctx* p = Get(nds);
    if (!p || !p->active) return;
    p->stIrq++;
    p->irqPending = true;
    WakeAt(*p, CpuNow(nds) + 16);
}

void VLine(NDS& nds, u16 vcount, u64 ts)
{
    Ctx* p = Get(nds);
    if (!p || !p->active || (s32)vcount != p->vX) return;
    nds.GPU.WMLine = -1;
    p->lineDue = true; p->lineDueV = vcount;
    WakeAt(*p, ts + 1);
}
}
#endif
