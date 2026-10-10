// LITEV_A9HLE: see ARM9HLE.h.
#ifdef LITEV_A9HLE
#include "ARM9HLE.h"
#include "ARM.h"
#include "NDS.h"
#include "ARMInterpreter.h"
#ifdef LITEV_GX_BULK
#include "GPU.h"
#include "DMA_Timings.h"
#endif
#include "NDSCart.h"
#ifdef LITEV_HLE_DIAG
#include "Savestate.h"
#endif
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <chrono>
#if defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace melonDS::A9HLE
{
#ifdef LITEV_HLE_DIAG
bool CheckPending = false;
#endif
#ifdef LITEV_A9HLE_GXCHECK
std::vector<u32>* GxTap = nullptr;
bool GxOtherSeen = false;
#endif
namespace
{
// ---- NitroSDK ARM9 OS of Pokemon Black / White (USA/EU) ------------------------------------
// The same OS code in every variant; addresses differ per game binary (Variant). The first
// instruction words of the hooked entries are the same in all of them.
constexpr u32 kWakeInstr = 0xE58C2064;    // OS_IrqHandler_ThreadSwitch wake loop (queue not empty): str r2, [ip, #0x64]
constexpr u32 kSetInstr = 0xE92D47F0;     // OS_SetIrqFunction: push {r4-r10, lr}
constexpr u32 kGetInstr = 0xE59F207C;     // OS_GetIrqFunction: ldr r2, =OS_IRQTable
constexpr u32 kGxInstr = 0xE92D40F8;      // MIi_FIFOCallback: push {r3-r7, lr}
constexpr u32 kSetInstrT = 0xB4F0;        // TWL SDK (W2), Thumb OS_SetIrqFunction: push {r4-r7}
constexpr u32 kGetInstrT = 0xB418;        // TWL SDK (W2), Thumb OS_GetIrqFunction: push {r3, r4}
constexpr u32 kGxInstrT = 0xB5F8;         // TWL SDK (W2), Thumb MIi_FIFOCallback: push {r3-r7, lr}

// DTCM / ITCM objects (the same in every variant)
constexpr u32 kIrqQueue = 0x02FE00A0;   // OS_IrqThreadQueue {head, tail} (DTCM)
constexpr u32 kIrqCheck = 0x02FE3FF8;   // OS_IRQ check flags (DTCM)
constexpr u32 kIrqTable = 0x02FE0020;   // OS_IRQTable[32] (DTCM)
constexpr u32 kBiosLdm = 0xE8BD500F, kBiosSubs = 0xE25EF004;  // BIOS IRQ epilogue: ldmia sp!, {r0-r3,r12,lr}; subs pc, lr, #4
constexpr u32 kGxChunk = 0x1D8;             // bytes per DMA (118 words)
// (per variant, from its code ranges: OS_IrqHandler = code[0].a (the BIOS jumps to [DTCM+0x3FFC]),
// OS_IRQTable[1] (HBlank) = the Thumb stub code[12].a | 1, OS_IrqHandler after the table call = code[0].a + 0x58)
constexpr int kNumWake = 12;    // code ranges of 1. and 2. (signature sig); the rest: 3. (irqSig)

// 8. for the TWL SDK build (W2): NNS G3D in Thumb. SBC MAT (entry), MAT_InternalDefault, NNS_G3dGeBufferOP_N,
// MI_CpuSend32 (ARM), the material function table, the material cache, NNS_G3dGlb, the mask table, the GE buffer object
struct MatT { u32 sbc, def, opn, snd, tab, cache, glb, mask, ge; Range code[4]; u64 sig; };
constexpr MatT kW2Mat = {0x020668A4, 0x02066584, 0x02067D48, 0x020786B0, 0x0209B370, 0x02143CB4, 0x02143ACC, 0x02094440, 0x021469B4,
                         {{0x020668A4, 0x02066918}, {0x02066584, 0x020668A4}, {0x02067D48, 0x02067DD0}, {0x020786B0, 0x020786C8}},
                         0x4fbe252de9fcc4e2ull};

// OSThread: +0 context {cpsr, r0-r14, pc, sp_svc, CP context (+0x48, 0x1C bytes)}, +0x64 state,
// +0x68 list link, +0x70 priority, +0x78 queue, +0x7C/+0x80 queue link prev/next
struct Variant
{
    const char* name;
    u32 wake, set, get, gx;     // hook entries
    // every function the round trip / hooks execute, with literal pools (exact bytes = sig/irqSig)
    Range code[kNumCode];
    Range gxCode;               // 5.: MIi_FIFOCallback with its literal pool (gxSig)
    u64 sig, irqSig, gxSig;
    u32 irqTable2;              // DMA / timer / ... entries {func, enable, arg} x 12
    u32 info;                   // OSThreadInfo: +0 u16 isNeedRescheduling, +4 current, +8 list, +C switchCallback
    u32 osi;                    // +0 switchCallback, +4 (nonzero: no reschedule), +8 &current, +1E u16 reschedule lock
    u32 gxParams;               // MIi_GXDmaParams {busy, dmaNo, src, length, callback, arg, ...}
    u32 hbObjPtr;               // HBlank callback object = [[hbObjPtr + hbObjOff] + 0x18]
    // return addresses of a thread asleep in OS_WaitIrq: OS_SaveContext's resume point and the
    // call chain OS_RescheduleThread <- OS_SleepThread <- OS_WaitIrq loop
    u32 ctxPc, ctxLr, retSleep, retWait, retLoad;
    u32 idlePc, idleLr;         // halted in OS_Halt, called from the idle loop
    // 1: TWL SDK build (W2): thread functions and OS_Set/GetIrqFunction are Thumb (other register use:
    // OS_RescheduleThread holds the current thread, not OSThreadInfo, in r4), no hooks 3 and 5
    u8 twl;
    u8 hbObjOff;
    s32 setCyc, getCyc, getCycPerBit;   // ponytail: fixed cycle estimates (guest averages in check mode)
    // 10.: MI_SendGXCommandAsync and the functions its synchronous part runs (besides MIi_FIFOCallback, Set/Get)
    u32 async;
    Range asyncCode[7];
    u64 asyncSig;
    // 11.: the display list's DMA-end IRQ: OSi_IrqCallback + its literals + the OSi_IrqDma0-3 stubs (+0x88, 16 bytes
    // each), OS_DisableIrqMask, MIi_DMACallback (with the IRQ handler code[0] and OS_SetIrqFunction code[3])
    Range dmaCode[3];
    u64 dmaSig;
    // 13.: SBC SHP (its entry), {SHP_InternalDefault + SHP with the literal pool, the GE flush + NNS_G3dGeSendDL +
    // NNS_G3dGeBufferOP_N with literals, MI_CpuSend32}
    u32 shp;
    Range shpCode[3];
    u64 shpSig;
    const MatT* matT;           // 8. in Thumb (W2)
};
constexpr Variant kPW = {
    "PW", 0x01FF8160, 0x0208478C, 0x02084830, 0x02082864,
    {
        {0x01FF80F0, 0x01FF82A8},   // OS_IrqHandler + OS_IrqHandler_ThreadSwitch (ITCM)
        {0x020775D0, 0x0207764C},   // CP_SaveContext, CP_RestoreContext
        {0x020845B4, 0x02084628},   // OS_WaitIrq
        {0x0208478C, 0x020848BC},   // OS_SetIrqFunction, OS_GetIrqFunction
        {0x02084FE4, 0x0208505C},   // thread queue insert
        {0x020851B8, 0x0208527C},   // OS_RescheduleThread
        {0x020857C8, 0x02085818},   // OS_SleepThread
        {0x020858A8, 0x020858D4},   // select runnable thread
        {0x02085C20, 0x02085CD4},   // OS_SaveContext, OS_LoadContext
        {0x02085D54, 0x02085D9C},   // (TWL divider check)
        {0x020879A0, 0x020879CC},   // OS_DisableInterrupts, OS_RestoreInterrupts
        {0x02087A04, 0x02087A10},   // OS_GetProcMode
        // native HBlank IRQ (3.): the rest of the IRQ path (OS_IrqHandler is the first range)
        {0x02005204, 0x0200520C},   // OS_IRQTable[HBlank] stub (Thumb, literal)
        {0x02005610, 0x02005624},   // -> HBlank callback with its object (Thumb, literals)
        {0x02030D40, 0x02030DC0},   // HBlank callback (callback list walk)
        {0x02085B38, 0x02085B48},   // OS idle thread loop
        {0x020882E8, 0x020882F4},   // OS_Halt
    },
    {0x02082864, 0x02082918},
    0xc03b33186e018603ull, 0xba271beda6c559d8ull, 0x907c24457e1359d8ull,
    0x02150F58, 0x0215100C, 0x02150FF0, 0x02150E8C, 0x020AA1B4,
    0x02085C68, 0x02085224, 0x02085808, 0x02084610, 0x02085270,
    0x020882F0, 0x02085B44, 0, 0x10, 500, 27, 17,
    0x02082778,
    {
        {0x020825AC, 0x02082608},   // MI_WaitDma
        {0x02082724, 0x02082864},   // (DMA range check), MI_SendGXCommandAsync with its literal pool
        {0x020848BC, 0x02084904},   // OSi_EnterDmaCallback (DMA IRQ table entry + IE)
        {0x02084980, 0x020849B0},   // OS_EnableIrqMask
        {0x020849E0, 0x02084A0C},   // OS_ResetRequestIrqMask
        {0x01FF8020, 0x01FF80F0},   // MIi_DmaSetParams (ITCM)
        {0x020879A0, 0x020879CC},   // OS_DisableInterrupts, OS_RestoreInterrupts
    },
    0xb26c62ca8cd70cfeull,
    {{0x0208462C, 0x020846F4}, {0x020849B0, 0x020849E0}, {0x02082918, 0x02082978}},
    0x872c1f487669f8b6ull,
    0x0206C03C, {{0x0206BF68, 0x0206C0DC}, {0x0206DB98, 0x0206DEA0}, {0x02082AC0, 0x02082AD8}},
    0xf633c408c5df9847ull,
};
// Pokemon Black: the same code; main-binary OS code 0x18 bytes lower, its data 0x20 lower
// (located by masked code match against PW + the literal pools; tools/hle/xmap.py)
constexpr u32 B(u32 a) { return a - 0x18; }
constexpr Variant kPB = {
    "PB", 0x01FF8160, B(0x0208478C), B(0x02084830), B(0x02082864),
    {
        {0x01FF80F0, 0x01FF82A8}, {B(0x020775D0), B(0x0207764C)}, {B(0x020845B4), B(0x02084628)},
        {B(0x0208478C), B(0x020848BC)}, {B(0x02084FE4), B(0x0208505C)}, {B(0x020851B8), B(0x0208527C)},
        {B(0x020857C8), B(0x02085818)}, {B(0x020858A8), B(0x020858D4)}, {B(0x02085C20), B(0x02085CD4)},
        {B(0x02085D54), B(0x02085D9C)}, {B(0x020879A0), B(0x020879CC)}, {B(0x02087A04), B(0x02087A10)},
        {0x02005204, 0x0200520C}, {0x02005610, 0x02005624}, {B(0x02030D40), B(0x02030DC0)},
        {B(0x02085B38), B(0x02085B48)}, {B(0x020882E8), B(0x020882F4)},
    },
    {B(0x02082864), B(0x02082918)},
    0x2334bee5ccee4f77ull, 0x6d63379d6bebab20ull, 0x39d10b67f1dc9b7cull,
    0x02150F38, 0x02150FEC, 0x02150FD0, 0x02150E6C, 0x020AA194,
    B(0x02085C68), B(0x02085224), B(0x02085808), B(0x02084610), B(0x02085270),
    B(0x020882F0), B(0x02085B44), 0, 0x10, 500, 27, 17,
    B(0x02082778),
    {
        {B(0x020825AC), B(0x02082608)}, {B(0x02082724), B(0x02082864)}, {B(0x020848BC), B(0x02084904)},
        {B(0x02084980), B(0x020849B0)}, {B(0x020849E0), B(0x02084A0C)}, {0x01FF8020, 0x01FF80F0}, {B(0x020879A0), B(0x020879CC)},
    },
    0x3f2c017ca1395eceull,
    {{B(0x0208462C), B(0x020846F4)}, {B(0x020849B0), B(0x020849E0)}, {B(0x02082918), B(0x02082978)}},
    0x16d7560ac9ef0156ull,
    B(0x0206C03C), {{B(0x0206BF68), B(0x0206C0DC)}, {B(0x0206DB98), B(0x0206DEA0)}, {B(0x02082AC0), B(0x02082AD8)}},
    0x82f8ec64895be6faull,
};
// Pokemon White 2 (USA/EU), TWL SDK build: OS_IrqHandler and the ARM context switch code are PW's
// (ITCM +0xB98), OS_WaitIrq, CP and OS_Save/LoadContext ARM as in PW; OS_SleepThread,
// OS_RescheduleThread, the queue insert, the thread select, the divider check and
// OS_Set/GetIrqFunction are Thumb. 97 spurious wake round trips/frame in the overworld (the
// HBlank IRQ is PW's empty-callback-list case, 263/frame), OS_SetIrqFunction 186 + OS_GetIrqFunction 93
// calls/frame (every MI_SendGXCommandAsync: 22% of the town's ARM9 guest instructions), MIi_FIFOCallback
// (Thumb, same chunking as PW's) 287 GXFIFO IRQs/frame.
constexpr Variant kW2 = {
    "W2", 0x01FF8CF8, 0x02079D4C, 0x02079DB4, 0x020784FC,
    {
        {0x01FF8C88, 0x01FF8E40},   // OS_IrqHandler + OS_IrqHandler_ThreadSwitch (ITCM)
        {0x02070058, 0x020700D4},   // CP_SaveContext, CP_RestoreContext
        {0x02079BDC, 0x02079C50},   // OS_WaitIrq
        {0x02079D4C, 0x02079E10},   // OS_SetIrqFunction, OS_GetIrqFunction (Thumb)
        {0x0207A320, 0x0207A37C},   // thread queue insert (Thumb)
        {0x0207A460, 0x0207A4D0},   // OS_RescheduleThread (Thumb)
        {0x0207A894, 0x0207A8C8},   // OS_SleepThread (Thumb)
        {0x0207A92C, 0x0207A944},   // select runnable thread (Thumb)
        {0x0207AB9C, 0x0207AC50},   // OS_SaveContext, OS_LoadContext
        {0x0207ACB8, 0x0207ACE8},   // (TWL divider check, Thumb)
        {0x0207C110, 0x0207C13C},   // OS_DisableInterrupts, OS_RestoreInterrupts
        {0x0207C174, 0x0207C180},   // OS_GetProcMode
        // native HBlank IRQ (3.): the same empty-callback-list case as PW, the stub chain and callback Thumb
        {0x0200522C, 0x02005234},   // OS_IRQTable[HBlank] stub (Thumb, literal)
        {0x0200566C, 0x02005680},   // -> HBlank callback with its object (Thumb, literals)
        {0x0203A5FC, 0x0203A63C},   // HBlank callback (callback list walk, Thumb)
        {0x0207AAD4, 0x0207AAE0},   // OS idle thread loop (Thumb)
        {0x0207C814, 0x0207C820},   // OS_Halt
    },
    {0x020784FC, 0x02078578},       // MIi_FIFOCallback (Thumb) with its literal pool
    0x60444b04a2da8f6full, 0x47569f144546b503ull, 0x8b5e996183dc6c58ull,
    0x0214C1B8, 0x0214C26C, 0x0214C250, 0x0214C0EC, 0x0209DA98,
    0x0207ABE4, 0x0207A49F, 0x0207A8BB, 0x02079C38, 0x0207A4C7,
    0x0207C81C, 0x0207AADF, 1, 0x14, 417, 25, 13,     // check mode: set 417, get 302 at bit 21 (~13/bit, Thumb loop)
    0x02078454,     // 10. (Thumb): MI_SendGXCommandAsync and its callees (TWL SDK build)
    {
        {0x02078454, 0x020784FC},   // MI_SendGXCommandAsync with its literal pool
        {0x0207840C, 0x02078454},   // (DMA range check)
        {0x020782C8, 0x02078300},   // MI_WaitDma
        {0x02079E10, 0x02079F00},   // OSi_EnterDmaCallback, OS_EnableIrqMask, OS_DisableIrqMask, OS_ResetRequestIrqMask
        {0x01FF8BF0, 0x01FF8C88},   // MIi_DmaSetParams (ITCM, Thumb)
        {0x0207C110, 0x0207C13C},   // OS_DisableInterrupts, OS_RestoreInterrupts
        {0, 0},
    },
    0x6d62f51b28952570ull,
    // 11. (Thumb): OSi_IrqCallback + literals + the 12 OSi_IrqDma/Timer stubs (+0x58, 12 bytes each), OS_DisableIrqMask,
    // MIi_DMACallback with its literals
    {{0x02079C54, 0x02079D3C}, {0x02079EC0, 0x02079EE4}, {0x0207857C, 0x020785C0}},
    0x0aa0dbea6ca9901full,
    // 13. (Thumb): SBC SHP; {SHP_InternalDefault + SBC SHP + literal, the GE flush .. NNS_G3dGeBufferOP_N with literals,
    // MI_CpuSend32}
    0x020669A8, {{0x02066918, 0x02066A10}, {0x02067BC8, 0x02067DD0}, {0x020786B0, 0x020786C8}},
    0x396f02a263a36a88ull,
    &kW2Mat,
};
constexpr const Variant* kVariants[] = {&kPW, &kPB, &kW2};
constexpr int kNumVariants = sizeof(kVariants) / sizeof(kVariants[0]);
// hook kind of (addr, instr) in variant v: 0 wake, 1 set, 2 get, 5 GX send, -1 none (thumb: a Thumb entry, instr the halfword)
inline int Kind(const Variant& v, u32 addr, u32 instr, bool thumb = false)
{
    if (!thumb && v.async && addr == v.async && instr == kGxInstr) return 10;
    if (!thumb && v.shp && addr == v.shp && instr == 0xE92D4010) return 13;
    if (thumb) return !v.twl ? -1 : addr == v.set && instr == kSetInstrT ? 1 : addr == v.get && instr == kGetInstrT ? 2
                    : v.gx && addr == v.gx && instr == kGxInstrT ? 5 : v.matT && addr == v.matT->sbc && instr == 0xB570 ? 8
                    : v.async && addr == v.async && instr == 0xB5F8 ? 10 : v.shp && addr == v.shp && instr == 0xB570 ? 13 : -1;
    return addr == v.wake && instr == kWakeInstr ? 0 : !v.twl && addr == v.set && instr == kSetInstr ? 1
         : !v.twl && addr == v.get && instr == kGetInstr ? 2 : v.gx && addr == v.gx && instr == kGxInstr ? 5 : -1;
}

// ponytail: fixed cycle estimate of the wake round trip (guest average measured in check mode on PW)
constexpr s32 kWakeCycles = 900;
// 3.: guest averages in check mode (PW f17000 / f6500): IRQ entry to return, empty queue / with the wake round trip
constexpr s32 kIrqCycles = 158, kIrqWakeCycles = 1032;

constexpr int kKinds = 15;    // 0 wake, 1 set, 2 get, 3 HBlank, 4 HBlank+wake, 5 GX send, 6 LZ, 7 card read, 8 G3D material, 9 _ll_sdiv, 10 GX async start, 11 GX DMA-end IRQ
constexpr u32 kAllHooks = 8191;
struct State
{
    int status = 0;                 // 0 no variant matched (yet), 1 active (v), -1 off
    const Variant* v = nullptr;     // the game's variant (status 1)
    u32 tried = 0;                  // variants whose signature was checked (bit per kVariants entry)
    bool init = false, on = true;   // prop read; prop on
    u32 mask = kAllHooks;           // 1 wake, 2 set, 4 get, 8 HBlank IRQ, 16 GX send, 32 card read, 64 LZ, 128 G3D material, 256 _ll_sdiv, 512 GX async start, 1024 GX DMA-end IRQ, 2048 G3D shape, 4096 VEC_Normalize
    std::vector<u8> code, irqCode, gxCode, asyncCode, dmaCode, shpCode;
    bool gxOk = false;              // MIi_FIFOCallback matches the variant (5.)
    bool asyncOk = false;           // MI_SendGXCommandAsync's synchronous part matches (10.)
    bool irqOk = false;             // IRQ path code matches the variant (3.)
    bool dmaOk = false;             // DMA-end IRQ path matches (11.)
    bool shpOk = false;             // G3D shape path matches (13.)
    bool matTOk = false;            // 8. in Thumb (W2) matches
    std::vector<u8> matTCode;
    bool hbLive = false, dmaLive = false;   // 3. / 11. verified when the wake hook block (A9HLEGuard) was compiled
    u32 biosRet = 0;                // BIOS IRQ entry verified (once): its return address, else 0
    u64 calls[kKinds] = {}, native[kKinds] = {}, fallback[kKinds] = {}, checks[kKinds] = {}, diffs[kKinds] = {}, irqDuring[kKinds] = {};
    u64 guestCyc[kKinds] = {}, guestN[kKinds] = {}, getBits = 0, gxWords = 0, lzBytes = 0, lzEst = 0, cardWords = 0;
    u32 biosOk = 0;                 // BIOS IRQ epilogue address verified once
    u64 ns = 0;                     // stats: host time inside Run (native calls)
    // host pointers of the fixed-address OS objects, valid for fkey (DTCM base/mask, ITCM size)
    const u8 *fq = nullptr, *fchk = nullptr, *fosi = nullptr, *fhp = nullptr, *ftb = nullptr, *fop = nullptr;
    // OS_IRQTable (32 words) and the DMA/timer table (12 entries of 3 words), for 2. and 4.
    u8 *ftab = nullptr, *ftab2 = nullptr;
    bool ftabD = false, ftab2D = false;     // in DTCM
    u32 fkey[3] = {~0u, ~0u, ~0u};
};
std::unordered_map<const melonDS::NDS*, State> g_State;
bool g_Stats = getenv("LITEV_A9HLE_STATS") && atoi(getenv("LITEV_A9HLE_STATS"));
#ifdef LITEV_HLE_DIAG
bool g_Check = getenv("LITEV_A9HLE_CHECK") && atoi(getenv("LITEV_A9HLE_CHECK"));
// cost measurement: compute the native result (nothing written) and run the guest code anyway
bool g_Dry = getenv("LITEV_A9HLE_DRY") && atoi(getenv("LITEV_A9HLE_DRY"));
// same, IRQ path (3.) only
bool g_DryIrq = g_Dry || (getenv("LITEV_A9HLE_DRYIRQ") && atoi(getenv("LITEV_A9HLE_DRYIRQ")));
const bool g_Time = g_Stats;    // host ns per native call
#else
// shipping: no compare / dry / timing code in the hooks (the in-order A55 pays for every hot byte)
constexpr bool g_Check = false, g_Dry = false, g_DryIrq = false, g_Time = false;
#endif
const char* kName[kKinds] = {"irqwake", "setirqfn", "getirqfn", "hblank", "hblank+wake", "gxsend", "lz", "cardread", "g3dmat", "llsdiv", "gxasync", "dmairq", "irqdefer", "g3dshp", "vecnorm"};
// kind -> LITEV_A9HLE_ONLY / debug.litev.a9hle mask bit
inline u32 Bit(int k) { return k == 5 ? 16 : k == 6 ? 64 : k == 7 ? 32 : k == 8 ? 128 : k == 9 ? 256 : k == 10 ? 512 : k == 11 ? 1024 : k == 13 ? 2048 : k == 14 ? 4096 : 1u << k; }

// stats only
u64 Now() { return (u64)std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
double TimerNs()   // cost of one Now() pair, subtracted from the per-call figure
{
    u64 t = Now(), z = 0;
    for (int i = 0; i < 100000; i++) { u64 a = Now(); z += Now() - a; }
    (void)t;
    return (double)z / 100000;
}
inline u32 R32(const u8* p) { u32 v; memcpy(&v, p, 4); return v; }
inline u16 R16(const u8* p) { u16 v; memcpy(&v, p, 2); return v; }

// 0 off, 1 (or unset) all hooks, else the mask of hooks (as LITEV_A9HLE_ONLY)
u32 ReadOn()
{
#if defined(__ANDROID__)
    char b[16] = {0}; int n = __system_property_get("debug.litev.a9hle", b);
    const char* e = n > 0 ? b : nullptr;
#else
    const char* e = getenv("debug.litev.a9hle");
#endif
    return e ? (u32)strtoul(e, nullptr, 0) : 1;
}

// code bytes as the ARM9 sees them (ITCM or main RAM), no timing / side effects
const u8* CodePtr(melonDS::ARMv5* c, u32 a)
{
    if (a < c->ITCMSize) return &c->ITCM[a & (ITCMPhysicalSize - 1)];
    if ((a >> 24) == 0x02) return &c->NDS.MainRAM[a & c->NDS.MainRAMMask];
    return nullptr;
}

void CodeBytes(melonDS::ARMv5* c, std::vector<u8>& out, const Range* r, int from, int to)
{
    out.clear();
    for (int i = from; i < to; i++)
        for (u32 a = r[i].a; a < r[i].b; a++) { const u8* p = CodePtr(c, a); out.push_back(p ? *p : 0); }
}

u64 Fnv(const std::vector<u8>& v)
{
    u64 h = 0xcbf29ce484222325ull;
    for (u8 b : v) h = (h ^ b) * 0x100000001b3ull;
    return h;
}

bool IrqCodeIntact(melonDS::ARMv5* c, const State& s)
{
    size_t o = 0;
    const Range* r = s.v->code;
    for (int i = kNumWake; i < kNumCode; i++)
    {
        u32 n = r[i].b - r[i].a;
        const u8* p = CodePtr(c, r[i].a);
        if (!p || memcmp(p, s.irqCode.data() + o, n) != 0) return false;
        o += n;
    }
    return true;
}

// BIOS IRQ entry (FreeBIOS or the original): push {r0-r3, r12, lr}; DTCM base; lr = return;
// ldr pc, [DTCM + 0x3FFC]; then the epilogue. Returns the return address (0: unknown BIOS).
u32 BiosIrqRet(melonDS::NDS& nds)
{
    const u8* b = nds.GetARM9BIOS().data();
    const u32 v = R32(b + 0x18);
    if ((v >> 24) != 0xEA) return 0;
    u32 a = 0x18 + 8 + (u32)(((s32)(v << 8)) >> 6);
    auto w = [&](u32 x) { return x + 4 <= ARM9BIOSSize ? R32(b + x) : 0u; };
    if (w(a) != 0xE92D500F || w(a + 4) != 0xEE190F11) return 0;   // push; mrc p15, 0, r0, c9, c1, 0
    a += 8;
    if (w(a) == 0xE3C000FF) a += 4;                                   // bic r0, r0, #0xFF
    else if (w(a) == 0xE1A00620 && w(a + 4) == 0xE1A00600) a += 8;    // lsr/lsl #12
    else return 0;
    if (w(a) != 0xE2800901) return 0;                                 // add r0, r0, #0x4000
    if (w(a + 4) != 0xE1A0E00F && w(a + 4) != 0xE28FE000) return 0;  // mov lr, pc / add lr, pc, #0
    if (w(a + 8) != 0xE510F004) return 0;                             // ldr pc, [r0, #-4]
    a += 12;
    if (w(a) != kBiosLdm || w(a + 4) != kBiosSubs) return 0;
    return 0xFFFF0000 | a;
}

bool CodeIntact(melonDS::ARMv5* c, const State& s, int k)
{
    size_t o = 0;
    for (int i = 0; i < kNumWake; i++)
    {
        const Range& r = s.v->code[i];
        if (k != 0 && i != 3) { o += r.b - r.a; continue; }     // 3: OS_SetIrqFunction, OS_GetIrqFunction
        u32 n = r.b - r.a;
        const u8* p = CodePtr(c, r.a);
        // ranges never straddle a mirror boundary
        if (!p || memcmp(p, s.code.data() + o, n) != 0) return false;
        o += n;
    }
    return true;
}

bool DmaIntact(melonDS::ARMv5* c, const State& s)
{
    size_t o = 0;
    for (const Range& r : s.v->dmaCode)
    {
        const u8* p = CodePtr(c, r.a);
        if (!p || memcmp(p, s.dmaCode.data() + o, r.b - r.a) != 0) return false;
        o += r.b - r.a;
    }
    return true;
}

bool ShpIntact(melonDS::ARMv5* c, const State& s)
{
    size_t o = 0;
    for (const Range& r : s.v->shpCode)
    {
        const u8* p = CodePtr(c, r.a);
        if (!p || memcmp(p, s.shpCode.data() + o, r.b - r.a) != 0) return false;
        o += r.b - r.a;
    }
    return true;
}

bool MatTIntact(melonDS::ARMv5* c, const State& s)
{
    size_t o = 0;
    for (const Range& r : s.v->matT->code)
    {
        const u8* p = CodePtr(c, r.a);
        if (!p || memcmp(p, s.matTCode.data() + o, r.b - r.a) != 0) return false;
        o += r.b - r.a;
    }
    return true;
}

bool AsyncIntact(melonDS::ARMv5* c, const State& s)
{
    size_t o = 0;
    for (const Range& r : s.v->asyncCode)
    {
        const u8* p = CodePtr(c, r.a);
        if (!p || memcmp(p, s.asyncCode.data() + o, r.b - r.a) != 0) return false;
        o += r.b - r.a;
    }
    return true;
}

bool GxIntact(melonDS::ARMv5* c, const State& s)
{
    const Range& r = s.v->gxCode;
    const u8* p = CodePtr(c, r.a);
    return p && memcmp(p, s.gxCode.data(), r.b - r.a) == 0;
}

// The game's variant is found when the JIT (or the interpreter) first reaches a hook entry of one:
// that variant's code signature is checked then (the OS code is in place by the time it runs).
// the console's state with the runtime switch read (no variant probing)
__attribute__((noinline, cold)) State& Init(melonDS::ARMv5* c)
{
    State& s = g_State[&c->NDS];         // map nodes are stable
    c->A9HLEState = &s;
    if (s.init) return s;
    s.init = true;
    const u32 on = ReadOn();
    if (!on) { s.on = false; s.status = -1; return s; }
    if (on != 1) s.mask = on;
    if (const char* m = getenv("LITEV_A9HLE_ONLY")) s.mask = (u32)strtoul(m, nullptr, 0);
    return s;
}
inline State& St(melonDS::ARMv5* c) { return c->A9HLEState ? *(State*)c->A9HLEState : Init(c); }

__attribute__((noinline, cold)) State& Probe(melonDS::ARMv5* c, u32 addr, u32 instr, bool thumb)
{
    State& s = St(c);
    if (s.status != 0) return s;
    for (int i = 0; i < kNumVariants; i++)
    {
        const Variant& v = *kVariants[i];
        if ((s.tried >> i & 1) || Kind(v, addr, instr, thumb) < 0) continue;
        s.tried |= 1u << i;
        std::vector<u8> code;
        CodeBytes(c, code, v.code, 0, kNumWake);
        const u64 h = Fnv(code);
        if (g_Stats) fprintf(stderr, "A9HLE: %s signature %016llx %s\n", v.name, (unsigned long long)h, h == v.sig ? "matches" : "differs");
        if (h != v.sig) continue;
        s.v = &v;
        s.code = std::move(code);
        s.status = 1;
        CodeBytes(c, s.irqCode, v.code, kNumWake, kNumCode);
        s.biosRet = BiosIrqRet(c->NDS);
        s.irqOk = Fnv(s.irqCode) == v.irqSig && s.biosRet;
        if (g_Stats) fprintf(stderr, "A9HLE: IRQ path signature %016llx, BIOS IRQ return %08x -> %s\n", (unsigned long long)Fnv(s.irqCode), s.biosRet, s.irqOk ? "on" : "off");
        for (u32 a = v.gxCode.a; a < v.gxCode.b; a++) { const u8* p = CodePtr(c, a); s.gxCode.push_back(p ? *p : 0); }
        s.gxOk = Fnv(s.gxCode) == v.gxSig;
        if (g_Stats) fprintf(stderr, "A9HLE: GX send signature %016llx -> %s\n", (unsigned long long)Fnv(s.gxCode), s.gxOk ? "on" : "off");
        if (v.matT)
        {
            CodeBytes(c, s.matTCode, v.matT->code, 0, 4);
            s.matTOk = Fnv(s.matTCode) == v.matT->sig;
            if (g_Stats) fprintf(stderr, "A9HLE: G3D material (Thumb) signature %016llx -> %s\n", (unsigned long long)Fnv(s.matTCode), s.matTOk ? "on" : "off");
        }
        if (v.async)
        {
            CodeBytes(c, s.asyncCode, v.asyncCode, 0, 7);
            s.asyncOk = Fnv(s.asyncCode) == v.asyncSig;
            if (g_Stats) fprintf(stderr, "A9HLE: GX async start signature %016llx -> %s\n", (unsigned long long)Fnv(s.asyncCode), s.asyncOk ? "on" : "off");
            CodeBytes(c, s.dmaCode, v.dmaCode, 0, 3);
            s.dmaOk = Fnv(s.dmaCode) == v.dmaSig && s.biosRet;
            if (g_Stats) fprintf(stderr, "A9HLE: GX DMA-end IRQ signature %016llx -> %s\n", (unsigned long long)Fnv(s.dmaCode), s.dmaOk ? "on" : "off");
            CodeBytes(c, s.shpCode, v.shpCode, 0, 3);
            s.shpOk = s.asyncOk && Fnv(s.shpCode) == v.shpSig;
            if (g_Stats) fprintf(stderr, "A9HLE: G3D shape signature %016llx -> %s\n", (unsigned long long)Fnv(s.shpCode), s.shpOk ? "on" : "off");
        }
        break;
    }
    if (!s.status && s.tried == (1u << kNumVariants) - 1) s.status = -1;    // no variant matches
    return s;
}
// the console's state for a hook entry (addr, instr): probes the variants with a hook there
inline State& Get(melonDS::ARMv5* c, u32 addr, u32 instr, bool thumb = false)
{
    State* s = (State*)c->A9HLEState;
    return s && s->status != 0 ? *s : Probe(c, addr, instr, thumb);
}
// the console's state once a variant is active, else nullptr (no probing)
inline State* Active(melonDS::ARMv5* c)
{
    State* s = (State*)c->A9HLEState;
    return s && s->status == 1 ? s : nullptr;
}

// ---- guest memory view with a write log --------------------------------------------------
// Reads are host loads from main RAM / DTCM. Writes (only after every address was validated)
// go straight to memory and only when the word changes; in check mode they are logged instead.
struct Wr { u32 a, v; u8 sz; };
// a validated, contiguous guest object in host memory
struct Obj
{
    u8* p = nullptr; u32 a = 0; bool dtcm = false;
    explicit operator bool() const { return p != nullptr; }
    u32 r(u32 o) const { return R32(p + o); }
    u16 r16(u32 o) const { return R16(p + o); }
};
struct Mem
{
    melonDS::ARMv5* c;
#ifdef LITEV_HLE_DIAG
    bool logOnly;
    u32 n = 0;
    bool bad = false;
    Wr log[128];
    Mem(melonDS::ARMv5* cpu, bool check) : c(cpu), logOnly(check) {}
#else
    static constexpr bool logOnly = false;
    Mem(melonDS::ARMv5* cpu, bool) : c(cpu) {}
#endif
    // DTCM or main RAM (the only places the hooked code writes), else nullptr
    __attribute__((always_inline)) u8* P(u32 a)
    {
        if (a < c->ITCMSize) return nullptr;
        if ((a & c->DTCMMask) == c->DTCMBase) return &c->DTCM[a & (DTCMPhysicalSize - 1)];
        if ((a >> 24) == 0x02) return &c->NDS.MainRAM[a & c->NDS.MainRAMMask];
        return nullptr;
    }
    // [a, a+len) in one host block (main RAM or DTCM, no mirror wrap), else empty
    __attribute__((always_inline)) Obj O(u32 a, u32 len)
    {
        Obj x;
        u8* p = P(a);
        if (!p || (a & 3) || P(a + len - 1) != p + len - 1) return x;
        x.p = p; x.a = a; x.dtcm = (a & c->DTCMMask) == c->DTCMBase;
        return x;
    }
    // queue a word write (applied by Flush, after every check passed); a: guest address (word
    // aligned), bit 0 set for DTCM (never holds JIT code: no invalidation check)
    struct Pw { u8* p; u32 a, v; } pw[96];   // (the W2 shape fold queues ~71)
    u32 np = 0;
    __attribute__((always_inline)) void W(const Obj& x, u32 o, u32 v) { pw[np++] = {x.p + o, (x.a + o) | (u32)x.dtcm, v}; }
    // only changed words are written, main RAM with the JIT invalidation check of ARM9Write32
    // (same effect, no bus dispatch); in check mode they are logged instead
    void Flush()
    {
        for (u32 i = 0; i < np; i++)
        {
            const Pw& w = pw[i];
#ifdef LITEV_HLE_DIAG
            if (__builtin_expect(logOnly, 0))
            {
                if (n < 128) log[n++] = {w.a & ~1u, w.v, 4}; else bad = true;
                continue;
            }
#endif
            if (R32(w.p) == w.v) continue;
            if (!(w.a & 1)) c->NDS.JIT.CheckAndInvalidate<0, melonDS::ARMJIT_Memory::memregion_MainRAM>(w.a);
            memcpy(w.p, &w.v, 4);
        }
        np = 0;
    }
};

// fixed-address objects: validated once per memory map (DTCM/ITCM setting) instead of per call
__attribute__((noinline)) bool Refix(melonDS::ARMv5* c, State& s, Mem& m)
{
    s.fkey[0] = c->DTCMBase; s.fkey[1] = c->DTCMMask; s.fkey[2] = c->ITCMSize;
    const Variant& v = *s.v;
    Obj q = m.O(kIrqQueue, 8), chk = m.O(kIrqCheck, 4), osi = m.O(v.osi, v.info + 0x10 - v.osi);
    Obj hp = m.O(c->DTCMBase + 0x3FFC, 4), tb = m.O(kIrqTable + 4, 4), op = m.O(v.hbObjPtr + v.hbObjOff, 4);
    s.fq = q.p; s.fchk = chk.p; s.fosi = osi.p; s.fhp = hp.p; s.ftb = tb.p; s.fop = op.p;
    Obj t1 = m.O(kIrqTable, 128), t2 = m.O(v.irqTable2, 12 * 12);
    s.ftab = t1.p; s.ftabD = t1.dtcm; s.ftab2 = t2.p; s.ftab2D = t2.dtcm;
    return q && chk && osi;
}
inline bool Fixed(melonDS::ARMv5* c, State& s, Mem& m)
{
    if (__builtin_expect(s.fkey[0] == c->DTCMBase && s.fkey[1] == c->DTCMMask && s.fkey[2] == c->ITCMSize, 1))
        return s.fq && s.fchk && s.fosi;
    return Refix(c, s, m);
}

struct Expect
{
    u32 R[16]; u32 CPSR; u32 IRQ[3]; u32 SVC[3]; bool banks = false;
    u32 retPc = 0; u32 cur = 0;     // cur: wake check also requires this current thread
};
#ifdef LITEV_HLE_DIAG
// ---- check mode -----------------------------------------------------------------------------
struct Pending
{
    int kind = 0;
    Expect e;
    std::vector<Wr> log;
    std::vector<u8> ram, dtcm;
    std::vector<melonDS::ARMv5::Idle2Access> acc;
    u64 t0 = 0, steps = 0;
    bool irq = false;
    bool vecSkip = false;   // native IRQ check: the guest's own entry through the IRQ vector
    u32 lzn[4] = {};        // LZ check: token counts of the native chunk
    u32 lzLo = 0, lzHi = 0; // LZ check: the window (an IRQ inside the guest chunk writes elsewhere)
    u32 lzFn = 0;           // LZ check: function entry; cycles of the instructions inside it (no IRQs)
    u64 lzOwn = 0, lzLastTs = 0; bool lzLastIn = false;
    std::vector<u32> gx;    // G3D material check: the GXFIFO words of the native path
    bool gxOn = false;      // ... captured from BulkWords (GxTap) + the guest's GXFIFO stores
    u32 fit[4] = {};        // _ll_sdiv check: normalization shifts, dividend shifts, nonzero, estimate
    std::vector<std::pair<u32, u32>> io;    // GX async check: the IO writes of the native plan (IME toggles left out)
    Expect fin; u32 finA = 0, finV = 0;     // G3D shape check, stage 2: registers at SHP's return, the SBC pointer word
    bool stage2 = false, small = false;     // G3D shape check: stage 2 pending; the small-list path (no IO, GX words)
} g_P;
#ifdef LITEV_A9HLE_GXCHECK
std::vector<u32> g_GxMatTap;
#endif

__attribute__((noinline, cold)) void ArmCheck(melonDS::ARMv5* c, int kind, const Mem& m, const Expect& e)
{
    melonDS::NDS& nds = c->NDS;
    g_P.kind = kind; g_P.e = e; g_P.log.assign(m.log, m.log + m.n); g_P.irq = false; g_P.steps = 0;
    g_P.ram.assign(nds.MainRAM, nds.MainRAM + nds.MainRAMMask + 1);
    g_P.dtcm.assign(c->DTCM, c->DTCM + DTCMPhysicalSize);
    g_P.acc.clear();
    c->Idle2Log = &g_P.acc;
    g_P.t0 = nds.ARM9Timestamp + c->Cycles;
    CheckPending = true;
}
#endif

void GuestFallback(melonDS::ARM* cpu)
{
    if (cpu->CPSR & 0x20) { ARMInterpreter::THUMBInstrTable[(cpu->CurInstr >> 6) & 0x3FF](cpu); return; }
    u32 icode = ((cpu->CurInstr >> 4) & 0xF) | ((cpu->CurInstr >> 16) & 0xFF0);
    ARMInterpreter::ARMInstrTable[icode](cpu);
}

u32 Flags(u32 a, u32 b)   // NZCV of cmp a, b
{
    u32 r = a - b;
    return (r & 0x80000000) | ((r == 0) << 30) | ((a >= b) << 29) | ((((a ^ b) & (a ^ r)) >> 31) << 28);
}

// ---- 1. spurious OS_WaitIrq wake-up ---------------------------------------------------------
// At the first iteration of OS_IrqHandler_ThreadSwitch's wake loop (IRQ mode, ip = queue head;
// the hook sits inside the loop so IRQs with an empty queue never leave the JIT). Returns false (guest runs) unless the only thread
// in OS_IrqThreadQueue is asleep in OS_WaitIrq with its IRQ flags still clear, it is the thread
// the switch would pick, and the interrupted thread is the one picked once it sleeps again.
// An IRQ that is already pending (or arrives during the guest's round trip) would be taken in
// the woken thread's short IRQs-on window; here it is taken right after the return instead.
// The IRQ context at the wake loop: the interrupted thread's CPSR, the IRQ stack pointer there (F),
// SVC sp, the interrupted thread's sp/lr, the queue head ip, and (native IRQ, 3.) the IRQ frame
// values that are still only queued writes.
struct IrqIn { u32 spsr, F, svcsp, sysSp, sysLr, ip; bool queued; u32 lrb, f[6]; };

bool Wake(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, const IrqIn& in, u32& tpc, u32& tcpsr)
{
    melonDS::NDS& nds = c->NDS;
    const u32 spsr = in.spsr;
    if ((spsr & 0x1F) != 0x1F && (spsr & 0x1F) != 0x10) return false;
    const u32 F = in.F, svcsp = in.svcsp;
    // host views of every guest object touched (validated; contiguous)
    if (!Fixed(c, s, m)) return false;
    Obj fr = m.O(F - 12, 40), sv = m.O(svcsp - 24, 24);
    if (!fr || !sv) return false;
    const u8* osi = s.fosi;
    // in-order cores: start the independent cold loads together
    __builtin_prefetch(osi); __builtin_prefetch(osi + 32); __builtin_prefetch(fr.p); __builtin_prefetch(sv.p);
    const u32 t = R32(s.fq);
    if (!t || in.ip != t || R32(s.fq + 4) != t) return false;
    const Variant& v = *s.v;
    const u32 info = v.info - v.osi;
    if (R16(osi + info) || R32(osi + info + 0xC) || R32(osi) || R32(osi + 4) || R16(osi + 0x1E) || R32(osi + 8) != v.info + 4) return false;
    const u32 cur = R32(osi + info + 4);
    if (!cur || cur == t) return false;
    Obj tp = m.O(t, 0x84), cp = m.O(cur, 0x64);
    if (!tp || !cp) return false;
    __builtin_prefetch(tp.p); __builtin_prefetch(tp.p + 64); __builtin_prefetch(cp.p); __builtin_prefetch(cp.p + 64);
    // t asleep in OS_WaitIrq on this queue, flags not satisfied
    if (tp.r(0x80) || tp.r(0x7C) || tp.r(0x64) != 0 || tp.r(0x78) != kIrqQueue || tp.r(0x40) != v.ctxPc || tp.r(0x3C) != v.ctxLr)
        return false;
    const u32 tsp = tp.r(0x38);
    Obj ts = m.O(tsp - 12, 44);
    if (!ts) return false;
    if (ts.r(24) != v.retSleep || ts.r(40) != v.retWait || ts.r(32) != kIrqQueue || ts.r(36) != kIrqCheck) return false;
    if (ts.r(28) & R32(s.fchk)) return false;                     // real wake-up
    tcpsr = tp.r(0);
    tpc = v.ctxPc;
    if ((tcpsr & 0xFF) != 0x9F) return false;                     // SYS mode, ARM, IRQs off
    // the switch picks t (first thread that is ready or t), and with t asleep again the first
    // ready thread is cur
    u32 first = 0, firstReady = 0;
    u32 x = R32(osi + info + 8);
    const u8* ram = nds.MainRAM;
    const u32 rmask = nds.MainRAMMask;
    for (int i = 0; x && i < 64; i++)
    {
        // thread structs: main RAM, not under DTCM, no wrap (+0x64 state, +0x68 next)
        if ((x >> 24) != 0x02 || ((x + 0x64) & c->DTCMMask) == c->DTCMBase || ((x + 0x6B) & c->DTCMMask) == c->DTCMBase
            || ((x + 0x64) & rmask) + 8 > rmask + 1 || (x & 3)) return false;
        const u8* xp = ram + ((x + 0x64) & rmask);
        u32 st = R32(xp);
        if (st > 0xFFFF) return false;
        if (!first && (st == 1 || x == t)) first = x;
        if (!firstReady && st == 1 && x != t) firstReady = x;
        if (first && firstReady) break;
        x = R32(xp + 4);
    }
    if (first != t || firstReady != cur) return false;
    // BIOS IRQ frame: [F] = return into the BIOS, [F+4..F+28) = r0-r3, r12, lr  (fr: F-12..F+28)
    const u32 lrb = in.queued ? in.lrb : fr.r(12);
    if (lrb != s.biosOk && !in.queued)
    {
        if ((lrb >> 12) != 0xFFFF0 || (lrb & 0xFFF) > ARM9BIOSSize - 8) return false;
        const u8* bios = nds.GetARM9BIOS().data() + (lrb & 0xFFF);
        if (R32(bios) != kBiosLdm || R32(bios + 4) != kBiosSubs) return false;
        s.biosOk = lrb;
    }
    u32 f[6];
    for (int i = 0; i < 6; i++) f[i] = in.queued ? in.f[i] : fr.r(16 + i * 4);
    const u32 tf[6] = {tp.r(4), tp.r(8), tp.r(0xC), tp.r(0x10), tp.r(0x34), v.ctxPc};

    // interrupted thread's context, as the IRQ path saves it
    m.W(cp, 0x00, spsr);
    for (int i = 0; i < 4; i++) m.W(cp, 4 + i * 4, f[i]);
    for (int i = 4; i < 12; i++) m.W(cp, 4 + i * 4, c->R[i]);
    m.W(cp, 0x34, f[4]);
    m.W(cp, 0x38, in.sysSp);                                     // user/sys r13, r14 (banked out in IRQ mode)
    m.W(cp, 0x3C, in.sysLr);
    m.W(cp, 0x40, f[5]);
    m.W(cp, 0x44, svcsp);
    // CP_SaveContext: divider numerator/denominator, sqrt param, DIVCNT&3, SQRTCNT&1
    // (the register values themselves: no lazy-divider materialisation needed for these bits)
    u32 cpx[7];
    nds.A9HLECpContext(cpx);
    for (int i = 0; i < 7; i++) m.W(cp, 0x48 + i * 4, cpx[i]);
    // IRQ stack below the frame: CP_SaveContext's push {r4}, push {r0 = C, r1 = t}
    m.W(fr, 0, c->R[4]);
    m.W(fr, 4, cur);
    m.W(fr, 8, t);
    // the IRQ frame rewritten with t's registers (the BIOS returned into t with them)
    for (int i = 0; i < 6; i++) m.W(fr, 16 + i * 4, tf[i]);
    // t slept again: its context holds r5 = the thread it switched to; its stack holds
    // OS_LoadContext's push {r0 = cur, lr} and CP_RestoreContext's push {r4 = OS_RescheduleThread's
    // r4: OSThreadInfo, TWL build: the current thread t}
    m.W(tp, 0x18, cur);
    m.W(ts, 0, v.twl ? t : v.info);
    m.W(ts, 4, cur);
    m.W(ts, 8, v.retLoad);
    // OS_LoadContext(cur) pushed {r0-r3, r12, lr = pc} on cur's SVC stack
    for (int i = 0; i < 6; i++) m.W(sv, i * 4, f[i]);
    m.Flush();

    // registers after the guest's return into cur
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    for (int i = 0; i < 4; i++) e.R[i] = f[i];
    e.R[12] = f[4];
    e.R[13] = in.sysSp;
    e.R[14] = in.sysLr;
    e.CPSR = spsr;
    e.IRQ[0] = F + 28; e.IRQ[1] = tpc; e.IRQ[2] = tcpsr;
    e.SVC[0] = svcsp; e.SVC[1] = f[5]; e.SVC[2] = spsr;
    e.banks = true;
    e.retPc = (f[5] - 4) & ((spsr & 0x20) ? ~1u : ~3u);
    e.cur = cur;
    return true;
}

void WakeCommit(melonDS::ARMv5* c, const Expect& e)
{
    c->R_SVC[1] = e.SVC[1];
    c->R_SVC[2] = e.SVC[2];
    for (int i = 0; i < 4; i++) c->R[i] = e.R[i];
    c->R[12] = e.R[12];
    c->R[13] = e.IRQ[0];
    c->R[14] = e.SVC[1];
    c->Cycles += kWakeCycles;
    c->JumpTo(e.SVC[1] - 4, true);      // subs pc, lr, #4: CPSR = SPSR_irq, IRQ bank out
    c->R_IRQ[1] = e.IRQ[1];
    c->R_IRQ[2] = e.IRQ[2];
}

// ---- 2. OS_SetIrqFunction(mask, func) ----------------------------------------------------
// table entry of IRQ bit i: offset in the DMA/timer table (kIrqTable2), or -1: OS_IRQTable[i]
inline int Ent2(u32 i) { return i >= 8 && i <= 11 ? (i - 8) * 12 : i >= 0x1C ? (i - 0x18) * 12 : i >= 3 && i <= 6 ? (i + 5) * 12 : -1; }

bool SetIrq(melonDS::ARMv5* c, State& s, Mem& m, Expect& e)
{
    const u32 mask = c->R[0], fn = c->R[1], sp = c->R[13];
    Fixed(c, s, m);
    Obj st = m.O(sp - 32, 32);
    if (!st || !s.ftab || !s.ftab2) return false;
    const u32 t2 = s.v->irqTable2;
    const Obj tab{s.ftab, kIrqTable, s.ftabD}, tab2{s.ftab2, t2, s.ftab2D};
    u32 lr = c->R[14];
    for (u32 b = mask; b; b &= b - 1)
    {
        const u32 i = __builtin_ctz(b);
        const int o = Ent2(i);
        if (o >= 0) { lr = t2 + o; m.W(tab2, o, fn); m.W(tab2, o + 4, 1); m.W(tab2, o + 8, 0); }
        else { lr = 0; m.W(tab, i * 4, fn); }
    }
    // push {r4-r10, lr}
    for (int i = 0; i < 7; i++) m.W(st, i * 4, c->R[4 + i]);
    m.W(st, 28, c->R[14]);
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.R[0] = 0; e.R[1] = fn; e.R[2] = t2; e.R[3] = 0; e.R[12] = 0x20; e.R[14] = lr;
    e.CPSR = (c->CPSR & 0x0FFFFFFF) | Flags(0x20, 0x20);
    e.retPc = c->R[14];
    if (e.retPc & 1) e.CPSR |= 0x20;
    return true;
}

// ---- 4. OS_GetIrqFunction(mask) ----------------------------------------------------------
bool GetIrq(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, u32& bits)
{
    const u32 mask = c->R[0];
    Fixed(c, s, m);
    if (!s.ftab || !s.ftab2) return false;
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    const u32 i = mask ? __builtin_ctz(mask) : 32;
    bits = i;
    u32 fl;
    if (i == 32) { e.R[0] = 0; e.R[1] = 32; e.R[2] = kIrqTable + 128; fl = Flags(32, 32); }
    else if (i >= 8 && i <= 11) { e.R[1] = i - 8; e.R[2] = e.R[1] * 12; e.R[0] = R32(s.ftab2 + e.R[2]); fl = Flags(i, 11); }
    else if (i >= 3 && i <= 6) { e.R[1] = i + 5; e.R[2] = e.R[1] * 12; e.R[0] = R32(s.ftab2 + e.R[2]); fl = Flags(i, 6); }
    else { e.R[1] = i; e.R[2] = kIrqTable + 4 * i; e.R[0] = R32(s.ftab + 4 * i); fl = i < 3 ? Flags(i, 3) : Flags(i, 6); }
    e.CPSR = (c->CPSR & 0x0FFFFFFF) | fl;
    e.retPc = c->R[14];
    if (e.retPc & 1) e.CPSR |= 0x20;
    return true;
}

// ---- 2./4. TWL SDK build (W2): Thumb OS_SetIrqFunction / OS_GetIrqFunction -------------------
// Set: push {r4-r7}; the same 32-bit loop and table writes as 2.; leaves r0 = 0, r1 = OS_IRQTable,
// r2 = the DMA/timer table, r3 = 32, flags of cmp r3, #32; bx lr.
__attribute__((noinline)) bool SetIrqT(melonDS::ARMv5* c, State& s, Mem& m, Expect& e)
{
    const u32 mask = c->R[0], fn = c->R[1], sp = c->R[13];
    Fixed(c, s, m);
    Obj st = m.O(sp - 16, 16);
    if (!st || !s.ftab || !s.ftab2) return false;
    const u32 t2 = s.v->irqTable2;
    const Obj tab{s.ftab, kIrqTable, s.ftabD}, tab2{s.ftab2, t2, s.ftab2D};
    for (u32 b = mask; b; b &= b - 1)
    {
        const u32 i = __builtin_ctz(b);
        const int o = Ent2(i);
        if (o >= 0) { m.W(tab2, o, fn); m.W(tab2, o + 8, 0); m.W(tab2, o + 4, 1); }
        else m.W(tab, i * 4, fn);
    }
    for (int i = 0; i < 4; i++) m.W(st, i * 4, c->R[4 + i]);
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.R[0] = 0; e.R[1] = kIrqTable; e.R[2] = t2; e.R[3] = 0x20;
    e.CPSR = (c->CPSR & 0x0FFFFFFF) | Flags(0x20, 0x20);
    e.retPc = c->R[14];
    e.CPSR = e.retPc & 1 ? e.CPSR | 0x20 : e.CPSR & ~0x20u;
    return true;
}

// Get: push {r3, r4}; walks the mask bit by bit like 4. (registers and flags of the path taken; r3, r4 popped)
__attribute__((noinline)) bool GetIrqT(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, u32& bits)
{
    const u32 mask = c->R[0], sp = c->R[13];
    Fixed(c, s, m);
    Obj st = m.O(sp - 8, 8);
    if (!st || !s.ftab || !s.ftab2) return false;
    m.W(st, 0, c->R[3]); m.W(st, 4, c->R[4]);
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    const u32 i = mask ? __builtin_ctz(mask) : 32;
    bits = i;
    u32 fl;
    auto NZ = [](u32 r) { return (r & 0x80000000) | ((r == 0) << 30); };
    if (i == 32) { e.R[0] = 0; e.R[1] = 1; e.R[2] = 0; fl = (1u << 30) | (1u << 29); }   // movs r0, #0 after cmp r3, #32
    else
    {
        e.R[2] = mask >> i;
        if (i >= 8 && i <= 11)
        {
            // subs r3, #8 (C=1, V=0); adds r1, r3, #0 (C=0, V=0); muls r1, r0, r1 (NZ)
            e.R[1] = 12 * (i - 8); e.R[0] = R32(s.ftab2 + e.R[1]); fl = NZ(e.R[1]);
        }
        else if (i >= 3 && i <= 6)
        {
            // adds r1, r3, #5; adds r2, r1, #0 (C=0, V=0); muls r2, r0, r2 (NZ)
            e.R[1] = i + 5; e.R[2] = 12 * (i + 5); e.R[0] = R32(s.ftab2 + e.R[2]); fl = NZ(e.R[2]);
        }
        else { e.R[1] = 1; e.R[0] = R32(s.ftab + 4 * i); fl = i < 3 ? Flags(i, 3) : Flags(i, 6); }
    }
    e.CPSR = (c->CPSR & 0x0FFFFFFF) | fl;
    e.retPc = c->R[14];
    e.CPSR = e.retPc & 1 ? e.CPSR | 0x20 : e.CPSR & ~0x20u;
    return true;
}

void Return(melonDS::ARMv5* c, const Expect& e, s32 cycles)
{
    for (int i = 0; i < 15; i++) c->R[i] = e.R[i];
    c->CPSR = e.CPSR;
    c->Cycles += cycles;
    c->JumpTo(e.retPc);
}

// ---- 3. HBlank IRQ ----------------------------------------------------------------------------
// At IRQ delivery (IRQs on, not in IRQ mode). The guest path: BIOS push {r0-r3, r12, lr} on the
// IRQ stack, OS_IrqHandler push {lr}, IF acknowledge of the lowest pending bit (HBlank), table
// call -> Thumb stub -> HBlank callback push {r4-r6, lr}; with an empty callback list it stores
// list head and 0 into its object; back in OS_IrqHandler either the queue is empty (no
// reschedule pending -> return) or the wake loop of 1. runs; BIOS pops and returns.
bool IrqNative(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, bool halted, int& kind, u32& tpc, u32& tcpsr)
{
    melonDS::NDS& nds = c->NDS;
    // (Irq checked that HBlank is the lowest pending enabled IRQ: OS_IrqHandler takes the lowest bit)
    const u32 cpsr = c->CPSR, mode = cpsr & 0x1F;
    if ((cpsr & 0x80) || (mode != 0x1F && mode != 0x10 && mode != 0x13)) return false;
    const bool thumb = cpsr & 0x20;
    // halted in OS_Halt (mcr wait-for-interrupt), called from the idle loop: the guest returns
    // there and halts again with the same registers
    if (halted && (thumb || c->R[15] - 4 != s.v->idlePc || c->R[0] != 0 || c->R[14] != s.v->idleLr)) return false;
    const u32 sp = c->R_IRQ[0];
    if (!Fixed(c, s, m) || !s.fhp || !s.ftb || !s.fop) return false;
    Obj fs = m.O(sp - 44, 44);
    if (!fs) return false;
    __builtin_prefetch(fs.p); __builtin_prefetch(fs.p + 40);
    if (R32(s.fhp) != s.v->code[0].a || R32(s.ftb) != (s.v->code[12].a | 1)) return false;
    Obj o2 = m.O(R32(s.fop) + 0x18, 4);
    if (!o2) return false;
    const u32 obj = o2.r(0);
    Obj ob = m.O(obj, 0x34);
    if (!ob) return false;
    const u32 lrIrq = c->R[15] + (thumb ? 2 : 0);
    const u32 f[6] = {c->R[0], c->R[1], c->R[2], c->R[3], c->R[12], lrIrq};
    const u32 head = R32(s.fq);
    // stack: [sp-44] callback push {r4, r5, r6, lr}, [sp-28] OS_IrqHandler push {lr = BIOS return},
    // [sp-24] BIOS push {r0-r3, r12, lr}; with a wake (head != 0) Wake writes [sp-40, sp-28) and
    // [sp-24, sp) again
    m.W(fs, 0, c->R[4]);
    m.W(fs, 16, s.biosRet);
    if (!head)
    {
        m.W(fs, 4, c->R[5]);
        m.W(fs, 8, c->R[6]);
        m.W(fs, 12, s.v->code[0].a + 0x58);
        for (int i = 0; i < 6; i++) m.W(fs, 20 + i * 4, f[i]);
    }
    if (ob.r(0x2C) == 0)
    {
        const u32 l = ob.r(0x10);
        if (l != obj + 8) return false;                 // HBlank callbacks queued: guest
        m.W(ob, 0x30, l);
        m.W(ob, 0x1C, 0);
    }
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.CPSR = cpsr;
    e.retPc = (lrIrq - 4) | (thumb ? 1 : 0);
    e.IRQ[0] = sp; e.IRQ[1] = lrIrq; e.IRQ[2] = cpsr;
    e.SVC[0] = c->R_SVC[0]; e.SVC[1] = c->R_SVC[1]; e.SVC[2] = c->R_SVC[2];
    e.banks = true;
    e.cur = 0;
    if (!head)
    {
        if (R16(s.fosi + (s.v->info - s.v->osi))) return false; // reschedule pending: thread switch
        kind = 3;
        return true;
    }
    IrqIn in{cpsr, sp - 28, c->R_SVC[0], c->R[13], c->R[14], head, true, s.biosRet, {f[0], f[1], f[2], f[3], f[4], f[5]}};
    if (!Wake(c, s, m, e, in, tpc, tcpsr)) return false;
    kind = 4;
    return true;
}

// ---- 11. the display list's DMA-end IRQ ---------------------------------------------------------
// MI_SendGXCommandAsync's last chunk is a DMA with an IRQ (10. starts it); its end IRQ runs BIOS ->
// OS_IrqHandler (IF acknowledge) -> OSi_IrqDma<n> stub -> OSi_IrqCallback(n): clears the DMA IRQ table entry,
// calls MIi_DMACallback (OS_DisableIrqMask(GXFIFO), GXSTAT IRQ mode restored, OS_SetIrqFunction(GXFIFO,
// the saved handler), MIi_GXDmaParams.busy = 0, the list's callback: NNS G3D's "[arg] = 0"), sets the IRQ
// check flag, OS_DisableIrqMask(the DMA bit) -> empty thread queue, no reschedule: return. ~120 guest
// instructions, an IRQ entry and the OS_SetIrqFunction hook exit, 55 a frame in the PW town. Natively at
// delivery (like 3.) when that DMA bit is the lowest pending enabled IRQ, the entry is MIi_DMACallback, the
// list callback is the 3-word clear (or none) and the thread queue is empty: the same IO writes in the same
// order (IF, IE, GXSTAT, IE; IME toggles left out: IRQs are off and IME ends unchanged), the IRQ stack bytes,
// the tables, flags and banked IRQ registers the guest leaves. Category B: a fixed cycle estimate.
// ponytail: fixed estimate (guest average in check mode, IRQ entry to the BIOS return)
constexpr s32 kDmaIrqCycles = 870, kDmaIrqCyclesT = 785;     // PW/PB; W2 (Thumb build: 783-788 in check mode)
constexpr u32 kClearCb[3] = {0xE3A01000, 0xE5801000, 0xE12FFF1E};    // mov r1, #0; str r1, [r0]; bx lr
constexpr u16 kClearCbT[3] = {0x2100, 0x6001, 0x4770};                 // Thumb (W2): movs r1, #0; str r1, [r0]; bx lr

struct IoPlan { u32 n = 0, a[4], v[4]; void Add(u32 x, u32 y) { a[n] = x; v[n++] = y; } };
// (out of line: keeps the HBlank path of IrqOne compact)
__attribute__((noinline)) bool DmaIrqNative(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, bool halted, u32 bit, IoPlan& io)
{
    melonDS::NDS& nds = c->NDS;
    const Variant& v = *s.v;
    const u32 cpsr = c->CPSR, mode = cpsr & 0x1F, n = bit - 8;
    if ((cpsr & 0x80) || (mode != 0x1F && mode != 0x10 && mode != 0x13)) return false;
    const bool thumb = cpsr & 0x20;
    if (halted && (thumb || c->R[15] - 4 != v.idlePc || c->R[0] != 0 || c->R[14] != v.idleLr)) return false;
    if (!Fixed(c, s, m) || !s.fhp || !s.ftab || !s.ftab2) return false;
    const u32 sp = c->R_IRQ[0];
    Obj fs = m.O(sp - 92, 92);
    if (!fs) return false;
    __builtin_prefetch(fs.p); __builtin_prefetch(fs.p + 64);
    const u32 cbFn = v.dmaCode[0].a, mdc = v.dmaCode[2].a;
    // the handler chain: BIOS -> OS_IrqHandler -> OSi_IrqDma<n> -> MIi_DMACallback; an empty thread queue, no reschedule
    const u32 stub = v.twl ? (cbFn + 0x58 + n * 12) | 1 : cbFn + 0x88 + n * 16;
    if (R32(s.fhp) != v.code[0].a || R32(s.ftab + bit * 4) != stub || R32(s.fq)
        || R16(s.fosi + (v.info - v.osi)) || !(nds.IME[0] & 1)) return false;
    const Obj tab{s.ftab, kIrqTable, s.ftabD}, tab2{s.ftab2, v.irqTable2, s.ftab2D};
    if (tab2.r(n * 12) != (mdc | v.twl)) return false;     // (W2: a Thumb address)
    // the IRQ bit of entry n (OSi_IrqCallback's u16 table)
    const u8* bt = m.P(R32(CodePtr(c, cbFn + (v.twl ? 0x40 : 0x78))) + n * 2);
    if (!bt || R16(bt) != bit) return false;
    Obj P = m.O(v.gxParams, 0x20);
    if (!P || P.dtcm) return false;
    const u32 ucb = P.r(0x10), arg = P.r(0x14);
    Obj A;
    if (ucb)
    {
        const u8* q = CodePtr(c, ucb & ~1u);
        if (!q || ((ucb & 1) ? (ucb & 2) || memcmp(q, kClearCbT, 6) : (ucb & 3) || memcmp(q, kClearCb, 12))) return false;
        A = m.O(arg, 4);
        if (!A) return false;
    }
    const u32 lrIrq = c->R[15] + (thumb ? 2 : 0);
    const u32 ie0 = nds.IE[0], ie1 = ie0 & ~0x200000u, en = tab2.r(n * 12 + 4), ie2 = en ? ie1 : ie1 & ~(1u << bit);
    // IRQ stack (fs = sp - 92): OS_SetIrqFunction push {r4-r10, lr} | MIi_DMACallback push {r3-r5, lr} |
    // OSi_IrqCallback push {r3-r5, lr} | OS_IrqHandler push {lr} | BIOS push {r0-r3, r12, lr}
    if (v.twl)
    {
        // TWL SDK build (Thumb), fs + 16 = sp - 76, in write order: OSi_IrqCallback push {r3-r5, lr} (sp-44), MIi_DMACallback
        // push {r3-r5, lr} (sp-60), OS_DisableIrqMask push {r3, r4} (sp-68), OS_SetIrqFunction push {r4-r7} (sp-76),
        // OS_DisableIrqMask(DMA bit) push {r3, r4} (sp-52, when the entry's enable flag is clear)
        // (r3 = the stub's ldr r3, =OSi_IrqCallback | 1 in every push)
        const u32 d = 1u << bit, r3 = cbFn | 1;
        struct { u32 o, v; } w[] = {
            {64, s.biosRet}, {68, c->R[0]}, {72, c->R[1]}, {76, c->R[2]}, {80, c->R[3]}, {84, c->R[12]}, {88, lrIrq},
            {48, r3}, {52, c->R[4]}, {56, c->R[5]}, {60, v.code[0].a + 0x58},
            {32, r3}, {36, d}, {40, n * 12}, {44, cbFn + 0x25},
            {24, r3}, {28, d},
            {16, v.gxParams}, {20, 0x200000}, {24, c->R[6]}, {28, c->R[7]},
        };
        for (auto& x : w) m.W(fs, x.o, x.v);
        if (!tab2.r(n * 12 + 4)) { m.W(fs, 40, r3); m.W(fs, 44, d); }
    }
    else
    {
        const u32 w[23] = {v.gxParams, 0x200000, c->R[6], c->R[7], c->R[8], c->R[9], c->R[10], mdc + 0x38,
                           bit, 1u << bit, n * 12, cbFn + 0x44,
                           0x80000000, c->R[4], c->R[5], v.code[0].a + 0x58,
                           s.biosRet, c->R[0], c->R[1], c->R[2], c->R[3], c->R[12], lrIrq};
        for (int i = 0; i < 23; i++) m.W(fs, i * 4, w[i]);
    }
    m.W(tab2, n * 12, 0);
    m.W(tab, 21 * 4, P.r(0x1C));
    m.W(P, 0, 0);
    if (ucb) m.W(A, 0, 0);
    Obj chk{(u8*)s.fchk, kIrqCheck, true};
    m.W(chk, 0, R32(s.fchk) | 1u << bit);
    const u32 gxstat = nds.ARM9Read32(0x04000600), gxw = (gxstat & ~0xC0000000u) | (P.r(0x18) << 30);
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.CPSR = cpsr;
    e.retPc = (lrIrq - 4) | (thumb ? 1 : 0);
    e.IRQ[0] = sp; e.IRQ[1] = lrIrq; e.IRQ[2] = cpsr;
    e.SVC[0] = c->R_SVC[0]; e.SVC[1] = c->R_SVC[1]; e.SVC[2] = c->R_SVC[2];
    e.banks = true;
    e.cur = 0;
    io.Add(0x04000214, 1u << bit); io.Add(0x04000210, ie1); io.Add(0x04000600, gxw);
    if (!en) io.Add(0x04000210, ie2);
    return true;
}

#ifdef LITEV_GX_BULK
// ---- 5. MI_SendGXCommandAsync: the display list at once ----------------------------------------
// MIi_FIFOCallback sends the list in 118-word immediate DMAs to GXFIFO, the next one from the
// GXFIFO "less than half full" IRQ. Under LITEV_GX_BULK each DMA finds the FIFO empty and runs
// in bulk, so the FIFO is empty again at once and the IRQ that follows only starts the next DMA:
// ~2 IRQs per list, 100-200 a frame in PW's 3D scenes. At MIi_FIFOCallback's entry, with the
// FIFO empty: every chunk but the last goes through GPU3D::BulkWords now (what those DMAs do)
// and MIi_GXDmaParams.src/length move past them; the guest code then sends the last chunk
// (DMA with IRQ -> MIi_DMACallback) as before. Category B: the elided IRQs take a fixed estimate
// (the DMA cycles themselves are the DMA's own burst timing).
// ponytail: fixed estimate per elided GXFIFO IRQ (BIOS, OS_IrqHandler, MIi_FIFOCallback, DMA
// setup): check mode measures ~850 guest cycles per chunk on 7 PW segments, ~488 of them the DMA
constexpr s32 kGxIrqCycles = 360;

// the words the guest would send before its last chunk (n), from src; false: guest path
bool GxPlan(melonDS::ARMv5* c, const State& s, u32& src, u32& len, u32& n)
{
    melonDS::NDS& nds = c->NDS;
    const u32 kGxParams = s.v->gxParams;
    if ((kGxParams & c->DTCMMask) == c->DTCMBase || ((kGxParams + 0xF) & c->DTCMMask) == c->DTCMBase) return false;
    const u8* p = &nds.MainRAM[kGxParams & nds.MainRAMMask];
    len = R32(p + 0xC); src = R32(p + 8);
    const u32 dma = R32(p + 4);
    // DMA reads the bus (main RAM, never TCM), like the guest's
    if (len <= kGxChunk || len > 0x100000 || (len & 3) || (src & 3) || dma > 3 || (src >> 24) != 0x02 || ((src + len - 1) >> 24) != 0x02)
        return false;
    if (nds.A9HLEDmaCnt(dma) & 0x80000000) return false;
    melonDS::GPU3D& gx = nds.GPU.GPU3D;
    if (!gx.GeometryEnabled || !gx.BulkReady()) return false;
    n = ((len - 1) / kGxChunk) * (kGxChunk / 4);
    return true;
}

// sends them (stops early if the FIFO stops being empty: a SWAP_BUFFERS); returns the count
u32 GxSend(melonDS::ARMv5* c, const State& s, u32 src, u32 len, u32 n)
{
    const u32 kGxParams = s.v->gxParams;
    melonDS::NDS& nds = c->NDS;
    melonDS::GPU3D& gx = nds.GPU.GPU3D;
    const u8* ram = nds.MainRAM;
    const u32 mask = nds.MainRAMMask, per = kGxChunk / 4;
    // the bulk DMA loop's timing (DMA::Run9): burst table restarted at each DMA
    const u8* bt = (nds.ARM9MemTimings[0x1000][6] == 2) ? DMATiming::MRAMRead32Bursts[0].data() : DMATiming::MRAMRead32Bursts[1].data();
    u32 done = 0, cyc = 0, bc = 0;
    for (u32 o = 0; o < 512; o += 64) __builtin_prefetch(ram + ((src + o) & mask));
#ifdef LITEV_GX_SEND_DIRECT
    // The list read in place (no copy) when it doesn't wrap the RAM mirror, and the DMA cycles
    // from a per-chunk prefix sum of the same burst table (the count restarts at every chunk).
    if ((src & mask) + n * 4 <= mask + 1)
    {
        const u32* w = (const u32*)(ram + (src & mask));
        static u32 cum[2][kGxChunk / 4 + 1];
        static bool cumOk[2];
        const int t = bt == DMATiming::MRAMRead32Bursts[0].data() ? 0 : 1;
        if (!cumOk[t])
        {
            for (u32 i = 0, b = 0, c2 = 0; i < per; i++) { if (bt[b] == 0) b = 0; c2 += bt[b++]; cum[t][i + 1] = c2; }
            cumOk[t] = true;
        }
        while (done < n && gx.BulkReady())
        {
            const u32 m = n - done < 64 ? n - done : 64;
            const char* pf = (const char*)(w + done) + 512;
            if (pf + 256 <= (const char*)(ram + mask + 1))
                for (u32 o = 0; o < 256; o += 64) __builtin_prefetch(pf + o);
            gx.BulkWords(w + done, m);
            done += m;
        }
        cyc = (done / per) * cum[t][per] + cum[t][done % per];
    }
    else
#endif
    while (done < n && gx.BulkReady())
    {
        u32 w[64];
        const u32 m = n - done < 64 ? n - done : 64;
        for (u32 i = 0; i < m; i++)
        {
            if ((done + i) % per == 0 || bt[bc] == 0) bc = 0;
            cyc += bt[bc++];
            w[i] = R32(ram + ((src + (done + i) * 4) & mask));
        }
        __builtin_prefetch(ram + ((src + (done + m) * 4 + 512) & mask));
        gx.BulkWords(w, m);
        done += m;
    }
    nds.ARM9Write32(kGxParams + 8, src + done * 4);
    nds.ARM9Write32(kGxParams + 0xC, len - done * 4);
    c->Cycles += (s32)(cyc << nds.ARM9ClockShift) + kGxIrqCycles * (s32)(done / per);
    return done;
}
#endif

#ifdef LITEV_A9HLE_GXCHECK
// check mode for 5. (interpreter, env LITEV_A9HLE_CHECK): the guest sends the list; every word
// reaching GXFIFO (and any other geometry command write) is recorded; when the guest's
// MIi_FIFOCallback reaches the point the native path would leave, the words, src and length
// must be what the native path computed.
std::vector<u32> g_GxTapBuf;
struct GxPending { bool on = false; u32 src = 0, len = 0, n = 0; u64 t0 = 0; std::vector<u32> w; } g_Gx;
u64 g_GxDmaCyc = 0;     // the DMA part of the guest cycles (same burst timing as GxSend)
void GxCheckAt(State& s, melonDS::ARMv5* c)
{
    melonDS::NDS& nds = c->NDS;
    if (!g_Gx.on) return;
    const u8* p = &nds.MainRAM[s.v->gxParams & nds.MainRAMMask];
    const u32 src = R32(p + 8), len = R32(p + 0xC), want = g_Gx.len - g_Gx.n * 4;
    if (len > want) return;     // guest not there yet
    g_Gx.on = false;
    GxTap = nullptr;
    bool bad = len != want || src != g_Gx.src + g_Gx.n * 4 || g_GxTapBuf != g_Gx.w;
    if (GxOtherSeen) bad = true;
    if (bad)
    {
        if (s.diffs[5] < 10)
            fprintf(stderr, "A9HLE CHECK gxsend: src %08x len %x (native %08x %x), words guest %zu native %zu%s\n", src, len,
                    g_Gx.src + g_Gx.n * 4, want, g_GxTapBuf.size(), g_Gx.w.size(), GxOtherSeen ? ", other GX writes" : "");
        s.diffs[5]++;
    }
    GxOtherSeen = false;
    // resolved checks; guest cycles per elided chunk (for kGxIrqCycles: minus the DMA's own)
    s.guestN[5] += g_Gx.n / (kGxChunk / 4);
    s.guestCyc[5] += nds.ARM9Timestamp + c->Cycles - g_Gx.t0;
    const u8* bt = (nds.ARM9MemTimings[0x1000][6] == 2) ? DMATiming::MRAMRead32Bursts[0].data() : DMATiming::MRAMRead32Bursts[1].data();
    for (u32 i = 0, bc = 0; i < g_Gx.n; i++) { if (i % (kGxChunk / 4) == 0 || bt[bc] == 0) bc = 0; g_GxDmaCyc += (u64)bt[bc++] << nds.ARM9ClockShift; }
    s.irqDuring[5]++;
}
#endif

// ---- 6. MIi_UncompressBackward: backward LZ (overlay / data unpacking) ------------------------
// NitroSDK's in-place backward LZ decompressor (ARM, position independent; PW/PB/W2 have it at
// 0x02004DF4): 18% of the ARM9's guest instructions in PW loading frames. Hooked at its loop head
// (`cmp r3, r1`); the native loop is the guest loop instruction by instruction (same byte reads
// and writes in the same order, so overlapping in-place data behaves the same) and stops at a
// token boundary, leaving exactly the guest's registers and flags for that point: when done (the
// cache clean/invalidate tail at +0x8C then runs as guest code), after kLzBudget output bytes (the
// guest loop head, where the hook runs again: IRQs and events run in between as they would between
// JIT blocks), or before any token that would leave the checked main-RAM window (the guest
// continues). Category B: a fixed cycle estimate per token / byte instead of the loop's own count.
constexpr u32 kLzInstr = 0xE1530001;        // cmp r3, r1 (the loop head)
constexpr u32 kLzHead = 0x24;               // loop head - function entry
constexpr u32 kLzCode[45] = {
    0xE3500000, 0x0A000029, 0xE92D01F0, 0xE9100006, 0xE0802002, 0xE0403C21, 0xE3C114FF, 0xE0401001, 0xE1A04002,
    0xE1530001, 0xDA000017, 0xE5735001, 0xE3A06008, 0xE2566001, 0xBAFFFFF9, 0xE3150080, 0x1A000003, 0xE5730001,
    0xE5528001, 0xE5620001, 0xEA00000A, 0xE573C001, 0xE5737001, 0xE187740C, 0xE3C77A0F, 0xE2877002, 0xE28CC020,
    0xE7D20007, 0xE5528001, 0xE5620001, 0xE25CC010, 0xAAFFFFFA, 0xE1530001, 0xE1A05085, 0xCAFFFFE9, 0xE3A00000,
    0xE3C1301F, 0xEE070F9A, 0xEE073F35, 0xEE073F3E, 0xE2833020, 0xE1530004, 0xBAFFFFF9, 0xE8BD01F0, 0xE12FFF1E};
// output bytes per native call (~5.5k ARM9 cycles: an IRQ waits at most ~1.3 scanlines longer)
constexpr u32 kLzBudget = 256;
// ponytail: fixed cycle estimate, least-squares fit of the guest's own cycles (check mode, cycles of
// the function's instructions only) over 779 chunks of f12000/f15780/f1300: per flag byte, literal,
// back-reference, copied byte; total within ~1%, per chunk within ~2%
constexpr s32 kLzCycFlag = 9, kLzCycLit = 25, kLzCycRef = 24, kLzCycByte = 14;

bool LzCodeAt(melonDS::ARMv5* c, u32 head)
{
    const u32 a = head - kLzHead;
    if (a & 3) return false;
    for (u32 i = 0; i < 45; i++)
    {
        const u8* p = CodePtr(c, a + i * 4);
        if (!p || R32(p) != kLzCode[i]) return false;
    }
    return true;
}

// guest state at the hook (pc = loop head) or a handoff point
struct Lz { u32 r[13]; u32 pc; u32 cpsr; s32 cyc; u32 out; u32 n[4]; };   // n: flag bytes, literals, back-references, copied bytes

// Runs the loop on the window [lo, hi) at host b (b[a - lo] = guest byte a), from the loop head.
// Returns false if nothing could be done natively (the guest runs the loop head).
bool LzRun(Lz& z, u8* b, u32 lo, u32 hi)
{
    u32 r0 = z.r[0], r1 = z.r[1], r2 = z.r[2], r3 = z.r[3], r5 = z.r[5], r6 = z.r[6], r7 = z.r[7], r8 = z.r[8], ip = z.r[12];
    const u32 head = z.pc, done = head + 0x68, inner = head + 0x10;
    u32 out = 0, nf = 0, nl = 0, nr = 0;
    auto M = [&](u32 a) -> u32 { return b[a - lo]; };
    u32 pc, fl;     // handoff point and NZCV there (fl: of the last flag-setting guest instruction)
    for (;;)
    {
        // loop head (+0): cmp r3, r1; ble done; ldrb r5, [r3, #-1]!; mov r6, #8
        fl = Flags(r3, r1);
        if ((s32)r3 <= (s32)r1) { pc = done; break; }
        r3--; r5 = M(r3); r6 = 8; nf++;
        bool head2 = false;
        for (;;)
        {
            // inner loop head (+0x10): the next token stays in the window and the budget, else hand over here
            if (r6 != 0)
            {
                bool ok;
                if (r5 & 0x80)
                {
                    ok = r3 - 2 >= lo;
                    if (ok)
                    {
                        const u32 b1 = M(r3 - 1), d = ((M(r3 - 2) | b1 << 8) & ~0xF000u) + 2, n = (b1 >> 4) + 3;
                        ok = r2 - n >= lo && r2 + d < hi;
                    }
                }
                else ok = r3 - 1 >= lo && r2 - 1 >= lo;
                if (!ok || out >= kLzBudget) { pc = inner; goto end; }
            }
            fl = Flags(r6, 1);      // subs r6, r6, #1; blt head
            r6--;
            if ((s32)r6 < 0) { head2 = true; break; }
            if (r5 & 0x80)
            {
                ip = M(--r3); r7 = M(--r3);
                r7 |= ip << 8; r7 &= ~0xF000u; r7 += 2; ip += 0x20;
                nr++;
                do { r0 = M(r2 + r7); r8 = M(r2 - 1); b[--r2 - lo] = (u8)r0; ip -= 0x10; out++; } while ((s32)ip >= 0);
            }
            else { r0 = M(--r3); r8 = M(r2 - 1); b[--r2 - lo] = (u8)r0; out++; nl++; }
            fl = Flags(r3, r1);     // cmp r3, r1; lsl r5, r5, #1; bgt inner
            r5 <<= 1;
            if (!((s32)r3 > (s32)r1)) { pc = done; goto end; }
        }
        if (head2 && out >= kLzBudget && (s32)r3 > (s32)r1) { pc = head; break; }
    }
end:
    if (!out && pc != done) return false;
    z.r[0] = r0; z.r[2] = r2; z.r[3] = r3; z.r[5] = r5; z.r[6] = r6; z.r[7] = r7; z.r[8] = r8; z.r[12] = ip;
    z.pc = pc; z.cpsr = (z.cpsr & 0x0FFFFFFF) | fl; z.out = out;
    z.n[0] = nf; z.n[1] = nl; z.n[2] = nr; z.n[3] = out - nl;
    z.cyc = kLzCycFlag * (s32)nf + kLzCycLit * (s32)nl + kLzCycRef * (s32)nr + kLzCycByte * (s32)(out - nl);
    return true;
}

// [lo, hi) is one main-RAM block in host memory (no TCM, no mirror wrap), else nullptr
u8* LzWindow(melonDS::ARMv5* c, u32 lo, u32 hi)
{
    melonDS::NDS& nds = c->NDS;
    if (hi <= lo || lo < c->ITCMSize || (lo >> 24) != 0x02 || ((hi - 1) >> 24) != 0x02) return nullptr;
    if ((lo & nds.MainRAMMask) > ((hi - 1) & nds.MainRAMMask)) return nullptr;
    const u32 dsz = ~c->DTCMMask + 1;   // DTCM window [DTCMBase, DTCMBase + dsz)
    if (c->DTCMBase < hi && lo < c->DTCMBase + dsz) return nullptr;
    return &nds.MainRAM[lo & nds.MainRAMMask];
}

__attribute__((noinline)) bool RunLz(melonDS::ARMv5* c, State& s, bool jit)
{
    if (!jit && !LzCodeAt(c, c->R[15] - 8)) return false;
    s.calls[6]++;
    Lz z;
    for (int i = 0; i < 13; i++) z.r[i] = c->R[i];
    z.pc = c->R[15] - 8; z.cpsr = c->CPSR;
    const u32 z0pc = z.pc;
    (void)z0pc;
    const u32 lo = z.r[1] - 2, hi = z.r[4];    // r1: source start, r4: destination end
    u8* w = CheckPending ? nullptr : LzWindow(c, lo, hi);
    bool ok = w != nullptr;
#ifdef LITEV_HLE_DIAG
    if (ok && (g_Check || g_Dry))
    {
        // on a copy; check: compare with the guest at the handoff point
        std::vector<u8> cp(w, w + (hi - lo));
        ok = LzRun(z, cp.data(), lo, hi);
        if (ok && g_Check)
        {
            Mem m(c, true);
            Expect e;
            for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
            for (int i = 0; i < 13; i++) e.R[i] = z.r[i];
            e.CPSR = z.cpsr; e.retPc = z.pc;
            ArmCheck(c, 6, m, e);
            for (u32 i = 0; i < hi - lo; i++) if (cp[i] != w[i]) g_P.log.push_back({lo + i, cp[i], 1});
            s.lzBytes += z.out;
            s.lzEst += (u64)z.cyc;
            for (int i = 0; i < 4; i++) g_P.lzn[i] = z.n[i];
            g_P.lzLo = lo; g_P.lzHi = hi;
            g_P.lzFn = z0pc - kLzHead; g_P.lzOwn = 0; g_P.lzLastIn = false;
        }
        if (ok) s.checks[6]++;
        ok = false;
    }
    else
#endif
    if (ok) ok = LzRun(z, w, lo, hi);
    if (!ok)
    {
        s.fallback[6] += !g_Check;
        GuestFallback(c);
        return true;
    }
    // written: [z.r[2], old r2): JIT invalidation per 16-byte granule (what the guest's strb do)
    melonDS::NDS& nds = c->NDS;
    for (u32 a = z.r[2] & ~15u; a < c->R[2]; a += 16)
        nds.JIT.CheckAndInvalidate<0, melonDS::ARMJIT_Memory::memregion_MainRAM>(a);
    s.native[6]++;
    s.lzBytes += z.out;
    for (int i = 0; i < 13; i++) c->R[i] = z.r[i];
    c->CPSR = z.cpsr;
    c->Cycles += z.cyc;
    c->JumpTo(z.pc);
    return true;
}

// ---- 7. CARD ROM read loop: the page natively -------------------------------------------------
// NitroSDK CARD's CPU read (PW/PB: in 0x02076e64; position independent, 9 exact words): poll
// ROMCTRL, read ROMDATA while DATA_READY, store the first 128 words at r5 + r3*4 (r3 counts), loop
// while BUSY. PW streams map/graphics data with it every frame (~600-800 polls/frame in loading and
// title; 13% of the ARM9's guest instructions in f12000 after hook 6), each poll and read an IO
// access through the bus handlers. Native, at the loop head: the reads through NDSCartSlot::HleRead9
// (the CART_SYNC refill inlined, the cart's plain ROM read; one call per page instead of two IO
// dispatches per word), the stores as one copy into main RAM (JIT invalidation per 16-byte granule:
// overlays are read straight to their destination), registers and flags as after the guest's last
// data iteration, handed back at the loop head (the guest's last poll sees BUSY clear and leaves).
// Category B: a fixed cycle estimate per word; an IRQ that would arrive inside the page is taken
// after it (one page at most per call, ~4k cycles).
constexpr u32 kCardInstr = 0xE5942000;      // ldr r2, [r4] (the loop head)
constexpr u32 kCardLoop[9] = {0xE5942000, 0xE3120502, 0x0A000003, 0xE59B1000, 0xE3530080, 0x37851103, 0x32833001, 0xE3120102, 0x1AFFFFF6};
// ponytail: fixed estimate per word (guest average in check mode without IRQs, PW f12000/f1300)
constexpr s32 kCardCycWord = 30;

bool CardCodeAt(melonDS::ARMv5* c, u32 a)
{
    if (a & 3) return false;
    for (u32 i = 0; i < 9; i++)
    {
        const u8* p = CodePtr(c, a + i * 4);
        if (!p || R32(p) != kCardLoop[i]) return false;
    }
    return true;
}

// the registers the loop uses, and the stores of words r3.. (< 128) go to one main-RAM host block
// the guest may write (protection unit), not under a TCM; false: the guest runs the loop
bool CardArgsOk(melonDS::ARMv5* c)
{
    melonDS::NDS& nds = c->NDS;
    if (c->R[4] != 0x040001A4 || c->R[11] != 0x04100010) return false;
    // IO reads pass the protection unit (else the guest's data abort)
    if (!(c->PU_Map[0x040001A4 >> 12] & 0x01) || !(c->PU_Map[0x04100010 >> 12] & 0x01)) return false;
    const u32 r3 = c->R[3];
    if (r3 >= 128) return true;
    // (a DTCM of >= 16 KB overlapping a range of <= 512 bytes holds one of its ends)
    const u32 r5 = c->R[5], a = r5 + r3 * 4, b = r5 + 511;
    return !((r5 & 3) || (a >> 24) != 0x02 || (b >> 24) != 0x02 || a < c->ITCMSize || ((a & c->DTCMMask) == c->DTCMBase)
             || ((b & c->DTCMMask) == c->DTCMBase) || ((a & nds.MainRAMMask) > (b & nds.MainRAMMask))
             || !(c->PU_Map[a >> 12] & 0x02) || !(c->PU_Map[b >> 12] & 0x02));
}

// n words read: stores, registers, flags (NZCV of the last tst r2, #BUSY with BUSY set; V of cmp r3, #128)
void CardCommit(melonDS::ARMv5* c, const u32* w, u32 n, u32 lastCnt)
{
    melonDS::NDS& nds = c->NDS;
    u32 r3 = c->R[3];
    if (r3 < 128)
    {
        const u32 k = n < 128 - r3 ? n : 128 - r3, a = c->R[5] + r3 * 4;
        memcpy(&nds.MainRAM[a & nds.MainRAMMask], w, k * 4);
        for (u32 g = a & ~15u; g < a + k * 4; g += 16)
            nds.JIT.CheckAndInvalidate<0, melonDS::ARMJIT_Memory::memregion_MainRAM>(g);
        r3 += k;
    }
    c->R[1] = w[n - 1]; c->R[2] = lastCnt; c->R[3] = r3;
    c->CPSR = (c->CPSR & 0x0FFFFFFF) | 0xA0000000;
}

#ifdef LITEV_HLE_DIAG
// check mode (interpreter): the guest runs the loop; at its exit the native path reruns from the
// entry state (the cart's ROM-read state restored), plus the guest's final poll, and must leave
// the same registers, flags, stores, cart state and transfer IRQ
struct CardPending
{
    bool on = false;
    u32 r[16] = {}, cpsr = 0, ifr = 0, exit = 0;
    std::vector<u8> cart, buf;
    u64 t0 = 0, steps = 0;
    bool irq = false;
} g_Card;

std::vector<u8> CartState(melonDS::NDS& nds)
{
    melonDS::Savestate st(1 << 16);
    nds.NDSCartSlot.HleRomState(&st);
    st.Finish();
    const u8* b = (const u8*)st.Buffer();
    return std::vector<u8>(b, b + st.Length());
}
void CartLoad(melonDS::NDS& nds, std::vector<u8>& v)
{
    melonDS::Savestate st(v.data(), (u32)v.size(), false);
    nds.NDSCartSlot.HleRomState(&st);
}

__attribute__((noinline, cold)) void CardCheckStart(melonDS::ARMv5* c, State& s, u32 pc)
{
    melonDS::NDS& nds = c->NDS;
    g_Card.on = true; g_Card.exit = pc + 0x24;
    for (int i = 0; i < 16; i++) g_Card.r[i] = c->R[i];
    g_Card.cpsr = c->CPSR; g_Card.ifr = nds.IF[0];
    g_Card.cart = CartState(nds);
    g_Card.buf.assign(512, 0);
    for (u32 i = 0; i < 512; i++) g_Card.buf[i] = nds.MainRAM[(c->R[5] + i) & nds.MainRAMMask];
    g_Card.t0 = nds.ARM9Timestamp + c->Cycles; g_Card.steps = 0; g_Card.irq = false;
    CheckPending = true;
    s.checks[7]++;
}

__attribute__((noinline, cold)) void CardCheckAt(melonDS::ARMv5* c, u32 pc)
{
    melonDS::NDS& nds = c->NDS;
    State& s = g_State[&nds];
    if (++g_Card.steps > 2000000) { fprintf(stderr, "A9HLE CHECK cardread: guest never left the loop\n"); g_Card.on = CheckPending = false; s.diffs[7]++; return; }
    if (pc == c->ExceptionBase + 0x18 && !g_Card.irq) { s.irqDuring[7]++; g_Card.irq = true; }
    if (pc != g_Card.exit || (c->CPSR & 0x3F) != (g_Card.cpsr & 0x3F)) return;
    g_Card.on = CheckPending = false;
    const u64 gcyc = nds.ARM9Timestamp + c->Cycles - g_Card.t0;
    const u32 g1 = c->R[1], g2 = c->R[2], g3 = c->R[3], gfl = c->CPSR & 0xF0000000, gif = nds.IF[0];
    std::vector<u8> gbuf(512), gcart = CartState(nds);
    for (u32 i = 0; i < 512; i++) gbuf[i] = nds.MainRAM[(g_Card.r[5] + i) & nds.MainRAMMask];
    // native from the entry state, on copies of the registers and the buffer
    CartLoad(nds, g_Card.cart);
    nds.IF[0] = g_Card.ifr;
    u32 w[4096], cnt = 0, n = nds.NDSCartSlot.HleRead9(w, 4096, cnt);
    u32 n1 = g_Card.r[1], n2 = g_Card.r[2], n3 = g_Card.r[3], nfl = g_Card.cpsr & 0xF0000000;
    std::vector<u8> nbuf = g_Card.buf;
    if (n)
    {
        for (u32 i = 0; i < n && n3 < 128; i++, n3++) memcpy(&nbuf[n3 * 4], &w[i], 4);
        n1 = w[n - 1]; n2 = cnt; nfl = 0xA0000000;
    }
    // the guest's final poll at the loop head: no data, BUSY clear -> exit (tst r2, #BUSY: Z, C from the immediate)
    const u32 fin = nds.NDSCartSlot.ReadROMCnt(0);
    bool ok = n && !(fin & 0x80800000);
    if (ok) { n2 = fin; nfl = 0x60000000 | (nfl & 0x10000000); }
    const u32 nif = nds.IF[0];
    std::vector<u8> ncart = CartState(nds);
    nds.IF[0] = gif;
    nds.UpdateIRQ(0);
    CartLoad(nds, gcart);
    char b[512]; int bl = 0, nd = 0;
    auto d = [&](const char* what, u32 g, u32 v) { if (g != v) { if (nd < 8) bl += snprintf(b + bl, sizeof(b) - bl, " %s guest %08x native %08x;", what, g, v); nd++; } };
    if (!ok) { bl += snprintf(b + bl, sizeof(b) - bl, " native %s;", n ? "did not reach the end" : "did nothing"); nd++; }
    else
    {
        d("r1", g1, n1); d("r2", g2, n2); d("r3", g3, n3); d("nzcv", gfl, nfl);
        d("xfer irq", (gif & ~g_Card.ifr) & (1u << 19), (nif & ~g_Card.ifr) & (1u << 19));
        for (u32 i = 0; i < 512; i++) if (gbuf[i] != nbuf[i]) { d("buf", gbuf[i], nbuf[i]); break; }
        if (gcart != ncart) { bl += snprintf(b + bl, sizeof(b) - bl, " cart state;"); nd++; }
    }
    if (nd)
    {
        if (s.diffs[7] < 10) fprintf(stderr, "A9HLE CHECK cardread %d diffs:%s\n", nd, b);
        s.diffs[7]++;
    }
    if (ok && !g_Card.irq) { s.guestCyc[7] += gcyc; s.guestN[7] += n; }   // cycles per word without IRQs
}
#endif

__attribute__((noinline)) bool RunCard(melonDS::ARMv5* c, State& s, bool jit)
{
    const u32 pc = c->R[15] - 8;
    if (!jit && !CardCodeAt(c, pc)) return false;
    s.calls[7]++;
    bool ok = !CheckPending && CardArgsOk(c);
#ifdef LITEV_HLE_DIAG
    // (from the first poll that finds data: before that the guest only polls)
    if (ok && g_Check) { if (c->NDS.NDSCartSlot.ReadROMCnt(0) & 0x800000) CardCheckStart(c, s, pc); ok = false; }
#endif
    u32 w[128], cnt = 0, n = 0;
    if (ok) n = c->NDS.NDSCartSlot.HleRead9(w, 128, cnt);
    if (!n)
    {
        s.fallback[7] += !g_Check;
        GuestFallback(c);
        return true;
    }
    CardCommit(c, w, n, cnt);
    s.native[7]++;
    s.cardWords += n;
    c->Cycles += kCardCycWord * (s32)n;
    c->JumpTo(pc);
    return true;
}

#ifdef LITEV_GX_BULK
// ---- 8. NNS G3D material: NNSi_G3dFuncSbc_MAT with the default material function -----------------
// NitroSystem G3D's SBC MAT command (PW 0x0206BEB8) calls NNSi_G3dFuncSbc_MAT_InternalDefault
// (MAT - 0x488) through its function table, which builds the material result (diffuse/ambient,
// specular/emission, polygon attr, texture image / palette) and sends it with one
// NNS_G3dGeBufferOP_N (no GE buffer: the packed command word to GXFIFO, then MI_CpuSend32 of the 6
// parameters). 81 calls a frame in the PW town, ~190 guest instructions each (16% of the ARM9's guest
// instructions; MAT_InternalDefault alone compiles to ~34 KB of JIT blocks). Natively at MAT's entry
// for the common case: no render callback at any timing of this command, no model material cache, no
// cached result (opt 0x20/0x40 with the material's bit set), no texture matrix, no material animation
// callback for this material, a visible material, render state flag 0x100 clear, no GE buffer, the
// geometry FIFO empty: the 7 words go through GPU3D::BulkWords (as GX_CPUSEND sends the parameters),
// every guest-memory byte (render state, material result, the stack frames of the 4 functions) and
// every register / flag as the guest leaves them. Anything else runs the guest code.
// Position independent: the exact code words (literal-pool globals and calls the native path never
// makes are masked), the globals read from the literal pools.
// Category B: a fixed cycle estimate (guest average in check mode).
constexpr u32 kMatInstr = 0xE92D4010;   // push {r4, lr}
constexpr u32 kMatOff = 0x488;          // MAT - MAT_InternalDefault (kMatCode[0])
// MAT_InternalDefault + MAT (from kMatCode[290]) with their literal pools
constexpr u32 kMatCode[334] = {
    0xE92D41F8, 0xE24DD01C, 0xE1A07000, 0xE1A05003, 0xE5C750AD, 0xE5973008, 0xE28700F4, 0xE3833008, 0xE5873008,
    0xE58700B0, 0xE597001C, 0xE1A08001, 0xE3500000, 0x15D74090, 0xE1A06002, 0x03A04000, 0xE3540001, 0x1A00000C,
    0xE5971008, 0xE1A00007, 0xE3C11040, 0xE5871008, 0xE597101C, 0xE12FFF31, 0xE597001C, 0xE3500000, 0x15D74090,
    0xE5970008, 0x03A04000, 0xE2000040, 0xEA000000, 0xE3A00000, 0xE3500000, 0x1A0000B5, 0xE5970004, 0xE5900038,
    0xE3500000, 0x0A000004, 0xE5971008, 0xE3110080, 0x03A01038, 0x00280195, 0x0A0000AB, 0xE3580020, 0x13580040,
    0x1A00000E, 0xE1A012A5, 0xE0871101, 0xE59110BC, 0xE205201F, 0xE3A03001, 0xE1110213, 0x0A000007, 0xE3500000,
    0x13A01038, 0x10280195, 0x1A00009D, 0xE59F1388, 0xE3A00038, 0xE0281095, 0xEA000099, 0xE3500000, 0x0A00000B,
    0xE28780BC, 0xE1A032A5, 0xE7982103, 0xE205001F, 0xE3A01001, 0xE1820011, 0xE7880103, 0xE5971004, 0xE3A00038,
    0xE5911038, 0xE0281095, 0xEA00000C, 0xE3580040, 0x128780F4, 0x1A000009, 0xE287E0BC, 0xE1A0C2A5, 0xE59F132C,
    0xE79E810C, 0xE3A00038, 0xE205201F, 0xE3A03001, 0xE1882213, 0xE0281095, 0xE78E210C, 0xE3A00000, 0xE5880000,
    0xE59730D8, 0xE3530000, 0x0A00000F, 0xE2932004, 0x0A000008, 0xE5D30005, 0xE1550000, 0x2A000005, 0xE1D310BA,
    0xE19200B1, 0xE0821001, 0xE2811004, 0xE0211590, 0xEA000000, 0xE3A01000, 0xE3510000, 0x15910000, 0x10830000,
    0x1A000000, 0xE3A00000, 0xE1D001BE, 0xE59F12B4, 0xE3100020, 0x15980000, 0x13800020, 0x15880000, 0xE1D621BE,
    0xE59F02A0, 0xE591C094, 0xE1A02342, 0xE2022007, 0xE790E102, 0xE5962004, 0xE1E0300E, 0xE00C3003, 0xE002200E,
    0xE1832002, 0xE5882004, 0xE1D6C1BE, 0xE5913098, 0xE5962008, 0xE1A0C4CC, 0xE20CC007, 0xE790C10C, 0xE1E0000C,
    0xE0033000, 0xE002000C, 0xE1830000, 0xE5880008, 0xE5963010, 0xE596000C, 0xE591209C, 0xE1E01003, 0xE0021001,
    0xE0000003, 0xE1810000, 0xE588000C, 0xE5960014, 0xE5880010, 0xE1D601BC, 0xE5880014, 0xE1D601BE, 0xE3100001,
    0x0A000022, 0xE3100002, 0x15981000, 0xE286002C, 0x13811001, 0x15881000, 0x1A000004, 0xE5901000, 0xE5881018,
    0xE5901004, 0xE2800008, 0xE588101C, 0xE1D611BE, 0xE3110004, 0x15981000, 0x13811002, 0x15881000, 0x1A000004,
    0xE1D010F0, 0xE1C812B0, 0xE1D010F2, 0xE2800004, 0xE1C812B2, 0xE1D611BE, 0xE3110008, 0x15980000, 0x13800004,
    0x15880000, 0x1A000003, 0xE5901000, 0xE5881024, 0xE5900004, 0xE5880028, 0xE5980000, 0xE3800008, 0xE5880000,
    0xE597C004, 0xE59C1008, 0xE3510000, 0x0A00000A, 0xE1A002A5, 0xE08C0100, 0xE590003C, 0xE205201F, 0xE3A03001,
    0xE1100213, 0x0A000003, 0xE59C300C, 0xE1A00008, 0xE1A02005, 0xE12FFF33, 0xE5980000, 0xE3100018, 0x0A000007,
    0xE1D602B0, 0xE1C802BC, 0xE1D602B2, 0xE1C802BE, 0xE5960024, 0xE5880030, 0xE5960028, 0xE5880034, 0xE58780B0,
    0xE3540002, 0x1A00000C, 0xE5971008, 0xE1A00007, 0xE3C11040, 0xE5871008, 0xE597101C, 0xE12FFF31, 0xE597001C,
    0xE3500000, 0x15D74090, 0xE5970008, 0x03A04000, 0xE2000040, 0xEA000000, 0xE3A00000, 0xE3500000, 0x1A000027,
    0xE59750B0, 0xE595100C, 0xE311081F, 0x0A000020, 0xE5950000, 0xE3100020, 0x13C1081F, 0x1585000C, 0xE5970008,
    0xE3C00002, 0xE5870008, 0xE3100C01, 0x1A00001A, 0xE59F009C, 0xE59F309C, 0xE58D0000, 0xE5952004, 0xE28D1004,
    0xE58D2004, 0xE5956008, 0xE3A02006, 0xE58D6008, 0xE595600C, 0xE58D600C, 0xE58D3010, 0xE5953010, 0xE58D3014,
    0xE5953014, 0xE58D3018, 0xEB0007DA, 0xE5950000, 0xE3100018, 0x0A000006, 0xE59710F0, 0xE1A00005, 0xE12FFF31,
    0xEA000002, 0xE5970008, 0xE3800002, 0xE5870008, 0xE3540003, 0x128DD01C, 0x18BD81F8, 0xE5971008, 0xE1A00007,
    0xE3C11040, 0xE5871008, 0xE597101C, 0xE12FFF31, 0xE28DD01C, 0xE8BD81F8, 0x02148DD4, 0x02148B6C, 0x020A196C,
    0x00293130, 0x00002B2A, 0xE92D4010, 0xE1A04000, 0xE5942008, 0xE3120C02, 0x1A000021, 0xE5940000, 0xE3120001,
    0xE5D03001, 0x1A000004, 0xE3120008, 0x0A000002, 0xE5D400AD, 0xE1530000, 0x0A000018, 0xE594E0D8, 0xE35E0000,
    0x0A00000F, 0xE29EC004, 0x0A000008, 0xE5DE0005, 0xE1530000, 0x2A000005, 0xE1DE20BA, 0xE19C00B2, 0xE08C2002,
    0xE2822004, 0xE0222390, 0xEA000000, 0xE3A02000, 0xE3520000, 0x15920000, 0x108E2000, 0x1A000000, 0xE3A02000,
    0xE1D2E0B0, 0xE59FC018, 0xE1A00004, 0xE79CC10E, 0xE12FFF3C, 0xE5940000, 0xE2800002, 0xE5840000, 0xE8BD8010,
    0x020A82E8,
};
constexpr u16 kMatSkip[] = {285, 286, 287, 333};    // literals: material cache, NNS_G3dGlb, mask table, MAT function table
// NNS_G3dGeBufferOP_N (MAT_InternalDefault's bl at word 263)
constexpr u32 kOpnCode[57] = {
    0xE92D4070, 0xE59F30D0, 0xE1A06000, 0xE593C000, 0xE1A05001, 0xE1A04002, 0xE35C0000, 0x0A000024, 0xE5930004,
    0xE3500000, 0x0A000016, 0xE59C2000, 0xE2820001, 0xE0801004, 0xE35100C0, 0x8A000011, 0xE58C0000, 0xE5930000,
    0xE3540000, 0xE0800102, 0xE5806004, 0x08BD8070, 0xE5932000, 0xE1A00005, 0xE4921004, 0xE0821101, 0xE1A02104,
    0xEB00533D, 0xE59F0064, 0xE5901000, 0xE5910000, 0xE0800004, 0xE5810000, 0xE8BD8070, 0xE59C0000, 0xE3500000,
    0x0A000001, 0xEBFFFF50, 0xEA000009, 0xE59F0038, 0xE5900004, 0xE3500000, 0x0A000005, 0xEBFFFF61, 0xEA000003,
    0xE5930004, 0xE3500000, 0x0A000000, 0xEBFFFF5C, 0xE59F1014, 0xE1A00005, 0xE1A02104, 0xE5816000, 0xEB00530A,
    0xE8BD8070, 0x0214BAD4, 0x04000400,
};
constexpr u16 kOpnSkip[] = {27, 37, 43, 48, 55};    // calls on the buffered / flush paths, the GE buffer literal
constexpr u32 kSend32Code[6] = {0xE080C002, 0xE150000C, 0xB8B00004, 0xB5812000, 0xBAFFFFFB, 0xE12FFF1E};   // MI_CpuSend32 (OP_N's bl at word 53)
// ponytail: fixed estimate (guest average in check mode, PW town / gift box / overworld / title)
constexpr s32 kMatCyc = 512, kMatCycNoSend = 325;

bool CodeEq(melonDS::ARMv5* c, u32 a, const u32* w, u32 n, const u16* sk, u32 ns)
{
    for (u32 i = 0, j = 0; i < n; i++)
    {
        if (j < ns && sk[j] == i) { j++; continue; }
        const u8* p = CodePtr(c, a + i * 4);
        if (!p || R32(p) != w[i]) return false;
    }
    return true;
}
u32 BlTarget(melonDS::ARMv5* c, u32 a)
{
    const u8* p = CodePtr(c, a);
    if (!p) return 0;
    const u32 w = R32(p);
    return (w >> 24) == 0xEB ? a + 8 + (u32)(((s32)(w << 8)) >> 6) : 0;
}
// entry: MAT's address; out: MAT_InternalDefault, OP_N, MI_CpuSend32. 0: not MAT, 1: MAT (code verified),
// 2: MAT's entry but the code differs somewhere (the JIT still depends on it: restoring it re-enables the hook)
int MatAt(melonDS::ARMv5* c, u32 entry, u32& def, u32& opn, u32& snd)
{
    def = entry - kMatOff;
    // cheap reject first (push {r4, lr} starts many functions): MAT's next two words
    const u8* p = (entry & 3) ? nullptr : CodePtr(c, entry + 4);
    const u8* q = p ? CodePtr(c, entry + 8) : nullptr;
    if (!q || R32(p) != kMatCode[291] || R32(q) != kMatCode[292]) return 0;
    opn = BlTarget(c, def + 263 * 4);
    snd = opn ? BlTarget(c, opn + 53 * 4) : 0;
    return CodeEq(c, def, kMatCode, 334, kMatSkip, 4) && opn && CodeEq(c, opn, kOpnCode, 57, kOpnSkip, 5)
           && snd && CodeEq(c, snd, kSend32Code, 6, nullptr, 0) ? 1 : 2;
}

bool MatNative(melonDS::ARMv5* c, Mem& m, Expect& e, u32 pc, u32 def, u32 opn, u32* gw, bool& send)
{
    bool bad = false;
    auto rd = [&](u32 a, u32 n) -> u32 {
        const u8* p = (a & (n - 1)) ? nullptr : m.P(a);
        if (!p || m.P(a + n - 1) != p + n - 1) { bad = true; return 0; }
        return n == 4 ? R32(p) : n == 2 ? R16(p) : *p;
    };
    auto lit = [&](u32 a) { return R32(CodePtr(c, a)); };
    const u32 rs = c->R[0], opt = c->R[1], sp = c->R[13];
    // NNSi_G3dFuncSbc_MAT: skip conditions, the material's resource entry, the function table
    Obj RS = m.O(rs, 0x100);
    if (!RS) return false;
    const u32 flag = RS.r(8);
    if (flag & 0x200) return false;
    const u32 sbc = RS.r(0), idx = rd(sbc + 1, 1);
    if (bad || (!(flag & 1) && (flag & 8) && idx == RS.p[0xAD])) return false;
    const u32 res = RS.r(0xD8);
    if (!res) return false;
    const u32 nEnt = rd(res + 5, 1), ofs = rd(res + 0xA, 2);
    if (bad || idx >= nEnt) return false;
    const u32 dict = res + 4, ent = dict + ofs + 4 + rd(dict + ofs, 2) * idx;
    const u32 md = res + rd(ent, 4);
    if (bad || !ent || rd(lit(def + 333 * 4) + rd(md, 2) * 4, 4) != def || bad) return false;
    // MAT_InternalDefault(rs, opt, md, idx)
    const u32 cb = RS.r(0x1C), cbT = cb ? RS.p[0x90] : 0;
    if (cbT >= 1 && cbT <= 3) return false;
    const u32 ro = RS.r(4);
    if (rd(ro + 0x38, 4) || bad) return false;
    const u32 bo = 0xBC + (idx >> 5) * 4, bit = 1u << (idx & 31), bits = RS.r(bo);
    if ((opt == 0x20 || opt == 0x40) && (bits & bit)) return false;
    const u32 r8 = opt == 0x40 ? lit(def + 285 * 4) + idx * 0x38 : rs + 0xF4;
    Obj R8 = m.O(r8, 0x38);
    const u32 h = rd(md + 0x1E, 2);
    if (!R8 || bad || (h & 1)) return false;
    const u32 anm = rd(ro + 8, 4);
    if (anm && (rd(ro + 0x3C + (idx >> 5) * 4, 4) & bit)) return false;    // material animation callback
    const u32 glb = lit(def + 286 * 4), mt = lit(def + 287 * 4);
    const u32 t1 = rd(mt + ((h >> 6) & 7) * 4, 4), t2 = rd(mt + ((h >> 9) & 7) * 4, 4);
    const u32 v0 = h & 0x20;
    const u32 v4 = (rd(glb + 0x94, 4) & ~t1) | (rd(md + 4, 4) & t1);
    const u32 v8 = (rd(glb + 0x98, 4) & ~t2) | (rd(md + 8, 4) & t2);
    const u32 m10 = rd(md + 0x10, 4);
    const u32 g9c = rd(glb + 0x9C, 4);
    u32 vc = (g9c & ~m10) | (rd(md + 0xC, 4) & m10);
    const u32 v10 = rd(md + 0x14, 4), v14 = rd(md + 0x1C, 2);
    send = vc & 0x1F0000;   // else an invisible material: [rs + 8] |= 2, no send
    const u32 fl = send ? (flag | 8) & ~2u : flag | 0xA;
    if (bad || (send && (fl & 0x100))) return false;     // (send path) the no-send flag
    if (send && v0) vc &= ~0x1F0000u;
    Obj st = m.O(sp - 80, 80);
    if (!st) return false;
    const u32 cmd0 = kMatCode[288], cmd1 = kMatCode[289];
    // stack: MAT push {r4, lr}; MAT_InternalDefault push {r3-r8, lr} + locals (the OP_N arguments); OP_N push {r4-r6, lr}
    m.W(st, 72, c->R[4]); m.W(st, 76, c->R[14]);
    m.W(st, 44, idx); m.W(st, 48, rs); m.W(st, 52, c->R[5]); m.W(st, 56, c->R[6]); m.W(st, 60, c->R[7]); m.W(st, 64, c->R[8]);
    m.W(st, 68, pc + 0x9C);
    if (send)
    {
        // NNS_G3dGeBufferOP_N: no buffering ([ge + 4] clear) and no GE buffer or an empty one: straight to GXFIFO
        const u32 ge = lit(opn + 55 * 4), gb = rd(ge, 4);
        if (rd(ge + 4, 4) || (gb && rd(gb, 4)) || bad) return false;
        m.W(st, 16, cmd0); m.W(st, 20, v4); m.W(st, 24, v8); m.W(st, 28, vc); m.W(st, 32, cmd1); m.W(st, 36, v10); m.W(st, 40, v14);
        m.W(st, 0, cbT); m.W(st, 4, r8); m.W(st, 8, vc); m.W(st, 12, def + 0x420);
    }
    // render state, material result
    m.W(RS, 0xAC, (RS.r(0xAC) & ~0xFF00u) | idx << 8);
    m.W(RS, 8, fl);
    m.W(RS, 0xB0, r8);
    if (opt == 0x40) m.W(RS, bo, bits | bit);
    m.W(RS, 0, sbc + 2);
    m.W(R8, 0, v0); m.W(R8, 4, v4); m.W(R8, 8, v8); m.W(R8, 0xC, vc); m.W(R8, 0x10, v10); m.W(R8, 0x14, v14);
    gw[0] = cmd0; gw[1] = v4; gw[2] = v8; gw[3] = vc; gw[4] = cmd1; gw[5] = v10; gw[6] = v14;
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.R[0] = sbc + 2; e.R[3] = idx;
    if (send) { e.R[1] = kOpnCode[56]; e.R[2] = v14; e.R[12] = sp - 36; e.R[14] = opn + 0xD8; }
    else { e.R[1] = vc; e.R[2] = anm ? idx & 31 : g9c; e.R[12] = ro; e.R[14] = t1; }
    e.retPc = c->R[14];
    e.CPSR = (c->CPSR & 0x0FFFFFDF) | Flags(cbT, 3) | ((e.retPc & 1) << 5);
    return true;
}

__attribute__((noinline)) bool RunMat(melonDS::ARMv5* c, State& s, bool jit)
{
    const u32 pc = c->R[15] - 8;
    u32 def, opn, snd;
    if (jit)
    {
        def = pc - kMatOff; opn = BlTarget(c, def + 263 * 4);
        (void)snd;
    }
    else if (MatAt(c, pc, def, opn, snd) != 1) return false;
    s.calls[8]++;
    melonDS::GPU3D& gx = c->NDS.GPU.GPU3D;
    Mem m(c, g_Check || g_Dry);
    Expect e;
    u32 gw[7];
    bool send = false;
    bool ok = !CheckPending && opn && MatNative(c, m, e, pc, def, opn, gw, send) && (!send || (gx.GeometryEnabled && gx.BulkReady()));
#ifdef LITEV_HLE_DIAG
    if (ok && (g_Check || g_Dry))
    {
        m.Flush();      // logs only
        if (g_Check)
        {
            ArmCheck(c, 8, m, e);
            g_P.gx.assign(gw, gw + (send ? 7 : 0));
            g_P.fit[0] = send;
#ifdef LITEV_A9HLE_GXCHECK
            if (!GxTap) { g_GxMatTap.clear(); GxTap = &g_GxMatTap; g_P.gxOn = true; }
#endif
        }
        s.checks[8]++;
        ok = false;
    }
#endif
    if (!ok)
    {
        s.fallback[8] += !g_Check;
        GuestFallback(c);
        return true;
    }
    m.Flush();
    if (send) gx.BulkWords(gw, 7);
    s.native[8]++;
    Return(c, e, send ? kMatCyc : kMatCycNoSend);
    return true;
}
#endif

#ifdef LITEV_GX_BULK
// ---- 8. in Thumb (W2, TWL SDK build) ---------------------------------------------------------------
// SBC MAT -> MAT_InternalDefault -> NNS_G3dGeBufferOP_N -> MI_CpuSend32: the same material logic as 8. (NNS_G3dGlb
// defaults at +0x14/+0x18/+0x1C in this build), natively at SBC MAT's (Thumb) entry: 154 calls a frame in the W2 town,
// ~250 guest instructions each (MAT_InternalDefault alone 21% of the town's ARM9 guest instructions). Fallback as 8.
// (render callbacks, material cache, a cached result, texture matrix, a material animation callback, GE buffering).
// ponytail: fixed estimates (check-mode guest averages, W2 town)
constexpr s32 kMatTCyc = 483, kMatTCycNoSend = 307;   // (no invisible material in the W2 states: 8.'s ratio)

bool MatTNative(melonDS::ARMv5* c, Mem& m, Expect& e, const MatT& t, u32* gw, bool& send)
{
    bool bad = false;
    auto rd = [&](u32 a, u32 n) -> u32 {
        const u8* p = (a & (n - 1)) ? nullptr : m.P(a);
        if (!p || m.P(a + n - 1) != p + n - 1) { bad = true; return 0; }
        return n == 4 ? R32(p) : n == 2 ? R16(p) : *p;
    };
    const u32 rs = c->R[0], opt = c->R[1], S = c->R[13];
    Obj RS = m.O(rs, 0x100);
    if (!RS) return false;
    // SBC MAT: skip conditions, the material, its function from the table
    const u32 flag = RS.r(8);
    if (flag & 0x200) return false;
    const u32 sbc = RS.r(0), idx = rd(sbc + 1, 1);
    if (bad || (!(flag & 1) && (flag & 8) && idx == RS.p[0xAD])) return false;
    const u32 res = RS.r(0xD8);
    if (!res) return false;
    const u32 nEnt = rd(res + 5, 1), ofs = rd(res + 0xA, 2);
    if (bad || idx >= nEnt) return false;
    const u32 dict = res + 4, ent = dict + ofs + 4 + rd(dict + ofs, 2) * idx;
    const u32 md = res + rd(ent, 4), hmd = rd(md, 2), fn = rd(t.tab + hmd * 4, 4);
    if (bad || !ent || fn != (t.def | 1)) return false;
    // MAT_InternalDefault(rs, opt, md, idx)
    const u32 cb = RS.r(0x1C), cbT = cb ? RS.p[0x90] : 0;
    if (cbT >= 1 && cbT <= 3) return false;
    const u32 ro = RS.r(4);
    if (rd(ro + 0x38, 4) || bad) return false;
    const u32 bo = 0xBC + (idx >> 5) * 4, bit = 1u << (idx & 31), bits = RS.r(bo);
    if ((opt == 0x20 || opt == 0x40) && (bits & bit)) return false;
    const u32 r8 = opt == 0x40 ? t.cache + idx * 0x38 : rs + 0xF4;
    Obj R8 = m.O(r8, 0x38);
    const u32 h = rd(md + 0x1E, 2);
    if (!R8 || bad || (h & 1)) return false;
    const u32 anm = rd(ro + 8, 4);
    if (anm && (rd(ro + 0x3C + (idx >> 5) * 4, 4) & bit)) return false;     // material animation callback
    const u32 t1 = rd(t.mask + ((h >> 6) & 7) * 4, 4), t2 = rd(t.mask + ((h >> 9) & 7) * 4, 4);
    const u32 v0 = h & 0x20;
    const u32 v4 = (rd(t.glb + 0x14, 4) & ~t1) | (rd(md + 4, 4) & t1);
    const u32 v8 = (rd(t.glb + 0x18, 4) & ~t2) | (rd(md + 8, 4) & t2);
    const u32 m10 = rd(md + 0x10, 4);
    u32 vc = (rd(t.glb + 0x1C, 4) & ~m10) | (rd(md + 0xC, 4) & m10);
    const u32 v10 = rd(md + 0x14, 4), v14 = rd(md + 0x1C, 2);
    send = vc & 0x1F0000;
    const u32 fl8 = flag | 8, fl = send ? fl8 & ~2u : fl8 | 2;
    if (bad || (send && (fl & 0x100))) return false;
    if (send)
    {
        // OP_N: nothing being sent, no GE buffer or an empty one: straight to GXFIFO (the other paths leave the same
        // registers once MI_CpuSend32 has run)
        const u32 gb = rd(t.ge, 4);
        if (rd(t.ge + 4, 4) || (gb && rd(gb, 4)) || bad) return false;
    }
    if (send && v0) vc &= ~0x1F0000u;
    // stack (S = sp): SBC MAT push {r4-r6, lr} at S-16; MAT_InternalDefault push {r3-r7, lr} at S-40, locals at M = S-72
    // ([M] the callback timing, [M+4] the command word, [M+8] the 6 parameters); OP_N push {r3-r7, lr} at M-24
    Obj st = m.O(S - 96, 96);
    if (!st) return false;
    const u32 M = S - 72;
    const u32 w0[4] = {c->R[4], c->R[5], c->R[6], c->R[14]}, w1[6] = {idx, rs, fn, hmd * 4, c->R[7], t.sbc + 0x67};
    for (int i = 0; i < 4; i++) m.W(st, 80 + i * 4, w0[i]);
    for (int i = 0; i < 6; i++) m.W(st, 56 + i * 4, w1[i]);
    m.W(st, 24, cbT);
    const u32 cmd0 = 0x00293130, cmd1 = 0x00002B2A;
    if (send)
    {
        const u32 w2[7] = {cmd0, v4, v8, vc, cmd1, v10, v14}, w3[6] = {ro, r8, rs, md, idx, t.def + 0x2D3};
        for (int i = 0; i < 7; i++) m.W(st, 28 + i * 4, w2[i]);
        for (int i = 0; i < 6; i++) m.W(st, i * 4, w3[i]);
    }
    // render state, material result
    m.W(RS, 0xAC, (RS.r(0xAC) & ~0xFF00u) | idx << 8);
    m.W(RS, 8, fl);
    m.W(RS, 0xB0, r8);
    if (opt == 0x40) m.W(RS, bo, bits | bit);
    m.W(RS, 0, sbc + 2);
    m.W(R8, 0, v0); m.W(R8, 4, v4); m.W(R8, 8, v8); m.W(R8, 0xC, vc); m.W(R8, 0x10, v10); m.W(R8, 0x14, v14);
    gw[0] = cmd0; gw[1] = v4; gw[2] = v8; gw[3] = vc; gw[4] = cmd1; gw[5] = v10; gw[6] = v14;
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.R[0] = sbc + 2; e.R[3] = idx;
    if (send) { e.R[1] = v0; e.R[2] = v14; e.R[12] = M + 32; e.R[14] = t.opn + 0x7D; }
    else { e.R[1] = fl8; e.R[2] = anm; e.R[12] = dict + ofs + 4; e.R[14] = t.sbc + 0x67; }
    e.retPc = c->R[14];
    // SBC MAT's adds r0, r0, #2 sets the flags
    const u32 r = sbc + 2, nzcv = (r & 0x80000000) | ((r == 0) << 30) | ((r < sbc) << 29) | (((~(sbc ^ 2u) & (sbc ^ r)) >> 31) << 28);
    e.CPSR = (c->CPSR & 0x0FFFFFDF) | nzcv | ((e.retPc & 1) << 5);
    return true;
}

__attribute__((noinline)) bool RunMatT(melonDS::ARMv5* c, State& s, bool jit)
{
    melonDS::GPU3D& gx = c->NDS.GPU.GPU3D;
    Mem m(c, g_Check || g_Dry);
    Expect e;
    u32 gw[7];
    bool send = false;
    bool ok = !CheckPending && s.matTOk && (jit || MatTIntact(c, s)) && MatTNative(c, m, e, *s.v->matT, gw, send)
              && (!send || (gx.GeometryEnabled && gx.BulkReady()));
#ifdef LITEV_HLE_DIAG
    if (ok && (g_Check || g_Dry))
    {
        m.Flush();      // logs only
        if (g_Check)
        {
            ArmCheck(c, 8, m, e);
            g_P.gx.assign(gw, gw + (send ? 7 : 0));
            g_P.fit[0] = send;
#ifdef LITEV_A9HLE_GXCHECK
            if (!GxTap) { g_GxMatTap.clear(); GxTap = &g_GxMatTap; g_P.gxOn = true; }
#endif
        }
        s.checks[8]++;
        ok = false;
    }
#endif
    if (!ok)
    {
        s.fallback[8] += !g_Check;
        GuestFallback(c);
        return true;
    }
    m.Flush();
    if (send) gx.BulkWords(gw, 7);
    s.native[8]++;
    Return(c, e, send ? kMatTCyc : kMatTCycNoSend);
    return true;
}
#endif

// ---- 14. VEC_Normalize (NitroSDK fx32 vector normalize) ----------------------------------------------------------
// len2 = x*x + y*y + z*z (64-bit), the divider in 64/64 mode for 2^56 / len2 and the square root unit in 64-bit mode
// for sqrt(len2 * 4), two busy-wait loops, then three 64-bit fixed-point products: 66 guest instructions, 14 JIT blocks /
// 11 KB of host code (the wait loops split it), 58 calls a frame in the PW title. Natively at its entry: the same 8 IO
// stores (the divider / sqrt registers end as the guest leaves them, results through the normal IO reads, so a zero
// length behaves as the hardware), the results, the stack frame, registers and flags. Position independent (exact
// code words; PW, PB, W2). Category B: a fixed cycle estimate.
constexpr u32 kVecInstr = 0xE92D4FF8;   // push {r3-r11, lr}
constexpr u32 kVecCode[69] = {
    0xE92D4FF8, 0xE5902004, 0xE5903000, 0xE0C76292, 0xE0E76393, 0xE5902008, 0xE59F50E8, 0xE0E76292, 0xE3A03002, 0xE1C530B0,
    0xE3A04000, 0xE5854010, 0xE3A03401, 0xE5853014, 0xE5856018, 0xE1A02107, 0xE585701C, 0xE3A03001, 0xE1C533B0, 0xE1A04106,
    0xE5854038, 0xE1822F26, 0xE585203C, 0xE1D523B0, 0xE3120902, 0x1AFFFFFC, 0xE59F209C, 0xE592C000, 0xE2423034, 0xE1D320B0,
    0xE3120902, 0x1AFFFFFC, 0xE59F7088, 0xE5908000, 0xE5976000, 0xE9904020, 0xE0823C96, 0xE1A00FCC, 0xE0222096, 0xE597B004,
    0xE089A893, 0xE1A04FC8, 0xE0222C9B, 0xE0299493, 0xE0299892, 0xE1A00FC5, 0xE0867593, 0xE0266093, 0xE29AA000, 0xE2A90A01,
    0xE1A046C0, 0xE5814000, 0xE1A00FCE, 0xE08C4E93, 0xE02CC093, 0xE0266592, 0xE2970000, 0xE2A60A01, 0xE1A036C0, 0xE02CCE92,
    0xE2940000, 0xE2AC0A01, 0xE1A006C0, 0xE5813004, 0xE5810008, 0xE8BD8FF8, 0x04000280, 0x040002B4, 0x040002A0};
// ponytail: fixed estimate (check-mode guest average, PW title)
constexpr s32 kVecCyc = 213;   // (every one of 22.9k check-mode calls: 213)

bool VecAt(melonDS::ARMv5* c, u32 a, int n)
{
    if (a & 3) return false;
    for (int i = 0; i < n; i++) { const u8* p = CodePtr(c, a + i * 4); if (!p || R32(p) != kVecCode[i]) return false; }
    return true;
}

__attribute__((noinline)) bool RunVec(melonDS::ARMv5* c, State& s, bool jit)
{
    const u32 pc = c->R[15] - 8;
    if (!jit && !VecAt(c, pc, 69)) return false;
    s.calls[14]++;
    melonDS::NDS& nds = c->NDS;
    const u32 src = c->R[0], dst = c->R[1], sp = c->R[13];
    Mem m(c, g_Check || g_Dry);
    Obj sv = m.O(src, 12), dv = m.O(dst, 12), st = m.O(sp - 40, 40);
    // the IO page passes the protection unit (else the guest's data abort)
    bool ok = !CheckPending && sv && dv && st && (c->PU_Map[0x04000280 >> 12] & 0x03) == 0x03;
    if (!ok)
    {
        s.fallback[14] += !g_Check;
        GuestFallback(c);
        return true;
    }
    const s32 x = (s32)sv.r(0), y = (s32)sv.r(4), z = (s32)sv.r(8);
    const u64 l2 = (u64)((s64)x * x) + (u64)((s64)y * y) + (u64)((s64)z * z);
    const u32 lo = (u32)l2, hi = (u32)(l2 >> 32);
    // the guest's stores (in check mode too: the guest repeats them with the same values), then the results
    nds.ARM9Write16(0x04000280, 2);
    nds.ARM9Write32(0x04000290, 0); nds.ARM9Write32(0x04000294, 0x01000000);
    nds.ARM9Write32(0x04000298, lo); nds.ARM9Write32(0x0400029C, hi);
    nds.ARM9Write16(0x040002B0, 1);
    nds.ARM9Write32(0x040002B8, lo << 2); nds.ARM9Write32(0x040002BC, (hi << 2) | (lo >> 30));
    const u32 sq = nds.ARM9Read32(0x040002B4), dlo = nds.ARM9Read32(0x040002A0), dhi = nds.ARM9Read32(0x040002A4);
    const u64 f = ((u64)dhi << 32 | dlo) * (u64)(s64)(s32)sq;
    auto comp = [&](s32 v, u32& plo) { const u64 p = f * (u64)(s64)v; plo = (u32)p; return (u32)((s32)((u32)(p >> 32) + 0x1000) >> 13); };
    u32 px, py, pz;
    const u32 rx = comp(x, px), ry = comp(y, py), rz = comp(z, pz);
    for (int i = 0; i < 9; i++) m.W(st, i * 4, c->R[3 + i]);
    m.W(st, 36, c->R[14]);
    m.W(dv, 0, rx); m.W(dv, 4, ry); m.W(dv, 8, rz);
    Expect e;
    for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
    e.R[0] = rz; e.R[2] = (u32)(f >> 32); e.R[12] = (u32)((f * (u64)(s64)z) >> 32); e.R[14] = (u32)z;   // ldmib r0, {r5, lr}
    e.retPc = c->R[14];
    e.CPSR = (c->CPSR & 0x0FFFFFDF) | (pz & 0x80000000) | ((pz == 0) << 30) | ((e.retPc & 1) << 5);    // adds r0, r4, #0
#ifdef LITEV_HLE_DIAG
    if (g_Check || g_Dry)
    {
        m.Flush();      // logs only
        if (g_Check) ArmCheck(c, 14, m, e);
        s.checks[14]++;
        GuestFallback(c);
        return true;
    }
#endif
    m.Flush();
    s.native[14]++;
    Return(c, e, kVecCyc);
    return true;
}

// ---- 9. _ll_sdiv: 64-bit signed divide of the compiler runtime -----------------------------------
// r1:r0 / r3:r2 by shift-and-subtract (~750 guest instructions a call; PW: 2-3 calls a frame from one
// caller in 3D scenes). Natively: the quotient, r3:r2 = |divisor| normalized (shifted left until bit
// 63 is set), the flags of the final cmp / rsbs, the 7 pushed words below sp; r4-r7, fp, ip, lr as on
// entry (popped). Divide by zero, the 32-bit fast path (both operands sign-extended 32-bit: the guest
// calls the 32-bit divide) and INT64_MIN operands run the guest code. Position independent (exact
// code words; the 32-bit path's call masked). Category B: cycles from a fit of the guest's own count.
constexpr u32 kSdivInstr = 0xE92D58F0;  // push {r4-r7, fp, ip, lr}
constexpr u32 kSdivCode[108] = {
    0xE92D58F0, 0xE0214003, 0xE1A040C4, 0xE1A04084, 0xE1935002, 0x1A000001, 0xE8BD58F0, 0xE12FFF1E, 0xE1A05FA0,
    0xE0855001, 0xE1A06FA2, 0xE0866003, 0xE1956006, 0x1A000006, 0xE1A01002, 0xEB000081, 0xE2144001, 0x11A00001,
    0xE1A01FC0, 0xE8BD58F0, 0xE12FFF1E, 0xE3510000, 0xAA000001, 0xE2700000, 0xE2E11000, 0xE3530000, 0xAA000001,
    0xE2722000, 0xE2E33000, 0xE1915000, 0x0A000046, 0xE3A05000, 0xE3A06001, 0xE3530000, 0x4A000004, 0xE2855001,
    0xE0922002, 0xE0B33003, 0x5AFFFFFB, 0xE0866005, 0xE3510000, 0xBA000005, 0xE3560001, 0x0A000003, 0xE2466001,
    0xE0900000, 0xE0B11001, 0x5AFFFFF9, 0xE3A07000, 0xE3A0C000, 0xE3A0B000, 0xEA000005, 0xE38CC001, 0xE2566001,
    0x0A000018, 0xE0900000, 0xE0B11001, 0xE0B77007, 0xE0500002, 0xE0D11003, 0xE2D77000, 0xE09CC00C, 0xE0ABB00B,
    0xE3570000, 0xAAFFFFF2, 0xE2566001, 0x0A00000A, 0xE0900000, 0xE0B11001, 0xE0A77007, 0xE0900002, 0xE0B11003,
    0xE2A77000, 0xE09CC00C, 0xE0ABB00B, 0xE3570000, 0xAAFFFFE6, 0xEAFFFFF2, 0xE0900002, 0xE0A11003, 0xE2147001,
    0x01A0000C, 0x01A0100B, 0x0A000009, 0xE2557020, 0xA1A00731, 0xAA00000F, 0xE2657020, 0xE1A00530, 0xE1800711,
    0xE1A01531, 0xEA000001, 0xE1A00731, 0xE3A01000, 0xE3540000, 0xBA000001, 0xE8BD58F0, 0xE12FFF1E, 0xE2700000,
    0xE2E11000, 0xE8BD58F0, 0xE12FFF1E, 0xE3A00000, 0xE3A01000, 0xE3540000, 0xBAFFFFF7, 0xE8BD58F0, 0xE12FFF1E,
};
constexpr u16 kSdivSkip[] = {15};
// ponytail: fit of the guest's own count (check mode without IRQs, PW town/title: 1190-1241 cycles a call, within
// ~1%): base + per normalization shift + per quotient bit + per dividend shift; zero dividend: a short path
constexpr s32 kSdivCyc0 = 631, kSdivCycNorm = 6, kSdivCycBit = 7, kSdivCycSh = 3, kSdivCycZero = 40;

__attribute__((noinline)) bool RunSdiv(melonDS::ARMv5* c, State& s, bool jit)
{
    const u32 pc = c->R[15] - 8;
    if (!jit && !CodeEq(c, pc, kSdivCode, 108, kSdivSkip, 1)) return false;
    s.calls[9]++;
    const u32 r0 = c->R[0], r1 = c->R[1], r2 = c->R[2], r3 = c->R[3], sp = c->R[13];
    const u64 n = (u64)r1 << 32 | r0, d = (u64)r3 << 32 | r2, kMin = 1ull << 63;
    Mem m(c, g_Check || g_Dry);
    Obj st = m.O(sp - 28, 28);
    bool ok = !CheckPending && st && d && (((r0 >> 31) + r1) | ((r2 >> 31) + r3)) && n != kMin && d != kMin;
    if (ok)
    {
        const u32 r4 = (r1 ^ r3) & ~1u;
        const u64 an = (s64)n < 0 ? -n : n, ad = (s64)d < 0 ? -d : d;
        const int k = __builtin_clzll(ad);
        const u64 q = an / ad, dd = an ? ad << k : ad;
        const int sh = an ? (__builtin_clzll(an) < k ? __builtin_clzll(an) : k) : 0;
        Expect e;
        for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
        u32 fl;
        u64 res = q;
        if ((s32)r4 >= 0) fl = Flags(r4, 0);
        else { fl = Flags(0, (u32)q); res = -q; }
        e.R[0] = (u32)res; e.R[1] = (u32)(res >> 32); e.R[2] = (u32)dd; e.R[3] = (u32)(dd >> 32);
        e.retPc = c->R[14];
        e.CPSR = (c->CPSR & 0x0FFFFFDF) | fl | ((e.retPc & 1) << 5);
        for (int i = 0; i < 4; i++) m.W(st, i * 4, c->R[4 + i]);
        m.W(st, 16, c->R[11]); m.W(st, 20, c->R[12]); m.W(st, 24, c->R[14]);
        const s32 cyc = an ? kSdivCyc0 + kSdivCycNorm * k + kSdivCycBit * (1 + k - sh) + kSdivCycSh * sh : kSdivCycZero;
#ifdef LITEV_HLE_DIAG
        if (g_Check || g_Dry)
        {
            m.Flush();      // logs only
            if (g_Check) { ArmCheck(c, 9, m, e); g_P.fit[0] = k; g_P.fit[1] = sh; g_P.fit[2] = an != 0; g_P.fit[3] = cyc; }
            s.checks[9]++;
            ok = false;
        }
#endif
        if (ok)
        {
            m.Flush();
            s.native[9]++;
            Return(c, e, cyc);
            return true;
        }
    }
    s.fallback[9] += !g_Check;
    GuestFallback(c);
    return true;
}

struct StatsDump
{
    ~StatsDump()
    {
        if (!g_Stats) return;
        for (auto& [k, s] : g_State)
            for (int i = 0; i < kKinds; i++)
                fprintf(stderr, "A9HLE %s: status=%d calls=%llu native=%llu fallback=%llu checks=%llu check_diffs=%llu irq_during=%llu guest_cyc_avg=%.0f\n",
                        kName[i], s.status, (unsigned long long)s.calls[i], (unsigned long long)s.native[i],
                        (unsigned long long)s.fallback[i], (unsigned long long)s.checks[i], (unsigned long long)s.diffs[i],
                        (unsigned long long)s.irqDuring[i], s.guestN[i] ? (double)s.guestCyc[i] / s.guestN[i] : 0.0);
        for (auto& [k, s] : g_State)
        {
            u64 n = s.native[0] + s.native[1] + s.native[2] + s.native[3] + s.native[4] + s.native[5];
            if (s.native[5]) fprintf(stderr, "A9HLE gxsend: %.1f words per native call\n", (double)s.gxWords / s.native[5]);
            if (s.native[6]) fprintf(stderr, "A9HLE lz: %.1f bytes per native call\n", (double)s.lzBytes / s.native[6]);
            if (s.native[7]) fprintf(stderr, "A9HLE cardread: %.1f words per native call\n", (double)s.cardWords / s.native[7]);
            if (s.checks[6] && s.guestN[6]) fprintf(stderr, "A9HLE lz check: %.1f bytes per call, guest cycles %llu, estimate %llu\n",
                                                   (double)s.lzBytes / s.checks[6], (unsigned long long)s.guestCyc[6], (unsigned long long)s.lzEst);
            if (n && g_Time) fprintf(stderr, "A9HLE: %.1f ns per native call (Run, timer pair %.1f ns subtracted)\n", (double)s.ns / n - TimerNs(), TimerNs());
        }
        for (auto& [k, s] : g_State)
            if (s.checks[2]) fprintf(stderr, "A9HLE getirqfn: avg lowest set bit %.2f\n", (double)s.getBits / s.checks[2]);
#ifdef LITEV_A9HLE_GXCHECK
        for (auto& [k, s] : g_State)
            if (s.checks[5]) fprintf(stderr, "A9HLE gxsend check: %llu resolved (irq_during above), %.0f guest cycles per elided chunk, of them DMA %.0f\n",
                                     (unsigned long long)s.irqDuring[5], s.guestN[5] ? (double)s.guestCyc[5] / s.guestN[5] : 0.0,
                                     s.guestN[5] ? (double)g_GxDmaCyc / s.guestN[5] : 0.0);
#endif
    }
} g_StatsDump;
}

void HookCompiled(melonDS::NDS& nds, u32 addr, const void* block)
{
    melonDS::ARMv5* c = &nds.ARM9;
    State* s = Active(c);
    if (!s || addr != s->v->wake) return;
    // the wake hook block depends on all of the variant's code (Deps): while it lives, the IRQ path is intact
    s->hbLive = s->irqOk && (s->mask & 8) && IrqCodeIntact(c, *s);
    s->dmaLive = s->dmaOk && (s->mask & 1024) && DmaIntact(c, *s) && CodeIntact(c, *s, 1);
    c->A9HLEGuard = s->hbLive || s->dmaLive ? block : nullptr;
}

void BlockGone(melonDS::NDS& nds, const void* block)
{
    if (nds.ARM9.A9HLEGuard == block) nds.ARM9.A9HLEGuard = nullptr;
}

namespace
{
// one IRQ natively (3. HBlank, 11. a display list's DMA end): 1 done, 0 guest path, -1 check / dry (guest path, stop)
int IrqOne(melonDS::ARMv5* c, State& s, bool halted, bool jit, u32 low)
{
    melonDS::NDS& nds = c->NDS;
    const u64 t0 = g_Time ? Now() : 0;
    Mem m(c, g_Check || g_DryIrq);
    Expect e;
    int kind = 3;
    u32 tpc = 0, tcpsr = 0;
    IoPlan io;
    if (low == 2)
    {
        if (jit ? !s.hbLive : (!s.irqOk || !(s.mask & 8) || !IrqCodeIntact(c, s))) return 0;
        if (!IrqNative(c, s, m, e, halted, kind, tpc, tcpsr)) return 0;
    }
    else
    {
        if (jit ? !s.dmaLive : (!s.dmaOk || !(s.mask & 1024) || !DmaIntact(c, s) || !CodeIntact(c, s, 1))) return 0;
        kind = 11;
        if (!DmaIrqNative(c, s, m, e, halted, __builtin_ctz(low), io)) return 0;
    }
    s.calls[kind]++;
    m.Flush();                                          // check / dry: logs only
#ifdef LITEV_HLE_DIAG
    if (__builtin_expect(g_Check, 0))
    {
        ArmCheck(c, kind, m, e);
        g_P.vecSkip = true;
        g_P.io.clear();
        for (u32 i = 0; i < io.n; i++) g_P.io.push_back({io.a[i], io.a[i] == 0x04000600 ? io.v[i] & 0xC0008000u : io.v[i]});
        s.checks[kind]++;
        return -1;
    }
    if (__builtin_expect(g_DryIrq, 0)) { s.checks[kind]++; return -1; }
#endif
    s.native[kind]++;
    if (kind == 11) for (u32 i = 0; i < io.n; i++) nds.ARM9Write32(io.a[i], io.v[i]);
    else
    {
        nds.IF[0] &= ~2u;                               // OS_IrqHandler: str r1, [IF]
        nds.GPU.GPU3D.CheckFIFOIRQ();
        nds.UpdateIRQ(0);
    }
    if (kind == 4) { c->R_SVC[1] = e.SVC[1]; c->R_SVC[2] = e.SVC[2]; }
    c->R_IRQ[1] = e.IRQ[1];
    c->R_IRQ[2] = e.IRQ[2];
    c->Cycles += kind == 4 ? kIrqWakeCycles : kind == 3 ? kIrqCycles : s.v->twl ? kDmaIrqCyclesT : kDmaIrqCycles;
    if (g_Time) s.ns += Now() - t0;
    return 1;
}
}

bool Irq(melonDS::ARMv5* c, bool halted)
{
    if (CheckPending) return false;
#ifdef LITEV_HLE_DIAG
    static bool dryHalted = false;      // dry: the halted call already computed this delivery
    if (__builtin_expect(g_DryIrq, 0) && dryHalted) { dryHalted = false; return false; }
#endif
    const bool jit = c->A9HLEGuard != nullptr;
    if (!jit && c->NDS.IsJITEnabled()) return false;
    State* sp = Active(c);
    if (!sp) return false;
    bool any = false;
    // the lowest pending enabled IRQ is the one OS_IrqHandler takes; after a native one, the next (an HBlank and a
    // DMA end can be pending together)
    for (int i = 0; i < 4; i++)
    {
        const u32 p = c->NDS.IE[0] & c->NDS.IF[0], low = p & (0u - p);
        if (low != 2 && !(low & 0xE00)) break;
        const int r = IrqOne(c, *sp, halted, jit, low);
#ifdef LITEV_HLE_DIAG
        if (r < 0) { dryHalted = g_DryIrq && halted; return false; }
#endif
        if (r <= 0) break;
        any = true;
    }
    return any;
}

bool Defer(melonDS::ARMv5* c)
{
    melonDS::NDS& nds = c->NDS;
    State* s = Active(c);
    if (!s || CheckPending || !(nds.CPUStop & 0xF)) return false;
    const u32 p = nds.IE[0] & nds.IF[0];
    if ((p & (0u - p)) != 0x200000) return false;       // GXFIFO the lowest pending IRQ
    if (!(c->A9HLEGuard ? s->dmaLive : !nds.IsJITEnabled() && s->dmaOk && (s->mask & 1024))) return false;
    s->calls[12]++;
    c->A9HLEDefer = true;
    return true;
}

int Deps(melonDS::NDS& nds, u32 addr, u32 instr, const Range*& r)
{
    if (instr == kCardInstr)
    {
        static thread_local Range card;
        card = {addr, addr + 9 * 4};
        r = &card;
        return 1;
    }
#ifdef LITEV_GX_BULK
    if (instr == kMatInstr && Active(&nds.ARM9) && addr == Active(&nds.ARM9)->v->shp)
    {
        // SHP chain, 10.'s code (+ MIi_FIFOCallback, OS_SetIrqFunction), the TWL check (wake range 9)
        const Variant& v = *Active(&nds.ARM9)->v;
        static thread_local Range sh[13];
        for (int i = 0; i < 3; i++) sh[i] = v.shpCode[i];
        for (int i = 0; i < 7; i++) sh[3 + i] = v.asyncCode[i];
        sh[10] = v.gxCode; sh[11] = v.code[3]; sh[12] = v.code[9];
        r = sh;
        return 13;
    }
    if (instr == kMatInstr)
    {
        static thread_local Range mat[3];
        u32 def, opn, snd;
        MatAt(&nds.ARM9, addr, def, opn, snd);      // (after IsHook != 0) a missing call target: an empty range
        mat[0] = {def, addr + 0xB0}; mat[1] = {opn, opn ? opn + 57 * 4 : 0}; mat[2] = {snd, snd ? snd + 24 : 0};
        r = mat;
        return 3;
    }
#endif
    if (instr == kVecInstr)
    {
        static thread_local Range vn;
        vn = {addr, addr + 69 * 4};
        r = &vn;
        return 1;
    }
    if (instr == kSdivInstr)
    {
        static thread_local Range sd;
        sd = {addr, addr + 108 * 4};
        r = &sd;
        return 1;
    }
    if (instr == kLzInstr)
    {
        static thread_local Range lz;
        lz = {addr - kLzHead, addr - kLzHead + 45 * 4};
        r = &lz;
        return 1;
    }
    const Variant& v = *Active(&nds.ARM9)->v;     // after IsHook != 0
    if (addr == v.wake)
    {
        static thread_local Range wk[kNumCode + 3];
        for (int i = 0; i < kNumCode; i++) wk[i] = v.code[i];
        for (int i = 0; i < 3; i++) wk[kNumCode + i] = v.dmaCode[i];
        r = wk;
        return kNumCode + 3;
    }
    if (addr == v.gx) { r = &v.gxCode; return 1; }
    if (v.matT && addr == v.matT->sbc) { r = v.matT->code; return 4; }
    if (v.shp && addr == v.shp)
    {
        static thread_local Range sh[13];
        for (int i = 0; i < 3; i++) sh[i] = v.shpCode[i];
        for (int i = 0; i < 7; i++) sh[3 + i] = v.asyncCode[i];
        sh[10] = v.gxCode; sh[11] = v.code[3]; sh[12] = v.code[9];
        r = sh;
        return 13;
    }
    if (addr == v.async)
    {
        static thread_local Range as[10];
        for (int i = 0; i < 7; i++) as[i] = v.asyncCode[i];
        as[7] = v.gxCode; as[8] = v.code[3]; as[9] = v.code[10];
        r = as;
        return 10;
    }
    r = &v.code[3];             // OS_SetIrqFunction / OS_GetIrqFunction
    return 1;
}

int IsHook(melonDS::NDS& nds, u32 addr, u32 instr, bool thumb)
{
    if (thumb)
    {
        if (!MaybeHookT(instr)) return 0;
        State& s = Get(&nds.ARM9, addr, instr & 0xFFFF, true);
        if (s.status != 1) return 0;
        const int k = Kind(*s.v, addr, instr & 0xFFFF, true);
        if (k < 0 || !(s.mask & Bit(k))) return 0;
        if (k == 5) return !s.gxOk ? 0 : GxIntact(&nds.ARM9, s) ? 1 : 2;
#ifdef LITEV_GX_BULK
        if (k == 8) return !s.matTOk ? 0 : MatTIntact(&nds.ARM9, s) ? 1 : 2;
        if (k == 10) return !s.gxOk || !s.asyncOk ? 0 : AsyncIntact(&nds.ARM9, s) && GxIntact(&nds.ARM9, s) && CodeIntact(&nds.ARM9, s, 0) ? 1 : 2;
        if (k == 13) return !s.shpOk || !(s.mask & 512) ? 0 : ShpIntact(&nds.ARM9, s) && AsyncIntact(&nds.ARM9, s) && GxIntact(&nds.ARM9, s)
                                                              && CodeIntact(&nds.ARM9, s, 0) ? 1 : 2;
#else
        if (k == 8 || k == 10 || k == 13) return 0;
#endif
        return CodeIntact(&nds.ARM9, s, k) ? 1 : 2;
    }
    if (!MaybeHook(instr)) return 0;
    if (instr == kCardInstr)
    {
        State& s = St(&nds.ARM9);
        return s.on && (s.mask & 32) && CardCodeAt(&nds.ARM9, addr) ? 1 : 0;
    }
#ifdef LITEV_GX_BULK
    if (instr == kMatInstr)
    {
        State& s = Get(&nds.ARM9, addr, instr);
        if (s.status == 1 && addr == s.v->shp)
            return !s.shpOk || !(s.mask & 2048) ? 0 : ShpIntact(&nds.ARM9, s) && AsyncIntact(&nds.ARM9, s) && GxIntact(&nds.ARM9, s)
                                                      && CodeIntact(&nds.ARM9, s, 0) ? 1 : 2;
        u32 def, opn, snd;
        return s.on && (s.mask & 128) ? MatAt(&nds.ARM9, addr, def, opn, snd) : 0;
    }
#endif
    if (instr == kVecInstr)
    {
        State& s = St(&nds.ARM9);
        if (!s.on || !(s.mask & 4096) || !VecAt(&nds.ARM9, addr, 3)) return 0;
        return VecAt(&nds.ARM9, addr, 69) ? 1 : 2;
    }
    if (instr == kSdivInstr)
    {
        State& s = St(&nds.ARM9);
        if (!s.on || !(s.mask & 256) || !CodeEq(&nds.ARM9, addr, kSdivCode, 2, nullptr, 0)) return 0;
        return CodeEq(&nds.ARM9, addr, kSdivCode, 108, kSdivSkip, 1) ? 1 : 2;
    }
    if (instr == kLzInstr)
    {
        // position independent: the code bytes around addr
        State& s = St(&nds.ARM9);
        return s.on && (s.mask & 64) && LzCodeAt(&nds.ARM9, addr) ? 1 : 0;
    }
    State& s = Get(&nds.ARM9, addr, instr);
    if (s.status != 1) return 0;
    const int k = Kind(*s.v, addr, instr);
    if (k < 0 || !(s.mask & Bit(k))) return 0;
    if (k == 5) return !s.gxOk ? 0 : GxIntact(&nds.ARM9, s) ? 1 : 2;
#ifdef LITEV_GX_BULK
    if (k == 10) return !s.gxOk || !s.asyncOk ? 0 : AsyncIntact(&nds.ARM9, s) && GxIntact(&nds.ARM9, s) && CodeIntact(&nds.ARM9, s, 1) ? 1 : 2;
#else
    if (k == 10) return 0;
#endif
    return CodeIntact(&nds.ARM9, s, k) ? 1 : 2;
}

namespace
{
#ifdef LITEV_GX_BULK
// 5.: a prefix: the native part, then the guest function from its first instruction
__attribute__((noinline)) bool RunGx(melonDS::ARMv5* c, State& s, bool jit)
{
    if (!s.gxOk) return false;
#ifdef LITEV_A9HLE_GXCHECK
    if (g_Check) GxCheckAt(s, c);
#endif
    u32 src, len, n;
    if ((jit || GxIntact(c, s)) && GxPlan(c, s, src, len, n))
    {
        if (g_Check)
        {
#ifdef LITEV_A9HLE_GXCHECK
            if (!g_Gx.on)
            {
                g_Gx.on = true; g_Gx.src = src; g_Gx.len = len; g_Gx.n = n; g_Gx.w.resize(n);
                g_Gx.t0 = c->NDS.ARM9Timestamp + c->Cycles;
                for (u32 i = 0; i < n; i++) g_Gx.w[i] = R32(&c->NDS.MainRAM[(src + i * 4) & c->NDS.MainRAMMask]);
                g_GxTapBuf.clear(); GxOtherSeen = false; GxTap = &g_GxTapBuf;
                s.checks[5]++;
            }
#endif
        }
        else if (g_Dry) s.checks[5]++;
        else
        {
            const u64 t0 = g_Time ? Now() : 0;
            s.gxWords += GxSend(c, s, src, len, n);
            s.native[5]++;
            if (g_Time) s.ns += Now() - t0;
        }
    }
    else s.fallback[5]++;
    GuestFallback(c);
    return true;
}
// ---- 10. MI_SendGXCommandAsync: its synchronous part natively -------------------------------------
// NNS G3D sends every shape's display list with MI_SendGXCommandAsync (55 lists a frame in the PW town):
// wait for the previous list and the DMA, save the GXFIFO IRQ mode / handler, GXFIFO IRQ mode "less
// than half", OS_SetIrqFunction(GXFIFO, MIi_FIFOCallback), OS_EnableIrqMask(GXFIFO), MIi_FIFOCallback
// (with 5.: every chunk but the last at once; the last one as a DMA with its IRQ handler
// MIi_DMACallback: OSi_EnterDmaCallback, MIi_DmaSetParams), OS_ResetRequestIrqMask(GXFIFO), IRQs
// restored: ~210 guest instructions and 3 hook exits (Get, Set, 5.) a list. Natively at its entry
// when the previous list is done, the FIFO empty and the DMA idle: the same IO writes in the same
// order (GXSTAT, IE, the bulk chunks, IE, the DMA registers, IF; the IME toggles around them are
// left out: IRQs are off and IME ends unchanged), every memory byte (MIi_GXDmaParams, OS_IRQTable,
// the DMA IRQ table, the stack frames) and register / flag the guest leaves. The DMA completion
// (IRQ -> MIi_DMACallback) stays guest code. Category B: a fixed cycle estimate (guest average in
// check mode) plus 5.'s. If the bulk send stops early (a SWAP_BUFFERS in the list: the FIFO is no
// longer empty), the state is the guest's at MIi_FIFOCallback's entry and the guest continues there.
// ponytail: fixed estimate (single-chunk lists, check mode without IRQs, entry to OS_RestoreInterrupts)
constexpr s32 kAsyncCyc = 1500;   // (guest: 1510 + 4 per DMA word, exact over 4.7k lists; the DMA charges its own)
constexpr s32 kAsyncCycT = 1351;  // TWL SDK build (W2): guest 1351 + 4 per DMA word (check mode, 6.6k lists, within 0.2%)

// The synchronous part from the register file R / cpsr at MI_SendGXCommandAsync's entry (arg: its 5th argument, the
// word at [sp]). 0: guest path (nothing done); 1: done, e = the state at its return (IO written, memory queued in m;
// check / dry (ck): nothing written, ck = the compare point at the OS_RestoreInterrupts call); 2: handed over at
// MIi_FIFOCallback's entry (registers set, memory flushed, c jumps there).
struct AsyncChk { Expect e2; std::vector<std::pair<u32, u32>> io; u32 words = 0; bool fin = false; };
__attribute__((noinline)) int AsyncCoreT(melonDS::ARMv5* c, State& s, Mem& m, const u32* R, u32 cpsr, u32 arg, bool jit, Expect& e, AsyncChk* ck);
int AsyncCore(melonDS::ARMv5* c, State& s, Mem& m, const u32* R, u32 cpsr, u32 arg, bool jit, Expect& e, AsyncChk* ck)
{
    if (s.v->twl) return AsyncCoreT(c, s, m, R, cpsr, arg, jit, e, ck);
    melonDS::NDS& nds = c->NDS;
    const Variant& v = *s.v;
    const u32 pc = v.async;
    const u32 dma = R[0], src = R[1], len = R[2], cb = R[3], sp = R[13];
    Obj P, st;
    u32 gxstat = 0, n = 0;
    bool ok = s.gxOk && s.asyncOk && (jit || (AsyncIntact(c, s) && GxIntact(c, s) && CodeIntact(c, s, 1)))
              && dma >= 1 && dma <= 3 && len && Fixed(c, s, m) && s.ftab && s.ftab2;
    if (ok)
    {
        // the guest waits for the previous list (MIi_GXDmaParams.busy), the DMA (MI_WaitDma) and an empty FIFO
        P = m.O(v.gxParams, 0x20); st = m.O(sp - 80, 80);
        ok = P && !P.dtcm && st && !P.r(0) && !(nds.A9HLEDmaCnt(dma) & 0x80000000);
        if (ok) { gxstat = nds.ARM9Read32(0x04000600); ok = gxstat & (1u << 25); }
    }
    if (ok && len > kGxChunk)
    {
        // what 5. sends at once at MIi_FIFOCallback's entry
        melonDS::GPU3D& gx = nds.GPU.GPU3D;
        ok = (s.mask & 16) && !ck && len <= 0x100000 && !(len & 3) && !(src & 3) && (src >> 24) == 0x02
             && ((src + len - 1) >> 24) == 0x02 && gx.GeometryEnabled && gx.BulkReady();
        n = ((len - 1) / kGxChunk) * (kGxChunk / 4);
    }
    if (!ok) return 0;
    const u32 params = v.gxParams, oldI = cpsr & 0x80, ie0 = nds.IE[0], ie1 = ie0 | 0x200000, dbit = 1u << (8 + dma);
    const u32 fifoCb = R32(CodePtr(c, v.async + 0xE8)), dmaCb = R32(CodePtr(c, v.gx + 0xA8));
    const Obj tab{s.ftab, kIrqTable, s.ftabD}, tab2{s.ftab2, v.irqTable2, s.ftab2D};
    const u32 gxw = (gxstat & ~0xC0000000u) | 0x40000000u;
    u32 done = 0;
    if (!ck)
    {
        nds.ARM9Write32(0x04000600, gxw);
        nds.ARM9Write32(0x04000210, ie1);
        if (n) done = GxSend(c, s, src, len, n);
    }
    // MIi_GXDmaParams {busy, dma, src, length, callback, arg, GXFIFO IRQ mode, GXFIFO IRQ handler}, OS_IRQTable[GXFIFO]
    m.W(P, 0, 1); m.W(P, 4, dma); m.W(P, 0x10, cb); m.W(P, 0x14, arg); m.W(P, 0x18, gxstat >> 30); m.W(P, 0x1C, R32(s.ftab + 21 * 4));
    m.W(tab, 21 * 4, fifoCb);
    m.W(st, 56, R[3]); m.W(st, 60, R[4]); m.W(st, 64, R[5]); m.W(st, 68, R[6]); m.W(st, 72, R[7]); m.W(st, 76, R[14]);
    if (done < n)
    {
        // the FIFO stopped being empty: hand over at MIi_FIFOCallback's entry (5. found it busy: the guest goes on)
        const u32 set[8] = {oldI, 0x04000600, params, 0x200000, R[8], R[9], R[10], pc + 0xC8};   // OS_SetIrqFunction's push
        for (int i = 0; i < 8; i++) m.W(st, 24 + i * 4, set[i]);
        m.Flush();
        for (int i = 0; i < 16; i++) c->R[i] = R[i];
        c->R[0] = c->R[1] = ie0; c->R[2] = nds.IME[0] & 0xFFFF; c->R[3] = 0x04000208; c->R[4] = oldI; c->R[5] = 0x04000600; c->R[6] = params; c->R[7] = 0x200000;
        c->R[12] = 0x20; c->R[13] = sp - 24; c->R[14] = pc + 0xD4;
        c->CPSR = (cpsr & 0x0FFFFFFF) | 0x60000000 | 0x80;
        c->Cycles += 700;
        c->JumpTo(v.gx);
        return 2;
    }
    const u32 chunk = len - n * 4, srcL = src + n * 4, ctrl = 0xC4400000u | (chunk >> 2);
    m.W(P, 8, src + len); m.W(P, 0xC, 0);
    m.W(tab2, dma * 12, dmaCb); m.W(tab2, dma * 12 + 8, 0); m.W(tab2, dma * 12 + 4, ie1 & dbit);
    m.W(st, 32, 0);     // MIi_FIFOCallback: str r7, [sp] over its push of r3
    const u32 fc[5] = {oldI, 0x04000600, params, 0x200000, pc + 0xD4};
    for (int i = 0; i < 5; i++) m.W(st, 36 + i * 4, fc[i]);
    const u32 ds[8] = {ctrl, srcL, chunk, params, 0, R[8], R[9], v.gx + 0x6C};   // MIi_DmaSetParams' push
    for (int i = 0; i < 8; i++) m.W(st, i * 4, ds[i]);
    const u32 sad = 0x040000B0 + dma * 12;
    for (int i = 0; i < 16; i++) e.R[i] = R[i];
    const u32 cpsrI = (cpsr & 0x0FFFFFFF) | 0x60000000 | 0x80;
    e.R[0] = 0x80; e.R[1] = cpsrI; e.R[2] = (cpsrI & ~0x80u) | oldI; e.R[3] = 0; e.R[12] = v.irqTable2;   // r3: MIi_FIFOCallback pops the 0 it stored over its r3
    e.R[14] = pc + 0xDC;    // lr: the bl OS_RestoreInterrupts
    e.retPc = R[14];
    e.CPSR = (cpsr & 0x0FFFFFDF) | 0x60000000 | ((e.retPc & 1) << 5);
    if (ck)
    {
        // compared where the guest calls OS_RestoreInterrupts at the end (the DMA's IRQ is taken right after it;
        // natively at the return instead): MI_SendGXCommandAsync's frame and registers, IRQs still off,
        // r1 = the IF value OS_ResetRequestIrqMask read (not compared)
        Expect& e2 = ck->e2;
        e2 = e;
        e2.R[0] = oldI; e2.R[2] = nds.IME[0] & 0xFFFF; e2.R[4] = oldI; e2.R[5] = 0x04000600; e2.R[6] = params; e2.R[7] = 0x200000;
        e2.R[13] = sp - 24; e2.R[14] = v.gx + 0x74;
        e2.retPc = pc + 0xD8; e2.CPSR = cpsrI & ~0x20u;
        ck->words = chunk >> 2;
        ck->io = {{0x04000600, gxw & 0xC0008000u}, {0x04000210, ie1}, {0x04000210, ie1 | dbit}, {sad, srcL}, {sad + 4, 0x04000400},
                  {sad + 8, ctrl}, {0x04000214, 0x200000}};
        return 1;
    }
    nds.ARM9Write32(0x04000210, ie1 | dbit);
    nds.ARM9Write32(sad, srcL); nds.ARM9Write32(sad + 4, 0x04000400); nds.ARM9Write32(sad + 8, ctrl);
    nds.ARM9Write32(0x04000214, 0x200000);
    return 1;
}

// 10. for the TWL SDK build (W2): MI_SendGXCommandAsync and its callees in Thumb (MIi_DmaSetParams in ITCM, with its own
// OS_Disable/RestoreInterrupts): the same IO writes in the same order as 10., this build's frames and registers.
// Stage 1 compares at its OS_RestoreInterrupts call (r1 = the IF value read, not modelled), stage 2 the registers at
// its return. No hand-over path: a list whose bulk send stops early runs the guest code.
__attribute__((noinline)) int AsyncCoreT(melonDS::ARMv5* c, State& s, Mem& m, const u32* R, u32 cpsr, u32 arg, bool jit, Expect& e, AsyncChk* ck)
{
    melonDS::NDS& nds = c->NDS;
    const Variant& v = *s.v;
    const u32 pc = v.async;
    const u32 dma = R[0], src = R[1], len = R[2], cb = R[3], S = R[13];
    Obj P, st;
    u32 gxstat = 0, n = 0;
    bool ok = s.gxOk && s.asyncOk && (jit || (AsyncIntact(c, s) && GxIntact(c, s) && CodeIntact(c, s, 0)))
              && dma >= 1 && dma <= 3 && len && Fixed(c, s, m) && s.ftab && s.ftab2;
    if (ok)
    {
        P = m.O(v.gxParams, 0x20); st = m.O(S - 80, 80);
        ok = P && !P.dtcm && st && !P.r(0) && !(nds.A9HLEDmaCnt(dma) & 0x80000000);
        if (ok) { gxstat = nds.ARM9Read32(0x04000600); ok = gxstat & (1u << 25); }
    }
    if (ok && len > kGxChunk)
    {
        // what 5. sends at once at MIi_FIFOCallback's entry (80% of the W2 town's lists)
        melonDS::GPU3D& gx = nds.GPU.GPU3D;
        ok = (s.mask & 16) && !ck && len <= 0x100000 && !(len & 3) && !(src & 3) && (src >> 24) == 0x02
             && ((src + len - 1) >> 24) == 0x02 && gx.GeometryEnabled && gx.BulkReady();
        n = ((len - 1) / kGxChunk) * (kGxChunk / 4);
    }
    if (!ok) return 0;
    const u32 params = v.gxParams, oldI = cpsr & 0x80, ie0 = nds.IE[0], ie1 = ie0 | 0x200000, dbit = 1u << (8 + dma);
    const u32 fifoCb = R32(CodePtr(c, pc + 0xA4)), dmaCb = R32(CodePtr(c, v.gx + 0x70));     // literals 0x020784FD, 0x0207857D
    const Obj tab{s.ftab, kIrqTable, s.ftabD}, tab2{s.ftab2, v.irqTable2, s.ftab2D};
    const u32 gxw = (gxstat & 0x3FFFFFFFu) | 0x40000000u;
    u32 done = 0;
    if (!ck)
    {
        nds.ARM9Write32(0x04000600, gxw);
        nds.ARM9Write32(0x04000210, ie1);
        if (n) done = GxSend(c, s, src, len, n);    // (writes MIi_GXDmaParams.src / length)
    }
    const u32 chunk = len - n * 4, srcL = src + n * 4, words = chunk >> 2, ctrl = 0xC4400000u | words;
    // MIi_GXDmaParams {busy, dma, src, length, callback, arg, GXFIFO IRQ mode, GXFIFO IRQ handler}, OS_IRQTable[GXFIFO]
    m.W(P, 0, 1); m.W(P, 4, dma); m.W(P, 0x10, cb); m.W(P, 0x14, arg);
    m.W(P, 0x18, gxstat >> 30); m.W(P, 0x1C, R32(s.ftab + 21 * 4));
    m.W(tab, 21 * 4, fifoCb);
    m.W(tab2, dma * 12, dmaCb); m.W(tab2, dma * 12 + 8, 0); m.W(tab2, dma * 12 + 4, ie1 & dbit);
    // stack (st = S-80), in write order: async push {r3-r7, lr} (S-24); the DMA range check push {r4, lr} (S-32);
    // MI_WaitDma push {r3-r5, lr} (S-40); OS_GetIrqFunction push {r3, r4} (S-32); OS_SetIrqFunction push {r4-r7} (S-40);
    // OS_EnableIrqMask push {r3, r4} (S-32); MIi_FIFOCallback push {r3-r7, lr} (S-48); OSi_EnterDmaCallback push
    // {r3-r5, lr} (S-64); OS_EnableIrqMask push {r3, r4} (S-72); MIi_FIFOCallback str r7, [sp] (S-48);
    // MIi_DmaSetParams push {r3-r7, lr} (S-72) + locals {r2, r3} (S-80); OS_ResetRequestIrqMask push {r3, r4} (S-56)
    struct { u32 o, v; } w[] = {
        {56, R[3]}, {60, R[4]}, {64, R[5]}, {68, R[6]}, {72, R[7]}, {76, R[14]},
        {48, 0x04000600}, {52, pc + 0x4D},
        {40, 0}, {44, 0x04000600}, {48, dma}, {52, pc + 0x53},
        {48, 0}, {52, 0x04000600},
        {40, 0x04000600}, {44, 0x200000}, {48, params}, {52, oldI},
        {48, 0x20}, {52, 0x04000600},
        {32, 0x20}, {36, 0x04000600}, {40, 0x200000}, {44, params}, {48, oldI}, {52, pc + 0x8F},
        {16, 0x20}, {20, srcL}, {24, chunk}, {28, v.gx + 0x33},
        {8, 12}, {12, dbit},
        {32, 0},
        {8, ctrl}, {12, 0x04000400}, {16, words}, {20, params}, {24, 0}, {28, v.gx + 0x47}, {0, 0x04000400}, {4, ctrl},
        {24, ctrl}, {28, 0x04000400},
    };
    if (done < n)
    {
        // the FIFO stopped being empty (a SWAP_BUFFERS in the list): hand over at MIi_FIFOCallback's (Thumb) entry with
        // the frames written so far (up to OS_EnableIrqMask's) and the guest's registers there; 5. finds it busy
        for (u32 i = 0; i < 20; i++) m.W(st, w[i].o, w[i].v);
        m.Flush();
        for (int i = 0; i < 16; i++) c->R[i] = R[i];
        c->R[0] = c->R[1] = ie0; c->R[2] = 0x04000210; c->R[3] = 0x20; c->R[4] = 0x04000600; c->R[5] = 0x200000; c->R[6] = params; c->R[7] = oldI;
        c->R[13] = S - 24; c->R[14] = pc + 0x8F;
        c->CPSR = (cpsr & 0x0FFFFFFF) | (ie0 & 0x80000000) | (ie0 ? 0 : 0x40000000u) | 0x80;    // adds r0, r1, #0 (old IE)
        c->Cycles += 700;
        c->JumpTo(v.gx | 1);
        return 2;
    }
    m.W(P, 8, src + len); m.W(P, 0xC, 0);
    for (auto& x : w) m.W(st, x.o, x.v);
    const u32 sad = 0x040000B0 + dma * 12;
    for (int i = 0; i < 16; i++) e.R[i] = R[i];
    // flags at the end: MIi_FIFOCallback's caller does adds r0, r7, #0 (r7 = the old I bit) before OS_RestoreInterrupts
    // (OS_RestoreInterrupts is ARM code: the CPSR it reads has T clear)
    const u32 fl = oldI ? 0 : 0x40000000u, cpsrI = (cpsr & 0x0FFFFFDF) | fl | 0x80;
    e.R[0] = 0x80; e.R[1] = cpsrI; e.R[2] = (cpsrI & ~0x80u) | oldI; e.R[3] = R[3]; e.R[12] = 0x80;
    e.R[14] = pc + 0x95;     // lr: the blx OS_RestoreInterrupts
    e.retPc = R[14];
    e.CPSR = (cpsr & 0x0FFFFFDF) | fl | ((e.retPc & 1) << 5);
    if (ck)
    {
        Expect& e2 = ck->e2;
        e2 = e;
        e2.R[0] = oldI; e2.R[2] = 0x04000214; e2.R[3] = 0; e2.R[4] = 0x04000600; e2.R[5] = 0x200000; e2.R[6] = params; e2.R[7] = oldI;
        e2.R[13] = S - 24; e2.R[14] = v.gx + 0x4D;     // lr: MIi_FIFOCallback's bl OS_ResetRequestIrqMask
        e2.retPc = pc + 0x90; e2.CPSR = cpsrI | 0x20;
        ck->words = words;
        ck->io = {{0x04000600, gxw & 0xC0008000u}, {0x04000210, ie1}, {0x04000210, ie1 | dbit}, {sad, srcL}, {sad + 4, 0x04000400},
                  {sad + 8, ctrl}, {0x04000214, 0x200000}};
        ck->fin = true;
        return 1;
    }
    nds.ARM9Write32(0x04000210, ie1 | dbit);
    nds.ARM9Write32(sad, srcL); nds.ARM9Write32(sad + 4, 0x04000400); nds.ARM9Write32(sad + 8, ctrl);
    nds.ARM9Write32(0x04000214, 0x200000);
    return 1;
}

__attribute__((noinline)) bool RunAsync(melonDS::ARMv5* c, State& s, bool jit)
{
    const bool chk = g_Check || g_Dry;
    Mem m(c, chk);
    Expect e;
    AsyncChk ck;
    Obj a = CheckPending ? Obj{} : m.O(c->R[13], 4);
    const int r = a ? AsyncCore(c, s, m, c->R, c->CPSR, a.r(0), jit, e, chk ? &ck : nullptr) : 0;
    if (r == 2) { s.native[10]++; return true; }
    if (r == 1 && chk)
    {
#ifdef LITEV_HLE_DIAG
        m.Flush();      // logs only
        if (g_Check)
        {
            ArmCheck(c, 10, m, ck.e2); g_P.fit[0] = ck.words; g_P.io = ck.io;
            g_P.small = !ck.fin;        // stage 2 (registers at the return, the DMA number word) for the Thumb build
            g_P.fin = e; g_P.finA = s.v->gxParams + 4; g_P.finV = c->R[0];
        }
#endif
        s.checks[10]++;
        GuestFallback(c);
        return true;
    }
    if (!r)
    {
        s.fallback[10] += !g_Check;
        GuestFallback(c);
        return true;
    }
    m.Flush();
    s.native[10]++;
    Return(c, e, s.v->twl ? kAsyncCycT : kAsyncCyc);
    return true;
}

// ---- 13. NNS G3D shape: SBC SHP -> NNSi_G3dFuncSbc_SHP_InternalDefault -> NNS_G3dGeSendDL -> 10. ---------------
// The SBC SHP command (PW 0x0206C03C, 81 calls a frame in the town) looks up the shape, calls the default shape
// function (no render callback), which sends the shape's display list with NNS_G3dGeSendDL: the GE buffer flushed,
// the "sending" flag set, the TWL check, and for a list of >= 0x100 bytes MI_SendGXCommandAsync on the configured
// DMA with the clear callback. ~85 guest instructions and a hook exit before 10. Natively at SHP's entry for that
// path: the 4 stack frames, the sending flag, 10.'s work from the register file the guest would have there, the
// epilogue (registers / flags popped, the SBC pointer + 2). Anything else (callbacks, a busy or buffered GE, a small
// list: NNS_G3dGeBufferOP_N) runs the guest code (10. still applies). Category B: 10.'s estimate + the prefix's.
// ponytail: SHP + SHP_InternalDefault + NNS_G3dGeSendDL around 10.: 224 cycles before it (check mode, the same for all
// 8.7k lists) + ~21 for the pops after it
constexpr s32 kShpCyc = 245;
// small lists (NNS_G3dGeBufferOP_N): base + per parameter word (check mode fit over 6.5k lists, within 1%)
constexpr s32 kShpSmallCyc0 = 213, kShpSmallCycWord = 12;
// ponytail: a skipped shape (push, 3 tests, pointer update, pop: ~10 instructions; W2 the same estimate)
constexpr s32 kShpSkipCyc = 24;   // (check mode: 24 over 3.4k skipped shapes)

// 13.: a skipped shape natively (or, check mode, compared at SHP's return)
bool ShpSkip(melonDS::ARMv5* c, State& s, Mem& m, Expect& e, bool chk)
{
#ifdef LITEV_HLE_DIAG
    if (chk)
    {
        m.Flush();      // logs only
        if (g_Check)
        {
            ArmCheck(c, 13, m, e);
            g_P.small = true; g_P.gx.clear(); g_P.fit[0] = 0;
#ifdef LITEV_A9HLE_GXCHECK
            if (!GxTap) { g_GxMatTap.clear(); GxTap = &g_GxMatTap; g_P.gxOn = true; }
#endif
        }
        s.checks[13]++;
        GuestFallback(c);
        return true;
    }
#endif
    (void)chk;
    m.Flush();
    s.native[13]++;
    Return(c, e, kShpSkipCyc);
    return true;
}

__attribute__((noinline)) bool RunShp(melonDS::ARMv5* c, State& s, bool jit)
{
    const Variant& v = *s.v;
    s.calls[13]++;
    const bool chk = g_Check || g_Dry;
    Mem m(c, chk);
    Expect e;
    AsyncChk ck;
    bool bad = false;
    auto rd = [&](u32 a, u32 n) -> u32 {
        const u8* p = (a & (n - 1)) ? nullptr : m.P(a);
        if (!p || m.P(a + n - 1) != p + n - 1) { bad = true; return 0; }
        return n == 4 ? R32(p) : n == 2 ? R16(p) : *p;
    };
    auto lit = [&](u32 a) { return R32(CodePtr(c, a)); };
    const u32 rs = c->R[0], sp = c->R[13], shpDef = v.shp - 0xD4, flush = v.shpCode[1].a, send = flush + 0x80;
    int r = 0;
    Obj RS, fr;
    u32 sbc = 0, r6 = 0, G = 0;
    bool ok = !CheckPending && s.shpOk && (s.mask & 512) && (jit || (ShpIntact(c, s) && CodeIntact(c, s, 0)));
    if (ok)
    {
        RS = m.O(rs, 0xE0); fr = m.O(sp - 48, 48);
        ok = RS && fr;
    }
    u32 R[16];
    if (ok)
    {
        // SBC SHP: the shape of index sbc[1] in the shape resource, its function from the table
        const u32 flag = RS.r(8);
        sbc = RS.r(0);
        if ((flag & 0x202) || !(flag & 1))
        {
            // the shape is skipped: SBC pointer + 2 (push {r4, lr}; flags of the tst that branched: N clear, Z set
            // only for the "visible" bit 0 test, C clear (tst #0x200 ran first: a rotated immediate), V unchanged)
            Obj st = m.O(sp - 8, 8);
            if (!st) goto out;
            m.W(st, 0, c->R[4]); m.W(st, 4, c->R[14]);
            m.W(RS, 0, sbc + 2);
            for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
            e.R[0] = sbc + 2;
            e.retPc = c->R[14];
            e.CPSR = (c->CPSR & 0x1FFFFFDF) | (!(flag & 0x200) && !(flag & 1) ? 0x40000000u : 0) | ((e.retPc & 1) << 5);
            return ShpSkip(c, s, m, e, chk);
        }
        const u32 idx = rd(sbc + 1, 1), ip = RS.r(0xDC);
        ok = ip && !bad;
        u32 shp = 0, ent = 0;
        if (ok)
        {
            const u32 cnt = rd(ip + 1, 1), ofs = rd(ip + 6, 2);
            ent = ip + ofs + 4 + rd(ip + ofs, 2) * idx;
            ok = idx < cnt && !bad && ent;
            if (ok) { shp = ip + rd(ent, 4); ok = !bad && rd(lit(v.shp + 0x9C) + rd(shp, 2) * 4, 4) == shpDef && !bad; }
        }
        // SHP_InternalDefault: no render callback at any timing ([rs + 0x91] 1/2/3 with a callback), no "no send" flag
        const u32 cbf = RS.r(0x20);
        r6 = cbf ? RS.p[0x91] : 0;
        ok = ok && (r6 < 1 || r6 > 3);
        u32 dl = 0, size = 0, dma = 0;
        if (ok)
        {
            dl = shp + rd(shp + 8, 4); size = rd(shp + 0xC, 4);
            dma = rd(lit(send + 0x194), 4);
            G = lit(send + 0x198);
        }
        if (ok && (size < 0x100 || dma == ~0u))
        {
            // the small path: NNS_G3dGeBufferOP_N(list[0], list + 1, size / 4 - 1) with no GE buffer and nothing being
            // sent: the command word to GXFIFO, MI_CpuSend32 of the rest = the whole list
            melonDS::GPU3D& gx = c->NDS.GPU.GPU3D;
            const u8* lp = m.P(dl);
            const u32 gb = rd(G, 4);   // OP_N: an empty GE buffer sends directly too
            ok = !bad && size >= 4 && !(size & 3) && (!gb || !rd(gb, 4)) && !rd(G + 4, 4) && !bad && lp && (dl >> 24) == 0x02
                 && m.P(dl + size - 1) == lp + size - 1 && (dl & c->DTCMMask) != c->DTCMBase && ((dl + size - 1) & c->DTCMMask) != c->DTCMBase
                 && (chk || (gx.GeometryEnabled && gx.BulkReady()));
            fr = m.O(sp - 56, 56);
            ok = ok && fr;
            if (!ok) goto out;
            const u32 n = size / 4 - 1, opn = BlTarget(c, send + 0x38);
            // frames (fr = sp - 56): OP_N push {r4-r6, lr} | GeSendDL push {r3-r5, lr} | SHP_InternalDefault | SHP
            const u32 w[14] = {size, dl, r6, send + 0x3C, idx, shp, rs, shpDef + 0x80, rs, c->R[5], c->R[6], v.shp + 0x8C, c->R[4], c->R[14]};
            for (int i = 0; i < 14; i++) m.W(fr, i * 4, w[i]);
            for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
            e.R[0] = sbc + 2; e.R[1] = 0x04000400; e.R[2] = n ? R32(lp + size - 4) : 0; e.R[3] = idx; e.R[12] = dl + size;
            e.R[14] = opn + 0xD8;
            e.retPc = c->R[14];
            e.CPSR = (c->CPSR & 0x0FFFFFDF) | Flags(r6, 3) | ((e.retPc & 1) << 5);
#ifdef LITEV_HLE_DIAG
            if (chk)
            {
                m.Flush();      // logs only
                if (g_Check)
                {
                    m.log[m.n++] = {rs, sbc + 2, 4};
                    ArmCheck(c, 13, m, e);
                    g_P.small = true;
                    g_P.gx.assign((const u32*)lp, (const u32*)(lp + size));
#ifdef LITEV_A9HLE_GXCHECK
                    if (!GxTap) { g_GxMatTap.clear(); GxTap = &g_GxMatTap; g_P.gxOn = true; }
#endif
                    g_P.fit[0] = size / 4;
                }
                s.checks[13]++;
                GuestFallback(c);
                return true;
            }
#endif
            m.W(RS, 0, sbc + 2);
            m.Flush();
            gx.BulkWords((const u32*)lp, size / 4);
            s.native[13]++;
            Return(c, e, kShpSmallCyc0 + kShpSmallCycWord * (s32)n);
            return true;
        }
        if (ok)
        {
            // NNS_G3dGeSendDL: a list of >= 0x100 bytes on a DMA (1-3), nothing being sent, the GE buffer empty,
            // no TWL path (initialized, not TWL mode)
            const u32 T = lit(v.code[9].a + 0x44), gb = rd(G, 4);
            ok = !bad && size >= 0x100 && dma >= 1 && dma <= 3 && !rd(G + 4, 4) && !rd(G + 8, 4) && (!gb || !rd(gb, 4))
                 && rd(T + 0x1C, 4) && !rd(T + 4, 4) && !bad && lit(send + 0x1A0) == flush + 0x74;
        }
        if (ok)
        {
            // frames (fr = sp - 48): flush push {r3, lr} | GeSendDL push {r3-r5, lr} (r3's slot: the async arg) |
            // SHP_InternalDefault push {r4-r6, lr} | SHP push {r4, lr}
            const u32 w[12] = {idx, send + 0x44, G + 4, shp, rs, shpDef + 0x80, rs, c->R[5], c->R[6], v.shp + 0x8C, c->R[4], c->R[14]};
            for (int i = 0; i < 12; i++) m.W(fr, i * 4, w[i]);
            const Obj g = m.O(G + 4, 4);      // (read above: valid)
            m.W(g, 0, 1);
            for (int i = 0; i < 16; i++) R[i] = c->R[i];
            R[0] = dma; R[1] = dl; R[2] = size; R[3] = flush + 0x74; R[4] = size; R[5] = dl; R[6] = r6;
            R[12] = G + 4; R[13] = sp - 40; R[14] = send + 0x190;
            r = AsyncCore(c, s, m, R, c->CPSR, G + 4, jit, e, chk ? &ck : nullptr);
        }
    }
out:
    if (r == 2) { s.native[13]++; return true; }
    if (r == 1)
    {
        // epilogue: GeSendDL pop {r3-r5, pc}, SHP_InternalDefault cmp r6, #2 / #3, pop {r4-r6, pc}, SHP [rs] += 2, pop {r4, pc}
        e.R[0] = sbc + 2; e.R[3] = G + 4; e.R[4] = c->R[4]; e.R[5] = c->R[5]; e.R[6] = c->R[6]; e.R[13] = sp;
        e.retPc = c->R[14];
        e.CPSR = (c->CPSR & 0x0FFFFFDF) | Flags(r6, 3) | ((e.retPc & 1) << 5);
    }
    if (r == 1 && chk)
    {
#ifdef LITEV_HLE_DIAG
        m.Flush();      // logs only
        if (g_Check)
        {
            ArmCheck(c, 13, m, ck.e2);
            g_P.small = false;
            g_P.fit[0] = ck.words; g_P.io = ck.io;
            g_P.fin = e; g_P.finA = rs; g_P.finV = sbc + 2;
        }
#endif
        s.checks[13]++;
        GuestFallback(c);
        return true;
    }
    if (!r)
    {
        s.fallback[13] += !g_Check;
        GuestFallback(c);
        return true;
    }
    m.W(RS, 0, sbc + 2);
    m.Flush();
    s.native[13]++;
    Return(c, e, kAsyncCyc + kShpCyc);
    return true;
}

// 13. for the TWL SDK build (W2): the same chain in Thumb (SBC SHP 0x020669A8 -> SHP_InternalDefault -> NNS_G3dGeSendDL
// -> 10. or NNS_G3dGeBufferOP_N), this build's frames, registers and the flags of SBC SHP's adds; 154 shapes a frame in
// the W2 town. Category B as 13.
// ponytail: fixed estimates (check mode, two W2 towns): 205 before 10. (80% of 6.3k lists; 208-210 the rest) + ~21 for
// the pops; small lists 212 + 12 per parameter word (18k lists)
constexpr s32 kShpCycT = 226, kShpSmallCycT0 = 212, kShpSmallCycWordT = 12;

__attribute__((noinline)) bool RunShpT(melonDS::ARMv5* c, State& s, bool jit)
{
    const Variant& v = *s.v;
    const bool chk = g_Check || g_Dry;
    Mem m(c, chk);
    Expect e;
    AsyncChk ck;
    bool bad = false;
    auto rd = [&](u32 a, u32 n) -> u32 {
        const u8* p = (a & (n - 1)) ? nullptr : m.P(a);
        if (!p || m.P(a + n - 1) != p + n - 1) { bad = true; return 0; }
        return n == 4 ? R32(p) : n == 2 ? R16(p) : *p;
    };
    auto lit = [&](u32 a) { return R32(CodePtr(c, a)); };
    const u32 rs = c->R[0], S = c->R[13], shpDef = v.shpCode[0].a, flush = v.shpCode[1].a, send = flush + 0x50, opn = send + 0x130;
    int r = 0;
    Obj RS, fr;
    u32 sbc = 0, G = 0;
    u32 R[16];
    bool ok = !CheckPending && s.shpOk && (s.mask & 512) && (jit || (ShpIntact(c, s) && CodeIntact(c, s, 0)));
    if (ok)
    {
        RS = m.O(rs, 0xE0); fr = m.O(S - 72, 72);
        ok = RS && fr;
    }
    if (ok)
    {
        const u32 flag = RS.r(8);
        sbc = RS.r(0);
        if ((flag & 0x202) || !(flag & 1))
        {
            // skipped: SBC pointer + 2; r2 = the last movs (0x200 / 1 / 2), flags of SBC SHP's adds
            m.W(fr, 56, c->R[4]); m.W(fr, 60, c->R[5]); m.W(fr, 64, c->R[6]); m.W(fr, 68, c->R[14]);
            m.W(RS, 0, sbc + 2);
            for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
            e.R[0] = sbc + 2; e.R[2] = (flag & 0x200) ? 0x200 : !(flag & 1) ? 1 : 2;
            e.retPc = c->R[14];
            const u32 rr = sbc + 2, nzcv = (rr & 0x80000000) | ((rr == 0) << 30) | ((rr < sbc) << 29) | (((~(sbc ^ 2u) & (sbc ^ rr)) >> 31) << 28);
            e.CPSR = (c->CPSR & 0x0FFFFFDF) | nzcv | ((e.retPc & 1) << 5);
            return ShpSkip(c, s, m, e, chk);
        }
        const u32 idx = rd(sbc + 1, 1), ip = RS.r(0xDC);
        ok = ip && !bad;
        u32 shp = 0, fn = 0, h = 0;
        if (ok)
        {
            const u32 cnt = rd(ip + 1, 1), ofs = rd(ip + 6, 2);
            const u32 ent = ip + ofs + 4 + rd(ip + ofs, 2) * idx;
            ok = idx < cnt && !bad && ent;
            if (ok) { shp = ip + rd(ent, 4); h = rd(shp, 2); fn = rd(lit(v.shp + 0x64) + h * 4, 4); ok = !bad && fn == (shpDef | 1); }
        }
        const u32 cbf = RS.r(0x20), r6 = cbf ? RS.p[0x91] : 0;
        ok = ok && (r6 < 1 || r6 > 3);
        u32 dl = 0, size = 0, dma = 0;
        if (ok)
        {
            dl = shp + rd(shp + 8, 4); size = rd(shp + 0xC, 4);
            dma = rd(lit(send + 0x100), 4);
            G = lit(send + 0x104);
            ok = !bad;
        }
        // GeSendDL push {r3-r5, lr} (S-48), SHP_InternalDefault push {r4-r6, lr} (S-32), SBC SHP push {r4-r6, lr} (S-16)
        const u32 w0[12] = {idx, r6, rs, shpDef + 0x5B, rs, fn, h * 4, v.shp + 0x5B, c->R[4], c->R[5], c->R[6], c->R[14]};
        if (ok && (size < 0x100 || dma == ~0u))
        {
            // small: NNS_G3dGeBufferOP_N(list[0], list + 1, size / 4 - 1), nothing being sent, no GE buffer or an empty one
            melonDS::GPU3D& gx = c->NDS.GPU.GPU3D;
            const u8* lp = m.P(dl);
            const u32 gb = rd(G, 4);
            ok = !bad && size >= 4 && !(size & 3) && !rd(G + 4, 4) && (!gb || !rd(gb, 4)) && !bad && lp && (dl >> 24) == 0x02
                 && m.P(dl + size - 1) == lp + size - 1 && (dl & c->DTCMMask) != c->DTCMBase && ((dl + size - 1) & c->DTCMMask) != c->DTCMBase
                 && (chk || (gx.GeometryEnabled && gx.BulkReady()));
            if (!ok) goto out;
            const u32 n = size / 4 - 1;
            // OP_N push {r3-r7, lr} (S-72)
            const u32 w1[6] = {idx, size, dl, shp, c->R[7], send + 0x27};
            for (int i = 0; i < 6; i++) m.W(fr, i * 4, w1[i]);
            for (int i = 0; i < 12; i++) m.W(fr, 24 + i * 4, w0[i]);
            for (int i = 0; i < 16; i++) e.R[i] = c->R[i];
            e.R[0] = sbc + 2; e.R[1] = 0x04000400; e.R[2] = n ? R32(lp + size - 4) : 0; e.R[3] = idx; e.R[12] = dl + size;
            e.R[14] = opn + 0x7D;
            e.retPc = c->R[14];
            const u32 rr = sbc + 2, nzcv = (rr & 0x80000000) | ((rr == 0) << 30) | ((rr < sbc) << 29) | (((~(sbc ^ 2u) & (sbc ^ rr)) >> 31) << 28);
            e.CPSR = (c->CPSR & 0x0FFFFFDF) | nzcv | ((e.retPc & 1) << 5);
#ifdef LITEV_HLE_DIAG
            if (chk)
            {
                m.Flush();      // logs only
                if (g_Check)
                {
                    m.log[m.n++] = {rs, sbc + 2, 4};
                    ArmCheck(c, 13, m, e);
                    g_P.small = true;
                    g_P.gx.assign((const u32*)lp, (const u32*)(lp + size));
#ifdef LITEV_A9HLE_GXCHECK
                    if (!GxTap) { g_GxMatTap.clear(); GxTap = &g_GxMatTap; g_P.gxOn = true; }
#endif
                    g_P.fit[0] = size / 4;
                }
                s.checks[13]++;
                GuestFallback(c);
                return true;
            }
#endif
            m.W(RS, 0, sbc + 2);
            m.Flush();
            gx.BulkWords((const u32*)lp, size / 4);
            s.native[13]++;
            Return(c, e, kShpSmallCycT0 + kShpSmallCycWordT * (s32)n);
            return true;
        }
        if (ok)
        {
            // GeSendDL's DMA path: nothing being sent, the GE buffer empty, no TWL path, a DMA (1-3)
            const u32 T = lit(v.code[9].a + 0x28), gb = rd(G, 4);
            ok = !bad && dma >= 1 && dma <= 3 && !rd(G + 4, 4) && !rd(G + 8, 4) && (!gb || !rd(gb, 4))
                 && rd(T + 0x1C, 4) && !rd(T + 4, 4) && !bad && lit(send + 0x10C) == ((flush + 0x48) | 1) && lit(send + 0x108) == G + 4;
        }
        if (ok)
        {
            // + the GE flush push {r3, lr} (S-56), GeSendDL's str r1, [sp] (the 5th argument) over its pushed r3
            for (int i = 0; i < 12; i++) m.W(fr, 24 + i * 4, w0[i]);
            m.W(fr, 16, idx); m.W(fr, 20, send + 0x2D);
            m.W(fr, 24, G + 4);
            const Obj g = m.O(G + 4, 4);
            m.W(g, 0, 1);
            for (int i = 0; i < 16; i++) R[i] = c->R[i];
            R[0] = dma; R[1] = dl; R[2] = size; R[3] = (flush + 0x48) | 1; R[4] = size; R[5] = dl; R[6] = shp;
            R[13] = S - 48; R[14] = send + 0xFF;
            r = AsyncCore(c, s, m, R, c->CPSR, G + 4, jit, e, chk ? &ck : nullptr);
        }
    }
out:
    if (r == 2) { s.native[13]++; return true; }
    if (r == 1)
    {
        // epilogue: GeSendDL pop {r3-r5, pc}, SHP_InternalDefault cmp r4, #2 / #3, pop {r4-r6, pc}, SBC SHP adds / pop
        e.R[0] = sbc + 2; e.R[3] = G + 4; e.R[4] = c->R[4]; e.R[5] = c->R[5]; e.R[6] = c->R[6]; e.R[13] = S;
        e.retPc = c->R[14];
        const u32 rr = sbc + 2, nzcv = (rr & 0x80000000) | ((rr == 0) << 30) | ((rr < sbc) << 29) | (((~(sbc ^ 2u) & (sbc ^ rr)) >> 31) << 28);
        e.CPSR = (c->CPSR & 0x0FFFFFDF) | nzcv | ((e.retPc & 1) << 5);
    }
    if (r == 1 && chk)
    {
#ifdef LITEV_HLE_DIAG
        m.Flush();      // logs only
        if (g_Check)
        {
            ArmCheck(c, 13, m, ck.e2);
            g_P.small = false;
            g_P.fit[0] = ck.words; g_P.io = ck.io;
            g_P.fin = e; g_P.finA = rs; g_P.finV = sbc + 2;
        }
#endif
        s.checks[13]++;
        GuestFallback(c);
        return true;
    }
    if (!r)
    {
        s.fallback[13] += !g_Check;
        GuestFallback(c);
        return true;
    }
    m.W(RS, 0, sbc + 2);
    m.Flush();
    s.native[13]++;
    Return(c, e, kAsyncCycT + kShpCycT);
    return true;
}
#endif

// 0. wake hook reached in the JIT (an IRQ other than the native HBlank one), 1. set, 2. get
__attribute__((noinline)) bool RunOs(melonDS::ARMv5* c, State& s, int k, bool jit)
{
    const u64 t0 = g_Time ? Now() : 0;
    Mem m(c, g_Check || g_Dry);
    Expect e;
    u32 tpc = 0, tcpsr = 0, bits = 0;
    // JIT: the hook block was compiled after IsHook verified the code, and it depends on every
    // checked byte (ARMJIT adds the Deps() ranges to the block), so a write there invalidates it.
    // Interpreter: compare per call.
    bool ok = !CheckPending && (jit || CodeIntact(c, s, k));
    if (ok)
    {
        if (k == 0)
        {
            IrqIn in{c->R_IRQ[2], c->R[13], c->R_SVC[0], c->R_IRQ[0], c->R_IRQ[1], c->R[12], false, 0, {}};
            ok = (c->CPSR & 0x3F) == 0x12 && Wake(c, s, m, e, in, tpc, tcpsr);
        }
        else if (s.v->twl) ok = k == 1 ? SetIrqT(c, s, m, e) : GetIrqT(c, s, m, e, bits);
        else ok = k == 1 ? SetIrq(c, s, m, e) : GetIrq(c, s, m, e, bits);
        if (ok) m.Flush();                              // check / dry: logs only
    }
#ifdef LITEV_HLE_DIAG
    if (ok && g_Check)
    {
        if (k == 2) s.getBits += bits;
        ArmCheck(c, k, m, e);
        s.checks[k]++;
        ok = false;
    }
    if (g_Dry && ok) { s.checks[k]++; ok = false; }
#endif
    if (!ok)
    {
        s.fallback[k] += !g_Check;
        GuestFallback(c);
        return true;
    }
    s.native[k]++;
    if (k == 0) WakeCommit(c, e);
    else Return(c, e, k == 1 ? s.v->setCyc : s.v->getCyc + s.v->getCycPerBit * (s32)bits);
    if (g_Time) s.ns += Now() - t0;
    return true;
}

// Thumb entries (TWL SDK build: OS_Set/GetIrqFunction)
__attribute__((noinline)) bool RunThumb(melonDS::ARMv5* c, bool jit)
{
    const u32 pc = c->R[15] - 4, in = c->CurInstr & 0xFFFF;
    State& s = Get(c, pc, in, true);
    if (s.status != 1) return false;
    const int k = Kind(*s.v, pc, in, true);
    if (k < 0 || !(s.mask & Bit(k))) return false;
    s.calls[k]++;
#ifdef LITEV_GX_BULK
    if (k == 5) return RunGx(c, s, jit);
    if (k == 8) return RunMatT(c, s, jit);
    if (k == 10) return RunAsync(c, s, jit);
    if (k == 13) return RunShpT(c, s, jit);
#endif
    return RunOs(c, s, k, jit);
}
}

bool Run(melonDS::ARM* cpu, bool jit)
{
    if (cpu->Num != 0) return false;
    auto* c = (melonDS::ARMv5*)cpu;
    if (cpu->CPSR & 0x20) return RunThumb(c, jit);
    const u32 pc = cpu->R[15] - 8, in = cpu->CurInstr;
    if (in == kLzInstr)
    {
        State& s = St(c);
        return s.on && (s.mask & 64) && RunLz(c, s, jit);
    }
    if (in == kCardInstr)
    {
        State& s = St(c);
        return s.on && (s.mask & 32) && RunCard(c, s, jit);
    }
#ifdef LITEV_GX_BULK
    if (in == kMatInstr)
    {
        State& s = St(c);
        if (s.status == 1 && pc == s.v->shp) return (s.mask & 2048) && RunShp(c, s, jit);
        return s.on && (s.mask & 128) && RunMat(c, s, jit);
    }
#endif
    if (in == kSdivInstr)
    {
        State& s = St(c);
        return s.on && (s.mask & 256) && RunSdiv(c, s, jit);
    }
    if (in == kVecInstr)
    {
        State& s = St(c);
        return s.on && (s.mask & 4096) && RunVec(c, s, jit);
    }
    State& s = Get(c, pc, in);
    if (s.status != 1) return false;
    const int k = Kind(*s.v, pc, in);
    if (k < 0 || !(s.mask & Bit(k))) return false;
    s.calls[k]++;
#ifdef LITEV_GX_BULK
    if (k == 5) return RunGx(c, s, jit);
    if (k == 10) return RunAsync(c, s, jit);
#endif
    return RunOs(c, s, k, jit);
}

#ifdef LITEV_HLE_DIAG
void CheckAt(melonDS::ARM* cpu, u32 pc)
{
    auto* c = (melonDS::ARMv5*)cpu;
    if (g_Card.on) { CardCheckAt(c, pc); return; }
    if (g_P.stage2)
    {
        // G3D shape check, stage 2: at SHP's return (an IRQ inside restores every register), the registers, flags and
        // the SBC pointer
        const Expect& f = g_P.fin;
        if (pc != (f.retPc & ~1u) || (cpu->CPSR & 0x3F) != (f.CPSR & 0x3F) || cpu->R[13] != f.R[13])
        {
            if (++g_P.steps > 2000000) { fprintf(stderr, "A9HLE CHECK %s: guest never returned\n", kName[g_P.kind]); g_P.stage2 = CheckPending = false; c->Idle2Log = nullptr; }
            return;
        }
        g_P.stage2 = CheckPending = false;
        c->Idle2Log = nullptr;
        State& s = g_State[&c->NDS];
        char b[512]; int bl = 0, nd = 0;
        for (int i = 0; i < 15; i++)
            if (cpu->R[i] != f.R[i]) { if (nd < 8) bl += snprintf(b + bl, sizeof(b) - bl, " r%d guest %08x native %08x;", i, cpu->R[i], f.R[i]); nd++; }
        if (cpu->CPSR != f.CPSR) { bl += snprintf(b + bl, sizeof(b) - bl, " cpsr guest %08x native %08x;", cpu->CPSR, f.CPSR); nd++; }
        Mem m(c, false);
        const u8* q = m.P(g_P.finA);
        if (!q || R32(q) != g_P.finV) { bl += snprintf(b + bl, sizeof(b) - bl, " sbc guest %08x native %08x;", q ? R32(q) : 0, g_P.finV); nd++; }
        if (nd) { if (s.diffs[g_P.kind] < 10) fprintf(stderr, "A9HLE CHECK %s final %d diffs:%s\n", kName[g_P.kind], nd, b); s.diffs[g_P.kind]++; }
        return;
    }
    const Expect& e = g_P.e;
    // guest registers as seen in the interrupted mode after the return
    u32 gR[15], gI[3] = {c->R_IRQ[0], c->R_IRQ[1], c->R_IRQ[2]}, gS[3] = {c->R_SVC[0], c->R_SVC[1], c->R_SVC[2]};
    for (int i = 0; i < 15; i++) gR[i] = cpu->R[i];
    bool atVec = false;     // native IRQ check completed at the entry of the next IRQ
    if (pc == c->ExceptionBase + 0x18)
    {
        if (g_P.vecSkip) g_P.vecSkip = false;
        else if (g_P.kind >= 3 && !g_P.irq && (c->CPSR & 0x1F) == 0x12 && c->R_IRQ[2] == e.CPSR && cpu->R[14] - 4 == (e.retPc & ~1u))
        {
            // the guest returned and took the next IRQ at once: undo that entry's banking
            atVec = true;
            std::swap(gR[13], gI[0]); std::swap(gR[14], gI[1]);
            if ((e.CPSR & 0x1F) == 0x13) { std::swap(gR[13], gS[0]); std::swap(gR[14], gS[1]); }
            if (g_P.kind == 4) { gI[1] = e.IRQ[1]; gI[2] = e.IRQ[2]; }   // overwritten by the new entry
        }
        else g_P.irq = true;
    }
    if (g_P.kind == 6)
    {
        const u64 now = c->NDS.ARM9Timestamp + c->Cycles;
        if (g_P.lzLastIn) g_P.lzOwn += now - g_P.lzLastTs;
        g_P.lzLastTs = now; g_P.lzLastIn = pc - g_P.lzFn < 45 * 4;
    }
    if (++g_P.steps > 2000000)
    {
        fprintf(stderr, "A9HLE CHECK %s: guest never returned to %08x\n", kName[g_P.kind], g_P.e.retPc);
#ifdef LITEV_A9HLE_GXCHECK
        if (g_P.gxOn) { GxTap = nullptr; g_P.gxOn = false; }
#endif
        CheckPending = false;
        c->Idle2Log = nullptr;
        g_State[&c->NDS].diffs[g_P.kind]++;
        return;
    }
    if (!atVec && (pc != (e.retPc & ~1u) || cpu->CPSR != e.CPSR)) return;
    if (g_P.kind == 10 || g_P.kind == 13) gR[1] = e.R[1];     // GX async: r1 = the IF value read (not modelled)
    if (g_P.kind == 6 && (cpu->R[2] != e.R[2] || cpu->R[3] != e.R[3] || cpu->R[6] != e.R[6])) return;   // LZ: same progress
    melonDS::NDS& nds = c->NDS;
    if ((g_P.kind == 0 || g_P.kind == 4) && R32(&nds.MainRAM[(g_State[&nds].v->info + 4) & nds.MainRAMMask]) != e.cur) return;
    CheckPending = false;
    c->Idle2Log = nullptr;
    State& s = g_State[&nds];
    const int k = g_P.kind;
    if (g_P.irq && (k == 1 || k == 2))
    {
        s.irqDuring[k]++;   // an IRQ handler ran inside the function: its writes would show as diffs
        return;
    }
    if (k < 8 || !g_P.irq) { s.guestCyc[k] += nds.ARM9Timestamp + c->Cycles - g_P.t0; s.guestN[k]++; }   // 8, 9: without IRQs
    if (k == 10 && getenv("LITEV_A9HLE_ASYNCFIT") && !g_P.irq)
        fprintf(stderr, "ASYNCFIT %u %llu\n", g_P.fit[0], (unsigned long long)(nds.ARM9Timestamp + c->Cycles - g_P.t0));
    if (k == 13 && getenv("LITEV_A9HLE_SHPFIT") && !g_P.irq)
        fprintf(stderr, "SHPFIT %d %u %llu\n", (int)g_P.small, g_P.fit[0], (unsigned long long)(nds.ARM9Timestamp + c->Cycles - g_P.t0));
    if (k == 8 && getenv("LITEV_A9HLE_MATFIT") && !g_P.irq)
        fprintf(stderr, "MATFIT %u %llu\n", g_P.fit[0], (unsigned long long)(nds.ARM9Timestamp + c->Cycles - g_P.t0));
    if (k == 9 && getenv("LITEV_A9HLE_LLFIT") && !g_P.irq)
        fprintf(stderr, "LLFIT %u %u %u %u %llu\n", g_P.fit[0], g_P.fit[1], g_P.fit[2], g_P.fit[3],
                (unsigned long long)(nds.ARM9Timestamp + c->Cycles - g_P.t0));
    if (k == 6 && getenv("LITEV_A9HLE_LZFIT"))
        fprintf(stderr, "LZFIT %u %u %u %u %llu\n", g_P.lzn[0], g_P.lzn[1], g_P.lzn[2], g_P.lzn[3], (unsigned long long)g_P.lzOwn);
    // expected bytes: native log over the snapshot; compared at every byte either side wrote
    std::unordered_map<u32, u8> exp;
    Mem m(c, false);
    auto old = [&](u32 a) -> u8 {
        if ((a & c->DTCMMask) == c->DTCMBase) return g_P.dtcm[a & (DTCMPhysicalSize - 1)];
        return g_P.ram[a & nds.MainRAMMask];
    };
    for (auto& w : g_P.log)
        for (u32 i = 0; i < w.sz; i++) exp[w.a + i] = (u8)(w.v >> (8 * i));
    std::unordered_set<u32> addrs;
    for (auto& [a, v] : exp) addrs.insert(a);
    for (auto& a : g_P.acc)
        if (a.Write)
            for (u32 i = 0; i < a.Size; i++)
                if (m.P(a.Addr + i)) addrs.insert(a.Addr + i);
    int nd = 0;
    char buf[1024]; int bl = 0;
    for (u32 a : addrs)
    {
        u8* p = m.P(a);
        if (!p || (k == 6 && (a < g_P.lzLo || a >= g_P.lzHi))) continue;
        auto it = exp.find(a);
        u8 want = it != exp.end() ? it->second : old(a);
        if (*p != want)
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " %08x guest %02x native %02x;", a, *p, want);
            nd++;
        }
    }
    for (int i = 0; i < 15; i++)
        if (gR[i] != e.R[i])
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " r%d guest %08x native %08x;", i, gR[i], e.R[i]);
            nd++;
        }
    if (e.banks)
    {
        const u32* gi = gI; const u32* gs = gS;
        for (int i = 0; i < 3; i++)
        {
            if (gi[i] != e.IRQ[i]) { if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " irq[%d] guest %08x native %08x;", i, gi[i], e.IRQ[i]); nd++; }
            if (gs[i] != e.SVC[i]) { if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " svc[%d] guest %08x native %08x;", i, gs[i], e.SVC[i]); nd++; }
        }
    }
    if (k == 10 || k == 11 || (k == 13 && !g_P.small))
    {
        std::vector<std::pair<u32, u32>> io;
        for (auto& a : g_P.acc)
            if (a.Write && (a.Addr >> 24) == 0x04 && a.Addr != 0x04000208)
                io.push_back({a.Addr, a.Addr == 0x04000600 ? a.Val & 0xC0008000u : a.Val});
        if (io != g_P.io)
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " IO writes: guest %zu, native %zu;", io.size(), g_P.io.size());
            if (getenv("LITEV_A9HLE_IODUMP") && !g_P.irq) { for (auto& x : io) fprintf(stderr, " %08x=%08x", x.first, x.second); fprintf(stderr, " |"); for (auto& x : g_P.io) fprintf(stderr, " %08x=%08x", x.first, x.second); fprintf(stderr, "\n"); }
            nd++;
        }
    }
    if (k == 8 || (k == 13 && g_P.small))
    {
        // GXFIFO words: the guest's stores to GXFIFO, then what reached BulkWords (MI_CpuSend32 under GX_CPUSEND)
        std::vector<u32> gw;
#ifdef LITEV_A9HLE_GXCHECK
        if (g_P.gxOn) { gw = g_GxMatTap; GxTap = nullptr; g_P.gxOn = false; }     // every GXFIFO word (stores and BulkWords)
        else
#endif
        for (auto& a : g_P.acc)
            if (a.Write && a.Addr >= 0x04000400 && a.Addr < 0x04000440) gw.push_back(a.Val);
        if (gw != g_P.gx)
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " GX words: guest %zu (first %08x), native %zu;", gw.size(), gw.empty() ? 0 : gw[0], g_P.gx.size());
            nd++;
        }
    }
    if (k == 3 || k == 4)
    {
        // IF: the guest acknowledged exactly HBlank (the native path does IF &= ~2)
        int acks = 0, other = 0;
        for (auto& a : g_P.acc)
            if (a.Write && (a.Addr & ~3u) == 0x04000214) { if (a.Addr == 0x04000214 && a.Size == 4 && a.Val == 2) acks++; else other++; }
        if (acks != 1 || other)
        {
            if (nd < 12) bl += snprintf(buf + bl, sizeof(buf) - bl, " IF writes: %d HBlank acks, %d other;", acks, other);
            nd++;
        }
    }
    if (k == 11)
        for (auto& a : g_P.acc)     // a lower IRQ (HBlank) raised after the delivery: the guest's handler took that one first
            if (a.Write && a.Addr == 0x04000214) { if (a.Val != g_P.io[0].second) g_P.irq = true; break; }
    if (nd && g_P.irq && k != 6)
        nd = 0;   // an IRQ arrived inside the guest round trip: the native path takes it after; counted in irq_during
    if (nd)
    {
        if (s.diffs[k] < 10) fprintf(stderr, "A9HLE CHECK %s (irq_in_window=%d) %d diffs:%s\n", kName[k], (int)g_P.irq, nd, buf);
        s.diffs[k]++;
    }
    if (g_P.irq) s.irqDuring[k]++;
    else if ((k == 13 || k == 10) && !g_P.small) { g_P.stage2 = true; g_P.steps = 0; CheckPending = true; c->Idle2Log = nullptr; }
}
#endif
}
#endif
