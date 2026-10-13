// LITEV_A7_NATIVE_WM: the NitroSDK ARM7 wireless driver (WM) natively, per WM build.
//
// From WM INITIALIZE on (a recognised WM build: the WMSP request dispatch table found by a scan of the ARM7's memory,
// the build id = a relocation-masked hash of its handlers), the ARM9's WM requests (PXI tag 10) are taken here instead
// of reaching the ARM7, and the Wi-Fi chip's IRQ comes here instead of the ARM7: the guest's WM / WL threads, IRQ
// handler and alarms never run. Two layers, as the guest's: WMSP (request handlers, WMStatus in ARM9 RAM, the callbacks
// to the ARM9 through the shared slot, the MP send queue and port packets, the V-count MP alarms incl. the child's
// V-count sync) and WL (chip bring-up, passive scan, beacon, auth / assoc on both sides, the MP CMD / reply / ACK
// exchange, lifetimes). Frames on air are the guest's byte for byte; mixed sessions (native and emulated drivers) work.
// Category B: deterministic; the guest's request latencies and MP cadence are modelled, not cycle-exact.
// Runtime: debug.litev.a7wm (prop on Android, env elsewhere; unset = on, 0 = off); headless LITEV_A7WM_INST=<mask>.
// Diagnostics (LITEV_HLE_DIAG): LITEV_WMTRACE=<prefix> (+ _A7W / _CHIP / _MEM) protocol traces; LITEV_A7WM_STATS=1.
#pragma once
#ifdef LITEV_A7_NATIVE_WM
#include "types.h"
namespace melonDS { class NDS; class Savestate; }
namespace melonDS::WMNative
{
// NDS::DoSavestate: section "WMNA" (the driver's state; absent = not active)
void DoSavestate(melonDS::NDS& nds, melonDS::Savestate* file);
// Wifi::Reset: the native driver lets go of the WM (a console reset ends the session)
void Reset(melonDS::NDS& nds);
// headless: name / allow per console (allow -1: debug.litev.a7wm decides)
void SetInstance(melonDS::NDS& nds, int inst, int allow);
// NDS: an ARM9 word to the ARM7 IPC FIFO with the WM tag; true = the native driver took it (not queued for the ARM7)
bool Fifo9(melonDS::NDS& nds, u32 val);
// GPU::StartScanline at the line the driver asked for (GPU::WMLine); ts = that line's start
void VLine(melonDS::NDS& nds, u16 vcount, u64 ts);
// NDS: an ARM7 word to the ARM9 IPC FIFO (the guest's WM callbacks, while the native driver does not own the WM)
void Fifo7(melonDS::NDS& nds, u32 val);
// Wifi::CheckIRQ while the native driver owns the chip: IF & IE nonzero
void ChipIRQ(melonDS::NDS& nds);
#ifdef LITEV_HLE_DIAG
// LITEV_WMTRACE=<prefix>: per-console trace of the WM protocol (PXI words both ways with the buffers they point at, the
// Wi-Fi frames sent / received) -> <prefix>.<console>.txt
void TraceFifo(melonDS::NDS& nds, int fromArm7, u32 val);
void TraceFrame(melonDS::NDS& nds, int tx, const u8* buf, int len, u64 us);
// ARM7 stores to main RAM (interpreter: every one; JIT: only the slow-path ones)
void TraceA7Write(melonDS::NDS& nds, u32 addr, u32 val, int size);
// Wi-Fi chip register writes (LITEV_WMTRACE_CHIP=<from>:<to>)
void TraceChip(melonDS::NDS& nds, int wr, u32 addr, u16 val, u64 us);
#else
inline void TraceFifo(melonDS::NDS&, int, u32) {}
inline void TraceFrame(melonDS::NDS&, int, const u8*, int, u64) {}
inline void TraceA7Write(melonDS::NDS&, u32, u32, int) {}
inline void TraceChip(melonDS::NDS&, int, u32, u16, u64) {}
#endif
}
#endif
