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

#include <stdio.h>
#include "NDS.h"
#include "DSi.h"
#include "DMA.h"
#include "GPU.h"
#include "GPU3D.h"
#ifdef LITEV_GXFIFO_DMA_INLINE
// pulls in the always-inline WriteToGXFIFO_Inline so the geometry-DMA loop below inlines the
// whole GXFIFO producer path (no per-word cross-TU bl to WriteToGXFIFO/CmdFIFOWrite).
#include "GPU3D_GXFIFO_inl.h"
#endif
#include "DMA_Timings.h"
#include "Platform.h"

#if defined(__ANDROID__)
#include <sys/system_properties.h>
#include <cstring>
#include <cstdlib>
// default-ON variant: true unless the prop is explicitly present and "0". Lets a compiled-in
// lever ship ON by default while an A/B (which always sets the prop 0/1) can still force it off.
static bool litevDmaPropDefaultOn(const char* name) {
    char b[8] = {0};
    int n = __system_property_get(name, b);
    return (n > 0) ? (atoi(b) != 0) : true;
}
#endif

namespace melonDS
{
using Platform::Log;
using Platform::LogLevel;

// DMA TIMINGS
//
// sequential timing:
// * 1 cycle per read or write
// * in 32bit mode, accessing a 16bit bus (mainRAM, palette, VRAM) incurs 1 cycle of penalty
// * in 32bit mode, transferring from mainRAM to another bank is 1 cycle faster
// * if source and destination are the same memory bank, there is a 1 cycle penalty
// * transferring from mainRAM to mainRAM is a trainwreck (all accesses are made nonsequential)
//
// nonsequential timing:
// * nonseq penalty is applied to the first read and write
// * I also figure it gets nonseq penalty again when resuming, after having been interrupted by
//   another DMA (TODO: check)
// * applied to all accesses for mainRAM->mainRAM, resulting in timings of 16-18 cycles per unit
//
// TODO: GBA slot
// TODO: re-add initial NS delay
// TODO: timings are nonseq when address is fixed/decrementing


DMA::DMA(u32 cpu, u32 num, melonDS::NDS& nds) :
    CPU(cpu),
    Num(num),
    NDS(nds)
{
    if (cpu == 0)
        CountMask = 0x001FFFFF;
    else
        CountMask = (num==3 ? 0x0000FFFF : 0x00003FFF);
}

void DMA::Reset()
{
    SrcAddr = 0;
    DstAddr = 0;
    Cnt = 0;

    StartMode = 0;
    CurSrcAddr = 0;
    CurDstAddr = 0;
    RemCount = 0;
    IterCount = 0;
    SrcAddrInc = 0;
    DstAddrInc = 0;

    Stall = false;

    Running = false;
    Executing = false;
    InProgress = false;
    MRAMBurstCount = 0;
    MRAMBurstTable = DMATiming::MRAMDummy;
}

void DMA::DoSavestate(Savestate* file)
{
    char magic[5] = "DMAx";
    magic[3] = '0' + Num + (CPU*4);
    file->Section(magic);

    file->Var32(&SrcAddr);
    file->Var32(&DstAddr);
    file->Var32(&Cnt);

    file->Var32(&StartMode);
    file->Var32(&CurSrcAddr);
    file->Var32(&CurDstAddr);
    file->Var32(&RemCount);
    file->Var32(&IterCount);
    file->Var32((u32*)&SrcAddrInc);
    file->Var32((u32*)&DstAddrInc);

    file->Var32(&Running);
    file->Bool32(&InProgress);
    file->Bool32(&IsGXFIFODMA);
    file->Var32(&MRAMBurstCount);
    file->Bool32(&Executing);
    file->Bool32(&Stall);

    file->VarArray(MRAMBurstTable.data(), sizeof(MRAMBurstTable));
}

void DMA::WriteCnt(u32 val)
{
    u32 oldcnt = Cnt;
    Cnt = val;

    if ((!(oldcnt & 0x80000000)) && (val & 0x80000000))
    {
        CurSrcAddr = SrcAddr;
        CurDstAddr = DstAddr;

        switch (Cnt & 0x00600000)
        {
        case 0x00000000: DstAddrInc = 1; break;
        case 0x00200000: DstAddrInc = -1; break;
        case 0x00400000: DstAddrInc = 0; break;
        case 0x00600000: DstAddrInc = 1; break;
        }

        switch (Cnt & 0x01800000)
        {
        case 0x00000000: SrcAddrInc = 1; break;
        case 0x00800000: SrcAddrInc = -1; break;
        case 0x01000000: SrcAddrInc = 0; break;
        case 0x01800000: SrcAddrInc = 1; break;
        }

        if (CPU == 0)
            StartMode = (Cnt >> 27) & 0x7;
        else
            StartMode = ((Cnt >> 28) & 0x3) | 0x10;

        if ((StartMode & 0x7) == 0)
            Start();
        else if (StartMode == 0x05 || StartMode == 0x12)
            NDS.NDSCartSlots[0]->CheckDMA(CPU);
        else if (StartMode == 0x07)
            NDS.GPU.GPU3D.CheckFIFODMA();

        if (StartMode==0x06 || StartMode==0x13)
            Log(LogLevel::Warn, "UNIMPLEMENTED ARM%d DMA%d START MODE %02X, %08X->%08X\n", CPU?7:9, Num, StartMode, SrcAddr, DstAddr);
    }
}

void DMA::Start()
{
    if (Running) return;

    if (!InProgress)
    {
        u32 countmask;
        if (CPU == 0)
            countmask = 0x001FFFFF;
        else
            countmask = (Num==3 ? 0x0000FFFF : 0x00003FFF);

        RemCount = Cnt & countmask;
        if (!RemCount)
            RemCount = countmask+1;
    }

    if (StartMode == 0x07 && RemCount > 112)
        IterCount = 112;
    else
        IterCount = RemCount;

    if ((Cnt & 0x01800000) == 0x01800000)
        CurSrcAddr = SrcAddr;

    if ((Cnt & 0x00600000) == 0x00600000)
        CurDstAddr = DstAddr;

    //printf("ARM%d DMA%d %08X %02X %08X->%08X %d bytes %dbit\n", CPU?7:9, Num, Cnt, StartMode, CurSrcAddr, CurDstAddr, RemCount*((Cnt&0x04000000)?4:2), (Cnt&0x04000000)?32:16);

    IsGXFIFODMA = (CPU == 0 && (CurSrcAddr>>24) == 0x02 && CurDstAddr == 0x04000400 && DstAddrInc == 0);

    // TODO eventually: not stop if we're running code in ITCM

    Running = 2;

    // safety measure
    MRAMBurstTable = DMATiming::MRAMDummy;

    InProgress = true;
    NDS.StopCPU(CPU, 1<<Num);
}

u32 DMA::UnitTimings9_16(bool burststart)
{
    u32 src_id = CurSrcAddr >> 14;
    u32 dst_id = CurDstAddr >> 14;

    u32 src_rgn = NDS.ARM9Regions[src_id];
    u32 dst_rgn = NDS.ARM9Regions[dst_id];

    u32 src_n, src_s, dst_n, dst_s;
    src_n = NDS.ARM9MemTimings[src_id][4];
    src_s = NDS.ARM9MemTimings[src_id][5];
    dst_n = NDS.ARM9MemTimings[dst_id][4];
    dst_s = NDS.ARM9MemTimings[dst_id][5];

    if (src_rgn == Mem9_MainRAM)
    {
        if (dst_rgn == Mem9_MainRAM)
            return 16;

        if (SrcAddrInc > 0)
        {
            if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
            {
                MRAMBurstCount = 0;

                if (dst_rgn == Mem9_GBAROM)
                {
                    if (dst_s == 4)
                        MRAMBurstTable = DMATiming::MRAMRead16Bursts[1];
                    else
                        MRAMBurstTable = DMATiming::MRAMRead16Bursts[2];
                }
                else
                    MRAMBurstTable = DMATiming::MRAMRead16Bursts[0];
            }

            u32 ret = MRAMBurstTable[MRAMBurstCount++];
            return ret;
        }
        else
        {
            // TODO: not quite right for GBA slot
            return (((CurSrcAddr & 0x1F) == 0x1E) ? 7 : 8) +
                   (burststart ? dst_n : dst_s);
        }
    }
    else if (dst_rgn == Mem9_MainRAM)
    {
        if (DstAddrInc > 0)
        {
            if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
            {
                MRAMBurstCount = 0;

                if (src_rgn == Mem9_GBAROM)
                {
                    if (src_s == 4)
                        MRAMBurstTable = DMATiming::MRAMWrite16Bursts[1];
                    else
                        MRAMBurstTable = DMATiming::MRAMWrite16Bursts[2];
                }
                else
                    MRAMBurstTable = DMATiming::MRAMWrite16Bursts[0];
            }

            u32 ret = MRAMBurstTable[MRAMBurstCount++];
            return ret;
        }
        else
        {
            return (burststart ? src_n : src_s) + 7;
        }
    }
    else if (src_rgn & dst_rgn)
    {
        return src_n + dst_n + 1;
    }
    else
    {
        if (burststart)
            return src_n + dst_n;
        else
            return src_s + dst_s;
    }
}

u32 DMA::UnitTimings9_32(bool burststart)
{
    u32 src_id = CurSrcAddr >> 14;
    u32 dst_id = CurDstAddr >> 14;

    u32 src_rgn = NDS.ARM9Regions[src_id];
    u32 dst_rgn = NDS.ARM9Regions[dst_id];

    // LITEV_DMA_TIMING_LAZY (default OFF): the 4 ARM9MemTimings[][6/7] loads are only used on
    // SOME paths — the hot geometry-DMA path (src=MainRAM, SrcAddrInc>0, MRAM burst table) uses
    // dst_n/dst_s only when the burst table is (re)selected, and never uses src_n/src_s. Defer
    // each load into the branch that actually reads it. BIT-EXACT: same values (ARM9MemTimings
    // is invariant within one call — no guest code runs mid-function), just loaded on demand;
    // MP-safe (no change to the returned timing, only when the host issues the load).
    u32 src_n, src_s, dst_n, dst_s;
#ifndef LITEV_DMA_TIMING_LAZY
    src_n = NDS.ARM9MemTimings[src_id][6];
    src_s = NDS.ARM9MemTimings[src_id][7];
    dst_n = NDS.ARM9MemTimings[dst_id][6];
    dst_s = NDS.ARM9MemTimings[dst_id][7];
#endif

    if (src_rgn == Mem9_MainRAM)
    {
        if (dst_rgn == Mem9_MainRAM)
            return 18;

        if (SrcAddrInc > 0)
        {
            if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
            {
                MRAMBurstCount = 0;
#ifdef LITEV_DMA_TIMING_LAZY
                dst_n = NDS.ARM9MemTimings[dst_id][6];
                dst_s = NDS.ARM9MemTimings[dst_id][7];
#endif
                if (dst_rgn == Mem9_GBAROM)
                {
                    if (dst_s == 8)
                        MRAMBurstTable = DMATiming::MRAMRead32Bursts[2];
                    else
                        MRAMBurstTable = DMATiming::MRAMRead32Bursts[3];
                }
                else if (dst_n == 2)
                    MRAMBurstTable = DMATiming::MRAMRead32Bursts[0];
                else
                    MRAMBurstTable = DMATiming::MRAMRead32Bursts[1];
            }

            u32 ret = MRAMBurstTable[MRAMBurstCount++];
            return ret;
        }
        else
        {
#ifdef LITEV_DMA_TIMING_LAZY
            dst_n = NDS.ARM9MemTimings[dst_id][6];
            dst_s = NDS.ARM9MemTimings[dst_id][7];
#endif
            // TODO: not quite right for GBA slot
            return (((CurSrcAddr & 0x1F) == 0x1C) ? (dst_n==2 ? 7:8) : 9) +
                   (burststart ? dst_n : dst_s);
        }
    }
    else if (dst_rgn == Mem9_MainRAM)
    {
        if (DstAddrInc > 0)
        {
            if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
            {
                MRAMBurstCount = 0;
#ifdef LITEV_DMA_TIMING_LAZY
                src_n = NDS.ARM9MemTimings[src_id][6];
                src_s = NDS.ARM9MemTimings[src_id][7];
#endif
                if (src_rgn == Mem9_GBAROM)
                {
                    if (src_s == 8)
                        MRAMBurstTable = DMATiming::MRAMWrite32Bursts[2];
                    else
                        MRAMBurstTable = DMATiming::MRAMWrite32Bursts[3];
                }
                else if (src_n == 2)
                    MRAMBurstTable = DMATiming::MRAMWrite32Bursts[0];
                else
                    MRAMBurstTable = DMATiming::MRAMWrite32Bursts[1];
            }

            u32 ret = MRAMBurstTable[MRAMBurstCount++];
            return ret;
        }
        else
        {
#ifdef LITEV_DMA_TIMING_LAZY
            src_n = NDS.ARM9MemTimings[src_id][6];
            src_s = NDS.ARM9MemTimings[src_id][7];
#endif
            return (burststart ? src_n : src_s) + 8;
        }
    }
    else if (src_rgn & dst_rgn)
    {
#ifdef LITEV_DMA_TIMING_LAZY
        src_n = NDS.ARM9MemTimings[src_id][6];
        dst_n = NDS.ARM9MemTimings[dst_id][6];
#endif
        return src_n + dst_n + 1;
    }
    else
    {
#ifdef LITEV_DMA_TIMING_LAZY
        src_n = NDS.ARM9MemTimings[src_id][6];
        src_s = NDS.ARM9MemTimings[src_id][7];
        dst_n = NDS.ARM9MemTimings[dst_id][6];
        dst_s = NDS.ARM9MemTimings[dst_id][7];
#endif
        if (burststart)
            return src_n + dst_n;
        else
            return src_s + dst_s;
    }
}

// Exact specialization of UnitTimings9_32's MainRAM-incrementing branch for the
// fixed 0x04000400 geometry FIFO destination. Callers retain the generic path
// when the source does not increment. The destination timing index is constant:
// 0x04000400 >> 14 == 0x1000. This performs the identical burst-table state
// transition and returns the identical guest-cycle value without region-map loads.
u32 DMA::UnitTimings9_32_GXFIFO(bool burststart)
{
    if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
    {
        MRAMBurstCount = 0;
        const u32 dst_n = NDS.ARM9MemTimings[0x1000][6];
        MRAMBurstTable = (dst_n == 2) ?
            DMATiming::MRAMRead32Bursts[0] : DMATiming::MRAMRead32Bursts[1];
    }
    return MRAMBurstTable[MRAMBurstCount++];
}

// TODO: the ARM7 ones don't take into account that the two wifi regions have different timings

u32 DMA::UnitTimings7_16(bool burststart)
{
    u32 src_id = CurSrcAddr >> 15;
    u32 dst_id = CurDstAddr >> 15;

    u32 src_rgn = NDS.ARM7Regions[src_id];
    u32 dst_rgn = NDS.ARM7Regions[dst_id];

    u32 src_n, src_s, dst_n, dst_s;
    src_n = NDS.ARM7MemTimings[src_id][0];
    src_s = NDS.ARM7MemTimings[src_id][1];
    dst_n = NDS.ARM7MemTimings[dst_id][0];
    dst_s = NDS.ARM7MemTimings[dst_id][1];

    if (src_rgn == Mem7_MainRAM)
    {
        if (dst_rgn == Mem7_MainRAM)
            return 16;

        if (SrcAddrInc > 0)
        {
            if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
            {
                MRAMBurstCount = 0;

                if (dst_rgn == Mem7_GBAROM || dst_rgn == Mem7_Wifi0 || dst_rgn == Mem7_Wifi1)
                {
                    if (dst_s == 4)
                        MRAMBurstTable = DMATiming::MRAMRead16Bursts[1];
                    else
                        MRAMBurstTable = DMATiming::MRAMRead16Bursts[2];
                }
                else
                    MRAMBurstTable = DMATiming::MRAMRead16Bursts[0];
            }

            u32 ret = MRAMBurstTable[MRAMBurstCount++];
            return ret;
        }
        else
        {
            // TODO: not quite right for GBA slot
            return (((CurSrcAddr & 0x1F) == 0x1E) ? 7 : 8) +
                   (burststart ? dst_n : dst_s);
        }
    }
    else if (dst_rgn == Mem7_MainRAM)
    {
        if (DstAddrInc > 0)
        {
            if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
            {
                MRAMBurstCount = 0;

                if (src_rgn == Mem7_GBAROM || src_rgn == Mem7_Wifi0 || src_rgn == Mem7_Wifi1)
                {
                    if (src_s == 4)
                        MRAMBurstTable = DMATiming::MRAMWrite16Bursts[1];
                    else
                        MRAMBurstTable = DMATiming::MRAMWrite16Bursts[2];
                }
                else
                    MRAMBurstTable = DMATiming::MRAMWrite16Bursts[0];
            }

            u32 ret = MRAMBurstTable[MRAMBurstCount++];
            return ret;
        }
        else
        {
            return (burststart ? src_n : src_s) + 7;
        }
    }
    else if (src_rgn & dst_rgn)
    {
        return src_n + dst_n + 1;
    }
    else
    {
        if (burststart)
            return src_n + dst_n;
        else
            return src_s + dst_s;
    }
}

u32 DMA::UnitTimings7_32(bool burststart)
{
    u32 src_id = CurSrcAddr >> 15;
    u32 dst_id = CurDstAddr >> 15;

    u32 src_rgn = NDS.ARM7Regions[src_id];
    u32 dst_rgn = NDS.ARM7Regions[dst_id];

    u32 src_n, src_s, dst_n, dst_s;
    src_n = NDS.ARM7MemTimings[src_id][2];
    src_s = NDS.ARM7MemTimings[src_id][3];
    dst_n = NDS.ARM7MemTimings[dst_id][2];
    dst_s = NDS.ARM7MemTimings[dst_id][3];

    if (src_rgn == Mem7_MainRAM)
    {
        if (dst_rgn == Mem7_MainRAM)
            return 18;

        if (SrcAddrInc > 0)
        {
            if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
            {
                MRAMBurstCount = 0;

                if (dst_rgn == Mem7_GBAROM || dst_rgn == Mem7_Wifi0 || dst_rgn == Mem7_Wifi1)
                {
                    if (dst_s == 8)
                        MRAMBurstTable = DMATiming::MRAMRead32Bursts[2];
                    else
                        MRAMBurstTable = DMATiming::MRAMRead32Bursts[3];
                }
                else if (dst_n == 2)
                    MRAMBurstTable = DMATiming::MRAMRead32Bursts[0];
                else
                    MRAMBurstTable = DMATiming::MRAMRead32Bursts[1];
            }

            u32 ret = MRAMBurstTable[MRAMBurstCount++];
            return ret;
        }
        else
        {
            // TODO: not quite right for GBA slot
            return (((CurSrcAddr & 0x1F) == 0x1C) ? (dst_n==2 ? 7:8) : 9) +
                   (burststart ? dst_n : dst_s);
        }
    }
    else if (dst_rgn == Mem7_MainRAM)
    {
        if (DstAddrInc > 0)
        {
            if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
            {
                MRAMBurstCount = 0;

                if (src_rgn == Mem7_GBAROM || src_rgn == Mem7_Wifi0 || src_rgn == Mem7_Wifi1)
                {
                    if (src_s == 8)
                        MRAMBurstTable = DMATiming::MRAMWrite32Bursts[2];
                    else
                        MRAMBurstTable = DMATiming::MRAMWrite32Bursts[3];
                }
                else if (src_n == 2)
                    MRAMBurstTable = DMATiming::MRAMWrite32Bursts[0];
                else
                    MRAMBurstTable = DMATiming::MRAMWrite32Bursts[1];
            }

            u32 ret = MRAMBurstTable[MRAMBurstCount++];
            return ret;
        }
        else
        {
            return (burststart ? src_n : src_s) + 8;
        }
    }
    else if (src_rgn & dst_rgn)
    {
        return src_n + dst_n + 1;
    }
    else
    {
        if (burststart)
            return src_n + dst_n;
        else
            return src_s + dst_s;
    }
}

void DMA::Run9()
{
    if (NDS.ARM9Timestamp >= NDS.ARM9Target) return;

    Executing = true;

    // add NS penalty for first accesses in burst
    bool burststart = (Running == 2);
    Running = 1;

    if (!(Cnt & (1<<26)))
    {
        while (IterCount > 0 && !Stall)
        {
            NDS.ARM9Timestamp += (UnitTimings9_16(burststart) << NDS.ARM9ClockShift);
            burststart = false;

            NDS.ARM9Write16(CurDstAddr, NDS.ARM9Read16(CurSrcAddr));

            CurSrcAddr += SrcAddrInc<<1;
            CurDstAddr += DstAddrInc<<1;
            IterCount--;
            RemCount--;

            if (NDS.ARM9Timestamp >= NDS.ARM9Target) break;
        }
    }
#ifdef LITEV_DMA_GXFIFO_FAST
    else if (IsGXFIFODMA)
    {
        // DraStic-style geometry-DMA fast path. IsGXFIFODMA already guarantees
        // src is MainRAM (0x02), dst is the fixed GXFIFO register 0x04000400, and
        // DstAddrInc==0. Reading directly from MainRAM is exactly what
        // NDS::ARM9Read32's 0x02000000 case does; calling WriteToGXFIFO (under the
        // GeometryEnabled guard) is exactly what ARM9Write32->ARM9IOWrite32->
        // GPU3D::Write32(0x400) resolves to. Timing + stall handling are untouched,
        // so this is bit-exact — it only elides the per-word address decode.
        GPU3D& gpu3d = NDS.GPU.GPU3D;
#if defined(__ANDROID__)
        // Exact fixed-GXFIFO timing specialization; default ON, but retained
        // as a launch-time A/B property for validation and regression isolation.
        static int _gxfifotiming = litevDmaPropDefaultOn("debug.litev.gxfifotiming") ? 1 : 0;
        // Candidate: keep the identical GXFIFO burst-table state transition in
        // Run9's hot loop, removing the per-word out-of-line helper call.  It
        // defaults ON after the byte-exact A/B gate; the generic path remains
        // available as the direct regression control.
        static int _gxtiminginline = litevDmaPropDefaultOn("debug.litev.gxtiminginline") ? 1 : 0;
#else
        static int _gxfifotiming = 1;
        static int _gxtiminginline = 0;
#endif
#ifdef LITEV_GXFIFO_DMA_INLINE
        // Inline the whole GXFIFO producer path into this burst loop so the packed-command decode
        // state + FIFO pointers stay in registers across the burst (no per-word cross-TU bl to
        // WriteToGXFIFO/CmdFIFOWrite). Byte-exact + cycle-exact — per-word accounting is untouched.
        // Hoisted prop read (debug.litev.gxinline) so it's not re-checked per word; default ON when
        // the flag is compiled in, so the compiled build is the fast path unless explicitly disabled.
#if defined(__ANDROID__)
        static int _gxinline = litevDmaPropDefaultOn("debug.litev.gxinline") ? 1 : 0;
#else
        static int _gxinline = 1;
#endif
#endif
        if (_gxtiminginline && _gxfifotiming && SrcAddrInc > 0)
        {
            while (IterCount > 0 && !Stall)
            {
                // Textually identical to UnitTimings9_32_GXFIFO: this route's
                // fixed destination has a constant timing index (0x1000).
                if (burststart || MRAMBurstTable[MRAMBurstCount] == 0)
                {
                    MRAMBurstCount = 0;
                    const u32 dst_n = NDS.ARM9MemTimings[0x1000][6];
                    MRAMBurstTable = (dst_n == 2) ?
                        DMATiming::MRAMRead32Bursts[0] : DMATiming::MRAMRead32Bursts[1];
                }
                NDS.ARM9Timestamp += (MRAMBurstTable[MRAMBurstCount++] << NDS.ARM9ClockShift);
                burststart = false;

                u32 val = *(u32*)&NDS.MainRAM[CurSrcAddr & NDS.MainRAMMask];
                if (gpu3d.GeometryEnabled)
                {
#ifdef LITEV_GXFIFO_DMA_INLINE
                    if (_gxinline) gpu3d.WriteToGXFIFO_Inline(val);
                    else           gpu3d.WriteToGXFIFO(val);
#else
                    gpu3d.WriteToGXFIFO(val);
#endif
                }

                CurSrcAddr += SrcAddrInc<<2;
                CurDstAddr += DstAddrInc<<2;
                IterCount--;
                RemCount--;

                if (NDS.ARM9Timestamp >= NDS.ARM9Target) break;
            }
        }
        else while (IterCount > 0 && !Stall)
        {
            const u32 timing = (_gxfifotiming && SrcAddrInc > 0) ?
                UnitTimings9_32_GXFIFO(burststart) : UnitTimings9_32(burststart);
            NDS.ARM9Timestamp += (timing << NDS.ARM9ClockShift);
            burststart = false;

            u32 val = *(u32*)&NDS.MainRAM[CurSrcAddr & NDS.MainRAMMask];
            if (gpu3d.GeometryEnabled)
            {
#ifdef LITEV_GXFIFO_DMA_INLINE
                if (_gxinline) gpu3d.WriteToGXFIFO_Inline(val);
                else           gpu3d.WriteToGXFIFO(val);
#else
                gpu3d.WriteToGXFIFO(val);
#endif
            }

            CurSrcAddr += SrcAddrInc<<2;
            CurDstAddr += DstAddrInc<<2;   // DstAddrInc==0 (fixed dst)
            IterCount--;
            RemCount--;

            if (NDS.ARM9Timestamp >= NDS.ARM9Target) break;
        }
    }
#endif
    else
    {
        while (IterCount > 0 && !Stall)
        {
            NDS.ARM9Timestamp += (UnitTimings9_32(burststart) << NDS.ARM9ClockShift);
            burststart = false;

            NDS.ARM9Write32(CurDstAddr, NDS.ARM9Read32(CurSrcAddr));

            CurSrcAddr += SrcAddrInc<<2;
            CurDstAddr += DstAddrInc<<2;
            IterCount--;
            RemCount--;

            if (NDS.ARM9Timestamp >= NDS.ARM9Target) break;
        }
    }

    Executing = false;
    Stall = false;

    if (RemCount)
    {
        if (IterCount == 0)
        {
            Running = 0;
            NDS.ResumeCPU(0, 1<<Num);

            if (StartMode == 0x07)
                NDS.GPU.GPU3D.CheckFIFODMA();
        }

        return;
    }

    if (!(Cnt & (1<<25)))
        Cnt &= ~(1<<31);

    if (Cnt & (1<<30))
        NDS.SetIRQ(0, IRQ_DMA0 + Num);

    Running = 0;
    InProgress = false;
    NDS.ResumeCPU(0, 1<<Num);

    if (StartMode == 0x05)
        NDS.NDSCartSlots[0]->CheckDMA(0);
}

void DMA::Run7()
{
    if (NDS.ARM7Timestamp >= NDS.ARM7Target) return;

    Executing = true;

    // add NS penalty for first accesses in burst
    bool burststart = (Running == 2);
    Running = 1;

    if (!(Cnt & (1<<26)))
    {
        while (IterCount > 0 && !Stall)
        {
            NDS.ARM7Timestamp += UnitTimings7_16(burststart);
            burststart = false;

            NDS.ARM7Write16(CurDstAddr, NDS.ARM7Read16(CurSrcAddr));

            CurSrcAddr += SrcAddrInc<<1;
            CurDstAddr += DstAddrInc<<1;
            IterCount--;
            RemCount--;

            if (NDS.ARM7Timestamp >= NDS.ARM7Target) break;
        }
    }
    else
    {
        while (IterCount > 0 && !Stall)
        {
            NDS.ARM7Timestamp += UnitTimings7_32(burststart);
            burststart = false;

            NDS.ARM7Write32(CurDstAddr, NDS.ARM7Read32(CurSrcAddr));

            CurSrcAddr += SrcAddrInc<<2;
            CurDstAddr += DstAddrInc<<2;
            IterCount--;
            RemCount--;

            if (NDS.ARM7Timestamp >= NDS.ARM7Target) break;
        }
    }

    Executing = false;
    Stall = false;

    if (RemCount)
    {
        if (IterCount == 0)
        {
            Running = 0;
            NDS.ResumeCPU(1, 1<<Num);
        }

        return;
    }

    if (!(Cnt & (1<<25)))
        Cnt &= ~(1<<31);

    if (Cnt & (1<<30))
        NDS.SetIRQ(1, IRQ_DMA0 + Num);

    Running = 0;
    InProgress = false;
    NDS.ResumeCPU(1, 1<<Num);

    if (StartMode == 0x12)
        NDS.NDSCartSlots[0]->CheckDMA(1);
}

void DMA::Run()
{
    if (!Running) return;
    if (CPU == 0) return Run9();
    else          return Run7();
}

}