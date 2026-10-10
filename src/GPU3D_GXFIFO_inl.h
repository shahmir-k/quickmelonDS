// GPU3D_GXFIFO_inl.h — single-source inline definitions of the GXFIFO producer path.
//
// LITEV_GXFIFO_DMA_INLINE lever (2026-09-16 GX-pipeline drill): the geometry-DMA path
// (DMA::Run9, LITEV_DMA_GXFIFO_FAST) calls GPU3D::WriteToGXFIFO once PER 32-bit word of the
// upload, which in turn calls CmdFIFOWrite. Because IPO/LTO is OFF for the .dev/Debug build,
// those two calls are NOT inlined across translation units, so every word pays two `bl` layers
// plus reloads of NumCommands/CurCommand/ParamCount/TotalParams (and the FIFO pointers) from
// `this`. Moving the bodies into this header lets DMA.cpp inline the whole producer path into
// the burst loop, keeping the packed-command decode state in registers across the burst.
//
// BYTE-EXACT BY CONSTRUCTION: these are the *same statements* as the out-of-line
// GPU3D::WriteToGXFIFO / GPU3D::CmdFIFOWrite (GPU3D.cpp). The public functions are thin wrappers
// over these, so there is ONE copy of the logic — no duplication, no drift. The decode state
// stays in the same member fields with the same lifetime, so a command spanning a DMA burst
// boundary (or DMA+CPU writes) decodes identically. Nothing is batched; per-word cycle
// accounting in DMA::Run9 is untouched → cycle-exact (category-A / MP-safe).
//
// Included by DMA.cpp (where the inline matters) and GPU3D.cpp (so the public wrappers delegate
// to the same code). NDS must be a COMPLETE type here (GXFIFOStall/Unstall) — that is why these
// bodies live here and not in GPU3D.h, which only forward-declares class NDS.
#ifndef LITEV_GPU3D_GXFIFO_INL_H
#define LITEV_GPU3D_GXFIFO_INL_H

#include "GPU3D.h"
#include "NDS.h"
#ifdef LITEV_A9HLE_GXCHECK
#include "ARM9HLE.h"
#endif

namespace melonDS
{

// GPU3D.cpp owns the definition; give it external linkage so the inline bodies below (compiled
// into other TUs, e.g. DMA.cpp) can reference the shared command-length table.
extern const u8 CmdNumParams[256];

__attribute__((always_inline)) inline void GPU3D::CmdFIFOWrite_Inline(const CmdFIFOEntry& entry) noexcept
{
#ifdef LITEV_GXFIFO_UNIFIED
    if (FifoN == 0 && PipeN < 4)
    {
        CmdQ[(CmdQHead + PipeN) & 511] = entry;
        PipeN++;
    }
    else
    {
        if (FifoN >= 256)
        {
            // store it to the stall queue. stall the system.
            // The JIT honours a GX stall only at block end, so one block can keep
            // writing past the 64-entry stall queue (Sims 3 in-game: ~90). FIFO::Write
            // would drop the entry and desync the command stream (lost MTX_MULT params
            // -> garbage projection -> no 3D). Run the geometry engine ahead instead.
            while (CmdStallQueue.IsFull() && !PipeEmpty()) ExecuteCommand();
            CmdStallQueue.Write(entry);
            NDS.GXFIFOStall();
            return;
        }

        CmdQ[(CmdQHead + PipeN + FifoN) & 511] = entry;
        FifoN++;
    }
#else
    if (CmdFIFO.IsEmpty() && !CmdPIPE.IsFull())
    {
        CmdPIPE.Write(entry);
    }
    else
    {
        if (CmdFIFO.IsFull())
        {
            // store it to the stall queue. stall the system.
            // The JIT honours a GX stall only at block end, so one block can keep
            // writing past the 64-entry stall queue (Sims 3 in-game: ~90). FIFO::Write
            // would drop the entry and desync the command stream (lost MTX_MULT params
            // -> garbage projection -> no 3D). Run the geometry engine ahead instead.
            while (CmdStallQueue.IsFull() && !PipeEmpty()) ExecuteCommand();
            CmdStallQueue.Write(entry);
            NDS.GXFIFOStall();
            return;
        }

        CmdFIFO.Write(entry);
    }
#endif

    GXStat |= (1<<27);

    if (entry.Command == 0x11 || entry.Command == 0x12)
    {
        GXStat |= (1<<14); // push/pop matrix
        NumPushPopCommands++;
    }
    else if (entry.Command == 0x70 || entry.Command == 0x71 || entry.Command == 0x72)
    {
        GXStat |= (1<<0); // box/pos/vec test
        NumTestCommands++;
    }
}

__attribute__((always_inline)) inline void GPU3D::WriteToGXFIFO_Inline(u32 val) noexcept
{
#ifdef LITEV_A9HLE_GXCHECK
    if (A9HLE::GxTap) A9HLE::GxTap->push_back(val);
#endif
    if (NumCommands == 0)
    {
        NumCommands = 4;
        CurCommand = val;
        ParamCount = 0;
        TotalParams = CmdNumParams[CurCommand & 0xFF];

        if (TotalParams > 0) return;
    }
    else
        ParamCount++;

    for (;;)
    {
        if ((CurCommand & 0xFF) || (NumCommands == 4 && CurCommand == 0))
        {
            CmdFIFOEntry entry;
            entry.Command = CurCommand & 0xFF;
            entry.Param = val;
            CmdFIFOWrite_Inline(entry);
        }

        if (ParamCount >= TotalParams)
        {
            CurCommand >>= 8;
            NumCommands--;
            if (NumCommands == 0) break;

            ParamCount = 0;
            TotalParams = CmdNumParams[CurCommand & 0xFF];
        }
        if (ParamCount < TotalParams)
            break;
    }
}

}

#endif // LITEV_GPU3D_GXFIFO_INL_H
