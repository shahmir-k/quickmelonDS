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

#ifndef MELONDS_JITBLOCK_H
#define MELONDS_JITBLOCK_H

#include "types.h"
#include "TinyVector.h"

namespace melonDS
{
typedef void (*JitBlockEntry)();

#ifdef LITEV_JIT_LINK
// liteDS-v2 Unit 4 (direct block linking). Offsets are into the JIT RX region,
// relative to Compiler::GetRXBase() (the same base SubEntryOffset/AddEntryOffset
// use), so a site/target can be resolved to an absolute code address anywhere.
//
// OutgoingLink: a patchable `B` slot inside THIS block's code that (when linked)
// jumps straight into TargetAddr's block. Persisted in the JitBlock so a restored
// block (same RX, same offsets) can re-establish its links without recompiling.
struct OutgoingLink { u32 PatchOffset; u32 TargetAddr; };
// LinkSite: a patch slot in SOURCE block's code that currently points at (an entry
// of) the block that owns this Incoming record. Rewritten back to the dispatcher
// when that block dies.
struct LinkSite { u32 SourceBlockAddr; u32 PatchOffset; };
#endif

class JitBlock
{
public:
    JitBlock(u32 num, u32 literalHash, u32 numAddresses, u32 numLiterals)
    {
        Num = num;
        NumAddresses = numAddresses;
        NumLiterals = numLiterals;
        Data.SetLength(numAddresses * 2 + numLiterals);
    }

    u32 StartAddr;
    u32 StartAddrLocal;
    u32 InstrHash, LiteralHash;
    u8 Num;
    u16 NumAddresses;
    u16 NumLiterals;

    JitBlockEntry EntryPoint;

#ifdef LITEV_JIT_LINK
    // Up to two outgoing static exits (a conditional block has taken + fall-through).
    u8 NumOutgoing = 0;
    OutgoingLink Outgoing[2];
    // Sites in other (or this) block(s) currently linked INTO this block.
    TinyVector<LinkSite> Incoming;
#endif

    const u32* AddressRanges() const { return &Data[0]; }
    u32* AddressRanges() { return &Data[0]; }
    const u32* AddressMasks() const { return &Data[NumAddresses]; }
    u32* AddressMasks() { return &Data[NumAddresses]; }
    const u32* Literals() const { return &Data[NumAddresses * 2]; }
    u32* Literals() { return &Data[NumAddresses * 2]; }

private:
    TinyVector<u32> Data;
};
}

#endif //MELONDS_JITBLOCK_H
