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

#ifndef LOCALMP_H
#define LOCALMP_H

#include "types.h"
#include "Platform.h"
#include "MPInterface.h"

namespace melonDS
{
struct MPStatusData
{
    u16 ConnectedBitmask; // bitmask of which instances are ready to send/receive packets
    u32 PacketWriteOffset;
    u32 ReplyWriteOffset;
    u16 MPHostinst; // instance ID from which the last CMD frame was sent
    u16 MPReplyBitmask;   // bitmask of which clients replied in time
};

constexpr u32 kPacketQueueSize = 0x10000;
constexpr u32 kReplyQueueSize = 0x10000;
constexpr u32 kMaxFrameSize = 0x948;

class LocalMP : public MPInterface
{
public:
    LocalMP() noexcept;
    LocalMP(const LocalMP&) = delete;
    LocalMP& operator=(const LocalMP&) = delete;
    LocalMP(LocalMP&& other) = delete;
    LocalMP& operator=(LocalMP&& other) = delete;
    ~LocalMP() noexcept;

    void Process() {}

    void Begin(int inst);
    void End(int inst);

    // Read-only observability for the headless MP test harness (no timing effect):
    // ConnectedBitmask == which instances have called Begin (associated); the
    // counters tally MP traffic so the harness can tell a live link from a dead one.
    [[nodiscard]] u16 ObserveConnectedBitmask() const noexcept override { return MPStatus.ConnectedBitmask; }
    [[nodiscard]] u64 ObserveCmdCount() const noexcept override { return CmdCount; }
    [[nodiscard]] u64 ObserveReplyCount() const noexcept override { return ReplyCount; }
    [[nodiscard]] u64 ObservePacketCount() const noexcept override { return PacketCount; }

    int SendPacket(int inst, u8* data, int len, u64 timestamp);
    int RecvPacket(int inst, u8* data, u64* timestamp);
    int SendCmd(int inst, u8* data, int len, u64 timestamp);
    int SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid);
    int SendAck(int inst, u8* data, int len, u64 timestamp);
    int RecvHostPacket(int inst, u8* data, u64* timestamp);
    u16 RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask);

private:
    void FIFORead(int inst, int fifo, void* buf, int len) noexcept;
    void FIFOWrite(int inst, int fifo, void* buf, int len) noexcept;
    int SendPacketGeneric(int inst, u32 type, u8* packet, int len, u64 timestamp) noexcept;
    int RecvPacketGeneric(int inst, u8* packet, bool block, u64* timestamp) noexcept;

    Platform::Mutex* MPQueueLock;
    MPStatusData MPStatus {};
    u8 MPPacketQueue[kPacketQueueSize] {};
    u8 MPReplyQueue[kReplyQueueSize] {};
    u32 PacketReadOffset[16] {};
    u32 ReplyReadOffset[16] {};

    int LastHostID = -1;
    Platform::Semaphore* SemPool[32] {};

    // Observability counters (harness only; incremented under MPQueueLock).
    u64 CmdCount = 0;      // type-1 (host CMD) frames sent
    u64 ReplyCount = 0;    // type-2 (client reply) frames sent
    u64 PacketCount = 0;   // all frames sent
};
}

#endif // LOCALMP_H
