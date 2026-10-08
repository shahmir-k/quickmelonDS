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

#include <string.h>
#include <algorithm>
#include <cstdlib>

#include "LockstepMP.h"
#ifdef __ANDROID__
#include <sys/system_properties.h>
#endif

namespace melonDS
{

void LockstepMP::Begin(int inst)
{
    std::lock_guard<std::mutex> lk(Lock);
    Connected |= (1 << inst);
    BeginTime[inst] = Now(inst);
    Changed.notify_all();
}

void LockstepMP::Stop()
{
    std::lock_guard<std::mutex> lk(Lock);
    Stopped = true;
    Changed.notify_all();
}

void LockstepMP::End(int inst)
{
    std::lock_guard<std::mutex> lk(Lock);
    Connected &= ~(1 << inst);
    Changed.notify_all();
}

void LockstepMP::Log(int inst, const char* call, int result, u64 extra)
{
    if (!Trace[inst])
    {
        // resolved once: this runs on every link call
        static const char* dir = [] {
            const char* d = getenv("LITEV_MP_TRACE");
#ifdef __ANDROID__
            // apps get no environment: debug.litev.mptrace=<dir the app can write>
            static char propDir[92] = {0};
            if (!d && __system_property_get("debug.litev.mptrace", propDir) > 0 && propDir[0]) d = propDir;
#endif
            return d;
        }();
        if (!dir) return;
        char path[512];
        snprintf(path, sizeof(path), "%s/inst%d.txt", dir, inst);
        Trace[inst] = fopen(path, "w");
        if (!Trace[inst]) return;
    }
    fprintf(Trace[inst], "%llu %s %d %llu\n", (unsigned long long)Now(inst), call, result, (unsigned long long)extra);
    fflush(Trace[inst]);
}

bool LockstepMP::PeersReached(int inst, u64 time) const
{
    for (int i = 0; i < kMaxInst; i++)
    {
        if (i == inst || !(Members & (1 << i))) continue;
        // strictly past: a console still at `time` can send more frames stamped `time`
        if (Now(i) <= time) return false;
    }
    return true;
}

template <typename Pred> void LockstepMP::WaitFor(std::unique_lock<std::mutex>& lk, Pred pred)
{
    auto start = std::chrono::steady_clock::now();
    bool reported = false;
    while (!pred() && !Stopped)
    {
        Changed.wait_for(lk, std::chrono::microseconds(100));
        if (!reported && std::chrono::steady_clock::now() - start > std::chrono::seconds(1))
        {
            reported = true; // diagnostics: a wait this long is a deadlock
            fprintf(stderr, "LockstepMP: long wait: host=%d connected=%x clocks:", HostID, Connected);
            for (int i = 0; i < kMaxInst; i++)
                if (Members & (1 << i))
                    fprintf(stderr, " [%d]=%llu reg=%zu fromhost=%zu replies=%zu", i, (unsigned long long)Now(i),
                            Regular[i].size(), FromHost[i].size(), Replies[i].size());
            fprintf(stderr, "\n");
        }
    }
}

void LockstepMP::Broadcast(int inst, u32 type, u8* data, int len, u64 timestamp, std::deque<Packet>* queues)
{
    for (int i = 0; i < kMaxInst; i++)
    {
        if (i == inst || !(Members & (1 << i))) continue;
        u64 now = Now(inst);
        // a console with Wi-Fi off can only Begin at or after its current time, and then never
        // reads anything sent before that
        if (!(Connected & (1 << i)))
            while (!queues[i].empty() && queues[i].front().Time < Now(i)) queues[i].pop_front();
        queues[i].push_back({inst, type, timestamp, now, std::vector<u8>(data, data + len)});
    }
}

int LockstepMP::SendPacket(int inst, u8* data, int len, u64 timestamp)
{
    std::lock_guard<std::mutex> lk(Lock);
    Log(inst, "SendPacket", len, timestamp);
    PacketCount++;
    Broadcast(inst, 0, data, len, timestamp, Regular);
    Changed.notify_all();
    return len;
}

int LockstepMP::RecvPacket(int inst, u8* data, u64* timestamp)
{
    std::unique_lock<std::mutex> lk(Lock);
    u64 now = Now(inst);
    if (now < kDelay) return 0;
    u64 visible = now - kDelay;
    Log(inst, "RecvPacketWait", 0, visible);

    // everything a peer sent up to `visible` must exist before we look
    WaitFor(lk, [&] { return PeersReached(inst, visible); });

    // earliest visible frame by (send time, sender), first-sent among equals
    std::deque<Packet>& q = Regular[inst];
    int best = -1;
    while (!q.empty() && q.front().Time < BeginTime[inst]) q.pop_front(); // sent before our Begin
    for (int i = 0; i < (int)q.size(); i++)
    {
        if (q[i].Time > visible || q[i].Time < BeginTime[inst]) continue;
        if (best < 0 || q[i].Time < q[best].Time || (q[i].Time == q[best].Time && q[i].Sender < q[best].Sender))
            best = i;
    }
    if (best < 0) return 0;

    Packet& p = q[best];
    int len = (int)p.Data.size();
    if (len) memcpy(data, p.Data.data(), len);
    // The frame arrives (now - p.Time) after it was sent; advance its wifi timestamp (us) by that,
    // or a client that syncs its wifi clock to it (association response) ends up permanently that
    // far behind the host, and every MP exchange is then late by as much.
    if (timestamp) *timestamp = p.Timestamp + (now - p.Time) * 1000000 / 33513982;
    Log(inst, "RecvPacket", len, p.Time);
    q.erase(q.begin() + best);
    return len;
}

int LockstepMP::SendCmd(int inst, u8* data, int len, u64 timestamp)
{
    std::lock_guard<std::mutex> lk(Lock);
    Log(inst, "SendCmd", len, timestamp);
    PacketCount++; CmdCount++;
    HostID = inst;
    Replies[inst].clear();
    Broadcast(inst, 1, data, len, timestamp, FromHost);
    Changed.notify_all();
    return len;
}

int LockstepMP::SendAck(int inst, u8* data, int len, u64 timestamp)
{
    std::lock_guard<std::mutex> lk(Lock);
    Log(inst, "SendAck", len, timestamp);
    PacketCount++;
    Broadcast(inst, 3, data, len, timestamp, FromHost);
    Changed.notify_all();
    return len;
}

int LockstepMP::SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid)
{
    std::lock_guard<std::mutex> lk(Lock);
    Log(inst, "SendReply", len, timestamp);
    PacketCount++; ReplyCount++;
    if (HostID >= 0 && HostID != inst && (Members & (1 << HostID)))
        Replies[HostID].push_back({inst, 2u | ((u32)aid << 16), timestamp, Now(inst), std::vector<u8>(data, data + len)});
    Changed.notify_all();
    return len;
}

int LockstepMP::RecvHostPacket(int inst, u8* data, u64* timestamp)
{
    std::unique_lock<std::mutex> lk(Lock);
    // The next host frame if the host had sent it kHostDelay before our current time, else
    // nothing. The client sees the host's frames, like its beacons, uniformly later than they were
    // sent, so it can run up to kHostDelay ahead of the host without waiting on it (each wait is a
    // thread hand-off; Netplay makes ~13 per frame). A frame never arrives before it was sent (a
    // client that took a CMD sent 1.7 ms in its future synced its Wi-Fi clock forward to the CMD's
    // timestamp and dropped the session). Host frames are queued in send order before the host's
    // clock moves on, so once the host is strictly past `visible` the answer can no longer change.
    // No deadlock: a host waiting on our reply waits for our clock to pass its own + kDelay
    // (> kHostDelay), and anything it sent before that is visible to us by then.
    while (!FromHost[inst].empty() && FromHost[inst].front().Time < BeginTime[inst]) FromHost[inst].pop_front();
    u64 now = Now(inst);
    if (now < kHostDelay) return 0;
    u64 visible = now - kHostDelay;
    Log(inst, "RecvHostWait", HostID, visible);
    WaitFor(lk, [&] {
        // (a console that was the last host itself waits on the others, never on its own clock)
        return (!FromHost[inst].empty() && FromHost[inst].front().Time <= visible)
            || (HostID >= 0 && HostID != inst ? Now(HostID) > visible : PeersReached(inst, visible));
    });
    while (!FromHost[inst].empty() && FromHost[inst].front().Time < BeginTime[inst]) FromHost[inst].pop_front();
    if (FromHost[inst].empty() || FromHost[inst].front().Time > visible) { Log(inst, "RecvHost", 0, 0); return 0; }

    Packet p = std::move(FromHost[inst].front());
    FromHost[inst].pop_front();
    int len = (int)p.Data.size();
    if (len) memcpy(data, p.Data.data(), len);
    // advanced by the delay, as for regular frames (the client syncs its Wi-Fi clock to it)
    if (timestamp) *timestamp = p.Timestamp + (now - p.Time) * 1000000 / 33513982;
    Log(inst, "RecvHost", len, p.Time);
    return len;
}

u16 LockstepMP::RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask)
{
    std::unique_lock<std::mutex> lk(Lock);
    u16 ret = 0;
    u16 replied = (1 << inst);
    u64 deadline = Now(inst) + kDelay;

    for (;;)
    {
        // which clients are past the deadline, BEFORE reading the queue: a reply sent before the
        // deadline is queued before its sender's clock passes it, so it is then always seen
        u16 passed = 0;
        for (int i = 0; i < kMaxInst; i++)
            if ((Members & (1 << i)) && Now(i) >= deadline) passed |= (1 << i);

        std::deque<Packet>& q = Replies[inst];
        while (!q.empty())
        {
            Packet p = std::move(q.front());
            q.pop_front();
            if (p.Sender == inst || p.Timestamp < timestamp - 32) { Log(inst, "ReplyStale", (int)(timestamp - p.Timestamp), p.Time); continue; } // stale
            if (p.Time >= deadline) { Log(inst, "ReplyLate", 0, p.Time - deadline); continue; } // too late for this CMD
            if (!p.Data.empty())
            {
                u32 aid = p.Type >> 16;
                memcpy(&data[(aid - 1) * 1024], p.Data.data(), std::min<size_t>(p.Data.size(), 1024));
                ret |= (1 << aid);
            }
            replied |= (1 << p.Sender);
        }

        if ((replied & Members) == Members || (ret & aidmask) == aidmask || Stopped)
        { Log(inst, "RecvReplies", ret, replied); return ret; }

        // a client that has run kDelay past the CMD without replying never will for this CMD
        if (((replied | passed) & Members) == Members)
        {
            // diagnostics: client clocks vs the deadline, and what is queued
            u64 other = 0; for (int i = 0; i < kMaxInst; i++) if (i != inst && (Members & (1 << i))) other = Now(i);
            Log(inst, "RecvRepliesGiveUp", (int)q.size(), other - deadline);
            Log(inst, "RecvReplies", ret, replied); return ret;
        }

        Changed.wait_for(lk, std::chrono::microseconds(100));
    }
}

}
