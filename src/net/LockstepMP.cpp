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
#include "../Platform.h"
#include <chrono>
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

static long long NowNs() { return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }

bool LockstepMP::StatsOn()
{
    if (St.On < 0)
    {
        const char* e = getenv("LITEV_MP_STATS");
        St.On = e && atoi(e);
#ifdef __ANDROID__
        char v[92] = {0};
        if (!St.On && __system_property_get("debug.litev.mpstats", v) > 0) St.On = atoi(v);
#endif
    }
    return St.On > 0;
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
    // (no clearing the reply queue here: what is in it now depends on thread timing; RecvReplies
    // drops replies sent before this CMD instead)
    CmdTime[inst] = Now(inst);
    if (StatsOn())
    {
        St.T0 = NowNs(); St.T1 = St.T2 = 0;
        for (int i = 0; i < kMaxInst; i++)
            if (i != inst && (Members & (1 << i)))
            {
                long long lead = (long long)Now(i) - (long long)Now(inst);
                St.Lead[lead < 0 ? 0 : lead < (long long)kHostDelay / 2 ? 1 : lead < (long long)kHostDelay ? 2 : 3]++;
                break;
            }
    }
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
    if (St.On > 0 && St.T1 && !St.T2) St.T2 = NowNs();
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
#ifdef LITEV_MP_HOSTDELAY
    if (now < kHostDelay) return 0;
#endif
    u64 visible = now - kHostDelay;
    Log(inst, "RecvHostWait", HostID, visible);
    // Ties at exactly `visible` go by console id: a peer at `visible` with a higher id counts as
    // past it, and what it sends at `visible` shows only from our next poll. Two clients at the
    // same clock waiting for a host (none yet, kHostDelay 0: the 3-player app froze when the in-game
    // host started syncing) then never wait on each other; the lower id goes first.
    auto past = [&](int i) { u64 t = Now(i); return t > visible || (t == visible && i > inst); };
    auto shows = [&](const Packet& p) { return p.Time < visible || (p.Time == visible && p.Sender < inst); };
    WaitFor(lk, [&] {
        if (!FromHost[inst].empty() && shows(FromHost[inst].front())) return true;
        // (a console that was the last host itself waits on the others, never on its own clock)
        if (HostID >= 0 && HostID != inst) return past(HostID);
        for (int i = 0; i < kMaxInst; i++)
            if (i != inst && (Members & (1 << i)) && !past(i)) return false;
        return true;
    });
    while (!FromHost[inst].empty() && FromHost[inst].front().Time < BeginTime[inst]) FromHost[inst].pop_front();
    if (FromHost[inst].empty() || !shows(FromHost[inst].front())) { Log(inst, "RecvHost", 0, 0); return 0; }

    Packet p = std::move(FromHost[inst].front());
    FromHost[inst].pop_front();
    if (St.On > 0 && p.Type == 1 && St.T0 && !St.T1) St.T1 = NowNs();
    int len = (int)p.Data.size();
    if (len) memcpy(data, p.Data.data(), len);
#ifdef LITEV_MP_HOSTDELAY
    // advanced by the delay, as for regular frames (the client syncs its Wi-Fi clock to it)
    if (timestamp) *timestamp = p.Timestamp + (now - p.Time) * 1000000 / 33513982;
#else
    if (timestamp) *timestamp = p.Timestamp;
#endif
    Log(inst, "RecvHost", len, p.Time);
    return len;
}

u16 LockstepMP::RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask)
{
    std::unique_lock<std::mutex> lk(Lock);
    u16 ret = 0;
    u16 replied = (1 << inst);
    u64 deadline = Now(inst) + kDelay;
    if (St.On > 0) St.Enter = NowNs();
    auto stats = [&] {
        if (St.On <= 0 || !St.T0) return;
        long long t3 = NowNs();
        St.Wait += t3 - St.Enter;
        if (St.T1 && St.T2)
        {
            St.Deliver += St.T1 - St.T0; St.Reply += St.T2 - St.T1;
            St.Wake += t3 - std::max(St.T2, St.Enter);
            St.Notify += St.T2 > St.Enter ? 1 : 0;
        }
        St.T0 = 0;
        if (++St.N == 600)
        {
            Platform::Log(Platform::LogLevel::Info, "LITEV_MPSTATS 600 CMDs: host wait %.0f us/CMD | CMD sent->client has it %.0f us, ->reply sent %.0f us, reply->host running %.0f us | host already waiting at reply %.0f%% | client lead at CMD: behind %u, <D/2 %u, <D %u, >=D %u\n",
                St.Wait / 600e3, St.Deliver / 600e3, St.Reply / 600e3, St.Wake / 600e3, St.Notify / 6.0, St.Lead[0], St.Lead[1], St.Lead[2], St.Lead[3]);
            St.Wait = St.Deliver = St.Reply = St.Wake = St.Notify = 0; St.N = 0;
            for (auto& l : St.Lead) l = 0;
        }
    };

    for (;;)
    {
        // which clients are past the deadline, BEFORE reading the queue: a reply sent before the
        // deadline is queued before its sender's clock passes it, so it is then always seen
        u16 passed = 0;
        for (int i = 0; i < kMaxInst; i++)
            if ((Members & (1 << i)) && Now(i) >= deadline) passed |= (1 << i);

        // Only replies sent in [this CMD, deadline) count, the first one per client. Which of them
        // are already queued depends on thread timing, so the result is taken only from a set that
        // cannot change any more: every client's first reply once all replied or passed the
        // deadline, or, as soon as every addressed AID has replied, only those replies. Nothing
        // else is consumed; replies sent before this CMD answer an older one and are dropped.
        std::deque<Packet>& q = Replies[inst];
        q.erase(std::remove_if(q.begin(), q.end(), [&](const Packet& p) {
            if (p.Time >= CmdTime[inst] && p.Sender != inst) return false;
            Log(inst, "ReplyOld", 0, CmdTime[inst] - p.Time);
            return true;
        }), q.end());
        int first[kMaxInst];
        std::fill(std::begin(first), std::end(first), -1);
        u16 seen = (1 << inst), seenAids = 0, addressedSenders = 0;
        for (int k = 0; k < (int)q.size(); k++)
        {
            const Packet& p = q[k];
            if (p.Time >= deadline || first[p.Sender] >= 0 || p.Timestamp < timestamp - 32) continue; // late / not first / stale
            first[p.Sender] = k;
            seen |= (1 << p.Sender);
            if (!p.Data.empty())
            {
                u32 aid = p.Type >> 16;
                seenAids |= (1 << aid);
                if (aidmask & (1 << aid)) addressedSenders |= (1 << p.Sender);
            }
        }
        const bool addressed = (seenAids & aidmask) == aidmask;
        if (addressed || (seen & Members) == Members || ((seen | passed) & Members) == Members || Stopped)
        {
            u16 take = addressed ? addressedSenders : seen;
            for (int i = 0; i < kMaxInst; i++)
            {
                int k = first[i];
                if (k < 0 || !(take & (1 << i))) continue;
                Packet p = std::move(q[k]);
                q.erase(q.begin() + k);
                for (int j = 0; j < kMaxInst; j++) if (first[j] > k) first[j]--;
                if (!p.Data.empty())
                {
                    u32 aid = p.Type >> 16;
                    memcpy(&data[(aid - 1) * 1024], p.Data.data(), std::min<size_t>(p.Data.size(), 1024));
                    ret |= (1 << aid);
                }
                replied |= (1 << p.Sender);
            }
            if (!addressed && ((replied & Members) != Members))
            {
                u64 other = 0; for (int i = 0; i < kMaxInst; i++) if (i != inst && (Members & (1 << i))) other = Now(i);
                Log(inst, "RecvRepliesGiveUp", (int)q.size(), other - deadline);
            }
            Log(inst, "RecvReplies", ret, replied); stats(); return ret;
        }

        Changed.wait_for(lk, std::chrono::microseconds(100));
    }
}

}
