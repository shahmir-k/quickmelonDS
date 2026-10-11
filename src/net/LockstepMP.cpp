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
#include "xxhash/xxhash.h"
#include "../Platform.h"
#include "../NDS.h"
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
    ClientHost[inst] = -1;
    NotifyAll();
}

void LockstepMP::SetWake(int inst, NDS& nds)
{
    Console[inst] = &nds;
#ifdef LITEV_MP_CLOCKWAKE
    nds.MPWakeAt = &WakeAt[inst];
    nds.MPWake = [this, inst] { Wake(inst); };
#endif
#ifdef LITEV_MP_POLL_INLINE
    nds.MPHostPoll = &HostFrameMaybe; nds.MPHostPollCtx = this; nds.MPHostPollInst = inst;
#endif
}

#ifdef LITEV_MP_CLOCKWAKE
void LockstepMP::Wake(int inst)
{
#ifdef LITEV_MP_FUTEX
    WakeLocked(inst);   // (no Lock: WaitersOn / SleepSeq are atomic)
#else
    std::lock_guard<std::mutex> lk(Lock);
    WakeLocked(inst);
#endif
}

#ifdef LITEV_MP_FUTEX
}
#if defined(__linux__)
#include <linux/futex.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <ctime>
void melonDS::LockstepMP::FutexWait(std::atomic<u32>* a, u32 v, u32 us)
{
    timespec ts{(time_t)(us / 1000000), (long)(us % 1000000) * 1000};
    syscall(SYS_futex, (u32*)a, FUTEX_WAIT_PRIVATE, v, &ts, nullptr, 0);
}
void melonDS::LockstepMP::FutexWake(std::atomic<u32>* a) { syscall(SYS_futex, (u32*)a, FUTEX_WAKE_PRIVATE, 1, nullptr, nullptr, 0); }
#elif defined(__APPLE__)
extern "C" int __ulock_wait(uint32_t op, void* addr, uint64_t value, uint32_t timeout_us);
extern "C" int __ulock_wake(uint32_t op, void* addr, uint64_t wake_value);
void melonDS::LockstepMP::FutexWait(std::atomic<u32>* a, u32 v, u32 us) { __ulock_wait(1 /*UL_COMPARE_AND_WAIT*/, (void*)a, v, us); }
void melonDS::LockstepMP::FutexWake(std::atomic<u32>* a) { __ulock_wake(1, (void*)a, 0); }
#else
#error "LITEV_MP_FUTEX: Linux/Android or macOS"
#endif
namespace melonDS {
#endif
#endif

void LockstepMP::Stop()
{
    std::lock_guard<std::mutex> lk(Lock);
    Stopped = true;
    NotifyAll();
}

void LockstepMP::End(int inst)
{
    std::lock_guard<std::mutex> lk(Lock);
    Connected &= ~(1 << inst);
    NotifyAll();
}

const char* LockstepMP::TraceDir()
{
    // resolved once
    static const char* dir = [] {
            const char* d = getenv("LITEV_MP_TRACE");
#ifdef __ANDROID__
            // apps get no environment: debug.litev.mptrace=<dir the app can write>
            static char propDir[92] = {0};
            if (!d && __system_property_get("debug.litev.mptrace", propDir) > 0 && propDir[0]) d = propDir;
#endif
            return d;
    }();
    return dir;
}

void LockstepMP::LogImpl(int inst, const char* call, int result, u64 extra)
{
    if (!Trace[inst])
    {
        const char* dir = TraceDir();
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

void LockstepMP::TLogImpl(int inst, const char* ev)
{
    static const char* dir = getenv("LITEV_MP_TL");
    if (!dir) return;
    if (!TL[inst])
    {
        char path[512];
        snprintf(path, sizeof(path), "%s/tl%d.txt", dir, inst);
        TL[inst] = fopen(path, "w");
        if (!TL[inst]) return;
    }
    fprintf(TL[inst], "%lld %s %llu %llu\n", NowNs(), ev, (unsigned long long)Now(inst), (unsigned long long)(HostID >= 0 ? Now(HostID) : 0));
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

template <typename Pred, typename Need> void LockstepMP::WaitFor(std::unique_lock<std::mutex>& lk, Pred pred, Need need, int inst, int kind)
{
    auto start = std::chrono::steady_clock::now();
    bool reported = false;
    const bool stats = St.On > 0;
    if (stats) WS[inst].Calls[kind]++;
    if (pred() || Stopped) return;
    if (stats) WS[inst].Blocked[kind]++;
    TLog(inst, kind == 0 ? "B_pkt" : "B_host");
    while (!pred() && !Stopped)
    {
#ifdef LITEV_MP_CLOCKWAKE
#ifdef LITEV_MP_FUTEX
        SeqSeen[inst] = SleepSeq[inst].load(std::memory_order_acquire);
#endif
        need();
        WakeOwn(inst);
        // the registration before a last look at the clocks (a peer stores its clock, then loads WakeAt)
        std::atomic_thread_fence(std::memory_order_seq_cst);
        if (pred()) break;
#endif
        bool timedOut = Sleep(lk, inst);
        if (stats) { WS[inst].Wakeups++; if (timedOut && pred()) WS[inst].Missed++; }
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
    TLog(inst, "U");
    if (stats) WS[inst].Ns[kind] += std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - start).count();
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
        if (queues == FromHost) FromHostChanged(i);
    }
}

void LockstepMP::TxNote(int inst, u32 type, const u8* data, int len, u64 timestamp)
{
    const u64 head[3] = { type | ((u64)(u32)len << 32), timestamp, Now(inst) };
    u64 h = XXH3_64bits_withSeed(head, sizeof(head), TxH[inst]);
    TxH[inst] = XXH3_64bits_withSeed(data, len, h);
    TxD[inst] = XXH3_64bits_withSeed(data, len, XXH3_64bits_withSeed(head, 8, TxD[inst]));
    TxN[inst]++;
    static const char* txlog = getenv("LITEV_MP_TXLOG");   // diagnostics: every sent frame, per console
    if (txlog)
    {
        static FILE* f[kMaxInst];
        if (!f[inst]) { char p[512]; snprintf(p, sizeof(p), "%s/tx%d.txt", txlog, inst); f[inst] = fopen(p, "w"); }
        if (f[inst]) fprintf(f[inst], "%u t%u len %d now %llu ts %llu data %016llx\n", TxN[inst], type, len, (unsigned long long)Now(inst),
                             (unsigned long long)timestamp, (unsigned long long)XXH3_64bits(data, len));
        // CMD: reply time, client mask; ACK: cmd count, clients that failed (both at 0xC+0x18/0x1A)
        if (f[inst] && len >= 0x28 && (type == 1 || type == 3)) fprintf(f[inst], "  w %04x %04x\n", *(const u16*)&data[0x24], *(const u16*)&data[0x26]);
    }
}

int LockstepMP::SendPacket(int inst, u8* data, int len, u64 timestamp)
{
    std::lock_guard<std::mutex> lk(Lock);
    Log(inst, "SendPacket", len, timestamp);
    TxNote(inst, 0, data, len, timestamp);
    PacketCount++;
    Broadcast(inst, 0, data, len, timestamp, Regular);
#ifndef LITEV_MP_CLOCKWAKE
    Changed.notify_all();   // (with LITEV_MP_CLOCKWAKE: regular frames are waited for by clock only)
#endif
    return len;
}

int LockstepMP::RecvPacket(int inst, u8* data, u64* timestamp)
{
    std::unique_lock<std::mutex> lk(Lock);
    u64 now = Now(inst);
    if (now < kDelay) return 0;
    u64 visible = now - kDelay;
    Log(inst, "RecvPacketWait", 0, visible);

    // An MP client (polling as one, NDS::MPClientRX, with a host it took a CMD/ACK from) gets its
    // host's regular frames (beacons) the way it gets the host's CMDs and ACKs: on time, not kDelay
    // late. A beacon kDelay late lands in a later exchange, where real hardware never has one (the
    // host sends its beacons, CMDs and ACKs one after the other on one radio): the client was
    // receiving it when the ACK or the next CMD came, took that late, and Mario Kart DS dropped the
    // client from the session ("Communication error", 8 consoles, depending on input timing). And
    // host frames go first: while one is visible and not taken yet, no regular frame is.
    // Deterministic: ClientHost is this console's own history, and the wait (host strictly past
    // our time, ties by id) is RecvHostPacket's, which the client already does on every tick.
    const int host = (Console[inst] && Console[inst]->MPClientRX) ? ClientHost[inst] : -1;
    const bool onTime = host >= 0 && host != inst && (Members & (1 << host)) && now >= kHostDelay;
    const u64 hvis = onTime ? now - kHostDelay : 0;
    auto hostPast = [&] { u64 t = Now(host); return t > hvis || (t == hvis && host > inst); };
    auto hostShows = [&](const Packet& p) { return p.Time < hvis || (p.Time == hvis && p.Sender < inst); };
    auto hostPending = [&] {
        for (const Packet& p : FromHost[inst])
            if (p.Time >= BeginTime[inst] && hostShows(p)) return true;
        return false;
    };

    // everything a peer sent up to `visible` (the host: up to `hvis`) must exist before we look
    WaitFor(lk, [&] { return onTime ? hostPending() || (PeersReached(inst, visible) && hostPast()) : PeersReached(inst, visible); }, [&] {
        for (int i = 0; i < kMaxInst; i++)
            if (i != inst && (Members & (1 << i)) && Now(i) <= visible) NeedClock(inst, i, visible + 1);
        if (onTime) NeedClock(inst, host, host > inst ? hvis : hvis + 1);
    }, inst, 0);
    if (onTime && hostPending()) { Log(inst, "RecvPacketHostFirst", 0, 0); return 0; }

    // earliest visible frame by (send time, sender), first-sent among equals
    std::deque<Packet>& q = Regular[inst];
    int best = -1;
    while (!q.empty() && q.front().Time < BeginTime[inst]) q.pop_front(); // sent before our Begin
    for (int i = 0; i < (int)q.size(); i++)
    {
        if (q[i].Time < BeginTime[inst]) continue;
        if (onTime && q[i].Sender == host ? !hostShows(q[i]) : q[i].Time > visible) continue;
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
    std::unique_lock<std::mutex> lk(Lock);
    Log(inst, "SendCmd", len, timestamp);
    TxNote(inst, 1, data, len, timestamp);
    PacketCount++; CmdCount++;
    HostID = inst;
#ifdef LITEV_MP_FASTPOLL
    HostIDSeen.store(inst, std::memory_order_relaxed);
#endif
    // (no clearing the reply queue here: what is in it now depends on thread timing; RecvReplies
    // drops replies sent before this CMD instead)
    CmdTime[inst] = Now(inst);
    TLog(inst, "CMD");
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
    NotifyOthers(inst);
#ifdef LITEV_MP_FASTPOLL
    lk.unlock();
    // the queued frame (and FromHostN) before any later store of this console's clock: a client
    // that sees the clock past the frame's time then also sees the frame queued
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
    return len;
}

int LockstepMP::SendAck(int inst, u8* data, int len, u64 timestamp)
{
    std::unique_lock<std::mutex> lk(Lock);
    Log(inst, "SendAck", len, timestamp);
    TxNote(inst, 3, data, len, timestamp);
    PacketCount++;
    // The ACK's first word (melonDS frame header) tells clients how long they may run without
    // polling for host frames: the rest of the host's CMD window (W_CmdCount, ~13-15 ms in Mario
    // Kart DS). But the host's game can start another CMD inside that window (MKDS does with 6+
    // consoles, ~1.5 ms after the ACK); a client running ahead then takes that CMD late, replies
    // past the host's deadline and is dropped from the session. Here a polling client waits on the
    // host's clock anyway, so it polls every tick instead: no runahead.
    if (len >= 4) memset(data, 0, 4);
    Broadcast(inst, 3, data, len, timestamp, FromHost);
    NotifyOthers(inst);
#ifdef LITEV_MP_FASTPOLL
    lk.unlock();
    // the queued frame (and FromHostN) before any later store of this console's clock: a client
    // that sees the clock past the frame's time then also sees the frame queued
    std::atomic_thread_fence(std::memory_order_seq_cst);
#endif
    return len;
}

int LockstepMP::SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid)
{
    std::lock_guard<std::mutex> lk(Lock);
    Log(inst, "SendReply", len, timestamp);
    TxNote(inst, 2, data, len, timestamp);
    if (St.On > 0 && St.T1 && !St.T2) St.T2 = NowNs();
    TLog(inst, "REPLY");
    PacketCount++; ReplyCount++;
    if (HostID >= 0 && HostID != inst && (Members & (1 << HostID)))
        Replies[HostID].push_back({inst, 2u | ((u32)aid << 16), timestamp, Now(inst), std::vector<u8>(data, data + len)});
    NotifyOne(HostID >= 0 ? HostID : inst);
    return len;
}

#ifdef LITEV_MP_FASTPOLL
// false: nothing from the host can be visible to `inst` now, so the poll returns 0 without the lock.
// Nothing queued for us and the host strictly past our time is exactly the case where the locked
// path returns 0 at once (host frames are queued before the host's clock moves on; the fence after
// SendCmd/SendAck orders the queue count before that clock). Also called straight from the
// client's Wi-Fi tick (NDS::MPHostPoll), which then skips the whole receive path.
bool LockstepMP::HostFrameMaybe(void* self, int inst)
{
    LockstepMP& m = *(LockstepMP*)self;
    const int host = m.HostIDSeen.load(std::memory_order_relaxed);
    const u64 now = m.Now(inst);
    if (host < 0 || host == inst || now < kHostDelay) return true;
    const u64 visible = now - kHostDelay;
    const u64 t = m.Now(host);
    std::atomic_thread_fence(std::memory_order_acquire);
    return !((t > visible || (t == visible && host > inst)) && m.FromHostN[inst].load(std::memory_order_relaxed) == 0);
}
#endif

int LockstepMP::RecvHostPacket(int inst, u8* data, u64* timestamp)
{
#ifdef LITEV_MP_FASTPOLL
    // An MP client polls on every Wi-Fi tick (~900 times per frame in Mario Kart DS) and almost
    // always gets nothing; with 8 consoles those polls queued on Lock for 2-3 ms per frame each.
    // Without the lock: nothing queued for us and the host strictly past our time is exactly the
    // case where the locked path below returns 0 at once (host frames are queued before the host's
    // clock moves on; the fence after SendCmd/SendAck orders the queue count before that clock).
    if (!HostFrameMaybe(this, inst))
    {
        if (St.On > 0) WS[inst].Calls[1]++;
        return 0;
    }
#endif
    std::unique_lock<std::mutex> lk(Lock, std::try_to_lock);
    if (!lk.owns_lock())
    {
        long long t = St.On > 0 ? NowNs() : 0;
        lk.lock();
        if (St.On > 0) { WS[inst].Contended++; WS[inst].LockNs += NowNs() - t; }
    }
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
    }, [&] {
        // past(i): clock > visible, or == visible for a higher id
        auto needPast = [&](int i) { NeedClock(inst, i, i > inst ? visible : visible + 1); };
        if (HostID >= 0 && HostID != inst) { needPast(HostID); return; }
        for (int i = 0; i < kMaxInst; i++)
            if (i != inst && (Members & (1 << i))) needPast(i);
    }, inst, 1);
    while (!FromHost[inst].empty() && FromHost[inst].front().Time < BeginTime[inst]) FromHost[inst].pop_front();
    FromHostChanged(inst);
    if (FromHost[inst].empty() || !shows(FromHost[inst].front())) { Log(inst, "RecvHost", 0, 0); return 0; }

    Packet p = std::move(FromHost[inst].front());
    FromHost[inst].pop_front();
    ClientHost[inst] = p.Sender;
    FromHostChanged(inst);
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
    if (St.On > 0) { St.Enter = NowNs(); WS[inst].Calls[2]++; }
    bool blocked = false;
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
            if (blocked && St.On > 0) WS[inst].Ns[2] += NowNs() - St.Enter;
            if (blocked) TLog(inst, "U");
            Log(inst, "RecvReplies", ret, replied); stats(); return ret;
        }

#ifdef LITEV_MP_CLOCKWAKE
        // the clients that have not replied yet: until they pass the deadline
        for (int i = 0; i < kMaxInst; i++)
            if ((Members & (1 << i)) && !(seen & (1 << i)) && !(passed & (1 << i))) NeedClock(inst, i, deadline);
        WakeOwn(inst);
        // (a reply arriving now notifies under Lock, which we hold until the wait)
        std::atomic_thread_fence(std::memory_order_seq_cst);
        {
            bool now = false;
            for (int i = 0; i < kMaxInst; i++)
                if ((Members & (1 << i)) && !(seen & (1 << i)) && !(passed & (1 << i)) && Now(i) >= deadline) now = true;
            if (now) continue;
        }
#endif
        if (!blocked && St.On > 0) WS[inst].Blocked[2]++;
        if (!blocked) TLog(inst, "B_rep");
        blocked = true;
        Sleep(lk, inst);
        if (St.On > 0) WS[inst].Wakeups++;
    }
}

}
