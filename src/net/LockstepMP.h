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

#ifndef LOCKSTEPMP_H
#define LOCKSTEPMP_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <vector>
#include <chrono>
#include <cstdio>

#include "MPInterface.h"

namespace melonDS
{
class NDS;

// Deterministic in-process wireless link between several emulated consoles, each running on its
// own thread (Netplay runs every player's console on every device, so every device must compute
// exactly the same thing). Unlike LocalMP, nothing depends on thread timing or wall-clock time.
// Every decision is made on the consoles' emulated system clocks (SysTimestamp, the same time base
// for consoles booted together), read through SetClock():
//
// - A regular frame (beacon, auth/association, data) sent at emulated time S is visible to the
//   others from their time S + kDelay. A console polling at time R first waits until every peer's
//   clock has reached R - kDelay, so it always sees exactly the same frames, in (time, sender)
//   order.
// - Host frames (CMD, ACK) sent at S are visible to the clients from their time S + kHostDelay,
//   so a client can run up to kHostDelay ahead of the host without a thread hand-off (halves
//   two-console wall time vs immediate delivery). Replies reach the host immediately. The waits
//   end on a clock condition instead of a timeout: a client that has run kDelay past the CMD
//   without replying never will; a client waiting for the host gets nothing once the host's clock
//   has passed its own - kHostDelay.
//
// Wi-Fi on/off (Begin/End) happens at a console's own time, which another thread may not have
// reached yet, so no decision depends on who is connected right now: frames go to every console
// with a clock, a console only reads frames sent after its latest Begin, and every wait is on the
// clocks of all of them (a console with Wi-Fi off simply never sends or replies).
//
// kDelay (4 ms; measured fastest on the RG DS, 2 ms makes the consoles wait on each other more) is longer than any MP reply window (~0.5 ms). Deadlock freedom: every wait is
// either "peer clock > my clock - kDelay" (regular frames) or "peer clock >= my clock + kDelay"
// (host frames, reply deadline): one strict, one inclusive, so two consoles can never both be
// waiting on each other (that would need each to be at least kDelay ahead of the other). With
// kHostDelay 0 the host-frame wait of a client with no host yet is "peer clock > my clock", so equal
// clocks break the tie by console id (RecvHostPacket).
// ponytail: waits poll the peers' clocks every 100 us; a per-peer wake threshold if that costs fps.
class LockstepMP : public MPInterface
{
public:
    // consoles on one link: the DS wireless maximum (also LAN's player bound and Netplay's)
    static constexpr int kMaxInst = 16;

    LockstepMP() noexcept
    {
#ifdef LITEV_MP_CLOCKWAKE
        for (auto& w : WakeAt) w.store(UINT64_MAX, std::memory_order_relaxed);
#endif
    }

    void Process() override {}
    void Begin(int inst) override;
    void End(int inst) override;

    // How to read instance `inst`'s emulated clock (NDS::GetSysTimestamp), from any thread.
    void SetClock(int inst, std::function<u64()> clock) { Clock[inst] = std::move(clock); Members |= (1 << inst); }
    // LITEV_MP_CLOCKWAKE: inst's console (`nds`) wakes the consoles waiting on its clock as soon as
    // it reaches what they wait for; nothing without the flag. After SetClock.
    void SetWake(int inst, NDS& nds);
    // Ends every wait (the session is shutting down).
    void Stop();

    u16 ObserveConnectedBitmask() const noexcept override { return Connected; }
    u64 ObserveCmdCount() const noexcept override { return CmdCount; }
    u64 ObserveReplyCount() const noexcept override { return ReplyCount; }
    u64 ObservePacketCount() const noexcept override { return PacketCount; }

    int SendPacket(int inst, u8* data, int len, u64 timestamp) override;
    int RecvPacket(int inst, u8* data, u64* timestamp) override;
    int SendCmd(int inst, u8* data, int len, u64 timestamp) override;
    int SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid) override;
    int SendAck(int inst, u8* data, int len, u64 timestamp) override;
    int RecvHostPacket(int inst, u8* data, u64* timestamp) override;
    u16 RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask) override;

    // LITEV_MP_STATS: per console, link calls and how many of them blocked (0 RecvPacket,
    // 1 RecvHostPacket, 2 RecvReplies), wait wakeups and blocked time, since the last take
    struct WaitStats { u32 Calls[3] {}, Blocked[3] {}, Wakeups = 0, Contended = 0, Missed = 0; long long Ns[3] {}, LockNs = 0; };
    WaitStats TakeWaitStats(int inst) { std::lock_guard<std::mutex> lk(Lock); WaitStats s = WS[inst]; WS[inst] = {}; return s; }

private:
    WaitStats WS[kMaxInst];
    // LITEV_MP_TL=<dir>: timeline, one line per blocking event: wall ns, event, own clock, host clock
    FILE* TL[kMaxInst] {};
    void TLog(int inst, const char* ev);
    static constexpr u64 kDelay = 33514 * 4; // 4 ms in system clock cycles (33.514 MHz)
    // host frames (CMD/ACK) reach the clients this much later; < kDelay, or a reply sent on time
    // would land past the host's deadline
#ifdef LITEV_MP_HOSTDELAY
    static constexpr u64 kHostDelay = 33514 * 2;
#else
    static constexpr u64 kHostDelay = 0;   // host frames delivered immediately
#endif

    struct Packet
    {
        int Sender;
        u32 Type;
        u64 Timestamp;      // the sender's wifi timestamp, passed through to the receiver
        u64 Time;           // the sender's system clock when it was sent
        std::vector<u8> Data;
    };

    std::mutex Lock;
    std::condition_variable Changed;

    u16 Connected = 0;      // Wi-Fi on right now (observation only)
    u16 Members = 0;        // consoles with a clock: everything waits on these
    u64 BeginTime[kMaxInst] {};
    u64 CmdTime[kMaxInst] {};   // when each console last sent a CMD (its clock)
    bool Stopped = false;
    std::function<u64()> Clock[kMaxInst];
    FILE* Trace[kMaxInst] {};   // LITEV_MP_TRACE=<dir>: one line per link call, per instance
    void Log(int inst, const char* call, int result, u64 extra);
    // LITEV_MP_STATS / debug.litev.mpstats=1: per CMD, where the host's wait for the replies goes
    struct
    {
        int On = -1;
        long long T0 = 0, T1 = 0, T2 = 0, Enter = 0;   // ns: CMD sent, CMD delivered, reply sent, host starts waiting
        double Deliver = 0, Reply = 0, Wake = 0, Wait = 0, Notify = 0;
        u32 N = 0, Lead[4] {};
    } St;
    bool StatsOn();
    std::deque<Packet> Regular[kMaxInst];   // regular frames, per receiver, in send order per sender
    std::deque<Packet> FromHost[kMaxInst];  // CMD/ACK, per receiver
    std::deque<Packet> Replies[kMaxInst];   // replies, per host
    int HostID = -1;
#ifdef LITEV_MP_FASTPOLL
    // copies for RecvHostPacket's lock-free poll, written under Lock: HostID, FromHost[i].size()
    std::atomic<int> HostIDSeen {-1};
    std::atomic<u32> FromHostN[kMaxInst] {};
#endif
    void FromHostChanged(int inst)
    {
#ifdef LITEV_MP_FASTPOLL
        FromHostN[inst].store((u32)FromHost[inst].size(), std::memory_order_relaxed);
#endif
    }

    u64 CmdCount = 0, ReplyCount = 0, PacketCount = 0;

    void Broadcast(int inst, u32 type, u8* data, int len, u64 timestamp, std::deque<Packet>* queues);
    u64 Now(int inst) const { return Clock[inst] ? Clock[inst]() : 0; }
    bool PeersReached(int inst, u64 time) const;
    // waits (lock held) until pred(); peers' clocks advance without notifying, so also re-check
    // every 100 us
    // need(): registers (NeedClock) the clocks the wait is for, before each sleep
    template <typename Pred, typename Need> void WaitFor(std::unique_lock<std::mutex>& lk, Pred pred, Need need, int inst, int kind);
#ifdef LITEV_MP_CLOCKWAKE
    std::atomic<u64> WakeAt[kMaxInst];      // per console: wake the waiters once its clock reaches this
    u16 WaitersOn[kMaxInst] {};             // (Lock) who waits on each console's clock
    std::condition_variable CV[kMaxInst];   // each console sleeps on its own: wakes go to who needs them
    u16 Notified = 0;                       // (Lock) diagnostics: woken since it went to sleep
    void Wake(int inst);
    // fallback poll: a wake can still be missed (the clock store and the WakeAt load in
    // NDS::RunSystem are unfenced), then the waiter notices at most this late
    static constexpr auto kPoll = std::chrono::microseconds(1000);
#else
    static constexpr auto kPoll = std::chrono::microseconds(100);
#endif
    // (Lock held) console `waiter` needs console `peer`'s clock to reach `t`
    void NeedClock(int waiter, int peer, u64 t)
    {
#ifdef LITEV_MP_CLOCKWAKE
        WaitersOn[peer] |= (u16)(1 << waiter);
        u64 cur = WakeAt[peer].load(std::memory_order_relaxed);
        while (t < cur && !WakeAt[peer].compare_exchange_weak(cur, t, std::memory_order_relaxed)) {}
#endif
    }
    // (Lock held, before sleeping) wake those waiting on our own clock if it is already there
    void WakeOwn(int inst)
    {
#ifdef LITEV_MP_CLOCKWAKE
        if (Now(inst) >= WakeAt[inst].load(std::memory_order_relaxed)) WakeLocked(inst);
#endif
    }
#ifdef LITEV_MP_CLOCKWAKE
    void WakeLocked(int inst)
    {
        WakeAt[inst].store(UINT64_MAX, std::memory_order_relaxed);
        for (int i = 0; i < kMaxInst; i++) if (WaitersOn[inst] & (1 << i)) CV[i].notify_one();
        Notified |= WaitersOn[inst];
        WaitersOn[inst] = 0;
    }
#endif
    // (Lock held) wake: everyone / console `inst` / every console but `inst`
    void NotifyAll()
    {
#ifdef LITEV_MP_CLOCKWAKE
        for (auto& cv : CV) cv.notify_one();
        Notified = 0xFFFF;
#endif
        Changed.notify_all();
    }
    void NotifyOne(int inst)
    {
#ifdef LITEV_MP_CLOCKWAKE
        CV[inst].notify_one();
        Notified |= (u16)(1 << inst);
#else
        Changed.notify_all();
#endif
    }
    void NotifyOthers(int inst)
    {
#ifdef LITEV_MP_CLOCKWAKE
        for (int i = 0; i < kMaxInst; i++) if (i != inst && (Members & (1 << i))) CV[i].notify_one();
        Notified |= (u16)(Members & ~(1 << inst));
#else
        Changed.notify_all();
#endif
    }
    // (Lock held) console `inst` sleeps until woken or kPoll; true = timed out
    bool Sleep(std::unique_lock<std::mutex>& lk, int inst)
    {
#ifdef LITEV_MP_CLOCKWAKE
        Notified &= (u16)~(1 << inst);
        return CV[inst].wait_for(lk, kPoll) == std::cv_status::timeout && !(Notified & (1 << inst));
#else
        return Changed.wait_for(lk, kPoll) == std::cv_status::timeout;
#endif
    }
};

}

#endif
