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
#include "Fiber.h"

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
#ifdef LITEV_MP_FASTPOLL
        TraceOn = TraceDir() != nullptr;
        TLOn = getenv("LITEV_MP_TL") != nullptr;
#endif
#ifdef LITEV_MP_CLOCKWAKE
        for (auto& w : WakeAt) w.store(UINT64_MAX, std::memory_order_relaxed);
#endif
        for (int& h : ClientHost) h = -1;
    }

    void Process() override {}
    void Begin(int inst) override;
    void End(int inst) override;

    // How to read instance `inst`'s emulated clock (NDS::GetSysTimestamp), from any thread.
    void SetClock(int inst, std::function<u64()> clock) { Clock[inst] = std::move(clock); Members |= (1 << inst); }
#ifdef LITEV_MP_FASTPOLL
    // the clock as a plain read of the console's timestamp (Now() runs several times per poll)
    void SetClockSource(int inst, const u64* src) { ClockSrc[inst] = src; Clock[inst] = [src] { return *src; }; Members |= (1 << inst); }
#endif
    // LITEV_MP_CLOCKWAKE: inst's console (`nds`) wakes the consoles waiting on its clock as soon as
    // it reaches what they wait for; nothing without the flag. After SetClock.
    void SetWake(int inst, NDS& nds);
#ifdef LITEV_MP_FASTPOLL
    static bool HostFrameMaybe(void* self, int inst);
#endif
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
public:
    // every frame a console sends (type, data, its clock): a running hash per console. A Netplay
    // remote copy is right when this matches its owner's console (Netplay desync check; Mac gate).
    u64 TxHash(int inst) const { return TxH[inst]; }
    u32 TxCount(int inst) const { return TxN[inst]; }
    u64 TxDataHash(int inst) const { return TxD[inst]; }   // contents only (not when)
private:
    u64 TxH[kMaxInst] {};
    u32 TxN[kMaxInst] {};
    u64 TxD[kMaxInst] {};
    void TxNote(int inst, u32 type, const u8* data, int len, u64 timestamp);
    // LITEV_MP_TL=<dir>: timeline, one line per blocking event: wall ns, event, own clock, host clock
    FILE* TL[kMaxInst] {};
#ifdef LITEV_MP_FASTPOLL
    // the poll path calls these ~2000 times a frame per client: inline no-ops unless tracing
    bool TLOn = false;
    void TLog(int inst, const char* ev) { if (TLOn) TLogImpl(inst, ev); }
#else
    void TLog(int inst, const char* ev) { TLogImpl(inst, ev); }
#endif
    void TLogImpl(int inst, const char* ev);
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
    const NDS* Console[kMaxInst] {};   // (SetWake) for NDS::MPClientRX
    int ClientHost[kMaxInst];           // the sender of the last host frame each console took (-1 none since Begin)
    FILE* Trace[kMaxInst] {};   // LITEV_MP_TRACE=<dir>: one line per link call, per instance
#ifdef LITEV_MP_FASTPOLL
    bool TraceOn = false;
    void Log(int inst, const char* call, int result, u64 extra) { if (TraceOn) LogImpl(inst, call, result, extra); }
#else
    void Log(int inst, const char* call, int result, u64 extra) { LogImpl(inst, call, result, extra); }
#endif
    void LogImpl(int inst, const char* call, int result, u64 extra);
    static const char* TraceDir();
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
#ifdef LITEV_MP_FASTPOLL
    const u64* ClockSrc[kMaxInst] {};
    u64 Now(int inst) const { return ClockSrc[inst] ? __atomic_load_n(ClockSrc[inst], __ATOMIC_RELAXED) : Clock[inst] ? Clock[inst]() : 0; }
#else
    u64 Now(int inst) const { return Clock[inst] ? Clock[inst]() : 0; }
#endif
    bool PeersReached(int inst, u64 time) const;
    // waits (lock held) until pred(); peers' clocks advance without notifying, so also re-check
    // every 100 us
    // need(): registers (NeedClock) the clocks the wait is for, before each sleep
    template <typename Pred, typename Need> void WaitFor(std::unique_lock<std::mutex>& lk, Pred pred, Need need, int inst, int kind);
#ifdef LITEV_MP_CLOCKWAKE
    std::atomic<u64> WakeAt[kMaxInst];      // per console: wake the waiters once its clock reaches this
#ifdef LITEV_MP_FUTEX
    // LITEV_MP_FUTEX: each console sleeps on its own sequence word (futex / __ulock), so a console
    // whose clock passes a waiter's need wakes it with one syscall and without taking Lock (on the
    // host of an 8-player session: cond_signal + the contended unlock were ~13% of its CPU)
    std::atomic<u16> WaitersOn[kMaxInst] {};   // who waits on each console's clock
    std::atomic<u32> SleepSeq[kMaxInst] {};
    u32 SeqSeen[kMaxInst] {};   // (its own console) SleepSeq before the last look at the clocks
    static void FutexWait(std::atomic<u32>* a, u32 v, u32 us);
    static void FutexWake(std::atomic<u32>* a);
    void WakeOne(int i) { SleepSeq[i].fetch_add(1, std::memory_order_release); FutexWake(&SleepSeq[i]); }
#else
    u16 WaitersOn[kMaxInst] {};             // (Lock) who waits on each console's clock
#endif
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
#ifdef LITEV_MP_FUTEX
        WaitersOn[peer].fetch_or((u16)(1 << waiter), std::memory_order_relaxed);
#else
        WaitersOn[peer] |= (u16)(1 << waiter);
#endif
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
#ifdef LITEV_MP_FUTEX
        const u16 w = WaitersOn[inst].exchange(0, std::memory_order_acq_rel);
        for (int i = 0; i < kMaxInst; i++) if (w & (1 << i)) WakeOne(i);
#else
        for (int i = 0; i < kMaxInst; i++) if (WaitersOn[inst] & (1 << i)) CV[i].notify_one();
        Notified |= WaitersOn[inst];
        WaitersOn[inst] = 0;
#endif
    }
#endif
    // (Lock held) wake: everyone / console `inst` / every console but `inst`
    void NotifyAll()
    {
#ifdef LITEV_MP_CLOCKWAKE
#ifdef LITEV_MP_FUTEX
        for (int i = 0; i < kMaxInst; i++) WakeOne(i);
#else
        for (auto& cv : CV) cv.notify_one();
#endif
        Notified = 0xFFFF;
#endif
        Changed.notify_all();
    }
    void NotifyOne(int inst)
    {
#ifdef LITEV_MP_CLOCKWAKE
#ifdef LITEV_MP_FUTEX
        WakeOne(inst);
#else
        CV[inst].notify_one();
#endif
        Notified |= (u16)(1 << inst);
#else
        Changed.notify_all();
#endif
    }
    void NotifyOthers(int inst)
    {
#ifdef LITEV_MP_CLOCKWAKE
#ifdef LITEV_MP_FUTEX
        for (int i = 0; i < kMaxInst; i++) if (i != inst && (Members & (1 << i))) WakeOne(i);
#else
        for (int i = 0; i < kMaxInst; i++) if (i != inst && (Members & (1 << i))) CV[i].notify_one();
#endif
        Notified |= (u16)(Members & ~(1 << inst));
#else
        Changed.notify_all();
#endif
    }
    // (Lock held) console `inst` sleeps until woken or kPoll; true = timed out
    bool Sleep(std::unique_lock<std::mutex>& lk, int inst)
    {
#ifdef LITEV_MP_FIBERS
        // a console on a fiber: let the next console on this core run, then look again
        if (Fiber::Active()) { lk.unlock(); Fiber::Yield(); lk.lock(); return false; }
#endif
#if defined(LITEV_MP_FUTEX)
        // (SleepSeq was read before the last look at the clocks: a wake since then changed it)
        lk.unlock();
        FutexWait(&SleepSeq[inst], SeqSeen[inst], (u32)kPoll.count());
        lk.lock();
        return false;
#elif defined(LITEV_MP_CLOCKWAKE)
        Notified &= (u16)~(1 << inst);
        return CV[inst].wait_for(lk, kPoll) == std::cv_status::timeout && !(Notified & (1 << inst));
#else
        return Changed.wait_for(lk, kPoll) == std::cv_status::timeout;
#endif
    }
};

}

#endif
