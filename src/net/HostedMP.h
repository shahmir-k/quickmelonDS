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

#ifndef HOSTEDMP_H
#define HOSTEDMP_H

#include <atomic>
#include <cstdio>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "MPInterface.h"
#include "LockstepMP.h"
#include "NetplayInput.h"

namespace melonDS
{

// Hosted Netplay (docs/HYBRID-NETPLAY.md, design F). One device, the server, runs every console
// over LockstepMP, exactly like Netplay. Every other device runs only its own console K, as a
// replica whose link (ReplayMP) returns exactly what the server's link returned to console K.
// Console K makes the same link calls at the same emulated times on both devices (same build,
// ROM, saves, inputs), so the replica stays bit-identical and nobody ever waits on it.
//
// One console's record stream: every link call is numbered. A call whose result is not empty (a
// received frame, received replies) is recorded with its number and result. Each emulated frame
// ends with a marker: the number of calls so far (the watermark), a hash of every call (kind,
// emulated time, arguments, result, bytes) and a state hash from the frontend (inputs, clock,
// RAM). A replica call with no record before the next marker returned nothing on the server.
//
// Record: u8 kind, u32 call, s32 result, u64 timestamp, u32 length, bytes. A frame marker is kind
// HostedFrame: call = calls so far, result = frame, timestamp = call hash, bytes = u64 state, then
// the input the server applied that frame (u32 keys, s16 touch x, s16 touch y), so a replica can
// follow the server's input (a dropped player, a second replica of the same console).
enum : u8
{
    HostedBegin, HostedEnd, HostedSendPacket, HostedRecvPacket, HostedSendCmd, HostedSendReply,
    HostedSendAck, HostedRecvHost, HostedRecvReplies, HostedFrame = 0xFF
};

// The call counter and hash, kept the same way by both sides.
struct HostedCalls
{
    u32 Count = 0;
    u64 Hash = 0;
    FILE* Trace = nullptr;  // LITEV_HOSTED_TRACE=<dir>: one line per call (diagnostics)
    void OpenTrace(const char* side, int inst);
    void Add(u8 kind, u64 now, s64 result, u64 timestamp, u64 arg, const u8* data, size_t len);
};

// Server side: wraps the real link and records what it returns to each console, without changing
// anything it returns. EndFrame and TakeRecords for console k run on console k's thread, like its
// link calls, so nothing here needs a lock.
class RecordMP : public MPInterface
{
public:
    explicit RecordMP(std::unique_ptr<LockstepMP> link) : Inner(std::move(link)) {}

    LockstepMP& Link() { return *Inner; }
    void SetClock(int inst, std::function<u64()> clock) { Clock[inst] = clock; Inner->SetClock(inst, std::move(clock)); }
    // console `inst` finished emulated frame `frame` with input `in` applied; `state` = the
    // frontend's state hash
    void EndFrame(int inst, int frame, u64 state, const NetplayFrameInput& in);
    // moves console `inst`'s records so far to the end of `out`
    void TakeRecords(int inst, std::vector<u8>& out) { out.insert(out.end(), Out[inst].begin(), Out[inst].end()); Out[inst].clear(); }

    void Process() override { Inner->Process(); }
    void Begin(int inst) override;
    void End(int inst) override;
    u16 ObserveConnectedBitmask() const noexcept override { return Inner->ObserveConnectedBitmask(); }
    u64 ObserveCmdCount() const noexcept override { return Inner->ObserveCmdCount(); }
    u64 ObserveReplyCount() const noexcept override { return Inner->ObserveReplyCount(); }
    u64 ObservePacketCount() const noexcept override { return Inner->ObservePacketCount(); }

    int SendPacket(int inst, u8* data, int len, u64 timestamp) override;
    int RecvPacket(int inst, u8* data, u64* timestamp) override;
    int SendCmd(int inst, u8* data, int len, u64 timestamp) override;
    int SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid) override;
    int SendAck(int inst, u8* data, int len, u64 timestamp) override;
    int RecvHostPacket(int inst, u8* data, u64* timestamp) override;
    u16 RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask) override;

private:
    std::unique_ptr<LockstepMP> Inner;
    std::function<u64()> Clock[LockstepMP::kMaxInst];
    HostedCalls Calls[LockstepMP::kMaxInst];
    std::vector<u8> Out[LockstepMP::kMaxInst];
    u8 Before[15 * 1024];   // RecvReplies: the reply slots before the call

    u64 Now(int inst) const { return Clock[inst] ? Clock[inst]() : 0; }
    void Put(int inst, u8 kind, u32 call, s32 result, u64 timestamp, const u8* data, size_t len);
};

// Replica side: console `inst`'s link, served from the server's record stream (Feed). A call
// blocks while the stream has nothing past it yet (the server has not got that far).
class ReplayMP : public MPInterface
{
public:
    explicit ReplayMP(int inst);

    void SetClock(std::function<u64()> clock) { Clock = std::move(clock); }
    // more of the record stream, in order, in any pieces (any thread)
    void Feed(const u8* data, size_t len);
    // the console finished emulated frame `frame`: waits for the server's marker for it and
    // compares. false = desync (the replica no longer matches the server's console); Error() says how.
    bool EndFrame(int frame, u64 state);
    // the input the server applied at `frame`, if its marker for it has arrived (wait: until it
    // does). false = not yet (or stopped).
    bool ServerInput(int frame, NetplayFrameInput& in, bool wait);
    // ends every wait (the session is shutting down)
    void Stop();
    // how long since the server was last heard from (ms): a wait for the stream ends the session
    // (Lost(), Error() says so) once this passes LostAfterMs
    void SetServerSilence(std::function<double()> ms) { ServerSilentMs = std::move(ms); }
    int LostAfterMs = 5000;
    bool Lost() const { return LostServer.load(); }
    std::string Error();
    // time spent waiting for the stream
    double StallMs() const { return StallUs.load() / 1000.0; }

    void Process() override {}
    void Begin(int inst) override;
    void End(int inst) override;

    int SendPacket(int inst, u8* data, int len, u64 timestamp) override;
    int RecvPacket(int inst, u8* data, u64* timestamp) override;
    int SendCmd(int inst, u8* data, int len, u64 timestamp) override;
    int SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid) override;
    int SendAck(int inst, u8* data, int len, u64 timestamp) override;
    int RecvHostPacket(int inst, u8* data, u64* timestamp) override;
    u16 RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask) override;

private:
    struct Record
    {
        u8 Kind;
        u32 Call;
        s32 Result;
        u64 Timestamp;
        std::vector<u8> Data;
    };

    int Inst;
    std::function<u64()> Clock;
    HostedCalls Calls;
    std::mutex Lock;
    std::condition_variable Changed;
    std::deque<Record> Queue;
    std::vector<u8> Partial;    // the start of a record not fully fed yet
    bool Stopped = false;
    std::string Err;
    std::atomic<u64> StallUs {0};
    std::function<double()> ServerSilentMs;
    std::atomic<bool> LostServer {false};
    const u64 CreatedUs;

    u64 Now() const { return Clock ? Clock() : 0; }
    // the record for this call (the next call number) if it returned something, else none
    bool Take(u8 kind, Record& out);
    void WaitLocked(std::unique_lock<std::mutex>& lk);
    // waits for more of the stream (or a stop), checking for a lost server
    void WaitMore(std::unique_lock<std::mutex>& lk, const std::function<bool()>& done);
    void Fail(const std::string& what);
};

// Hosted Netplay transport (UDP, star). The server sends each client its console's record stream,
// reliably and in order: chunks carry their byte offset, and the client acknowledges what it has
// received contiguously plus the first run it holds past a gap. The server re-sends:
// - a gap's chunks at once, when the client holds a chunk sent well after them (fast re-send);
// - any chunk not acknowledged within 1.5 measured round trips (at least kMinRtoMs, doubling with
//   each re-send of it, at most kWindow chunks per kResendMs tick);
// and a small push also carries the previous one while that is unacknowledged, so one lost packet
// costs a frame, not a round trip. A lost or late packet thus stalls the replica, never desyncs it.
// The server learns each client's address from its acknowledgements (the client sends one every
// 10 ms); a client silent for kGoneMs is gone, and its stream is buffered but not sent until it
// answers again. Inputs go the other way through NetplayInput.
class HostedServer
{
public:
    HostedServer(int bindPort, const NetFaults& faults = {});
    // waits (up to DrainMs) until every client still answering has its whole stream; one that has
    // gone quiet (left: a deliberate stop) is not waited for
    ~HostedServer();
    bool Ok() const { return Socket >= 0; }
    int DrainMs = 10000;    // how long the destructor waits for the clients
    // append to console's stream and send it
    void Push(int console, const u8* data, size_t len);
    // bytes sent to console's client so far (headers included), and how many of them were re-sends
    void Traffic(int console, u64& sent, u64& resent);

private:
    struct Chunk
    {
        u64 Offset;
        u32 Len;
        u64 SentUs;     // last sent, 0 = never
        u8 Resends;     // the timeout doubles with each
        bool Held;      // the client has it, past a gap
    };
    struct Stream
    {
        std::vector<u8> Bytes;      // not acknowledged yet, from offset Acked
        u64 Acked = 0;
        std::deque<Chunk> Chunks;   // the same bytes, as sent, oldest first
        u8 Addr[16] {};             // sockaddr_in
        bool Known = false;
        bool Gone = false;          // logged as gone
        u64 LastAckUs = 0;
        u64 RttUs = 0;              // smoothed round trip, 0 = no sample yet
        u64 SentBytes = 0, ResentBytes = 0;
    };
    int Socket = -1;
    NetFaults Faults;
    std::mutex Lock;
    Stream Streams[LockstepMP::kMaxInst];
    std::atomic<bool> Running {true};
    std::thread Receiver;

    bool Live(int console, u64 now);                    // Lock held
    void SendChunk(int console, Chunk& c, u64 now);     // Lock held
    size_t SendBytes(int console, u64 from, u64 to);    // Lock held; the packet's size
    void ReceiveLoop();
};

class HostedClient
{
public:
    // console: the console this device runs; server: "ip:port" of the HostedServer
    HostedClient(int console, const std::string& server, std::function<void(const u8*, size_t)> sink, const NetFaults& faults = {});
    ~HostedClient();
    bool Ok() const { return Socket >= 0; }

private:
    int Console;
    int Socket = -1;
    u8 ServerAddr[16] {};
    NetFaults Faults;
    std::function<void(const u8*, size_t)> Sink;
    u64 Expected = 0;                           // stream bytes received contiguously
    std::map<u64, std::vector<u8>> Ahead;       // chunks past a gap (reordered or after a loss)
    std::atomic<bool> Running {true};
    std::thread Receiver;

    void SendAck();
    void ReceiveLoop();
};

}

#endif
