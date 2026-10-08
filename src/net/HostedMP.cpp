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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "HostedMP.h"
#include "../Platform.h"
#include "xxhash/xxhash.h"

namespace melonDS
{

namespace
{
constexpr size_t kHeader = 1 + 4 + 4 + 8 + 4;  // kind, call, result, timestamp, length

u64 NowUs()
{
    return (u64)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

template <typename T> void Append(std::vector<u8>& out, T v)
{
    const u8* p = (const u8*)&v;
    out.insert(out.end(), p, p + sizeof(v));
}
}

void HostedCalls::Add(u8 kind, u64 now, s64 result, u64 timestamp, u64 arg, const u8* data, size_t len)
{
    // a sent frame's 12-byte TX header is not all initialised (SendMPAck leaves bytes 4-7): hash
    // the frame itself
    bool send = kind == HostedSendPacket || kind == HostedSendCmd || kind == HostedSendReply || kind == HostedSendAck;
    if (send) { if (len > 12) { data += 12; len -= 12; } else len = 0; }
    u64 v[6] = { Hash, ((u64)Count << 8) | kind, now, (u64)result, timestamp ^ (arg << 1), len ? XXH3_64bits(data, len) : 0 };
    Hash = XXH3_64bits(v, sizeof(v));
    if (Trace) fprintf(Trace, "%u k%u t%llu r%lld ts%llu a%llu d%016llx\n", Count, kind, (unsigned long long)now, (long long)result,
                       (unsigned long long)timestamp, (unsigned long long)arg, (unsigned long long)v[5]);
    Count++;
}

void HostedCalls::OpenTrace(const char* side, int inst)
{
    const char* dir = getenv("LITEV_HOSTED_TRACE");
    if (!dir) return;
    std::string path = std::string(dir) + "/" + side + std::to_string(inst) + ".txt";
    Trace = fopen(path.c_str(), "w");
}

// ---- server: RecordMP ----

void RecordMP::Put(int inst, u8 kind, u32 call, s32 result, u64 timestamp, const u8* data, size_t len)
{
    std::vector<u8>& out = Out[inst];
    Append(out, kind);
    Append(out, call);
    Append(out, result);
    Append(out, timestamp);
    Append(out, (u32)len);
    out.insert(out.end(), data, data + len);
}

void RecordMP::EndFrame(int inst, int frame, u64 state, const NetplayFrameInput& in)
{
    if (frame == 0) Calls[inst].OpenTrace("server", inst);
    if (Calls[inst].Trace) fprintf(Calls[inst].Trace, "frame %d\n", frame);
    u8 data[16];
    memcpy(data, &state, 8);
    memcpy(data + 8, &in.Keys, 4);
    memcpy(data + 12, &in.TouchX, 2);
    memcpy(data + 14, &in.TouchY, 2);
    Put(inst, HostedFrame, Calls[inst].Count, frame, Calls[inst].Hash, data, sizeof(data));
}

void RecordMP::Begin(int inst)
{
    Inner->Begin(inst);
    Calls[inst].Add(HostedBegin, Now(inst), 0, 0, 0, nullptr, 0);
}

void RecordMP::End(int inst)
{
    Inner->End(inst);
    Calls[inst].Add(HostedEnd, Now(inst), 0, 0, 0, nullptr, 0);
}

int RecordMP::SendPacket(int inst, u8* data, int len, u64 timestamp)
{
    int r = Inner->SendPacket(inst, data, len, timestamp);
    Calls[inst].Add(HostedSendPacket, Now(inst), r, timestamp, 0, data, len);
    return r;
}

int RecordMP::SendCmd(int inst, u8* data, int len, u64 timestamp)
{
    int r = Inner->SendCmd(inst, data, len, timestamp);
    Calls[inst].Add(HostedSendCmd, Now(inst), r, timestamp, 0, data, len);
    return r;
}

int RecordMP::SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid)
{
    int r = Inner->SendReply(inst, data, len, timestamp, aid);
    Calls[inst].Add(HostedSendReply, Now(inst), r, timestamp, aid, data, data ? len : 0);
    return r;
}

int RecordMP::SendAck(int inst, u8* data, int len, u64 timestamp)
{
    int r = Inner->SendAck(inst, data, len, timestamp);
    Calls[inst].Add(HostedSendAck, Now(inst), r, timestamp, 0, data, len);
    return r;
}

int RecordMP::RecvPacket(int inst, u8* data, u64* timestamp)
{
    int r = Inner->RecvPacket(inst, data, timestamp);
    u64 ts = r > 0 && timestamp ? *timestamp : 0;
    u32 call = Calls[inst].Count;
    Calls[inst].Add(HostedRecvPacket, Now(inst), r, ts, 0, data, r > 0 ? r : 0);
    if (r) Put(inst, HostedRecvPacket, call, r, ts, data, r > 0 ? r : 0);
    return r;
}

int RecordMP::RecvHostPacket(int inst, u8* data, u64* timestamp)
{
    int r = Inner->RecvHostPacket(inst, data, timestamp);
    u64 ts = r > 0 && timestamp ? *timestamp : 0;
    u32 call = Calls[inst].Count;
    Calls[inst].Add(HostedRecvHost, Now(inst), r, ts, 0, data, r > 0 ? r : 0);
    if (r) Put(inst, HostedRecvHost, call, r, ts, data, r > 0 ? r : 0);
    return r;
}

u16 RecordMP::RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask)
{
    // the link writes each reply to its client's 1 KB slot, as far as the reply goes: compare the
    // slots with what they held before to record exactly the bytes written (what follows them is
    // the same on the replica)
    memcpy(Before, data, sizeof(Before));
    u16 r = Inner->RecvReplies(inst, data, timestamp, aidmask);
    std::vector<u8> slots;  // per reply: u8 aid, u16 length, bytes
    for (int aid = 1; aid < 16; aid++)
    {
        if (!(r & (1 << aid))) continue;
        const u8* slot = &data[(aid - 1) * 1024];
        int len = 1024;
        while (len > 0 && slot[len - 1] == Before[(aid - 1) * 1024 + len - 1]) len--;
        Append(slots, (u8)aid);
        Append(slots, (u16)len);
        slots.insert(slots.end(), slot, slot + len);
    }
    u32 call = Calls[inst].Count;
    Calls[inst].Add(HostedRecvReplies, Now(inst), r, timestamp, aidmask, slots.data(), slots.size());
    if (r) Put(inst, HostedRecvReplies, call, r, timestamp, slots.data(), slots.size());
    return r;
}

// ---- replica: ReplayMP ----

void ReplayMP::Feed(const u8* data, size_t len)
{
    std::lock_guard<std::mutex> lk(Lock);
    Partial.insert(Partial.end(), data, data + len);
    size_t pos = 0;
    while (Partial.size() - pos >= kHeader)
    {
        const u8* p = Partial.data() + pos;
        Record r;
        u32 dlen;
        memcpy(&r.Kind, p, 1);
        memcpy(&r.Call, p + 1, 4);
        memcpy(&r.Result, p + 5, 4);
        memcpy(&r.Timestamp, p + 9, 8);
        memcpy(&dlen, p + 17, 4);
        if (Partial.size() - pos < kHeader + dlen) break;
        r.Data.assign(p + kHeader, p + kHeader + dlen);
        Queue.push_back(std::move(r));
        pos += kHeader + dlen;
    }
    Partial.erase(Partial.begin(), Partial.begin() + pos);
    Changed.notify_all();
}

void ReplayMP::Stop()
{
    std::lock_guard<std::mutex> lk(Lock);
    Stopped = true;
    Changed.notify_all();
}

std::string ReplayMP::Error()
{
    std::lock_guard<std::mutex> lk(Lock);
    return Err;
}

void ReplayMP::Fail(const std::string& what)
{
    if (Err.empty()) Err = what;
}

ReplayMP::ReplayMP(int inst) : Inst(inst), CreatedUs(NowUs()) {}

void ReplayMP::WaitMore(std::unique_lock<std::mutex>& lk, const std::function<bool()>& done)
{
    u64 start = NowUs(), silentSeen = 0;
    while (!done() && !Stopped)
    {
        Changed.wait_for(lk, std::chrono::milliseconds(100));
        // silent since the session started at most (nothing may have arrived yet), and still
        // silent 200 ms later (this device may just have woken up, with packets not read yet)
        double silent = ServerSilentMs ? std::min(ServerSilentMs(), (NowUs() - CreatedUs) / 1000.0) : 0;
        if (done() || silent <= LostAfterMs) silentSeen = 0;
        else if (!silentSeen) silentSeen = NowUs();
        else if (NowUs() - silentSeen >= 200000)
        {
            Fail("lost the server: nothing from it for " + std::to_string((int)silent) + " ms");
            LostServer = true;
            Stopped = true;
        }
    }
    StallUs += NowUs() - start;
}

void ReplayMP::WaitLocked(std::unique_lock<std::mutex>& lk)
{
    if (!Queue.empty() || Stopped) return;
    WaitMore(lk, [&] { return !Queue.empty(); });
}

bool ReplayMP::ServerInput(int frame, NetplayFrameInput& in, bool wait)
{
    std::unique_lock<std::mutex> lk(Lock);
    const Record* m = nullptr;
    auto find = [&]
    {
        for (const Record& r : Queue)
            if (r.Kind == HostedFrame && r.Result >= frame) { m = r.Result == frame ? &r : nullptr; return true; }
        return false;
    };
    if (!find() && wait) WaitMore(lk, find);
    if (!m || m->Data.size() < 16) return false;
    memcpy(&in.Keys, &m->Data[8], 4);
    memcpy(&in.TouchX, &m->Data[12], 2);
    memcpy(&in.TouchY, &m->Data[14], 2);
    return true;
}

bool ReplayMP::Take(u8 kind, Record& out)
{
    u32 call = Calls.Count;
    std::unique_lock<std::mutex> lk(Lock);
    WaitLocked(lk);
    if (Queue.empty()) return false;    // stopped
    Record& r = Queue.front();
    if (r.Kind == HostedFrame)
    {
        // the server's console made fewer calls this frame: this one has no counterpart
        if (call >= r.Call) Fail("call " + std::to_string(call) + " past the server's " + std::to_string(r.Call) + " in frame " + std::to_string(r.Result));
        return false;
    }
    if (r.Call > call) return false;    // the server's call returned nothing
    if (r.Call < call || r.Kind != kind)
    {
        Fail("call " + std::to_string(call) + " kind " + std::to_string(kind) + ": the server's record is call " + std::to_string(r.Call) + " kind " + std::to_string(r.Kind));
        Queue.pop_front();
        return false;
    }
    out = std::move(r);
    Queue.pop_front();
    return true;
}

bool ReplayMP::EndFrame(int frame, u64 state)
{
    if (frame == 0) Calls.OpenTrace("replica", Inst);
    if (Calls.Trace) fprintf(Calls.Trace, "frame %d\n", frame);
    std::unique_lock<std::mutex> lk(Lock);
    WaitLocked(lk);
    while (!Queue.empty() && Queue.front().Kind != HostedFrame)
    {
        // the server's console received something this frame that this one never asked for
        Fail("frame " + std::to_string(frame) + ": the server's console made more calls (record for call " + std::to_string(Queue.front().Call) + ", here " + std::to_string(Calls.Count) + " calls)");
        Queue.pop_front();
        WaitLocked(lk);
    }
    if (Queue.empty()) return Err.empty(); // stopped
    Record m = std::move(Queue.front());
    Queue.pop_front();
    u64 serverState = 0;
    if (m.Data.size() >= sizeof(serverState)) memcpy(&serverState, m.Data.data(), sizeof(serverState));
    if (m.Result != frame) Fail("frame " + std::to_string(frame) + ": the server's marker is for frame " + std::to_string(m.Result));
    else if (m.Call != Calls.Count || m.Timestamp != Calls.Hash)
        Fail("frame " + std::to_string(frame) + ": link calls differ (server " + std::to_string(m.Call) + " calls, here " + std::to_string(Calls.Count) + ")");
    else if (serverState != state) Fail("frame " + std::to_string(frame) + ": state differs (inputs, clock or RAM)");
    return Err.empty();
}

void ReplayMP::Begin(int)
{
    Calls.Add(HostedBegin, Now(), 0, 0, 0, nullptr, 0);
}

void ReplayMP::End(int)
{
    Calls.Add(HostedEnd, Now(), 0, 0, 0, nullptr, 0);
}

// sends: the server's copy of this console is the one that feeds the network; only the hash
int ReplayMP::SendPacket(int, u8* data, int len, u64 timestamp)
{
    Calls.Add(HostedSendPacket, Now(), len, timestamp, 0, data, len);
    return len;
}

int ReplayMP::SendCmd(int, u8* data, int len, u64 timestamp)
{
    Calls.Add(HostedSendCmd, Now(), len, timestamp, 0, data, len);
    return len;
}

int ReplayMP::SendReply(int, u8* data, int len, u64 timestamp, u16 aid)
{
    Calls.Add(HostedSendReply, Now(), len, timestamp, aid, data, data ? len : 0);
    return len;
}

int ReplayMP::SendAck(int, u8* data, int len, u64 timestamp)
{
    Calls.Add(HostedSendAck, Now(), len, timestamp, 0, data, len);
    return len;
}

int ReplayMP::RecvPacket(int, u8* data, u64* timestamp)
{
    Record r;
    int len = 0;
    u64 ts = 0;
    if (Take(HostedRecvPacket, r))
    {
        len = r.Result;
        ts = r.Timestamp;
        if (!r.Data.empty()) memcpy(data, r.Data.data(), r.Data.size());
        if (timestamp && len > 0) *timestamp = ts;
    }
    Calls.Add(HostedRecvPacket, Now(), len, ts, 0, data, len > 0 ? len : 0);
    return len;
}

int ReplayMP::RecvHostPacket(int, u8* data, u64* timestamp)
{
    Record r;
    int len = 0;
    u64 ts = 0;
    if (Take(HostedRecvHost, r))
    {
        len = r.Result;
        ts = r.Timestamp;
        if (!r.Data.empty()) memcpy(data, r.Data.data(), r.Data.size());
        if (timestamp && len > 0) *timestamp = ts;
    }
    Calls.Add(HostedRecvHost, Now(), len, ts, 0, data, len > 0 ? len : 0);
    return len;
}

u16 ReplayMP::RecvReplies(int, u8* data, u64 timestamp, u16 aidmask)
{
    Record r;
    u16 mask = 0;
    if (Take(HostedRecvReplies, r))
    {
        mask = (u16)r.Result;
        for (size_t pos = 0; pos + 3 <= r.Data.size(); )
        {
            u8 aid = r.Data[pos];
            u16 len;
            memcpy(&len, &r.Data[pos + 1], 2);
            pos += 3;
            if (aid < 1 || aid > 15 || len > 1024 || pos + len > r.Data.size()) { Fail("bad reply record"); break; }
            memcpy(&data[(aid - 1) * 1024], &r.Data[pos], len);
            pos += len;
        }
    }
    Calls.Add(HostedRecvReplies, Now(), mask, timestamp, aidmask, r.Data.data(), mask ? r.Data.size() : 0);
    return mask;
}

// ---- transport ----

namespace
{
constexpr u32 kChunkMagic = 0x484E5331;    // "HNS1"
constexpr u32 kAckMagic = 0x484E4131;      // "HNA1"
constexpr size_t kChunk = 1200;            // stream bytes per packet (fits any MTU)
constexpr int kWindow = 16;                // packets re-sent per tick, at most
constexpr int kResendMs = 10;              // re-send tick
constexpr int kMinRtoMs = 10;              // re-send a chunk after 1.5 round trips, at least this
constexpr int kFirstRtoMs = 200;           // before the first round trip is measured
constexpr int kGoneMs = 3000;              // a client silent this long is gone

#pragma pack(push, 1)
struct ChunkHeader
{
    u32 Magic;
    u8 Console;
    u64 Offset;
    u16 Len;
};
struct AckPacket
{
    u32 Magic;
    u8 Console;
    u64 Offset;     // the client has the stream up to here
    u64 Next, NextEnd;  // and [Next, NextEnd) past a gap (Next = Offset: no gap)
};
#pragma pack(pop)

int OpenSocket(int port)
{
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    if (s < 0) return -1;
    sockaddr_in a {};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons((u16)port);
    timeval tv {0, 1000}; // 1 ms, so delayed packets are released on time
    setsockopt(s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int big = 1 << 20;
    setsockopt(s, SOL_SOCKET, SO_RCVBUF, &big, sizeof(big));
    if (bind(s, (sockaddr*)&a, sizeof(a)) != 0) { close(s); return -1; }
    return s;
}
}

HostedServer::HostedServer(int bindPort, const NetFaults& faults) : Faults(faults)
{
    Socket = OpenSocket(bindPort);
    if (Socket >= 0) Receiver = std::thread([this] { ReceiveLoop(); });
}

HostedServer::~HostedServer()
{
    if (Socket < 0) return;
    for (u64 start = NowUs(); NowUs() - start < (u64)DrainMs * 1000; )
    {
        bool all = true;
        {
            std::lock_guard<std::mutex> lk(Lock);
            for (int c = 0; c < LockstepMP::kMaxInst; c++) all &= Streams[c].Bytes.empty() || !Live(c, NowUs());
        }
        if (all) break;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    Running = false;
    Receiver.join();
    close(Socket);
}

bool HostedServer::Live(int console, u64 now)
{
    Stream& s = Streams[console];
    if (!s.Known) return false;
    bool gone = now - s.LastAckUs > (u64)kGoneMs * 1000;
    if (gone != s.Gone)
        Platform::Log(gone ? Platform::LogLevel::Warn : Platform::LogLevel::Info, gone ? "hosted: console %d: its client stopped answering, stream paused\n"
                                                                                       : "hosted: console %d: its client answers again, stream resumed\n", console);
    s.Gone = gone;
    return !gone;
}

size_t HostedServer::SendBytes(int console, u64 from, u64 to)
{
    Stream& s = Streams[console];
    ChunkHeader h {kChunkMagic, (u8)console, from, (u16)(to - from)};
    u8 packet[sizeof(ChunkHeader) + kChunk];
    memcpy(packet, &h, sizeof(h));
    memcpy(packet + sizeof(h), s.Bytes.data() + (from - s.Acked), h.Len);
    sendto(Socket, packet, sizeof(h) + h.Len, 0, (sockaddr*)s.Addr, sizeof(sockaddr_in));
    s.SentBytes += sizeof(h) + h.Len;
    return sizeof(h) + h.Len;
}

void HostedServer::SendChunk(int console, Chunk& c, u64 now)
{
    Stream& s = Streams[console];
    size_t n = SendBytes(console, std::max(c.Offset, s.Acked), c.Offset + c.Len); // a chunk can be partly acknowledged
    if (c.SentUs) { s.ResentBytes += n; c.Resends += c.Resends < 3; }
    c.SentUs = now;
}

void HostedServer::Traffic(int console, u64& sent, u64& resent)
{
    std::lock_guard<std::mutex> lk(Lock);
    sent = Streams[console].SentBytes;
    resent = Streams[console].ResentBytes;
}

void HostedServer::Push(int console, const u8* data, size_t len)
{
    if (console < 0 || console >= LockstepMP::kMaxInst || !len) return;
    std::lock_guard<std::mutex> lk(Lock);
    Stream& s = Streams[console];
    u64 end = s.Acked + s.Bytes.size(), now = NowUs();
    s.Bytes.insert(s.Bytes.end(), data, data + len);
    bool live = Live(console, now);
    // a small push also carries the previous one if that is not acknowledged yet: a single lost
    // packet is then made up by the next one, a frame later, instead of a round trip later
    if (live && len <= kChunk && !s.Chunks.empty())
    {
        Chunk& prev = s.Chunks.back();
        u64 from = std::max(prev.Offset, s.Acked);
        if (!prev.Held && prev.SentUs && from < end && end + len - from <= kChunk)
        {
            s.Chunks.push_back({end, (u32)len, now, 0, false});
            s.ResentBytes += end - from;
            SendBytes(console, from, end + len);
            return;
        }
    }
    for (u64 o = end; o < end + len; o += kChunk)
    {
        s.Chunks.push_back({o, (u32)std::min<u64>(kChunk, end + len - o), 0, 0, false});
        if (live) SendChunk(console, s.Chunks.back(), now);
    }
}

void HostedServer::ReceiveLoop()
{
    std::multimap<u64, std::vector<u8>> pending;   // test faults: (due time, packet)
    u8 buf[2048];
    u64 lastSend = 0;
    while (Running)
    {
        sockaddr_in from {};
        socklen_t fromLen = sizeof(from);
        ssize_t len = recvfrom(Socket, buf, sizeof(buf), 0, (sockaddr*)&from, &fromLen);
        u64 now = NowUs(), due;
        if (len == (ssize_t)sizeof(AckPacket) && Faults.Deliver(now, due))
        {
            std::vector<u8> p((u8*)&from, (u8*)&from + sizeof(from));   // sender address, then the ack
            p.insert(p.end(), buf, buf + len);
            pending.emplace(due, std::move(p));
        }
        while (!pending.empty() && pending.begin()->first <= now)
        {
            AckPacket a;
            memcpy(&from, pending.begin()->second.data(), sizeof(from));
            memcpy(&a, pending.begin()->second.data() + sizeof(from), sizeof(a));
            pending.erase(pending.begin());
            if (a.Magic != kAckMagic || a.Console >= LockstepMP::kMaxInst) continue;
            std::lock_guard<std::mutex> lk(Lock);
            Stream& s = Streams[a.Console];
            memcpy(s.Addr, &from, sizeof(from));   // the client's address, as the server sees it
            s.Known = true;
            s.LastAckUs = now;
            Live(a.Console, now);
            u64 end = s.Acked + s.Bytes.size();
            if (a.Offset > s.Acked && a.Offset <= end)
            {
                // round trip: the newest chunk this acknowledges, unless a re-send was involved
                // (its ack may answer either copy, or have waited for an earlier chunk's)
                bool resent = false;
                u64 sentUs = 0;
                while (!s.Chunks.empty() && s.Chunks.front().Offset + s.Chunks.front().Len <= a.Offset)
                {
                    resent |= s.Chunks.front().Resends > 0;
                    sentUs = s.Chunks.front().SentUs;
                    s.Chunks.pop_front();
                }
                if (!resent && sentUs && now > sentUs)
                    s.RttUs = s.RttUs ? (7 * s.RttUs + (now - sentUs)) / 8 : now - sentUs;
                s.Bytes.erase(s.Bytes.begin(), s.Bytes.begin() + (a.Offset - s.Acked));
                s.Acked = a.Offset;
            }
            if (a.Next > s.Acked && a.Next < a.NextEnd && a.NextEnd <= end)
            {
                // the client holds [Next, NextEnd): no need to re-send that. The gap's chunks sent
                // well before it (a quarter round trip: not just reordered) are lost: re-send each
                // once now rather than after the timeout.
                u64 nextUs = 0;
                for (Chunk& ch : s.Chunks)
                {
                    if (ch.Offset >= a.NextEnd) break;
                    if (ch.Offset >= a.Next && ch.Offset + ch.Len <= a.NextEnd)
                    {
                        if (!nextUs) nextUs = ch.SentUs;
                        ch.Held = true;
                    }
                }
                for (Chunk& ch : s.Chunks)
                {
                    if (ch.Offset >= a.Next) break;
                    if (!ch.Resends && ch.SentUs && ch.SentUs + s.RttUs / 4 < nextUs) SendChunk(a.Console, ch, now);
                }
            }
        }
        if (now - lastSend >= kResendMs * 1000)
        {
            lastSend = now;
            std::lock_guard<std::mutex> lk(Lock);
            for (int c = 0; c < LockstepMP::kMaxInst; c++)
            {
                Stream& s = Streams[c];
                if (s.Chunks.empty() || !Live(c, now)) continue;
                u64 rto = s.RttUs ? std::max<u64>(kMinRtoMs * 1000, s.RttUs * 3 / 2) : kFirstRtoMs * 1000;
                int n = 0;
                for (Chunk& ch : s.Chunks)
                {
                    if (n == kWindow) break;
                    if (!ch.Held && (!ch.SentUs || now - ch.SentUs >= rto << ch.Resends)) { SendChunk(c, ch, now); n++; }
                }
            }
        }
    }
}

HostedClient::HostedClient(int console, const std::string& server, std::function<void(const u8*, size_t)> sink, const NetFaults& faults)
    : Console(console), Faults(faults), Sink(std::move(sink))
{
    sockaddr_in a {};
    a.sin_family = AF_INET;
    size_t colon = server.rfind(':');
    inet_pton(AF_INET, server.substr(0, colon).c_str(), &a.sin_addr);
    a.sin_port = htons((u16)atoi(server.substr(colon + 1).c_str()));
    memcpy(ServerAddr, &a, sizeof(a));
    Socket = OpenSocket(0);
    if (Socket >= 0) Receiver = std::thread([this] { ReceiveLoop(); });
}

HostedClient::~HostedClient()
{
    Running = false;
    if (Receiver.joinable()) Receiver.join();
    if (Socket >= 0) close(Socket);
}

void HostedClient::SendAck()
{
    // past a gap: the first contiguous run held
    u64 next = Expected, nextEnd = Expected;
    if (!Ahead.empty())
    {
        next = nextEnd = Ahead.begin()->first;
        for (auto& [o, d] : Ahead)
        {
            if (o > nextEnd) break;
            nextEnd = std::max<u64>(nextEnd, o + d.size());
        }
    }
    AckPacket a {kAckMagic, (u8)Console, Expected, next, nextEnd};
    sendto(Socket, &a, sizeof(a), 0, (sockaddr*)ServerAddr, sizeof(sockaddr_in));
}

void HostedClient::ReceiveLoop()
{
    std::multimap<u64, std::vector<u8>> pending;   // test faults: (due time, packet)
    u8 buf[2048];
    u64 lastAck = 0;
    while (Running)
    {
        ssize_t len = recv(Socket, buf, sizeof(buf), 0);
        u64 now = NowUs(), due;
        if (len >= (ssize_t)sizeof(ChunkHeader) && Faults.Deliver(now, due))
            pending.emplace(due, std::vector<u8>(buf, buf + len));
        bool got = false;
        while (!pending.empty() && pending.begin()->first <= now)
        {
            std::vector<u8> p = std::move(pending.begin()->second);
            pending.erase(pending.begin());
            ChunkHeader h;
            memcpy(&h, p.data(), sizeof(h));
            if (h.Magic != kChunkMagic || h.Console != Console || p.size() != sizeof(h) + h.Len) continue;
            got = true;
            if (h.Offset > Expected)
            {
                // keep the longer copy (a re-sent chunk can arrive after the packet that carried it
                // with the next one): the server may already know this client holds all of it
                auto it = Ahead.find(h.Offset);
                if (it != Ahead.end() ? it->second.size() < h.Len : Ahead.size() < 1024)
                    Ahead[h.Offset].assign(p.begin() + sizeof(h), p.end());
                continue;
            }
            Ahead[h.Offset].assign(p.begin() + sizeof(h), p.end());
            // everything contiguous from Expected on
            while (!Ahead.empty() && Ahead.begin()->first <= Expected)
            {
                auto it = Ahead.begin();
                u64 end = it->first + it->second.size();
                if (end > Expected)
                {
                    Sink(it->second.data() + (Expected - it->first), end - Expected);
                    Expected = end;
                }
                Ahead.erase(it);
            }
        }
        if (got || now - lastAck >= kResendMs * 1000)
        {
            SendAck();
            lastAck = now;
        }
    }
}

}
