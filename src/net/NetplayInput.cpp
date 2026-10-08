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

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/tcp.h>
#include <poll.h>
#include <unistd.h>
#include <cmath>
#include <cstdarg>
#include <cstdint>
#include <cstdio>
#include <algorithm>
#include <chrono>
#include <cstring>
#include <vector>

#include "NetplayInput.h"

namespace melonDS
{

namespace
{
constexpr u32 kMagic = 0x4E504932; // "NPI2"
constexpr u32 kHashMagic = 0x4E504831; // "NPH1"

#pragma pack(push, 1)
struct WireEntry
{
    s32 Frame;
    u32 Keys;
    s16 TouchX, TouchY;
};
struct WireHeader
{
    u32 Magic;
    u8 Player;
    u8 Count;
    s32 Ack;    // the sender has all of the receiver's inputs up to this applied frame
};
#pragma pack(pop)

struct WireHash
{
    u32 Magic;
    u8 Player;
    s32 Frame;
    u64 Hash;
};

u64 NowUs()
{
    return (u64)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

static sockaddr_in ParseAddr(const std::string& ipPort)
{
    sockaddr_in a {};
    a.sin_family = AF_INET;
    size_t colon = ipPort.rfind(':');
    inet_pton(AF_INET, ipPort.substr(0, colon).c_str(), &a.sin_addr);
    a.sin_port = htons((u16)atoi(ipPort.substr(colon + 1).c_str()));
    return a;
}

NetplayInput::NetplayInput(int localPlayer, int delayFrames, int bindPort, const std::vector<std::pair<int, std::string>>& peers, int latencyMs, const NetFaults& faults)
    : Local(localPlayer), DelayFrames(delayFrames), Faults(faults)
{
    Faults.LatencyMs = latencyMs;
    for (int p = 0; p < kMaxPlayers; p++)
        PeerAck[p] = RemoteUpTo[p] = delayFrames - 1; // frames before Delay are never sent

    Socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (Socket < 0) return;

    sockaddr_in bindAddr {};
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bindAddr.sin_port = htons((u16)bindPort);
    timeval tv {0, 1000}; // 1 ms, so delayed packets are released on time
    setsockopt(Socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    bool ok = bind(Socket, (sockaddr*)&bindAddr, sizeof(bindAddr)) == 0 && !peers.empty();
    for (auto& [player, addr] : peers)
    {
        if (player < 0 || player >= kMaxPlayers || player == Local || PeerSet[player]) { ok = false; break; }
        sockaddr_in a = ParseAddr(addr);
        static_assert(sizeof(a) <= sizeof(PeerAddr[0]));
        memcpy(PeerAddr[player], &a, sizeof(a));
        PeerSet[player] = true;
        Peers.push_back(player);
    }
    if (!ok)
    {
        close(Socket);
        Socket = -1;
        return;
    }

    Receiver = std::thread([this] { ReceiveLoop(); });
}

NetplayInput::~NetplayInput()
{
    Running = false;
    if (Receiver.joinable()) Receiver.join();
    if (Socket >= 0) close(Socket);
}

void NetplayInput::SubmitLocal(int frame, const NetplayFrameInput& input)
{
    int applied = frame + DelayFrames;
    {
        std::lock_guard<std::mutex> lk(Lock);
        Inputs[Local][applied] = input;
        Unacked.emplace_back(applied, input);
        Changed.notify_all();
    }
    Send();
}

void NetplayInput::Send()
{
    std::vector<u8> packet;
    for (int peer : Peers)
    {
        {
            std::lock_guard<std::mutex> lk(Lock);
            // what this peer has not acknowledged yet (Unacked also holds what only others lack)
            size_t first = 0;
            while (first < Unacked.size() && Unacked[first].first <= PeerAck[peer]) first++;
            int count = std::min<int>((int)(Unacked.size() - first), kMaxPerPacket);
            WireHeader header {kMagic, (u8)Local, (u8)count, RemoteUpTo[peer]};
            packet.resize(sizeof(header) + count * sizeof(WireEntry));
            memcpy(packet.data(), &header, sizeof(header));
            u8* p = packet.data() + sizeof(header);
            for (int i = 0; i < count; i++, p += sizeof(WireEntry))
            {
                auto& [f, in] = Unacked[first + i];
                WireEntry e {f, in.Keys, in.TouchX, in.TouchY};
                memcpy(p, &e, sizeof(e));
            }
        }
        sendto(Socket, packet.data(), packet.size(), 0, (sockaddr*)PeerAddr[peer], sizeof(sockaddr_in));
    }
}

void NetplayInput::SendHash(int frame, u64 hash)
{
    WireHash h {kHashMagic, (u8)Local, frame, hash};
    for (int peer : Peers)
        sendto(Socket, &h, sizeof(h), 0, (sockaddr*)PeerAddr[peer], sizeof(sockaddr_in));
}

bool NetplayInput::PeerHash(int player, int frame, u64& hash)
{
    if (!IsPeer(player)) return false;
    std::lock_guard<std::mutex> lk(Lock);
    auto it = PeerHashes[player].find(frame);
    if (it == PeerHashes[player].end()) return false;
    hash = it->second;
    return true;
}

double NetplayInput::MsSincePeer() const
{
    u64 oldest = UINT64_MAX;
    for (int peer : Peers) oldest = std::min<u64>(oldest, LastPeerUs[peer].load());
    return (NowUs() - oldest) / 1000.0;
}

void NetplayInput::Abort()
{
    Running = false;
    std::lock_guard<std::mutex> lk(Lock);
    Changed.notify_all();
}

NetplayFrameInput NetplayInput::Get(int player, int frame)
{
    if (frame < DelayFrames) return {}; // nobody has input before the delay has elapsed
    if (player < 0 || player >= kMaxPlayers) return {};

    std::unique_lock<std::mutex> lk(Lock);
    auto& inputs = Inputs[player];
    if (inputs.find(frame) == inputs.end())
    {
        u64 start = NowUs();
        Changed.wait(lk, [&] { return inputs.find(frame) != inputs.end() || !Running; });
        if (player != Local) StallUs += NowUs() - start;
    }
    NetplayFrameInput in = inputs[frame];
    inputs.erase(inputs.begin(), inputs.lower_bound(frame - 64)); // keep a little history
    return in;
}

void NetplayInput::ReceiveLoop()
{
    std::multimap<u64, std::vector<u8>> pending; // artificial latency: (due time, packet)
    u8 buf[2048];
    u64 lastSend = 0;
    while (Running)
    {
        // heartbeat: re-sends what each peer has not acknowledged, and our acknowledgements
        if (NowUs() - lastSend >= kResendMs * 1000)
        {
            Send();
            lastSend = NowUs();
        }

        ssize_t len = recv(Socket, buf, sizeof(buf), 0);
        u64 now = NowUs();
        if (len > 4 && IsPeer(buf[4])) LastPeerUs[buf[4]] = now; // both packet kinds: magic, player
        u64 due;
        if (len >= (ssize_t)sizeof(WireHeader) && Faults.Deliver(now, due))
            pending.emplace(due, std::vector<u8>(buf, buf + len));

        while (!pending.empty() && pending.begin()->first <= now)
        {
            std::vector<u8>& packet = pending.begin()->second;
            WireHash wh;
            if (packet.size() == sizeof(WireHash) && (memcpy(&wh, packet.data(), sizeof(wh)), wh.Magic == kHashMagic))
            {
                if (IsPeer(wh.Player))
                {
                    std::lock_guard<std::mutex> lk(Lock);
                    auto& hashes = PeerHashes[wh.Player];
                    hashes[wh.Frame] = wh.Hash;
                    while (hashes.size() > 64) hashes.erase(hashes.begin());
                }
                pending.erase(pending.begin());
                continue;
            }
            WireHeader header;
            memcpy(&header, packet.data(), sizeof(header));
            if (header.Magic == kMagic && IsPeer(header.Player) &&
                packet.size() >= sizeof(header) + header.Count * sizeof(WireEntry))
            {
                std::lock_guard<std::mutex> lk(Lock);
                int from = header.Player;
                auto& inputs = Inputs[from];
                const u8* p = packet.data() + sizeof(header);
                for (int i = 0; i < header.Count; i++, p += sizeof(WireEntry))
                {
                    WireEntry e;
                    memcpy(&e, p, sizeof(e));
                    if (e.Frame > RemoteUpTo[from]) // older ones were received (and maybe consumed) already
                        inputs.emplace(e.Frame, NetplayFrameInput {e.Keys, e.TouchX, e.TouchY});
                }
                while (inputs.count(RemoteUpTo[from] + 1)) RemoteUpTo[from]++;
                PeerAck[from] = std::max(PeerAck[from], header.Ack);
                int allAcked = INT32_MAX;
                for (int peer : Peers) allAcked = std::min(allAcked, PeerAck[peer]);
                while (!Unacked.empty() && Unacked.front().first <= allAcked) Unacked.pop_front();
                Changed.notify_all();
            }
            pending.erase(pending.begin());
        }
    }
    std::lock_guard<std::mutex> lk(Lock);
    Changed.notify_all();
}

// ---- session start ----

namespace
{
constexpr u32 kSetupMagic = 0x4E505331; // "NPS1"
constexpr int kPings = 12;
constexpr u32 kMaxSave = 32u << 20;

bool SendAll(int fd, const void* p, size_t n)
{
#ifdef MSG_NOSIGNAL
    constexpr int flags = MSG_NOSIGNAL;
#else
    constexpr int flags = 0;    // macOS: SO_NOSIGPIPE on the socket
#endif
    for (const u8* b = (const u8*)p; n; )
    {
        ssize_t k = send(fd, b, n, flags);
        if (k <= 0) return false;
        b += k; n -= k;
    }
    return true;
}

bool RecvAll(int fd, void* p, size_t n)
{
    for (u8* b = (u8*)p; n; )
    {
        ssize_t k = recv(fd, b, n, 0);
        if (k <= 0) return false;
        b += k; n -= k;
    }
    return true;
}

template <typename T> bool SendVal(int fd, T v) { return SendAll(fd, &v, sizeof(v)); }
template <typename T> bool RecvVal(int fd, T& v) { return RecvAll(fd, &v, sizeof(v)); }

bool SendBlob(int fd, const std::vector<u8>& b)
{
    return SendVal(fd, (u32)b.size()) && (b.empty() || SendAll(fd, b.data(), b.size()));
}

bool RecvBlob(int fd, std::vector<u8>& b)
{
    u32 len;
    if (!RecvVal(fd, len) || len > kMaxSave) return false;
    b.resize(len);
    return !len || RecvAll(fd, b.data(), len);
}

void SetupSocket(int fd, int timeoutS)
{
    timeval tv {timeoutS, 0};   // a guest waits here for the last one to join
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    int one = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
#ifdef SO_NOSIGPIPE
    setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &one, sizeof(one));
#endif
}

std::string IpString(u32 ip) // network order
{
    char s[INET_ADDRSTRLEN] = {0};
    in_addr a {};
    a.s_addr = ip;
    inet_ntop(AF_INET, &a, s, sizeof(s));
    return s;
}

void AddLog(std::string& log, const char* fmt, ...) __attribute__((format(printf, 2, 3)));
void AddLog(std::string& log, const char* fmt, ...)
{
    char line[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    log += line;
    log += '\n';
}

bool HostHandshake(NetplaySetup& s)
{
    struct Guest { int Fd = -1; int Player = 0; u32 Ip = 0; u16 Port = 0; u64 Rom = 0; std::vector<u8> Save; double OneWayMs = 0; };
    std::vector<Guest> guests;
    auto closeAll = [&] { for (Guest& g : guests) if (g.Fd >= 0) close(g.Fd); };

    int ls = socket(AF_INET, SOCK_STREAM, 0);
    int one = 1;
    setsockopt(ls, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));
    sockaddr_in a {};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_ANY);
    a.sin_port = htons((u16)(s.Port + 1));
    if (ls < 0 || bind(ls, (sockaddr*)&a, sizeof(a)) != 0 || listen(ls, NetplayInput::kMaxPlayers) != 0)
    {
        AddLog(s.Log, "Netplay: cannot listen on port %d", s.Port + 1);
        if (ls >= 0) close(ls);
        return false;
    }
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(s.TimeoutS);
    const int numGuests = s.NumPlayers - (s.HostPlays ? 1 : 0);
    while ((int)guests.size() < numGuests)
    {
        int ms = (int)std::chrono::duration_cast<std::chrono::milliseconds>(deadline - std::chrono::steady_clock::now()).count();
        pollfd pf {ls, POLLIN, 0};
        if (ms <= 0 || poll(&pf, 1, ms) != 1) break;
        sockaddr_in from {};
        socklen_t fromLen = sizeof(from);
        Guest g;
        g.Fd = accept(ls, (sockaddr*)&from, &fromLen);
        if (g.Fd < 0) continue;
        SetupSocket(g.Fd, s.TimeoutS + 30);
        g.Ip = from.sin_addr.s_addr;
        u32 magic = 0;
        u8 player = 0;
        bool ok = RecvVal(g.Fd, magic) && magic == kSetupMagic && RecvVal(g.Fd, player) && RecvVal(g.Fd, g.Port)
               && RecvVal(g.Fd, g.Rom) && RecvBlob(g.Fd, g.Save);
        g.Player = player;
        bool dup = false;
        for (Guest& o : guests) dup |= o.Player == g.Player;
        if (!ok || (g.Player == 0 && s.HostPlays) || g.Player >= NetplayInput::kMaxPlayers || dup)
        {
            AddLog(s.Log, "Netplay: rejected a connection from %s (player %d)", IpString(g.Ip).c_str(), g.Player);
            close(g.Fd);
            continue;
        }
        AddLog(s.Log, "Netplay: player %d joined from %s (save %zu bytes)", g.Player, IpString(g.Ip).c_str(), g.Save.size());
        guests.push_back(std::move(g));
    }
    close(ls);
    if ((int)guests.size() < numGuests)
    {
        AddLog(s.Log, "Netplay: only %zu of %d other players connected", guests.size(), numGuests);
        closeAll();
        return false;
    }

    // Input delay: time a few round trips to every guest. An input travels between every pair of
    // players, guest to guest too, so the worst pair is bounded by the two slowest guests' one-way
    // times added (the path through the host); 90th percentiles, plus a frame of margin.
    // ponytail: guest-to-guest is bounded, not measured (over-estimates by up to a frame through
    // a shared access point); measure every pair if that delay ever costs too much.
    bool ok = true;
    for (Guest& g : guests)
    {
        std::vector<double> rtt;
        for (int i = 0; ok && i < kPings; i++)
        {
            u8 b = (u8)i;
            auto t0 = std::chrono::steady_clock::now();
            ok = SendVal(g.Fd, b) && RecvVal(g.Fd, b);
            rtt.push_back(std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        }
        if (!ok) { AddLog(s.Log, "Netplay: player %d stopped answering", g.Player); break; }
        std::sort(rtt.begin(), rtt.end());
        g.OneWayMs = rtt[rtt.size() * 9 / 10] / 2;
        AddLog(s.Log, "Netplay: player %d round trip median %.1f ms, p90 %.1f ms", g.Player, rtt[rtt.size() / 2], g.OneWayMs * 2);
    }
    if (ok && s.Delay <= 0)
    {
        std::vector<double> oneWay;
        for (Guest& g : guests) oneWay.push_back(g.OneWayMs);
        std::sort(oneWay.rbegin(), oneWay.rend());
        double worst = s.Hosted ? 2 * oneWay[0] : oneWay[0] + (oneWay.size() > 1 ? oneWay[1] : 0);
        s.Delay = std::clamp((int)std::ceil(worst / (1000.0 / 60)) + 1, 1, 8);
        AddLog(s.Log, "Netplay: worst one way %.1f ms -> input delay %d frames", worst, s.Delay);
    }

    u8 status = 0;
    for (Guest& g : guests)
        if (g.Rom != s.RomId)
        {
            AddLog(s.Log, "Netplay: player %d runs a different game", g.Player);
            status = 1;
        }
    // every guest: status, delay, then every player (address 0 = the host itself) with its save
    for (Guest& g : guests)
    {
        ok = ok && SendVal(g.Fd, kSetupMagic) && SendVal(g.Fd, status) && SendVal(g.Fd, (u8)s.Delay)
                && SendVal(g.Fd, (u8)(guests.size() + (s.HostPlays ? 1 : 0)));
        if (s.HostPlays)
            ok = ok && SendVal(g.Fd, (u8)0) && SendVal(g.Fd, (u32)0) && SendVal(g.Fd, (u16)s.Port) && SendBlob(g.Fd, s.Save);
        for (Guest& o : guests)
            ok = ok && SendVal(g.Fd, (u8)o.Player) && SendVal(g.Fd, o.Ip) && SendVal(g.Fd, o.Port)
                    && SendBlob(g.Fd, &o == &g ? std::vector<u8>() : o.Save);
    }
    // wait for every guest to have it all (it closes first), so nobody's data is cut short
    for (Guest& g : guests) { u8 b; recv(g.Fd, &b, 1, 0); }
    closeAll();
    if (!ok) { AddLog(s.Log, "Netplay: session setup failed"); return false; }
    if (status) return false;
    for (Guest& g : guests)
    {
        s.Peers.emplace_back(g.Player, IpString(g.Ip) + ":" + std::to_string(g.Port));
        s.Saves[g.Player] = std::move(g.Save);
    }
    return true;
}

bool GuestHandshake(NetplaySetup& s)
{
    sockaddr_in host = ParseAddr(s.Host);
    host.sin_port = htons((u16)(ntohs(host.sin_port) + 1));
    int fd = -1;
    auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(s.TimeoutS);
    while (fd < 0 && std::chrono::steady_clock::now() < deadline)
    {
        fd = socket(AF_INET, SOCK_STREAM, 0);
        if (connect(fd, (sockaddr*)&host, sizeof(host)) != 0)
        {
            close(fd);
            fd = -1;
            usleep(500000);
        }
    }
    if (fd < 0)
    {
        AddLog(s.Log, "Netplay: could not reach the host %s", s.Host.c_str());
        return false;
    }
    SetupSocket(fd, s.TimeoutS + 30);
    bool ok = SendVal(fd, kSetupMagic) && SendVal(fd, (u8)s.Player) && SendVal(fd, (u16)s.Port)
           && SendVal(fd, s.RomId) && SendBlob(fd, s.Save);
    for (int i = 0; ok && i < kPings; i++)
    {
        u8 b;
        ok = RecvVal(fd, b) && SendVal(fd, b);
    }
    u32 magic = 0;
    u8 status = 1, delay = 0, count = 0;
    ok = ok && RecvVal(fd, magic) && magic == kSetupMagic && RecvVal(fd, status) && RecvVal(fd, delay)
            && RecvVal(fd, count) && delay >= 1 && delay <= 8;
    std::string hostIp = s.Host.substr(0, s.Host.rfind(':'));
    bool self = false;
    for (int i = 0; ok && i < count; i++)
    {
        u8 player;
        u32 ip;
        u16 port;
        std::vector<u8> save;
        ok = RecvVal(fd, player) && RecvVal(fd, ip) && RecvVal(fd, port) && RecvBlob(fd, save)
          && player < NetplayInput::kMaxPlayers;
        if (!ok) break;
        if (player == s.Player) { self = true; continue; }
        s.Peers.emplace_back(player, (ip ? IpString(ip) : hostIp) + ":" + std::to_string(port));
        s.Saves[player] = std::move(save);
    }
    close(fd);
    if (!ok || !self)
    {
        AddLog(s.Log, "Netplay: session setup with the host failed");
        return false;
    }
    if (status)
    {
        AddLog(s.Log, "Netplay: a player runs a different game");
        return false;
    }
    s.Delay = delay;
    AddLog(s.Log, "Netplay: %d players, input delay %d frames", (int)count, s.Delay);
    return true;
}
}

bool NetplayHandshake(NetplaySetup& s)
{
    s.Peers.clear();
    s.Saves.clear();
    return s.Player == 0 && !s.Join ? HostHandshake(s) : GuestHandshake(s);
}

}
