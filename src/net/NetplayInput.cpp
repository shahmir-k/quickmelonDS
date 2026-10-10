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
#include <filesystem>
#include <mutex>
#include <tuple>
#include <vector>

#include "NetplayInput.h"
#include "../Platform.h"
#include "../teakra/src/makedsp1/sha256.h"
#if defined(__aarch64__)
#include <arm_neon.h>
#ifndef __APPLE__
#include <sys/auxv.h>
#include <asm/hwcap.h>
#endif
#endif

namespace melonDS
{

namespace
{
constexpr u32 kHashMagic = 0x4E504831; // "NPH1"

#pragma pack(push, 1)
#ifdef LITEV_NP_WIRE_RUNS
constexpr u32 kMagic = 0x4E504933; // "NPI3"
struct WireEntry    // Len frames from Frame on, all with this input
{
    s32 Frame;
    u16 Len;
    u32 Keys;
    s16 TouchX, TouchY;
};
struct WireHeader
{
    u32 Magic;
    u8 Player;
    u8 Count;
    s32 Ack;    // the sender has all of the receiver's inputs up to this applied frame
    u32 SentUs;     // the sender's clock now
    u32 EchoUs;     // the receiver's SentUs in the last packet the sender took (0 = none yet)
    u32 EchoAgeUs;  // how long ago the sender took it: round trip = now - EchoUs - EchoAgeUs
};
#else
constexpr u32 kMagic = 0x4E504932; // "NPI2"
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
#endif
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
    : Local(localPlayer), DelayFrames(delayFrames), Faults(faults), CurDelay(delayFrames)
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

#ifdef LITEV_NP_ADAPTIVE_DELAY
// The delay this player's inputs need: the slowest peer's round trip (90th percentile, last 2 s),
// halved, plus a frame. Up at once; down one frame
// at a time, only after the need stayed lower for 2 s (a shorter delay merges one input sample).
void NetplayInput::Adapt(int frame)
{
    s64 rtt = RttMaxUs.load();
    if (rtt < 0 || frame < DelayFrames) return;
    int need = std::clamp((int)std::ceil(rtt / 2.0 / (1e6 / 60)) + 1, 1, kMaxDelay);
    int cur = CurDelay.load();
    if (need > cur) { CurDelay = need; LowFrames = 0; }
    else if (need < cur && ++LowFrames >= 120) { CurDelay = cur - 1; LowFrames = 90; }   // then -1 per 30 frames
    else if (need >= cur) LowFrames = 0;
}
#endif

void NetplayInput::SubmitLocal(int frame, const NetplayFrameInput& input)
{
#ifdef LITEV_NP_ADAPTIVE_DELAY
    if (Adaptive) Adapt(frame);
#endif
    int applied = frame + CurDelay.load();
    {
        std::lock_guard<std::mutex> lk(Lock);
#ifdef LITEV_NP_ADAPTIVE_DELAY
        // the applied frames stay contiguous: a longer delay holds the previous input over the
        // gap, a shorter one merges this sample into the next (its presses are kept)
        NetplayFrameInput in = input;
        in.Keys &= Carry.Keys;  // active low
        if (in.TouchX < 0) { in.TouchX = Carry.TouchX; in.TouchY = Carry.TouchY; }
        if (LastApplied >= 0 && applied <= LastApplied) { Carry = in; return; }
        Carry = {};
        for (int f = LastApplied + 1; LastApplied >= 0 && f < applied; f++)
        {
            Inputs[Local][f] = LastInput;
            Unacked.emplace_back(f, LastInput);
        }
        LastApplied = applied;
        LastInput = in;
#else
        const NetplayFrameInput& in = input;
#endif
        Inputs[Local][applied] = in;
        Unacked.emplace_back(applied, in);
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
#ifdef LITEV_NP_WIRE_RUNS
            // runs of equal inputs over consecutive frames, oldest first
            std::vector<WireEntry> runs;
            for (size_t i = first; i < Unacked.size(); i++)
            {
                auto& [f, in] = Unacked[i];
                WireEntry* r = runs.empty() ? nullptr : &runs.back();
                if (r && r->Frame + r->Len == f && r->Len < 0xFFFF && r->Keys == in.Keys && r->TouchX == in.TouchX && r->TouchY == in.TouchY)
                    r->Len++;
                else if (runs.size() == kMaxPerPacket) break;
                else runs.push_back({f, 1, in.Keys, in.TouchX, in.TouchY});
            }
            int count = (int)runs.size();
            u64 now = NowUs();
            WireHeader header {kMagic, (u8)Local, (u8)count, RemoteUpTo[peer], (u32)now, EchoSent[peer], EchoSent[peer] ? (u32)(now - EchoAt[peer]) : 0};
            packet.resize(sizeof(header) + count * sizeof(WireEntry));
            memcpy(packet.data(), &header, sizeof(header));
            if (count) memcpy(packet.data() + sizeof(header), runs.data(), count * sizeof(WireEntry));
#else
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
#endif
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

int NetplayInput::AckedBy(int peer)
{
    if (!IsPeer(peer)) return INT32_MAX;
    std::lock_guard<std::mutex> lk(Lock);
    return PeerAck[peer];
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
    if (Dropped(player)) return {};
    if (inputs.find(frame) == inputs.end())
    {
        u64 start = NowUs();
        auto known = [&] { return inputs.find(frame) != inputs.end() || !Running; };
        if (DropAfterMs <= 0 || player == Local) Changed.wait(lk, known);
        else
            // late input only stalls (a paused guest still sends its 10 ms heartbeat); only a
            // device silent for DropAfterMs is dropped
            while (!Changed.wait_for(lk, std::chrono::milliseconds(100), known))
                if (NowUs() - std::max(start, LastPeerUs[player].load()) > (u64)DropAfterMs * 1000)
                {
                    DroppedMask |= 1u << player;
                    Platform::Log(Platform::LogLevel::Warn, "Netplay: player %d dropped at frame %d: silent for %d ms; it plays on with nothing pressed\n",
                                  player, frame, DropAfterMs);
                    break;
                }
        if (player != Local) StallUs += NowUs() - start;
    }
    if (Dropped(player)) return {};
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
            if (header.Magic == kMagic && IsPeer(header.Player) && !Dropped(header.Player) &&
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
#ifdef LITEV_NP_WIRE_RUNS
                    for (int k = std::max(0, RemoteUpTo[from] + 1 - e.Frame); k < e.Len; k++)
                        inputs.emplace(e.Frame + k, NetplayFrameInput {e.Keys, e.TouchX, e.TouchY});
#else
                    if (e.Frame > RemoteUpTo[from]) // older ones were received (and maybe consumed) already
                        inputs.emplace(e.Frame, NetplayFrameInput {e.Keys, e.TouchX, e.TouchY});
#endif
                }
#ifdef LITEV_NP_WIRE_RUNS
                // round trip: our send time echoed back, less how long the peer held it
                EchoSent[from] = header.SentUs;
                EchoAt[from] = now;
                if (header.EchoUs)
                {
                    auto& rtt = Rtt[from];
                    rtt.emplace_back(now, (u32)now - header.EchoUs - header.EchoAgeUs);
                    while (rtt.front().first + 2000000 < now) rtt.pop_front();
                    // the slowest peer's 90th percentile over the last 2 s: a lone Wi-Fi spike costs
                    // one stall, not seconds of extra input lag
                    u32 worst = 0;
                    for (int peer : Peers)
                    {
                        std::vector<u32> us;
                        for (auto& [t, v] : Rtt[peer]) us.push_back(v);
                        if (us.empty()) continue;
                        std::nth_element(us.begin(), us.begin() + us.size() * 9 / 10, us.end());
                        worst = std::max(worst, us[us.size() * 9 / 10]);
                    }
                    RttMaxUs = worst;
                }
#endif
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

// ---- ROM identity ----

namespace
{
#if defined(__aarch64__)
// ARMv8 SHA-256 instructions (the RG DS's A55s have them): the SD card, not the hash, is then
// the cost of describing a ROM
const u32 kShaK[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

__attribute__((target("sha2"))) void ShaBlocksArm(u32 state[8], const u8* p, size_t blocks)
{
    uint32x4_t st0 = vld1q_u32(state), st1 = vld1q_u32(state + 4);
    for (; blocks--; p += 64)
    {
        uint32x4_t s0 = st0, s1 = st1, w[4];
        for (int i = 0; i < 4; i++) w[i] = vreinterpretq_u32_u8(vrev32q_u8(vld1q_u8(p + 16 * i)));
        for (int i = 0; i < 16; i++)
        {
            uint32x4_t t = vaddq_u32(w[i & 3], vld1q_u32(kShaK + 4 * i)), a = st0;
            st0 = vsha256hq_u32(st0, st1, t);
            st1 = vsha256h2q_u32(st1, a, t);
            if (i < 12) w[i & 3] = vsha256su1q_u32(vsha256su0q_u32(w[i & 3], w[(i + 1) & 3]), w[(i + 2) & 3], w[(i + 3) & 3]);
        }
        st0 = vaddq_u32(st0, s0);
        st1 = vaddq_u32(st1, s1);
    }
    vst1q_u32(state, st0);
    vst1q_u32(state + 4, st1);
}

bool HasShaArm()
{
#ifdef __APPLE__
    return true;
#else
    static const bool has = getauxval(AT_HWCAP) & HWCAP_SHA2;
    return has;
#endif
}
#endif

// SHA-256 (teakra's makedsp1 implementation; whole blocks on the ARMv8 instructions when present)
struct Sha256
{
    SHA256_CTX C;
    Sha256() { sha256_init(&C); }
    void Add(const u8* p, size_t n)
    {
#if defined(__aarch64__)
        if (C.datalen == 0 && HasShaArm())
        {
            size_t blocks = n / 64;
            ShaBlocksArm(C.state, p, blocks);
            C.bitlen += blocks * 512;
            p += blocks * 64;
            n -= blocks * 64;
        }
#endif
        sha256_update(&C, p, n);
    }
    void Get(u8 out[32]) { sha256_final(&C, out); }
};

std::string TrimText(const char* p, size_t n)
{
    std::string t(p, strnlen(p, n));
    while (!t.empty() && t.back() == ' ') t.pop_back();
    return t;
}

// a file's size and first 16 header bytes (title + game code); false = unreadable or too short
bool RomHead(Platform::FileHandle* f, u64& size, u8 head[16])
{
    size = Platform::FileLength(f);
    Platform::FileRewind(f);
    return size >= 16 && Platform::FileRead(head, 16, 1, f) == 1;
}
}

std::string NetplayRom::Hex() const
{
    char h[65];
    for (int i = 0; i < 32; i++) snprintf(h + 2 * i, 3, "%02x", Sha[i]);
    return h;
}

std::string NetplayRom::GameTitle() const { return TrimText(Title, 12); }

std::string NetplayRom::Name() const
{
    char line[200];
    snprintf(line, sizeof(line), "%s \"%s\" (sha256 %s, %llu bytes)", TrimText(GameCode, 4).c_str(),
             GameTitle().c_str(), Hex().c_str(), (unsigned long long)Size);
    return line;
}

NetplayRom NetplayDescribeRom(const std::string& path)
{
    // ponytail: keyed by path + size, so a file replaced in place by one of the same size keeps
    // its old hash until the app restarts; add the mtime if that ever happens
    static std::mutex lock;
    static std::map<std::pair<std::string, u64>, NetplayRom> known;
    NetplayRom rom;
    Platform::FileHandle* f = Platform::OpenFile(path, Platform::FileMode::Read);
    if (!f) return rom;
    u8 head[16];
    u64 size;
    if (RomHead(f, size, head))
    {
        std::lock_guard<std::mutex> lk(lock);
        auto it = known.find({path, size});
        if (it != known.end()) rom = it->second;
        else
        {
            Sha256 sha;
            std::vector<u8> buf(1 << 20);
            Platform::FileRewind(f);
            u64 left = size;
            while (left)
            {
                u64 n = std::min<u64>(left, buf.size());
                if (Platform::FileRead(buf.data(), n, 1, f) != 1) break;
                sha.Add(buf.data(), n);
                left -= n;
            }
            if (!left)
            {
                sha.Get(rom.Sha);
                rom.Size = size;
                memcpy(rom.Title, head, 12);
                memcpy(rom.GameCode, head + 12, 4);
                known[{path, size}] = rom;
            }
        }
    }
    Platform::CloseFile(f);
    return rom;
}

std::string NetplayFindRom(const NetplayRom& rom, const std::vector<std::string>& paths)
{
    for (const std::string& path : paths)
    {
        Platform::FileHandle* f = Platform::OpenFile(path, Platform::FileMode::Read);
        if (!f) continue;
        u8 head[16];
        u64 size;
        bool maybe = RomHead(f, size, head) && size == rom.Size && !memcmp(head + 12, rom.GameCode, 4);
        Platform::CloseFile(f);
        if (maybe && NetplayDescribeRom(path) == rom) return path;
    }
    return "";
}

// ---- session start ----

namespace
{
constexpr u32 kSetupMagic = 0x4E505333; // "NPS3" (NPS2: XXH3 ROM ids, no transfer)
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

// ---- ROM transfer (inside session setup) ----

constexpr u32 kChunk = 1u << 20;
constexpr int kStallS = 15;                 // a transfer fails after this long without progress
enum : u8 { kData, kFill, kEnd };           // stream frames: data, a run of one byte value, end
enum : u8 { kAsk, kUpload, kRecv, kTick, kDone };   // host -> guest, after the roster

std::mutex gXferLock;
std::condition_variable gXferCv;
NetplayXferStatus gXfer;
int gAnswer = -1;                           // NetplayAnswer: -1 none yet, 0 no, 1 yes
std::atomic<bool> gCancel {false};
std::vector<int> gSetupFds;                 // the setup sockets, which a cancel shuts down

void SetXfer(int state, u64 bytes = 0, u64 total = 0, const std::string& title = "")
{
    std::lock_guard<std::mutex> lk(gXferLock);
    gXfer = NetplayXferStatus {};
    gXfer.State = state;
    gXfer.Bytes = bytes;
    gXfer.Total = total;
    gXfer.Title = title;
}

void XferBytes(u64 bytes)
{
    std::lock_guard<std::mutex> lk(gXferLock);
    gXfer.Bytes = bytes;
}

void TrackFd(int fd, bool on)
{
    std::lock_guard<std::mutex> lk(gXferLock);
    if (on)
    {
        gSetupFds.push_back(fd);
        if (gCancel) shutdown(fd, SHUT_RDWR);
    }
    else gSetupFds.erase(std::remove(gSetupFds.begin(), gSetupFds.end(), fd), gSetupFds.end());
}

void SetTimeout(int fd, int seconds)
{
    timeval tv {seconds, 0};
    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
}

std::string Megabytes(u64 bytes)
{
    char t[32];
    snprintf(t, sizeof(t), "%llu MB", (unsigned long long)((bytes + (1 << 20) - 1) >> 20));
    return t;
}

std::string CachePath(const NetplaySetup& s, const NetplayRom& rom, const char* ext)
{
    return s.CacheDir + "/" + rom.Hex() + ext;
}

// where this device has `rom`: the cache (a received copy, verified when it arrived; marked as
// just used), else FindRom
std::string Resolve(const NetplaySetup& s, const NetplayRom& rom)
{
    if (!s.CacheDir.empty())
    {
        std::error_code ec;
        std::string path = CachePath(s, rom, ".nds");
        if (std::filesystem::file_size(path, ec) == rom.Size && !ec)
        {
            std::filesystem::last_write_time(path, std::filesystem::file_time_type::clock::now(), ec);
            return path;
        }
    }
    return s.FindRom ? s.FindRom(rom) : "";
}

// The cache's size cap: least recently used first (.part files count, and go too), never one
// this session uses.
void TrimCache(NetplaySetup& s, const std::map<int, NetplayRom>& inUse)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    std::vector<std::tuple<fs::file_time_type, u64, fs::path>> files;
    u64 total = 0;
    for (const fs::directory_entry& e : fs::directory_iterator(s.CacheDir, ec))
    {
        std::string ext = e.path().extension().string();
        if ((ext != ".nds" && ext != ".part") || !e.is_regular_file(ec)) continue;
        u64 size = e.file_size(ec);
        files.emplace_back(e.last_write_time(ec), size, e.path());
        total += size;
    }
    std::sort(files.begin(), files.end());
    for (auto& [time, size, path] : files)
    {
        if (total <= s.CacheMaxBytes) break;
        bool used = false;
        for (auto& [p, rom] : inUse) used |= path.stem().string() == rom.Hex();
        if (used || !fs::remove(path, ec)) continue;
        total -= size;
        AddLog(s.Log, "Netplay: ROM cache over %s: removed %s", Megabytes(s.CacheMaxBytes).c_str(), path.filename().string().c_str());
    }
}

// a ROM on its way here: CacheDir/<sha>.part and the SHA-256 of what it holds so far
struct Incoming
{
    NetplayRom Rom;
    u64 Have = 0;
    Sha256 Sha;
};

// what an earlier, interrupted transfer left of `rom` (hashed now, so the end needs no second pass)
void StartIncoming(const NetplaySetup& s, Incoming& in)
{
    std::string part = CachePath(s, in.Rom, ".part");
    std::error_code ec;
    std::filesystem::create_directories(s.CacheDir, ec);
    in.Have = 0;
    in.Sha = Sha256();
    FILE* f = fopen(part.c_str(), "rb");
    if (!f) return;
    std::vector<u8> buf(kChunk);
    size_t n;
    while (in.Have < in.Rom.Size && (n = fread(buf.data(), 1, std::min<u64>(buf.size(), in.Rom.Size - in.Have), f)) > 0)
    {
        in.Sha.Add(buf.data(), n);
        in.Have += n;
    }
    fclose(f);
    if (in.Have == in.Rom.Size) // all there but never checked (or damaged): start over
    {
        remove(part.c_str());
        in.Have = 0;
        in.Sha = Sha256();
    }
}

// Sends `rom` (at `path`) from `offset` as stream frames. tick: after every chunk.
// Test switches: LITEV_NP_XFER_CUT=<bytes> drops the connection once the file position would pass
// that many bytes; LITEV_NP_XFER_CORRUPT=1 flips one byte of the first data chunk. Each acts once
// per process.
bool SendRom(int fd, const std::string& path, const NetplayRom& rom, u64 offset, const std::function<void()>& tick)
{
    static u64 cut = getenv("LITEV_NP_XFER_CUT") ? strtoull(getenv("LITEV_NP_XFER_CUT"), nullptr, 10) : 0;
    static bool corrupt = getenv("LITEV_NP_XFER_CORRUPT") != nullptr;
    Platform::FileHandle* f = Platform::OpenFile(path, Platform::FileMode::Read);
    if (!f) return false;
    std::vector<u8> buf(kChunk);
    SetXfer(NetplayXferStatus::Sending, offset, rom.Size, rom.GameTitle());
    bool ok = offset <= rom.Size && Platform::FileSeek(f, (s64)offset, Platform::FileSeekOrigin::Start);
    for (u64 at = offset; ok && at < rom.Size;)
    {
        u32 n = (u32)std::min<u64>(kChunk, rom.Size - at);
        ok = Platform::FileRead(buf.data(), n, 1, f) == 1;
        if (ok && cut && at + n > cut)
        {
            cut = 0;
            shutdown(fd, SHUT_RDWR);
            ok = false;
        }
        if (!ok) break;
        bool fill = std::all_of(buf.begin() + 1, buf.begin() + n, [&](u8 b) { return b == buf[0]; });
        if (!fill && corrupt)
        {
            corrupt = false;
            buf[n / 2] ^= 0x55;
        }
        ok = fill ? SendVal(fd, kFill) && SendVal(fd, n) && SendVal(fd, buf[0])
                  : SendVal(fd, kData) && SendVal(fd, n) && SendAll(fd, buf.data(), n);
        at += n;
        XferBytes(at);
        if (tick) tick();
    }
    Platform::CloseFile(f);
    return ok && SendVal(fd, kEnd);
}

enum class Got { Lost, Damaged, Ok };

// Receives stream frames into `in`'s .part; at the end, its SHA-256 decides: Ok = renamed to
// <sha>.nds, Damaged = .part deleted (`in` starts over), Lost = the connection failed (the .part
// keeps what arrived).
Got RecvRom(int fd, NetplaySetup& s, Incoming& in, const std::function<void()>& tick)
{
    std::string part = CachePath(s, in.Rom, ".part");
    FILE* f = fopen(part.c_str(), in.Have ? "ab" : "wb");
    if (!f)
    {
        AddLog(s.Log, "Netplay: cannot write %s", part.c_str());
        return Got::Lost;
    }
    SetXfer(NetplayXferStatus::Receiving, in.Have, in.Rom.Size, in.Rom.GameTitle());
    std::vector<u8> buf(kChunk);
    bool ok = true;
    for (;;)
    {
        u8 kind = kEnd, b = 0;
        u32 n = 0;
        ok = RecvVal(fd, kind) && (kind == kEnd || (RecvVal(fd, n) && n <= kChunk && in.Have + n <= in.Rom.Size));
        if (!ok || kind == kEnd) break;
        if (kind == kFill) { ok = RecvVal(fd, b); memset(buf.data(), b, n); }
        else ok = kind == kData && RecvAll(fd, buf.data(), n);
        ok = ok && fwrite(buf.data(), 1, n, f) == n;
        if (!ok) break;
        in.Sha.Add(buf.data(), n);
        in.Have += n;
        XferBytes(in.Have);
        if (tick) tick();
    }
    fclose(f);
    if (!ok) return Got::Lost;
    u8 sha[32];
    in.Sha.Get(sha);
    if (in.Have != in.Rom.Size || memcmp(sha, in.Rom.Sha, 32))
    {
        remove(part.c_str());
        in.Have = 0;
        in.Sha = Sha256();
        return Got::Damaged;
    }
    std::string path = CachePath(s, in.Rom, ".nds");
    if (rename(part.c_str(), path.c_str()) != 0) return Got::Lost;
    AddLog(s.Log, "Netplay: received %s", in.Rom.Name().c_str());
    return Got::Ok;
}

bool HostHandshake(NetplaySetup& s)
{
    struct Guest { int Fd = -1; int Player = 0; u32 Ip = 0; u16 Port = 0; NetplayRom Rom; std::vector<u8> Save; double OneWayMs = 0; };
    std::vector<Guest> guests;
    auto closeAll = [&] { for (Guest& g : guests) if (g.Fd >= 0) { TrackFd(g.Fd, false); close(g.Fd); } };

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
        if (ms <= 0 || gCancel) break;
        if (poll(&pf, 1, std::min(ms, 250)) != 1) continue;
        sockaddr_in from {};
        socklen_t fromLen = sizeof(from);
        Guest g;
        g.Fd = accept(ls, (sockaddr*)&from, &fromLen);
        if (g.Fd < 0) continue;
        TrackFd(g.Fd, true);
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
            TrackFd(g.Fd, false);
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
#ifdef LITEV_NP_ADAPTIVE_DELAY
        const int maxDelay = s.Hosted ? 8 : NetplayInput::kMaxDelay;   // plain Netplay adapts from here
#else
        const int maxDelay = 8;
#endif
        s.Delay = std::clamp((int)std::ceil(worst / (1000.0 / 60)) + 1, 1, maxDelay);
        AddLog(s.Log, "Netplay: worst one way %.1f ms -> input delay %d frames", worst, s.Delay);
    }

    std::map<int, NetplayRom> roms;     // every player's
    if (s.HostPlays)
    {
        roms[0] = s.Rom;
        AddLog(s.Log, "Netplay: player 0 runs %s", s.Rom.Name().c_str());
    }
    for (Guest& g : guests)
    {
        roms[g.Player] = g.Rom;
        AddLog(s.Log, "Netplay: player %d runs %s", g.Player, g.Rom.Name().c_str());
    }
    s.Roms = roms;
    // every guest: delay, then every player (address 0 = the host itself) with its ROM and save
    for (Guest& g : guests)
    {
        ok = ok && SendVal(g.Fd, kSetupMagic) && SendVal(g.Fd, (u8)s.Delay)
                && SendVal(g.Fd, (u8)(guests.size() + (s.HostPlays ? 1 : 0)));
        if (s.HostPlays)
            ok = ok && SendVal(g.Fd, (u8)0) && SendVal(g.Fd, (u32)0) && SendVal(g.Fd, (u16)s.Port) && SendVal(g.Fd, s.Rom)
                    && SendBlob(g.Fd, s.Save);
        for (Guest& o : guests)
            ok = ok && SendVal(g.Fd, (u8)o.Player) && SendVal(g.Fd, o.Ip) && SendVal(g.Fd, o.Port) && SendVal(g.Fd, o.Rom)
                    && SendBlob(g.Fd, &o == &g ? std::vector<u8>() : o.Save);
    }

    // Which ROM must travel where. The host runs every console: each guest's ROM is found here or
    // must come from that guest. Each guest reports which of the others' it lacks (a Hosted
    // replica: none) and how much of it an interrupted transfer left; the host sends it, after
    // receiving it first itself when it lacks it too.
    struct Xfer
    {
        NetplayRom Rom;
        int Player = 0;                     // whose game it is
        Guest* Owner = nullptr;             // the host lacks it: it comes from this guest
        std::string HostPath;               // where the host has it ("" = not yet)
        Incoming HostIn;                    // the host receiving it
        std::vector<std::pair<Guest*, u64>> To;     // guests that need it, and what their .part holds
    };
    std::map<std::string, Xfer> xfers;      // by SHA-256
    std::string missing;
    auto xferFor = [&](int player) -> Xfer& {
        const NetplayRom& rom = roms[player];
        Xfer& x = xfers[rom.Hex()];
        if (!x.Rom.Size)
        {
            x.Rom = x.HostIn.Rom = rom;
            x.Player = player;
            x.HostPath = player == 0 && s.HostPlays && !s.RomPath.empty() ? s.RomPath : Resolve(s, rom);
            for (Guest& o : guests) if (o.Rom == rom && !x.Owner) x.Owner = &o;
            if (x.HostPath.empty() && !s.CacheDir.empty()) StartIncoming(s, x.HostIn);
        }
        return x;
    };
    for (Guest& g : guests)
    {
        Xfer& x = xferFor(g.Player);
        if (!x.HostPath.empty()) s.RomPaths[g.Player] = x.HostPath;
        else if (s.CacheDir.empty() || !x.Owner) AddLog(missing, "Netplay: the host does not have player %d's game %s", g.Player, g.Rom.Name().c_str());
    }
    for (Guest& g : guests)
    {
        u8 count = 0;
        ok = ok && RecvVal(g.Fd, count);
        for (int i = 0; ok && i < count; i++)
        {
            u8 player = 0;
            u64 have = 0;
            ok = RecvVal(g.Fd, player) && RecvVal(g.Fd, have) && roms.count(player);
            if (!ok) break;
            Xfer& x = xferFor(player);
            if (have == ~0ull || (x.HostPath.empty() && (s.CacheDir.empty() || !x.Owner)))
                AddLog(missing, "Netplay: player %d does not have player %d's game %s", g.Player, player, x.Rom.Name().c_str());
            else x.To.emplace_back(&g, have);
        }
    }
    for (auto it = xfers.begin(); it != xfers.end();)
        it = it->second.HostPath.empty() || !it->second.To.empty() ? std::next(it) : xfers.erase(it);

    // consent, before any byte moves: every device asked what it would send or receive
    auto who = [&](const Xfer& x, bool host) {
        std::string t;
        if (host) t = s.HostPlays ? "player 0 (the host)" : "the host";
        for (auto& [g, have] : x.To) t += (t.empty() ? "player " : ", player ") + std::to_string(g->Player);
        return t;
    };
    std::map<Guest*, std::string> asks;
    std::string hostAsk;
    for (auto& [hex, x] : xfers)
    {
        std::string game = x.Rom.GameTitle() + " (" + Megabytes(x.Rom.Size) + ", player " + std::to_string(x.Player) + "'s game)";
        const std::string keep = " and keep it in the Netplay ROM cache?\n";
        if (x.HostPath.empty())
        {
            asks[x.Owner] += "Send your copy of " + x.Rom.GameTitle() + " (" + Megabytes(x.Rom.Size) + ") to " + who(x, true) + "?\n";
            hostAsk += "Download " + game + " from player " + std::to_string(x.Owner->Player)
                     + (x.To.empty() ? "" : ", pass it on to " + who(x, false)) + keep;
        }
        else hostAsk += "Send your copy of " + x.Rom.GameTitle() + " (" + Megabytes(x.Rom.Size) + ") to " + who(x, false) + "?\n";
        for (auto& [g, have] : x.To) asks[g] += "Download " + game + " from the host" + keep;
    }
    if (missing.empty())
    {
        for (auto& [g, q] : asks)
            ok = ok && SendVal(g->Fd, kAsk) && SendBlob(g->Fd, std::vector<u8>(q.begin(), q.end()));
        if (ok && !hostAsk.empty() && !(s.Consent && s.Consent(hostAsk)))
            AddLog(missing, "Netplay: the host said no to sending or receiving a game");
        for (auto& [g, q] : asks)
        {
            u8 yes = 0;
            ok = ok && RecvVal(g->Fd, yes);
            if (ok && !yes) AddLog(missing, "Netplay: player %d said no to sending or receiving a game", g->Player);
        }
    }

    // the transfers, one at a time; every guest not in one hears from the host every second
    // (its no-progress timeout is short now)
    if (ok && missing.empty() && !xfers.empty())
    {
        std::vector<std::chrono::steady_clock::time_point> lastTick(guests.size());
        for (Guest& g : guests) SetTimeout(g.Fd, kStallS);
        auto ticker = [&](Guest* busy) {
            return [&, busy] {
                auto now = std::chrono::steady_clock::now();
                for (size_t i = 0; i < guests.size(); i++)
                    if (&guests[i] != busy && now - lastTick[i] > std::chrono::seconds(1))
                    {
                        lastTick[i] = now;
                        SendVal(guests[i].Fd, kTick);
                    }
            };
        };
        ticker(nullptr)();
        for (auto& [hex, x] : xfers)
        {
            for (int attempt = 0; ok && x.HostPath.empty() && attempt < 2; attempt++)
            {
                AddLog(s.Log, "Netplay: receiving player %d's game from %llu of %llu bytes", x.Player,
                       (unsigned long long)x.HostIn.Have, (unsigned long long)x.Rom.Size);
                Got got = Got::Lost;
                ok = SendVal(x.Owner->Fd, kUpload) && SendVal(x.Owner->Fd, x.HostIn.Have)
                  && (got = RecvRom(x.Owner->Fd, s, x.HostIn, ticker(x.Owner))) != Got::Lost;
                if (got == Got::Ok) x.HostPath = CachePath(s, x.Rom, ".nds");
                if (got == Got::Damaged) AddLog(s.Log, "Netplay: player %d's game arrived damaged (SHA-256 mismatch)", x.Player);
            }
            if (ok && x.HostPath.empty())
            {
                AddLog(missing, "Netplay: player %d's game %s arrived damaged twice", x.Player, x.Rom.Name().c_str());
                break;
            }
            for (auto& [g, have] : x.To)
            {
                u8 got = 0;
                for (int attempt = 0; ok && got != 2 && attempt < 2; attempt++)
                {
                    AddLog(s.Log, "Netplay: sending player %d's game to player %d from %llu bytes", x.Player, g->Player, (unsigned long long)have);
                    ok = SendVal(g->Fd, kRecv) && SendVal(g->Fd, (u8)x.Player) && SendVal(g->Fd, have)
                      && SendRom(g->Fd, x.HostPath, x.Rom, have, ticker(g)) && RecvVal(g->Fd, got);
                    have = 0;   // damaged: again from the start
                }
                if (ok && got != 2)
                    AddLog(missing, "Netplay: player %d's game %s arrived damaged twice at player %d", x.Player, x.Rom.Name().c_str(), g->Player);
            }
            if (!ok || !missing.empty()) break;
        }
        if (ok && !s.CacheDir.empty()) TrimCache(s, roms);
        for (Guest& g : guests)
            if (!s.RomPaths.count(g.Player) && xfers.count(g.Rom.Hex())) s.RomPaths[g.Player] = xfers[g.Rom.Hex()].HostPath;
    }

    // everyone: the verdict ("" = go, else why not)
    std::vector<u8> verdict(missing.begin(), missing.end());
    for (Guest& g : guests) ok = ok && SendVal(g.Fd, kDone) && SendBlob(g.Fd, verdict);
    // wait for every guest to have it all (it closes first), so nobody's data is cut short
    for (Guest& g : guests) { u8 b; recv(g.Fd, &b, 1, 0); }
    closeAll();
    SetXfer(ok && missing.empty() ? NetplayXferStatus::Done : NetplayXferStatus::Failed);
    if (!ok) { AddLog(s.Log, gCancel ? "Netplay: cancelled" : "Netplay: session setup failed"); return false; }
    if (!missing.empty())
    {
        s.Log += missing;
        AddLog(s.Log, "Netplay: cancelled: a device lacks another player's game, or said no to sending or receiving it");
        return false;
    }
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
    while (fd < 0 && !gCancel && std::chrono::steady_clock::now() < deadline)
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
    TrackFd(fd, true);
    bool ok = SendVal(fd, kSetupMagic) && SendVal(fd, (u8)s.Player) && SendVal(fd, (u16)s.Port)
           && SendVal(fd, s.Rom) && SendBlob(fd, s.Save);
    for (int i = 0; ok && i < kPings; i++)
    {
        u8 b;
        ok = RecvVal(fd, b) && SendVal(fd, b);
    }
    u32 magic = 0;
    u8 delay = 0, count = 0;
    ok = ok && RecvVal(fd, magic) && magic == kSetupMagic && RecvVal(fd, delay)
            && RecvVal(fd, count) && delay >= 1 && delay <= 8;
    std::string hostIp = s.Host.substr(0, s.Host.rfind(':'));
    bool self = false;
    std::map<std::string, Incoming> incoming;   // the games this device lacks, by SHA-256
    std::vector<std::pair<int, u64>> needs;     // (player, what its .part holds; ~0 = cannot receive)
    for (int i = 0; ok && i < count; i++)
    {
        u8 player;
        u32 ip;
        u16 port;
        NetplayRom rom;
        std::vector<u8> save;
        ok = RecvVal(fd, player) && RecvVal(fd, ip) && RecvVal(fd, port) && RecvVal(fd, rom) && RecvBlob(fd, save)
          && player < NetplayInput::kMaxPlayers;
        if (!ok) break;
        AddLog(s.Log, "Netplay: player %d runs %s", player, rom.Name().c_str());
        s.Roms[player] = rom;
        if (player == s.Player) { self = true; continue; }
        s.Peers.emplace_back(player, (ip ? IpString(ip) : hostIp) + ":" + std::to_string(port));
        s.Saves[player] = std::move(save);
        if (s.Join) continue;   // a Hosted replica runs only its own console
        std::string path = Resolve(s, rom);
        if (!path.empty()) { s.RomPaths[player] = path; continue; }
        AddLog(s.Log, "Netplay: this device does not have player %d's game %s", player, rom.Name().c_str());
        if (incoming.count(rom.Hex())) continue;
        u64 have = ~0ull;
        if (!s.CacheDir.empty())
        {
            Incoming& in = incoming[rom.Hex()];
            in.Rom = rom;
            StartIncoming(s, in);
            have = in.Have;
        }
        needs.emplace_back(player, have);
    }
    ok = ok && SendVal(fd, (u8)needs.size());
    for (auto& [player, have] : needs) ok = ok && SendVal(fd, (u8)player) && SendVal(fd, have);

    // the host's requests, until its verdict ("" = go)
    std::vector<u8> verdict;
    bool done = false;
    while (ok && !done)
    {
        u8 cmd = 0;
        ok = RecvVal(fd, cmd);
        if (!ok) break;
        if (cmd != kDone && cmd != kAsk) SetTimeout(fd, kStallS);   // transfers: no progress for long = failed
        switch (cmd)
        {
        case kAsk:
        {
            std::vector<u8> q;
            ok = RecvBlob(fd, q);
            bool yes = ok && s.Consent && s.Consent(std::string(q.begin(), q.end()));
            if (ok && !yes) AddLog(s.Log, "Netplay: said no to: %s", std::string(q.begin(), q.end()).c_str());
            ok = ok && SendVal(fd, (u8)yes);
            break;
        }
        case kUpload:
        {
            u64 offset = 0;
            ok = RecvVal(fd, offset);
            if (ok) AddLog(s.Log, "Netplay: sending this device's game to the host from %llu bytes", (unsigned long long)offset);
            ok = ok && SendRom(fd, s.RomPath, s.Rom, offset, nullptr);
            break;
        }
        case kRecv:
        {
            u8 player = 0;
            u64 offset = 0;
            ok = RecvVal(fd, player) && RecvVal(fd, offset) && s.Roms.count(player) && incoming.count(s.Roms[player].Hex());
            if (!ok) break;
            Incoming& in = incoming[s.Roms[player].Hex()];
            if (offset != in.Have) { in.Have = 0; in.Sha = Sha256(); ok = offset == 0; }   // the host restarts a damaged one
            AddLog(s.Log, "Netplay: receiving player %d's game from %llu bytes", player, (unsigned long long)offset);
            Got got = ok ? RecvRom(fd, s, in, nullptr) : Got::Lost;
            if (got == Got::Damaged) AddLog(s.Log, "Netplay: player %d's game arrived damaged (SHA-256 mismatch)", player);
            ok = got != Got::Lost && SendVal(fd, (u8)(got == Got::Ok ? 2 : 1));
            break;
        }
        case kTick:
            break;
        case kDone:
            ok = RecvBlob(fd, verdict);
            done = true;
            break;
        default:
            ok = false;
        }
    }
    TrackFd(fd, false);
    close(fd);
    SetXfer(ok && verdict.empty() ? NetplayXferStatus::Done : NetplayXferStatus::Failed);
    if (!ok || !self)
    {
        AddLog(s.Log, gCancel ? "Netplay: cancelled" : "Netplay: session setup with the host failed");
        return false;
    }
    if (!verdict.empty())
    {
        s.Log.append(verdict.begin(), verdict.end());
        AddLog(s.Log, "Netplay: cancelled: a device lacks another player's game, or said no to sending or receiving it");
        return false;
    }
    if (!incoming.empty()) TrimCache(s, s.Roms);
    for (auto& [player, rom] : s.Roms)
        if (player != s.Player && !s.Join && !s.RomPaths.count(player)) s.RomPaths[player] = CachePath(s, rom, ".nds");
    s.Delay = delay;
    AddLog(s.Log, "Netplay: %d players, input delay %d frames", (int)count, s.Delay);
    return true;
}
}

bool NetplayHandshake(NetplaySetup& s)
{
    s.Peers.clear();
    s.Saves.clear();
    s.RomPaths.clear();
    s.Roms.clear();
    gCancel = false;
    SetXfer(NetplayXferStatus::Idle);
    return s.Player == 0 && !s.Join ? HostHandshake(s) : GuestHandshake(s);
}

NetplayXferStatus NetplayGetXferStatus()
{
    std::lock_guard<std::mutex> lk(gXferLock);
    return gXfer;
}

bool NetplayAskUser(const std::string& question)
{
    std::unique_lock<std::mutex> lk(gXferLock);
    gXfer = NetplayXferStatus {};
    gXfer.State = NetplayXferStatus::Asking;
    gXfer.Question = question;
    gAnswer = -1;
    // the others wait about this long for the answer (setup socket timeout)
    gXferCv.wait_for(lk, std::chrono::seconds(90), [] { return gAnswer >= 0 || gCancel; });
    bool yes = gAnswer == 1 && !gCancel;
    gXfer = NetplayXferStatus {};
    return yes;
}

void NetplayAnswer(bool yes)
{
    std::lock_guard<std::mutex> lk(gXferLock);
    gAnswer = yes ? 1 : 0;
    gXferCv.notify_all();
}

void NetplayCancelSetup()
{
    std::lock_guard<std::mutex> lk(gXferLock);
    gCancel = true;
    for (int fd : gSetupFds) shutdown(fd, SHUT_RDWR);
    gXferCv.notify_all();
}

}
