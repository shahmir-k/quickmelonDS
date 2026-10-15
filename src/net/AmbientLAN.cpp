// LITEV_AMBIENT_LAN: lobby-free local wireless (see AmbientLAN.h)
#ifdef LITEV_AMBIENT_LAN

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <chrono>
#include <algorithm>
#include <math.h>

#ifdef __WIN32__
#include <winsock2.h>
#include <ws2tcpip.h>
typedef int socklen_t;
#define closesocket_ closesocket
#else
#include <unistd.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <sys/select.h>
#include <sys/socket.h>
#define closesocket_ close
#endif

#include "AmbientLAN.h"
#include "Platform.h"
#include "MPLinkFlags.h"
#ifdef __ANDROID__
#include <sys/system_properties.h>
#endif

namespace melonDS
{
namespace
{
constexpr u32 kPresMagic = 0x4E534453;      // "SDSN"
constexpr u8 kPresVersion = 1;
enum : u8 { P_Live = 1, P_Full = 2, P_Req = 3, P_Bye = 4, P_Ping = 5, P_Pong = 6 };   // (ping / pong: + u64 sender time, us)
constexpr u32 kNifi = 0x4946494E;           // MPPacketHeader magic (LAN.cpp)
enum { Chan_Ctl = 0, Chan_MP = 1 };
constexpr u32 kLiveMs = 2000, kFullMinMs = 500, kExpireMs = 6000, kIdleMs = 10000, kOwnBeaconStaleMs = 3000;

u64 NowUs() { return (u64)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
u32 NowMs() { return (u32)(NowUs() / 1000); }

// Tests: a slower network on the receiving side of each end (set on both ends: the round trip). A received MP frame or
// presence ping / pong is held RTT/2 +- JITTER/2 ms, LOSS % are dropped (env LITEV_AMBIENT_RTT_MS, _JITTER_MS, _LOSS).
// LITEV_AMBIENT_RTT_AT=S:MS[:R]: the RTT goes to MS from S s after start, over R s; LITEV_AMBIENT_SPIKE=EVERY:DUR:MS:
// MS for DUR ms every EVERY ms. ENet's handshake, acks and control channel are not held. Android (device tests):
// debug.litev.ambdelay=RTT[:JITTER[:LOSS]], read when the first LAN session or presence packet needs it.
struct NetSim
{
    double Rtt = 0, Jit = 0, Loss = 0, AtS = -1, AtRtt = 0, RampS = 0, SpEvery = 0, SpDur = 0, SpRtt = 0;
    u64 T0 = NowUs();
    bool On = false;
    NetSim()
    {
        auto f = [](const char* k, double d) { const char* v = getenv(k); return v ? atof(v) : d; };
        Rtt = f("LITEV_AMBIENT_RTT_MS", 0); Jit = f("LITEV_AMBIENT_JITTER_MS", 0); Loss = f("LITEV_AMBIENT_LOSS", 0);
        if (const char* v = getenv("LITEV_AMBIENT_RTT_AT")) sscanf(v, "%lf:%lf:%lf", &AtS, &AtRtt, &RampS);
        if (const char* v = getenv("LITEV_AMBIENT_SPIKE")) sscanf(v, "%lf:%lf:%lf", &SpEvery, &SpDur, &SpRtt);
#ifdef __ANDROID__
        char b[PROP_VALUE_MAX] = {};
        if (__system_property_get("debug.litev.ambdelay", b) > 0) sscanf(b, "%lf:%lf:%lf", &Rtt, &Jit, &Loss);
#endif
        On = Rtt > 0 || Jit > 0 || Loss > 0 || AtS >= 0 || SpEvery > 0;
    }
    double RttMs(u64 now) const
    {
        const double t = (now - T0) / 1e6;
        double rtt = Rtt;
        if (AtS >= 0 && t >= AtS) rtt += (AtRtt - Rtt) * (RampS > 0 ? std::min(1.0, (t - AtS) / RampS) : 1.0);
        if (SpEvery > 0 && fmod(t * 1000, SpEvery) < SpDur) rtt = std::max(rtt, SpRtt);
        return rtt;
    }
    // how long a packet received now is held (us), ~0 = lost
    u64 Hold(u64 now) const
    {
        thread_local u64 x = 0x9E3779B97F4A7C15ull ^ now;
        auto rnd = [] { x ^= x << 13; x ^= x >> 7; x ^= x << 17; return (x >> 11) * (1.0 / 9007199254740992.0); };
        if (Loss > 0 && rnd() * 100 < Loss) return ~0ull;
        const double ms = RttMs(now) / 2 + (rnd() - 0.5) * Jit;
        return ms <= 0 ? 0 : (u64)(ms * 1000);
    }
};
const NetSim& Sim() { static const NetSim s; return s; }
u16 G16(const u8* p) { return p[0] | p[1] << 8; }
u32 G32(const u8* p) { return p[0] | p[1] << 8 | p[2] << 16 | (u32)p[3] << 24; }
void P16(std::vector<u8>& v, u16 x) { v.push_back(x); v.push_back(x >> 8); }
void P32(std::vector<u8>& v, u32 x) { for (int i = 0; i < 4; i++) v.push_back(x >> (8 * i)); }
void P64(std::vector<u8>& v, u64 x) { for (int i = 0; i < 8; i++) v.push_back(x >> (8 * i)); }
u32 Fnv(const u8* p, size_t n, u32 h = 2166136261u) { for (size_t i = 0; i < n; i++) h = (h ^ p[i]) * 16777619u; return h; }
bool MacEq(const u8* a, const u8* b) { return !memcmp(a, b, 6); }

// data = DS TX header (12) + 802.11 frame. The Nintendo vendor IE of a beacon (OUI 00:09:BF, type 0): ie, il.
bool NintendoIE(const u8* d, int len, const u8** ie, u32* il)
{
    if (len < 12 + 36) return false;
    const u8* body = d + 12 + 24;
    const int bl = len - 12 - 24;
    for (int o = 12; o + 2 <= bl; )
    {
        const u8 id = body[o], l = body[o + 1];
        if (o + 2 + l > bl) break;
        if (id == 0xDD && l >= 8 && body[o + 2] == 0x00 && body[o + 3] == 0x09 && body[o + 4] == 0xBF && body[o + 5] == 0x00)
        { *ie = body + o + 2; *il = l; return true; }
        o += 2 + l;
    }
    return false;
}
// the WMGameInfo's GGID (IE + 8: game info; + 4 in it)
bool BeaconGGID(const u8* d, int len, u32& ggid)
{
    const u8* ie; u32 il;
    if (!NintendoIE(d, len, &ie, &il) || il < 8 + 8) return false;
    ggid = G32(ie + 8 + 4);
    return true;
}
bool IsBeacon(const u8* d, int len) { return len >= 12 + 36 && d[12] == 0x80; }
// a beacon's hash without the fields that change every beacon (TX header status, sequence, TSF, DTIM count, the
// Nintendo IE's active zone / V-count)
u32 BeaconHash(const u8* d, int len)
{
    std::vector<u8> m(d, d + len);
    memset(&m[0], 0, 8);
    m[12 + 22] = m[12 + 23] = 0;
    memset(&m[12 + 24], 0, 8);
    u8* body = &m[12 + 24];
    const int bl = len - 12 - 24;
    for (int o = 12; o + 2 <= bl; )
    {
        const u8 id = body[o], l = body[o + 1];
        if (o + 2 + l > bl) break;
        if (id == 5 && l >= 1) body[o + 2] = 0;
        if (id == 0xDD && l >= 8 && body[o + 2] == 0x00 && body[o + 3] == 0x09 && body[o + 4] == 0xBF) memset(&body[o + 6], 0, 4);
        o += 2 + l;
    }
    return Fnv(m.data(), m.size());
}
// LITEV_AMBIENT_TRACE=1 (env; tests): every frame sent / received / synthesized, logged
const bool kTrace = getenv("LITEV_AMBIENT_TRACE") != nullptr;
const bool kWaitTrace = getenv("LITEV_AMBIENT_WAIT_TRACE") != nullptr;
void Trace(const char* what, int slot, u32 type, const u8* d, int len, u64 timestamp)
{
    if (len < 12 + 24) return;
    const u8* f = d + 12;
    Platform::Log(Platform::LogLevel::Info, "AMBTRACE %llu %s slot %d type %x fc %02x%02x ch %u a1 %02x:%02x:%02x:%02x:%02x:%02x a2 %02x:%02x:%02x:%02x:%02x:%02x len %d ts %llu\n",
                  (unsigned long long)(NowUs() / 1000), what, slot, type, f[1], f[0], d[9], f[4], f[5], f[6], f[7], f[8], f[9], f[10], f[11], f[12], f[13], f[14], f[15], len, (unsigned long long)timestamp);
}
bool SameGame(const char* a, const char* b) { return a[0] && !memcmp(a, b, 3); }

int OpenUdp(u16 port, bool reuse)
{
    int s = (int)socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (s < 0) return -1;
    int one = 1;
    setsockopt(s, SOL_SOCKET, SO_BROADCAST, (const char*)&one, sizeof(one));
    if (reuse)
    {
        setsockopt(s, SOL_SOCKET, SO_REUSEADDR, (const char*)&one, sizeof(one));
#ifdef SO_REUSEPORT
        setsockopt(s, SOL_SOCKET, SO_REUSEPORT, (const char*)&one, sizeof(one));
#endif
    }
    sockaddr_in a = {};
    a.sin_family = AF_INET; a.sin_addr.s_addr = htonl(INADDR_ANY); a.sin_port = htons(port);
    if (bind(s, (sockaddr*)&a, sizeof(a)) < 0) { closesocket_(s); return -1; }
    return s;
}
}

void AmbientLAN::MacFromID(u64 id, u8* mac)
{
    mac[0] = 0x02;      // locally administered, unicast
    for (int i = 1; i < 6; i++) mac[i] = (u8)(id >> (8 * (i - 1)));
}

AmbientLAN::AmbientLAN(const Config& cfg) : Cfg(cfg), Vis(cfg.Vis)
{
    MacFromID(Cfg.DeviceID, MyMAC);
    if (Cfg.Name.size() > 32) Cfg.Name.resize(32);
    Snap = std::make_shared<const std::vector<Peer>>();
    View = Snap;

    if (enet_initialize() != 0) { Platform::Log(Platform::LogLevel::Error, "Ambient: enet init failed\n"); return; }
    for (int i = 0; i < 64 && !Host; i++)
    {
        ENetAddress a; a.host = ENET_HOST_ANY; a.port = (u16)(Cfg.LinkPort + i);
        Host = enet_host_create(&a, 16, 2, 0, 0);
        if (Host) LinkPort = a.port;
    }
    BcastSock = OpenUdp(Cfg.PresencePort, true);
    UniSock = OpenUdp(Cfg.UnicastPort, false);
    if (!Host || UniSock < 0)
    {
        Platform::Log(Platform::LogLevel::Error, "Ambient: no sockets (link %d, unicast %d)\n", Host != nullptr, UniSock >= 0);
        if (Host) { enet_host_destroy(Host); Host = nullptr; }
        return;
    }
    sockaddr_in a = {}; socklen_t al = sizeof(a);
    getsockname(UniSock, (sockaddr*)&a, &al);
    UniPort = ntohs(a.sin_port);
    Platform::Log(Platform::LogLevel::Info, "Ambient: device %016llx mac %02x:%02x:%02x:%02x:%02x:%02x presence %u/%u link %u\n",
                  (unsigned long long)Cfg.DeviceID, MyMAC[0], MyMAC[1], MyMAC[2], MyMAC[3], MyMAC[4], MyMAC[5],
                  Cfg.PresencePort, UniPort, LinkPort);
    Presence = std::thread([this] { PresenceLoop(); });
}

AmbientLAN::~AmbientLAN()
{
    Quit = true;
    if (Presence.joinable()) Presence.join();
    if (UniSock >= 0)
    {
        std::vector<std::pair<u32, u16>> to;
        { std::lock_guard<std::mutex> g(Mu); for (auto& p : Peers) to.push_back({p.IP, p.UniPort}); for (auto& e : Extra) to.push_back(e); }
        SendPresence(BuildPresence(P_Bye), true, to);
    }
    if (Host)
    {
        for (int s = 0; s < 16; s++) if (Links[s].P) enet_peer_disconnect(Links[s].P, 0);
        enet_host_flush(Host);
        while (!RX.empty()) { enet_packet_destroy(RX.front().Packet); RX.pop_front(); }
        for (auto& d : Delayed) enet_packet_destroy(d.second.Packet);
        enet_host_destroy(Host);
        enet_deinitialize();
    }
    if (BcastSock >= 0) closesocket_(BcastSock);
    if (UniSock >= 0) closesocket_(UniSock);
}

void AmbientLAN::AddPeerAddress(u32 ip, u16 port)
{
    std::lock_guard<std::mutex> g(Mu);
    for (auto& e : Extra) if (e.first == ip && e.second == port) return;
    Extra.push_back({ip, port});
}

void AmbientLAN::SetVisibility(int v)
{
    Vis = v;
    std::lock_guard<std::mutex> g(Mu);
    OwnDirty = true;
}

// ---- presence thread ----

std::vector<u8> AmbientLAN::BuildPresence(u8 kind)
{
    std::vector<u8> v;
    P32(v, kPresMagic); v.push_back(kPresVersion); v.push_back(kind); P16(v, LinkPort);
    P64(v, Cfg.DeviceID); P32(v, Cfg.BuildID);
    std::vector<u8> beacon;
    u8 mac[6];
    { std::lock_guard<std::mutex> g(Mu); beacon = OwnBeacon; memcpy(mac, MyMAC, 6); }
    const u8 flags = WifiOn ? 1 : 0;
    // summary hash: everything a FULL carries
    u32 h = Fnv((const u8*)Cfg.Name.data(), Cfg.Name.size());
    h = Fnv((const u8*)Cfg.GameCode, 4, h); h = Fnv(mac, 6, h); h = Fnv(&flags, 1, h);
    if (!beacon.empty()) { const u32 bh = BeaconHash(beacon.data(), (int)beacon.size()); h = Fnv((const u8*)&bh, 4, h); }
    P32(v, h);
    if (kind == P_Full)
    {
        for (int i = 0; i < 4; i++) v.push_back(Cfg.GameCode[i]);
        v.insert(v.end(), mac, mac + 6);
        v.push_back(flags);
        v.push_back((u8)Cfg.Name.size());
        v.insert(v.end(), Cfg.Name.begin(), Cfg.Name.end());
        P16(v, (u16)beacon.size());
        v.insert(v.end(), beacon.begin(), beacon.end());
    }
    return v;
}

std::vector<u8> AmbientLAN::BuildPing(u8 kind, u64 t)
{
    std::vector<u8> v;
    P32(v, kPresMagic); v.push_back(kPresVersion); v.push_back(kind); P16(v, LinkPort);
    P64(v, Cfg.DeviceID); P32(v, Cfg.BuildID); P32(v, 0); P64(v, t);
    return v;
}

u32 AmbientLAN::Peer::RttUs() const
{
    if (!RttN) return 0;
    u32 v[16];
    std::copy(RttS, RttS + RttN, v);
    std::nth_element(v, v + (RttN - 1) / 2, v + RttN);
    return v[(RttN - 1) / 2];       // (the lower median: half the window may spike)
}

u32 AmbientLAN::Peer::JitterUs() const
{
    const u32 m = RttUs();
    u64 d = 0;
    for (int i = 0; i < RttN; i++) d += RttS[i] > m ? RttS[i] - m : m - RttS[i];
    return RttN ? (u32)(d / RttN) : 0;
}

void AmbientLAN::SendPresence(const std::vector<u8>& pkt, bool bcast, const std::vector<std::pair<u32, u16>>& to)
{
    sockaddr_in a = {};
    a.sin_family = AF_INET;
    if (bcast)
    {
        a.sin_addr.s_addr = htonl(INADDR_BROADCAST); a.sin_port = htons(Cfg.PresencePort);
        if (sendto(UniSock, (const char*)pkt.data(), pkt.size(), 0, (sockaddr*)&a, sizeof(a)) > 0) StPresTx++;
    }
    for (auto& [ip, port] : to)
    {
        a.sin_addr.s_addr = ip; a.sin_port = htons(port);
        if (sendto(UniSock, (const char*)pkt.data(), pkt.size(), 0, (sockaddr*)&a, sizeof(a)) > 0) StPresTx++;
    }
}

u8 AmbientLAN::HandlePresence(const u8* p, int n, u32 ip, u16 port, u32 now, bool bcast)
{
    if (n < 24 || G32(p) != kPresMagic || p[4] != kPresVersion) return 0;
    const u8 kind = p[5];
    const u64 id = (u64)G32(p + 8) | (u64)G32(p + 12) << 32;
    if (id == Cfg.DeviceID) return 0;
    StPresRx++;
    auto it = std::find_if(Peers.begin(), Peers.end(), [&](const Peer& x) { return x.ID == id; });
    if (kind == P_Bye) { if (it != Peers.end()) Peers.erase(it); return 0; }
    if (kind == P_Req && it == Peers.end()) return 0;     // (a request alone does not make a device nearby)
    // round trips (the verdict, AmbientLAN.h), answered here at once
    if (kind == P_Ping || kind == P_Pong)
    {
        if (it == Peers.end() || n < 32) return 0;
        u64 t; memcpy(&t, p + 24, 8);
        if (kind == P_Ping) { PingEcho = t; return P_Pong; }
        const u64 nowUs = NowUs();
        if (t > nowUs || nowUs - t > 10000000) return 0;      // (not one of ours)
        it->RttS[it->RttPos] = (u32)std::min<u64>(nowUs - t, 1000000);
        it->RttPos = (it->RttPos + 1) % 16;
        if (it->RttN < 16) it->RttN++;
        it->Recv++;
        return 0;
    }
    const bool fresh = it == Peers.end();
    if (fresh) { Peers.emplace_back(); it = Peers.end() - 1; it->ID = id; }
    Peer& e = *it;
    e.IP = ip; e.UniPort = port; e.LinkPort = G16(p + 6); e.BuildID = G32(p + 16); e.SeenMs = now;
    if (bcast) e.BcastMs = now;
    const u32 hash = G32(p + 20);
    if (kind == P_Full && n >= 24 + 12)
    {
        const u8* q = p + 24;
        memcpy(e.Game, q, 4); memcpy(e.MAC, q + 4, 6); e.Flags = q[10];
        const u32 nl = std::min<u32>(q[11], 32);
        if (24 + 12 + nl + 2 > (u32)n) return 0;
        e.Name.assign((const char*)q + 12, nl);
        const u8* b = q + 12 + nl;
        const u32 bl = G16(b);
        if (24 + 12 + nl + 2 + bl > (u32)n || bl > 2048) return 0;
        e.Beacon.assign(b + 2, b + 2 + bl);
        e.BeaconUs = NowUs();
        e.HasGGID = !e.Beacon.empty() && BeaconGGID(e.Beacon.data(), (int)e.Beacon.size(), e.GGID);
        e.Hash = hash; e.Full = true;
    }
    else if (kind == P_Req)
    {
        if (now - e.ReqMs >= 1000 || !e.ReqMs) { e.ReqMs = now; return P_Full; }
    }
    else if (kind == P_Live && (!e.Full || e.Hash != hash) && Announcing)     // (not announced: asking would be ignored)
    {
        // stale details: ask that device (unicast), at most once a second
        if (now - e.ReqMs >= 1000 || !e.ReqMs) { e.ReqMs = now; return P_Req; }
    }
    // a device new here gets our details at once (else it waits up to kLiveMs for our liveness: a game run in
    // fast-forward can finish its scan before that)
    if (fresh && Announcing) { e.ReqMs = now; return P_Full; }
    return 0;
}

void AmbientLAN::PresenceLoop()
{
    u32 lastLive = 0, lastUni = 0, lastFull = 0, lastPub = 0;
    u32 lastHash = 0;
    bool wasVisible = false;
    u8 buf[4096];
    struct Held { u64 At; std::vector<u8> Data; u32 IP; u16 Port; };
    std::vector<Held> held;         // (tests, NetSim: pings / pongs on their way)
    bool changed = false;
    auto handle = [&](const u8* d, int n, u32 ip, u16 port, bool bcast)
    {
        u8 reply;
        { std::lock_guard<std::mutex> g(Mu); reply = HandlePresence(d, n, ip, port, NowMs(), bcast); }
        if (reply == P_Pong) SendPresence(BuildPing(P_Pong, PingEcho), false, {{ip, port}});
        else if (reply) SendPresence(BuildPresence(reply), false, {{ip, port}});
        changed = true;
    };
    while (!Quit)
    {
        fd_set fds; FD_ZERO(&fds);
        int mx = UniSock;
        FD_SET(UniSock, &fds);
        if (BcastSock >= 0) { FD_SET(BcastSock, &fds); mx = std::max(mx, BcastSock); }
        timeval tv = {0, 100000};
        changed = false;
        if (!held.empty())
        {
            const u64 n = NowUs();
            for (size_t i = 0; i < held.size(); )
                if (held[i].At <= n) { const Held h = held[i]; held.erase(held.begin() + i); handle(h.Data.data(), (int)h.Data.size(), h.IP, h.Port, false); }
                else i++;
            u64 due = ~0ull;
            for (const auto& h : held) due = std::min(due, h.At);
            if (due != ~0ull) tv.tv_usec = (long)std::min<u64>(100000, due > n ? due - n : 0);
        }
        if (select(mx + 1, &fds, nullptr, nullptr, &tv) > 0)
        {
            for (int s : {UniSock, BcastSock})
            {
                if (s < 0 || !FD_ISSET(s, &fds)) continue;
                for (;;)
                {
                    sockaddr_in from = {}; socklen_t fl = sizeof(from);
#ifdef MSG_DONTWAIT
                    const int n = (int)recvfrom(s, (char*)buf, sizeof(buf), MSG_DONTWAIT, (sockaddr*)&from, &fl);
#else
                    const int n = (int)recvfrom(s, (char*)buf, sizeof(buf), 0, (sockaddr*)&from, &fl);
#endif
                    if (n <= 0) break;
                    if (Sim().On && n >= 32 && (buf[5] == P_Ping || buf[5] == P_Pong))
                    {
                        const u64 hold = Sim().Hold(NowUs());
                        if (hold != ~0ull) held.push_back({NowUs() + hold, std::vector<u8>(buf, buf + n), from.sin_addr.s_addr, ntohs(from.sin_port)});
                    }
                    else handle(buf, n, from.sin_addr.s_addr, ntohs(from.sin_port), s == BcastSock);
#ifndef MSG_DONTWAIT
                    break;
#endif
                }
            }
        }
        const u32 now = NowMs();
        // unicast goes to the devices broadcast does not reach (not heard by broadcast lately: a router that drops it)
        // and to NSD finds not known yet; all of them for a goodbye
        std::vector<std::pair<u32, u16>> to, all;
        bool dirty;
        {
            std::lock_guard<std::mutex> g(Mu);
            // the game stopped beaconing (left its wireless menu, became a child): drop the beacon from the summary
            if (!OwnBeacon.empty() && now - OwnBeaconMs.load() > kOwnBeaconStaleMs) { OwnBeacon.clear(); OwnDirty = true; }
            const size_t before = Peers.size();
            Peers.erase(std::remove_if(Peers.begin(), Peers.end(), [&](const Peer& p) { return now - p.SeenMs > kExpireMs; }), Peers.end());
            changed |= Peers.size() != before;
            for (auto& p : Peers)
            {
                all.push_back({p.IP, p.UniPort});
                if (!p.BcastMs || now - p.BcastMs > kExpireMs) to.push_back({p.IP, p.UniPort});
            }
            for (auto& e : Extra)
                if (std::none_of(all.begin(), all.end(), [&](auto& t) { return t == e; })) { to.push_back(e); all.push_back(e); }
            if (changed || now - lastPub >= 1000)
            {
                Snap = std::make_shared<const std::vector<Peer>>(Peers);
                lastPub = now;
            }
            dirty = OwnDirty;
        }
        const int vis = Vis;
        const bool visible = Foreground && (vis == Vis_Everyone || (vis == Vis_WirelessOnly && WifiOn));
        Announcing = visible;
        if (!visible)
        {
            if (wasVisible) SendPresence(BuildPresence(P_Bye), true, all);
            wasVisible = false;
            continue;
        }
        if (!wasVisible) { dirty = true; wasVisible = true; }
        // round trips (the verdict, AmbientLAN.h): devices running this game or hosting (a beacon: Pokémon Black joins
        // White) get a ping a second (5 at 100 ms when one appears), linked ones 4 a second; the slowest linked one's
        // median is the session's
        {
            std::vector<std::pair<u32, u16>> ping;
            u32 linkRtt = 0, linkRttMax = 0;
            {
                std::lock_guard<std::mutex> g(Mu);
                for (auto& p : Peers)
                {
                    const bool linked = std::find(LinkedIDs.begin(), LinkedIDs.end(), p.ID) != LinkedIDs.end();
                    if (!linked && (!p.Full || !(SameGame(p.Game, Cfg.GameCode) || p.HasGGID))) continue;
                    if (linked)
                    {
                        linkRtt = std::max(linkRtt, p.RttUs());
                        for (int i = 0; i < p.RttN; i++) linkRttMax = std::max(linkRttMax, p.RttS[i]);
                    }
                    if (p.PingMs && now - p.PingMs < (p.PingN < 5 ? 100u : linked ? 250u : 1000u)) continue;
                    p.PingMs = now;
                    if (p.PingN < 255) p.PingN++;
                    if (++p.Sent >= 16) { p.LossPct = (u8)((p.Sent - std::min(p.Recv, p.Sent)) * 100 / p.Sent); p.Sent = p.Recv = 0; }
                    ping.push_back({p.IP, p.UniPort});
                }
            }
            LinkRttUs = linkRtt;
            LinkRttMaxUs = linkRttMax;
            if (!ping.empty()) SendPresence(BuildPing(P_Ping, NowUs()), false, ping);
        }
        if (dirty && now - lastFull >= kFullMinMs)
        {
            std::vector<u8> full = BuildPresence(P_Full);
            const u32 h = G32(&full[20]);
            { std::lock_guard<std::mutex> g(Mu); OwnDirty = false; }
            if (h != lastHash || !lastFull)
            {
                SendPresence(full, true, to);
                lastHash = h; lastFull = lastLive = lastUni = now;
            }
        }
        else
        {
            // liveness: broadcast every 2 s, unicast every 4 s (two per expiry window)
            const bool b = now - lastLive >= kLiveMs, u = !to.empty() && now - lastUni >= 2 * kLiveMs;
            if (b || u) SendPresence(BuildPresence(P_Live), b, u ? to : std::vector<std::pair<u32, u16>>());
            if (b) lastLive = now;
            if (u) lastUni = now;
        }
    }
}

std::vector<std::string> AmbientLAN::NearbyRows()
{
    std::shared_ptr<const std::vector<Peer>> s;
    std::vector<u64> linked;
    { std::lock_guard<std::mutex> g(Mu); s = Snap; linked = LinkedIDs; }
    std::vector<std::string> rows;
    for (const auto& p : *s)
    {
        char row[256], gg[16] = "-";
        if (p.HasGGID) snprintf(gg, sizeof(gg), "%08x", p.GGID);
        const u32 ip = p.IP;
        snprintf(row, sizeof(row), "%016llx\t%s\t%.4s\t%u.%u.%u.%u:%u\t%s\t%08x\t%d\t%u\t%u\t%u", (unsigned long long)p.ID, p.Name.c_str(),
                 p.Game[0] ? p.Game : "----", ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, ip >> 24, p.UniPort, gg, p.BuildID,
                 std::find(linked.begin(), linked.end(), p.ID) != linked.end() ? 1 : 0, p.RttUs(), p.JitterUs(), (unsigned)p.LossPct);
        rows.emplace_back(row);
    }
    return rows;
}

AmbientLAN::Stats AmbientLAN::GetStats()
{
    Stats s;
    s.Frames = StFrames; s.PresenceTx = StPresTx; s.PresenceRx = StPresRx; s.LinkTx = StLinkTx; s.LinkRx = StLinkRx;
    s.BeaconsInjected = StInjected; s.LinksOpened = StOpened; s.BlockedUs = StBlockedUs; s.Exchanges = StExchanges;
    s.HostWaitUs = StHostWaitUs; s.HostWaitCalls = StHostWaitCalls;
    s.HostWaitMisses = StHostWaitMisses; s.HostWaitTimeouts = StHostWaitTimeouts;
    s.ClientWaitUs = StClientWaitUs; s.ClientWaitCalls = StClientWaitCalls; s.ClientWaitMisses = StClientWaitMisses;
    for (int t = 0; t < 4; t++) s.Stale[t] = StStale[t];
    s.LinkRttUs = LinkRttUs;
    s.LinkTxBytes = StLinkTxBytes; s.LinkRxBytes = StLinkRxBytes;
    s.EmuUs = EmuBaseUs; s.PredFpsX100 = StPredFps; s.Verdict = StVerdict;
    s.LastFrameBlockedUs = StLastBlocked; s.LastFrameExchanges = StLastExch; s.Links = StLinks; s.Nearby = StNearby; s.MaxRttMs = StMaxRtt;
    s.SlowFpsX100 = StSlowFps; s.SlowWaitUsPerFrame = StSlowWait;
    s.SlowExchangesPerFrameX100 = StSlowExch; s.SlowdownPending = StSlowPending;
    return s;
}

// ---- emulator thread ----

void AmbientLAN::RefreshView(bool force)
{
    const u32 now = NowMs();
    if (!force && now - ViewMs < 100) return;
    ViewMs = now;
    std::shared_ptr<const std::vector<Peer>> s;
    { std::lock_guard<std::mutex> g(Mu); s = Snap; }
    if (s == View) return;
    View = s;
    StNearby = (u32)View->size();
    RebuildCands();
}

// the nearby beacons the local game gets (AmbientLAN.h: GGIDs wanted)
void AmbientLAN::RebuildCands()
{
    std::set<u32> want = OwnGGIDs;
    for (const auto& p : *View) if (p.HasGGID && SameGame(p.Game, Cfg.GameCode)) want.insert(p.GGID);
    std::vector<Cand> c;
    const u64 now = EmuNow(), wall = NowUs();
    for (size_t i = 0; i < View->size(); i++)
    {
        const Peer& p = (*View)[i];
        if (!p.HasGGID || (!want.empty() && !want.count(p.GGID))) continue;
        u32 tu = G16(&p.Beacon[12 + 24 + 8]);
        const u32 iv = std::clamp<u32>(tu * 1024u, 20000u, 1000000u);
        u64 due = now + (p.ID % iv);    // (a new one: spread the phases)
        // its TSF: as received, aged by the real time since, then the chip's time only (monotonic, one beacon interval
        // per interval, also when that device's beacon changes)
        u64 tsf; memcpy(&tsf, &p.Beacon[12 + 24], 8);
        u64 off = tsf + (wall - p.BeaconUs) - now;
        for (const auto& o : Cands) if (o.ID == p.ID) { due = o.DueUs; off = o.TsfOff; }
        c.push_back({i, p.ID, due, iv, off});
    }
    Cands.swap(c);
    NextInjectUs = ~0ull;
    for (const auto& o : Cands) NextInjectUs = std::min(NextInjectUs, o.DueUs);
}

int AmbientLAN::Inject(u8* data, u64* timestamp)
{
    if (NextInjectUs == ~0ull || Vis == Vis_Hidden) return 0;
    const u64 now = EmuNow();
    if (now < NextInjectUs) return 0;
    // a due beacon on the channel the chip listens on (one on another channel stays due: the chip would drop it)
    Cand* best = nullptr;
    for (auto& c : Cands)
        if (c.DueUs <= now && (*View)[c.Idx].Beacon[9] == WifiChannel && (!best || c.DueUs < best->DueUs)) best = &c;
    int len = 0;
    if (best)
    {
        const Peer& p = (*View)[best->Idx];
        len = (int)p.Beacon.size();
        memcpy(data, p.Beacon.data(), len);
        const u64 tsf = now + best->TsfOff;
        memcpy(&data[12 + 24], &tsf, 8);
        best->DueUs = std::max(best->DueUs + best->IntervalUs, now);
        StInjected++;
        if (kTrace) Trace("beacon", -1, 0, data, len, 0);
    }
    NextInjectUs = ~0ull;
    for (const auto& c : Cands) NextInjectUs = std::min(NextInjectUs, c.DueUs);
    if (timestamp) *timestamp = 0;
    return len;
}

// the chip's clock (beacons keep their pace per emulated second in fast-forward / slow motion); when it goes back (a
// savestate, a reset) the schedule goes back with it, the TSF does not
u64 AmbientLAN::EmuNow()
{
    const u64 now = WifiUs;
    if (now < EmuUs)
    {
        const u64 d = EmuUs - now;
        for (auto& c : Cands) { c.DueUs = c.DueUs > d ? c.DueUs - d : 0; c.TsfOff += d; }
        if (NextInjectUs != ~0ull) NextInjectUs = NextInjectUs > d ? NextInjectUs - d : 0;
    }
    return EmuUs = now;
}

const AmbientLAN::Peer* AmbientLAN::PeerByMAC(const u8* mac)
{
    RefreshView(true);
    for (const auto& p : *View)
        if (p.Full && (MacEq(p.MAC, mac) || (p.Beacon.size() >= 12 + 24 && MacEq(&p.Beacon[12 + 10], mac)))) return &p;
    return nullptr;
}

int AmbientLAN::SlotByMAC(const u8* mac)
{
    for (int s = 0; s < 16; s++) if (Links[s].P && MacEq(Links[s].MAC, mac)) return s;
    return -1;
}

u32 AmbientLAN::UpMask() const
{
    u32 m = 0;
    for (int s = 0; s < 16; s++) if (Links[s].Up) m |= 1u << s;
    return m;
}

void AmbientLAN::LinksChanged(bool opened)
{
    std::vector<u64> ids;
    NumLinks = 0;
    for (int s = 0; s < 16; s++) if (Links[s].P) { NumLinks++; ids.push_back(Links[s].ID); }
    // a player joining: the whole group to normal speed (the join runs at 1x on both ends; a parent tells its other
    // children); one leaving: only the leaver (its last link closed), the others keep the group's speed
    if (opened || !NumLinks)
    {
        const bool fast = StSpeed || PendCode > 0;
        PendCode = -1;
        StSpeed = StShown = 0;
        StSpeedWho = 0;
        SpeedReq = -1;
        if (fast && ParentSlot < 0 && UpMask()) SpeedDecide(0, 0, 0);
    }
    StLinks = NumLinks;
    std::lock_guard<std::mutex> g(Mu);
    LinkedIDs.swap(ids);
}

int AmbientLAN::OpenLink(const Peer& p)
{
    if (Vis == Vis_Hidden || !p.LinkPort) return -1;
    int s = 0;
    while (s < 16 && Links[s].P) s++;
    if (s == 16) return -1;
    ENetAddress a; a.host = p.IP; a.port = p.LinkPort;
    ENetPeer* e = enet_host_connect(Host, &a, 2, (u32)Cfg.DeviceID);
    if (!e) return -1;
    Link& l = Links[s];
    l = Link();
    l.ID = p.ID; l.Generation = ++NextLinkGeneration; l.P = e; memcpy(l.MAC, p.MAC, 6); l.ActiveMs = NowMs();
    e->data = &l;
    enet_peer_timeout(e, 0, 5000, 5000);
    StOpened++;
    LinksChanged(true);
    Platform::Log(Platform::LogLevel::Info, "Ambient: link %d -> %016llx (%s), verdict %d (emulation %.1f ms)\n", s, (unsigned long long)p.ID, p.Name.c_str(),
                  (int)PeerVerdict(p.ID), EmuBaseUs / 1000.0);
    return s;
}

void AmbientLAN::CloseLink(int s, const char* why)
{
    Link& l = Links[s];
    if (!l.P) return;
    DropQueuedForSlot(s);
    l.P->data = nullptr;
    enet_peer_disconnect(l.P, 0);
    Platform::Log(Platform::LogLevel::Info, "Ambient: link %d to %016llx closed (%s)\n", s, (unsigned long long)l.ID, why);
    if (ParentSlot == s) ParentSlot = -1;
    l = Link();
    LinksChanged(false);
}

void AmbientLAN::NoteBeacon(const u8* data, int len)
{
    OwnBeaconMs = NowMs();
    const u32 h = BeaconHash(data, len);
    if (h == OwnBeaconHash) return;
    OwnBeaconHash = h;
    u32 gg;
    if (BeaconGGID(data, len, gg) && OwnGGIDs.insert(gg).second) RebuildCands();
    std::lock_guard<std::mutex> g(Mu);
    OwnBeacon.assign(data, data + len);
    OwnDirty = true;
}

void AmbientLAN::SendTo(u32 slots, u32 type, const u8* data, int len, u64 timestamp)
{
    if (!Host || !slots) return;
    ENetPacket* pkt = nullptr;
    bool sent = false;
    for (int s = 0; s < 16; s++)
    {
        if (!(slots & (1u << s)) || !Links[s].P) continue;
        Link& l = Links[s];
        if (!l.Up)
        {
            if (type == 0 && l.Pending.size() < 64)
            {
                std::vector<u8> f(sizeof(MPPacketHeader) + len);
                MPPacketHeader h = {kNifi, 0, type, (u32)len, timestamp};
                memcpy(f.data(), &h, sizeof(h)); memcpy(f.data() + sizeof(h), data, len);
                l.Pending.push_back(std::move(f));
            }
            continue;
        }
        if (!pkt)
        {
            pkt = enet_packet_create(nullptr, sizeof(MPPacketHeader) + len, ENET_PACKET_FLAG_UNSEQUENCED);
            MPPacketHeader h = {kNifi, 0, type, (u32)len, timestamp};
            memcpy(pkt->data, &h, sizeof(h));
            memcpy(pkt->data + sizeof(h), data, len);
        }
        enet_peer_send(l.P, Chan_MP, pkt);
        StLinkTx++;
        StLinkTxBytes += len;
        if (kTrace) Trace("tx", s, type, data, len, timestamp);
        sent = true;
    }
    if (pkt && !pkt->referenceCount) enet_packet_destroy(pkt);
    if (sent) enet_host_flush(Host);
}

// a received MP frame: from a link of this session? (a child takes frames from its parent only)
bool AmbientLAN::Admit(ENetEvent& ev)
{
    Link* l = (Link*)ev.peer->data;
    if (!l || ev.packet->dataLength < sizeof(MPPacketHeader) + 12 + 24) return false;
    const MPPacketHeader* h = (const MPPacketHeader*)ev.packet->data;
    if (h->Magic != kNifi || h->Length + sizeof(MPPacketHeader) > ev.packet->dataLength) return false;
    const int s = (int)(l - Links);
    if (ParentSlot >= 0 && s != ParentSlot) return false;
    const u8* f = ev.packet->data + sizeof(MPPacketHeader) + 12;
    if (f[0] != 0x80) l->ActiveMs = NowMs();
    if (!(f[10] & 1)) memcpy(l->MAC, f + 10, 6);     // (the sender's own MAC)
    if (f[0] == 0xC0 && !l->CloseAtMs) l->CloseAtMs = NowMs() + 500;   // deauth: the session is over
    StLinkRx++;
    StLinkRxBytes += h->Length;
    if (kTrace) Trace("rx", s, h->Type, ev.packet->data + sizeof(MPPacketHeader), (int)h->Length, h->Timestamp);
    return true;
}

void AmbientLAN::OnEvent(ENetEvent& ev)
{
    switch (ev.type)
    {
    case ENET_EVENT_TYPE_CONNECT:
    {
        Link* l = (Link*)ev.peer->data;
        if (l)          // ours, up now: the frames sent meanwhile
        {
            l->Up = true;
            for (auto& f : l->Pending)
            {
                ENetPacket* p = enet_packet_create(f.data(), f.size(), ENET_PACKET_FLAG_UNSEQUENCED);
                enet_peer_send(ev.peer, Chan_MP, p);
                StLinkTx++;
            }
            l->Pending.clear();
            enet_host_flush(Host);
            break;
        }
        // incoming: only from a nearby device (its id in the connect data, from the address it announced from)
        RefreshView(true);
        const Peer* p = nullptr;
        for (const auto& x : *View) if ((u32)x.ID == ev.data && x.IP == ev.peer->address.host) p = &x;
        int s = 0;
        while (s < 16 && Links[s].P) s++;
        if (!p || s == 16 || Vis == Vis_Hidden)
        {
            Platform::Log(Platform::LogLevel::Info, "Ambient: refused a link from %08x (%s)\n", ev.peer->address.host, p ? "full" : "not nearby");
            enet_peer_disconnect(ev.peer, 0);
            break;
        }
        Link& n = Links[s];
        n = Link();
        n.ID = p->ID; n.Generation = ++NextLinkGeneration; n.P = ev.peer; n.Up = true; memcpy(n.MAC, p->MAC, 6); n.ActiveMs = NowMs();
        ev.peer->data = &n;
        enet_peer_timeout(ev.peer, 0, 5000, 5000);
        LinksChanged(true);
        Platform::Log(Platform::LogLevel::Info, "Ambient: link %d <- %016llx (%s), verdict %d (emulation %.1f ms)\n", s, (unsigned long long)p->ID, p->Name.c_str(),
                      (int)PeerVerdict(p->ID), EmuBaseUs / 1000.0);
        break;
    }
    case ENET_EVENT_TYPE_DISCONNECT:
    {
        Link* l = (Link*)ev.peer->data;
        if (!l) break;
        const int s = (int)(l - Links);
        Platform::Log(Platform::LogLevel::Info, "Ambient: link %d to %016llx lost: rtt %u ms, last receive %u ms ago, last send %u ms ago\n", s,
                      (unsigned long long)l->ID, ev.peer->roundTripTime, enet_time_get() - ev.peer->lastReceiveTime, enet_time_get() - ev.peer->lastSendTime);
        if (ParentSlot == s) ParentSlot = -1;
        DropQueuedForSlot(s);
        *l = Link();
        LinksChanged(false);
        break;
    }
    case ENET_EVENT_TYPE_RECEIVE:
    {
        const Link* l = (const Link*)ev.peer->data;
        if (l && ev.channelID == Chan_Ctl) SpeedControl((int)(l - Links), ev.packet->data, ev.packet->dataLength);
        enet_packet_destroy(ev.packet);
        break;
    }
    default:
        break;
    }
}

bool AmbientLAN::QueueCurrent(const QueuedPacket& q) const
{
    if (!q.Packet || q.Slot < 0 || q.Slot >= 16) return false;
    const Link& l = Links[q.Slot];
    // A slot can be reused even by the same peer before a deferred RX frame is consumed.
    return l.P && l.ID == q.ID && l.Generation == q.Generation;
}

void AmbientLAN::DropQueuedForSlot(int slot)
{
    for (auto it = Delayed.begin(); it != Delayed.end(); )
    {
        if (it->second.Slot == slot) { enet_packet_destroy(it->second.Packet); it = Delayed.erase(it); }
        else ++it;
    }
    for (auto it = RX.begin(); it != RX.end(); )
    {
        if (it->Slot == slot)
        {
            enet_packet_destroy(it->Packet);
            it = RX.erase(it);
        }
        else ++it;
    }
}

// as LAN::ProcessLAN: 0 = per frame, 1 = a regular frame wanted, 2 = waiting for an MP frame, 3 = 2 without blocking
// waitMs: 2's wait (-1: RecvTimeout)
void AmbientLAN::Pump(int type, int waitMs)
{
    if (!Host) return;
    u32 now = NowMs();
    while (!RX.empty())
    {
        const QueuedPacket q = RX.front();
        ENetPacket* p = q.Packet;
        if (!QueueCurrent(q)) { RX.pop_front(); enet_packet_destroy(p); continue; }
        MPPacketHeader* h = (MPPacketHeader*)p->data;
        // stale after a frame (LAN.cpp), but a management frame (auth / assoc / deauth) waits for the chip up to 1 s: a
        // parent's chip only listens around its beacons, and a child here does not send right after a real beacon
        // (it heard a synthesized one), so its auth request arrives while the parent dozes
        const u8 fc = p->data[sizeof(MPPacketHeader) + 12];
        const u32 keep = (h->Type == 0 && (fc & 0x0C) == 0 && fc != 0x80) ? 1000 : 16;
        if (h->Magic > now || now - h->Magic > keep)
        {
            if (kWaitTrace)
                Platform::Log(Platform::LogLevel::Info,
                              "AMBWAIT stale frame type=%04x age_ms=%u keep_ms=%u slot=%d\n",
                              h->Type, h->Magic > now ? 0u : now - h->Magic, keep, q.Slot);
            StStale[h->Type & 3]++;
            RX.pop_front(); enet_packet_destroy(p); continue;
        }
        if (type >= 2) return;
        if (type == 1)
        {
            if (h->Type == 0) return;
            RX.pop_front(); enet_packet_destroy(p);
        }
        break;
    }
    int timeout = type == 2 ? (waitMs < 0 ? RecvTimeout : waitMs) : 0;
    u32 last = NowMs();
    ENetEvent ev;
    for (;;)
    {
        if (!Delayed.empty() && ReleaseDelayed()) return;
        int t = timeout;
        if (!Delayed.empty())       // (tests, NetSim: wake for the next held frame)
        {
            u64 due = ~0ull;
            for (const auto& d : Delayed) due = std::min(due, d.first);
            const u64 n = NowUs();
            t = std::min<int>(t, due <= n ? 0 : (int)((due - n + 999) / 1000));
        }
        const int r = enet_host_service(Host, &ev, t);
        if (r > 0)
        {
            if (ev.type == ENET_EVENT_TYPE_RECEIVE && ev.channelID == Chan_MP)
            {
                if (Admit(ev))
                {
                    const Link* l = (const Link*)ev.peer->data;
                    const QueuedPacket q = {ev.packet, l->ID, l->Generation, (int)(l - Links)};
                    if (!Sim().On)
                    {
                        ((MPPacketHeader*)ev.packet->data)->Magic = NowMs();
                        RX.push_back(q);
                        return;
                    }
                    const u64 hold = Sim().Hold(NowUs());
                    if (hold == ~0ull) enet_packet_destroy(ev.packet);
                    else Delayed.push_back({NowUs() + hold, q});
                }
                else enet_packet_destroy(ev.packet);
            }
            else OnEvent(ev);
        }
        else if (r < 0 || t == timeout) { if (!Delayed.empty()) ReleaseDelayed(); return; }    // nothing within the wait
        if (type == 2)
        {
            const u32 n = NowMs();
            timeout -= (int)(n - last);
            last = n;
            if (timeout <= 0) { if (!Delayed.empty()) ReleaseDelayed(); return; }
        }
    }
}

// tests (NetSim): the held frames that are due reach RX
bool AmbientLAN::ReleaseDelayed()
{
    bool any = false;
    const u64 now = NowUs();
    for (size_t i = 0; i < Delayed.size(); )
    {
        if (Delayed[i].first > now) { i++; continue; }
        const QueuedPacket q = Delayed[i].second;
        Delayed.erase(Delayed.begin() + i);
        if (!QueueCurrent(q)) { enet_packet_destroy(q.Packet); continue; }
        ((MPPacketHeader*)q.Packet->data)->Magic = NowMs();
        RX.push_back(q);
        any = true;
    }
    return any;
}

// ---- group fast-forward (AmbientLAN.h) ----
// control channel: {1, code, seq u32, at u64, who u64} parent -> children: the group speed from the chip time `at`;
// {2, code} child -> parent: its key asks for `code`

int AmbientLAN::ToggleSpeed(int onCode)
{
    const int c = StShown ? 0 : onCode;
    StShown = c;
    SpeedReq = c;
    return c;
}

std::string AmbientLAN::SpeedWho()
{
    const u64 id = StSpeedWho;
    if (id == Cfg.DeviceID) return Cfg.Name;
    std::lock_guard<std::mutex> g(Mu);
    for (const auto& p : *Snap) if (p.ID == id) return p.Name;
    return "";
}

void AmbientLAN::SendCtl(u32 slots, const u8* m, int n)
{
    if (!Host || !slots) return;
    ENetPacket* pkt = enet_packet_create(m, n, ENET_PACKET_FLAG_RELIABLE);
    for (int s = 0; s < 16; s++) if ((slots >> s & 1) && Links[s].Up) enet_peer_send(Links[s].P, Chan_Ctl, pkt);
    if (!pkt->referenceCount) enet_packet_destroy(pkt);
    enet_host_flush(Host);
}

// parent: the group speed from chip time atUs, for every member
void AmbientLAN::SpeedDecide(int code, u64 who, u64 atUs)
{
    SpeedSeq++;
    PendCode = code; PendWho = who; PendAtUs = atUs; PendWallUs = NowUs();
    StShown = code;
    u8 m[22] = {1, (u8)code};
    memcpy(m + 2, &SpeedSeq, 4); memcpy(m + 6, &atUs, 8); memcpy(m + 14, &who, 8);
    SendCtl(UpMask(), m, sizeof(m));
}

void AmbientLAN::SpeedControl(int slot, const u8* p, size_t n)
{
    if (n >= 2 && p[0] == 2 && ParentSlot < 0) SpeedDecide(p[1] & 63, Links[slot].ID, WifiUs + kSpeedLagUs);
    else if (n >= 22 && p[0] == 1 && slot == ParentSlot)
    {
        u32 seq;
        memcpy(&seq, p + 2, 4);
        if (seq == SpeedSeq) return;
        SpeedSeq = seq;
        PendCode = p[1] & 63; memcpy(&PendAtUs, p + 6, 8); memcpy(&PendWho, p + 14, 8); PendWallUs = NowUs();
        StShown = PendCode;
    }
}

void AmbientLAN::SpeedTick()
{
    const int req = SpeedReq.exchange(-1);
    if (req >= 0 && NumLinks)
    {
        if (ParentSlot >= 0 && Links[ParentSlot].Up) { const u8 m[2] = {2, (u8)req}; SendCtl(1u << ParentSlot, m, 2); }
        else if (ParentSlot >= 0) StShown = StSpeed.load();     // (not up yet: the press is dropped)
        else SpeedDecide(req, Cfg.DeviceID, WifiUs + kSpeedLagUs);
    }
    // at the agreed chip time (one far ahead: a clock not following the parent's yet, at once; a chip that stopped
    // polling: after 2 s)
    const u64 now = WifiUs;
    if (PendCode < 0 || (now < PendAtUs && PendAtUs - now <= 4 * kSpeedLagUs && NowUs() - PendWallUs < 2000000)) return;
    Platform::Log(Platform::LogLevel::Info, "Ambient: group speed %d by %016llx (chip %llu us, due %llu)\n", PendCode,
                  (unsigned long long)PendWho, (unsigned long long)now, (unsigned long long)PendAtUs);
    StSpeed = PendCode;
    StSpeedWho = PendWho;
    PendCode = -1;
}

int AmbientLAN::Pop(u8* data, u64* timestamp)
{
    while (!RX.empty() && !QueueCurrent(RX.front()))
    {
        enet_packet_destroy(RX.front().Packet);
        RX.pop_front();
    }
    if (RX.empty()) return 0;
    const QueuedPacket q = RX.front();
    ENetPacket* p = q.Packet;
    RX.pop_front();
    const MPPacketHeader* h = (const MPPacketHeader*)p->data;
    u32 len = std::min<u32>(h->Length, 2048);
    memcpy(data, p->data + sizeof(MPPacketHeader), len);
    if (h->Type == 1)
    {
        LastHostID = q.ID;
        LastHostSet = true;
        FrameExchanges++;
    }
    if (timestamp) *timestamp = h->Timestamp;
    enet_packet_destroy(p);
    return (int)len;
}

void AmbientLAN::Process()
{
    const u32 frameBlocked = FrameBlockedUs, frameExchanges = FrameExchanges;
    StFrames++;
    StLastBlocked = FrameBlockedUs; StLastExch = FrameExchanges;
    StBlockedUs += FrameBlockedUs; StExchanges += FrameExchanges;
    FrameBlockedUs = FrameExchanges = 0;
    if (!Host) return;
    Pump(0);
    RefreshView(false);
    const u32 now = NowMs();
    u32 rtt = 0;
    for (int s = 0; s < 16; s++)
    {
        Link& l = Links[s];
        if (!l.P) continue;
        if (l.Up) rtt = std::max<u32>(rtt, l.P->roundTripTime);
        if (l.CloseAtMs && (s32)(now - l.CloseAtMs) >= 0) CloseLink(s, "deauth");
        else if (now - l.ActiveMs > kIdleMs) CloseLink(s, "idle");
    }
    StMaxRtt = rtt;
    SpeedTick();
    Judge(frameBlocked, frameExchanges);
    if (StSlowDismissRequested.exchange(false)) { Slowdown.Dismiss(); VerdictDismissed = true; }
    const int sp = StSpeed;     // (NetplaySpeed codes: 63 uncapped, else x4)
    const auto slow = Slowdown.Observe(NowUs(), UpMask() != 0, frameBlocked, frameExchanges, sp == 63 ? 0 : sp ? sp * 1500 : 6000);
    StSlowFps = slow.FpsX100;
    StSlowWait = slow.WaitUsPerFrame;
    StSlowExch = slow.ExchangesPerFrameX100;
    // the Netplay offer: the verdict; the fps-and-wait detector only without round trips (presence blocked): it cannot
    // tell a slow network from a partner busy emulating (2 x 8 ms frames over localhost waited 8 ms a frame at 57 fps)
    StSlowPending = (StVerdict == Verdict_LanBad && !VerdictDismissed) || (slow.Pending && !LinkRttUs);
}

// the wait for a peer's MP frame: RecvTimeout, or the link's round trip + margin when that is longer (a slow network
// keeps its session, slowly, until it moves to Netplay), at most 250 ms; the longest of the last 16 round trips (4 s
// linked), not their median: a 100 ms Wi-Fi spike keeps the waits long for the next 4 s
int AmbientLAN::LinkWaitMs() const
{
    return std::max(RecvTimeout, std::min(250, (int)(LinkRttMaxUs * 3 / 2000) + 10));
}

// one exchange a frame does not fit 58 fps because of the round trip: this device alone fits it (a device too slow
// for 58 fps on its own is no case for Netplay: Netplay would not make it faster) and the round trip is 2 ms or more
bool AmbientLAN::LinkRule(u32 emuUs, u32 rttUs)
{
    return emuUs < 1000000 / 58 && rttUs >= 2000 && emuUs + rttUs > 1000000 / 58;
}

AmbientLAN::Verdict AmbientLAN::PeerVerdict(u64 id)
{
    std::lock_guard<std::mutex> g(Mu);
    for (const auto& p : Peers)
        if (p.ID == id) return p.RttN < 3 ? Verdict_Unknown : LinkRule(EmuBaseUs, p.RttUs()) ? Verdict_LanBad : Verdict_LanOk;
    return Verdict_Unknown;
}

// once a frame (Process): the emulation time and the session verdict (AmbientLAN.h)
void AmbientLAN::Judge(u32 frameBlocked, u32 frameExchanges)
{
    const u32 run = RunUs;
    const u64 now = NowUs();
    if (run)
    {
        const u32 e = run > frameBlocked ? run - frameBlocked : 0;
        FrameEmuUs = (FrameEmuUs * 15 + std::min<u32>(e, 2 * FrameEmuUs)) / 16;     // (a loading frame: at most 2x)
        RunUs = 0;
        // E for the round-trip rule: the lowest half-second average of the last 2 s (a loading phase emulates more)
        EmSum += e; EmN++;
        if (!EmT0) EmT0 = now;
        if (now - EmT0 >= 500000)
        {
            EmRing[EmPos++ % 4] = (u32)(EmSum / EmN);
            u32 m = ~0u;
            for (u32 x : EmRing) if (x) m = std::min(m, x);
            EmuBaseUs = m;
            EmT0 = now; EmSum = 0; EmN = 0;
        }
    }
    if (!UpMask())
    {
        if (StVerdict) Platform::Log(Platform::LogLevel::Info, "Ambient: session verdict cleared (no links)\n");
        StVerdict = Verdict_Unknown; StPredFps = 0;
        FirstExchUs = VT0 = 0; VFrames = VExch = VRunN = VBad = VWeak = VOk = 0; VExchFrames = 0; VRunUs = 0; VerdictDismissed = false;
        return;
    }
    const u64 stalls = StHostWaitTimeouts + StClientWaitMisses;
    if (frameExchanges && !FirstExchUs) FirstExchUs = now;
    if (!VT0) { VT0 = now; VStallAt = stalls; VEmuAtLink = EmuBaseUs; }
    VFrames++; VExch += frameExchanges;
    if (FirstExchUs) VExchFrames++;
    if (run) { VRunUs += run; VRunN++; }
    if (now - VT0 < 500000) return;
    const u32 rtt = LinkRttUs;
    if (FirstExchUs && now - FirstExchUs >= 1000000 && rtt && VExch)
    {
        const double net = (double)VExch / VFrames * rtt;
        const double frame = VRunN ? (double)VRunUs / VRunN : EmuBaseUs + net;
        const double pred = std::min(60.0, 1e6 / frame);
        StPredFps = (u32)(pred * 100);
        // the link rule (2 windows): one exchange a frame does not fit 58 (E: the session's lowest; a loading phase
        // emulates more). With no wait timing out and frames missing 50 fps (a margin: 55-57 fps is no case for
        // Netplay), 4 windows: they make 50 without the round trips; or (only after 600 frames of exchanges) the round
        // trips take 40% of them and they make 40 without (a partner slow too). 4 windows and the 600 frames wait out
        // setup bursts: an MKDS lobby / race start runs 5-6 exchanges a frame at 14-48 fps at 0 ms for ~400 frames,
        // then 1 a frame (Pokémon B/W: 2 a frame from link to the end, Shrek 3: no burst)
        const bool linkRule = LinkRule(std::min<u32>(VEmuAtLink, EmuBaseUs), rtt);
        const bool calm = frame > 1e6 / 50 && stalls == VStallAt && net >= 2000;     // (2 ms: a round trip that matters)
        const bool strong = linkRule || (calm && frame - net <= 1e6 / 50);
        const bool weak = calm && net >= frame * 0.4 && frame - net <= 1e6 / 40 && VExchFrames >= 600;
        VBad = linkRule ? VBad + 1 : 0;
        VWeak = strong || weak ? VWeak + 1 : 0;
        VOk = strong || weak ? 0 : VOk + 1;
        int v = StVerdict;
        if (VBad >= 2 || VWeak >= 4) v = Verdict_LanBad;
        else if (VOk >= (v == Verdict_LanBad ? 4u : 2u)) v = Verdict_LanOk;
        if (v != StVerdict)
        {
            Platform::Log(Platform::LogLevel::Info, "Ambient: session verdict %s: %.1f fps (frame %.1f ms, of which %.2f exchanges/frame x %.1f ms round trips; emulation %.1f ms)\n",
                          v == Verdict_LanBad ? "LAN_BAD" : "LAN_OK", pred, frame / 1000.0, (double)VExch / VFrames, rtt / 1000.0, EmuBaseUs / 1000.0);
            StVerdict = v;
        }
    }
    VT0 = now; VFrames = VExch = VRunN = 0; VRunUs = 0; VStallAt = stalls;
}

void AmbientLAN::Begin(int inst)
{
    WifiOn = true;
    LastHostSet = false;
    std::lock_guard<std::mutex> g(Mu);
    OwnDirty = true;
}

void AmbientLAN::End(int inst)
{
    WifiOn = false;
    for (int s = 0; s < 16; s++) CloseLink(s, "Wi-Fi off");
    while (!RX.empty()) { enet_packet_destroy(RX.front().Packet); RX.pop_front(); }
    for (auto& d : Delayed) enet_packet_destroy(d.second.Packet);
    Delayed.clear();
    if (Host) enet_host_flush(Host);
    ParentSlot = -1;
    std::lock_guard<std::mutex> g(Mu);
    OwnDirty = true;
}

int AmbientLAN::SendPacket(int inst, u8* data, int len, u64 timestamp)
{
    if (!Host || len < 12 + 24) return len;
    const u8* f = data + 12;
    if (!(f[10] & 1) && !MacEq(f + 10, MyMAC))       // the console's real MAC (firmware) is the one announced
    {
        std::lock_guard<std::mutex> g(Mu);
        memcpy(MyMAC, f + 10, 6);
        OwnDirty = true;
    }
    if (IsBeacon(data, len))
    {
        NoteBeacon(data, len);
        ParentSlot = -1;                    // beaconing = a parent
        SendTo(UpMask(), 0, data, len, timestamp);
        return len;
    }
    const u8* a1 = f + 4;
    if (a1[0] & 1) { if (NumLinks) SendTo(0xFFFF, 0, data, len, timestamp); return len; }
    int s = SlotByMAC(a1);
    if (s < 0)
    {
        const Peer* p = PeerByMAC(a1);
        if (!p || (s = OpenLink(*p)) < 0) return len;
    }
    // a child's auth request (sequence 1): joining that device's session; any other session of ours is over
    if (f[0] == 0xB0 && len >= 12 + 24 + 4 && G16(f + 24 + 2) == 1)
    {
        ParentSlot = s;
        for (int o = 0; o < 16; o++) if (o != s) CloseLink(o, "joining another session");
    }
    Links[s].ActiveMs = NowMs();
    if (f[0] == 0xC0 && !Links[s].CloseAtMs) Links[s].CloseAtMs = NowMs() + 500;
    SendTo(1u << s, 0, data, len, timestamp);
    return len;
}

int AmbientLAN::RecvPacket(int inst, u8* data, u64* timestamp)
{
    if (NumLinks)
    {
        Pump(1);
        return Pop(data, timestamp);
    }
    return Inject(data, timestamp);
}

int AmbientLAN::SendCmd(int inst, u8* data, int len, u64 timestamp)
{
    FrameExchanges++;
    for (int s = 0; s < 16; s++) if (Links[s].Up) Links[s].ActiveMs = NowMs();
    SendTo(0xFFFF, 1, data, len, timestamp);
    return len;
}

int AmbientLAN::SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid)
{
    int s = ParentSlot;
    if (s < 0 && LastHostSet) for (int o = 0; o < 16; o++) if (Links[o].P && Links[o].ID == LastHostID) s = o;
    if (s >= 0) { Links[s].ActiveMs = NowMs(); SendTo(1u << s, 2 | (aid << 16), data, len, timestamp); }
    return len;
}

int AmbientLAN::SendAck(int inst, u8* data, int len, u64 timestamp)
{
    SendTo(0xFFFF, 3, data, len, timestamp);
    return len;
}

int AmbientLAN::RecvHostPacket(int inst, u8* data, u64* timestamp)
{
    if (LastHostSet)
    {
        bool up = false;
        for (int s = 0; s < 16; s++) if (Links[s].Up && Links[s].ID == LastHostID) up = true;
        if (!up) return -1;     // the host is gone
    }
    if (!Host || !NumLinks) return 0;
    const u64 t0 = NowUs();
#ifdef LITEV_LAN_RX_SPIN
    // as LAN (LITEV_LAN_RX_SPIN): poll the socket without sleeping for a while first (waking a sleeping core costs a
    // scheduler round trip); debug.litev.lanspin / LITEV_LAN_RX_SPIN_US us, default 2000, 0 = off
    static const int spinUs = [] {
        const char* v = nullptr;
#ifdef __ANDROID__
        static char b[PROP_VALUE_MAX];
        if (__system_property_get("debug.litev.lanspin", b) > 0) v = b;
#else
        v = getenv("LITEV_LAN_RX_SPIN_US");
#endif
        return v ? atoi(v) : 2000;
    }();
    if (spinUs > 0 && RX.empty())
    {
        const u64 end = t0 + spinUs;
        do
        {
            enet_uint32 cond = ENET_SOCKET_WAIT_RECEIVE;
            if (enet_socket_wait(Host->socket, &cond, 0) == 0 && (cond & ENET_SOCKET_WAIT_RECEIVE)) Pump(3);
        }
        while (RX.empty() && NowUs() < end);
    }
#endif
    const u64 spent = NowUs() - t0, wait = (u64)LinkWaitMs() * 1000;     // (the spin counts toward it)
    Pump(2, spent >= wait ? 0 : (int)((wait - spent + 999) / 1000));
    const int r = Pop(data, timestamp);
    const u64 waited = NowUs() - t0;
    FrameBlockedUs += (u32)waited;
    StClientWaitUs += waited;
    StClientWaitCalls++;
    if (r <= 0) StClientWaitMisses++;
    return r;
}

u16 AmbientLAN::RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask)
{
    const u32 up = UpMask();
    if (!Host || !up) return 0;
    const u64 t0 = NowUs();
    u16 ret = 0;
    u32 replied = 0;
    std::deque<QueuedPacket> later;      // regular frames meanwhile (a joining child's auth): for RecvPacket
    const u64 end = t0 + (u64)LinkWaitMs() * 1000;    // one wait for the call: old replies / regular frames meanwhile do not extend it
    for (;;)
    {
        const u64 now = NowUs();
        Pump(2, now >= end ? 0 : (int)((end - now + 999) / 1000));
        if (RX.empty()) break;
        const QueuedPacket q = RX.front();
        ENetPacket* p = q.Packet;
        RX.pop_front();
        if (!QueueCurrent(q)) { enet_packet_destroy(p); continue; }
        const MPPacketHeader* h = (const MPPacketHeader*)p->data;
        if (h->Type == 0) { later.push_back(q); continue; }
        if ((h->Type & 0xFFFF) == 2 && h->Timestamp >= timestamp - 32)
        {
            const u32 len = std::min<u32>(h->Length, 1024);
            const u32 aid = h->Type >> 16;
            if (len && aid >= 1 && aid < 16)
            {
                memcpy(&data[(aid - 1) * 1024], p->data + sizeof(MPPacketHeader), len);
                ret |= 1 << aid;
            }
            replied |= 1u << q.Slot;
            if ((replied & up) == up || (ret & aidmask) == aidmask) { enet_packet_destroy(p); break; }
        }
        else if (kWaitTrace && (h->Type & 0xFFFF) == 2)
            Platform::Log(Platform::LogLevel::Info,
                          "AMBWAIT old reply slot=%d reply_ts=%llu want_ts=%llu\n",
                          q.Slot, (unsigned long long)h->Timestamp, (unsigned long long)timestamp);
        enet_packet_destroy(p);
    }
    RX.insert(RX.begin(), later.begin(), later.end());
    const u64 waited = NowUs() - t0;
    FrameBlockedUs += (u32)waited;
    StHostWaitUs += waited;
    StHostWaitCalls++;
    if ((ret & aidmask) != aidmask) StHostWaitMisses++;
    if ((replied & up) != up && (ret & aidmask) != aidmask)
    {
        StHostWaitTimeouts++;
        if (kWaitTrace)
            Platform::Log(Platform::LogLevel::Info,
                          "AMBWAIT host timeout waited_us=%llu up=%04x replied=%04x aidmask=%04x ret=%04x ts=%llu\n",
                          (unsigned long long)waited, up, replied, aidmask, ret, (unsigned long long)timestamp);
    }
    return ret;
}

}

#endif
