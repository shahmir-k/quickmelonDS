// LITEV_AMBIENT_LAN: lobby-free local wireless (see AmbientLAN.h)
#ifdef LITEV_AMBIENT_LAN

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <chrono>
#include <thread>
#include <algorithm>

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
#if defined(LITEV_LAN_RX_SPIN) && defined(__ANDROID__)
#include <sys/system_properties.h>
#endif

namespace melonDS
{
namespace
{
constexpr u32 kPresMagic = 0x4E534453;      // "SDSN"
constexpr u8 kPresVersion = 1;
enum : u8 { P_Live = 1, P_Full = 2, P_Req = 3, P_Bye = 4 };
constexpr u32 kNifi = 0x4946494E;           // MPPacketHeader magic (LAN.cpp)
enum { Chan_Ctl = 0, Chan_MP = 1 };
constexpr u32 kHandoffCtl = 0x46444E48;     // "HNDF", Ambient-only reliable control
enum : u32 { HandoffFence = 1, HandoffReady = 2, HandoffAbort = 3,
             HandoffPrepared = 4, HandoffArmed = 5, HandoffGo = 6,
             HandoffResetDone = 7, HandoffOffer = 8, HandoffAccept = 9,
             HandoffDecline = 10, HandoffCancel = 11, HandoffRom = 12 };
constexpr size_t kHandoffRomBytes = 88;
constexpr u32 kLiveMs = 2000, kFullMinMs = 500, kExpireMs = 6000, kIdleMs = 10000, kOwnBeaconStaleMs = 3000;
// MP uses ENet's unsequenced channel. After a second, a missing datagram is
// past the game's 16 ms receive window; a cut also waits for a quiet drain.
constexpr u32 kMPGapExpireMs = 1000, kHandoffMPDrainMs = 1000;
// The game's MP clock is frozen through checkpoint transfer and both startup
// proofs. Keep the already-consented link alive long enough for that bounded
// transaction; ordinary gameplay still closes after 10 seconds of inactivity.
constexpr u32 kHandoffIdleMs = 120000;

u64 NowUs() { return (u64)std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::steady_clock::now().time_since_epoch()).count(); }
u32 NowMs() { return (u32)(NowUs() / 1000); }
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
const bool kFenceTrace = getenv("LITEV_AMBIENT_FENCE_TRACE") != nullptr;
const u32 kTestDropMPSeq = getenv("LITEV_AMBIENT_DROP_MP_SEQ") ?
    (u32)strtoul(getenv("LITEV_AMBIENT_DROP_MP_SEQ"), nullptr, 10) : 0;
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

    if (const char* rtt = getenv("LITEV_HANDOFF_SIM_RTT_MS")) SimHalfRttUs = (u64)(atof(rtt) * 500);
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
        while (!RX.empty()) { DestroyRX(RX.front()); RX.pop_front(); }
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
    if (it == Peers.end()) { Peers.emplace_back(); it = Peers.end() - 1; it->ID = id; }
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
    return 0;
}

void AmbientLAN::PresenceLoop()
{
    u32 lastLive = 0, lastUni = 0, lastFull = 0, lastPub = 0;
    u32 lastHash = 0;
    bool wasVisible = false;
    u8 buf[4096];
    while (!Quit)
    {
        fd_set fds; FD_ZERO(&fds);
        int mx = UniSock;
        FD_SET(UniSock, &fds);
        if (BcastSock >= 0) { FD_SET(BcastSock, &fds); mx = std::max(mx, BcastSock); }
        timeval tv = {0, 100000};
        bool changed = false;
        if (select(mx + 1, &fds, nullptr, nullptr, &tv) > 0)
        {
            const u32 now = NowMs();
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
                    u8 reply;
                    { std::lock_guard<std::mutex> g(Mu); reply = HandlePresence(buf, n, from.sin_addr.s_addr, ntohs(from.sin_port), now, s == BcastSock); }
                    if (reply) SendPresence(BuildPresence(reply), false, {{from.sin_addr.s_addr, ntohs(from.sin_port)}});
                    changed = true;
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
        snprintf(row, sizeof(row), "%016llx\t%s\t%.4s\t%u.%u.%u.%u:%u\t%s\t%08x\t%d", (unsigned long long)p.ID, p.Name.c_str(),
                 p.Game[0] ? p.Game : "----", ip & 0xFF, (ip >> 8) & 0xFF, (ip >> 16) & 0xFF, ip >> 24, p.UniPort, gg, p.BuildID,
                 std::find(linked.begin(), linked.end(), p.ID) != linked.end() ? 1 : 0);
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
    const u64 now = NowUs();
    for (size_t i = 0; i < View->size(); i++)
    {
        const Peer& p = (*View)[i];
        if (!p.HasGGID || (!want.empty() && !want.count(p.GGID))) continue;
        u32 tu = G16(&p.Beacon[12 + 24 + 8]);
        const u32 iv = std::clamp<u32>(tu * 1024u, 20000u, 1000000u);
        u64 due = now + (p.ID % iv);    // (a new one: spread the phases)
        for (const auto& o : Cands) if (o.ID == p.ID) due = o.DueUs;
        c.push_back({i, p.ID, due, iv});
    }
    Cands.swap(c);
    NextInjectUs = ~0ull;
    for (const auto& o : Cands) NextInjectUs = std::min(NextInjectUs, o.DueUs);
}

int AmbientLAN::Inject(u8* data, u64* timestamp)
{
    if (NextInjectUs == ~0ull || Vis == Vis_Hidden) return 0;
    const u64 now = NowUs();
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
        u64 tsf; memcpy(&tsf, &data[12 + 24], 8);
        tsf += now - p.BeaconUs;
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

void AmbientLAN::LinksChanged()
{
    std::vector<u64> ids;
    NumLinks = 0;
    for (int s = 0; s < 16; s++) if (Links[s].P) { NumLinks++; ids.push_back(Links[s].ID); }
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
    LinksChanged();
    Platform::Log(Platform::LogLevel::Info, "Ambient: link %d -> %016llx (%s)\n", s, (unsigned long long)p.ID, p.Name.c_str());
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
    LinksChanged();
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
    bool sent = false;
    for (int s = 0; s < 16; s++)
    {
        if (!(slots & (1u << s)) || !Links[s].P) continue;
        Link& l = Links[s];
        if (l.LocalFence) l.FenceAbort = true; // watermark cannot omit a post-fence MP send
        if (!l.Up)
        {
            if (type == 0 && l.Pending.size() < 64)
            {
                std::vector<u8> f(sizeof(MPPacketHeader) + len);
                MPPacketHeader h = {kNifi, ++l.TxSeq, type, (u32)len, timestamp};
                if (kFenceTrace && h.SenderID <= 8)
                    fprintf(stderr, "AMBFENCE pending seq=%u type=%u len=%d\n", h.SenderID, type, len);
                memcpy(f.data(), &h, sizeof(h)); memcpy(f.data() + sizeof(h), data, len);
                l.Pending.push_back(std::move(f));
            }
            continue;
        }
        // SenderID is unused by Ambient (manual LAN uses it for player id).
        // A separate packet per recipient gives each link a contiguous watermark.
        ENetPacket* pkt = enet_packet_create(nullptr, sizeof(MPPacketHeader) + len, ENET_PACKET_FLAG_UNSEQUENCED);
        if (!pkt) continue;
        MPPacketHeader h = {kNifi, ++l.TxSeq, type, (u32)len, timestamp};
        if (kFenceTrace && h.SenderID <= 8)
            fprintf(stderr, "AMBFENCE tx seq=%u type=%u len=%d\n", h.SenderID, type, len);
        memcpy(pkt->data, &h, sizeof(h));
        memcpy(pkt->data + sizeof(h), data, len);
        if (kTestDropMPSeq && h.SenderID == kTestDropMPSeq)
        {
            if (kFenceTrace) fprintf(stderr, "AMBFENCE test dropped MP seq=%u\n", h.SenderID);
            enet_packet_destroy(pkt);
            continue;
        }
        if (enet_peer_send(l.P, Chan_MP, pkt) != 0) { enet_packet_destroy(pkt); l.SequenceOverflow = true; continue; }
        StLinkTx++;
        if (kTrace) Trace("tx", s, type, data, len, timestamp);
        sent = true;
    }
    if (sent) enet_host_flush(Host);
}

AmbientLAN::Link* AmbientLAN::HandoffLink()
{
    Link* one = nullptr;
    for (Link& l : Links)
        if (l.P) { if (one) return nullptr; one = &l; }
    return one && one->Up ? one : nullptr;
}

const AmbientLAN::Link* AmbientLAN::HandoffLink() const
{
    return const_cast<AmbientLAN*>(this)->HandoffLink();
}

bool AmbientLAN::HandoffBuildCompatible() const
{
    const Link* link = HandoffLink();
    if (!Cfg.AllowHandoff || !link || !View) return false;
    for (const Peer& peer : *View)
        if (peer.ID == link->ID)
        {
            if (peer.BuildID != Cfg.BuildID && peer.BuildID != LoggedBuildMismatch)
            {
                LoggedBuildMismatch = peer.BuildID;
                Platform::Log(Platform::LogLevel::Warn, "Ambient: no Netplay switch with %016llx: handoff compatibility "
                              "%08x there, %08x here (handoff protocol or emulation LITEV options differ)\n",
                              (unsigned long long)peer.ID, peer.BuildID, Cfg.BuildID);
            }
            return peer.BuildID == Cfg.BuildID;
        }
    return false;
}

bool AmbientLAN::HandoffPeerIPv4(std::string& address) const
{
    address.clear();
    const Link* l = HandoffLink();
    if (!l || !l->P || !HandoffBuildCompatible()) return false;
    char host[64]{};
    if (enet_address_get_host_ip(&l->P->address, host, sizeof(host)) != 0 || !host[0]) return false;
    address = host;
    return true;
}

bool AmbientLAN::SendOfferControl(Link& l, u32 type, u64 epoch, u32 mark,
                                  const std::array<u8, 16>& token)
{
    if (!Host || !l.P || !l.Up) return false;
    std::vector<u8> v;
    P32(v, kHandoffCtl); P32(v, type); P64(v, epoch); P32(v, mark);
    v.insert(v.end(), token.begin(), token.end());
    ENetPacket* p = enet_packet_create(v.data(), v.size(), ENET_PACKET_FLAG_RELIABLE);
    if (!p) return false;
    if (enet_peer_send(l.P, Chan_Ctl, p) != 0) { enet_packet_destroy(p); return false; }
    enet_host_flush(Host);
    return true;
}

void AmbientLAN::CloseHandoffOffer(Link& l)
{
    l.LastClosedOfferEpoch = std::max(l.LastClosedOfferEpoch, l.OfferEpoch);
    l.OfferEpoch = 0;
    l.OfferToken = {};
    l.OfferMode = 0;
    l.LocalOffered = l.PeerOffered = l.LocalOfferAccepted = l.PeerOfferAccepted = false;
    l.LocalOfferRom = {};
    l.PeerOfferRom = {};
}

bool AmbientLAN::ProposeHandoff(u64 epoch, const std::array<u8, 16>& token, HandoffMode mode)
{
    Link* l = HandoffLink();
    if (!l || !HandoffBuildCompatible() || !epoch || token == std::array<u8, 16>{} ||
        (mode != HandoffPlainNetplay && mode != HandoffHostedNetplay) || l->OfferEpoch || l->FenceEpoch ||
        epoch <= std::max(l->LastClosedOfferEpoch, l->LastClosedFenceEpoch)) return false;
    if (!SendOfferControl(*l, HandoffOffer, epoch, mode, token)) return false;
    l->OfferEpoch = epoch;
    l->OfferToken = token;
    l->OfferMode = mode;
    l->LocalOffered = true;
    return true;
}

bool AmbientLAN::IncomingHandoffOffer(u64& epoch, HandoffMode& mode) const
{
    const Link* l = HandoffLink();
    if (!l || !HandoffBuildCompatible() || !l->PeerOffered || l->LocalOfferAccepted || l->FenceEpoch)
        return false;
    epoch = l->OfferEpoch;
    mode = static_cast<HandoffMode>(l->OfferMode);
    return true;
}

bool AmbientLAN::RespondToHandoffOffer(u64 epoch, bool accept)
{
    Link* l = HandoffLink();
    if (!l || !HandoffBuildCompatible() || !l->PeerOffered || l->LocalOfferAccepted ||
        l->OfferEpoch != epoch || l->FenceEpoch) return false;
    if (!SendOfferControl(*l, accept ? HandoffAccept : HandoffDecline, epoch,
                          accept ? l->OfferMode : 0, l->OfferToken)) return false;
    if (accept) l->LocalOfferAccepted = true;
    else CloseHandoffOffer(*l);
    return true;
}

bool AmbientLAN::HandoffOfferReady(u64 epoch, const std::array<u8, 16>& token,
                                    HandoffMode mode) const
{
    const Link* l = HandoffLink();
    return l && HandoffBuildCompatible() && l->OfferEpoch == epoch &&
           l->OfferToken == token && l->OfferMode == mode &&
           ((l->LocalOffered && l->PeerOfferAccepted) ||
            (l->PeerOffered && l->LocalOfferAccepted));
}

bool AmbientLAN::GetHandoffOfferStatus(HandoffOfferStatus& status) const
{
    const Link* l = HandoffLink();
    if (!l || !HandoffBuildCompatible() || !l->OfferEpoch) return false;
    status = {l->OfferEpoch, l->ID, static_cast<HandoffMode>(l->OfferMode),
              l->LocalOffered, (l->LocalOffered && l->PeerOfferAccepted) ||
                                   (l->PeerOffered && l->LocalOfferAccepted)};
    return true;
}

bool AmbientLAN::PublishHandoffRom(const NetplayRom& rom)
{
    Link* l = HandoffLink();
    if (!l || !rom.Size || rom.Size > (512u << 20) ||
        !HandoffOfferReady(l->OfferEpoch, l->OfferToken, static_cast<HandoffMode>(l->OfferMode))) return false;
    if (l->LocalOfferRom.Size) return l->LocalOfferRom == rom;
    std::vector<u8> v;
    P32(v, kHandoffCtl); P32(v, HandoffRom); P64(v, l->OfferEpoch);
    v.insert(v.end(), l->OfferToken.begin(), l->OfferToken.end());
    P64(v, rom.Size);
    v.insert(v.end(), rom.Sha, rom.Sha + 32);
    v.insert(v.end(), rom.GameCode, rom.GameCode + 4);
    v.insert(v.end(), rom.Title, rom.Title + 12);
    if (v.size() != kHandoffRomBytes) return false;
    ENetPacket* p = enet_packet_create(v.data(), v.size(), ENET_PACKET_FLAG_RELIABLE);
    if (!p) return false;
    if (enet_peer_send(l->P, Chan_Ctl, p) != 0) { enet_packet_destroy(p); return false; }
    enet_host_flush(Host);
    l->LocalOfferRom = rom;
    return true;
}

bool AmbientLAN::GetPeerHandoffRom(NetplayRom& rom) const
{
    const Link* l = HandoffLink();
    if (!l || !l->PeerOfferRom.Size ||
        !HandoffOfferReady(l->OfferEpoch, l->OfferToken, static_cast<HandoffMode>(l->OfferMode))) return false;
    rom = l->PeerOfferRom;
    return true;
}

void AmbientLAN::ReceiveHandoffRom(Link& l, const u8* bytes, size_t length)
{
    if (length != kHandoffRomBytes || G32(bytes) != kHandoffCtl || G32(bytes + 4) != HandoffRom)
        return;
    const u64 epoch = (u64)G32(bytes + 8) | ((u64)G32(bytes + 12) << 32);
    std::array<u8, 16> token{};
    memcpy(token.data(), bytes + 16, token.size());
    if (!HandoffOfferReady(epoch, token, static_cast<HandoffMode>(l.OfferMode))) return;
    NetplayRom rom;
    rom.Size = (u64)G32(bytes + 32) | ((u64)G32(bytes + 36) << 32);
    if (!rom.Size || rom.Size > (512u << 20)) return;
    memcpy(rom.Sha, bytes + 40, 32);
    memcpy(rom.GameCode, bytes + 72, 4);
    memcpy(rom.Title, bytes + 76, 12);
    if (l.PeerOfferRom.Size && l.PeerOfferRom != rom)
    {
        SendOfferControl(l, HandoffCancel, l.OfferEpoch, 0, l.OfferToken);
        CloseHandoffOffer(l);
        return;
    }
    l.PeerOfferRom = rom;
}

void AmbientLAN::CancelHandoffOffer()
{
    Link* l = HandoffLink();
    if (!l || !l->OfferEpoch) return;
    if (l->FenceEpoch) { AbortHandoffFence(); return; }
    SendOfferControl(*l, HandoffCancel, l->OfferEpoch, 0, l->OfferToken);
    CloseHandoffOffer(*l);
}

AmbientLAN::Link* AmbientLAN::PacketLink(const QueuedPacket& q) const
{
    return QueueCurrent(q) ? const_cast<Link*>(&Links[q.Slot]) : nullptr;
}

void AmbientLAN::DestroyRX(const QueuedPacket& q)
{
    enet_packet_destroy(q.Packet);
}

void AmbientLAN::RetireMP(const QueuedPacket& q)
{
    ENetPacket* p = q.Packet;
    if (!p || p->dataLength < sizeof(MPPacketHeader)) return;
    Link* l = PacketLink(q);
    if (!l || !((MPPacketHeader*)p->data)->SenderID) return;
    const u32 seq = ((MPPacketHeader*)p->data)->SenderID;
    if (kFenceTrace && seq <= 8)
        fprintf(stderr, "AMBFENCE retire seq=%u through=%u\n", seq, l->RetiredThrough);
    if (seq <= l->RetiredThrough) return;
    if (seq == l->RetiredThrough + 1)
    {
        l->RetiredThrough = seq;
        while (l->RetiredBeyond.erase(l->RetiredThrough + 1)) l->RetiredThrough++;
        l->GapSinceMs = l->RetiredBeyond.empty() ? 0 : NowMs();
    }
    else if (seq - l->RetiredThrough <= 1024)
    {
        l->RetiredBeyond.emplace(seq, NowMs());
        if (!l->GapSinceMs) l->GapSinceMs = NowMs();
    }
    else
        l->SequenceOverflow = true;   // no silent handoff after a large missing range
}

void AmbientLAN::SettleMPGaps(Link& l, u32 now)
{
    if (l.RetiredBeyond.empty()) { l.GapSinceMs = 0; return; }
    // A sequence number missing for kMPGapExpireMs while a later one was retired is lost. Every
    // such gap expires at once: expiring only the oldest per period (as this did) let RetiredThrough
    // fall ~1000 packets behind on Wi-Fi with a loss every second, and a fence mark more than 1024
    // ahead then tripped SequenceOverflow ("cannot start Ambient cut fence", r8b-mac).
    u32 through = 0;
    for (const auto& [seq, ms] : l.RetiredBeyond) if (now - ms >= kMPGapExpireMs) through = seq;
    if (!through) return;
    if (kFenceTrace)
        fprintf(stderr, "AMBFENCE expired MP gaps %u..%u\n", l.RetiredThrough + 1, through);
    l.RetiredThrough = through;
    l.RetiredBeyond.erase(l.RetiredBeyond.begin(), l.RetiredBeyond.upper_bound(through));
    while (l.RetiredBeyond.erase(l.RetiredThrough + 1)) l.RetiredThrough++;
    l.GapSinceMs = l.RetiredBeyond.empty() ? 0 : now;
}

void AmbientLAN::SettleHandoffMP(Link& l, u32 now)
{
    if (!l.LocalFence || !l.PeerFence || l.FenceAbort || l.SequenceOverflow ||
        l.RetiredThrough >= l.PeerLastSent) return;
    // Both fences are known and this side's game is frozen at its cut: it consumes nothing more,
    // so every packet of the peer's up to its fence mark not consumed by now - queued, still in
    // flight or lost - is lost to both checkpoints alike. Retire them now: waiting 1 s for the
    // unreliable channel to go quiet (as this did) froze both games 1-2.5 s longer at every cut.
    // Late arrivals are dropped at admission (DropThroughSeq), also across a rollback.
    (void)now;
    if (l.PeerLastSent - l.RetiredThrough > 4096)
    { l.SequenceOverflow = true; return; }
    const int slot = (int)(&l - Links);
    for (auto it = RX.begin(); it != RX.end(); )
    {
        const auto* h = (const MPPacketHeader*)it->Packet->data;
        if (it->Slot == slot && h->SenderID && h->SenderID <= l.PeerLastSent)
        {
            const QueuedPacket q = *it;
            it = RX.erase(it);
            DestroyRX(q);
        }
        else ++it;
    }
    if (kFenceTrace)
        fprintf(stderr, "AMBFENCE drained lost MP %u..%u\n", l.RetiredThrough + 1, l.PeerLastSent);
    l.RetiredThrough = l.PeerLastSent;
    l.RetiredBeyond.erase(l.RetiredBeyond.begin(), l.RetiredBeyond.upper_bound(l.PeerLastSent));
    l.GapSinceMs = l.RetiredBeyond.empty() ? 0 : now;
}

void AmbientLAN::SendFenceControl(Link& l, u32 type, u64 epoch, u32 mark)
{
    if (!Host || !l.P || !l.Up) return;
    std::vector<u8> v;
    P32(v, kHandoffCtl); P32(v, type); P64(v, epoch); P32(v, mark);
    v.insert(v.end(), l.FenceToken.begin(), l.FenceToken.end());
    ENetPacket* p = enet_packet_create(v.data(), v.size(), ENET_PACKET_FLAG_RELIABLE);
    if (!p) { l.FenceAbort = true; return; }
    if (enet_peer_send(l.P, Chan_Ctl, p) != 0) { enet_packet_destroy(p); l.FenceAbort = true; return; }
    enet_host_flush(Host);
}

// A fenced link carries only handoff control while each side stages its checkpoint copies, and an
// Android peer's emulator thread (which services this link) loads ROMs and runs the reversible proof
// in blocks of up to ~7 s (RG DS, Pokémon B/W 2026-10-14) - past the normal 5 s ENet timeout, which
// dropped the link right after PREPARED. Allow 30 s while fenced; the normal timeout comes back with
// the reset.
static void FencedLinkTimeout(ENetPeer* p, bool fenced)
{
    if (p) enet_peer_timeout(p, 0, fenced ? 30000 : 5000, fenced ? 30000 : 5000);
}

bool AmbientLAN::StartHandoffFence(u64 epoch, const std::array<u8, 16>& token)
{
    Link* l = HandoffLink();
    auto no = [&](const char* why) { FenceStartFailure = why; return false; };
    if (!l) return no("no single live Ambient link");
    if (!HandoffBuildCompatible()) return no("peer not in the presence view or handoff build differs");
    if (!epoch) return no("no epoch");
    if (l->FenceAbort) return no(l->PeerAbort ? "peer aborted the fence" : "fence already aborted");
    if (l->SequenceOverflow) return no("sequence overflow");
    // no token: the peer's fence token, else the accepted offer's (either side may fence first)
    const auto& effectiveToken = token != std::array<u8, 16>{} ? token : l->PeerFence ? l->FenceToken : l->OfferToken;
    if (Cfg.RequireHandoffOffer &&
        !HandoffOfferReady(epoch, effectiveToken, static_cast<HandoffMode>(l->OfferMode)))
        return no(l->OfferEpoch != epoch ? "offer epoch differs" : l->OfferToken != effectiveToken ? "offer token differs" :
                  "offer not accepted on both sides");
    if (epoch <= l->LastClosedFenceEpoch) return no("epoch already closed");
    if (l->FenceEpoch && l->FenceEpoch != epoch) return no("another fence epoch is open");
    if (l->LocalFence) return no("already fenced");
    if (l->PeerFence)
    {
        if (token != std::array<u8, 16>{} && token != l->FenceToken) return no("peer fence token differs");
    }
    else
    {
        if (effectiveToken == std::array<u8, 16>{}) return no("no token");
        l->FenceToken = effectiveToken;
    }
    l->FenceEpoch = epoch;
    l->ActiveMs = NowMs();
    l->FenceLastSent = l->TxSeq;
    l->LocalFence = true;
    l->LocalFenceMs = NowMs();
    FencedLinkTimeout(l->P, true);
    SendFenceControl(*l, HandoffFence, epoch, l->FenceLastSent);
    return !l->FenceAbort;
}

void AmbientLAN::EndCommittedHandoff()
{
    if (Link* l = HandoffLink()) { CompleteHandoffReset(*l); l->ResetCompleted = false; }
}

bool AmbientLAN::GetHandoffFenceToken(std::array<u8, 16>& token) const
{
    const Link* l = HandoffLink();
    if (!l || !l->FenceEpoch || l->FenceToken == std::array<u8, 16>{}) return false;
    token = l->FenceToken;
    return true;
}

bool AmbientLAN::HasPeerHandoffFence() const
{
    const Link* l = HandoffLink();
    return l && HandoffBuildCompatible() && l->PeerFence && !l->FenceAbort;
}

u64 AmbientLAN::PeerHandoffEpoch() const
{
    const Link* l = HandoffLink();
    return l && l->PeerFence ? l->FenceEpoch : 0;
}

bool AmbientLAN::CanAcknowledgeHandoffFence() const
{
    const Link* l = HandoffLink();
    return l && HandoffBuildCompatible() && l->LocalFence && l->PeerFence && !l->FenceAbort && !l->SequenceOverflow &&
           l->RetiredThrough >= l->PeerLastSent && HandoffQueuedFrames() == 0;
}

bool AmbientLAN::AcknowledgeHandoffFence()
{
    Link* l = HandoffLink();
    if (!CanAcknowledgeHandoffFence() || l->LocalReady) return false;
    l->LocalReady = true;
    SendFenceControl(*l, HandoffReady, l->FenceEpoch, l->RetiredThrough);
    return !l->FenceAbort;
}

bool AmbientLAN::HandoffFenceComplete() const
{
    const Link* l = HandoffLink();
    return l && CanAcknowledgeHandoffFence() && l->LocalReady && l->PeerReady &&
           l->PeerRetiredThrough >= l->FenceLastSent;
}

bool AmbientLAN::HandoffFenceAborted() const
{
    const Link* l = HandoffLink();
    return !l || l->FenceAbort || l->SequenceOverflow;
}

bool AmbientLAN::AcknowledgeHandoffPrepared()
{
    Link* l = HandoffLink();
    if (!l || !HandoffFenceComplete() || l->FenceAbort || l->LocalPrepared) return false;
    l->LocalPrepared = true;
    SendFenceControl(*l, HandoffPrepared, l->FenceEpoch, 0);
    return !l->FenceAbort;
}

bool AmbientLAN::HandoffPreparedComplete() const
{
    const Link* l = HandoffLink();
    return l && HandoffFenceComplete() && l->LocalPrepared && l->PeerPrepared && !l->FenceAbort;
}

bool AmbientLAN::AcknowledgeHandoffArmed()
{
    Link* l = HandoffLink();
    if (!l || !HandoffPreparedComplete() || l->FenceAbort || l->LocalArmed) return false;
    l->LocalArmed = true;
    SendFenceControl(*l, HandoffArmed, l->FenceEpoch, 0);
    return !l->FenceAbort;
}

bool AmbientLAN::HandoffArmedComplete() const
{
    const Link* l = HandoffLink();
    return l && HandoffPreparedComplete() && l->LocalArmed && l->PeerArmed && !l->FenceAbort;
}

bool AmbientLAN::AcknowledgeHandoffGo()
{
    Link* l = HandoffLink();
    if (!l || !HandoffArmedComplete() || l->FenceAbort || l->LocalGo) return false;
    l->LocalGo = true;
    SendFenceControl(*l, HandoffGo, l->FenceEpoch, 0);
    return !l->FenceAbort;
}

bool AmbientLAN::HandoffGoComplete() const
{
    const Link* l = HandoffLink();
    return l && HandoffArmedComplete() && l->LocalGo && l->PeerGo && !l->FenceAbort;
}

void AmbientLAN::AbortHandoffFence()
{
    Link* l = HandoffLink();
    if (!l) return;
    if (!l->AbortSent && l->FenceEpoch) { l->AbortSent = true; SendFenceControl(*l, HandoffAbort, l->FenceEpoch, 0); }
    l->FenceAbort = true;
}

bool AmbientLAN::ResetHandoffFenceAfterAbort()
{
    Link* l = HandoffLink();
    if (!l) return false;
    if (l->ResetCompleted) { l->ResetCompleted = false; return true; }
    if (!l->FenceAbort || !l->AbortSent || !l->PeerAbort) return false;
    if (!l->LocalResetDone)
    {
        l->LocalResetDone = true;
        SendFenceControl(*l, HandoffResetDone, l->FenceEpoch, 0);
    }
    if (!l->PeerResetDone) return false;
    CompleteHandoffReset(*l);
    l->ResetCompleted = false;
    return true;
}

void AmbientLAN::CompleteHandoffReset(Link& l)
{
    if (l.FenceEpoch)
    {
        l.LastClosedFenceEpoch = std::max(l.LastClosedFenceEpoch, l.FenceEpoch);
        l.LastClosedFenceToken = l.FenceToken;
    }
    l.FenceEpoch = 0;
    FencedLinkTimeout(l.P, false);
    l.ActiveMs = NowMs(); // give a rolled-back source its normal idle grace period
    l.FenceToken = {};
    l.FenceLastSent = l.PeerLastSent = l.PeerRetiredThrough = 0;
    l.LocalFence = l.PeerFence = l.LocalReady = l.PeerReady = false;
    l.LocalPrepared = l.PeerPrepared = l.LocalArmed = l.PeerArmed = l.LocalGo = l.PeerGo = false;
    l.FenceAbort = l.AbortSent = l.PeerAbort = l.LocalResetDone = l.PeerResetDone = false;
    CloseHandoffOffer(l);
    l.ResetCompleted = true;
}

bool AmbientLAN::InjectClosedHandoffFenceForTest()
{
    Link* l = HandoffLink();
    if (!Host || !l || !l->P || !l->LastClosedFenceEpoch || l->LocalFence ||
        l->LastClosedFenceToken == std::array<u8, 16>{}) return false;
    std::vector<u8> v;
    P32(v, kHandoffCtl); P32(v, HandoffFence); P64(v, l->LastClosedFenceEpoch); P32(v, l->TxSeq);
    v.insert(v.end(), l->LastClosedFenceToken.begin(), l->LastClosedFenceToken.end());
    ENetPacket* p = enet_packet_create(v.data(), v.size(), ENET_PACKET_FLAG_RELIABLE);
    if (!p) return false;
    if (enet_peer_send(l->P, Chan_Ctl, p) != 0) { enet_packet_destroy(p); return false; }
    enet_host_flush(Host);
    return true;
}

AmbientLAN::HandoffFenceStats AmbientLAN::GetHandoffFenceStats() const
{
    const Link* l = HandoffLink();
    if (!l) return {0, 0, 0, 0, 0, false, false, true};
    return {l->FenceEpoch, l->FenceLastSent, l->PeerLastSent, l->RetiredThrough,
            l->PeerRetiredThrough, l->LocalReady, l->PeerReady, l->FenceAbort || l->SequenceOverflow};
}

void AmbientLAN::ReceiveHandoffControl(Link& l, u32 type, u64 epoch, u32 mark,
                                       const std::array<u8, 16>& token)
{
    if (type == HandoffOffer)
    {
        if (!HandoffBuildCompatible() || !epoch || token == std::array<u8, 16>{} ||
            (mark != HandoffPlainNetplay && mark != HandoffHostedNetplay) || l.FenceEpoch ||
            epoch <= std::max(l.LastClosedOfferEpoch, l.LastClosedFenceEpoch)) return;
        if (l.OfferEpoch)
        {
            // Concurrent offers fail closed. Both users can retry once the
            // reliable decline has retired; neither game is frozen.
            if (l.LocalOffered || l.OfferEpoch != epoch || l.OfferToken != token || l.OfferMode != mark)
                SendOfferControl(l, HandoffDecline, epoch, 0, token);
            return;
        }
        l.OfferEpoch = epoch;
        l.OfferToken = token;
        l.OfferMode = mark;
        l.PeerOffered = true;
        return;
    }
    if (type == HandoffAccept)
    {
        if (l.LocalOffered && l.OfferEpoch == epoch && l.OfferToken == token && l.OfferMode == mark)
            l.PeerOfferAccepted = true;
        return;
    }
    if (type == HandoffDecline || type == HandoffCancel)
    {
        if (mark == 0 && l.OfferEpoch == epoch && l.OfferToken == token && !l.FenceEpoch)
            CloseHandoffOffer(l);
        return;
    }

    if (epoch && epoch <= l.LastClosedFenceEpoch)
    {
        if (getenv("LITEV_AMBIENT_CUT_STALE_REPLAY") &&
            *getenv("LITEV_AMBIENT_CUT_STALE_REPLAY"))
            fprintf(stderr, "CUTFENCE ignored stale closed epoch %llx\n", (unsigned long long)epoch);
    }
    else if (type == HandoffFence && Cfg.RequireHandoffOffer &&
             !HandoffOfferReady(epoch, token, static_cast<HandoffMode>(l.OfferMode)))
    {
        // A link by itself is never consent to freeze the game. Ignore a
        // premature/mismatched fence; the sender will time out and abort.
    }
    else if (!epoch || token == std::array<u8, 16>{} ||
             (l.FenceEpoch && (l.FenceEpoch != epoch || l.FenceToken != token))) l.FenceAbort = true;
    else if (type == HandoffFence)
    {
        l.FenceEpoch = epoch;
        l.ActiveMs = NowMs();
        l.FenceToken = token;
        l.PeerFence = true;
        FencedLinkTimeout(l.P, true);
        l.PeerLastSent = mark;
        l.PeerFenceMs = NowMs();
        l.DropThroughSeq = std::max(l.DropThroughSeq, mark);
    }
    else if (type == HandoffReady && l.FenceEpoch == epoch)
    {
        l.PeerReady = true;
        l.PeerRetiredThrough = mark;
    }
    else if (type == HandoffPrepared && l.FenceEpoch == epoch && mark == 0)
        l.PeerPrepared = true;
    else if (type == HandoffArmed && l.FenceEpoch == epoch && mark == 0 && l.PeerPrepared)
        l.PeerArmed = true;
    else if (type == HandoffGo && l.FenceEpoch == epoch && mark == 0 && l.PeerArmed)
        l.PeerGo = true;
    else if (type == HandoffAbort && l.FenceEpoch == epoch)
    { l.PeerAbort = true; l.FenceAbort = true; }
    else if (type == HandoffResetDone && l.FenceEpoch == epoch && mark == 0 &&
             l.FenceAbort && l.PeerAbort)
    {
        l.PeerResetDone = true;
        if (l.LocalResetDone) CompleteHandoffReset(l);
    }
}

// a received MP frame: from a link of this session? (a child takes frames from its parent only)
bool AmbientLAN::Admit(ENetEvent& ev)
{
    Link* l = (Link*)ev.peer->data;
    if (!l || ev.packet->dataLength < sizeof(MPPacketHeader)) return false;
    const MPPacketHeader* h = (const MPPacketHeader*)ev.packet->data;
    if (kFenceTrace && h->SenderID <= 8)
        fprintf(stderr, "AMBFENCE admit seq=%u type=%u len=%u packet=%zu parent=%d slot=%d\n",
                h->SenderID, h->Type, h->Length, ev.packet->dataLength,
                ParentSlot, (int)(l - Links));
    if (ev.packet->dataLength < sizeof(MPPacketHeader) + 12 + 24) return false;
    if (h->Magic != kNifi || h->Length + sizeof(MPPacketHeader) > ev.packet->dataLength) return false;
    const int s = (int)(l - Links);
    if (ParentSlot >= 0 && s != ParentSlot) return false;
    l->LastMPMs = NowMs();
    if (h->SenderID && h->SenderID <= l->DropThroughSeq &&
        (l->LocalFence || l->LastClosedFenceEpoch)) return false;
    const u8* f = ev.packet->data + sizeof(MPPacketHeader) + 12;
    if (f[0] != 0x80) l->ActiveMs = NowMs();
    if (!(f[10] & 1)) memcpy(l->MAC, f + 10, 6);     // (the sender's own MAC)
    if (f[0] == 0xC0 && !l->CloseAtMs) l->CloseAtMs = NowMs() + 500;   // deauth: the session is over
    StLinkRx++;
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
        LinksChanged();
        Platform::Log(Platform::LogLevel::Info, "Ambient: link %d <- %016llx (%s)\n", s, (unsigned long long)p->ID, p->Name.c_str());
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
        LinksChanged();
        break;
    }
    case ENET_EVENT_TYPE_RECEIVE:
    {
        Link* l = (Link*)ev.peer->data;
        if (l && ev.channelID == Chan_Ctl)
        {
            if (SimHalfRttUs)   // (in order, as the reliable channel delivers them)
                DelayedControls.push_back({NowUs() + SimHalfRttUs, (int)(l - Links), l->ID,
                                           std::vector<u8>(ev.packet->data, ev.packet->data + ev.packet->dataLength)});
            else ReceiveControl(*l, ev.packet->data, ev.packet->dataLength);
        }
        enet_packet_destroy(ev.packet);
        break;
    }
    default:
        break;
    }
}

void AmbientLAN::DeliverDelayedControls()
{
    while (!DelayedControls.empty() && DelayedControls.front().DueUs <= NowUs())
    {
        const DelayedControl d = std::move(DelayedControls.front());
        DelayedControls.erase(DelayedControls.begin());
        if (Links[d.Slot].P && Links[d.Slot].ID == d.ID) ReceiveControl(Links[d.Slot], d.Bytes.data(), d.Bytes.size());
    }
}

void AmbientLAN::ReceiveControl(Link& l, const u8* d, size_t length)
{
    if (length == kHandoffRomBytes) ReceiveHandoffRom(l, d, length);
    else if (length == 36 && G32(d) == kHandoffCtl)
    {
        const u32 type = G32(d + 4);
        const u64 epoch = (u64)G32(d + 8) | ((u64)G32(d + 12) << 32);
        const u32 mark = G32(d + 16);
        std::array<u8, 16> token{};
        memcpy(token.data(), d + 20, token.size());
        ReceiveHandoffControl(l, type, epoch, mark, token);
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
void AmbientLAN::Pump(int type)
{
    if (!Host) return;
    u32 now = NowMs();
    while (!RX.empty())
    {
        const QueuedPacket q = RX.front();
        ENetPacket* p = q.Packet;
        if (!QueueCurrent(q)) { RX.pop_front(); DestroyRX(q); continue; }
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
            RX.pop_front(); RetireMP(q); DestroyRX(q); continue;
        }
        if (type >= 2) return;
        if (type == 1)
        {
            if (h->Type == 0) return;
            RX.pop_front(); RetireMP(q); DestroyRX(q);
        }
        break;
    }
    int timeout = type == 2 ? RecvTimeout : 0;
    u32 last = NowMs();
    ENetEvent ev;
    // (tests, LITEV_HANDOFF_SIM_RTT_MS: delayed control messages arrive during the wait, as real ones do)
    auto service = [&] {
        if (!SimHalfRttUs) return enet_host_service(Host, &ev, timeout);
        for (int waited = 0;; waited++)
        {
            DeliverDelayedControls();
            const int r = enet_host_service(Host, &ev, 0);
            if (r != 0 || waited >= timeout) return r;
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    };
    while (service() > 0)
    {
        if (ev.type == ENET_EVENT_TYPE_RECEIVE && ev.channelID == Chan_MP)
        {
            if (Admit(ev))
            {
                ((MPPacketHeader*)ev.packet->data)->Magic = NowMs();
                const Link* l = (const Link*)ev.peer->data;
                RX.push_back({ev.packet, l->ID, l->Generation, (int)(l - Links)});
                return;
            }
            enet_packet_destroy(ev.packet);
        }
        else OnEvent(ev);
        if (type == 2)
        {
            const u32 t = NowMs();
            timeout -= (int)(t - last);
            last = t;
            if (timeout <= 0) return;
        }
    }
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
    if (!PacketLink(q)) { DestroyRX(q); return Pop(data, timestamp); }
    RetireMP(q);
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
    DestroyRX(q);
    return (int)len;
}

void AmbientLAN::Process()
{
    DeliverDelayedControls();
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
        SettleMPGaps(l, now);
        SettleHandoffMP(l, now);
        if (l.Up) rtt = std::max<u32>(rtt, l.P->roundTripTime);
        if (l.CloseAtMs && (s32)(now - l.CloseAtMs) >= 0) CloseLink(s, "deauth");
        else if (now - l.ActiveMs > (l.FenceEpoch ? kHandoffIdleMs : kIdleMs))
            CloseLink(s, "idle");
    }
    StMaxRtt = rtt;
    if (StSlowDismissRequested.exchange(false)) Slowdown.Dismiss();
    const auto slow = Slowdown.Observe(NowUs(), UpMask() != 0, frameBlocked, frameExchanges);
    StSlowFps = slow.FpsX100;
    StSlowWait = slow.WaitUsPerFrame;
    StSlowExch = slow.ExchangesPerFrameX100;
    StSlowPending = slow.Pending;
}

size_t AmbientLAN::HandoffQueuedFrames() const
{
    size_t count = RX.size();
    for (const Link& link : Links) count += link.Pending.size();
    return count;
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
    while (!RX.empty()) { DestroyRX(RX.front()); RX.pop_front(); }
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
    // After the peer has announced a cut, it may intentionally send no more
    // host CMDs. Do not amplify each virtual receive poll into a 50 ms wall
    // wait while this console finishes its current emulated frame. The final
    // checkpoint still requires a zero busy mask and a fully retired fence.
    Pump(HasPeerHandoffFence() ? 3 : 2);
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
    for (;;)
    {
        Pump(2);
        if (RX.empty()) break;
        const QueuedPacket q = RX.front();
        ENetPacket* p = q.Packet;
        RX.pop_front();
        if (!QueueCurrent(q)) { DestroyRX(q); continue; }
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
            if ((replied & up) == up || (ret & aidmask) == aidmask) { RetireMP(q); DestroyRX(q); break; }
        }
        else if (kWaitTrace && (h->Type & 0xFFFF) == 2)
            Platform::Log(Platform::LogLevel::Info,
                          "AMBWAIT old reply slot=%d reply_ts=%llu want_ts=%llu\n",
                          q.Slot, (unsigned long long)h->Timestamp, (unsigned long long)timestamp);
        RetireMP(q);
        DestroyRX(q);
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
