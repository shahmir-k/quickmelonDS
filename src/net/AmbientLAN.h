// LITEV_AMBIENT_LAN: "Just Play" local wireless - lobby-free DS wireless between SereneDS devices on one subnet.
//
// Three layers, so a room of 16 costs a solo player ~nothing:
//  1. Presence (own thread, own UDP sockets, never the emulator thread): every device announces itself every 2 s on
//     the subnet (broadcast to kPresencePort) and by unicast to every peer it knows (NSD / mDNS addresses from the app,
//     or LITEV_AMBIENT_PEERS), and keeps a nearby table (expires after 6 s). A liveness packet carries only a summary
//     hash; the details (name, game code, MAC, the game's current beacon frame) go out once when they change and on
//     request from a peer whose hash is stale (the liveness-vs-details split of gba-link's directory).
//  2. Beacon synthesis: the local Wi-Fi chip (MPInterface::RecvPacket) is fed beacons rebuilt from the cached frames of
//     nearby devices at their own beacon interval, only for GGIDs the local game wants (below), only while no link is
//     open. No per-beacon network traffic.
//  3. Links only for real sessions: an ENet link (LAN.cpp's frame format: MPPacketHeader + the DS TX header + 802.11
//     frame, unsequenced) opens when the local game sends a unicast frame (auth) to a nearby device's MAC, and is
//     accepted by the other side only from a device in its nearby table. A child talks only to its parent, a parent to
//     its children. Links close on deauth, Wi-Fi off, 10 s without session frames, or ENet timeout (a DS leaving range).
//
// GGIDs wanted: the GGIDs the local game has beaconed + the GGIDs beaconed by devices running the same game (game code,
// region ignored); none known yet (a game that only scans) = every beacon. NitroSDK's scan request carries no GGID
// (WMSP's scan SSID is empty), games filter the found parents themselves.
#pragma once
#ifdef LITEV_AMBIENT_LAN

#include <atomic>
#include <deque>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <enet/enet.h>

#include "types.h"
#include "MPInterface.h"

namespace melonDS
{

class AmbientLAN : public MPInterface
{
public:
    enum Visibility { Vis_Everyone = 0, Vis_WirelessOnly = 1, Vis_Hidden = 2 };

    struct Config
    {
        u64 DeviceID = 0;           // stable per install (the app: from its Ed25519 identity key)
        std::string Name;           // shown to others (<= 32 bytes)
        char GameCode[4] = {};
        u32 BuildID = 0;
        int Vis = Vis_Everyone;
        u16 PresencePort = 7065;    // subnet broadcast (shared, SO_REUSEPORT)
        u16 UnicastPort = 0;        // this device's presence socket (0 = any; the app advertises it over NSD)
        u16 LinkPort = 7066;        // ENet; the next free one if taken
    };

    // Counters for the UI / CLI and the step-2 slowdown prompt (cumulative; diff two reads)
    struct Stats
    {
        u64 Frames, PresenceTx, PresenceRx, LinkTx, LinkRx, BeaconsInjected, LinksOpened, BlockedUs, Exchanges;
        u32 LastFrameBlockedUs, LastFrameExchanges, Links, Nearby, MaxRttMs;
    };

    explicit AmbientLAN(const Config& cfg);
    AmbientLAN(const AmbientLAN&) = delete;
    AmbientLAN& operator=(const AmbientLAN&) = delete;
    ~AmbientLAN() override;

    // the console MAC for a device id: locally administered (02:..), unique per install
    static void MacFromID(u64 id, u8* mac);

    bool Ok() const { return Host != nullptr; }
    u16 GetUnicastPort() const { return UniPort; }
    u16 GetLinkPort() const { return LinkPort; }
    // a peer address found another way (Android NSD / mDNS; tests): presence goes there by unicast too
    void AddPeerAddress(u32 ipNetOrder, u16 port);
    void SetVisibility(int vis);
    void SetForeground(bool fg) { Foreground = fg; }
    // nearby devices: "id \t name \t game \t ip:port \t ggid \t build \t linked"
    std::vector<std::string> NearbyRows();
    Stats GetStats();

    void Process() override;
    void Begin(int inst) override;
    void End(int inst) override;
    int SendPacket(int inst, u8* data, int len, u64 timestamp) override;
    int RecvPacket(int inst, u8* data, u64* timestamp) override;
    int SendCmd(int inst, u8* data, int len, u64 timestamp) override;
    int SendReply(int inst, u8* data, int len, u64 timestamp, u16 aid) override;
    int SendAck(int inst, u8* data, int len, u64 timestamp) override;
    int RecvHostPacket(int inst, u8* data, u64* timestamp) override;
    u16 RecvReplies(int inst, u8* data, u64 timestamp, u16 aidmask) override;

    struct Peer
    {
        u64 ID = 0;
        u32 IP = 0;                 // network order
        u16 UniPort = 0, LinkPort = 0;
        u32 SeenMs = 0, ReqMs = 0, Hash = 0, BuildID = 0;
        u32 BcastMs = 0;            // last heard by broadcast (0: never; then it gets our presence by unicast)
        bool Full = false;          // details received
        bool HasGGID = false;
        u32 GGID = 0;
        u8 Flags = 0;               // bit 0: Wi-Fi on
        char Game[4] = {};
        u8 MAC[6] = {};
        std::string Name;
        std::vector<u8> Beacon;     // the DS TX header + beacon frame as sent (empty: not beaconing)
        u64 BeaconUs = 0;           // when it arrived (TSF aging)
    };

private:
    struct Link
    {
        u64 ID = 0;
        ENetPeer* P = nullptr;
        bool Up = false;
        u8 MAC[6] = {};
        u32 ActiveMs = 0, CloseAtMs = 0;
        std::vector<std::vector<u8>> Pending;   // frames sent while connecting
    };
    struct Cand { size_t Idx; u64 ID; u64 DueUs; u32 IntervalUs; };

    Config Cfg;
    u8 MyMAC[6];

    // ---- presence (presence thread; Mu guards the shared parts) ----
    std::mutex Mu;
    std::vector<Peer> Peers;                                // presence thread
    std::shared_ptr<const std::vector<Peer>> Snap;          // published under Mu
    std::vector<std::pair<u32, u16>> Extra;                 // under Mu
    std::vector<u8> OwnBeacon;                              // under Mu
    bool OwnDirty = true;                                   // under Mu
    std::vector<u64> LinkedIDs;                             // under Mu (UI)
    std::atomic<u32> OwnBeaconMs{0};
    std::atomic<bool> WifiOn{false}, Foreground{true}, Quit{false}, Announcing{false};
    std::atomic<int> Vis;
    int BcastSock = -1, UniSock = -1;
    u16 UniPort = 0;
    std::thread Presence;
    void PresenceLoop();
    u8 HandlePresence(const u8* p, int n, u32 ip, u16 port, u32 now, bool bcast);     // (under Mu) the reply to send
    std::vector<u8> BuildPresence(u8 kind);
    void SendPresence(const std::vector<u8>& pkt, bool bcast, const std::vector<std::pair<u32, u16>>& to);

    // ---- links + synthesis (emulator thread) ----
    ENetHost* Host = nullptr;
    u16 LinkPort = 0;
    Link Links[16];
    int NumLinks = 0;               // slots in use (connecting or up)
    int ParentSlot = -1;            // child: the link to the session's parent
    u64 LastHostID = 0;             // the device whose CMD came last (RecvHostPacket: -1 once it is gone)
    bool LastHostSet = false;
    std::deque<ENetPacket*> RX;
    std::shared_ptr<const std::vector<Peer>> View;
    u32 ViewMs = 0;
    std::set<u32> OwnGGIDs;
    u32 OwnBeaconHash = 0;
    std::vector<Cand> Cands;
    u64 NextInjectUs = ~0ull;
    u32 FrameBlockedUs = 0, FrameExchanges = 0;

    std::atomic<u64> StFrames{0}, StPresTx{0}, StPresRx{0}, StLinkTx{0}, StLinkRx{0}, StInjected{0}, StOpened{0}, StBlockedUs{0}, StExchanges{0};
    std::atomic<u32> StLastBlocked{0}, StLastExch{0}, StLinks{0}, StNearby{0}, StMaxRtt{0};

    void RefreshView(bool force);
    void RebuildCands();
    int Inject(u8* data, u64* timestamp);
    const Peer* PeerByMAC(const u8* mac);
    int SlotByMAC(const u8* mac);
    int OpenLink(const Peer& p);
    void CloseLink(int s, const char* why);
    void LinksChanged();
    void NoteBeacon(const u8* data, int len);
    void SendTo(u32 slots, u32 type, const u8* data, int len, u64 timestamp);
    void Pump(int type);
    void OnEvent(ENetEvent& ev);
    bool Admit(ENetEvent& ev);
    int Pop(u8* data, u64* timestamp);
    u32 UpMask() const;
};

}

#endif
