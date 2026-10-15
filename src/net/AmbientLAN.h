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
#include "AmbientSlowdown.h"

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
        u64 HostWaitUs, HostWaitCalls, HostWaitMisses, HostWaitTimeouts;
        u64 ClientWaitUs, ClientWaitCalls, ClientWaitMisses;
        u64 Stale[4];               // MP frames dropped as stale (too old for the chip), by type: regular, CMD, reply, ack
        u32 LastFrameBlockedUs, LastFrameExchanges, Links, Nearby, MaxRttMs;
        u32 LinkRttUs;              // the slowest linked device's median presence round trip
        u64 LinkTxBytes, LinkRxBytes;   // MP frame bytes (DS TX header + 802.11 frame) over the links
        u32 EmuUs, PredFpsX100;     // emulation per frame (no link waits); the session's predicted fps (SessionVerdict)
        int Verdict;                // SessionVerdict
        u32 SlowFpsX100, SlowWaitUsPerFrame, SlowExchangesPerFrameX100;
        bool SlowdownPending;
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
    // nearby devices: "id \t name \t game \t ip:port \t ggid \t build \t linked \t rtt_us \t jitter_us \t loss_%"
    std::vector<std::string> NearbyRows();
    Stats GetStats();
    void DismissSlowdownPrompt() { StSlowDismissRequested = true; StSlowPending = false; }
    bool HasLinks() const { return StLinks.load(std::memory_order_relaxed) != 0; }
    // Group fast-forward: ONE speed for the LAN session (the linked players), in NetplaySpeed codes (0 = normal). Any
    // member's key asks for it; the parent decides and relays it (a child links only to its parent) and every member
    // switches at the same emulated time (the parent's chip clock + kSpeedLagUs: a child's clock follows its parent's),
    // so a host never runs faster than its children (it would time out their replies). A link opening is normal speed
    // for the whole group (the join runs at 1x); a player leaving runs 1x (its last link closed), the others keep the
    // group's speed. No links = no group (the frontend's own fast-forward, off for a player who just left).
    static constexpr u64 kSpeedLagUs = 250000;
    int ToggleSpeed(int onCode);                // any thread: the key; returns the code it shows
    int SpeedShown() const { return StShown.load(); }
    int Speed() const { return StSpeed.load(); }
    std::string SpeedWho();                     // the name of the player whose key set Speed()
    // the frontend, after each frame: its emulation wall time (RunFrame, link waits included)
    void NoteFrameUs(u32 us) { RunUs = us; }

    // Can a LAN session hold 58 fps on this network, or should it be Netplay? From the presence round trips (RTT: the
    // median of the last 16; same-game or hosting devices 1 a second, linked ones 4) and this device's emulation per
    // frame (E: NoteFrameUs without the link waits; the lowest half-second average of the last 2 s, as a loading
    // phase emulates more; 10 ms until measured, an RG DS). Each MP exchange stops emulation for one round trip.
    //  - PeerVerdict (a device, before or at link open): LAN_BAD when E + 1 x RTT > 1000/58 ms (not even one exchange
    //    a frame fits), with E alone under it (a device slow on its own is no case for Netplay) and RTT >= 2 ms;
    //    UNKNOWN before 3 round trips. RG DS #1 measured E 3.8-4.8 ms in Pokémon B/W: LAN_BAD above ~12.5 ms.
    //  - SessionVerdict (the live session), a vote every 0.5 s from 1 s after the first exchange, from the window's
    //    frames F (NoteFrameUs: emulation + link waits, no pacing) and round trips N = exchanges/frame x RTT:
    //      the link rule (the PeerVerdict rule, with the lower of E now and at link: a loading phase emulates more):
    //        LAN_BAD after 2 windows in a row;
    //      with F missing 50 fps and no wait timing out (a timeout: a partner loading), LAN_BAD after 4 in a row:
    //        F - N makes 50 (the network holds it back), or, after 600 frames of exchanges, N is 40% of F and F - N
    //        makes 40 (a partner slow too).
    //    Back to LAN_OK after 4 good windows. Setup bursts, measured at 0 ms: an MKDS lobby / race start runs 5-6
    //    exchanges a frame at 14-48 fps for ~400 frames, then 1 a frame; Pokémon B/W runs 2 a frame from link to the
    //    end (join, room, trade, battle), Shrek 3 (no burst). The 4 windows, the timeouts a burst brings and the 600
    //    frames keep a burst on a delay where steady play is fine from switching. All steady rates are >= 1 a frame,
    //    as the link rule assumes. RTT here: the median of 16 (4 s linked): a Wi-Fi spike moves neither it nor 2
    //    windows. 50: a margin; pacing is not in it (fast-forward never makes it bad); one exchange a frame on a 0-5
    //    ms LAN stays under 20 ms. LAN_BAD also raises Stats::SlowdownPending at once.
    enum Verdict { Verdict_Unknown = 0, Verdict_LanOk = 1, Verdict_LanBad = 2 };
    Verdict PeerVerdict(u64 id);
    Verdict SessionVerdict() const { return (Verdict)StVerdict.load(); }

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
        // presence round trips (same game or hosting, linked): the last 16, the loss of the last 16 pings
        u32 RttS[16] = {};
        u8 RttN = 0, RttPos = 0, PingN = 0, LossPct = 0;
        u16 Sent = 0, Recv = 0;
        u32 PingMs = 0;
        u32 RttUs() const;          // median (0: none yet)
        u32 JitterUs() const;       // mean deviation from it
    };

private:
    struct Link
    {
        u64 ID = 0;
        u64 Generation = 0;         // never reused by another occupant of this slot
        ENetPeer* P = nullptr;
        bool Up = false;
        u8 MAC[6] = {};
        u32 ActiveMs = 0, CloseAtMs = 0;
        std::vector<std::vector<u8>> Pending;   // frames sent while connecting
    };
    struct QueuedPacket
    {
        ENetPacket* Packet = nullptr;
        u64 ID = 0, Generation = 0;
        int Slot = -1;
    };
    struct Cand { size_t Idx; u64 ID; u64 DueUs; u32 IntervalUs; u64 TsfOff = 0; };    // (us: the chip's clock; TSF = the chip's clock + TsfOff)

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
    std::vector<u8> BuildPing(u8 kind, u64 t);
    u64 PingEcho = 0;                                       // presence thread
    std::atomic<u32> LinkRttUs{0};                          // the slowest linked device's median round trip
    std::atomic<u32> LinkRttMaxUs{0};                       // ... and the longest of their last 16 (the link waits)

    // ---- links + synthesis (emulator thread) ----
    ENetHost* Host = nullptr;
    u16 LinkPort = 0;
    Link Links[16];
    u64 NextLinkGeneration = 0;
    int NumLinks = 0;               // slots in use (connecting or up)
    int ParentSlot = -1;            // child: the link to the session's parent
    u64 LastHostID = 0;             // the device whose CMD came last (RecvHostPacket: -1 once it is gone)
    bool LastHostSet = false;
    std::deque<QueuedPacket> RX;
    std::shared_ptr<const std::vector<Peer>> View;
    u32 ViewMs = 0;
    std::set<u32> OwnGGIDs;
    u32 OwnBeaconHash = 0;
    std::vector<Cand> Cands;
    u64 NextInjectUs = ~0ull;
    u64 EmuUs = 0;
    u32 FrameBlockedUs = 0, FrameExchanges = 0;
    u32 RunUs = 0;                  // NoteFrameUs
    std::atomic<u32> FrameEmuUs{10000};     // its emulation part, average
    u64 FirstExchUs = 0, VT0 = 0, VRunUs = 0, VStallAt = 0;     // the session verdict's windows
    u32 VEmuAtLink = 0;
    u32 VFrames = 0, VExch = 0, VRunN = 0, VBad = 0, VWeak = 0, VOk = 0;
    u64 VExchFrames = 0;            // frames since the first exchange (the weak rule waits out setup bursts)
    std::atomic<u32> EmuBaseUs{10000};      // E: the lowest half-second emulation average of the last 2 s
    u64 EmT0 = 0, EmSum = 0;
    u32 EmN = 0, EmPos = 0, EmRing[4] = {};
    bool VerdictDismissed = false;
    std::atomic<int> StVerdict{0};
    std::atomic<u32> StPredFps{0};
    int LinkWaitMs() const;
    static bool LinkRule(u32 emuUs, u32 rttUs);
    void Judge(u32 frameBlocked, u32 frameExchanges);
    int PendCode = -1;              // group speed to apply at PendAtUs (chip clock), from PendWho
    u64 PendAtUs = 0, PendWho = 0, PendWallUs = 0;
    u32 SpeedSeq = 0;               // parent: its decisions; child: the last one applied
    std::atomic<int> SpeedReq{-1}, StSpeed{0}, StShown{0};
    std::atomic<u64> StSpeedWho{0};
    AmbientSlowdown Slowdown;

    std::atomic<u64> StFrames{0}, StPresTx{0}, StPresRx{0}, StLinkTx{0}, StLinkRx{0}, StInjected{0}, StOpened{0}, StBlockedUs{0}, StExchanges{0};
    std::atomic<u64> StHostWaitUs{0}, StHostWaitCalls{0}, StHostWaitMisses{0}, StHostWaitTimeouts{0};
    std::atomic<u64> StClientWaitUs{0}, StClientWaitCalls{0}, StClientWaitMisses{0};
    std::atomic<u64> StStale[4] = {};
    std::atomic<u64> StLinkTxBytes{0}, StLinkRxBytes{0};
    std::atomic<u32> StLastBlocked{0}, StLastExch{0}, StLinks{0}, StNearby{0}, StMaxRtt{0};
    std::atomic<u32> StSlowFps{0}, StSlowWait{0}, StSlowExch{0};
    std::atomic<bool> StSlowPending{false}, StSlowDismissRequested{false};

    void RefreshView(bool force);
    void RebuildCands();
    int Inject(u8* data, u64* timestamp);
    u64 EmuNow();
    const Peer* PeerByMAC(const u8* mac);
    int SlotByMAC(const u8* mac);
    int OpenLink(const Peer& p);
    void CloseLink(int s, const char* why);
    void LinksChanged(bool opened);
    void NoteBeacon(const u8* data, int len);
    void SendTo(u32 slots, u32 type, const u8* data, int len, u64 timestamp);
    void Pump(int type, int waitMs = -1);
    void SpeedDecide(int code, u64 who, u64 atUs);
    void SpeedControl(int slot, const u8* p, size_t n);
    void SpeedTick();
    void SendCtl(u32 slots, const u8* m, int n);
    std::vector<std::pair<u64, QueuedPacket>> Delayed;      // tests (NetSim): received MP frames held until then (us)
    bool ReleaseDelayed();
    void OnEvent(ENetEvent& ev);
    bool Admit(ENetEvent& ev);
    int Pop(u8* data, u64* timestamp);
    bool QueueCurrent(const QueuedPacket& q) const;
    void DropQueuedForSlot(int slot);
    u32 UpMask() const;
    friend struct AmbientLANQueueTestAccess;
};

}

#endif
