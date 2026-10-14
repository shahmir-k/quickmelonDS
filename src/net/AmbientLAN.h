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
#include <array>
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
#include "NetplayInput.h"

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
        bool AllowHandoff = true;
        bool RequireHandoffOffer = false; // Android requires explicit peer consent before any cut fence
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
        u32 LastFrameBlockedUs, LastFrameExchanges, Links, Nearby, MaxRttMs;
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
    // nearby devices: "id \t name \t game \t ip:port \t ggid \t build \t linked"
    std::vector<std::string> NearbyRows();
    Stats GetStats();
    void DismissSlowdownPrompt() { StSlowDismissRequested = true; StSlowPending = false; }
    bool HasLinks() const { return StLinks.load(std::memory_order_relaxed) != 0; }
    // Emulator-thread-only diagnostic: MP frames already admitted locally or buffered while a link opens.
    size_t HandoffQueuedFrames() const;
    // Compatibility gate only: presence is unauthenticated. The checkpoint
    // exchange must still verify its full native build identity and peer.
    bool HandoffBuildCompatible() const;
    // The address of the sole *connected* ENet peer, not a presence-advertised
    // address. Bind the checkpoint and startup sockets to this IPv4 peer.
    bool HandoffPeerIPv4(std::string& address) const;
    enum HandoffMode { HandoffPlainNetplay = 1, HandoffHostedNetplay = 2 };
    // Two-player, reliable peer consent before the first frozen frame. All methods
    // are emulator-thread-only, just like the fence API below. The offer epoch
    // increases for every new attempt on this live link; the token is random.
    bool ProposeHandoff(u64 epoch, const std::array<u8, 16>& token, HandoffMode mode);
    bool IncomingHandoffOffer(u64& epoch, HandoffMode& mode) const;
    bool RespondToHandoffOffer(u64 epoch, bool accept);
    bool HandoffOfferReady(u64 epoch, const std::array<u8, 16>& token, HandoffMode mode) const;
    struct HandoffOfferStatus { u64 Epoch; u64 PeerID; HandoffMode Mode; bool Outgoing, Accepted; };
    bool GetHandoffOfferStatus(HandoffOfferStatus& status) const;
    bool PublishHandoffRom(const NetplayRom& rom);
    bool GetPeerHandoffRom(NetplayRom& rom) const;
    void CancelHandoffOffer();
    // Emulator-thread-only, two-peer cut fence. Call after the host stops issuing
    // MP commands; the client may keep running until it reaches its own idle cut.
    // A received FENCE is not READY: unsequenced MP packets must first retire.
    // The offering host must choose a nonzero epoch strictly greater than
    // every earlier offer on this live link (reconnect resets the counter).
    bool StartHandoffFence(u64 epoch, const std::array<u8, 16>& token = {});
    bool GetHandoffFenceToken(std::array<u8, 16>& token) const;
    // After a committed handoff's Netplay session ended (every device leaves at the same frame):
    // the link's fence and offer are over on both sides, with no message needed.
    void EndCommittedHandoff();
    bool HasPeerHandoffFence() const;
    u64 PeerHandoffEpoch() const;
    bool CanAcknowledgeHandoffFence() const;
    bool AcknowledgeHandoffFence();
    bool HandoffFenceComplete() const;
    bool HandoffFenceAborted() const;
    // Provisional transition votes. GO alone MUST NOT start a Lockstep frame;
    // a checkpoint-bound startup rendezvous and peer-input gate still follow.
    // None of these votes is durable commit.
    bool AcknowledgeHandoffPrepared();
    bool HandoffPreparedComplete() const;
    bool AcknowledgeHandoffArmed();
    bool HandoffArmedComplete() const;
    bool AcknowledgeHandoffGo();
    bool HandoffGoComplete() const;
    void AbortHandoffFence();
    // A failed offer may be retried only after ABORT and RESET_DONE cross on
    // the ordered reliable control channel in both directions. The receiver
    // finalizes reset while processing RESET_DONE, before a later new FENCE.
    bool ResetHandoffFenceAfterAbort();
    bool InjectClosedHandoffFenceForTest(); // diagnostic replay of the last aborted FENCE
    struct HandoffFenceStats { u64 Epoch; u32 LastSent, PeerLastSent, RetiredThrough, PeerRetiredThrough; bool LocalReady, PeerReady, Aborted; };
    HandoffFenceStats GetHandoffFenceStats() const;

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
    mutable u32 LoggedBuildMismatch = 0;   // (HandoffBuildCompatible: the peer BuildID last reported)
    struct Link
    {
        u64 ID = 0;
        u64 Generation = 0;         // never reused by another occupant of this slot
        ENetPeer* P = nullptr;
        bool Up = false;
        u8 MAC[6] = {};
        u32 ActiveMs = 0, CloseAtMs = 0;
        std::vector<std::vector<u8>> Pending;   // frames sent while connecting
        u32 TxSeq = 0, RetiredThrough = 0;
        std::set<u32> RetiredBeyond;
        u32 GapSinceMs = 0, LastMPMs = 0;
        u32 LocalFenceMs = 0, PeerFenceMs = 0, DropThroughSeq = 0;
        bool SequenceOverflow = false;
        u64 FenceEpoch = 0;
        // Retry epochs on one link must increase. Any delayed control from a
        // prior failed offer is then rejected without a bounded replay cache.
        u64 LastClosedFenceEpoch = 0;
        std::array<u8, 16> LastClosedFenceToken{}; // test replay only
        u64 OfferEpoch = 0, LastClosedOfferEpoch = 0;
        std::array<u8, 16> OfferToken{};
        u32 OfferMode = 0;
        bool LocalOffered = false, PeerOffered = false, LocalOfferAccepted = false, PeerOfferAccepted = false;
        NetplayRom LocalOfferRom, PeerOfferRom;
        std::array<u8, 16> FenceToken{};
        u32 FenceLastSent = 0, PeerLastSent = 0, PeerRetiredThrough = 0;
        bool LocalFence = false, PeerFence = false, LocalReady = false, PeerReady = false;
        bool LocalPrepared = false, PeerPrepared = false, LocalArmed = false, PeerArmed = false, LocalGo = false, PeerGo = false;
        bool FenceAbort = false, AbortSent = false, PeerAbort = false;
        bool LocalResetDone = false, PeerResetDone = false, ResetCompleted = false;
    };
    struct QueuedPacket
    {
        ENetPacket* Packet = nullptr;
        u64 ID = 0, Generation = 0;
        int Slot = -1;
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
    u64 NextLinkGeneration = 0;
    int NumLinks = 0;               // slots in use (connecting or up)
    int ParentSlot = -1;            // child: the link to the session's parent
    u64 LastHostID = 0;             // the device whose CMD came last (RecvHostPacket: -1 once it is gone)
    bool LastHostSet = false;
    std::deque<QueuedPacket> RX;
    Link* PacketLink(const QueuedPacket& q) const;
    void DestroyRX(const QueuedPacket& q);
    void RetireMP(const QueuedPacket& q);
    void SettleMPGaps(Link& link, u32 now);
    void SettleHandoffMP(Link& link, u32 now);
    void SendFenceControl(Link& link, u32 type, u64 epoch, u32 mark);
    bool SendOfferControl(Link& link, u32 type, u64 epoch, u32 mark,
                          const std::array<u8, 16>& token);
    void ReceiveHandoffControl(Link& link, u32 type, u64 epoch, u32 mark,
                               const std::array<u8, 16>& token);
    void ReceiveHandoffRom(Link& link, const u8* bytes, size_t length);
    void CloseHandoffOffer(Link& link);
    void CompleteHandoffReset(Link& link);
    Link* HandoffLink();
    const Link* HandoffLink() const;
    std::shared_ptr<const std::vector<Peer>> View;
    u32 ViewMs = 0;
    std::set<u32> OwnGGIDs;
    u32 OwnBeaconHash = 0;
    std::vector<Cand> Cands;
    u64 NextInjectUs = ~0ull;
    u32 FrameBlockedUs = 0, FrameExchanges = 0;
    AmbientSlowdown Slowdown;

    std::atomic<u64> StFrames{0}, StPresTx{0}, StPresRx{0}, StLinkTx{0}, StLinkRx{0}, StInjected{0}, StOpened{0}, StBlockedUs{0}, StExchanges{0};
    std::atomic<u64> StHostWaitUs{0}, StHostWaitCalls{0}, StHostWaitMisses{0}, StHostWaitTimeouts{0};
    std::atomic<u64> StClientWaitUs{0}, StClientWaitCalls{0}, StClientWaitMisses{0};
    std::atomic<u32> StLastBlocked{0}, StLastExch{0}, StLinks{0}, StNearby{0}, StMaxRtt{0};
    std::atomic<u32> StSlowFps{0}, StSlowWait{0}, StSlowExch{0};
    std::atomic<bool> StSlowPending{false}, StSlowDismissRequested{false};

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
    bool QueueCurrent(const QueuedPacket& q) const;
    void DropQueuedForSlot(int slot);
    u32 UpMask() const;
    friend struct AmbientLANQueueTestAccess;
    friend struct AmbientLANOfferTestAccess;
};

}

#endif
