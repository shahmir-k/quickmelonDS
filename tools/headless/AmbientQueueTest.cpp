// Deterministic regression for RX frames retained across an AmbientLAN link-slot reuse.
#include "AmbientLAN.h"
#include "Platform.h"
#include "MPLinkFlags.h"

#include <arpa/inet.h>
#ifdef NDEBUG
#undef NDEBUG // Assertions are the test, including calls with side effects, even in Release builds.
#endif
#include <cassert>
#include <chrono>
#include <cstring>
#include <cstdio>
#include <thread>

namespace melonDS::Platform
{
void Log(LogLevel, const char*, ...) {}
}

namespace melonDS
{
struct AmbientLANQueueTestAccess
{
    static u32 NowMs()
    {
        return (u32)std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }
    static u64 NowUs()
    {
        return (u64)std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
    }

    static AmbientLAN::QueuedPacket Make(int slot, u64 id, u64 generation, u32 type, u8 marker, u32 seq = 0)
    {
        constexpr int len = 12 + 24;
        ENetPacket* p = enet_packet_create(nullptr, sizeof(MPPacketHeader) + len, 0);
        assert(p);
        MPPacketHeader h = {NowMs(), seq, type, len, 100};
        memcpy(p->data, &h, sizeof(h));
        memset(p->data + sizeof(h), 0, len);
        p->data[sizeof(h)] = marker;
        return {p, id, generation, slot};
    }

    static void Run()
    {
        AmbientLAN::Config cfg;
        cfg.DeviceID = 0x101;
        cfg.BuildID = 0x1234;
        cfg.Name = "rx-test";
        cfg.PresencePort = 0;
        cfg.LinkPort = 18066;
        AmbientLAN ambient(cfg);
        assert(ambient.Ok());
        AmbientLAN::Peer peer;
        peer.ID = 0x201;
        peer.IP = inet_addr("127.0.0.1");
        peer.LinkPort = ambient.GetLinkPort();
        const int slot = ambient.OpenLink(peer);
        assert(slot >= 0);
        const u64 oldGen = ambient.Links[slot].Generation;
        ambient.RX.push_back(Make(slot, peer.ID, oldGen, 0, 0xA1));
        ambient.RX.push_back(Make(slot, peer.ID, oldGen, 2 | (1u << 16), 0xA2));
        ambient.CloseLink(slot, "test close");
        assert(ambient.RX.empty()); // close disposes queued management and reply frames

        // Rejoin the same peer, so an ID-only check cannot hide slot-reuse bugs.
        assert(ambient.OpenLink(peer) == slot);
        const u64 newGen = ambient.Links[slot].Generation;
        assert(newGen != oldGen);
        ambient.Links[slot].Up = true;

        auto buildPeer = std::make_shared<std::vector<AmbientLAN::Peer>>(1);
        (*buildPeer)[0].ID = peer.ID;
        (*buildPeer)[0].BuildID = 0x9876;
        ambient.View = buildPeer;
        assert(!ambient.HandoffBuildCompatible());
        std::array<u8, 16> token{};
        token[0] = 1;
        assert(!ambient.StartHandoffFence(1, token));
        (*buildPeer)[0].BuildID = cfg.BuildID;
        assert(ambient.HandoffBuildCompatible());
        std::string peerAddress;
        assert(ambient.HandoffPeerIPv4(peerAddress) && peerAddress == "127.0.0.1");
        ambient.Cfg.AllowHandoff = false;
        assert(!ambient.HandoffBuildCompatible());
        ambient.Cfg.AllowHandoff = true;

        // Simulate a frame that was set aside by RecvReplies before the close and reinserted later.
        ambient.RX.push_back(Make(slot, 0x201, oldGen, 0, 0xB1));
        ambient.RX.push_back(Make(slot, peer.ID, newGen, 0, 0xB2));
        ambient.Pump(1);
        u8 frame[2048] = {};
        u64 timestamp = 0;
        assert(ambient.Pop(frame, &timestamp) == 12 + 24);
        assert(frame[0] == 0xB2 && timestamp == 100);
        assert(ambient.RX.empty());

        ambient.RX.push_back(Make(slot, 0x201, oldGen, 2 | (1u << 16), 0xC1));
        ambient.RX.push_back(Make(slot, peer.ID, newGen, 2 | (1u << 16), 0xC2));
        u8 replies[15 * 1024] = {};
        assert(ambient.RecvReplies(0, replies, 100, 1u << 1) == (1u << 1));
        assert(replies[0] == 0xC2);
        assert(ambient.RX.empty());

        // With a link present the game sees network frames, not synthesized beacons.
        auto beaconPeer = std::make_shared<std::vector<AmbientLAN::Peer>>(1);
        (*beaconPeer)[0].ID = 0x303;
        (*beaconPeer)[0].Beacon.assign(64, 0);
        (*beaconPeer)[0].Beacon[9] = 6;
        (*beaconPeer)[0].Beacon[50] = 0xD1;
        (*beaconPeer)[0].BeaconUs = NowUs();
        ambient.View = beaconPeer;
        WifiChannel = 6;
        const auto armBeacon = [&] {
            ambient.View = beaconPeer;
            ambient.Cands = {{0, 0x303, WifiUs, 1000000}};
            ambient.NextInjectUs = ambient.Cands[0].DueUs;
        };
        armBeacon();
        assert(ambient.NumLinks == 1);
        assert(ambient.RecvPacket(0, frame, &timestamp) == 0);

        // Deauth deadline and 10-second non-beacon idle both return to synthesis.
        ambient.Links[slot].CloseAtMs = NowMs() - 1;
        ambient.Process();
        assert(ambient.NumLinks == 0 && ambient.RX.empty());
        armBeacon();
        assert(ambient.RecvPacket(0, frame, &timestamp) == 64 && frame[50] == 0xD1);

        assert(ambient.OpenLink(peer) == slot);
        ambient.Links[slot].ActiveMs = NowMs() - 10001;
        ambient.Process();
        assert(ambient.NumLinks == 0 && ambient.RX.empty());
        armBeacon();
        assert(ambient.RecvPacket(0, frame, &timestamp) == 64 && frame[50] == 0xD1);

        // Beacons and their TSF follow the chip's clock, not the wall clock (fast-forward / slow motion).
        WifiUs = 50000000;
        armBeacon();
        ambient.Cands[0].TsfOff = 300 - WifiUs;
        ambient.RecvPacket(0, frame, &timestamp);
        u64 tsf;
        memcpy(&tsf, frame + 36, 8);
        assert(tsf == 300);
        assert(ambient.RecvPacket(0, frame, &timestamp) == 0);       // not due again at the same emulated time
        WifiUs += 1000000;
        assert(ambient.RecvPacket(0, frame, &timestamp) == 64);      // one interval of emulated time, no wall time
        memcpy(&tsf, frame + 36, 8);
        assert(tsf == 1000300);
        WifiUs -= 30000000;                                          // a savestate from earlier: the same phase, TSF on
        assert(ambient.RecvPacket(0, frame, &timestamp) == 0);
        WifiUs += 1000000;
        assert(ambient.RecvPacket(0, frame, &timestamp) == 64);
        memcpy(&tsf, frame + 36, 8);
        assert(tsf == 2000300);

        // RecvReplies: old replies streaming in do not extend its wait past RecvTimeout.
        auto linkPeer = std::make_shared<std::vector<AmbientLAN::Peer>>(1);
        (*linkPeer)[0].ID = 0x404;
        (*linkPeer)[0].IP = inet_addr("127.0.0.1");
        (*linkPeer)[0].SeenMs = NowMs();
        { std::lock_guard<std::mutex> g(ambient.Mu); ambient.Peers = *linkPeer; ambient.Snap = linkPeer; }     // (nearby: a link is accepted)
        ENetHost* other = enet_host_create(nullptr, 1, 2, 0, 0);
        assert(other);
        ENetAddress a; a.host = inet_addr("127.0.0.1"); a.port = ambient.GetLinkPort();
        ENetPeer* op = enet_host_connect(other, &a, 2, 0x404);
        assert(op);
        ENetEvent ev;
        for (int i = 0; i < 200 && !(ambient.UpMask() && op->state == ENET_PEER_STATE_CONNECTED); i++)
        {
            ambient.Pump(0);
            enet_host_service(other, &ev, 5);
        }
        assert(ambient.UpMask());
        std::atomic<bool> stop{false};
        std::thread sender([&] {
            const u64 s0 = NowUs();
            while (!stop && NowUs() - s0 < 300000)
            {
                constexpr int len = 12 + 24;
                ENetPacket* p = enet_packet_create(nullptr, sizeof(MPPacketHeader) + len, ENET_PACKET_FLAG_UNSEQUENCED);
                MPPacketHeader h = {0x4946494E, 0, 2 | (1u << 16), len, 1};      // a reply to an old CMD
                memcpy(p->data, &h, sizeof(h));
                memset(p->data + sizeof(h), 0, len);
                p->data[sizeof(h) + 12] = 0x28;
                enet_peer_send(op, 1, p);
                enet_host_service(other, &ev, 0);
                std::this_thread::sleep_for(std::chrono::milliseconds(2));
            }
        });
        ambient.SetRecvTimeout(25);
        const u64 t0 = NowUs();
        assert(ambient.RecvReplies(0, replies, 1000000, 1u << 1) == 0);
        const u64 waited = NowUs() - t0;
        stop = true;
        sender.join();
        std::printf("ambient-queue-test: RecvReplies under an old-reply stream waited %.1f ms (RecvTimeout 25)\n", waited / 1000.0);
        assert(waited >= 24000 && waited < 40000);

        // The LAN verdict (AmbientLAN.h). A device before a link: emulation + one round trip against 1000/58 ms.
        ambient.Quit = true;        // (the presence thread would write LinkRttUs meanwhile)
        ambient.Presence.join();
        ambient.EmuBaseUs = 10000; ambient.VEmuAtLink = 10000;
        ambient.VStallAt = ambient.StHostWaitTimeouts + ambient.StClientWaitMisses;   // (the deadline test above timed out)
        {
            std::lock_guard<std::mutex> g(ambient.Mu);
            AmbientLAN::Peer& p = ambient.Peers[0];
            p.RttN = 2; p.RttS[0] = p.RttS[1] = 20000;
        }
        assert(ambient.PeerVerdict(0x404) == AmbientLAN::Verdict_Unknown);    // (2 round trips: not yet)
        { std::lock_guard<std::mutex> g(ambient.Mu); auto& p = ambient.Peers[0]; p.RttN = 4; p.RttS[2] = 20000; p.RttS[3] = 400000; }
        assert(ambient.PeerVerdict(0x404) == AmbientLAN::Verdict_LanBad);     // 10 + 20 ms > 17.2 (the spike is not the median)
        { std::lock_guard<std::mutex> g(ambient.Mu); auto& p = ambient.Peers[0]; for (auto& r : p.RttS) r = 5000; }
        assert(ambient.PeerVerdict(0x404) == AmbientLAN::Verdict_LanOk);      // 10 + 5 ms: a 0-5 ms LAN
        ambient.EmuBaseUs = 17600;
        assert(ambient.PeerVerdict(0x404) == AmbientLAN::Verdict_LanOk);      // a device too slow on its own: not the network
        ambient.EmuBaseUs = 16000;
        { std::lock_guard<std::mutex> g(ambient.Mu); for (auto& r : ambient.Peers[0].RttS) r = 1500; }
        assert(ambient.PeerVerdict(0x404) == AmbientLAN::Verdict_LanOk);      // a 1.5 ms LAN never
        { std::lock_guard<std::mutex> g(ambient.Mu); for (auto& r : ambient.Peers[0].RttS) r = 5000; }
        ambient.EmuBaseUs = 10000;

        // The live session: half-second windows of exchanges/frame x the link's round trip + emulation.
        assert(ambient.UpMask());
        // a window: its frames' emulation + link waits (NoteFrameUs), exchanges/frame, the round trip
        auto window = [&](u32 rttUs, u32 exchPerFrame, u32 frameUs, bool settled = true) {
            ambient.LinkRttUs = rttUs;
            ambient.FirstExchUs = NowUs() - 2000000;
            ambient.VT0 = NowUs() - 600000;
            ambient.VFrames = 29; ambient.VExch = 29 * exchPerFrame - exchPerFrame;
            ambient.VRunN = 29; ambient.VRunUs = 29ull * frameUs;
            if (settled && ambient.VExchFrames < 1000) ambient.VExchFrames = 1000;     // (past the setup bursts)
            ambient.NoteFrameUs(frameUs);
            ambient.Judge(frameUs - 10000, exchPerFrame);     // (10 ms of each frame emulates, the rest waits)
            return ambient.SessionVerdict();
        };
        assert(window(2000, 1, 11000) == AmbientLAN::Verdict_Unknown);    // one good window
        assert(window(2000, 1, 11000) == AmbientLAN::Verdict_LanOk);
        assert(window(5000, 1, 15000) == AmbientLAN::Verdict_LanOk);      // a 5 ms LAN, one exchange a frame
        assert(window(40000, 1, 50000) == AmbientLAN::Verdict_LanOk);     // a spike: one window
        assert(window(2000, 1, 11000) == AmbientLAN::Verdict_LanOk);
        assert(window(5000, 1, 18500) == AmbientLAN::Verdict_LanOk);      // 54 fps: within the margin
        assert(window(5000, 1, 18500) == AmbientLAN::Verdict_LanOk);
        assert(window(9000, 1, 21000) == AmbientLAN::Verdict_LanOk);      // 21 ms frames, 12 without the network: bad
        assert(window(9000, 1, 21000) == AmbientLAN::Verdict_LanBad);
        assert(ambient.GetStats().SlowdownPending == false);       // (raised by Process, below)
        ambient.Process();
        assert(ambient.GetStats().SlowdownPending);
        for (int i = 0; i < 3; i++) assert(window(2000, 1, 11000) == AmbientLAN::Verdict_LanBad);   // recovery needs 4
        assert(window(2000, 1, 11000) == AmbientLAN::Verdict_LanOk);
        // a race start: 5 exchanges a frame, frames slow at 0 ms too (partners loading): not the network, also when
        // the round trips are half the frame but waits time out
        for (int i = 0; i < 4; i++) assert(window(4600, 5, 60000) == AmbientLAN::Verdict_LanOk);
        for (int i = 0; i < 4; i++) { ambient.StHostWaitTimeouts++; assert(window(4800, 5, 44000) == AmbientLAN::Verdict_LanOk); }
        // Shrek: 3 exchanges a frame at 6.5 ms (+ 1 ms turnaround each): 31 fps, 73 without the round trips (4 windows)
        for (int i = 0; i < 3; i++) assert(window(6500, 3, 32500) == AmbientLAN::Verdict_LanOk);
        assert(window(6500, 3, 32500) == AmbientLAN::Verdict_LanBad);
        for (int i = 0; i < 4; i++) window(2000, 1, 11000);
        // ... at 6 ms with a slower partner: 24 fps, the round trips 17 ms of 42, 24.6 without them (after 600 frames)
        for (int i = 0; i < 3; i++) assert(window(6000, 3, 42000) == AmbientLAN::Verdict_LanOk);
        assert(window(6000, 3, 42000) == AmbientLAN::Verdict_LanBad);
        for (int i = 0; i < 4; i++) window(2000, 1, 11000);
        // ... the same in the first 600 frames of exchanges (a setup burst looks like it): not counted
        ambient.VExchFrames = 0;
        for (int i = 0; i < 8; i++) { ambient.VExchFrames = 30 * i; assert(window(6000, 3, 42000, false) == AmbientLAN::Verdict_LanOk); }
        for (int i = 0; i < 4; i++) window(2000, 1, 11000);
        // a burst window that runs 55 fps without the round trips, between partner timeouts: 3 in a row do not switch
        for (int i = 0; i < 3; i++) assert(window(2800, 5, 32000) == AmbientLAN::Verdict_LanOk);
        ambient.StHostWaitTimeouts++;
        assert(window(2800, 5, 32000) == AmbientLAN::Verdict_LanOk);
        // an MKDS setup burst at 7 ms: 5 exchanges a frame, 35 ms of round trips in 80 ms frames (45 without them,
        // as slow at 0 ms): never bad, also with no wait timing out
        for (int i = 0; i < 8; i++) assert(window(7000, 5, 80000) == AmbientLAN::Verdict_LanOk);
        enet_host_destroy(other);
    }

    static void RunOfferGate()
    {
        AmbientLAN::Config cfg;
        cfg.DeviceID = 0x401;
        cfg.BuildID = 0x1234;
        cfg.Name = "offer-test";
        cfg.RequireHandoffOffer = true;
        cfg.PresencePort = 0;
        cfg.LinkPort = 18067;
        AmbientLAN ambient(cfg);
        assert(ambient.Ok());
        AmbientLAN::Peer peer;
        peer.ID = 0x402;
        peer.IP = inet_addr("127.0.0.1");
        peer.LinkPort = ambient.GetLinkPort();
        const int slot = ambient.OpenLink(peer);
        assert(slot >= 0);
        auto& link = ambient.Links[slot];
        link.Up = true;
        auto view = std::make_shared<std::vector<AmbientLAN::Peer>>(1, peer);
        (*view)[0].BuildID = cfg.BuildID;
        ambient.View = view;
        assert(ambient.HandoffBuildCompatible());

        std::array<u8, 16> token{};
        token[0] = 0xA5;
        // Wire types 1=FENCE, 8=OFFER, 9=ACCEPT, 11=CANCEL.
        ambient.ReceiveHandoffControl(link, 1, 100, 0, token);
        assert(!link.PeerFence && link.FenceEpoch == 0); // no consent: ignore a peer cut
        ambient.ReceiveHandoffControl(link, 8, 100, AmbientLAN::HandoffPlainNetplay, token);
        AmbientLAN::HandoffOfferStatus status{};
        assert(ambient.GetHandoffOfferStatus(status));
        assert(status.Epoch == 100 && status.PeerID == peer.ID && !status.Outgoing && !status.Accepted);
        u64 epoch = 0;
        AmbientLAN::HandoffMode mode = AmbientLAN::HandoffHostedNetplay;
        assert(ambient.IncomingHandoffOffer(epoch, mode));
        assert(epoch == 100 && mode == AmbientLAN::HandoffPlainNetplay);
        assert(!ambient.HandoffOfferReady(epoch, token, mode));
        link.LocalOfferAccepted = true; // the response send is exercised by the integration gate
        assert(ambient.GetHandoffOfferStatus(status) && status.Accepted);
        assert(ambient.HandoffOfferReady(epoch, token, mode));
        std::vector<u8> manifest(88);
        auto put32 = [&](size_t at, u32 value) {
            for (int i = 0; i < 4; i++) manifest[at + i] = (u8)(value >> (8 * i));
        };
        put32(0, 0x46444E48); // HNDF
        put32(4, 12);         // ROM manifest
        put32(8, 100);
        memcpy(manifest.data() + 16, token.data(), token.size());
        put32(32, 0x2000000);
        memset(manifest.data() + 40, 0xA5, 32);
        memcpy(manifest.data() + 72, "IRAO", 4);
        memcpy(manifest.data() + 76, "POKEMON WHITE", 12);
        NetplayRom described;
        manifest[16] ^= 1;
        ambient.ReceiveHandoffRom(link, manifest.data(), manifest.size());
        assert(!ambient.GetPeerHandoffRom(described)); // wrong offer token
        manifest[16] ^= 1;
        ambient.ReceiveHandoffRom(link, manifest.data(), manifest.size());
        assert(ambient.GetPeerHandoffRom(described));
        assert(described.Size == 0x2000000 && described.Sha[0] == 0xA5 &&
               !memcmp(described.GameCode, "IRAO", 4));
        std::array<u8, 16> wrong = token;
        wrong[0] ^= 1;
        ambient.ReceiveHandoffControl(link, 1, 100, 0, wrong);
        assert(!link.PeerFence && link.FenceEpoch == 0);
        ambient.ReceiveHandoffControl(link, 1, 100, 0, token);
        assert(link.PeerFence && link.FenceEpoch == 100);
        link.FenceEpoch = 0;
        link.PeerFence = false;
        ambient.CloseHandoffOffer(link);
        ambient.ReceiveHandoffControl(link, 8, 100, AmbientLAN::HandoffPlainNetplay, token);
        assert(!link.PeerOffered); // replay of a closed offer

        link.OfferEpoch = 101;
        link.OfferToken = token;
        link.OfferMode = AmbientLAN::HandoffHostedNetplay;
        link.LocalOffered = true;
        ambient.ReceiveHandoffControl(link, 9, 101, AmbientLAN::HandoffHostedNetplay, wrong);
        assert(!link.PeerOfferAccepted);
        ambient.ReceiveHandoffControl(link, 9, 101, AmbientLAN::HandoffHostedNetplay, token);
        assert(link.PeerOfferAccepted);
        assert(ambient.HandoffOfferReady(101, token, AmbientLAN::HandoffHostedNetplay));
        ambient.ReceiveHandoffControl(link, 11, 101, 0, token);
        assert(!ambient.HandoffOfferReady(101, token, AmbientLAN::HandoffHostedNetplay));
        assert(link.LastClosedOfferEpoch == 101);
    }

    static void RunHandoffLoss()
    {
        AmbientLAN::Config cfg;
        cfg.DeviceID = 0x501;
        cfg.Name = "loss-test";
        cfg.PresencePort = 0;
        cfg.LinkPort = 18068;
        AmbientLAN ambient(cfg);
        assert(ambient.Ok());
        AmbientLAN::Peer peer;
        peer.ID = 0x502;
        peer.IP = inet_addr("127.0.0.1");
        peer.LinkPort = ambient.GetLinkPort();
        const int slot = ambient.OpenLink(peer);
        assert(slot >= 0);
        auto& link = ambient.Links[slot];
        link.Up = true;
        auto retire = [&](u32 seq) {
            const auto q = Make(slot, peer.ID, link.Generation, 0, 0, seq);
            ambient.RetireMP(q);
            ambient.DestroyRX(q);
        };
        retire(1);
        retire(3); // packet 2 was lost on the unsequenced MP channel
        retire(4);
        assert(link.RetiredThrough == 1 && link.RetiredBeyond.size() == 2);
        for (auto& r : link.RetiredBeyond) r.second = NowMs() - 1001;   // (a gap expires 1 s after a later packet retired)
        ambient.SettleMPGaps(link, NowMs());
        assert(link.RetiredThrough == 4 && link.RetiredBeyond.empty());

        // The last packet can also be lost, leaving no later sequence to
        // expose the gap. Both frozen peers wait for a quiet drain before the
        // receiver marks that packet expired and votes READY.
        link.LocalFence = link.PeerFence = true;
        link.FenceEpoch = 101;
        link.PeerLastSent = 5;
        link.DropThroughSeq = 5;
        link.LocalFenceMs = link.PeerFenceMs = link.LastMPMs = NowMs() - 1001;
        ambient.SettleHandoffMP(link, NowMs());
        assert(link.RetiredThrough == 5);

        auto late = Make(slot, peer.ID, link.Generation, 0, 0, 5);
        ((MPPacketHeader*)late.Packet->data)->Magic = 0x4946494E;
        ENetEvent event{};
        event.peer = link.P;
        event.packet = late.Packet;
        assert(!ambient.Admit(event)); // frozen cut never feeds a late frame to the game
        ambient.DestroyRX(late);
        ambient.CompleteHandoffReset(link);   // the rollback closes fence 101
        assert(link.LastClosedFenceEpoch == 101);
        late = Make(slot, peer.ID, link.Generation, 0, 0, 5);
        ((MPPacketHeader*)late.Packet->data)->Magic = 0x4946494E;
        event.packet = late.Packet;
        assert(!ambient.Admit(event)); // rollback keeps the old-cut drop boundary
        ambient.DestroyRX(late);
    }
};
}

int main()
{
    melonDS::AmbientLANQueueTestAccess::Run();
    melonDS::AmbientLANQueueTestAccess::RunOfferGate();
    melonDS::AmbientLANQueueTestAccess::RunHandoffLoss();
    std::puts("ambient-queue-test: PASS");
}
