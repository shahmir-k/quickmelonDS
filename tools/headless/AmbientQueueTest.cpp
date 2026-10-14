// Deterministic regression for RX frames retained across an AmbientLAN link-slot reuse.
#include "AmbientLAN.h"
#include "Platform.h"

#include <arpa/inet.h>
#ifdef NDEBUG
#undef NDEBUG // Assertions are the test, including calls with side effects, even in Release builds.
#endif
#include <cassert>
#include <chrono>
#include <cstring>
#include <cstdio>

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

    static AmbientLAN::QueuedPacket Make(int slot, u64 id, u64 generation, u32 type, u8 marker)
    {
        constexpr int len = 12 + 24;
        ENetPacket* p = enet_packet_create(nullptr, sizeof(MPPacketHeader) + len, 0);
        assert(p);
        MPPacketHeader h = {NowMs(), 0, type, len, 100};
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
        (*beaconPeer)[0].Beacon[40] = 0xD1;
        (*beaconPeer)[0].BeaconUs = NowUs();
        ambient.View = beaconPeer;
        WifiChannel = 6;
        const auto armBeacon = [&] {
            ambient.View = beaconPeer;
            ambient.Cands = {{0, 0x303, NowUs() - 1, 1000000}};
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
        assert(ambient.RecvPacket(0, frame, &timestamp) == 64 && frame[40] == 0xD1);

        assert(ambient.OpenLink(peer) == slot);
        ambient.Links[slot].ActiveMs = NowMs() - 10001;
        ambient.Process();
        assert(ambient.NumLinks == 0 && ambient.RX.empty());
        armBeacon();
        assert(ambient.RecvPacket(0, frame, &timestamp) == 64 && frame[40] == 0xD1);
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
};
}

int main()
{
    melonDS::AmbientLANQueueTestAccess::Run();
    melonDS::AmbientLANQueueTestAccess::RunOfferGate();
    std::puts("ambient-queue-test: PASS");
}
