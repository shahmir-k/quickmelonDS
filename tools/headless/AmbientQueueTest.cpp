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
        enet_host_destroy(other);
    }
};
}

int main()
{
    melonDS::AmbientLANQueueTestAccess::Run();
    std::puts("ambient-queue-test: PASS");
}
