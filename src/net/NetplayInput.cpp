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
#include <unistd.h>
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

u64 NowUs()
{
    return (u64)std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
}

NetplayInput::NetplayInput(int localPlayer, int delayFrames, int bindPort, const std::string& peer, int latencyMs)
    : Local(localPlayer), DelayFrames(delayFrames), LatencyMs(latencyMs),
      PeerAck(delayFrames - 1), RemoteUpTo(delayFrames - 1) // frames before Delay are never sent
{
    Socket = socket(AF_INET, SOCK_DGRAM, 0);
    if (Socket < 0) return;

    sockaddr_in bindAddr {};
    bindAddr.sin_family = AF_INET;
    bindAddr.sin_addr.s_addr = htonl(INADDR_ANY);
    bindAddr.sin_port = htons((u16)bindPort);
    timeval tv {0, 1000}; // 1 ms, so delayed packets are released on time
    setsockopt(Socket, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    if (bind(Socket, (sockaddr*)&bindAddr, sizeof(bindAddr)) < 0)
    {
        close(Socket);
        Socket = -1;
        return;
    }

    sockaddr_in peerAddr {};
    peerAddr.sin_family = AF_INET;
    size_t colon = peer.rfind(':');
    inet_pton(AF_INET, peer.substr(0, colon).c_str(), &peerAddr.sin_addr);
    peerAddr.sin_port = htons((u16)atoi(peer.substr(colon + 1).c_str()));
    static_assert(sizeof(peerAddr) <= sizeof(PeerAddr));
    memcpy(PeerAddr, &peerAddr, sizeof(peerAddr));

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
    {
        std::lock_guard<std::mutex> lk(Lock);
        int count = std::min<int>((int)Unacked.size(), kMaxPerPacket);
        WireHeader header {kMagic, (u8)Local, (u8)count, RemoteUpTo};
        packet.resize(sizeof(header) + count * sizeof(WireEntry));
        memcpy(packet.data(), &header, sizeof(header));
        u8* p = packet.data() + sizeof(header);
        for (int i = 0; i < count; i++, p += sizeof(WireEntry))
        {
            auto& [f, in] = Unacked[i];
            WireEntry e {f, in.Keys, in.TouchX, in.TouchY};
            memcpy(p, &e, sizeof(e));
        }
    }
    sendto(Socket, packet.data(), packet.size(), 0, (sockaddr*)PeerAddr, sizeof(sockaddr_in));
}

NetplayFrameInput NetplayInput::Get(int player, int frame)
{
    if (frame < DelayFrames) return {}; // nobody has input before the delay has elapsed

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
    std::deque<std::pair<u64, std::vector<u8>>> pending; // artificial latency: (due time, packet)
    u8 buf[2048];
    u64 lastSend = 0;
    while (Running)
    {
        // heartbeat: re-sends what the peer has not acknowledged, and our acknowledgement
        if (NowUs() - lastSend >= kResendMs * 1000)
        {
            Send();
            lastSend = NowUs();
        }

        ssize_t len = recv(Socket, buf, sizeof(buf), 0);
        u64 now = NowUs();
        if (len >= (ssize_t)sizeof(WireHeader))
            pending.emplace_back(now + (u64)LatencyMs * 1000, std::vector<u8>(buf, buf + len));

        while (!pending.empty() && pending.front().first <= now)
        {
            std::vector<u8>& packet = pending.front().second;
            WireHeader header;
            memcpy(&header, packet.data(), sizeof(header));
            if (header.Magic == kMagic && header.Player < 2 && header.Player != Local &&
                packet.size() >= sizeof(header) + header.Count * sizeof(WireEntry))
            {
                std::lock_guard<std::mutex> lk(Lock);
                auto& inputs = Inputs[header.Player];
                const u8* p = packet.data() + sizeof(header);
                for (int i = 0; i < header.Count; i++, p += sizeof(WireEntry))
                {
                    WireEntry e;
                    memcpy(&e, p, sizeof(e));
                    if (e.Frame > RemoteUpTo) // older ones were received (and maybe consumed) already
                        inputs.emplace(e.Frame, NetplayFrameInput {e.Keys, e.TouchX, e.TouchY});
                }
                while (inputs.count(RemoteUpTo + 1)) RemoteUpTo++;
                PeerAck = std::max(PeerAck, header.Ack);
                while (!Unacked.empty() && Unacked.front().first <= PeerAck) Unacked.pop_front();
                Changed.notify_all();
            }
            pending.pop_front();
        }
    }
    std::lock_guard<std::mutex> lk(Lock);
    Changed.notify_all();
}

}
