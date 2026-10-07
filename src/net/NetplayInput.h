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

#ifndef NETPLAYINPUT_H
#define NETPLAYINPUT_H

#include <atomic>
#include <condition_variable>
#include <deque>
#include <map>
#include <mutex>
#include <string>
#include <thread>

#include "types.h"

namespace melonDS
{

// One player's input for one emulated frame.
struct NetplayFrameInput
{
    u32 Keys = 0xFFF;       // NDS::SetKeyMask format (active low), 0xFFF = nothing pressed
    s16 TouchX = -1;        // -1 = not touching
    s16 TouchY = -1;
};

// Netplay input exchange: every device emulates every player's console, so only inputs cross the
// network. A player's input sampled at frame F is applied at frame F + Delay on every device; the
// local device sends it at once, so the network has Delay frames to deliver it and nobody waits.
// When an input is late, Get() blocks until it arrives (a stutter, never a desync).
//
// UDP, one socket per device. Each packet carries the sender's last kRedundancy frames, so a lost
// packet is covered by the next one. Two players for now (peer = the other device).
class NetplayInput
{
public:
    // localPlayer: this device's player index (0 or 1); bindPort: local UDP port; peer: "ip:port".
    // latencyMs: artificial one-way delay added to received packets (testing only).
    NetplayInput(int localPlayer, int delayFrames, int bindPort, const std::string& peer, int latencyMs = 0);
    ~NetplayInput();

    bool Ok() const { return Socket >= 0; }
    int Delay() const { return DelayFrames; }
    int LocalPlayer() const { return Local; }

    // The local player's input sampled at `frame` (applied at frame + Delay everywhere).
    void SubmitLocal(int frame, const NetplayFrameInput& input);

    // The input of `player` for emulated frame `frame`; blocks until known.
    NetplayFrameInput Get(int player, int frame);

    // Total time spent blocked in Get() waiting for the remote player (ms).
    double StallMs() const { return StallUs.load() / 1000.0; }

private:
    static constexpr int kRedundancy = 8;

    int Local, DelayFrames, LatencyMs;
    int Socket = -1;
    u8 PeerAddr[16] {};     // sockaddr_in
    std::atomic<bool> Running {true};
    std::thread Receiver;

    std::mutex Lock;
    std::condition_variable Changed;
    std::map<int, NetplayFrameInput> Inputs[2];    // by applied frame
    std::deque<std::pair<int, NetplayFrameInput>> Sent; // last frames sent (applied frame, input)
    std::atomic<u64> StallUs {0};

    void ReceiveLoop();
};

}

#endif
