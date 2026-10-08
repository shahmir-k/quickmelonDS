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
#include <utility>
#include <vector>

#include "types.h"
#include "LockstepMP.h"

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
// UDP, one socket per device, full mesh: the local input goes to every peer. Each packet to a peer
// carries every input that peer has not acknowledged yet (up to kMaxPerPacket, oldest first) and
// acknowledges that peer's inputs received so far; the receive thread also re-sends every
// kResendMs, so nothing is lost even if a peer starts late or this side is blocked.
// Up to kMaxPlayers players (the DS wireless maximum; N itself comes from the lobby); player indices need not be contiguous.
class NetplayInput
{
public:
    static constexpr int kMaxPlayers = LockstepMP::kMaxInst; // the DS wireless maximum

    // localPlayer: this device's player index; bindPort: local UDP port; peers: every other
    // player, (player, "ip:port"). latencyMs: artificial one-way delay added to received packets
    // (testing only).
    NetplayInput(int localPlayer, int delayFrames, int bindPort, const std::vector<std::pair<int, std::string>>& peers, int latencyMs = 0);
    // Two players: the peer is the other one.
    NetplayInput(int localPlayer, int delayFrames, int bindPort, const std::string& peer, int latencyMs = 0)
        : NetplayInput(localPlayer, delayFrames, bindPort, {{1 - localPlayer, peer}}, latencyMs) {}
    ~NetplayInput();

    bool Ok() const { return Socket >= 0; }
    int Delay() const { return DelayFrames; }
    int LocalPlayer() const { return Local; }

    // The local player's input sampled at `frame` (applied at frame + Delay everywhere).
    void SubmitLocal(int frame, const NetplayFrameInput& input);

    // The input of `player` for emulated frame `frame`; blocks until known (or Abort()).
    NetplayFrameInput Get(int player, int frame);
    // Session ending: Get() stops waiting and returns no input from now on.
    void Abort();

    // Desync check: each device sends the hash of its own console's state every so often to every
    // peer; the others compare it with their copy of that console (PeerHash(owner, ...)).
    // Unreliable: a lost report only skips one check.
    void SendHash(int frame, u64 hash);
    bool PeerHash(int player, int frame, u64& hash);

    // Total time spent blocked in Get() waiting for remote players (ms).
    double StallMs() const { return StallUs.load() / 1000.0; }
    // Time since the last packet from the quietest peer (ms); huge before the first one.
    double MsSincePeer() const;

private:
    static constexpr int kMaxPerPacket = 64;
    static constexpr int kResendMs = 10;

    int Local, DelayFrames, LatencyMs;
    int Socket = -1;
    std::vector<int> Peers;                 // the other players
    u8 PeerAddr[kMaxPlayers][16] {};        // sockaddr_in, by player
    bool PeerSet[kMaxPlayers] {};
    std::atomic<bool> Running {true};
    std::thread Receiver;

    std::mutex Lock;
    std::condition_variable Changed;
    std::map<int, NetplayFrameInput> Inputs[kMaxPlayers];   // by applied frame
    std::map<int, u64> PeerHashes[kMaxPlayers];   // each peer's own console, by frame
    std::deque<std::pair<int, NetplayFrameInput>> Unacked; // (applied frame, input), oldest first
    int PeerAck[kMaxPlayers];       // that peer has all our inputs up to this applied frame
    int RemoteUpTo[kMaxPlayers];    // we have all of that player's inputs up to this applied frame
    std::atomic<u64> StallUs {0};
    std::atomic<u64> LastPeerUs[kMaxPlayers] {};

    bool IsPeer(int player) const { return player >= 0 && player < kMaxPlayers && PeerSet[player]; }
    void ReceiveLoop();
    void Send();
};

// Netplay session start (TCP, host-centric): the host (player 0) listens on Port + 1 and waits for
// NumPlayers - 1 guests; each guest connects to it and sends its player index, UDP port, ROM id
// and save. The host times round trips to every guest, picks the input delay, and sends every
// guest the full player list (addresses as the host sees them), every other player's save and the
// delay. Everyone thus agrees on the players, the delay and every console's starting save.
struct NetplaySetup
{
    // in
    int Player = 0;             // this device's player; 0 = host
    int NumPlayers = 2;         // host: how many players (itself included) to wait for
    int Port = 7100;            // this device's UDP input port
    std::string Host;           // guests: the host's "ip:port" (its UDP port; TCP = port + 1)
    u64 RomId = 0;              // must match on every device
    std::vector<u8> Save;       // this player's save
    int Delay = 0;              // host: fixed input delay, 0 = from the measured round trips
    int TimeoutS = 60;          // how long to wait for the others
    // out
    std::vector<std::pair<int, std::string>> Peers;     // every other player: (player, "ip:port")
    std::map<int, std::vector<u8>> Saves;               // every other player's save
    std::string Log;            // what happened (errors included), one line per event
};
bool NetplayHandshake(NetplaySetup& s);

}

#endif
