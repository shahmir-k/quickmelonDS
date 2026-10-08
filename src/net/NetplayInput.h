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
#include <cstring>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "types.h"
#include "LockstepMP.h"

namespace melonDS
{

// Testing only: what a bad network does to received packets (fixed delay, random extra delay,
// which also reorders them, and loss). Default: nothing.
struct NetFaults
{
    int LatencyMs = 0, JitterMs = 0, LossPct = 0;
    std::mt19937 Rng {12345};
    // a packet received at nowUs is handed on at dueUs; false = lost
    bool Deliver(u64 nowUs, u64& dueUs)
    {
        if (LossPct && (int)(Rng() % 100) < LossPct) return false;
        dueUs = nowUs + (u64)LatencyMs * 1000 + (JitterMs ? Rng() % ((u64)JitterMs * 1000 + 1) : 0);
        return true;
    }
};

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
    // (testing only); faults: more of that (jitter, loss).
    NetplayInput(int localPlayer, int delayFrames, int bindPort, const std::vector<std::pair<int, std::string>>& peers, int latencyMs = 0, const NetFaults& faults = {});
    // Two players: the peer is the other one.
    NetplayInput(int localPlayer, int delayFrames, int bindPort, const std::string& peer, int latencyMs = 0)
        : NetplayInput(localPlayer, delayFrames, bindPort, {{1 - localPlayer, peer}}, latencyMs) {}
    ~NetplayInput();

    bool Ok() const { return Socket >= 0; }
    int Delay() const { return DelayFrames; }
    int LocalPlayer() const { return Local; }

    // The local player's input sampled at `frame` (applied at frame + Delay everywhere).
    void SubmitLocal(int frame, const NetplayFrameInput& input);

    // The input of `player` for emulated frame `frame`; blocks until known (or Abort(), or the
    // player is dropped).
    NetplayFrameInput Get(int player, int frame);
    // Hosted Netplay server only: a player whose input Get() has waited this long for is dropped,
    // and from then on Get() returns no input for it (nothing pressed) without waiting. The server
    // is the only device that applies inputs to that console (its replicas follow its record
    // stream), so this is deterministic. 0 = wait forever (Netplay: every device applies every
    // input itself, so all of them must wait).
    int DropAfterMs = 0;
    bool Dropped(int player) const { return (DroppedMask.load() >> player) & 1; }
    // `peer` has acknowledged all of our inputs up to this applied frame. Hosted Netplay: the server
    // applies a player's input exactly when it acknowledged it (it ignores a dropped player's
    // packets), so a replica may apply its own input at frame F <= this; past it, only the
    // server's (from its record stream) is certain.
    int AckedBy(int peer);
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

    int Local, DelayFrames;
    NetFaults Faults;
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
    std::atomic<u32> DroppedMask {0};

    bool IsPeer(int player) const { return player >= 0 && player < kMaxPlayers && PeerSet[player]; }
    void ReceiveLoop();
    void Send();
};

// A ROM's identity. Every player may run a different game (Pokemon White with Pokemon Black):
// console j runs player j's ROM on every device that runs it, found there by this. SHA-256, as a
// ROM received from a peer is cached under it: a forged file must not take a real ROM's place.
struct NetplayRom
{
    u8 Sha[32] {};          // SHA-256 of the whole file
    u64 Size = 0;           // 0 = no ROM (unreadable)
    char GameCode[4] {};    // header 0x0C
    char Title[12] {};      // header 0x00
    bool operator==(const NetplayRom& o) const { return !memcmp(Sha, o.Sha, 32) && Size == o.Size; }
    bool operator!=(const NetplayRom& o) const { return !(*this == o); }
    std::string Hex() const;    // the SHA-256 in hex (the cache file name)
    std::string GameTitle() const;  // the header title, trimmed
    std::string Name() const;   // for logs: game code, title, hash, size
};
// The ROM at `path` (Platform::OpenFile: a file or, on Android, a document URI). The whole file is
// hashed once per path and size in this process, then remembered.
NetplayRom NetplayDescribeRom(const std::string& path);
// The first of `paths` that holds `rom`, "" = none. Size and game code filter first, so only
// candidates that pass are hashed.
std::string NetplayFindRom(const NetplayRom& rom, const std::vector<std::string>& paths);

// ROM transfer during session setup, for the UI: what is happening, the consent question, cancel.
struct NetplayXferStatus
{
    enum { Idle, Asking, Sending, Receiving, Done, Failed };
    int State = Idle;
    u64 Bytes = 0, Total = 0;   // Sending/Receiving: this ROM's bytes so far (resumed ones too) / size
    std::string Title;          // Sending/Receiving: the game
    std::string Question;       // Asking: what to say yes or no to
};
NetplayXferStatus NetplayGetXferStatus();
// A NetplaySetup::Consent for a UI: shows `question` in the status (Asking) and waits for
// NetplayAnswer (or a cancel = no).
bool NetplayAskUser(const std::string& question);
void NetplayAnswer(bool yes);
// Ends a session setup in progress (any thread): it fails at once on this device, and the others
// see the connection drop. Received bytes stay in the cache's .part files for the next attempt.
void NetplayCancelSetup();

// Netplay session start (TCP, host-centric): the host (player 0) listens on Port + 1 and waits for
// NumPlayers - 1 guests; each guest connects to it and sends its player index, UDP port, ROM and
// save. The host times round trips to every guest, picks the input delay, and sends every guest
// the full player list (addresses as the host sees them, each player's ROM), every other player's
// save and the delay. Each device then finds every other console it runs (Netplay: all; Hosted:
// the server all, a replica none) a local copy of that player's ROM; if any device lacks one, the
// session fails on every device, naming the game.
// Unless a device can receive it (CacheDir): then, once every device involved said yes (Consent:
// the owner to sending, the receiver to receiving), the ROM travels over the setup connection
// (star: a guest's ROM goes to the host, which keeps a copy and passes it on), into
// CacheDir/<sha256>.part, which becomes <sha256>.nds once its SHA-256 matches (a mismatch is
// retried once from the start). An interrupted transfer resumes from its .part next session.
// Everyone thus agrees on the players, the delay, every console's ROM and every console's
// starting save.
struct NetplaySetup
{
    // in
    int Player = 0;             // this device's player; 0 = host
    int NumPlayers = 2;         // host: how many players (itself included) to wait for
    int Port = 7100;            // this device's UDP input port
    std::string Host;           // guests: the host's "ip:port" (its UDP port; TCP = port + 1)
    NetplayRom Rom;             // this player's ROM (unused on a dedicated Hosted server)
    std::string RomPath;        // where it is (to send it to a player who lacks it)
    // where this device has `rom` ("" = it does not), asked for every other console it runs;
    // unset = it has none
    std::function<std::string(const NetplayRom& rom)> FindRom;
    std::vector<u8> Save;       // this player's save
    std::string CacheDir;       // ROMs received from the others: <CacheDir>/<sha256>.nds ("" = none)
    u64 CacheMaxBytes = 2ull << 30;     // the cache's size cap: least recently used files go first
    // asked before any ROM byte moves (what and to or from whom); unset = no
    std::function<bool(const std::string& question)> Consent;
    int Delay = 0;              // host: fixed input delay, 0 = from the measured round trips
    bool Hosted = false;        // host: Hosted Netplay server (the guests' inputs only travel to the
                                // host and back: delay from twice the slowest one-way time)
    bool HostPlays = true;      // host: it is player 0 (else a dedicated server: NumPlayers guests,
                                // player 0 among them)
    bool Join = false;          // a Hosted Netplay replica: a guest (even as player 0) that runs
                                // only its own console
    int TimeoutS = 60;          // how long to wait for the others
    // out
    std::vector<std::pair<int, std::string>> Peers;     // every other player: (player, "ip:port")
    std::map<int, std::vector<u8>> Saves;               // every other player's save
    std::map<int, std::string> RomPaths;                // every other console this device runs: its ROM
    std::map<int, NetplayRom> Roms;                     // every player's ROM
    std::string Log;            // what happened (errors included), one line per event
};
bool NetplayHandshake(NetplaySetup& s);

}

#endif
