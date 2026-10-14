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

#include <algorithm>
#include <array>
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

#if defined(LITEV_NP_QUIET_HB) && !(defined(LITEV_NP_LEAN_IO) && defined(LITEV_NP_WIRE_RUNS))
#error "LITEV_NP_QUIET_HB needs LITEV_NP_LEAN_IO and LITEV_NP_WIRE_RUNS"
#endif

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

#ifdef LITEV_NP_SPEED
// Netplay speed (fast-forward / uncapped) is ONE session state, changed by any player's fast-forward
// key. A key press travels in the input stream: bits 16..23 of a player's Keys (NDS::SetKeyMask reads
// only bits 0..11) hold the speed that player last asked for (bits 16..21) and a press counter
// (bits 22..23). A change of those bits at applied frame F is an event "set the session speed to
// this"; every device knows every player's input for F before it runs F, so every device applies the
// same events at the same frame (SpeedAt). Speed is pacing only: emulation is unaffected.
// Code: 0 = normal (1x), 1..62 = multiplier x4 (0.25x steps, 2x = 8), 63 = uncapped.
namespace NetplaySpeed
{
constexpr int kShift = 16;
constexpr u32 kMask = 0xFFu << kShift;      // code + press counter
constexpr int kUncapped = 63;
inline int Encode(float m) { return m <= 0 ? kUncapped : m == 1.0f ? 0 : (int)std::min(62.0f, std::max(1.0f, m * 4 + 0.5f)); }
inline float Multiplier(int code) { return code == kUncapped ? 0.0f : code == 0 ? 1.0f : code / 4.0f; }  // 0 = uncapped
inline int Field(const NetplayFrameInput& in) { return (int)((in.Keys & kMask) >> kShift); }   // code | presses << 6
inline int Of(const NetplayFrameInput& in) { return Field(in) & 63; }
inline void Set(NetplayFrameInput& in, int field) { in.Keys = (in.Keys & ~kMask) | ((u32)(field & 0xFF) << kShift); }
}
#endif

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
    int Delay() const { return DelayFrames; }   // the session's starting delay (frames before it have no input)
#ifdef LITEV_NP_ADAPTIVE_DELAY
    // Plain Netplay: the local player's delay follows the round trip to its slowest peer (Delay() is
    // only where it starts). Deterministic: every device applies each input at the frame its sender
    // chose. Off for Hosted Netplay (the server's acknowledgement semantics assume a fixed delay).
    bool Adaptive = false;
    static constexpr int kMaxDelay = 120;   // 2 s of input lag; past that a session only stalls
#endif
    int CurrentDelay() const { return CurDelay.load(); }
    // the slowest peer's round trip (90th percentile, last 2 s, ms), -1 = no measurement yet
    double PeerRttMs() const { return RttMaxUs.load() / 1000.0; }
#ifdef LITEV_NP_SPEED
    // The session speed for local frame `frame`, NEVER waiting: the speed events (a player's fast-forward
    // press, NetplaySpeed) of input frames up to frame - kSpeedLag, applied in frame order from the inputs
    // already here; at one frame, a press for normal speed beats one for fast-forward. If some player's
    // input for a frame has not arrived, the speed decided last stays (a device behind applies a switch
    // late; pacing only, emulation is untouched).
    // History: v1 waited until every input for frame - kSpeedLag was here: deadlock (the consoles' frame
    // counts sit more than kSpeedLag + delay apart at the same emulated time). v2 took the fastest of
    // every player's held request: one player could hold the session in fast-forward, and a player
    // whose key showed "off" while another's request ran it could not end it ("stuck").
    // requesters: bit p = player p's press set the speed returned; inputFrame: the input frame where it
    // took effect (+ kSpeedLag = the nominal switch frame, equal on every device). Called by one
    // thread, frames ascending.
    static constexpr int kSpeedLag = 30;
    int SpeedAt(int frame, u32* requesters = nullptr, int* inputFrame = nullptr);
    // The local player's fast-forward key (any thread): toggles what this device shows, from fast
    // (onCode) to normal or back, and sends that as an event with its next input. Returns the code shown.
    int ToggleSpeed(int onCode);
    // Asks for `code` (sent as an event even if it is what this player asked for last). Testing.
    void RequestSpeed(int code);
    // What the key shows: the session speed, or this player's own press while it is still in flight.
    int ShownSpeed() const { return Shown.load(); }
    // Puts the local request into an input about to be submitted (SubmitLocal records it).
    void StampSpeed(NetplayFrameInput& in) const { NetplaySpeed::Set(in, (int)Request.load()); }
    // Hosted Netplay: the server submits nothing, so it records its own input's speed bits here
    // (frames ascending) before SpeedAt(frame).
    void NoteLocalSpeed(int frame, const NetplayFrameInput& in);
    // Hosted Netplay replica: the server's session speed as of its record for `frame`.
    void ServerSpeed(int frame, int code);
    // The frontend's frame period (us) now: the adaptive delay turns the round trip into frames with
    // it (fast-forward = more frames of delay for the same network). Default 60 fps.
    void SetFramePeriodUs(int us) { FramePeriodUs = std::max(us, 500); }
#endif
    int LocalPlayer() const { return Local; }

    // The local player's input sampled at `frame` (applied at frame + Delay everywhere).
    void SubmitLocal(int frame, const NetplayFrameInput& input);
    // Handoff-only frame-zero proof over a separate UDP startup port. Later
    // input/hash packets carry the epoch+token too, rejecting old-session
    // datagrams at a reused endpoint. This is not active-LAN authentication.
    bool ArmHandoffSession(u64 epoch, const std::array<u8, 16>& token,
                           int localStartupPort,
                           const std::vector<std::pair<int, std::string>>& startupPeers);
    // After SubmitLocal(0), wait until every expected peer proves its real
    // frame-0 input, epoch and token. No frame may run until this succeeds.
    bool WaitForHandoffPeerInput(int timeoutMs);

    // The input of `player` for emulated frame `frame`; blocks until known (or Abort(), or the
    // player is dropped).
    NetplayFrameInput Get(int player, int frame);
    // Hosted Netplay server only: a player whose device sent nothing (not even the 10 ms heartbeat) this long while Get() waits for it is dropped,
    // and from then on Get() returns no input for it (nothing pressed) without waiting. The server
    // is the only device that applies inputs to that console (its replicas follow its record
    // stream), so this is deterministic. 0 = wait forever (Netplay: every device applies every
    // input itself, so all of them must wait).
    int DropAfterMs = 0;
#ifdef LITEV_NP_QUIET_HB
    // Plain Netplay: between frames, a heartbeat only to a peer that needs one (an acknowledgement
    // owed, inputs of ours it has not acknowledged for a while, or silence). Off for Hosted Netplay
    // (a replica applies its own input only once the server acknowledged it: it needs the 10 ms one).
    bool QuietHeartbeat = false;
#endif
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
#ifdef LITEV_NP_WIRE_RUNS
    static constexpr int kMaxPerPacket = 96;    // runs of equal inputs (14 B each: < 1400 B a packet)
#else
    static constexpr int kMaxPerPacket = 64;
#endif
    static constexpr int kResendMs = 10;

    int Local, DelayFrames, BindPort;
    int HandoffStartPort = 0;
    u8 HandoffStartAddr[kMaxPlayers][16] {};
    u64 HandoffEpoch = 0;
    std::array<u8, 16> HandoffToken{};
    bool HandoffProofComplete = false; // Lock: normal packets wait for frame-zero peer proof
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
    std::atomic<int> CurDelay;
    std::atomic<s64> RttMaxUs {-1};
#ifdef LITEV_NP_WIRE_RUNS
    u32 EchoSent[kMaxPlayers] {};       // the peer's send time in its last packet (its clock, us)
    u64 EchoAt[kMaxPlayers] {};         // when we took that packet
    std::deque<std::pair<u64, u32>> Rtt[kMaxPlayers];   // (when, round trip us), the last second
#endif
#ifdef LITEV_NP_SPEED
    std::map<int, int> SpeedChanges[kMaxPlayers];  // applied frame -> speed field, where it changes (= an event)
    int SpeedLast[kMaxPlayers] {};                 // the field at the newest frame recorded
    int LocalUpTo = -1;                            // our own inputs are recorded up to this applied frame
    std::atomic<int> FramePeriodUs {16667};
    int SpeedCursor = -1, SpeedCode = 0, SpeedFrom = -1;   // decided up to this input frame: code, since
    u32 SpeedWho = 0;
    std::atomic<u32> Request {0};                  // the local field: code | presses << 6 (changes only on a press)
    std::atomic<int> Shown {0};                    // what the key shows (Lock to write)
    void NoteSpeed(int player, int frame, const NetplayFrameInput& in);   // frames ascending, under Lock
    void FollowSession(int code);                  // under Lock: the key shows `code` unless our press is in flight
#endif
#ifdef LITEV_NP_ADAPTIVE_DELAY
    int LastApplied = -1;               // the local input's newest applied frame
    NetplayFrameInput LastInput, Carry; // the input there; what a skipped sample pressed
    int LowFrames = 0;                  // frames the measured need has been below the current delay
    void Adapt(int frame);
#endif
    bool IsPeer(int player) const { return player >= 0 && player < kMaxPlayers && PeerSet[player]; }
#ifdef LITEV_NP_LEAN_IO
    int WakePipe[2] {-1, -1};           // SubmitLocal -> the receive thread: "send now"
    std::atomic<u64> LastSendUs {0};    // the last Send(): the heartbeat waits kResendMs after it
    u32 RttP90[kMaxPlayers] {};         // each peer's 90th percentile (0 = none yet)
#endif
#ifdef LITEV_NP_QUIET_HB
    static constexpr int kAckDelayMs = 20;      // an acknowledgement waits this long for a frame's send to carry it
    static constexpr int kKeepaliveMs = 100;    // the longest a peer goes without a packet (it shows "waiting" after 500)
    u64 LastSentTo[kMaxPlayers] {};     // our last packet to that peer
    u64 OweAckSince[kMaxPlayers] {};    // it sent us inputs since then and wants an acknowledgement (0 = none)
    bool HeartbeatDue(int peer, u64 now) const;     // under Lock
    u64 NextHeartbeatUs();                          // takes Lock
#endif
    void Receive(const u8* data, size_t len, u64 now);   // one packet (receive thread)
    void ReceiveLoop();
    void Send(bool all = true);    // all = a frame's send (else the heartbeat)
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
// hashed once per path and size in this process, then remembered. Set fresh for
// a live handoff: a same-size replacement at a SAF URI must not reuse that hash.
NetplayRom NetplayDescribeRom(const std::string& path, bool fresh = false);
// Exact bytes acquired by a console, before its cartridge secure area can be
// re-encrypted in memory. This identity must be used for live checkpoints.
NetplayRom NetplayDescribeRomBytes(const u8* bytes, u64 size);
// The first of `paths` that holds `rom`, "" = none. Size and game code filter first, so only
// candidates that pass are hashed.
std::string NetplayFindRom(const NetplayRom& rom, const std::vector<std::string>& paths,
                           bool fresh = false);

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

// A fenced, already-running Ambient session contributes one exact console
// savestate per player. This exchange is deliberately separate from the
// start-from-ROM Netplay handshake: callers keep both original emulators and
// their Ambient links alive until replica load and a later commit succeed.
struct NetplayLiveCheckpoint
{
    u64 Epoch = 0;
    u64 Build = 0;                 // exact native-core/protocol compatibility id
    u64 RawClock = 0;
    u32 Frame = 0;
    u8 Player = 0;
    std::array<u8, 16> FenceToken{};
    NetplayRom Rom;
    std::vector<u8> Firmware;          // source console's complete SPI firmware (not in NDS savestate)
    std::vector<u8> State;
};

// player 0 listens on tcpPort; player 1 connects to hostAddress (ip:tcpPort).
// The expected peer ROM comes from the already-validated Netplay roster.
// Synchronous: call on a worker while the emulator thread keeps Ambient
// Process() serviced. preparePeer must verify/load the remote state into an
// isolated scratch replica; neither side reports success until both vote ready.
using NetplayLivePrepare = std::function<bool(const NetplayLiveCheckpoint&, std::string&)>;
// The checkpoint transport runs on a network worker, but a frontend may need
// to construct its replica on the emulator/GL thread. Give Request() to the
// transport's preparePeer callback and poll Service() on that owner thread
// while continuing to Process() the frozen Ambient link. One request at a
// time; cancel and join the worker before destroying the bridge.
class NetplayLivePrepareBridge
{
public:
    NetplayLivePrepareBridge() = default;
    NetplayLivePrepareBridge(const NetplayLivePrepareBridge&) = delete;
    NetplayLivePrepareBridge& operator=(const NetplayLivePrepareBridge&) = delete;

    bool Request(const NetplayLiveCheckpoint& checkpoint, std::string& error,
                 const std::atomic<bool>& cancel, int timeoutMs = 10000);
    // Returns true if a request was serviced (including a failed prepare).
    bool Service(const NetplayLivePrepare& prepare);
    bool HasPending() const;
    void Cancel();

private:
    mutable std::mutex Lock;
    std::condition_variable Changed;
    const NetplayLiveCheckpoint* Pending = nullptr; // worker keeps it alive until Service returns
    bool Started = false;
    bool Finished = false;
    bool Success = false;
    bool Stopped = false;
    std::string ResultError;
};

bool NetplayExchangeLiveCheckpoints(u16 tcpPort, const std::string& hostAddress,
                                    const std::string& expectedPeerIPv4,
                                    const NetplayLiveCheckpoint& local, const NetplayRom& expectedPeerRom,
                                    NetplayLivePrepare preparePeer, std::atomic<bool>& cancel,
                                    NetplayLiveCheckpoint& peer, std::string& error);

// A second, bounded rendezvous after both peers have voted GO. Neither caller
// may run the first Netplay frame until this confirms that the *same* pair of
// checkpoints is present on both sides. A lost GO leaves the other peer here
// (still on its frozen Ambient source) rather than running predicted inputs.
// This is a startup gate, not a distributed durable-commit guarantee: a peer
// disappearing after the response remains an in-doubt recovery case.
bool NetplayAwaitLiveStart(u16 tcpPort, const std::string& hostAddress,
                           const std::string& expectedPeerIPv4,
                           const NetplayLiveCheckpoint& local, const NetplayLiveCheckpoint& peer,
                           std::atomic<bool>& cancel, std::string& error);

}

#endif
