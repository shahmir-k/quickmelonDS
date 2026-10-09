/*
    Copyright 2016-2026 melonDS team

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

#include <stdio.h>
#include <string.h>

#ifdef __WIN32__
    #include <winsock2.h>
    #include <ws2tcpip.h>

    #define socket_t    SOCKET
    #define sockaddr_t  SOCKADDR
    #define sockaddr_in_t  SOCKADDR_IN
#else
    #include <unistd.h>
    #include <netinet/in.h>
    #include <sys/select.h>
    #include <sys/socket.h>

    #define socket_t    int
    #define sockaddr_t  struct sockaddr
    #define sockaddr_in_t  struct sockaddr_in
    #define closesocket close
#endif

#ifndef INVALID_SOCKET
    #define INVALID_SOCKET  (socket_t)-1
#endif

#include "LAN.h"
#ifdef LITEV_LAN_STATS
#include <chrono>
#include <time.h>
#endif
#ifdef LITEV_LAN_RX_SPIN
#include <chrono>
#include <stdlib.h>
#ifdef __ANDROID__
#include <sys/system_properties.h>
#endif
#endif


namespace melonDS
{

#ifdef LITEV_LAN_STATS
// Measurement only (LITEV_LAN_STATS): where LAN multiplayer blocks. Waits are timed in us and
// bucketed by position in the DS frame (MP timestamp modulo one frame, 8 buckets); the totals
// are logged and reset every 60 Process() calls (~1 s of emulation).
static struct
{
    u64 HostWaitUs, ClientWaitUs;
    u32 HostWaits, ClientWaits, HostMisses, ClientMisses, Cmds, Replies, Packets;
    u32 HostPos[8], ClientPos[8];
    // per-stage breakdown of one exchange (kernel receive times from SO_TIMESTAMPNS in ENet)
    u64 CmdWakeUs, TurnUs, ReplyWakeUs, SpanUs;   // sums, us
    u32 CmdWakeN, TurnN, ReplyWakeN, SpanN;
} LanStats;
// Kernel receive timestamps need ENet patched to read SO_TIMESTAMPNS
// (melonDS-profiler tools/profiler/enet-rx-timestamps.patch) and LITEV_LAN_KTIME; without them
// the per-stage numbers stay 0.
#ifdef LITEV_LAN_KTIME
extern "C" unsigned long long enet_litev_last_rx_ns;
#else
static const unsigned long long enet_litev_last_rx_ns = 0;
#endif
static u64 LanRealNs()
{
    timespec ts; clock_gettime(CLOCK_REALTIME, &ts);
    return (u64)ts.tv_sec * 1000000000ull + ts.tv_nsec;
}
static u64 LanCmdKernelNs;  // client: kernel receive time of the CMD being answered
static u64 LanCmdSendNs;    // host: when the current CMD was handed to the socket
static u64 LanNowUs()
{
    return (u64) std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
static int LanFramePos(u64 timestamp) { return (int) ((timestamp % 16715) / 2090) & 7; }
#endif

const u32 kDiscoveryMagic = 0x444E414C; // LAND
const u32 kLANMagic = 0x504E414C; // LANP
const u32 kPacketMagic = 0x4946494E; // NIFI

const u32 kProtocolVersion = 1;

const u32 kLocalhost = 0x0100007F;

enum
{
    Chan_Cmd = 0,           // channel 0 -- control commands
    Chan_MP,                // channel 1 -- MP data exchange
};

enum
{
    Cmd_ClientInit = 1,     // 01 -- host->client -- init new client and assign ID
    Cmd_PlayerInfo,         // 02 -- client->host -- send client player info to host
    Cmd_PlayerList,         // 03 -- host->client -- broadcast updated player list
    Cmd_PlayerConnect,      // 04 -- both -- signal connected state (ready to receive MP frames)
    Cmd_PlayerDisconnect,   // 05 -- both -- signal disconnected state (not receiving MP frames)
    Cmd_StartSession,       // 06 -- host->client -- group command: mode, player count[, server id]
};

const int kDiscoveryPort = 7063;
const int kLANPort = 7064;


LAN::LAN() noexcept : Inited(false)
{
    DiscoveryMutex = Platform::Mutex_Create();
    PlayersMutex = Platform::Mutex_Create();

    DiscoverySocket = INVALID_SOCKET;
    DiscoveryLastTick = 0;

    Active = false;
    IsHost = false;
    Host = nullptr;
    //Lag = false;

    memset(RemotePeers, 0, sizeof(RemotePeers));
    memset(Players, 0, sizeof(Players));
    NumPlayers = 0;
    MaxPlayers = 0;

    ConnectedBitmask = 0;

    LastHostID = -1;
    LastHostPeer = nullptr;

    FrameCount = 0;

    // TODO make this somewhat nicer
    if (enet_initialize() != 0)
    {
        Platform::Log(Platform::LogLevel::Error, "LAN: failed to initialize enet\n");
        return;
    }

    Platform::Log(Platform::LogLevel::Info, "LAN: enet initialized\n");
    Inited = true;
}

LAN::~LAN() noexcept
{
    EndSession();

    Inited = false;
    enet_deinitialize();

    Platform::Mutex_Free(DiscoveryMutex);
    Platform::Mutex_Free(PlayersMutex);

    Platform::Log(Platform::LogLevel::Info, "LAN: enet deinitialized\n");
}


std::map<u32, LAN::DiscoveryData> LAN::GetDiscoveryList()
{
    Platform::Mutex_Lock(DiscoveryMutex);
    auto ret = DiscoveryList;
    Platform::Mutex_Unlock(DiscoveryMutex);
    return ret;
}

std::vector<LAN::Player> LAN::GetPlayerList()
{
    Platform::Mutex_Lock(PlayersMutex);

    std::vector<Player> ret;
    for (int i = 0; i < 16; i++)
    {
        if (Players[i].Status == Player_None) continue;

        // make a copy of the player entry, fix up the address field
        Player newp = Players[i];
        if (newp.ID == MyPlayer.ID)
        {
            newp.IsLocalPlayer = true;
            newp.Address = kLocalhost;
        }
        else
        {
            newp.IsLocalPlayer = false;
            if (newp.Status == Player_Host)
                newp.Address = HostAddress;
        }

        ret.push_back(newp);
    }

    Platform::Mutex_Unlock(PlayersMutex);
    return ret;
}


bool LAN::StartDiscovery()
{
    if (!Inited) return false;

    int res;

    DiscoverySocket = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (DiscoverySocket < 0)
    {
        DiscoverySocket = INVALID_SOCKET;
        return false;
    }

    sockaddr_in_t saddr;
    memset(&saddr, 0, sizeof(saddr));
    saddr.sin_family = AF_INET;
    saddr.sin_addr.s_addr = htonl(INADDR_ANY);
    saddr.sin_port = htons(kDiscoveryPort);
    res = bind(DiscoverySocket, (const sockaddr_t*)&saddr, sizeof(saddr));
    if (res < 0)
    {
        closesocket(DiscoverySocket);
        DiscoverySocket = INVALID_SOCKET;
        return false;
    }

    int opt_true = 1;
    res = setsockopt(DiscoverySocket, SOL_SOCKET, SO_BROADCAST, (const char*)&opt_true, sizeof(int));
    if (res < 0)
    {
        closesocket(DiscoverySocket);
        DiscoverySocket = INVALID_SOCKET;
        return false;
    }

    DiscoveryLastTick = (u32)Platform::GetMSCount();
    DiscoveryList.clear();

    Active = true;
    return true;
}

void LAN::EndDiscovery()
{
    if (!Inited) return;

    if (DiscoverySocket != INVALID_SOCKET)
    {
        closesocket(DiscoverySocket);
        DiscoverySocket = INVALID_SOCKET;
    }

    if (!IsHost)
        Active = false;
}

bool LAN::StartHost(const char* playername, int numplayers)
{
    if (!Inited) return false;
    if (numplayers > 16) return false;

    ENetAddress addr;
    addr.host = ENET_HOST_ANY;
    addr.port = kLANPort;

    Host = enet_host_create(&addr, 16, 2, 0, 0);
    if (!Host)
    {
        return false;
    }

    Platform::Mutex_Lock(PlayersMutex);

    Player* player = &Players[0];
    memset(player, 0, sizeof(Player));
    player->ID = 0;
    strncpy(player->Name, playername, 31);
    player->Status = Player_Host;
    player->Address = kLocalhost;
    NumPlayers = 1;
    MaxPlayers = numplayers;
    StartRequest = -1;
    memcpy(&MyPlayer, player, sizeof(Player));

    Platform::Mutex_Unlock(PlayersMutex);

    HostAddress = kLocalhost;
    LastHostID = -1;
    LastHostPeer = nullptr;

    Active = true;
    IsHost = true;

    StartDiscovery();
    return true;
}

bool LAN::StartClient(const char* playername, const char* host)
{
    if (!Inited) return false;

    Host = enet_host_create(nullptr, 16, 2, 0, 0);
    if (!Host)
    {
        return false;
    }

    ENetAddress addr;
    enet_address_set_host(&addr, host);
    addr.port = kLANPort;
    ENetPeer* peer = enet_host_connect(Host, &addr, 2, 0);
    if (!peer)
    {
        enet_host_destroy(Host);
        Host = nullptr;
        return false;
    }

    Platform::Mutex_Lock(PlayersMutex);

    Player* player = &MyPlayer;
    memset(player, 0, sizeof(Player));
    player->ID = 0;
    strncpy(player->Name, playername, 31);
    player->Status = Player_Connecting;
    StartRequest = -1;

    Platform::Mutex_Unlock(PlayersMutex);

    ENetEvent event;
    int conn = 0;
    u32 starttick = (u32)Platform::GetMSCount();
    const int conntimeout = 5000;
    for (;;)
    {
        u32 curtick = (u32)Platform::GetMSCount();
        if (curtick < starttick) break;
        int timeout = conntimeout - (int)(curtick - starttick);
        if (timeout < 0) break;
        if (enet_host_service(Host, &event, timeout) > 0)
        {
            if (conn == 0 && event.type == ENET_EVENT_TYPE_CONNECT)
            {
                conn = 1;
            }
            else if (conn == 1 && event.type == ENET_EVENT_TYPE_RECEIVE)
            {
                u8* data = event.packet->data;
                if (event.channelID != Chan_Cmd) continue;
                if (data[0] != Cmd_ClientInit) continue;
                if (event.packet->dataLength != 11) continue;

                u32 magic = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
                u32 version = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);
                if (magic != kLANMagic) continue;
                if (version != kProtocolVersion) continue;
                if (data[10] > 16) continue;

                MaxPlayers = data[10];

                // send player information
                MyPlayer.ID = data[9];
                u8 cmd[9+sizeof(Player)];
                cmd[0] = Cmd_PlayerInfo;
                cmd[1] = (u8)kLANMagic;
                cmd[2] = (u8)(kLANMagic >> 8);
                cmd[3] = (u8)(kLANMagic >> 16);
                cmd[4] = (u8)(kLANMagic >> 24);
                cmd[5] = (u8)kProtocolVersion;
                cmd[6] = (u8)(kProtocolVersion >> 8);
                cmd[7] = (u8)(kProtocolVersion >> 16);
                cmd[8] = (u8)(kProtocolVersion >> 24);
                memcpy(&cmd[9], &MyPlayer, sizeof(Player));
                ENetPacket* pkt = enet_packet_create(cmd, 9+sizeof(Player), ENET_PACKET_FLAG_RELIABLE);
                enet_peer_send(event.peer, Chan_Cmd, pkt);

                conn = 2;
                break;
            }
            else if (event.type == ENET_EVENT_TYPE_DISCONNECT)
            {
                conn = 0;
                break;
            }
        }
        else
            break;
    }

    if (conn != 2)
    {
        enet_peer_reset(peer);
        enet_host_destroy(Host);
        Host = nullptr;
        return false;
    }

    HostAddress = addr.host;
    LastHostID = -1;
    LastHostPeer = nullptr;
    RemotePeers[0] = peer;
    peer->data = &Players[0];

    Active = true;
    IsHost = false;
    return true;
}

void LAN::EndSession()
{
    if (!Active) return;
    if (IsHost) EndDiscovery();

    Active = false;

    while (!RXQueue.empty())
    {
        ENetPacket* packet = RXQueue.front();
        RXQueue.pop();
        enet_packet_destroy(packet);
    }

    for (int i = 0; i < 16; i++)
    {
        if (i == MyPlayer.ID) continue;

        if (RemotePeers[i])
            enet_peer_disconnect(RemotePeers[i], 0);

        RemotePeers[i] = nullptr;
    }

    // send the disconnects now, so the others see this player leave instead of timing out
    enet_host_flush(Host);
    enet_host_destroy(Host);
    Host = nullptr;
    IsHost = false;
}


void LAN::ProcessDiscovery()
{
    if (DiscoverySocket == INVALID_SOCKET)
        return;

    u32 tick = (u32)Platform::GetMSCount();
    if ((tick - DiscoveryLastTick) < 1000)
        return;

    DiscoveryLastTick = tick;

    if (IsHost)
    {
        // advertise this LAN session over the network

        DiscoveryData beacon;
        memset(&beacon, 0, sizeof(beacon));
        beacon.Magic = kDiscoveryMagic;
        beacon.Version = kProtocolVersion;
        beacon.Tick = tick;
        snprintf(beacon.SessionName, 64, "%s's game", MyPlayer.Name);
        beacon.NumPlayers = NumPlayers;
        beacon.MaxPlayers = MaxPlayers;
        beacon.Status = 0; // TODO

        sockaddr_in_t saddr;
        memset(&saddr, 0, sizeof(saddr));
        saddr.sin_family = AF_INET;
        saddr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        saddr.sin_port = htons(kDiscoveryPort);

        sendto(DiscoverySocket, (const char*)&beacon, sizeof(beacon), 0, (const sockaddr_t*)&saddr, sizeof(saddr));
    }
    else
    {
        Platform::Mutex_Lock(DiscoveryMutex);

        // listen for LAN sessions

        fd_set fd;
        struct timeval tv;
        for (;;)
        {
            FD_ZERO(&fd); FD_SET(DiscoverySocket, &fd);
            tv.tv_sec = 0; tv.tv_usec = 0;
            if (!select(DiscoverySocket+1, &fd, nullptr, nullptr, &tv))
                break;

            DiscoveryData beacon;
            sockaddr_in_t raddr;
            socklen_t ralen = sizeof(raddr);

            int rlen = recvfrom(DiscoverySocket, (char*)&beacon, sizeof(beacon), 0, (sockaddr_t*)&raddr, &ralen);
            if (rlen < sizeof(beacon)) continue;
            if (beacon.Magic != kDiscoveryMagic) continue;
            if (beacon.Version != kProtocolVersion) continue;
            if (beacon.MaxPlayers > 16) continue;
            if (beacon.NumPlayers > beacon.MaxPlayers) continue;

            u32 key = ntohl(raddr.sin_addr.s_addr);

            if (DiscoveryList.find(key) != DiscoveryList.end())
            {
                if (beacon.Tick <= DiscoveryList[key].Tick)
                    continue;
            }

            beacon.Magic = tick;
            beacon.SessionName[63] = '\0';
            DiscoveryList[key] = beacon;
        }

        // cleanup: remove hosts that haven't given a sign of life in the last 5 seconds

        std::vector<u32> deletelist;

        for (const auto& [key, data] : DiscoveryList)
        {
            u32 age = tick - data.Magic;
            if (age < 5000) continue;

            deletelist.push_back(key);
        }

        for (const auto& key : deletelist)
        {
            DiscoveryList.erase(key);
        }

        Platform::Mutex_Unlock(DiscoveryMutex);
    }
}

void LAN::HostUpdatePlayerList()
{
    u8 cmd[2+sizeof(Players)];
    cmd[0] = Cmd_PlayerList;
    cmd[1] = (u8)NumPlayers;
    memcpy(&cmd[2], Players, sizeof(Players));
    ENetPacket* pkt = enet_packet_create(cmd, 2+sizeof(Players), ENET_PACKET_FLAG_RELIABLE);
    enet_host_broadcast(Host, Chan_Cmd, pkt);
}

void LAN::ClientUpdatePlayerList()
{
}

void LAN::ProcessHostEvent(ENetEvent& event)
{
    switch (event.type)
    {
    case ENET_EVENT_TYPE_CONNECT:
        {
            if ((NumPlayers >= MaxPlayers) || (NumPlayers >= 16))
            {
                // game is full, reject connection
                enet_peer_disconnect(event.peer, 0);
                break;
            }

            // client connected; assign player number

            int id;
            for (id = 0; id < 16; id++)
            {
                if (id >= NumPlayers) break;
                if (Players[id].Status == Player_None) break;
            }

            if (id < 16)
            {
                u8 cmd[11];
                cmd[0] = Cmd_ClientInit;
                cmd[1] = (u8)kLANMagic;
                cmd[2] = (u8)(kLANMagic >> 8);
                cmd[3] = (u8)(kLANMagic >> 16);
                cmd[4] = (u8)(kLANMagic >> 24);
                cmd[5] = (u8)kProtocolVersion;
                cmd[6] = (u8)(kProtocolVersion >> 8);
                cmd[7] = (u8)(kProtocolVersion >> 16);
                cmd[8] = (u8)(kProtocolVersion >> 24);
                cmd[9] = (u8)id;
                cmd[10] = MaxPlayers;
                ENetPacket* pkt = enet_packet_create(cmd, 11, ENET_PACKET_FLAG_RELIABLE);
                enet_peer_send(event.peer, Chan_Cmd, pkt);

                Platform::Mutex_Lock(PlayersMutex);

                Players[id].ID = id;
                Players[id].Status = Player_Connecting;
                Players[id].Address = event.peer->address.host;
                event.peer->data = &Players[id];
                NumPlayers++;

                Platform::Mutex_Unlock(PlayersMutex);

                RemotePeers[id] = event.peer;
            }
            else
            {
                // ???
                enet_peer_disconnect(event.peer, 0);
            }
        }
        break;

    case ENET_EVENT_TYPE_DISCONNECT:
        {
            Player* player = (Player*)event.peer->data;
            if (!player) break;

            ConnectedBitmask &= ~(1 << player->ID);

            int id = player->ID;
            RemotePeers[id] = nullptr;

            player->ID = 0;
            player->Status = Player_None;
            NumPlayers--;

            // broadcast updated player list
            HostUpdatePlayerList();
        }
        break;

    case ENET_EVENT_TYPE_RECEIVE:
        {
            if (event.packet->dataLength < 1) break;

            u8* data = (u8*)event.packet->data;
            switch (data[0])
            {
            case Cmd_PlayerInfo: // client sending player info
                {
                    if (event.packet->dataLength != (9+sizeof(Player))) break;

                    u32 magic = data[1] | (data[2] << 8) | (data[3] << 16) | (data[4] << 24);
                    u32 version = data[5] | (data[6] << 8) | (data[7] << 16) | (data[8] << 24);
                    if ((magic != kLANMagic) || (version != kProtocolVersion))
                    {
                        enet_peer_disconnect(event.peer, 0);
                        break;
                    }

                    Player player;
                    memcpy(&player, &data[9], sizeof(Player));
                    player.Name[31] = '\0';

                    Player* hostside = (Player*)event.peer->data;
                    if (player.ID != hostside->ID)
                    {
                        enet_peer_disconnect(event.peer, 0);
                        break;
                    }

                    Platform::Mutex_Lock(PlayersMutex);

                    player.Status = Player_Client;
                    player.Address = event.peer->address.host;
                    memcpy(hostside, &player, sizeof(Player));

                    Platform::Mutex_Unlock(PlayersMutex);

                    // broadcast updated player list
                    HostUpdatePlayerList();
                }
                break;

            case Cmd_PlayerConnect: // player connected
                {
                    if (event.packet->dataLength != 1) break;
                    Player* player = (Player*)event.peer->data;
                    if (!player) break;

                    ConnectedBitmask |= (1 << player->ID);
                }
                break;

            case Cmd_PlayerDisconnect: // player disconnected
                {
                    if (event.packet->dataLength != 1) break;
                    Player* player = (Player*)event.peer->data;
                    if (!player) break;

                    ConnectedBitmask &= ~(1 << player->ID);
                }
                break;
            }

            enet_packet_destroy(event.packet);
        }
        break;
    case ENET_EVENT_TYPE_NONE:
        break;
    }
}

void LAN::ProcessClientEvent(ENetEvent& event)
{
    switch (event.type)
    {
    case ENET_EVENT_TYPE_CONNECT:
        {
            // another client is establishing a direct connection to us

            int playerid = -1;
            for (int i = 0; i < 16; i++)
            {
                Player* player = &Players[i];
                if (i == MyPlayer.ID) continue;
                if (player->Status != Player_Client) continue;

                if (player->Address == event.peer->address.host)
                {
                    playerid = i;
                    break;
                }
            }

            if (playerid < 0)
            {
                enet_peer_disconnect(event.peer, 0);
                break;
            }

            RemotePeers[playerid] = event.peer;
            event.peer->data = &Players[playerid];
        }
        break;

    case ENET_EVENT_TYPE_DISCONNECT:
        {
            Player* player = (Player*)event.peer->data;
            if (!player) break;

            ConnectedBitmask &= ~(1 << player->ID);

            int id = player->ID;
            RemotePeers[id] = nullptr;

            Platform::Mutex_Lock(PlayersMutex);
            player->Status = Player_Disconnected;
            Platform::Mutex_Unlock(PlayersMutex);

            ClientUpdatePlayerList();
        }
        break;

    case ENET_EVENT_TYPE_RECEIVE:
        {
            if (event.packet->dataLength < 1) break;

            u8* data = (u8*)event.packet->data;
            switch (data[0])
            {
            case Cmd_PlayerList: // host sending player list
                {
                    if (event.packet->dataLength != (2+sizeof(Players))) break;
                    if (data[1] > 16) break;

                    Platform::Mutex_Lock(PlayersMutex);

                    NumPlayers = data[1];
                    memcpy(Players, &data[2], sizeof(Players));
                    for (int i = 0; i < 16; i++)
                    {
                        Players[i].Name[31] = '\0';
                    }

                    Platform::Mutex_Unlock(PlayersMutex);

                    // establish connections to any new clients
                    for (int i = 0; i < 16; i++)
                    {
                        Player* player = &Players[i];
                        if (i == MyPlayer.ID) continue;
                        if (player->Status != Player_Client) continue;

                        if (!RemotePeers[i])
                        {
                            ENetAddress peeraddr;
                            peeraddr.host = player->Address;
                            peeraddr.port = kLANPort;
                            ENetPeer* peer = enet_host_connect(Host, &peeraddr, 2, 0);
                            if (!peer)
                            {
                                // TODO deal with this
                                continue;
                            }
                        }
                    }
                }
                break;

            case Cmd_PlayerConnect: // player connected
                {
                    if (event.packet->dataLength != 1) break;
                    Player* player = (Player*)event.peer->data;
                    if (!player) break;

                    ConnectedBitmask |= (1 << player->ID);
                }
                break;

            case Cmd_PlayerDisconnect: // player disconnected
                {
                    if (event.packet->dataLength != 1) break;
                    Player* player = (Player*)event.peer->data;
                    if (!player) break;

                    ConnectedBitmask &= ~(1 << player->ID);
                }
                break;

            case Cmd_StartSession: // host starting a Netplay session
                {
                    if (event.packet->dataLength != 3 && event.packet->dataLength != 4) break;
                    int server = event.packet->dataLength == 4 ? data[3] : 0;
                    StartRequest = (server << 16) | (data[1] << 8) | data[2];
                }
                break;
            }

            enet_packet_destroy(event.packet);
        }
        break;
    case ENET_EVENT_TYPE_NONE:
        break;
    }
}

void LAN::HostStartSession(u8 mode, u8 players, u8 server)
{
    if (!Active || !IsHost) return;
    u8 cmd[4] = {Cmd_StartSession, mode, players, server};
    ENetPacket* pkt = enet_packet_create(cmd, 4, ENET_PACKET_FLAG_RELIABLE);
    enet_host_broadcast(Host, Chan_Cmd, pkt);
    enet_host_flush(Host);
}

void LAN::ProcessEvent(ENetEvent& event)
{
    if (IsHost)
        ProcessHostEvent(event);
    else
        ProcessClientEvent(event);
}

// 0 = per-frame processing of events and eventual misc. frame
// 1 = checking if a misc. frame has arrived
// 2 = waiting for a MP frame
// 3 = checking if a MP frame has arrived (2 without blocking; LITEV_LAN_RX_SPIN)
void LAN::ProcessLAN(int type)
{
    if (!Host) return;

    u32 time_last = (u32)Platform::GetMSCount();

    // see if we have queued packets already, get rid of the stale ones
    // any incoming packet should be consumed by the core quickly, so if
    // they've been sitting in the queue for more than one frame's time,
    // we can assume they're stale
    while (!RXQueue.empty())
    {
        ENetPacket* enetpacket = RXQueue.front();
        MPPacketHeader* header = (MPPacketHeader*)&enetpacket->data[0];
        u32 packettime = header->Magic;

        if ((packettime > time_last) || (packettime < (time_last - 16)))
        {
            RXQueue.pop();
            enet_packet_destroy(enetpacket);
        }
        else
        {
            // we got a packet, depending on what the caller wants we might be able to return now
            if (type >= 2) return;
            if (type == 1)
            {
                // if looking for a misc. frame, we shouldn't be receiving a MP frame
                if (header->Type == 0)
                    return;

                RXQueue.pop();
                enet_packet_destroy(enetpacket);
            }

            break;
        }
    }

    // MPInterface::RecvTimeout (frontend-settable, default 25 ms): LAN used to hardcode 25 ms,
    // too short for Wi-Fi, where poll/reply round trips plus the peers' frame-pacing phase
    // offset (up to a frame) regularly exceed it and in-game joins fail.
    int timeout = (type == 2) ? RecvTimeout : 0;
    time_last = (u32)Platform::GetMSCount();

    ENetEvent event;
    while (enet_host_service(Host, &event, timeout) > 0)
    {
        if (event.type == ENET_EVENT_TYPE_RECEIVE && event.channelID == Chan_MP)
        {
            MPPacketHeader* header = (MPPacketHeader*)&event.packet->data[0];

            bool good = true;
            if (event.packet->dataLength < sizeof(MPPacketHeader))
                good = false;
            else if (header->Magic != 0x4946494E)
                good = false;
            else if (header->SenderID == MyPlayer.ID)
                good = false;

            if (!good)
            {
                enet_packet_destroy(event.packet);
            }
            else
            {
#ifdef LITEV_LAN_STATS
                {
                    u64 now = LanRealNs(), k = enet_litev_last_rx_ns;
                    if (k && now > k && now - k < 1000000000ull)
                    {
                        if (header->Type == 1)
                        {
                            LanStats.CmdWakeUs += (now - k) / 1000; LanStats.CmdWakeN++;
                            LanCmdKernelNs = k;
                        }
                        else if (IsHost && (header->Type & 0xFFFF) == 2)
                        {
                            LanStats.ReplyWakeUs += (now - k) / 1000; LanStats.ReplyWakeN++;
                            if (LanCmdSendNs && k > LanCmdSendNs) { LanStats.SpanUs += (k - LanCmdSendNs) / 1000; LanStats.SpanN++; }
                            LanCmdSendNs = 0;
                        }
                    }
                }
#endif
                // mark this packet with the time it was received
                header->Magic = (u32)Platform::GetMSCount();

                event.packet->userData = event.peer;
                RXQueue.push(event.packet);

                // return now -- if we are receiving MP frames, if we keep going
                // we'll consume too many even if we have no timeout set
                return;
            }
        }
        else
        {
            ProcessEvent(event);
        }

        if (type == 2)
        {
            u32 time = (u32)Platform::GetMSCount();
            if (time < time_last) return;
            timeout -= (int)(time - time_last);
            if (timeout <= 0) return;
            time_last = time;
        }
    }
}

void LAN::Process()
{
    if (!Active) return;

    ProcessDiscovery();
    ProcessLAN(0);

    FrameCount++;
    if (FrameCount >= 60)
    {
        FrameCount = 0;
#ifdef LITEV_LAN_STATS
        auto& st = LanStats;
        Platform::Log(Platform::LogLevel::Info,
            "LAN_STATS 60f: host waits=%u miss=%u %.1f ms (pos %u %u %u %u %u %u %u %u) | client waits=%u miss=%u %.1f ms (pos %u %u %u %u %u %u %u %u) | sent cmd=%u reply=%u pkt=%u\n",
            st.HostWaits, st.HostMisses, st.HostWaitUs / 1000.0,
            st.HostPos[0], st.HostPos[1], st.HostPos[2], st.HostPos[3], st.HostPos[4], st.HostPos[5], st.HostPos[6], st.HostPos[7],
            st.ClientWaits, st.ClientMisses, st.ClientWaitUs / 1000.0,
            st.ClientPos[0], st.ClientPos[1], st.ClientPos[2], st.ClientPos[3], st.ClientPos[4], st.ClientPos[5], st.ClientPos[6], st.ClientPos[7],
            st.Cmds, st.Replies, st.Packets);
        Platform::Log(Platform::LogLevel::Info,
            "LAN_STAGES 60f: host cmd-send->reply-kernel=%.0f us (n=%u) reply kernel->app=%.0f us | client cmd kernel->app=%.0f us (n=%u) cmd kernel->reply-send=%.0f us\n",
            st.SpanN ? (double)st.SpanUs / st.SpanN : 0.0, st.SpanN, st.ReplyWakeN ? (double)st.ReplyWakeUs / st.ReplyWakeN : 0.0,
            st.CmdWakeN ? (double)st.CmdWakeUs / st.CmdWakeN : 0.0, st.CmdWakeN, st.TurnN ? (double)st.TurnUs / st.TurnN : 0.0);
        st = {};
#endif

        Platform::Mutex_Lock(PlayersMutex);

        for (int i = 0; i < 16; i++)
        {
            if (Players[i].Status == Player_None) continue;
            if (i == MyPlayer.ID) continue;
            if (!RemotePeers[i]) continue;

            Players[i].Ping = RemotePeers[i]->roundTripTime;
        }

        Platform::Mutex_Unlock(PlayersMutex);
    }
}


void LAN::Begin(int inst)
{
    if (!Host) return;

    ConnectedBitmask |= (1 << MyPlayer.ID);
    LastHostID = -1;
    LastHostPeer = nullptr;

    u8 cmd = Cmd_PlayerConnect;
    ENetPacket* pkt = enet_packet_create(&cmd, 1, ENET_PACKET_FLAG_RELIABLE);
    enet_host_broadcast(Host, Chan_Cmd, pkt);
}

void LAN::End(int inst)
{
    if (!Host) return;

    ConnectedBitmask &= ~(1 << MyPlayer.ID);

    u8 cmd = Cmd_PlayerDisconnect;
    ENetPacket* pkt = enet_packet_create(&cmd, 1, ENET_PACKET_FLAG_RELIABLE);
    enet_host_broadcast(Host, Chan_Cmd, pkt);
}


int LAN::SendPacketGeneric(u32 type, u8* packet, int len, u64 timestamp)
{
    if (!Host) return 0;

    // TODO make the reliable part optional?
    //u32 flags = ENET_PACKET_FLAG_RELIABLE;
    u32 flags = ENET_PACKET_FLAG_UNSEQUENCED;

    ENetPacket* enetpacket = enet_packet_create(nullptr, sizeof(MPPacketHeader)+len, flags);

    MPPacketHeader pktheader;
    pktheader.Magic = 0x4946494E;
    pktheader.SenderID = MyPlayer.ID;
    pktheader.Type = type;
    pktheader.Length = len;
    pktheader.Timestamp = timestamp;
    memcpy(&enetpacket->data[0], &pktheader, sizeof(MPPacketHeader));
    if (len)
        memcpy(&enetpacket->data[sizeof(MPPacketHeader)], packet, len);

    if (((type & 0xFFFF) == 2) && LastHostPeer)
        enet_peer_send(LastHostPeer, Chan_MP, enetpacket);
    else
        enet_host_broadcast(Host, Chan_MP, enetpacket);
    enet_host_flush(Host);

    return len;
}

int LAN::RecvPacketGeneric(u8* packet, bool block, u64* timestamp)
{
    if (!Host) return 0;

    ProcessLAN(block ? 2 : 1);
    if (RXQueue.empty()) return 0;

    ENetPacket* enetpacket = RXQueue.front();
    RXQueue.pop();
    MPPacketHeader* header = (MPPacketHeader*)&enetpacket->data[0];

    u32 len = header->Length;
    if (len)
    {
        if (len > 2048) len = 2048;

        memcpy(packet, &enetpacket->data[sizeof(MPPacketHeader)], len);

        if (header->Type == 1)
        {
            LastHostID = header->SenderID;
            LastHostPeer = (ENetPeer*)enetpacket->userData;
        }
    }

    if (timestamp) *timestamp = header->Timestamp;
    enet_packet_destroy(enetpacket);
    return len;
}


int LAN::SendPacket(int inst, u8* packet, int len, u64 timestamp)
{
#ifdef LITEV_LAN_STATS
    LanStats.Packets++;
#endif
    return SendPacketGeneric(0, packet, len, timestamp);
}

int LAN::RecvPacket(int inst, u8* packet, u64* timestamp)
{
    return RecvPacketGeneric(packet, false, timestamp);
}


int LAN::SendCmd(int inst, u8* packet, int len, u64 timestamp)
{
#ifdef LITEV_LAN_STATS
    LanStats.Cmds++;
    LanCmdSendNs = LanRealNs();
#endif
    return SendPacketGeneric(1, packet, len, timestamp);
}

int LAN::SendReply(int inst, u8* packet, int len, u64 timestamp, u16 aid)
{
#ifdef LITEV_LAN_STATS
    LanStats.Replies++;
    if (LanCmdKernelNs)
    {
        LanStats.TurnUs += (LanRealNs() - LanCmdKernelNs) / 1000; LanStats.TurnN++;
        LanCmdKernelNs = 0;
    }
#endif
    return SendPacketGeneric(2 | (aid<<16), packet, len, timestamp);
}

int LAN::SendAck(int inst, u8* packet, int len, u64 timestamp)
{
    return SendPacketGeneric(3, packet, len, timestamp);
}

#ifdef LITEV_LAN_RX_SPIN
// LAN latency: a client blocked at its sync point sleeps in poll() (inside enet_host_service) until
// the host's CMD arrives, and waking a sleeping core costs a scheduler/cpuidle round trip. Polling
// the socket without blocking for a short while first keeps the thread awake while the frame is
// expected; after that it falls back to the normal blocking wait, so a long wait does not burn CPU.
// Only the client's host-frame wait spins (the host's reply wait does not). Spin length in us:
// prop debug.litev.lanspin (Android) / env LITEV_LAN_RX_SPIN_US, read once; default 2000, 0 = off.
static int LanSpinUs()
{
    const char* v = nullptr;
#ifdef __ANDROID__
    static char b[PROP_VALUE_MAX];
    if (__system_property_get("debug.litev.lanspin", b) > 0) v = b;
#else
    v = getenv("LITEV_LAN_RX_SPIN_US");
#endif
    return v ? atoi(v) : 2000;
}
#endif

int LAN::RecvHostPacket(int inst, u8* packet, u64* timestamp)
{
    if (LastHostID != -1)
    {
        // check if the host is still connected

        if (!(ConnectedBitmask & (1<<LastHostID)))
            return -1;
    }

#ifdef LITEV_LAN_RX_SPIN
    static const int spinUs = LanSpinUs();
    if (spinUs > 0 && RXQueue.empty())
    {
        auto end = std::chrono::steady_clock::now() + std::chrono::microseconds(spinUs);
        do
        {
            // poll(0) on the socket; ENet itself only runs once a datagram is there
            enet_uint32 cond = ENET_SOCKET_WAIT_RECEIVE;
            if (enet_socket_wait(Host->socket, &cond, 0) == 0 && (cond & ENET_SOCKET_WAIT_RECEIVE))
                ProcessLAN(3);
        }
        while (RXQueue.empty() && std::chrono::steady_clock::now() < end);
    }
#endif

#ifdef LITEV_LAN_STATS
    u64 t0 = LanNowUs();
    int r = RecvPacketGeneric(packet, true, timestamp);
    LanStats.ClientWaitUs += LanNowUs() - t0;
    LanStats.ClientWaits++;
    if (r <= 0) LanStats.ClientMisses++;
    else if (timestamp) LanStats.ClientPos[LanFramePos(*timestamp)]++;
    return r;
#else
    return RecvPacketGeneric(packet, true, timestamp);
#endif
}

u16 LAN::RecvReplies(int inst, u8* packets, u64 timestamp, u16 aidmask)
{
    if (!Host) return 0;
#ifdef LITEV_LAN_STATS
    struct Timer
    {
        u64 t0 = LanNowUs(); u64 ts; u16 mask; u16* ret;
        ~Timer()
        {
            LanStats.HostWaitUs += LanNowUs() - t0;
            LanStats.HostWaits++;
            LanStats.HostPos[LanFramePos(ts)]++;
            if ((*ret & mask) != mask) LanStats.HostMisses++;
        }
    };
#endif

    u16 ret = 0;
#ifdef LITEV_LAN_STATS
    Timer timer{LanNowUs(), timestamp, aidmask, &ret};
#endif
    u16 myinstmask = 1 << MyPlayer.ID;

    if ((myinstmask & ConnectedBitmask) == ConnectedBitmask)
        return 0;

    for (;;)
    {
        ProcessLAN(2);
        if (RXQueue.empty())
        {
            // no more replies available
            return ret;
        }

        ENetPacket* enetpacket = RXQueue.front();
        RXQueue.pop();
        MPPacketHeader* header = (MPPacketHeader*)&enetpacket->data[0];
        bool good = true;
        if ((header->Type & 0xFFFF) != 2)
            good = false;
        else if (header->Timestamp < (timestamp - 32))
            good = false;

        if (good)
        {
            u32 len = header->Length;
            if (len)
            {
                if (len > 1024) len = 1024;

                u32 aid = header->Type >> 16;
                memcpy(&packets[(aid-1)*1024], &enetpacket->data[sizeof(MPPacketHeader)], len);

                ret |= (1<<aid);
            }

            myinstmask |= (1<<header->SenderID);
            if (((myinstmask & ConnectedBitmask) == ConnectedBitmask) ||
                ((ret & aidmask) == aidmask))
            {
                // all the clients have sent their reply
                enet_packet_destroy(enetpacket);
                return ret;
            }
        }

        enet_packet_destroy(enetpacket);
    }
}

}

