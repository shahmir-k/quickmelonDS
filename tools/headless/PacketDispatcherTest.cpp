// Live handoff must relabel the visible console without losing pending packets.
#include "PacketDispatcher.h"
#include "Platform.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>
#include <mutex>

namespace melonDS::Platform
{
struct Mutex { std::mutex Value; };
Mutex* Mutex_Create() { return new Mutex(); }
void Mutex_Free(Mutex* mutex) { delete mutex; }
void Mutex_Lock(Mutex* mutex) { mutex->Value.lock(); }
void Mutex_Unlock(Mutex* mutex) { mutex->Value.unlock(); }
}

int main()
{
    PacketDispatcher packets;
    packets.registerInstance(0);
    const unsigned char pending = 0xA5;
    packets.sendPacket(nullptr, 0, &pending, 1, 16, 1u << 0);

    assert(!packets.reassignInstance(0, 16));
    packets.registerInstance(1);
    assert(!packets.reassignInstance(0, 1)); // never replace another console
    packets.unregisterInstance(1);
    assert(packets.reassignInstance(0, 1));
    assert(packets.reassignInstance(1, 1));
    assert(!packets.reassignInstance(0, 1));

    unsigned char received = 0;
    int length = 0;
    assert(!packets.recvPacket(nullptr, nullptr, &received, &length, 0));
    assert(packets.recvPacket(nullptr, nullptr, &received, &length, 1));
    assert(length == 1 && received == pending);

    const unsigned char next = 0x5A;
    packets.sendPacket(nullptr, 0, &next, 1, 16, 1u << 1);
    assert(packets.recvPacket(nullptr, nullptr, &received, &length, 1));
    assert(length == 1 && received == next);
    packets.unregisterInstance(1);
    assert(!packets.recvPacket(nullptr, nullptr, &received, &length, 1));
    puts("packet-dispatcher-test: PASS");
}
