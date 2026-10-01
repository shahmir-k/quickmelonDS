/*
    liteDS-v2 headless harness - local-multiplayer wiring.

    Provides MPInterface::Current / Set() for the headless binary WITHOUT the
    LAN/Netplay backends (which drag in ENet/Slirp). Only MPInterface_Local is
    supported; anything else falls back to a no-op dummy. This is compiled
    directly into liteDS-headless instead of linking the `net-utils` library, so
    two in-process NDS instances get local wireless with zero heavy deps.

    LocalMP itself only needs Platform mutex/semaphore (already in core).
*/

#include "MPInterface.h"
#include "LocalMP.h"

namespace melonDS
{
namespace
{
class HeadlessDummyMP : public MPInterface
{
public:
    void Process() override {}
    void Begin(int) override {}
    void End(int) override {}
    int SendPacket(int, u8*, int, u64) override { return 0; }
    int RecvPacket(int, u8*, u64*) override { return 0; }
    int SendCmd(int, u8*, int, u64) override { return 0; }
    int SendReply(int, u8*, int, u64, u16) override { return 0; }
    int SendAck(int, u8*, int, u64) override { return 0; }
    int RecvHostPacket(int, u8*, u64*) override { return 0; }
    u16 RecvReplies(int, u8*, u64, u16) override { return 0; }
};
}

std::unique_ptr<MPInterface> MPInterface::Current(std::make_unique<HeadlessDummyMP>());
MPInterfaceType MPInterface::CurrentType = MPInterface_Dummy;

void MPInterface::Set(MPInterfaceType type)
{
    if (type == MPInterface_Local)
        Current = std::make_unique<LocalMP>();
    else
        Current = std::make_unique<HeadlessDummyMP>();
    CurrentType = type;
}

}
