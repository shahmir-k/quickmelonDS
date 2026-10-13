#include "AmbientSlowdown.h"

#ifdef NDEBUG
#undef NDEBUG
#endif
#include <cassert>
#include <cstdio>

using melonDS::AmbientSlowdown;
using melonDS::u32;
using melonDS::u64;

static AmbientSlowdown::Sample Run(AmbientSlowdown& detector, u64& now, int frames,
                                   u32 frameUs, u32 waitUs, u32 exchanges)
{
    AmbientSlowdown::Sample sample;
    for (int i = 0; i < frames; i++)
    {
        now += frameUs;
        sample = detector.Observe(now, true, waitUs, exchanges);
    }
    return sample;
}

int main()
{
    u64 now = 1000000;
    AmbientSlowdown detector;
    assert(!Run(detector, now, 420, 16667, 5000, 1).Pending); // fast LAN is not slow
    assert(!Run(detector, now, 260, 25000, 0, 1).Pending);    // CPU-limited, not network-limited
    assert(!Run(detector, now, 260, 25000, 5000, 0).Pending); // no game exchanges
    assert(!Run(detector, now, 196, 25000, 5000, 3).Pending); // under five sustained seconds
    auto slow = Run(detector, now, 44, 25000, 5000, 3);
    assert(slow.Pending && slow.FpsX100 < 5800);
    assert(slow.WaitUsPerFrame > 3000 && slow.ExchangesPerFrameX100 > 0);
    detector.Dismiss();
    assert(!Run(detector, now, 300, 25000, 5000, 3).Pending); // at most once per link session

    detector.Observe(now + 25000, false, 0, 0);              // link closed: new session
    now += 25000;
    assert(Run(detector, now, 240, 25000, 5000, 3).Pending);

    detector.Observe(now + 25000, false, 0, 0);
    now += 25000;
    Run(detector, now, 120, 25000, 5000, 3);
    now += 500000;                                             // pause/background must break the streak
    detector.Observe(now, true, 5000, 3);
    assert(!Run(detector, now, 120, 25000, 5000, 3).Pending);
    std::puts("ambient-slowdown-test: PASS");
}
