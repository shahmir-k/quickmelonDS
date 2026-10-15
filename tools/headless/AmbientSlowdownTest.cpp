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
                                   u32 frameUs, u32 waitUs, u32 exchanges, u32 targetFpsX100 = 6000)
{
    AmbientSlowdown::Sample sample;
    for (int i = 0; i < frames; i++)
    {
        now += frameUs;
        sample = detector.Observe(now, true, waitUs, exchanges, targetFpsX100);
    }
    return sample;
}

int main()
{
    u64 now = 1000000;
    AmbientSlowdown detector;
    assert(!Run(detector, now, 360, 25000, 5000, 3).Pending); // slow link startup: grace period
    assert(!Run(detector, now, 200, 25000, 5000, 3).Pending); // first five seconds after grace
    assert(Run(detector, now, 80, 25000, 5000, 3).Pending);  // sustained slowdown after grace
    assert(Run(detector, now, 60, 16667, 500, 3).Pending);   // brief recovery is insufficient
    assert(!Run(detector, now, 180, 16667, 500, 3).Pending); // sustained recovery clears offer
    assert(!Run(detector, now, 180, 25000, 5000, 3).Pending); // new slowdown needs five seconds
    assert(Run(detector, now, 100, 25000, 5000, 3).Pending);  // it can offer again if not dismissed
    detector.Dismiss();
    assert(!Run(detector, now, 300, 25000, 5000, 3).Pending); // dismiss once per link session

    detector.Observe(now + 25000, false, 0, 0);
    now += 25000;
    assert(!Run(detector, now, 80, 125000, 100000, 3).Pending); // ten-second startup at 8 FPS
    assert(Run(detector, now, 48, 125000, 100000, 3).Pending); // 125-ms frames must preserve streak
    detector.Observe(now + 25000, false, 0, 0);
    now += 25000;
    assert(!Run(detector, now, 30, 350000, 300000, 3).Pending); // a network-waiting 350-ms frame is not a pause
    assert(Run(detector, now, 20, 350000, 300000, 3).Pending);
    detector.Observe(now + 25000, false, 0, 0);
    now += 25000;
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
    assert(!Run(detector, now, 400, 25000, 5000, 3).Pending); // grace excludes the initial ten seconds
    assert(Run(detector, now, 240, 25000, 5000, 3).Pending);

    detector.Observe(now + 25000, false, 0, 0);
    now += 25000;
    Run(detector, now, 560, 25000, 5000, 3);
    now += 500000;                                             // pause/background must break the streak
    detector.Observe(now, true, 5000, 3);
    assert(!Run(detector, now, 120, 25000, 5000, 3).Pending);

    // the session's fast-forward: judged against its speed, not 60 fps
    detector.Observe(now + 25000, false, 0, 0);
    now += 25000;
    assert(!Run(detector, now, 1200, 33333, 5000, 3, 3000).Pending);  // 0.5x: 30 fps at its speed is not slow
    assert(!Run(detector, now, 1200, 5000, 4000, 3, 0).Pending);      // uncapped at 200 fps
    assert(!Run(detector, now, 1200, 16667, 5000, 3, 12000).Pending); // 2x missing its target, still 60 fps: not slow
    assert(Run(detector, now, 1200, 25000, 5000, 3, 12000).Pending);  // 2x below 1x with network waits: slow
    std::puts("ambient-slowdown-test: PASS");
}
