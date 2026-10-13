#pragma once

#include "types.h"

namespace melonDS
{

// Shared, frontend-independent decision for offering a LAN-to-Netplay handoff.
// Call once per emulated frame, after the preceding frame's MP wait has been counted.
class AmbientSlowdown
{
public:
    struct Sample
    {
        u32 FpsX100 = 0;
        u32 WaitUsPerFrame = 0;
        u32 ExchangesPerFrameX100 = 0;
        bool Pending = false;
    };

    Sample Observe(u64 nowUs, bool linked, u32 blockedUs, u32 exchanges)
    {
        if (!linked)
        {
            *this = AmbientSlowdown();
            return Last;
        }

        // A paused/backgrounded emulator has no measurable play rate. Do not turn
        // that gap into an apparent low-FPS LAN interval or offer a second prompt.
        if (LastFrameUs && (nowUs <= LastFrameUs || nowUs - LastFrameUs > 250000))
            RestartWindow(nowUs);
        LastFrameUs = nowUs;
        if (!WindowStartUs) WindowStartUs = nowUs;
        if (blockedUs > 3000 && exchanges)
        {
            if (!SlowSinceUs || nowUs - LastCandidateUs > 100000) SlowSinceUs = nowUs;
            LastCandidateUs = nowUs;
        }
        else if (SlowSinceUs && nowUs - LastCandidateUs > 100000) SlowSinceUs = 0;
        Frames++;
        BlockedUs += blockedUs;
        Exchanges += exchanges;

        const u64 span = nowUs - WindowStartUs;
        if (span < 1000000) return Last;
        Last.FpsX100 = (u32)((u64)Frames * 100000000ull / span);
        Last.WaitUsPerFrame = (u32)(BlockedUs / Frames);
        Last.ExchangesPerFrameX100 = (u32)((u64)Exchanges * 100 / Frames);

        const bool networkSlow = Last.FpsX100 < 5800 && Last.WaitUsPerFrame > 3000 && Exchanges > 0;
        if (networkSlow)
        {
            if (SlowSinceUs && !Offered && nowUs - SlowSinceUs >= 5000000)
            {
                Offered = true;
                Last.Pending = true;
            }
        }
        else SlowSinceUs = 0;

        WindowStartUs = nowUs;
        Frames = 0;
        BlockedUs = 0;
        Exchanges = 0;
        return Last;
    }

    void Dismiss() { Last.Pending = false; }
    Sample Current() const { return Last; }

private:
    void RestartWindow(u64 nowUs)
    {
        WindowStartUs = nowUs;
        SlowSinceUs = 0;
        LastCandidateUs = 0;
        Frames = 0;
        BlockedUs = 0;
        Exchanges = 0;
    }

    u64 LastFrameUs = 0, WindowStartUs = 0, SlowSinceUs = 0, LastCandidateUs = 0;
    u64 BlockedUs = 0, Exchanges = 0;
    u32 Frames = 0;
    bool Offered = false;
    Sample Last;
};

}
