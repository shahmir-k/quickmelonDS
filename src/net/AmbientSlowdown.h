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
        // that gap into an apparent low-FPS LAN interval. Network wait itself can
        // exceed 250 ms, so only treat an unexplained frame gap as a pause.
        if (!LinkedSinceUs) LinkedSinceUs = nowUs;
        if (LastFrameUs && (nowUs <= LastFrameUs ||
            (nowUs - LastFrameUs > 250000 && blockedUs < (nowUs - LastFrameUs) / 2)))
        {
            RestartWindow(nowUs);
            LinkedSinceUs = nowUs;
        }
        LastFrameUs = nowUs;
        if (!WindowStartUs) WindowStartUs = nowUs;
        const bool inStartupGrace = nowUs - LinkedSinceUs < 10000000;
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
            RecoverySinceUs = 0;
            // Judge sustained slowdown by complete windows, not frame spacing:
            // at 5-8 FPS a valid network-waiting frame can take >100 ms.
            if (inStartupGrace) SlowSinceUs = 0;
            else if (!SlowSinceUs) SlowSinceUs = nowUs;
            else if (!Dismissed && nowUs - SlowSinceUs >= 5000000)
                Last.Pending = true;
        }
        else
        {
            SlowSinceUs = 0;
            if (!RecoverySinceUs) RecoverySinceUs = nowUs;
            // Pending describes an offer not yet accepted by the frontend. A
            // recovered link must not keep surfacing that stale offer forever.
            if (Last.Pending && nowUs - RecoverySinceUs >= 2000000)
                Last.Pending = false;
        }

        WindowStartUs = nowUs;
        Frames = 0;
        BlockedUs = 0;
        Exchanges = 0;
        return Last;
    }

    void Dismiss() { Last.Pending = false; Dismissed = true; }
    Sample Current() const { return Last; }

private:
    void RestartWindow(u64 nowUs)
    {
        WindowStartUs = nowUs;
        SlowSinceUs = 0;
        RecoverySinceUs = 0;
        Frames = 0;
        BlockedUs = 0;
        Exchanges = 0;
    }

    u64 LastFrameUs = 0, WindowStartUs = 0, LinkedSinceUs = 0;
    u64 SlowSinceUs = 0, RecoverySinceUs = 0;
    u64 BlockedUs = 0, Exchanges = 0;
    u32 Frames = 0;
    bool Dismissed = false;
    Sample Last;
};

}
