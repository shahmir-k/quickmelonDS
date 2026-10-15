#pragma once
namespace melonDS
{
// Set by the lockstep links (Netplay, Hosted Netplay), which deliver regular frames kDelay late:
// an MP client then takes them only between the host's ACK and its next CMD (Wifi::USTick).
inline bool LinkDelaysRegularFrames = false;
// The Wi-Fi chip's channel (the console that switched last; 0 = none). AmbientLAN (one console per process) synthesizes
// a nearby device's beacon when the chip listens on that beacon's channel (the chip drops other channels' frames).
inline unsigned char WifiChannel = 0;
// The chip's microsecond clock (USTimestamp) at its last receive poll: AmbientLAN paces synthesized beacons in emulated
// time (fast-forward / slow motion keep the beacons per emulated second).
inline unsigned long long WifiUs = 0;
}
