#pragma once
namespace melonDS
{
// Set by the lockstep links (Netplay, Hosted Netplay), which deliver regular frames kDelay late:
// an MP client then takes them only between the host's ACK and its next CMD (Wifi::USTick).
inline bool LinkDelaysRegularFrames = false;
}
