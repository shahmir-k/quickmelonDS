/*
    liteDS-v2 headless harness - scripted input (--input-script).

    The headless runner normally feeds NO input, so menu-driven test ROMs
    (e.g. ARMWrestler, or advancing past a game's title screen) cannot be
    driven. This module parses a tiny text "input script" that describes which
    DS buttons are held on each frame, and hands the frame loop an active-low
    key mask suitable for NDS::SetKeyMask.

    -----------------------------------------------------------------------
    Format (one directive per line):

        <frame> <keys>

      <frame>  a non-negative decimal frame index (the frame the RunFrame loop
               is about to execute, 0-based).
      <keys>   either
                 - a comma-separated list of button names from
                   {A,B,SELECT,START,RIGHT,LEFT,UP,DOWN,R,L,X,Y}
                   (case-insensitive), or
                 - a hex mask "0x..." of PRESSED buttons using the bit layout
                   below (bit set = pressed), or
                 - the word "NONE" / an empty key field = release all buttons.

      Semantics: "from this frame onward, hold EXACTLY these keys" (level, not
      edge). The most recent directive with frame <= current frame wins; keys
      not listed are released. Directives need not be sorted; ties on the same
      frame: the later line wins.

      Blank lines and lines beginning with '#' are ignored. Trailing '#'
      comments are allowed.

    Button bit layout (matches NDS::SetKeyMask's expectations, active-HIGH here;
    we invert to active-low for the hardware):
        0:A 1:B 2:SELECT 3:START 4:RIGHT 5:LEFT 6:UP 7:DOWN 8:R 9:L 10:X 11:Y

    Example (press START on frame 90, release on frame 120):
        # advance past the title screen
        90  START
        120 NONE
    -----------------------------------------------------------------------

    Determinism: scripted input changes emulation, so a golden trace recorded
    with a script is only reproducible when replayed with the same script. The
    parsed script exposes a 64-bit xxhash of its normalized directive list,
    which the trace header carries as a sanity field (0 == "no script").
*/

#pragma once

#include <string>
#include <vector>
#include "types.h"

namespace liteds
{

// Active-HIGH pressed-mask bit positions for the 12 DS face/shoulder buttons.
enum InputBit : melonDS::u32
{
    kBitA      = 0,
    kBitB      = 1,
    kBitSELECT = 2,
    kBitSTART  = 3,
    kBitRIGHT  = 4,
    kBitLEFT   = 5,
    kBitUP     = 6,
    kBitDOWN   = 7,
    kBitR      = 8,
    kBitL      = 9,
    kBitX      = 10,
    kBitY      = 11,
};

// Active-low "all released" mask, matching the harness's historical
// SetKeyMask(0xFFFF) call.
constexpr melonDS::u32 kKeyMaskNone = 0xFFFF;

class InputScript
{
public:
    InputScript() = default;

    // Parse an input-script file. On failure, returns false and fills `err`.
    // An empty/whitespace-only file is valid (no directives => always release).
    bool LoadFile(const std::string& path, std::string& err);

    // True if a script was actually loaded (at least attempted). A default
    // InputScript (never loaded) reports false and always yields kKeyMaskNone.
    bool Loaded() const { return loaded_; }

    // Active-HIGH pressed mask in effect at `frame` (bit set == pressed).
    melonDS::u32 PressedMaskForFrame(int frame) const;

    // Active-low mask ready for NDS::SetKeyMask at `frame`
    // (== (~PressedMaskForFrame) & 0xFFFF).
    melonDS::u32 KeyMaskForFrame(int frame) const
    {
        return (~PressedMaskForFrame(frame)) & 0xFFFFu;
    }

    // xxhash of the normalized directive list; 0 when no script is loaded.
    // Used as the trace-header sanity field so a script-driven golden trace
    // can flag replay under a different (or missing) script.
    melonDS::u64 Hash() const { return hash_; }

    // Number of parsed directives (diagnostics).
    size_t DirectiveCount() const { return directives_.size(); }

    // Map a single button name (case-insensitive) to its pressed-mask bit,
    // or return false if unknown.
    static bool NameToBit(const std::string& name, melonDS::u32& bitOut);

private:
    struct Directive { int frame; melonDS::u32 pressed; };

    bool loaded_ = false;
    melonDS::u64 hash_ = 0;
    // Sorted ascending by frame after LoadFile().
    std::vector<Directive> directives_;
};

} // namespace liteds
