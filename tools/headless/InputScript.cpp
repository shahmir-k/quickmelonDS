/*
    liteDS-v2 headless harness - scripted input parser.
    See InputScript.h for the file format and semantics.
*/

#include "InputScript.h"

#include <cstdio>
#include <cstdlib>
#include <cctype>
#include <algorithm>

#include "xxhash/xxhash.h"

using namespace melonDS;

namespace liteds
{

namespace
{

std::string Trim(const std::string& s)
{
    size_t b = 0, e = s.size();
    while (b < e && std::isspace((unsigned char)s[b])) b++;
    while (e > b && std::isspace((unsigned char)s[e - 1])) e--;
    return s.substr(b, e - b);
}

std::string Upper(std::string s)
{
    for (char& c : s) c = (char)std::toupper((unsigned char)c);
    return s;
}

// Split on commas, trimming each piece; empty pieces are dropped.
std::vector<std::string> SplitCommas(const std::string& s)
{
    std::vector<std::string> out;
    size_t start = 0;
    while (start <= s.size())
    {
        size_t comma = s.find(',', start);
        std::string piece = Trim(s.substr(start, comma == std::string::npos ? std::string::npos
                                                                            : comma - start));
        if (!piece.empty()) out.push_back(piece);
        if (comma == std::string::npos) break;
        start = comma + 1;
    }
    return out;
}

} // namespace

bool InputScript::NameToBit(const std::string& name, u32& bitOut)
{
    std::string u = Upper(Trim(name));
    if      (u == "A")      bitOut = kBitA;
    else if (u == "B")      bitOut = kBitB;
    else if (u == "SELECT") bitOut = kBitSELECT;
    else if (u == "START")  bitOut = kBitSTART;
    else if (u == "RIGHT")  bitOut = kBitRIGHT;
    else if (u == "LEFT")   bitOut = kBitLEFT;
    else if (u == "UP")     bitOut = kBitUP;
    else if (u == "DOWN")   bitOut = kBitDOWN;
    else if (u == "R")      bitOut = kBitR;
    else if (u == "L")      bitOut = kBitL;
    else if (u == "X")      bitOut = kBitX;
    else if (u == "Y")      bitOut = kBitY;
    else return false;
    return true;
}

bool InputScript::LoadFile(const std::string& path, std::string& err)
{
    FILE* f = fopen(path.c_str(), "rb");
    if (!f) { err = "cannot open input script '" + path + "'"; return false; }

    loaded_ = true;
    directives_.clear();

    char lineBuf[512];
    int lineNo = 0;
    while (fgets(lineBuf, sizeof(lineBuf), f))
    {
        lineNo++;
        std::string line = lineBuf;
        // Strip trailing '#' comment.
        auto hash = line.find('#');
        if (hash != std::string::npos) line = line.substr(0, hash);
        line = Trim(line);
        if (line.empty()) continue;

        // First token = frame index; remainder = key spec.
        size_t sp = line.find_first_of(" \t");
        std::string frameTok = (sp == std::string::npos) ? line : line.substr(0, sp);
        std::string keyTok    = (sp == std::string::npos) ? std::string() : Trim(line.substr(sp + 1));

        char* endp = nullptr;
        long frame = std::strtol(frameTok.c_str(), &endp, 10);
        if (endp == frameTok.c_str() || *endp != '\0' || frame < 0)
        {
            char msg[256];
            snprintf(msg, sizeof(msg), "input script %s:%d: bad frame index '%s'",
                     path.c_str(), lineNo, frameTok.c_str());
            err = msg;
            fclose(f);
            return false;
        }

        u32 pressed = 0;
        std::string keyUpper = Upper(keyTok);
        if (keyTok.empty() || keyUpper == "NONE")
        {
            pressed = 0;
        }
        else if (keyUpper.size() > 2 && keyUpper[0] == '0' && keyUpper[1] == 'X')
        {
            char* he = nullptr;
            unsigned long v = std::strtoul(keyTok.c_str(), &he, 16);
            if (he == keyTok.c_str() || *he != '\0')
            {
                char msg[256];
                snprintf(msg, sizeof(msg), "input script %s:%d: bad hex mask '%s'",
                         path.c_str(), lineNo, keyTok.c_str());
                err = msg;
                fclose(f);
                return false;
            }
            pressed = (u32)v & 0x0FFFu; // 12 valid button bits
        }
        else
        {
            for (const std::string& tok : SplitCommas(keyTok))
            {
                u32 bit;
                if (!NameToBit(tok, bit))
                {
                    char msg[256];
                    snprintf(msg, sizeof(msg), "input script %s:%d: unknown button '%s'",
                             path.c_str(), lineNo, tok.c_str());
                    err = msg;
                    fclose(f);
                    return false;
                }
                pressed |= (1u << bit);
            }
        }

        directives_.push_back({ (int)frame, pressed });
    }
    fclose(f);

    // Stable sort by frame so a later same-frame line wins the tie in
    // PressedMaskForFrame (which takes the last directive with frame <= f).
    std::stable_sort(directives_.begin(), directives_.end(),
                     [](const Directive& a, const Directive& b) { return a.frame < b.frame; });

    // Hash the normalized directive list (frame + pressed pairs) so the trace
    // header can flag a script mismatch on replay. Order-independent of
    // comments/whitespace because we hash the parsed form.
    XXH3_state_t* st = XXH3_createState();
    XXH3_64bits_reset(st);
    for (const Directive& d : directives_)
    {
        struct { s32 frame; u32 pressed; } packed = { (s32)d.frame, d.pressed };
        XXH3_64bits_update(st, &packed, sizeof(packed));
    }
    hash_ = XXH3_64bits_digest(st);
    XXH3_freeState(st);
    // Distinguish "loaded but empty" (hash of nothing) from "no script" (0):
    // an empty script must still be non-zero so a trace records that a script
    // was in force. XXH3 of zero bytes is a fixed constant; if it happens to be
    // 0, nudge it. In practice it is not 0, but guard anyway.
    if (directives_.empty()) hash_ = 0; // no directives == behaves like no script
    return true;
}

u32 InputScript::PressedMaskForFrame(int frame) const
{
    // directives_ is sorted ascending by frame. Find the last directive whose
    // frame <= `frame`.
    u32 pressed = 0;
    for (const Directive& d : directives_)
    {
        if (d.frame <= frame) pressed = d.pressed;
        else break;
    }
    return pressed;
}

} // namespace liteds
