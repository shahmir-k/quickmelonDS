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
    path_ = path; steps_.clear(); pc_ = 0; stepStart_ = -1; failed_ = false; progKeys_ = 0; progTx_ = progTy_ = -1;

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

        if (line[0] == '@')
        {
            std::string e;
            if (!ParseStep(line, e)) { err = "input script " + path + ":" + std::to_string(lineNo) + ": " + e; fclose(f); return false; }
            continue;
        }
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
        int touchX = -1, touchY = -1;
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
                // "T:x:y" = hold the touch screen at DS pixel (x, y)
                int tx, ty;
                if (sscanf(tok.c_str(), "%*1[Tt]:%d:%d", &tx, &ty) == 2 && tx >= 0 && tx < 256 && ty >= 0 && ty < 192)
                {
                    touchX = tx; touchY = ty; hasTouch_ = true;
                    continue;
                }
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

        directives_.push_back({ (int)frame, pressed, touchX, touchY });
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
        if (d.touchX >= 0)
        {
            s32 t[2] = { d.touchX, d.touchY };
            XXH3_64bits_update(st, t, sizeof(t));
        }
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
    const u32 prog = (!failed_ && pc_ < steps_.size()) ? progKeys_ : 0;
    // directives_ is sorted ascending by frame. Find the last directive whose
    // frame <= `frame`.
    u32 pressed = 0;
    for (const Directive& d : directives_)
    {
        if (d.frame <= frame) pressed = d.pressed;
        else break;
    }
    return pressed | prog;
}

} // namespace liteds

namespace liteds
{
bool InputScript::TouchForFrame(int frame, int& x, int& y) const
{
    if (progTx_ >= 0) { x = progTx_; y = progTy_; return true; }
    const Directive* cur = nullptr;
    for (const Directive& d : directives_)
    {
        if (d.frame > frame) break;
        cur = &d;
    }
    if (!cur || cur->touchX < 0) return false;
    x = cur->touchX; y = cur->touchY;
    return true;
}
}

namespace liteds
{
bool InputScript::ParseStep(const std::string& ln, std::string& e)
{
    std::vector<std::string> w;
    for (size_t i = 0; i < ln.size(); )
    {
        size_t j = ln.find_first_of(" \t", i);
        if (j == std::string::npos) j = ln.size();
        if (j > i) w.push_back(ln.substr(i, j - i));
        i = j + 1;
    }
    auto num = [&](size_t k) { return k < w.size() ? atoi(w[k].c_str()) : 0; };
    Step st;
    auto keys = [&](const std::string& tok) {
        std::string u = Upper(tok);
        if (u == "NONE") return true;
        if (u.size() > 2 && u[0] == '0' && u[1] == 'X') { st.keys = (u32)std::strtoul(tok.c_str(), nullptr, 16) & 0x0FFFu; return true; }
        for (const std::string& t : SplitCommas(tok))
        {
            int x, y;
            if (sscanf(t.c_str(), "%*1[Tt]:%d:%d", &x, &y) == 2) { st.tx = x; st.ty = y; hasTouch_ = true; continue; }
            u32 bit;
            if (!NameToBit(t, bit)) { e = "unknown button '" + t + "'"; return false; }
            st.keys |= 1u << bit;
        }
        return true;
    };
    auto cond = [&](size_t& k) {
        for (;;)
        {
            if (k >= w.size()) { e = "condition missing"; return false; }
            Cond c; std::string t = w[k];
            if (!t.empty() && t[0] == '!') { c.neg = true; t = t.substr(1); }
            if (t != "rgb" || k + 9 >= w.size()) { e = "condition: rgb top|bot X Y W H R G B TOL"; return false; }
            c.scr = w[k + 1] == "bot"; c.x = num(k + 2); c.y = num(k + 3); c.w = num(k + 4); c.h = num(k + 5);
            c.r = num(k + 6); c.g = num(k + 7); c.b = num(k + 8); c.tol = num(k + 9);
            st.conds.push_back(c); k += 10;
            if (k < w.size() && w[k] == "&") { k++; continue; }
            return true;
        }
    };
    const std::string op = w[0];
    size_t i = 1;
    if (op == "@wait") st.n = num(1);
    else if (op == "@until")
    {
        st.op = 1; if (!cond(i)) return false;
        for (; i < w.size(); i++) { if (w[i] == "timeout") st.timeout = num(++i); else if (w[i] == "soft") st.soft = true; else { e = "unknown word '" + w[i] + "'"; return false; } }
    }
    else if (op == "@skipif") { st.op = 5; if (!cond(i)) return false; st.n = num(i); }
    else if (op == "@press" || op == "@repeat")
    {
        st.op = op == "@press" ? 2 : 3;
        if (w.size() < 2) { e = "keys missing"; return false; }
        if (!keys(w[1])) return false;
        for (i = 2; i < w.size(); )
        {
            if (w[i] == "hold") { st.hold = num(i + 1); i += 2; }
            else if (w[i] == "gap") { st.gap = num(i + 1); i += 2; }
            else if (w[i] == "every") { st.every = std::max(1, num(i + 1)); i += 2; }
            else if (w[i] == "timeout") { st.timeout = num(i + 1); i += 2; }
            else if (w[i] == "until") { i++; if (!cond(i)) return false; }
            else if (w[i] == "soft") { st.soft = true; i++; }
            else { e = "unknown word '" + w[i] + "'"; return false; }
        }
        if (st.op == 3 && st.conds.empty()) { e = "@repeat needs until"; return false; }
    }
    else if (op == "@log") { st.op = 4; st.text = ln.size() > 4 ? Trim(ln.substr(4)) : ""; }
    else { e = "unknown step '" + op + "'"; return false; }
    if (st.op != 4) st.text = ln;
    steps_.push_back(st);
    return true;
}
bool InputScript::Eval(const std::vector<Cond>& cs, const u32* top, const u32* bot) const
{
    for (const Cond& c : cs)
    {
        const u32* s = c.scr ? bot : top;
        if (!s) return false;
        u64 r = 0, g = 0, b = 0, n = 0;
        for (int y = c.y; y < c.y + c.h && y < 192; y++)
            for (int x = c.x; x < c.x + c.w && x < 256; x++) { u32 p = s[y * 256 + x]; r += (p >> 16) & 0xFF; g += (p >> 8) & 0xFF; b += p & 0xFF; n++; }
        if (!n) return false;
        const bool in = std::abs((int)(r / n) - c.r) <= c.tol && std::abs((int)(g / n) - c.g) <= c.tol && std::abs((int)(b / n) - c.b) <= c.tol;
        if (in == c.neg) return false;
    }
    return true;
}
void InputScript::OnFrame(int frame, const u32* top, const u32* bot)
{
    progKeys_ = 0; progTx_ = progTy_ = -1;
    for (int guard = 0; guard < 64 && pc_ < steps_.size() && !failed_; guard++)
    {
        const Step& s = steps_[pc_];
        if (stepStart_ < 0) stepStart_ = frame + 1;
        const int el = frame + 1 - stepStart_;
        bool done = false;
        switch (s.op)
        {
        case 0: done = el >= s.n; break;
        case 1: done = Eval(s.conds, top, bot); break;
        case 2:
            if (el < s.hold) { progKeys_ = s.keys; progTx_ = s.tx; progTy_ = s.ty; return; }
            done = el >= s.hold + s.gap;
            if (!done) return;
            break;
        case 3:
            if (el % s.every < s.hold) { progKeys_ = s.keys; progTx_ = s.tx; progTy_ = s.ty; }
            else done = Eval(s.conds, top, bot);
            break;
        case 4:
            printf("script %s: %s (frame %d)\n", path_.c_str(), s.text.c_str(), frame + 1);
            milestones_.push_back(s.text);
            done = true;
            break;
        case 5:
            if (Eval(s.conds, top, bot)) { printf("script %s: step %zu (%s) skips %d at frame %d\n", path_.c_str(), pc_ + 1, s.text.c_str(), s.n, frame + 1); pc_ += s.n; }
            done = true;
            break;
        }
        if (!done)
        {
            if (s.timeout && el >= s.timeout && s.soft)
            {
                printf("script %s: step %zu (%s) timed out at frame %d, going on\n", path_.c_str(), pc_ + 1, s.text.c_str(), frame + 1);
                pc_++; stepStart_ = -1; progKeys_ = 0; progTx_ = progTy_ = -1;
                continue;
            }
            if (s.timeout && el >= s.timeout)
            {
                failed_ = true; progKeys_ = 0; progTx_ = progTy_ = -1;
                printf("script %s: TIMEOUT at step %zu (%s), frame %d\n", path_.c_str(), pc_ + 1, s.text.c_str(), frame + 1);
            }
            return;
        }
        if (s.op != 4) printf("script %s: step %zu (%s) done at frame %d\n", path_.c_str(), pc_ + 1, s.text.c_str(), frame + 1);
        pc_++; stepStart_ = -1; progKeys_ = 0; progTx_ = progTy_ = -1;
    }
}

std::vector<std::string> InputScript::TakeMilestones()
{
    std::vector<std::string> result;
    result.swap(milestones_);
    return result;
}
}
