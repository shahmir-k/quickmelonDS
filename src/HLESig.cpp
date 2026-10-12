#if defined(LITEV_A7HLE) || defined(LITEV_A9HLE)
#include "HLESig.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>

namespace melonDS::HLESig
{
namespace
{
// B / BL / BLX imm target of the ARM word w at a (Thumb bit set for BLX), false if w is no such branch
bool Branch(u32 w, u32 a, u32& t)
{
    if ((w >> 25 & 7) != 5) return false;
    s32 o = (s32)(w << 8) >> 6;
    if ((w >> 28) == 0xF) { t = (a + 8 + o + (w >> 23 & 2)) | 1; return true; }
    t = a + 8 + o;
    return true;
}
}

u32 Map::operator()(u32 ref) const
{
    for (u32 i = 0; i < loc.size(); i++)
        if (ref >= r->c[i].a && ref < r->c[i].b) return loc[i] + (ref - r->c[i].a);
    // a data address: the pointer literal at or below it (a global or the start of the struct it is in)
    const std::pair<u32, s32>* best = nullptr;
    for (const auto& d : data)
        if (d.first <= ref && ref - d.first < 0x400) best = &d;
    if (!best)
        for (const auto& d : data)
            if (d.first > ref && d.first - ref < 0x400) { best = &d; break; }
    return best ? ref + best->second : 0;
}

bool Relocate(const Recipe& r, u32 entry, Rd rd, void* ctx, Map& m)
{
    m.r = &r;
    m.loc.assign(r.nc, 0);
    m.data.clear();
    std::vector<bool> got(r.nc, false);
    // locate: repeat over the chunks until none is added (links can point backwards in the list)
    for (bool more = true; more;)
    {
        more = false;
        for (u32 k = 0; k < r.nc; k++)
        {
            if (got[k]) continue;
            const Chunk& c = r.c[k];
            u32 at;
            if (c.how == 0) at = entry - c.off;
            else
            {
                if (!got[c.src]) continue;
                const Chunk& s = r.c[c.src];
                if (c.how == 2) at = m.loc[c.src] + (s.b - s.a);
                else if (c.how == 3) at = m.loc[c.src] - (c.b - c.a);
                else
                {
                    u32 wa = m.loc[c.src] + 4 * c.word, w, t;
                    if (!rd(ctx, wa, w)) return false;
                    if (!Branch(w, wa, t)) t = w;          // (a literal: its value)
                    at = (t & ~1u) - c.off;
                }
            }
            if (at & 3) return false;
            m.loc[k] = at; got[k] = true; more = true;
        }
    }
    static const bool dbg = getenv("LITEV_HLESIG_DEBUG");
#define FAIL(...) do { if (dbg) { fprintf(stderr, "HLESig %08x: ", entry); fprintf(stderr, __VA_ARGS__); fputc(10, stderr); } return false; } while (0)
    for (u32 k = 0; k < r.nc; k++) if (!got[k]) FAIL("chunk %u not located", k);
    // masked code of every chunk, and every link
    const Link* l = r.l;
    const Link* le = r.l + r.nl;
    for (u32 k = 0; k < r.nc; k++)
    {
        const Chunk& c = r.c[k];
        u64 h = 0xcbf29ce484222325ull;
        for (u32 i = 0, n = (c.b - c.a) / 4; i < n; i++)
        {
            const u32 wa = m.loc[k] + 4 * i;
            u32 w;
            if (!rd(ctx, wa, w)) return false;
            u32 mw = w;
            for (; l < le && l->chunk == k && l->word == i; l++)
            {
                if (l->kind == 0 || l->kind == 3)
                {
                    u32 t;
                    if (!Branch(w, wa, t)) FAIL("chunk %u word %u: no branch", k, i);
                    mw = w & 0xFF000000;
                    if (l->kind == 3) continue;    // (a call out of the variant's code: not checked, as with exact bytes)
                    const Chunk& d = r.c[l->to];
                    if ((t & ~1u) - m.loc[l->to] != (l->ref & ~1u) - d.a || ((t ^ l->ref) & 1)) FAIL("chunk %u word %u: branch to %08x", k, i, t);
                    mw = w & 0xFF000000;
                }
                else
                {
                    if (l->kind == 1)
                    {
                        const Chunk& d = r.c[l->to];
                        if ((w & ~1u) - m.loc[l->to] != (l->ref & ~1u) - d.a || ((w ^ l->ref) & 1)) FAIL("chunk %u word %u: literal %08x", k, i, w);
                    }
                    else
                    {
                        const s32 dl = (s32)(w - l->ref);
                        auto it = std::find_if(m.data.begin(), m.data.end(), [&](const auto& p) { return p.first == l->ref; });
                        if (it == m.data.end()) m.data.push_back({l->ref, dl});
                        else if (it->second != dl) FAIL("chunk %u word %u: pointer %08x moved twice", k, i, l->ref);      // one pointer, one delta
                    }
                    mw = 0;
                }
            }
            for (int b = 0; b < 4; b++) h = (h ^ ((mw >> (8 * b)) & 0xFF)) * 0x100000001b3ull;
        }
        if (h != c.mfnv) FAIL("chunk %u at %08x: masked code differs", k, m.loc[k]);
    }
#undef FAIL
    std::sort(m.data.begin(), m.data.end());
    return true;
}
}
#endif
