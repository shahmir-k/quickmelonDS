#if defined(LITEV_A7HLE) || defined(LITEV_A9HLE)
#include "HLESig.h"
#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>

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
// Thumb branch at a (h: its halfword, h2 the next): BL / BLX pair (n = 2 units), B, B<cond> (n = 1); target with the Thumb bit
// set unless BLX (ARM)
bool BranchT(u32 h, u32 h2, u32 a, u32& t, u32& n)
{
    if ((h & 0xF800) == 0xF000)
    {
        if ((h2 & 0xE800) != 0xE800) return false;
        t = a + 4 + ((u32)((s32)(h << 21) >> 9)) + ((h2 & 0x7FF) << 1);
        t = (h2 & 0xF800) == 0xE800 ? t & ~3u : t | 1;
        n = 2;
        return true;
    }
    if ((h & 0xF800) == 0xE000) { t = (a + 4 + (u32)((s32)(h << 21) >> 20)) | 1; n = 1; return true; }
    if ((h & 0xF000) == 0xD000 && (h >> 8 & 0xF) < 0xE) { t = (a + 4 + (u32)((s32)(h << 24) >> 23)) | 1; n = 1; return true; }
    return false;
}
inline u32 MaskB(u32 u, bool thumb) { return !thumb ? u & 0xFF000000 : (u & 0xF000) == 0xD000 ? u & 0xFF00 : u & 0xF800; }

// the units of chunk c at a (false: unreadable)
bool Units(const Chunk& c, u32 a, Rd rd, void* ctx, std::vector<u32>& u)
{
    const u32 len = c.b - c.a;
    u.resize(c.thumb ? len / 2 : len / 4);
    if (!c.thumb)
    {
        for (u32 i = 0; i < u.size(); i++) if (!rd(ctx, a + 4 * i, u[i])) return false;
        return true;
    }
    u32 w = 0, wa = 1;
    for (u32 i = 0; i < u.size(); i++)
    {
        const u32 x = a + 2 * i;
        if ((x & ~3u) != wa) { wa = x & ~3u; if (!rd(ctx, wa, w)) return false; }
        u[i] = (x & 2) ? w >> 16 : w & 0xFFFF;
    }
    return true;
}
inline void Hash(u64& h, u32 v, bool thumb) { for (int b = 0; b < (thumb ? 2 : 4); b++) h = (h ^ ((v >> (8 * b)) & 0xFF)) * 0x100000001b3ull; }
// masked FNV of chunk k's units (its links' units masked as the generator does)
u64 MaskedHash(const Recipe& r, u32 k, std::vector<u32> u)
{
    const Chunk& c = r.c[k];
    for (const Link* l = r.l, *le = r.l + r.nl; l < le; l++)
    {
        if (l->chunk != k) continue;
        const u32 i = l->word;
        if (i >= u.size()) return 0;
        if (l->kind == 0 || l->kind == 3)
        {
            const bool pair = c.thumb && (u[i] & 0xF800) == 0xF000;
            u[i] = MaskB(u[i], c.thumb);
            if (pair && i + 1 < u.size()) u[i + 1] = MaskB(u[i + 1], true);
        }
        else { u[i] = 0; if (c.thumb && i + 1 < u.size()) u[i + 1] = 0; }
    }
    u64 h = 0xcbf29ce484222325ull;
    for (u32 v : u) Hash(h, v, c.thumb);
    return h;
}
}

bool EntryOk(const Recipe& r, u32 entry, Rd rd, void* ctx)
{
    for (u32 k = 0; k < r.nc; k++)
    {
        const Chunk& c = r.c[k];
        if (c.how != 0) continue;
        const u32 a = entry - c.off, n = (c.b - c.a) / (c.thumb ? 2 : 4);
        u32 u[512];
        if (n > 512) return true;               // (Relocate checks it)
        u32 w = 0, wa = 1;
        for (u32 i = 0; i < n; i++)
        {
            const u32 x = a + (c.thumb ? 2 * i : 4 * i);
            if ((x & ~3u) != wa) { wa = x & ~3u; if (!rd(ctx, wa, w)) return false; }
            u[i] = !c.thumb ? w : (x & 2) ? w >> 16 : w & 0xFFFF;
        }
        for (const Link* l = r.l, *le = r.l + r.nl; l < le; l++)
        {
            if (l->chunk != k || l->word >= n) continue;
            const u32 i = l->word;
            if (l->kind == 0 || l->kind == 3)
            {
                const bool pair = c.thumb && (u[i] & 0xF800) == 0xF000;
                u[i] = MaskB(u[i], c.thumb);
                if (pair && i + 1 < n) u[i + 1] = MaskB(u[i + 1], true);
            }
            else { u[i] = 0; if (c.thumb && i + 1 < n) u[i + 1] = 0; }
        }
        u64 h = 0xcbf29ce484222325ull;
        for (u32 i = 0; i < n; i++) Hash(h, u[i], c.thumb);
        if (h != c.mfnv) return false;
    }
    return true;
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

bool Map::Delta(u32 base, u32 size, s32& d) const
{
    bool any = false;
    for (const auto& x : data)
    {
        if (x.first - base >= size) continue;
        if (any && x.second != d) return false;
        d = x.second; any = true;
    }
    return any;
}

bool Relocate(const Recipe& r, u32 entry, Rd rd, void* ctx, Map& m, const u8* ram, u32 ramLen)
{
    m.r = &r;
    m.loc.assign(r.nc, 0);
    m.data.clear();
    std::vector<bool> got(r.nc, false);
    std::vector<u32> u;
    static const bool dbg = getenv("LITEV_HLESIG_DEBUG");
#define FAIL(...) do { if (dbg) { fprintf(stderr, "HLESig %08x: ", entry); fprintf(stderr, __VA_ARGS__); fputc(10, stderr); } return false; } while (0)
    // located without another chunk: the entry chunk (its masked code first: a cheap reject), then the main-RAM scans (one
    // match each)
    for (u32 k = 0; k < r.nc; k++)
    {
        const Chunk& c = r.c[k];
        if (c.how != 0) continue;
        m.loc[k] = entry - c.off; got[k] = true;
        if (!Units(c, m.loc[k], rd, ctx, u) || MaskedHash(r, k, u) != c.mfnv) FAIL("entry chunk %u differs", k);
    }
    for (u32 k = 0; k < r.nc; k++)
    {
        const Chunk& c = r.c[k];
        if (c.how != 5) continue;
        if (!ram) FAIL("chunk %u: no RAM to scan", k);
        const u32 len = c.b - c.a, w0 = (u32)c.off;
        u32 hits = 0, at = 0;
        // (a tight pass over the first unit, then the masked code of the few candidates)
        if (ramLen < len) FAIL("chunk %u: RAM too short to scan", k);
        const u32 end = ramLen - len;
        if (!c.thumb)
        {
            const u32* w = (const u32*)ram;
            for (u32 i = 0; i <= end / 4; i++)
            {
                if (w[i] != w0) continue;
                if (Units(c, 0x02000000 + i * 4, rd, ctx, u) && MaskedHash(r, k, u) == c.mfnv) { at = 0x02000000 + i * 4; hits++; }
            }
        }
        else
        {
            const u16* h = (const u16*)ram;
            for (u32 i = 0; i <= end / 2; i++)
            {
                if (h[i] != w0) continue;
                if (Units(c, 0x02000000 + i * 2, rd, ctx, u) && MaskedHash(r, k, u) == c.mfnv) { at = 0x02000000 + i * 2; hits++; }
            }
        }
        if (hits != 1) FAIL("chunk %u: %u scan matches", k, hits);
        m.loc[k] = at; got[k] = true;
    }
    // the others through a located chunk (links can point backwards in the list)
    for (bool more = true; more;)
    {
        more = false;
        for (u32 k = 0; k < r.nc; k++)
        {
            if (got[k]) continue;
            const Chunk& c = r.c[k];
            if (!got[c.src]) continue;
            const Chunk& s = r.c[c.src];
            u32 at;
            if (c.how == 2) at = m.loc[c.src] + (s.b - s.a);
            else if (c.how == 3) at = m.loc[c.src] - (c.b - c.a);
            else if (c.how == 4) at = m.loc[c.src] + c.off;
            else
            {
                u32 t;
                if (!s.thumb)
                {
                    u32 wa = m.loc[c.src] + 4 * c.word, w;
                    if (!rd(ctx, wa, w)) return false;
                    if (!Branch(w, wa, t)) t = w;          // (a literal: its value)
                }
                else
                {
                    const u32 ha = m.loc[c.src] + 2 * c.word;
                    u32 w0, w1, h, h2, n;
                    if (!rd(ctx, ha & ~3u, w0) || !rd(ctx, (ha + 2) & ~3u, w1)) return false;
                    h = (ha & 2) ? w0 >> 16 : w0 & 0xFFFF;
                    h2 = (ha & 2) ? w1 & 0xFFFF : w0 >> 16;
                    if (!BranchT(h, h2, ha, t, n))
                    {
                        if (ha & 2) FAIL("chunk %u: Thumb literal at %08x misaligned", k, ha);
                        t = w0;
                    }
                }
                at = (t & ~1u) - c.off;
            }
            if (at & (c.thumb ? 1 : 3)) FAIL("chunk %u misaligned at %08x", k, at);
            m.loc[k] = at; got[k] = true; more = true;
        }
    }
    for (u32 k = 0; k < r.nc; k++) if (!got[k]) FAIL("chunk %u not located", k);
    // masked code of every chunk, and every link
    const Link* l = r.l;
    const Link* le = r.l + r.nl;
    for (u32 k = 0; k < r.nc; k++)
    {
        const Chunk& c = r.c[k];
        if (!Units(c, m.loc[k], rd, ctx, u)) return false;
        for (; l < le && l->chunk == k; l++)
        {
            const u32 i = l->word, ua = m.loc[k] + (c.thumb ? 2 : 4) * i;
            if (i >= u.size() || (c.thumb && l->kind != 0 && l->kind != 3 && i + 1 >= u.size())) FAIL("chunk %u: link unit %u out of range", k, i);
            if (l->kind == 0 || l->kind == 3)
            {
                u32 t, n = 1;
                const bool ok = c.thumb ? BranchT(u[i], i + 1 < u.size() ? u[i + 1] : 0, ua, t, n) : Branch(u[i], ua, t);
                if (!ok) FAIL("chunk %u unit %u: no branch", k, i);
                if (l->kind == 3) continue;    // (a call out of the variant's code: not checked, as with exact bytes)
                const Chunk& d = r.c[l->to];
                if ((t & ~1u) - m.loc[l->to] != (l->ref & ~1u) - d.a || ((t ^ l->ref) & 1)) FAIL("chunk %u unit %u: branch to %08x", k, i, t);
            }
            else
            {
                if (c.thumb && (ua & 2)) FAIL("chunk %u unit %u: literal misaligned", k, i);
                const u32 w = c.thumb ? u[i] | u[i + 1] << 16 : u[i];
                if (l->kind == 1)
                {
                    const Chunk& d = r.c[l->to];
                    if ((w & ~1u) - m.loc[l->to] != (l->ref & ~1u) - d.a || ((w ^ l->ref) & 1)) FAIL("chunk %u unit %u: literal %08x", k, i, w);
                }
                else
                {
                    const s32 dl = (s32)(w - l->ref);
                    auto it = std::find_if(m.data.begin(), m.data.end(), [&](const auto& p) { return p.first == l->ref; });
                    if (it == m.data.end()) m.data.push_back({l->ref, dl});
                    else if (it->second != dl) FAIL("chunk %u unit %u: pointer %08x moved twice", k, i, l->ref);      // one pointer, one delta
                }
            }
        }
        if (MaskedHash(r, k, u) != c.mfnv) FAIL("chunk %u at %08x: masked code differs", k, m.loc[k]);
    }
#undef FAIL
    std::sort(m.data.begin(), m.data.end());
    return true;
}
}
#endif
