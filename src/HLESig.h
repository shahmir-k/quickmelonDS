// HLESig: find HLE'd NitroSDK / NitroSystem functions in any game by code pattern (LITEV_A7HLE / LITEV_A9HLE).
//
// An HLE variant is written against one reference build (exact code bytes at fixed addresses). The same SDK code in
// another game differs only where the linker put things: branch / call offsets and literal-pool pointers. A recipe
// (generated from the reference by tools/hle/reloc.py) splits the variant's code ranges into chunks at function starts and
// stores per chunk a masked FNV-1a 64 (B / BL / BLX offsets that leave the chunk and pointer literals zeroed), how to find
// the chunk in another build (the entry chunk at the candidate entry; the others through a call or a literal of a chunk
// already found, or next to one), every link (which chunk a branch / literal must land in, at which offset) and the
// pointer literals into data. Relocate() applies it to a candidate entry: every chunk's masked code must match, every link
// must land where the reference's does, a pointer used in several places must move by one delta. The result maps any
// reference address of the variant (code: through its chunk; data: through the pointer literal at or below it) to the
// candidate build, so a variant's address fields can be rewritten and its native code runs unchanged.
//
// Cost: only when the JIT compiles a block whose instruction passes the variant's two-word pre-filter (entry instruction
// + the next word); nothing per frame. The result is then verified like a static variant (exact bytes of the relocated
// chunks at compile time, the JIT block depends on them).
#pragma once
#if defined(LITEV_A7HLE) || defined(LITEV_A9HLE)
#include "types.h"
#include <vector>
#include <utility>

namespace melonDS::HLESig
{
struct Chunk
{
    u32 a, b;               // reference range [a, b)
    u64 mfnv;               // masked FNV-1a 64 of its words
    u8 how;                 // 0 entry chunk (entry - a = off), 1 via word `word` of chunk `src` (a branch's target or a
                            // literal's value, minus `off`), 2 right after chunk `src`, 3 right before chunk `src`
    u8 src; u16 word; s32 off;
};
struct Link
{
    u16 chunk, word;        // the word (in chunk order: Recipe::l sorted by chunk)
    u8 kind;                // 0 branch into chunk `to` (offset masked), 1 literal pointer into chunk `to`, 2 literal pointer into data,
                            // 3 branch out of the variant's code (offset masked; the target is not checked, as exact bytes do not)
    u8 to;                  // (0 / 1) the chunk it lands in
    u32 ref;                // (0 / 1) reference target, (2) reference value
};
struct Recipe
{
    u32 entry, w0, w1;      // reference entry and its first two words (pre-filter)
    u16 nc; const Chunk* c;
    u16 nl; const Link* l;
};
// guest word at a (false: not readable)
using Rd = bool (*)(void* ctx, u32 a, u32& v);

struct Map
{
    const Recipe* r = nullptr;
    std::vector<u32> loc;                           // candidate address of each chunk
    std::vector<std::pair<u32, s32>> data;          // reference pointer value -> delta (sorted)
    // candidate address of a reference address of the variant; 0 if it is in no chunk and near no pointer literal
    u32 operator()(u32 ref) const;
    // the candidate chunk ranges (code the native version replaces: snapshot + JIT dependencies)
    template <class R> void Ranges(std::vector<R>& out) const
    {
        out.clear();
        for (u32 i = 0; i < loc.size(); i++) out.push_back({loc[i], loc[i] + (r->c[i].b - r->c[i].a)});
    }
};
// relocate recipe r to the candidate entry (Thumb bit clear); true and m filled if every check passes
bool Relocate(const Recipe& r, u32 entry, Rd rd, void* ctx, Map& m);
}
#endif
