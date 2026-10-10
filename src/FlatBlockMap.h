#ifndef MELONDS_FLATBLOCKMAP_H
#define MELONDS_FLATBLOCKMAP_H

// LITEV_JIT_FLATMAPS: u32 -> pointer hash map with linear probing (open addressing, power-of-two
// size, backward-shift deletion: no tombstones). Replaces std::unordered_map for the JIT's block
// maps (JitBlocks9/7, RestoreCandidates): a lookup is one multiply + usually one cache line instead
// of a modulo (udiv) and a chase through bucket array -> node -> chain. On the RG DS's in-order
// A55 each of those is a cache miss: the map operations were ~8% of a JIT compile.
// Same contents as the std map after every operation (only iteration order differs, and nothing
// that iterates depends on it: reset deletes everything).

#include <cstddef>
#include <vector>
#include "types.h"

namespace melonDS
{
template <typename V>   // V: a pointer type; nullptr marks an empty slot (never stored)
class FlatBlockMap
{
public:
    struct Slot { u32 first; V second; };
    struct iterator
    {
        Slot* p; Slot* e;
        Slot& operator*() const { return *p; }
        Slot* operator->() const { return p; }
        iterator& operator++() { do ++p; while (p != e && !p->second); return *this; }
        iterator operator++(int) { iterator t = *this; ++*this; return t; }
        bool operator==(const iterator& o) const { return p == o.p; }
        bool operator!=(const iterator& o) const { return p != o.p; }
    };

    FlatBlockMap() { Rehash(1 << 12); }

    iterator begin()
    {
        iterator it{Slots.data(), Slots.data() + Slots.size()};
        if (it.p != it.e && !it.p->second) ++it;
        return it;
    }
    iterator end() { return {Slots.data() + Slots.size(), Slots.data() + Slots.size()}; }
    size_t size() const { return Count; }
    bool empty() const { return Count == 0; }

    iterator find(u32 key)
    {
        for (size_t i = Home(key);; i = (i + 1) & Mask)
        {
            Slot& s = Slots[i];
            if (!s.second) return end();
            if (s.first == key) return {&s, Slots.data() + Slots.size()};
        }
    }
    V& operator[](u32 key)
    {
        for (size_t i = Home(key);; i = (i + 1) & Mask)
        {
            Slot& s = Slots[i];
            if (s.first == key && s.second) return s.second;
            if (!s.second)
            {
                if ((Count + 1) * 2 > Slots.size())
                {
                    Rehash(Slots.size() * 2);
                    return (*this)[key];
                }
                // the caller assigns a non-null value right away (all uses are map[k] = v)
                s.first = key;
                Count++;
                return s.second;
            }
        }
    }
    void erase(iterator it) { EraseSlot(it.p - Slots.data()); }
    size_t erase(u32 key)
    {
        iterator it = find(key);
        if (it == end()) return 0;
        erase(it);
        return 1;
    }
    void clear()
    {
        for (Slot& s : Slots) s.second = nullptr;
        Count = 0;
    }

private:
    std::vector<Slot> Slots;
    size_t Mask = 0, Count = 0;
    int Shift = 32;
    size_t Home(u32 key) const { return (u32)(key * 0x9E3779B1u) >> Shift; }   // Fibonacci hashing
    void Rehash(size_t n)
    {
        std::vector<Slot> old;
        old.swap(Slots);
        Slots.assign(n, Slot{0, nullptr});
        Mask = n - 1;
        Shift = 32 - __builtin_ctzll(n);
        Count = 0;
        for (const Slot& s : old)
            if (s.second) (*this)[s.first] = s.second;
    }
    void EraseSlot(size_t i)
    {
        for (size_t j = i;;)
        {
            j = (j + 1) & Mask;
            if (!Slots[j].second) break;
            size_t k = Home(Slots[j].first);
            // slot j may stay if its home lies cyclically in (i, j]
            if (i <= j ? (i < k && k <= j) : (i < k || k <= j)) continue;
            Slots[i] = Slots[j];
            i = j;
        }
        Slots[i].second = nullptr;
        Count--;
    }
};
// Pending JIT links (LITEV_JIT_LINK) keyed by the target address they wait for: a FlatBlockMap
// from target to a singly linked list of sites. Replaces std::unordered_multimap, whose
// equal_range for every compiled block (almost always a miss) was ~half of LinkBlock on the A55.
// Sites of one target come out in a different order than the multimap gave them; they are only
// patched / listed (host code), so that has no guest-visible effect.
template <typename Site, typename Alloc>
class PendingSiteMap
{
public:
    struct Node { Site site; Node* next; };
    void insert(u32 key, const Site& site)
    {
        Node* n = (Node*)Alloc::Alloc(sizeof(Node));
        Node*& head = Heads[key];
        n->site = site;
        n->next = head;
        head = n;
        Count++;
    }
    // call f(site) for every site waiting on key, and drop them
    template <typename F> void drain(u32 key, F f)
    {
        auto it = Heads.find(key);
        if (it == Heads.end()) return;
        Node* n = it->second;
        Heads.erase(it);
        while (n)
        {
            Node* next = n->next;
            f(n->site);
            Alloc::Free(n, sizeof(Node));
            Count--;
            n = next;
        }
    }
    // remove the first site waiting on key that matches pred; true if one was removed
    template <typename P> bool remove_one(u32 key, P pred)
    {
        auto it = Heads.find(key);
        if (it == Heads.end()) return false;
        for (Node** pn = &it->second; *pn; pn = &(*pn)->next)
        {
            if (!pred((*pn)->site)) continue;
            Node* n = *pn;
            *pn = n->next;
            Alloc::Free(n, sizeof(Node));
            Count--;
            if (!it->second) Heads.erase(it);
            return true;
        }
        return false;
    }
    void clear()
    {
        for (auto& kv : Heads)
            for (Node* n = kv.second; n; )
            {
                Node* next = n->next;
                Alloc::Free(n, sizeof(Node));
                n = next;
            }
        Heads.clear();
        Count = 0;
    }
    size_t size() const { return Count; }
    bool empty() const { return Count == 0; }
private:
    FlatBlockMap<Node*> Heads;
    size_t Count = 0;
};
}

#endif
