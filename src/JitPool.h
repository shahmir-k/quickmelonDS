#ifndef MELONDS_JITPOOL_H
#define MELONDS_JITPOOL_H

// LITEV_JIT_POOL_ALLOC: free-list allocator for the JIT's small bookkeeping objects (JitBlock,
// TinyVector buffers, block / link hash-map nodes). Each compiled block made ~6-8 small
// allocations; on the RG DS (Android scudo) operator new + the page faults behind it were ~14% of
// a compile. Size classes of 16 bytes up to 1 KB, LIFO free lists (recently freed = cache-warm),
// 64 KB chunks that are never returned. Per thread, so several emulator instances on different
// threads never share a list; memory freed on another thread just joins that thread's list.
// Host-only: no guest-visible effect (nothing orders or hashes by these addresses).

#include <cstddef>
#include <cstdlib>
#include <new>

namespace melonDS::JitPool
{
constexpr size_t Granule = 16, MaxSize = 1024, NumClasses = MaxSize / Granule, ChunkSize = 64 * 1024;

struct State
{
    void* Free[NumClasses] = {};
    char* Bump = nullptr;
    char* End = nullptr;
};
inline thread_local State Pool;

inline void* Alloc(size_t n)
{
    if (n == 0) n = 1;
    if (n > MaxSize) return ::operator new(n);
    const size_t c = (n - 1) / Granule;
    State& s = Pool;
    if (void* p = s.Free[c])
    {
        s.Free[c] = *(void**)p;
        return p;
    }
    const size_t sz = (c + 1) * Granule;
    if ((size_t)(s.End - s.Bump) < sz)
    {
        // ponytail: the chunk's unused tail is dropped (< 1 KB per 64 KB)
        s.Bump = (char*)std::malloc(ChunkSize);
        if (!s.Bump) throw std::bad_alloc();
        s.End = s.Bump + ChunkSize;
    }
    void* p = s.Bump;
    s.Bump += sz;
    return p;
}

inline void Free(void* p, size_t n)
{
    if (!p) return;
    if (n == 0) n = 1;
    if (n > MaxSize) { ::operator delete(p); return; }
    const size_t c = (n - 1) / Granule;
    *(void**)p = Pool.Free[c];
    Pool.Free[c] = p;
}

// std allocator for node-based containers
template <typename T>
struct Allocator
{
    using value_type = T;
    Allocator() noexcept = default;
    template <typename U> Allocator(const Allocator<U>&) noexcept {}
    T* allocate(size_t n) { return (T*)Alloc(n * sizeof(T)); }
    void deallocate(T* p, size_t n) noexcept { Free(p, n * sizeof(T)); }
    template <typename U> bool operator==(const Allocator<U>&) const noexcept { return true; }
    template <typename U> bool operator!=(const Allocator<U>&) const noexcept { return false; }
};
}

#endif
