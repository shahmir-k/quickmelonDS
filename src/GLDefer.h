// GLDefer: records the GL calls of the hi-res 2D OpenGL renderer (GPU_OpenGL.cpp,
// GPU2D_OpenGL.cpp) on the emulator thread and replays them, in order, on a GL worker
// thread (LITEV_GLHI_DEFER, debug.litev.glhidefer=0: off). Arguments are taken when the call
// is made and data pointers (texture uploads, buffer data) are copied then, so the worker
// issues exactly the calls the emulator thread would have issued; only the driver time moves.
// Calls that return a value run on the worker synchronously (the recorder is flushed first).
// Every GL object of the renderer is created, used and deleted on the worker's context.
#ifndef GLDEFER_H
#define GLDEFER_H

#include <atomic>
#include <cstring>
#include <functional>
#include <memory>
#include <mutex>
#include <type_traits>
#include <utility>
#include <vector>
#include "types.h"
#include "GLWorker.h"

namespace melonDS
{
namespace GLDefer
{

// a command stream: [header][closure][payload...], in 1 MB blocks that never move
class Stream
{
public:
    static constexpr size_t BlockSize = 1 << 20;
    u8* Alloc(size_t n)
    {
        n = (n + 15) & ~size_t(15);
        if (Cur < 0 || Used[Cur] + n > Size[Cur])
        {
            Cur++;
            if (Cur == (int)Blocks.size() || Size[Cur] < n)
            {
                const size_t sz = n > BlockSize ? n : BlockSize;
                if (Cur == (int)Blocks.size()) { Blocks.emplace_back(); Used.push_back(0); Size.push_back(0); }
                Blocks[Cur].reset(new u8[sz]);
                Size[Cur] = sz;
            }
            Used[Cur] = 0;
        }
        u8* p = Blocks[Cur].get() + Used[Cur];
        Used[Cur] += n;
        return p;
    }
    template <class F> void Push(F f)
    {
        static_assert(std::is_trivially_copyable<F>::value && std::is_trivially_destructible<F>::value, "GLDefer: plain captures only");
        u8* p = Alloc(sizeof(Hdr) + sizeof(F));
        Hdr* h = (Hdr*)p;
        h->Size = (u32)((sizeof(Hdr) + sizeof(F) + 15) & ~size_t(15));
        h->Run = [](u8* q) { (*(F*)q)(); };
        memcpy(p + sizeof(Hdr), &f, sizeof(F));
        N++;
    }
    bool Empty() const { return N == 0; }
    void Replay()
    {
        for (int b = 0; b <= Cur; b++)
        {
            u8* p = Blocks[b].get();
            u8* end = p + Used[b];
            while (p < end)
            {
                Hdr* h = (Hdr*)p;
                if (h->Size == Payload) { p += h->Skip; continue; }
                h->Run(p + sizeof(Hdr));
                p += h->Size;
            }
        }
        Cur = -1; N = 0;
    }
private:
    struct alignas(16) Hdr { u32 Size; u32 Skip; void (*Run)(u8*); };
    static constexpr u32 Payload = 0xFFFFFFFFu;
    std::vector<std::unique_ptr<u8[]>> Blocks;
    std::vector<size_t> Used, Size;
    int Cur = -1;
    size_t N = 0;
public:
    // payload: a header marking it so Replay skips it
    void* CopyData(const void* src, size_t n)
    {
        if (!src || !n) return nullptr;
        u8* p = Alloc(sizeof(Hdr) + n);
        Hdr* h = (Hdr*)p;
        h->Size = Payload;
        h->Skip = (u32)((sizeof(Hdr) + n + 15) & ~size_t(15));
        memcpy(p + sizeof(Hdr), src, n);
        return p + sizeof(Hdr);
    }
};

// the emulator thread's recorder: one per renderer; the worker owns the GL context
class Recorder
{
public:
    std::unique_ptr<GLWorker> W;
    // queued batches the worker has not finished; the emulator thread waits above MaxPending
    std::atomic<int> Pending {0};
    int MaxPending = 2;

    Recorder() { for (auto& s : Free) s = std::make_unique<Stream>(); }

    Stream& S() { if (!Rec) Rec = Take(); return *Rec; }
    // hand the recorded calls to the worker (no wait)
    void Submit()
    {
        if (!Rec || Rec->Empty()) return;
        Stream* s = Rec; Rec = nullptr;
        Pending.fetch_add(1, std::memory_order_relaxed);
        W->Run([this, s] { s->Replay(); Give(s); Pending.fetch_sub(1, std::memory_order_release); }, false);
    }
    // run f on the worker after everything recorded so far, and wait for it
    void RunSync(const std::function<void()>& f) { Submit(); W->Run([&f] { f(); }, true); }
    // wait for the worker to finish everything recorded so far
    void Drain() { Submit(); W->Wait(); }
private:
    Stream* Rec = nullptr;
    std::mutex M;
    std::unique_ptr<Stream> Free[4];
    std::vector<std::unique_ptr<Stream>> Extra;
    Stream* Take()
    {
        std::lock_guard<std::mutex> l(M);
        for (auto& s : Free) if (s) { Stream* r = s.release(); return r; }
        return new Stream();   // ponytail: only past 4 in flight (MaxPending keeps it at 3)
    }
    void Give(Stream* s)
    {
        std::lock_guard<std::mutex> l(M);
        for (auto& f : Free) if (!f) { f.reset(s); return; }
        delete s;
    }
};

// the recorder calls go to; only the thread that set up the renderer (the emulator thread)
// records, every other thread (the worker, the 3D and present threads) calls GL directly
extern Recorder* Active;
extern thread_local bool RecordThread;
inline bool Recording() { return Active && RecordThread; }

// trivially copyable argument pack
template <class... P> struct Pack {};
template <class H, class... T> struct Pack<H, T...> { H h; Pack<T...> t; };
template <size_t I, class Pk> auto& Get(Pk& p) { if constexpr (I == 0) return p.h; else return Get<I - 1>(p.t); }
template <class... P> struct Maker;
template <> struct Maker<> { static Pack<> Make() { return {}; } };
template <class H, class... T> struct Maker<H, T...>
{
    template <class A, class... B> static Pack<H, T...> Make(A a, B... b) { return Pack<H, T...>{ (H)a, Maker<T...>::Make(b...) }; }
};
template <class R, class... P, class Pk, size_t... I> R ApplyP(R (*fn)(P...), Pk& p, std::index_sequence<I...>) { return fn(Get<I>(p)...); }

template <class... P, class... A> inline void Call(void (*fn)(P...), A... a)
{
    if (!Recording()) { fn((P)a...); return; }
    auto pk = Maker<P...>::Make(a...);
    Active->S().Push([fn, pk]() mutable { ApplyP(fn, pk, std::index_sequence_for<P...>{}); });
}
template <class R, class... P, class... A> inline R CallSync(R (*fn)(P...), A... a)
{
    if (!Recording()) return fn((P)a...);
    R r{};
    Active->RunSync([&] { r = fn((P)a...); });
    return r;
}
template <class... P, class... A> inline void CallSyncV(void (*fn)(P...), A... a)
{
    if (!Recording()) { fn((P)a...); return; }
    Active->RunSync([&] { fn((P)a...); });
}
// any trivially copyable callable, recorded (runs on the worker in order)
template <class F> inline void Exec(F f)
{
    if (!Recording()) { f(); return; }
    Active->S().Push(f);
}
// copy n bytes of call data now; the copy lives until the call is replayed
inline const void* Data(const void* p, size_t n) { return Recording() ? Active->S().CopyData(p, n) : p; }

}
}

#endif
