// LITEV_MP_FIBERS: cooperative fibers for Netplay's remote consoles. Several consoles share one worker
// thread; a console that waits on the lockstep link yields to the next one on the same core instead of
// sleeping in the kernel (futex wake-up latency and switch cost, and the console that was waited for
// usually runs next on the same core). arm64 only (the context switch is a few instructions of asm).
#pragma once
#ifdef LITEV_MP_FIBERS
#include <cstddef>

namespace melonDS::Fiber
{
struct Fiber;

// on a worker thread: create a fiber that runs fn(arg) on its own stack (fn must not return before
// calling Finish); Resume runs it until it yields or finishes (false once finished)
Fiber* Create(void (*fn)(void*), void* arg, size_t stackSize = 8u << 20);
bool Resume(Fiber* f);
void Destroy(Fiber* f);

// inside a fiber
bool Active();          // this thread is running a fiber
void Yield();           // back to the worker; returns when resumed
[[noreturn]] void Finish();
double CpuMs();         // thread CPU time spent in the current fiber so far

// per-fiber thread state to swap (e.g. NDS::Current): called on every switch in and out
void SetSwapHook(void (*save)(void** slot), void (*restore)(void* const* slot));
}
#endif
