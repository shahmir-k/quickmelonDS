#include "Fiber.h"
#ifdef LITEV_MP_FIBERS
#if !defined(__aarch64__)
#error "LITEV_MP_FIBERS: arm64 only"
#endif
#include <sys/mman.h>
#include <ctime>
#include <cstdint>
#include <cstdlib>

#ifdef __APPLE__
#define FSYM(x) "_" #x
#define FTYPE(x) ""
#else
#define FSYM(x) #x
#define FTYPE(x) ".type " #x ", %function\n"
#endif

// litev_fiber_switch(void** saveSP, void* newSP): saves the callee-saved state on this stack, stores sp
// in *saveSP, switches to newSP and restores the state saved there.
// litev_fiber_start: first entry of a new fiber (x19 = fn, x20 = arg).
__asm__(
    ".text\n"
    ".globl " FSYM(litev_fiber_switch) "\n" FTYPE(litev_fiber_switch)
    ".p2align 2\n"
    FSYM(litev_fiber_switch) ":\n"
    "  sub sp, sp, #160\n"
    "  stp x19, x20, [sp, #0]\n"
    "  stp x21, x22, [sp, #16]\n"
    "  stp x23, x24, [sp, #32]\n"
    "  stp x25, x26, [sp, #48]\n"
    "  stp x27, x28, [sp, #64]\n"
    "  stp x29, x30, [sp, #80]\n"
    "  stp d8, d9, [sp, #96]\n"
    "  stp d10, d11, [sp, #112]\n"
    "  stp d12, d13, [sp, #128]\n"
    "  stp d14, d15, [sp, #144]\n"
    "  mov x2, sp\n"
    "  str x2, [x0]\n"
    "  mov sp, x1\n"
    "  ldp x19, x20, [sp, #0]\n"
    "  ldp x21, x22, [sp, #16]\n"
    "  ldp x23, x24, [sp, #32]\n"
    "  ldp x25, x26, [sp, #48]\n"
    "  ldp x27, x28, [sp, #64]\n"
    "  ldp x29, x30, [sp, #80]\n"
    "  ldp d8, d9, [sp, #96]\n"
    "  ldp d10, d11, [sp, #112]\n"
    "  ldp d12, d13, [sp, #128]\n"
    "  ldp d14, d15, [sp, #144]\n"
    "  add sp, sp, #160\n"
    "  ret\n"
    ".globl " FSYM(litev_fiber_start) "\n" FTYPE(litev_fiber_start)
    ".p2align 2\n"
    FSYM(litev_fiber_start) ":\n"
    "  mov x0, x20\n"
    "  blr x19\n"
    "  brk #0\n");

extern "C" void litev_fiber_switch(void** saveSP, void* newSP);
extern "C" void litev_fiber_start();

namespace melonDS::Fiber
{
struct Fiber
{
    void* sp = nullptr;          // saved stack pointer while switched out
    void* stack = nullptr;
    size_t stackSize = 0;
    bool done = false;
    double cpuMs = 0;            // thread CPU time spent inside
    void* swapSlot = nullptr;    // the hook's saved per-fiber state
};

static thread_local Fiber* Cur = nullptr;
static thread_local void* WorkerSP = nullptr;
static thread_local double InAt = 0;
static void (*SaveHook)(void**) = nullptr;
static void (*RestoreHook)(void* const*) = nullptr;

static double ThreadCpuMs()
{
    timespec ts;
    clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
    return ts.tv_sec * 1e3 + ts.tv_nsec / 1e6;
}

void SetSwapHook(void (*save)(void**), void (*restore)(void* const*)) { SaveHook = save; RestoreHook = restore; }

Fiber* Create(void (*fn)(void*), void* arg, size_t stackSize)
{
    Fiber* f = new Fiber;
    f->stackSize = stackSize;
    f->stack = mmap(nullptr, stackSize, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (f->stack == MAP_FAILED) abort();
    // the frame litev_fiber_switch restores: x19 = fn, x20 = arg, x29 = 0, x30 = litev_fiber_start
    uintptr_t top = ((uintptr_t)f->stack + stackSize) & ~(uintptr_t)15;
    uint64_t* frame = (uint64_t*)(top - 160);
    for (int i = 0; i < 20; i++) frame[i] = 0;
    frame[0] = (uint64_t)(uintptr_t)fn;
    frame[1] = (uint64_t)(uintptr_t)arg;
    frame[10] = 0;
    frame[11] = (uint64_t)(uintptr_t)&litev_fiber_start;
    f->sp = frame;
    return f;
}

bool Resume(Fiber* f)
{
    if (f->done) return false;
    Cur = f;
    if (RestoreHook) RestoreHook(&f->swapSlot);
    InAt = ThreadCpuMs();
    litev_fiber_switch(&WorkerSP, f->sp);
    // back on the worker: the fiber yielded or finished (its sp saved by the switch)
    Cur = nullptr;
    return !f->done;
}

static void SwitchOut()
{
    Fiber* f = Cur;
    f->cpuMs += ThreadCpuMs() - InAt;
    if (SaveHook) SaveHook(&f->swapSlot);
    litev_fiber_switch(&f->sp, WorkerSP);
}

bool Active() { return Cur != nullptr; }
void Yield() { SwitchOut(); }
void Finish() { Cur->done = true; SwitchOut(); abort(); }
double CpuMs() { return Cur ? Cur->cpuMs + (ThreadCpuMs() - InAt) : ThreadCpuMs(); }

void Destroy(Fiber* f)
{
    if (!f) return;
    munmap(f->stack, f->stackSize);
    delete f;
}
}
#endif
