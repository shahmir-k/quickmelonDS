// GLWorker: a thread with its own EGL context shared with the creating thread's current
// context; Run() queues work on it (used by the hybrid's GL 3D and present threads and by
// the frontend's software present thread). Without EGL it runs work inline.
#ifndef GLWORKER_H
#define GLWORKER_H

#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <thread>
#ifdef __ANDROID__
#include <EGL/egl.h>
#include <sched.h>
#include <sys/resource.h>
#include <pthread.h>
#endif
#include "Platform.h"
#if defined(__ANDROID__) && defined(LITEV_TOPO_PIN)
#include "LitevCores.h"
#endif

namespace melonDS
{

// A thread with its own EGL context shared with the caller's; Run() queues work on it.
class GLWorker
{
public:
    // run f on the GL thread (inline when there is none); wait = block until done
    void Run(std::function<void()> f, bool wait)
    {
        if (!Threaded) { f(); return; }
        {
            std::lock_guard<std::mutex> l(M);
            Q.push_back(std::move(f));
        }
        CV.notify_one();
        if (wait) Wait();
    }
    void Wait()
    {
        if (!Threaded) return;
        std::unique_lock<std::mutex> l(M);
        DoneCV.wait(l, [this] { return Q.empty() && !Busy; });
    }
    void WaitQueued()   // every queued job has started (the running one may continue)
    {
        if (!Threaded) return;
        std::unique_lock<std::mutex> l(M);
        DoneCV.wait(l, [this] { return Q.empty(); });
    }

    bool Threaded = false;

public:
    ~GLWorker() { StopThread(); }
private:
    std::thread T;
    std::mutex M;
    std::condition_variable CV, DoneCV;
    std::deque<std::function<void()>> Q;
    bool Busy = false, Quit = false;
#ifdef __ANDROID__
    EGLDisplay Dpy = EGL_NO_DISPLAY;
    EGLContext Ctx = EGL_NO_CONTEXT;
    EGLSurface Surf = EGL_NO_SURFACE;
#endif

public:
    void StartThread(const char* name)
    {
#ifdef __ANDROID__
        Dpy = eglGetCurrentDisplay();
        EGLContext share = eglGetCurrentContext();
        if (Dpy == EGL_NO_DISPLAY || share == EGL_NO_CONTEXT) return;
        EGLint cfgId = 0, n = 0;
        EGLConfig cfg = nullptr;
        eglQueryContext(Dpy, share, EGL_CONFIG_ID, &cfgId);
        const EGLint cfgAttr[] = {EGL_CONFIG_ID, cfgId, EGL_NONE};
        if (!eglChooseConfig(Dpy, cfgAttr, &cfg, 1, &n) || n < 1) return;
        const EGLint ctxAttr[] = {EGL_CONTEXT_CLIENT_VERSION, 3, EGL_NONE};
        Ctx = eglCreateContext(Dpy, cfg, share, ctxAttr);
        if (Ctx == EGL_NO_CONTEXT) return;

        bool ok = false, started = false;
        T = std::thread([this, cfg, name, &ok, &started] {
            // surfaceless if supported, else a 1x1 pbuffer
            bool cur = eglMakeCurrent(Dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, Ctx);
            if (!cur)
            {
                const EGLint pb[] = {EGL_WIDTH, 1, EGL_HEIGHT, 1, EGL_NONE};
                Surf = eglCreatePbufferSurface(Dpy, cfg, pb);
                cur = Surf != EGL_NO_SURFACE && eglMakeCurrent(Dpy, Surf, Surf, Ctx);
            }
            // off the emu thread's core 3 (it is pinned there by the app)
#ifdef LITEV_TOPO_PIN
            LitevTopo::PinSelf(LitevTopo::CoreRole::RenderCritical);
#else
            cpu_set_t set; CPU_ZERO(&set);
            CPU_SET(0, &set); CPU_SET(1, &set); CPU_SET(2, &set);
            sched_setaffinity(0, sizeof(set), &set);
#endif
            // the emu thread waits for this job when it falls a frame behind: let it win
            // cores 0-2 against the 2D thread and the app's background threads
            setpriority(PRIO_PROCESS, 0, -10);
            pthread_setname_np(pthread_self(), name);
            {
                std::lock_guard<std::mutex> l(M);
                ok = cur; started = true;
            }
            DoneCV.notify_all();
            if (!cur) return;
            Loop();
            eglMakeCurrent(Dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
        });
        {
            std::unique_lock<std::mutex> l(M);
            DoneCV.wait(l, [&] { return started; });
        }
        Threaded = ok;
        Platform::Log(Platform::Info, "LITEV_HYB GL thread %s: %s\n", name, ok ? "on" : "FAILED (inline)");
        if (!ok)
        {
            T.join();
            if (Surf != EGL_NO_SURFACE) eglDestroySurface(Dpy, Surf);
            eglDestroyContext(Dpy, Ctx);
            Ctx = EGL_NO_CONTEXT; Surf = EGL_NO_SURFACE;
        }
#endif
    }
    void StopThread()
    {
        if (!Threaded) return;
        {
            std::lock_guard<std::mutex> l(M);
            Quit = true;
        }
        CV.notify_one();
        T.join();
        Threaded = false;
#ifdef __ANDROID__
        if (Surf != EGL_NO_SURFACE) eglDestroySurface(Dpy, Surf);
        eglDestroyContext(Dpy, Ctx);
#endif
    }
private:
    void Loop()
    {
        for (;;)
        {
            std::function<void()> f;
            {
                std::unique_lock<std::mutex> l(M);
                CV.wait(l, [this] { return Quit || !Q.empty(); });
                if (Q.empty()) return;   // Quit with nothing left
                f = std::move(Q.front());
                Q.pop_front();
                Busy = true;
            }
            DoneCV.notify_all();   // WaitQueued
            f();
            {
                std::lock_guard<std::mutex> l(M);
                Busy = false;
            }
            DoneCV.notify_all();
        }
    }
};

}

#endif // GLWORKER_H
