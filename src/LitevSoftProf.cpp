#include "LitevSoftProf.h"

#ifdef LITEV_SOFTPROF

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#ifdef __ANDROID__
#include <sys/system_properties.h>
#include <unistd.h>
#endif

#if defined(__linux__) || defined(__ANDROID__)
#include <sys/prctl.h>
#endif

namespace melonDS
{
namespace LitevSP
{

State S;

void NameThread(const char* n)
{
#if defined(__linux__) || defined(__ANDROID__)
    prctl(PR_SET_NAME, n, 0, 0, 0);
#else
    (void)n;
#endif
}

namespace
{
struct RingEnt { double t, cpu; int ev; int tid; };
std::atomic<int> RingN { 0 };
}

void Ev(int ev)
{
    // magic statics: thread-safe one-time init
    static const int RingCap = [] {
        int cap = 0;
#ifdef __ANDROID__
        char b[PROP_VALUE_MAX] = {};
        if (__system_property_get("debug.litev.pipering", b) > 0) cap = atoi(b);
#endif
        return cap > 0 ? cap : 0;
    }();
    static RingEnt* const Ring = RingCap ? (RingEnt*)calloc(RingCap, sizeof(RingEnt)) : nullptr;
    if (!Ring) return;
    const int i = RingN.fetch_add(1, std::memory_order_relaxed);
    if (i >= RingCap) return;
    timespec c; clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c);
    Ring[i] = { NowMs(), c.tv_sec * 1e3 + c.tv_nsec / 1e6, ev, 
#ifdef __ANDROID__
        (int)gettid()
#else
        0
#endif
    };
    if (i == RingCap - 1)
    {
        if (FILE* f = fopen("/sdcard/Android/data/com.sereneds.app/files/pipering.csv", "w"))
        {
            fprintf(f, "t_ms,cpu_ms,ev,tid\n");
            for (int k = 0; k < RingCap; k++)
                fprintf(f, "%.3f,%.3f,%d,%d\n", Ring[k].t, Ring[k].cpu, Ring[k].ev, Ring[k].tid);
            fclose(f);
        }
        Platform::Log(Platform::LogLevel::Info, "LITEV_PIPERING wrote %d events\n", RingCap);
    }
}

void Tick()
{
    int f = S.Frames.fetch_add(1, std::memory_order_relaxed) + 1;
    if (f < 60) return;
    S.Frames.store(0, std::memory_order_relaxed);

    const double n = 60.0;
    double barrier = S.EmuBarrier.Take() / n;
    double bar3d   = S.Emu3DBarrier.Take() / n;
    double texbar  = S.EmuTexBarrier.Take() / n;
    double snap    = S.EmuSnap.Take() / n;
    double clr     = S.S3DClear.Take() / n;
    double rast    = S.S3DRaster.Take() / n;
    double finw    = S.S3DFinalWall.Take() / n;
    double s2dw    = S.S2DWall.Take() / n;
    double rwall   = S.RenderWall.Take() / n;
    double wake    = S.S3DWake.Take() / n;
    double tot3d   = S.S3DTotal.Take() / n;
    double win     = S.EmuWindow.Take() / n;

    char bands[256]; bands[0] = 0;
    char fins[256];  fins[0] = 0;
    int nb = S.NB3D.load(std::memory_order_relaxed);
    if (nb < 1) nb = 1;
    if (nb > MAXB) nb = MAXB;
    for (int b = 0; b < nb; b++)
    {
        char tmp[32];
        snprintf(tmp, sizeof(tmp), "%s%.2f", b ? "/" : "", S.S3DBand[b].Take() / n);
        strncat(bands, tmp, sizeof(bands) - strlen(bands) - 1);
        snprintf(tmp, sizeof(tmp), "%s%.2f", b ? "/" : "", S.S3DFinal[b].Take() / n);
        strncat(fins, tmp, sizeof(fins) - strlen(fins) - 1);
    }

    char s2db[256]; s2db[0] = 0;
    char s2blk[256]; s2blk[0] = 0;
    for (int b = 0; b < 4; b++)
    {
        double bd = S.S2DBand[b].Take() / n;
        double bl = S.S2DBlock[b].Take() / n;
        if (bd <= 0.0 && bl <= 0.0) continue;
        char tmp[32];
        snprintf(tmp, sizeof(tmp), "%s%.2f", s2db[0] ? "/" : "", bd);
        strncat(s2db, tmp, sizeof(s2db) - strlen(s2db) - 1);
        snprintf(tmp, sizeof(tmp), "%s%.2f", s2blk[0] ? "/" : "", bl);
        strncat(s2blk, tmp, sizeof(s2blk) - strlen(s2blk) - 1);
    }

    int npost = S.NPost.exchange(0, std::memory_order_relaxed);
    int nrend = S.NRender.exchange(0, std::memory_order_relaxed);
    int nid   = S.NIdent.exchange(0, std::memory_order_relaxed);
    int nfin  = S.NFinish.exchange(0, std::memory_order_relaxed);
    uint64_t ce[5];
    for (int i = 0; i < 5; i++) ce[i] = S.CompositeCE[i].exchange(0, std::memory_order_relaxed);

    Platform::Log(Platform::LogLevel::Info,
        "LITEV_SOFTPROF 60f: N[post=%d rend=%d ident=%d fin=%d] EMU[bar3D=%.2f texbar=%.2f bar2D=%.2f snap=%.2f budget=%.2f] | "
        "3D[LATENCY=%.2f wake=%.2f clear=%.2f raster=%.2f final=%.2f bands=%s fin=%s] | "
        "2D[wall=%.2f bands=%s block=%s ce=%llu/%llu/%llu/%llu/%llu]\n",
        npost, nrend, nid, nfin,
        bar3d, texbar, barrier, snap, win,
        tot3d, wake, clr, rast, finw, bands, fins, s2dw, s2db, s2blk,
        (unsigned long long)ce[0], (unsigned long long)ce[1], (unsigned long long)ce[2],
        (unsigned long long)ce[3], (unsigned long long)ce[4]);
    (void)rwall;
}

} // namespace LitevSP
} // namespace melonDS

#endif
