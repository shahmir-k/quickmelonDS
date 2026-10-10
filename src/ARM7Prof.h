// LITEV_A7PROF (diagnostic, headless/interpreter only): per-guest-function ARM7 profile.
// Run with --mode interp and LITEV_A7PROF=<out prefix>. Hooks the ARMv4 interpreter loop:
// every instruction is charged (instructions + cycles) to its PC and, via a shadow call stack
// (call = LR points just past the previous instruction; return = PC hits a stacked return
// address; IRQ = vector 0x18), to the current function (self) and every function on the stack
// (inclusive). IO accesses are tallied per current function so functions can be classified
// by the hardware they touch (SPU 0x0400040x-0x040005xx, IPC 0x0400018x, SPI 0x040001Cx, ...).
#pragma once
#ifdef LITEV_A7PROF
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <map>
#include <unordered_map>
#include <vector>
#include <algorithm>

namespace melonDS::A7Prof
{
struct PcStat { uint64_t n = 0, cyc = 0; uint32_t size = 4; };
struct FnStat { uint64_t selfN = 0, selfC = 0, inclN = 0, inclC = 0, calls = 0; std::map<uint32_t, uint64_t> io; std::map<uint32_t, uint64_t> callers; };
struct Frame { uint32_t ret, fn; bool irq; uint32_t node; };
struct Node { uint32_t parent, fn; uint64_t selfN = 0, selfC = 0, calls = 0; };

struct Prof
{
    const char* out = getenv("LITEV_A7PROF");
    uint64_t frames = 0, skipFrames = (uint64_t)atoi(getenv("LITEV_A7PROF_SKIP") ? getenv("LITEV_A7PROF_SKIP") : "5");
    std::unordered_map<uint32_t, PcStat> pcs;
    std::unordered_map<uint32_t, FnStat> fns;
    std::vector<Frame> st;
    std::vector<Node> nodes{Node{0, 0xFFFFFFFF}};   // calling-context tree, node 0 = root
    std::unordered_map<uint64_t, uint32_t> nodeIdx;
    uint32_t curNode() const { return st.empty() ? 0 : st.back().node; }
    uint32_t prevPc = 0, prevSize = 0, prevMode = 0;
    bool active() const { return out && frames >= skipFrames; }

    uint32_t cur() const { return st.empty() ? 0xFFFFFFFF : st.back().fn; }

    // called before executing the instruction at pc (size 2/4), cpsr current, lr = R[14]
    void Pre(uint32_t pc, uint32_t size, uint32_t cpsr, uint32_t lr)
    {
        uint32_t mode = cpsr & 0x1F;
        if (pc != prevPc + prevSize)
        {
            if (pc == 0x18 && mode == 0x12)
            {
                uint32_t r = (lr - 4) & ~1u;
                Push(r, 0x18, true);
            }
            else if (prevSize && (lr & ~1u) == prevPc + prevSize)
                Push(prevPc + prevSize, pc, false);
            else
            {
                bool popped = false;
                for (size_t i = st.size(); i-- > 0 && st.size() - i <= 16;)
                    if (st[i].ret == pc) { st.resize(i); popped = true; break; }
                // exception return that went somewhere else (thread switch): drop the IRQ frame
                if (!popped && prevMode == 0x12 && mode != 0x12)
                    for (size_t i = st.size(); i-- > 0;)
                        if (st[i].irq) { st.resize(i); break; }
            }
            if (st.size() > 256) st.clear();
        }
        prevPc = pc; prevSize = size; prevMode = mode;
    }
    void Push(uint32_t ret, uint32_t fn, bool irq)
    {
        if (active()) { auto& f = fns[fn]; f.calls++; f.callers[cur()]++; }
        uint64_t key = ((uint64_t)curNode() << 32) | fn;
        auto it = nodeIdx.find(key);
        uint32_t n;
        if (it != nodeIdx.end()) n = it->second;
        else { n = (uint32_t)nodes.size(); nodes.push_back(Node{curNode(), fn}); nodeIdx[key] = n; }
        if (active()) nodes[n].calls++;
        st.push_back({ret, fn, irq, n});
    }
    void Post(uint32_t pc, uint32_t cyc)
    {
        if (!active()) return;
        auto& p = pcs[pc]; p.n++; p.cyc += cyc; p.size = prevSize;
        auto& f = fns[cur()]; f.selfN++; f.selfC += cyc;
        auto& nd = nodes[curNode()]; nd.selfN++; nd.selfC += cyc;
        // inclusive: each distinct function on the stack once
        uint32_t seen[8]; int ns = 0;
        for (size_t i = st.size(); i-- > 0;)
        {
            uint32_t fn = st[i].fn; bool dup = false;
            for (int k = 0; k < ns; k++) if (seen[k] == fn) { dup = true; break; }
            if (dup) continue;
            if (ns < 8) seen[ns++] = fn;
            auto& g = fns[fn]; g.inclN++; g.inclC += cyc;
        }
    }
    uint64_t jitCyc = 0, jitEntries = 0, jitCompiles = 0;
    void Jit(bool ran, uint64_t cyc) { if (!active()) return; jitCyc += cyc; jitEntries++; jitCompiles += !ran; }
    void IO(uint32_t addr) { if (active()) fns[cur()].io[addr & ~1u]++; }
    // returns true once, on the first profiled frame: the caller dumps guest memory then
    bool EndFrame() { frames++; return out && frames == skipFrames; }

    ~Prof()
    {
        if (!out) return;
        uint64_t f = frames > skipFrames ? frames - skipFrames : 1;
        std::string o(out);
        FILE* a = fopen((o + ".pcs.tsv").c_str(), "w");
        fprintf(a, "pc\tinstr_per_frame\tcyc_per_frame\tsize\n");
        for (auto& [pc, s] : pcs) fprintf(a, "%08x\t%.2f\t%.2f\t%u\n", pc, s.n / (double)f, s.cyc / (double)f, s.size);
        fclose(a);
        FILE* b = fopen((o + ".fns.tsv").c_str(), "w");
        fprintf(b, "fn\tself_i\tself_c\tincl_i\tincl_c\tcalls\ttop_callers\tio\n");
        for (auto& [fn, s] : fns)
        {
            fprintf(b, "%08x\t%.1f\t%.1f\t%.1f\t%.1f\t%.2f\t", fn, s.selfN / (double)f, s.selfC / (double)f,
                    s.inclN / (double)f, s.inclC / (double)f, s.calls / (double)f);
            std::vector<std::pair<uint64_t, uint32_t>> c;
            for (auto& [k, v] : s.callers) c.push_back({v, k});
            std::sort(c.rbegin(), c.rend());
            for (size_t i = 0; i < c.size() && i < 3; i++) fprintf(b, "%08x:%.1f ", c[i].second, c[i].first / (double)f);
            fprintf(b, "\t");
            std::vector<std::pair<uint64_t, uint32_t>> io;
            for (auto& [k, v] : s.io) io.push_back({v, k});
            std::sort(io.rbegin(), io.rend());
            for (size_t i = 0; i < io.size() && i < 6; i++) fprintf(b, "%08x:%.1f ", io[i].second, io[i].first / (double)f);
            fprintf(b, "\n");
        }
        fclose(b);
        FILE* t = fopen((o + ".cct.tsv").c_str(), "w");
        fprintf(t, "node\tparent\tfn\tself_i\tself_c\tcalls\n");
        for (size_t i = 0; i < nodes.size(); i++)
            if (nodes[i].selfN || nodes[i].calls)
                fprintf(t, "%zu\t%u\t%08x\t%.2f\t%.2f\t%.3f\n", i, nodes[i].parent, nodes[i].fn, nodes[i].selfN / (double)f, nodes[i].selfC / (double)f, nodes[i].calls / (double)f);
        fclose(t);
        if (jitEntries)
            fprintf(stderr, "A7PROF JIT: executed cycles/frame %.0f, C++ entries/frame %.1f, compiles/frame %.2f\n", jitCyc / (double)f, jitEntries / (double)f, jitCompiles / (double)f);
        fprintf(stderr, "A7PROF: %llu frames profiled -> %s.{pcs,fns}.tsv\n", (unsigned long long)f, out);
    }
};
inline Prof g;
}
#endif
