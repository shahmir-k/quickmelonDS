// Feeds fake topologies into the role placement (src/LitevCores.h, LITEV_TOPO_PIN) and prints
// role -> cores. Build + run (any host):
//   c++ -std=c++17 -DLITEV_TOPO_PIN -Isrc tools/topotest/topotest.cpp -o /tmp/topotest && /tmp/topotest
#include "LitevCores.h"
#include <cassert>
#include <cstdio>

using namespace melonDS::LitevTopo;
using V = std::vector<int>;
static const CoreType IN = CoreType::InOrder, OOO = CoreType::OoO, UNK = CoreType::Unknown;

static Placement show(const char* name, std::vector<CoreInfo> c)
{
    Placement p = Place(c);
    printf("== %s\n%s\n", name, p.Describe().c_str());
    return p;
}

int main()
{
    std::vector<CoreInfo> rgds;
    for (int i = 0; i < 4; i++) rgds.push_back({i, 1024, 1992000, IN});
    Placement p = show("RG DS (4x A55)", rgds);
    // must equal today's measured-good placement
    assert(p.pin && p.emu == 3 && p.workers == V({0, 1, 2}) && p.critical == V({0, 1, 2}) && p.background == V({0, 1, 2}));

    std::vector<CoreInfo> thor = {
        {0, 280, 2016000, IN}, {1, 280, 2016000, IN}, {2, 280, 2016000, IN},
        {3, 855, 2803200, OOO}, {4, 855, 2803200, OOO}, {5, 855, 2803200, OOO}, {6, 855, 2803200, OOO},
        {7, 1024, 3187200, OOO}};
    p = show("AYN Thor (SD 8 Gen 2)", thor);
    assert(p.emu == 7 && p.workers == V({3, 4, 5, 6}) && p.critical == V({3, 4, 5, 6}) && p.background == V({0, 1, 2}));

    // Duo Lite: capacity values unknown -> exercise the max_freq fallback (A53 1.8 vs A73 2.0 GHz:
    // only the in-order/OoO rule separates them).
    std::vector<CoreInfo> duo;
    for (int i = 0; i < 4; i++) duo.push_back({i, 0, 1804800, IN});
    for (int i = 4; i < 8; i++) duo.push_back({i, 0, 2016000, OOO});
    p = show("Retroid Pocket Duo Lite (4x A53 + 4x A73, max_freq only)", duo);
    assert(p.emu == 7 && p.workers == V({4, 5, 6}) && p.critical == V({4, 5, 6}) && p.background == V({0, 1, 2, 3}));

    p = show("2-core", {{0, 1024, 2000000, IN}, {1, 1024, 2000000, IN}});
    assert(!p.pin && p.workerCount == 1 && p.CoresFor(CoreRole::Emu) == V({0, 1}));

    p = show("1-core", {{0, 1024, 2000000, IN}});
    assert(!p.pin && p.workerCount == 1 && p.CoresFor(CoreRole::Background) == V({0}));

    std::vector<CoreInfo> nodata;
    for (int i = 0; i < 8; i++) nodata.push_back({i, 0, 0, UNK});
    p = show("8 cores, no capacity/freq (desktop VM)", nodata);
    assert(!p.pin && p.workerCount == 3);

    // hypothetical: in-order core clocked >1.5x the OoO one -> the in-order core gets the emu
    p = show("hypothetical 2x in-order 3.2 GHz + 2x OoO 2.0 GHz", {{0, 0, 3200000, IN}, {1, 0, 3200000, IN}, {2, 0, 2000000, OOO}, {3, 0, 2000000, OOO}});
    assert(p.emu == 1);

    puts("all placement checks passed");
}
