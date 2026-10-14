# Black ↔ White Just Play latency gate (Mac only)

`pokemon_ambient_latency.py` runs two separate, real-time `liteDS-headless` processes from
Union Room door savestates. The closed-loop scripts use framebuffer color conditions to
advance, then `@log` marks each milestone. The headless `--script-shot-dir` option records
both DS screens at the exact milestone frame; the runner records SHA-256 of each PPM and
the core's XXH3 hashes, confirms the phases are in order and visually distinct, and
rejects any script timeout, guest abort, process failure or missing phase. Only a complete
zero-delay baseline permits a delayed case. The trade scripts are a candidate, **not** a
validated complete trade: the historical zero-delay run reached both `offered` marks but
timed out before returning to the Union Room. That is an honest baseline failure, not a
measured latency limit. The battle scripts did historically return to the battle room.

Build this branch's Mac headless with `LITEV_HEADLESS_LAN` and `LITEV_AMBIENT_LAN` enabled.
Supply *your own* Black/White ROMs, matching `.ml5` savestates and `.sav` files. No
copyrighted input is checked in. Start with one baseline case:

```sh
python3 tools/headless/pokemon_ambient_latency.py \
  --headless /absolute/path/to/liteDS-headless \
  --black-rom /absolute/path/to/black.nds --black-state /absolute/path/to/black.ml5 \
  --black-save /absolute/path/to/black.sav \
  --white-rom /absolute/path/to/white.nds --white-state /absolute/path/to/white.ml5 \
  --white-save /absolute/path/to/white.sav \
  --scenario battle --delays 0 --repeats 1 --output /absolute/path/to/results
```

The default sweep is round trips of `0,5,10,20,35,50,100,200` ms (`--delays`, NetSim:
both processes hold received MP frames and presence pings for half of it, plus `--jitter-pct`
of jitter), with `--frame-load-ms 8` (a device's frame cost), three repetitions each. It takes hours; coordinate
Mac load before running. `--prepare-only` hashes/copies all inputs, writes a manifest,
and launches no emulator. Each case retains raw logs, screenshots, emulated save output,
and a JSON result in a new directory; the runner never removes previous results. Sources
are copied (not symlinked or hardlinked), input copies are read-only, and hashes are
checked before and after. The original game files are never passed to the emulators.

For the fixed trade fixtures, the last script step waits passively for the trade
selection panel (the game does not return to the Union Room). The runner compares
the trainer-board icon colors at `offered` and `returned to trade selection` on
**both** consoles: blue/orange must become orange/blue on Black, and the reverse on
White. Missing, grayscale or unexpectedly colored icons fail closed. The historical
final screenshots match the expected *post*-trade colors, but there is no saved
`offered` frame from that run, so this before/after oracle remains unproven until a
fresh zero-delay baseline. No trade latency claim may be made before it passes.

NetSim (`LITEV_AMBIENT_RTT_MS`, `_JITTER_MS`, `_LOSS`, `_RTT_AT`, `_SPIKE`; AmbientLAN.cpp) holds the
received MP frames and presence pings / pongs on both ends; ENet's connection setup, acks and
control channel are not held. Each case's `session` records the join, the disconnect, the
slowdown offer and the LAN verdict times (s after the link), host waits / timeouts and stale drops.
