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

The default full sweep is `0,2.5,5,10,20,40,60,80,100,150` ms **per received
MP packet** on both processes, three repetitions each. It takes hours; coordinate
Mac load before running. `--prepare-only` hashes/copies all inputs, writes a manifest,
and launches no emulator. Each case retains raw logs, screenshots, emulated save output,
and a JSON result in a new directory; the runner never removes previous results. Sources
are copied (not symlinked or hardlinked), input copies are read-only, and hashes are
checked before and after. The original game files are never passed to the emulators.

`LITEV_AMBIENT_DELAY_MS` delays only delivered MP frames. It does not delay the ENet
connection, ACKs or control traffic; it adds no jitter, loss or relay. The reported
`nominal_added_rtt_ms = 2 × delay` is an injected component, **not measured path RTT**.
Use these runs to locate a candidate boundary, then repeat with full-path packet faults
and real network measurements before publishing any game/phase-specific online gate.
