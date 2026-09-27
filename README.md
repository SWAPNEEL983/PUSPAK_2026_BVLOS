# UAV-X: Resilient BVLOS Swarm — Sector Sweep / T-Sweep Simulator

Submission for **Grand Challenge 1 — UAV-X: Resilient BVLOS Swarm Challenge**, PUSHPAK Grand Challenge 2026 (IIT Bombay Techfest 2026-27, presented by IISER Bhopal).

A proof-of-concept simulator for a UAV swarm that surveys a disaster area for Points of Interest (PoIs). The swarm keeps an end-to-end multi-hop radio link to a Ground Control Station (GCS) outside the area, and it manages battery endurance by relieving UAVs on schedule.

> **Team:** [ team name ] · **Team ID:** [ … ] · **Demo video:** [ link ]

---

## Mission model

| Parameter | Value |
|---|---|
| Operational area | 1000 m × 1000 m (25 × 25 grid of 40 m cells; continuous positions in the Python port) |
| Points of interest | 10, random, > 2 cells apart; appear at random times within the first 10 min |
| Radio range | 100 m per hop, multi-hop (unit-disk links) |
| Minimum separation | 20 m (continuous check in the Python port; one UAV per cell in the grid sims) |
| UAV speed | 5 m/s |
| Endurance | 20 min per flight; mission bounded so everyone lands by 45:00 |
| Report deadline | Each PoI reported within 10 s of discovery |
| Sensing | PoI detected within 1 cell (3 × 3-cell footprint); only UAVs linked to the GCS sense |

---

## Strategies

### T-sweep, report only (primary)
- The GCS (OPS) sits 75 m left of the area. A **sentinel** UAV at the middle of the left edge is the only node within range of it.
- UAVs enter through an **arrival lane** and leave through a **departure lane** on either side of the sentinel.
- A line of **12 UAVs** (80 m apart) sweeps the area left to right and back.
- An **11-relay trunk** trails the line along the middle row. Each trunk relay drops off at its column once the line has passed.
- The line only advances if every line UAV stays linked to the GCS.
- A **relief scheduler** plans each UAV's departure backwards from its battery deadline, giving every UAV its own slot in the single-lane exit. A fresh UAV is launched in time to take over the post.

### Sector Sweep (alternative strategies)
- **Adaptive search with global memory.** The swarm shares a map of every cell seen so far. Explorers head for the nearest unseen cell inside a 140 m frontier around the network. Once everything has been seen they re-check the stalest cells.
- **Relay network.** Every 5 ticks the planner builds a convex hull of the searchers, then an MST from the sentinel to the hull. Relays are placed wherever an MST edge is longer than 100 m.
- **Modes:**
  - adaptive search with PoIs held
  - adaptive search, report only
  - formation sweep, report only
  - T-sweep with PoIs held on an MST network

### Connectivity rules (Python port)
- **Tether rule.** `tether = R + v × (T − t_comms − t_confirm)`, which gives about 135–150 m. An explorer may only move to a point within the tether of a stable connected UAV. If it is already outside, it may only move closer.
- **Connectivity-preserving moves.** No move may disconnect a stable connected UAV.
- **Make-before-break.** When a PoI's link is rerouted, the new branch must be fully manned before the old chain is removed.

---

## Repository structure

```
PUSHPAK_2026_BVLOS/
├── requirements.txt                       # Python dependencies
├── sector_sweep.py                        # Continuous-space simulator (tether, connectivity-preserving moves, make-before-break)
├── fleet_sweep.py                         # Fixed-fleet Monte Carlo success-rate sweep (imports sector_sweep.py)
├── mst_montecarlo.py                      # Static MST drone-count Monte Carlo
├── PUSHPAK_T_sweep_report_only.html       # T-sweep simulator with relief scheduling (browser)
├── Sector_Sweep_global_memory.html        # Multi-strategy simulator (browser)
├── docs/
│   └── UAV-X_Stage1_Technical_Proposal.pdf
└── README.md
```

---

## Installation

**Requirements:** [ OS ] · Python [ version ] · any modern web browser

```bash
git clone [ repo URL ]
cd [ repo-name ]
pip install -r requirements.txt
```

Dependencies: `numpy`, `scipy`, `networkx`, `shapely`, `pandas`, `matplotlib`, `tqdm`, `pyyaml`, `pytest` (optional: `numba`, `seaborn`).

---

## Usage

### Browser simulators (no install needed)
Open either HTML file in a browser.
- **`PUSHPAK_T_sweep_report_only.html`** has Start, Step and New layout buttons, a speed slider, and a sensor-footprint toggle. The status panel shows the mission clock, PoIs found, reports within 10 s, drones used, drones in the air, longest flight and rule breaks.
- **`Sector_Sweep_global_memory.html`** lets you set the strategy, the PoI appearance window and the explorer count, then press **New layout** and **Start**.

The T-sweep page exposes a headless test hook, `window.__PUSHPAK`, with `reset`, `tick`, `results`, `done`, `setRng` and `state`.

### Python
```bash
python sector_sweep.py        [ arguments … ]
python mst_montecarlo.py      [ arguments … ]
python fleet_sweep.py --workers N   [ further arguments … ]
```

---

## Results so far

**Continuous-space Python port** (staleness search, earlier version):
- All 10 PoIs completed.
- Every report arrived within 10 s (worst case 5 s).
- Minimum inter-UAV separation was 20.001 m.
- Missions took about 9,900–10,100 s.

**Static MST network size** (sentinel + 10 PoI anchors + relays, 1,000 layouts at 25 × 25):

| Mean | Std | Median | 95th percentile | Max |
|---|---|---|---|---|
| 27.8 | 2.9 | 28 | 33 (CI 32–33) | 36 |

**Fixed-fleet sweep** (5 of 10 layouts done):
- A run counts as a success if the mission finishes within 240 min with every report within 10 s and no separation violations.
- Battery use is ignored in this sweep.
- Without a fleet cap, peak demand was 61–67 drones.

| Fleet cap | 33 | 38 | 43 | 48 | 53 | 58 | 63–90 |
|---|---|---|---|---|---|---|---|
| Successes / 5 | 0 | 1 | 2 | 4 | 4 | 5 | 5 |

Not yet measured: packet delivery ratio, latency, connectivity availability, recovery time, relay reallocations, and performance after UAV or link failures.

---

## Roadmap
-  Finish the fleet sweep on 30+ layouts and find the smallest fleet with ≥ 95% success
-  Priority weighting for newly emerging high-priority regions
-  Inject UAV failures, link outages and packet loss; measure recovery time and PDR
-  Explicit geofence and charge-constraint checks in the metrics
-  Verify that `max_tether_gap_m` is 0 after the tether fixes
-  Merge PoI links at discovery, add Steiner points and drop-off relays to lower peak fleet size
-  Adopt the organizers' benchmark scenarios and standardised log format

---

## Team

| Member | Role / contribution |
|---|---|
| [ … ] | [ … ] |

## License
[ … ]

## References
1. A. M. Andrew, "Another efficient algorithm for convex hulls in two dimensions," *Information Processing Letters*, 9(5), 1979.
2. R. C. Prim, "Shortest connection networks and some generalizations," *Bell System Technical Journal*, 36(6), 1957.
3. E. W. Dijkstra, "A note on two problems in connexion with graphs," *Numerische Mathematik*, 1, 1959.
