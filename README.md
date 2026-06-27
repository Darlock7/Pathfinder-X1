<div align="center">

# PATHFINDER — X1

### A sub-250 g thrust-vectored flying wing. The Mavic Mini, redrawn.

![MATLAB](https://img.shields.io/badge/MATLAB-R2024b%2B-orange?logo=mathworks&logoColor=white)
![Stack](https://img.shields.io/badge/Autopilot-ArduPilot%20%2F%20PX4-success)
![Weight](https://img.shields.io/badge/Target-%3C249g%20(FAA%20exempt)-blue)
![Phase](https://img.shields.io/badge/Phase-X1%20Aero%20Testbed-yellow)

*Pack it in · Fly it far · Bring the shot home*

</div>

---

## Premise

Quadcopters spend most of their battery fighting gravity. A flying wing trades hover for cruise and gets **3×+ the endurance** from the same 249 g. Pathfinder uses **twin thrust-vectored pushers** to get full 3-axis authority across three regimes — **no flaps, fewer parts, cleaner wing**.

| Regime | AoA / vector | Speed | Use |
|---|---|---|---|
| **Cruise** | Low AoA | ~22 m/s | Max range, min power |
| **STOL** | Higher AoA, vectored | ~5 m/s | Hand-launch / short land |
| **Harrier hover** | High AoA, full vector | 1–2 m/s | Cinematic dwell loiter |

## Development arc

| | Purpose | Defining features |
|---|---|---|
| **X1** *(current)* | Prove pure thrust-only control on a flying wing | Twin pusher props · roll-axis vectored thrust · **zero control surfaces** · static FPV · 3D-print + ribs + CF spar |
| **X2** | Match Mavic Mini, beat it on endurance | Full active thrust control · autonomous STOL + hover · gimbaled FPV · app + goggles |
| **X3** | Manufacturable, safer than a quad | Full CF body · small outboard elevons · recovery chute · sub-249 g production |

**Failure tolerance (X3):** elevon glide · vectored thrust · chute — three independent layers vs. a quad's single point of failure.

> Full concept pitch: [`docs/Pathfinder_Series_Pitch.pdf`](docs/Pathfinder_Series_Pitch.pdf)

---

## Engineering approach

Pathfinder reuses the proven end-to-end **MATLAB MDO pipeline** architecture from the Nimbus flying-wing project: from-scratch physics models for each subsystem, tightly coupled and driven by a global optimizer. Key adaptation for Pathfinder: a **thrust-vectoring control-allocation** layer (no control surfaces in X1) and a hard **sub-250 g mass budget** as the binding constraint.

```
Pathfinder-X1/
├── main_pathfinder.m        # top-level sizing / optimization driver
├── run_project.m            # adds all src/ paths
├── src/
│   ├── aerodynamics/        # XFOIL surrogates, AVL polars, spanwise aero  (ported)
│   ├── stability/           # static + dynamic stability via AVL           (ported)
│   ├── propulsion/          # propeller surrogate + propulsion sizing       (ported)
│   ├── energy/              # battery / endurance model                     (ported)
│   ├── mission/             # flight-regime profiles, cruise-speed solve    (ported, adapting)
│   ├── geometry/            # wing + centerbody + vertical surface geometry (ported)
│   ├── control/            # NEW: thrust-vector allocation across regimes
│   ├── mass/               # NEW: sub-250 g weight budget tracker
│   └── tools/              # RealFlight (RFX) export
└── resources/              # airfoil + propeller surrogate model databases
```

## Status
**X1 — Aero testbed.** Goal: validate thrust-only control (differential thrust + vectoring) in simulation, then in hardware. Hover (bicopter/tailsitter regime) is the long-pole risk and the first thing X1 exists to de-risk.

## Author
**Juan Sanchez** — Aerospace Engineering (Flight Dynamics & Control), UC San Diego.
