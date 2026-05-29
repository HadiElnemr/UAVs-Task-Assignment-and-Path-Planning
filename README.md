# Multi-UAV Task Assignment and Path Planning (MUTAPP)

Optimisation techniques for the **multi-UAV task assignment and path planning** problem (NP-hard; related to multi travelling salesman / MTSP).

| Algorithm | Script | Description |
|-----------|--------|-------------|
| Simulated annealing (SA) | `runs/run_sa.py` | SA task assignment, then SA path planning |
| Whale optimisation (WOA) | `runs/run_woa.py` | WOA path planning with random task assignment |
| Dragonfly algorithm (DA) | `runs/run_da.py` | DA path planning with random task assignment |
| Hybrid ACO + WOA | `runs/run_hybrid_aco_woa.py` | Ant colony task assignment + WOA paths |
| Hybrid ACO + DA | `runs/run_hybrid_aco_da.py` | Ant colony task assignment + DA paths |
| Genetic algorithm (GA) | `runs/run_ga.py` | GA task assignment + GA path planning |
| PSO demo | `runs/run_pso.py` | 2D particle swarm toy demo (not full MUTAPP) |
| All of the above | `runs/run_all.py` | Run every algorithm on one benchmark |

Plots are saved under `outputs/<algorithm>/benchmark_<n>_<name>/` with **titles naming the algorithm** and benchmark. Use `--show` to open interactive windows as well.

More background and figures: [project page](https://hadielnemr.github.io/projects/multi_uav_task_assignment_and_path_planning/).

## Installation

Requires **Python 3.10+**.

```bash
cd UAVs-Task-Assignment-and-Path-Planning
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## How to run

Always run from the **repository root**.

### See plots on screen

| Goal | Command |
|------|---------|
| **Live animation** (mid-points moving) | `unset MPLBACKEND && python runs/run_woa.py -b 1 --live` |
| **Final plots in a window** | `python runs/run_woa.py -b 1 --show` |
| **Open saved PNGs** (default after each run) | plots auto-open in your image viewer; or `--open` |

If `--live` does nothing, install Tk for matplotlib:

```bash
sudo apt install python3-tk
unset MPLBACKEND
python runs/run_woa.py -b 1 --live
```

Saved PNGs always land under `outputs/` (printed as absolute paths when the run finishes).

**Quick example** (smallest benchmark, ~seconds):

```bash
source .venv/bin/activate
python runs/run_woa.py --benchmark 1
```

**Common options** (all `runs/run_*.py` scripts):

| Option | Meaning |
|--------|---------|
| `--benchmark`, `-b` | Test instance `1`–`4` (default `1`) |
| `--output-dir`, `-o` | Plot output directory (default `outputs`) |
| `--show` | Show final plots interactively (still saves PNGs) |
| `--live` | **Real-time** matplotlib: mid-points (◆) move each step; dashed = current, solid = best |
| `--seed` | Random seed for reproducibility |

For `--live`, use an interactive matplotlib backend (do **not** set `MPLBACKEND=Agg`). You will see **diamond markers** on intermediate path points updating each optimisation step; **dashed** lines are the current candidate and **solid** lines are the best so far.

![WOA live path planning on benchmark 1](docs/assets/woa_liveplot_15s.gif)

*15 s timelapse of `python runs/run_woa.py -b 1 --live` (full run sped up for preview).*

```bash
python runs/run_woa.py -b 1 --live
python runs/run_sa.py -b 1 --live    # map + cost/temperature panels
```

Without `--live`, only final PNGs are written under `outputs/` (no animation).

**Benchmarks**

| `-b` | Map | Scenario |
|------|-----|----------|
| 1 | 10×10 | 5 UAVs, 5 tasks |
| 2 | 100×100 | 10 UAVs on y-axis, 15 tasks |
| 3 | 100×100 | 10 UAVs, 15 tasks (random) |
| 4 | 1000×1000 | 50 UAVs, 70 tasks (slow) |

**Examples**

```bash
# Dragonfly algorithm, benchmark 2
python runs/run_da.py -b 2

# Hybrid ant colony + whale, save plots only (headless)
MPLBACKEND=Agg python runs/run_hybrid_aco_woa.py -b 1 -o outputs

# Simulated annealing on large benchmark (can take several minutes)
python runs/run_sa.py -b 4

# WOA with 5 statistical runs (reports mean fitness)
python runs/run_woa.py -b 1 --runs 5

# Run all algorithms on benchmark 1 (skip SA if desired)
python runs/run_all.py -b 1 --skip-sa
```

## Project layout

```
mutapp/           # Shared models, algorithms, plotting, pipelines
runs/             # One CLI script per algorithm
docs/assets/      # README media (e.g. live-plot demo GIF)
outputs/          # Generated plots (git-ignored)
legacy/           # Original per-folder copies (archived)
notebooks/        # Jupyter notebooks from development
```

## Path planning model

Path planning inserts **mid-points** between UAV positions and assigned tasks (proof of concept). Obstacle avoidance was left as future work.

## Course and authors

Part of **Metaheuristic Optimisation Techniques for Multi-Cooperative Systems** course in the 2022-23 Winter Semester (German University in Cairo), taught by Assist. Prof. Omar Shehata.

Developed by [Hadi Elnemr](https://github.com/HadiElnemr), [David Michael](https://github.com/DavidMicheal), [Mohammed Ashraf](https://github.com/MohammedAshraf965), and [Ahmed Fathy](https://github.com/AhmedFathyAbdelkhalek).
