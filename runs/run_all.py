#!/usr/bin/env python3
"""Run every MUTAPP algorithm on the selected benchmark."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mutapp.cli import apply_seed, base_parser, resolve_live
from mutapp.display import configure_display, open_paths


def main() -> None:
    parser = base_parser("Run all MUTAPP algorithms")
    parser.add_argument(
        "--skip-sa",
        action="store_true",
        help="Skip simulated annealing (slow on benchmark 4)",
    )
    parser.add_argument("--open", action="store_true", help="Open saved PNGs after all runs")
    args = parser.parse_args()
    configure_display(interactive=args.live or args.show)
    apply_seed(args.seed)
    live = resolve_live(args.live)

    from mutapp.pipelines import (
        run_da,
        run_genetic_algorithm,
        run_hybrid_aco_woa,
        run_simulated_annealing,
        run_woa,
    )
    from mutapp.pso_demo import run_pso_demo
    from mutapp.pipelines import ALGO_PSO

    all_plots: list[Path] = []

    runners = [
        ("WOA", lambda: run_woa(args.benchmark, args.output_dir, show=args.show or live, live=live)),
        ("DA", lambda: run_da(args.benchmark, args.output_dir, show=args.show or live, live=live)),
        (
            "Hybrid ACO+WOA",
            lambda: run_hybrid_aco_woa(
                args.benchmark, args.output_dir, show=args.show or live, live=live
            ),
        ),
        (
            "Hybrid ACO+DA",
            lambda: run_da(
                args.benchmark,
                args.output_dir,
                show=args.show or live,
                live=live,
                use_aco_task_assignment=True,
            ),
        ),
        (
            "GA",
            lambda: run_genetic_algorithm(
                args.benchmark, args.output_dir, show=args.show or live, live=live
            ),
        ),
    ]
    if not args.skip_sa:
        runners.insert(
            0,
            (
                "SA",
                lambda: run_simulated_annealing(
                    args.benchmark, args.output_dir, show=args.show or live, live=live
                ),
            ),
        )

    for name, fn in runners:
        print(f"\n=== {name} ===")
        result = fn()
        all_plots.extend(Path(p) for p in result.get("plots", []))

    print("\n=== PSO demo ===")
    pso = run_pso_demo(args.output_dir / "pso_demo", algorithm_label=ALGO_PSO, show=args.show or live)
    all_plots.append(Path(pso["plot_png"]))

    print("All runs complete.")
    for p in all_plots:
        print(f"  saved: {p.resolve()}")
    if args.open or (not live and not args.show):
        open_paths(all_plots)


if __name__ == "__main__":
    main()
