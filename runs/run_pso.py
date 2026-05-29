#!/usr/bin/env python3
"""Run the standalone 2D PSO demonstration."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from mutapp.cli import apply_seed, base_parser, resolve_live
from mutapp.display import configure_display, open_paths
from mutapp.pipelines import ALGO_PSO


def main() -> None:
    parser = base_parser("PSO 2D demo (toy objective; not full MUTAPP)")
    parser.add_argument("--open", action="store_true", help="Open saved plots after the run")
    args = parser.parse_args()
    configure_display(interactive=args.live or args.show)
    apply_seed(args.seed)

    from mutapp.pso_demo import run_pso_demo

    out = args.output_dir / "pso_demo"
    result = run_pso_demo(out, algorithm_label=ALGO_PSO, show=args.show or args.live)
    print(f"Finished: {ALGO_PSO}")
    print(f"Best fitness: {result['best_fitness']}")
    paths = [Path(result["plot_png"])]
    if result.get("plot_gif"):
        paths.append(Path(result["plot_gif"]))
    for p in paths:
        print(f"  saved: {p.resolve()}")
    if args.open or not (args.live or args.show):
        open_paths(paths)


if __name__ == "__main__":
    main()
