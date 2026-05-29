"""Shared helpers for ``runs/`` entry scripts."""

from __future__ import annotations

import sys
from pathlib import Path

from mutapp.cli import apply_seed, base_parser, resolve_live
from mutapp.display import configure_display, open_paths


def parse_and_configure(description: str, *, extra_args=None):
    parser = base_parser(description)
    parser.add_argument(
        "--open",
        action="store_true",
        help="Open saved PNG plots with the system image viewer after the run",
    )
    if extra_args:
        extra_args(parser)
    args = parser.parse_args()
    configure_display(interactive=args.live or args.show)
    apply_seed(args.seed)
    return args


def finish(result: dict, args, *, live: bool) -> None:
    print(f"Finished: {result['algorithm']}")
    for key in ("best_fitness", "best_path_cost", "mean_fitness"):
        if key in result:
            print(f"{key.replace('_', ' ').title()}: {result[key]}")
    plot_paths = [Path(p) for p in result.get("plots", [])]
    for p in plot_paths:
        print(f"  saved: {p.resolve()}")
    if not plot_paths:
        print("  (no plot files written)", file=sys.stderr)
    if args.open or (not live and not args.show and plot_paths):
        open_paths(plot_paths)


def live_flag(args) -> bool:
    return resolve_live(args.live)
