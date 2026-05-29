#!/usr/bin/env python3
"""Run whale optimisation (WOA) path planning."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from runs._common import finish, live_flag, parse_and_configure


def main() -> None:
    def extra(parser):
        parser.add_argument("--runs", type=int, default=1, help="Independent runs for mean stats")

    args = parse_and_configure("Whale optimisation algorithm (WOA)", extra_args=extra)
    live = live_flag(args)

    from mutapp.pipelines import run_woa

    result = run_woa(
        args.benchmark,
        args.output_dir,
        show=args.show or live,
        live=live,
        statistical_runs=args.runs,
    )
    finish(result, args, live=live)


if __name__ == "__main__":
    main()
