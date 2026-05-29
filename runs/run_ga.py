#!/usr/bin/env python3
"""Run genetic algorithm (GA) task assignment + path planning."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from runs._common import finish, live_flag, parse_and_configure


def main() -> None:
    args = parse_and_configure("Genetic algorithm (GA)")
    live = live_flag(args)

    from mutapp.pipelines import run_genetic_algorithm

    result = run_genetic_algorithm(
        args.benchmark, args.output_dir, show=args.show or live, live=live
    )
    finish(result, args, live=live)


if __name__ == "__main__":
    main()
