#!/usr/bin/env python3
"""Run hybrid ant colony task assignment + WOA path planning."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from runs._common import finish, live_flag, parse_and_configure


def main() -> None:
    args = parse_and_configure("Hybrid ACO (task assignment) + WOA (path planning)")
    live = live_flag(args)

    from mutapp.pipelines import run_hybrid_aco_woa

    result = run_hybrid_aco_woa(
        args.benchmark, args.output_dir, show=args.show or live, live=live
    )
    finish(result, args, live=live)


if __name__ == "__main__":
    main()
