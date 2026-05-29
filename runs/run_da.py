#!/usr/bin/env python3
"""Run dragonfly algorithm (DA) path planning."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from runs._common import finish, live_flag, parse_and_configure


def main() -> None:
    args = parse_and_configure("Dragonfly algorithm (DA)")
    live = live_flag(args)

    from mutapp.pipelines import run_da

    result = run_da(
        args.benchmark,
        args.output_dir,
        show=args.show or live,
        live=live,
        use_aco_task_assignment=False,
    )
    finish(result, args, live=live)


if __name__ == "__main__":
    main()
