"""Shared CLI helpers for algorithm runners."""

from __future__ import annotations

import argparse
from pathlib import Path


def base_parser(description: str) -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument(
        "--benchmark",
        "-b",
        type=int,
        default=1,
        choices=[1, 2, 3, 4],
        help="Benchmark instance (1=small … 4=large)",
    )
    parser.add_argument(
        "--output-dir",
        "-o",
        type=Path,
        default=Path("outputs"),
        help="Directory for saved plots",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Display plots interactively (also saves PNGs)",
    )
    parser.add_argument(
        "--live",
        action="store_true",
        help="Real-time map updates during optimisation (needs interactive matplotlib; do not set MPLBACKEND=Agg)",
    )
    parser.add_argument(
        "--seed",
        type=int,
        default=None,
        help="Random seed for reproducibility",
    )
    return parser


def resolve_live(live: bool) -> bool:
    if not live:
        return False
    from mutapp.display import backend_is_interactive

    if backend_is_interactive():
        return True
    print(
        "Warning: --live needs an interactive matplotlib backend.\n"
        "  Try: sudo apt install python3-tk\n"
        "  Then: unset MPLBACKEND && python runs/run_woa.py -b 1 --live",
        file=__import__("sys").stderr,
    )
    return False


def apply_seed(seed: int | None) -> None:
    if seed is not None:
        import random

        import numpy as np

        random.seed(seed)
        np.random.seed(seed)
