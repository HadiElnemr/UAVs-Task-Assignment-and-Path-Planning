"""Standard benchmark instances (sys1–sys4)."""

from __future__ import annotations

import copy
import random
from dataclasses import dataclass

from mutapp.models import Point, System, Task, UAV


@dataclass(frozen=True)
class BenchmarkSpec:
    index: int
    name: str
    x_map: int
    y_map: int
    description: str


BENCHMARKS: tuple[BenchmarkSpec, ...] = (
    BenchmarkSpec(1, "small", 10, 10, "5 UAVs, 5 tasks on a 10×10 map"),
    BenchmarkSpec(2, "medium_a", 100, 100, "10 UAVs on y-axis, 15 random tasks on 100×100"),
    BenchmarkSpec(3, "medium_b", 100, 100, "10 UAVs and 15 tasks, all random on 100×100"),
    BenchmarkSpec(4, "large", 1000, 1000, "50 UAVs on x-axis, 70 random tasks on 1000×1000"),
)

# Default metaheuristic settings per benchmark (from original scripts)
WOA_PARAMS = [(40, 5, 100), (50, 4, 200), (50, 5, 200), (100, 3, 200)]
DA_PARAMS = [(30, 400), (40, 400), (50, 200), (100, 200)]
HYBRID_DA_PARAMS = [(20, 300), (30, 400), (40, 400), (30, 400)]


def get_benchmark(benchmark: int) -> BenchmarkSpec:
    if benchmark < 1 or benchmark > len(BENCHMARKS):
        raise ValueError(f"benchmark must be 1–{len(BENCHMARKS)}, got {benchmark}")
    return BENCHMARKS[benchmark - 1]


def _build_sys1() -> System:
    tasks = [
        Task(Point(2, 5), 0),
        Task(Point(1, 7), 1),
        Task(Point(6, 2), 2),
        Task(Point(5, 9), 3),
        Task(Point(1, 1), 4),
    ]
    uavs = [
        UAV(Point(0, 2), [], 0),
        UAV(Point(0, 1), [], 1),
        UAV(Point(1, 8), [], 2),
        UAV(Point(1, 6), [], 3),
        UAV(Point(2, 1), [], 4),
    ]
    return System(copy.deepcopy(uavs), copy.deepcopy(tasks))


def _build_sys2() -> System:
    uavs = [UAV(Point(0, random.randint(0, 100)), [], i) for i in range(10)]
    tasks = [Task(Point(random.randint(0, 100), random.randint(0, 100)), i) for i in range(15)]
    return System(copy.deepcopy(uavs), copy.deepcopy(tasks))


def _build_sys3() -> System:
    uavs = [UAV(Point(random.randint(0, 100), random.randint(0, 100)), [], i) for i in range(10)]
    tasks = [Task(Point(random.randint(0, 100), random.randint(0, 100)), i) for i in range(15)]
    return System(copy.deepcopy(uavs), copy.deepcopy(tasks))


def _build_sys4() -> System:
    uavs = [UAV(Point(i * 20, 0), [], i) for i in range(50)]
    tasks = [Task(Point(random.randint(0, 1000), random.randint(0, 1000)), i) for i in range(70)]
    return System(copy.deepcopy(uavs), copy.deepcopy(tasks))


_BUILDERS = (_build_sys1, _build_sys2, _build_sys3, _build_sys4)


def load_system(benchmark: int) -> System:
    """Return a deep-copyable fresh system for the given benchmark (1–4)."""
    return _BUILDERS[benchmark - 1]()
