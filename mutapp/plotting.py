"""Save labelled plots for each algorithm run."""

from __future__ import annotations

from pathlib import Path
from typing import List, Optional, Sequence

import matplotlib.pyplot as plt

from mutapp.benchmarks import BenchmarkSpec
from mutapp.models import Task, UAV


def _slug(text: str) -> str:
    return "".join(c if c.isalnum() else "_" for c in text).strip("_").lower()


def output_dir(base: Path, algorithm: str, benchmark: BenchmarkSpec) -> Path:
    path = base / _slug(algorithm) / f"benchmark_{benchmark.index}_{benchmark.name}"
    path.mkdir(parents=True, exist_ok=True)
    return path


def save_paths_plot(
    uavs: List[UAV],
    tasks: List[Task],
    map_dim: int,
    algorithm: str,
    benchmark: BenchmarkSpec,
    output_base: Path,
    *,
    phase: str = "path planning",
    show: bool = False,
) -> Path:
    fig, ax = plt.subplots(figsize=(8, 8))
    for task in tasks:
        ax.plot(
            task.position.x,
            task.position.y,
            marker="o",
            markersize=10,
            markerfacecolor="green",
            markeredgecolor="darkgreen",
        )
    for uav in uavs:
        xs = [uav.position.x]
        ys = [uav.position.y]
        ax.plot(uav.position.x, uav.position.y, marker="o", markersize=9, color="red")
        for point in uav.path:
            xs.append(point.x)
            ys.append(point.y)
        ax.plot(xs, ys, "-o", markersize=4, linewidth=1.2)

    ax.set_xlim(0, map_dim)
    ax.set_ylim(0, map_dim)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.grid(True, alpha=0.3)
    ax.set_title(
        f"{algorithm}\n{phase} — {benchmark.description}\n"
        f"Benchmark {benchmark.index} ({benchmark.x_map}×{benchmark.y_map})",
        fontsize=11,
    )
    out = output_dir(output_base, algorithm, benchmark) / f"{_slug(phase)}_paths.png"
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    if show:
        plt.show(block=False)
        plt.pause(0.1)
    else:
        plt.close(fig)
    return out


def save_convergence_plot(
    best_fitnesses: Sequence[float],
    algorithm: str,
    benchmark: BenchmarkSpec,
    output_base: Path,
    *,
    phase: str = "path planning",
    ylabel: str = "Best fitness",
    show: bool = False,
) -> Path:
    fig, ax = plt.subplots(figsize=(8, 4))
    ax.plot(range(1, len(best_fitnesses) + 1), list(best_fitnesses), linewidth=1.5)
    ax.set_xlabel("Iteration")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.3)
    ax.set_title(
        f"{algorithm}\n{phase} convergence — benchmark {benchmark.index} ({benchmark.name})",
        fontsize=11,
    )
    out = output_dir(output_base, algorithm, benchmark) / f"{_slug(phase)}_convergence.png"
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    if show:
        plt.show(block=False)
        plt.pause(0.1)
    else:
        plt.close(fig)
    return out


def save_sa_diagnostics_plot(
    costs: Sequence[float],
    temperatures: Sequence[float],
    algorithm: str,
    benchmark: BenchmarkSpec,
    output_base: Path,
    *,
    show: bool = False,
) -> Path:
    fig, ax1 = plt.subplots(figsize=(8, 4))
    colour = "tab:blue"
    ax1.set_xlabel("Iteration")
    ax1.set_ylabel("Best path cost", color=colour)
    ax1.plot(costs, color=colour, label="Best cost")
    ax1.tick_params(axis="y", labelcolor=colour)
    ax2 = ax1.twinx()
    colour2 = "tab:orange"
    ax2.set_ylabel("Temperature", color=colour2)
    ax2.plot(temperatures, color=colour2, label="Temperature")
    ax2.tick_params(axis="y", labelcolor=colour2)
    fig.suptitle(
        f"{algorithm}\nSimulated annealing — benchmark {benchmark.index} ({benchmark.name})",
        fontsize=11,
    )
    out = output_dir(output_base, algorithm, benchmark) / "sa_cost_temperature.png"
    fig.tight_layout()
    fig.savefig(out, dpi=150)
    if show:
        plt.show(block=False)
        plt.pause(0.1)
    else:
        plt.close(fig)
    return out


def save_task_assignment_convergence(
    values: Sequence[float],
    algorithm: str,
    benchmark: BenchmarkSpec,
    output_base: Path,
    *,
    method: str = "task assignment",
    show: bool = False,
) -> Optional[Path]:
    if not values:
        return None
    return save_convergence_plot(
        values,
        algorithm,
        benchmark,
        output_base,
        phase=method,
        ylabel="Objective value",
        show=show,
    )
