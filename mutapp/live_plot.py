"""Real-time matplotlib dashboards (restored from legacy ``visualise.py``)."""

from __future__ import annotations

from typing import List, Optional, Sequence

import matplotlib.pyplot as plt

from mutapp.benchmarks import BenchmarkSpec
from mutapp.models import Task, UAV

_COLOURS = "cmbgryk"


def live_supported() -> bool:
    from mutapp.display import backend_is_interactive

    return backend_is_interactive()


def _is_task_node(path_index: int, no_path_points: int) -> bool:
    """Task positions sit at the end of each task segment in the path list."""
    return (path_index + 1) % (no_path_points + 1) == 0


class LivePathDashboard:
    """
    Live map of UAV routes with distinguishable intermediate waypoints.

    - Green circles: task targets (fixed)
    - Coloured squares: UAV start positions
    - Diamonds: intermediate path mid-points (these move during optimisation)
    - Dashed routes: current candidate; solid routes: best-so-far
    """

    def __init__(
        self,
        algorithm: str,
        benchmark: BenchmarkSpec,
        tasks: List[Task],
        x_map: int,
        y_map: int,
        *,
        no_path_points: int = 1,
        sa_diagnostics: bool = False,
    ) -> None:
        if not live_supported():
            raise RuntimeError(
                "Live plotting needs an interactive matplotlib backend. "
                "Unset MPLBACKEND (or use TkAgg/Qt5Agg), then pass --live."
            )
        plt.ion()
        self.algorithm = algorithm
        self.benchmark = benchmark
        self.tasks = tasks
        self.x_map = x_map
        self.y_map = y_map
        self.no_path_points = no_path_points
        self._iteration = 0

        self.fig_path, self.ax_path = plt.subplots(figsize=(9, 8))
        self._set_path_title()
        self.ax_path.set_xlim(0, x_map)
        self.ax_path.set_ylim(0, y_map)
        self.ax_path.set_xlabel("x")
        self.ax_path.set_ylabel("y")
        self.ax_path.grid(True, alpha=0.3)
        self._draw_tasks()

        self.fig_diag: Optional[plt.Figure] = None
        self.ax_cost = None
        self.ax_temp = None
        if sa_diagnostics:
            self.fig_diag, self.ax_cost = plt.subplots(figsize=(7, 4))
            self.ax_cost.set_xlabel("Iteration")
            self.ax_cost.set_ylabel("Best path cost", color="tab:blue")
            self.ax_temp = self.ax_cost.twinx()
            self.ax_temp.set_ylabel("Temperature", color="tab:orange")
            self.fig_diag.suptitle(f"{algorithm} — cost & temperature")

        self._refresh_path()

    def _set_path_title(self) -> None:
        suffix = f" — step {self._iteration}" if self._iteration else ""
        self.ax_path.set_title(
            f"{self.algorithm}\n"
            f"Live path planning{suffix} — benchmark {self.benchmark.index} "
            f"({self.benchmark.x_map}×{self.benchmark.y_map})\n"
            f"◆ mid-points (moving)  ■ UAV  ● task",
            fontsize=9,
        )

    def _draw_tasks(self) -> None:
        for task in self.tasks:
            self.ax_path.plot(
                task.position.x,
                task.position.y,
                marker="o",
                markersize=11,
                markerfacecolor="green",
                markeredgecolor="darkgreen",
                linestyle="none",
                zorder=6,
            )

    def _draw_uav_routes(
        self,
        uavs: List[UAV],
        *,
        linestyle: str,
        alpha: float,
        linewidth: float,
        label_prefix: str,
        zorder: int,
    ) -> None:
        for idx, uav in enumerate(uavs):
            if uav.number_of_assigned_tasks() == 0 and not uav.path:
                continue
            colour = _COLOURS[idx % len(_COLOURS)]
            xs = [uav.position.x]
            ys = [uav.position.y]

            self.ax_path.plot(
                uav.position.x,
                uav.position.y,
                marker="s",
                markersize=9,
                color=colour,
                markerfacecolor=colour,
                linestyle="none",
                label=f"{label_prefix} UAV {uav.number}",
                zorder=zorder + 1,
            )

            mid_x: List[float] = []
            mid_y: List[float] = []
            for p_idx, point in enumerate(uav.path):
                xs.append(point.x)
                ys.append(point.y)
                if not _is_task_node(p_idx, self.no_path_points):
                    mid_x.append(point.x)
                    mid_y.append(point.y)

            self.ax_path.plot(
                xs, ys, color=colour, linestyle=linestyle, linewidth=linewidth, alpha=alpha, zorder=zorder
            )
            if mid_x:
                self.ax_path.scatter(
                    mid_x,
                    mid_y,
                    c=colour,
                    marker="D",
                    s=42,
                    alpha=alpha,
                    edgecolors="black",
                    linewidths=0.4,
                    zorder=zorder + 2,
                )

    def update_paths(
        self,
        best_uavs: List[UAV],
        *,
        current_uavs: Optional[List[UAV]] = None,
        iteration: Optional[int] = None,
    ) -> None:
        """Redraw routes; show moving mid-points on ``current_uavs`` when provided."""
        if iteration is not None:
            self._iteration = iteration

        self.ax_path.cla()
        self._set_path_title()
        self.ax_path.set_xlim(0, self.x_map)
        self.ax_path.set_ylim(0, self.y_map)
        self.ax_path.set_xlabel("x")
        self.ax_path.set_ylabel("y")
        self.ax_path.grid(True, alpha=0.3)
        self._draw_tasks()

        if current_uavs is not None:
            self._draw_uav_routes(
                current_uavs,
                linestyle="--",
                alpha=0.45,
                linewidth=1.0,
                label_prefix="current",
                zorder=2,
            )

        self._draw_uav_routes(
            best_uavs,
            linestyle="-",
            alpha=0.95,
            linewidth=1.8,
            label_prefix="best",
            zorder=4,
        )

        if len(best_uavs) <= 8:
            self.ax_path.legend(loc="upper right", fontsize=6, ncol=2)

        self._refresh_path()

    def update_convergence(self, values: Sequence[float], *, ylabel: str = "Best fitness") -> None:
        if self.fig_diag is None:
            self.fig_diag, self.ax_cost = plt.subplots(figsize=(7, 4))
            self.ax_cost.set_xlabel("Iteration")
        assert self.ax_cost is not None
        self.ax_cost.cla()
        self.ax_cost.plot(range(1, len(values) + 1), list(values), color="tab:blue")
        self.ax_cost.set_xlabel("Iteration")
        self.ax_cost.set_ylabel(ylabel)
        self.ax_cost.set_title(f"{self.algorithm} — convergence")
        self.ax_cost.grid(True, alpha=0.3)
        self._refresh_diag()

    def update_sa_diagnostics(
        self, costs: Sequence[float], temperatures: Sequence[float]
    ) -> None:
        if self.fig_diag is None or self.ax_cost is None or self.ax_temp is None:
            return
        self.ax_cost.cla()
        self.ax_temp.cla()
        iters = range(len(costs))
        self.ax_cost.plot(iters, costs, color="tab:blue")
        self.ax_cost.set_ylabel("Best path cost", color="tab:blue")
        self.ax_temp.plot(iters, temperatures, color="tab:orange")
        self.ax_temp.set_ylabel("Temperature", color="tab:orange")
        self.ax_cost.set_xlabel("Iteration")
        self.ax_cost.grid(True, alpha=0.3)
        self._refresh_diag()

    def _refresh_path(self) -> None:
        self.fig_path.canvas.draw()
        self.fig_path.canvas.flush_events()
        plt.pause(0.02)

    def _refresh_diag(self) -> None:
        if self.fig_diag is None:
            return
        self.fig_diag.canvas.draw()
        self.fig_diag.canvas.flush_events()
        plt.pause(0.01)

    def hold_open(self) -> None:
        """Keep windows open until the user closes them."""
        plt.ioff()
        plt.show(block=True)
