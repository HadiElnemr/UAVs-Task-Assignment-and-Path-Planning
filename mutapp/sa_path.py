"""Simulated annealing for UAV path planning (mid-point placement)."""

from __future__ import annotations

import copy
import math
import random
from typing import TYPE_CHECKING, List, Optional, Tuple

from mutapp.config import MapConfig, SAPathParams
from mutapp.models import Point, System, UAV, path_objective, temperature_linear_update

if TYPE_CHECKING:
    from mutapp.live_plot import LivePathDashboard


def simulated_annealing_path(
    sys: System,
    map_cfg: MapConfig,
    params: SAPathParams,
    *,
    live: Optional["LivePathDashboard"] = None,
) -> Tuple[List[UAV], float, List[float], List[float]]:
    """Run SA path planning on the current task assignment in ``sys``."""
    x_map, y_map = map_cfg.x_map, map_cfg.y_map
    no_path_points = map_cfg.no_path_points
    beta = (params.T0 - params.Tf) / params.i_max

    uavs: List[UAV] = sys.list_of_UAVs
    for uav in uavs:
        uav.path = []
        uav.position = Point.rand_position(x_map, y_map)
        for task in uav.list_of_tasks:
            for _ in range(no_path_points):
                uav.path.append(Point.rand_position(x_map, y_map))
            uav.path.append(task.position)

    best = copy.deepcopy(uavs)
    best_eval = path_objective(uavs, sys)
    curr, curr_eval = uavs, best_eval
    costs: List[float] = []
    temperatures: List[float] = []
    t = params.T0
    if live is not None:
        live.update_paths(best, current_uavs=uavs, iteration=0)

    for iteration in range(params.i_max):
        costs.append(best_eval)
        temperatures.append(t)

        for uav in uavs:
            for idx, position in enumerate(uav.path):
                if (idx + 1) % (no_path_points + 1) == 0:
                    continue
                scaler = 0.2
                diff_x = random.random() * scaler * x_map * (1 if random.random() < 0.5 else -1)
                diff_y = random.random() * scaler * y_map * (1 if random.random() < 0.5 else -1)
                position.add_shift(diff_x, diff_y)
                position.x = max(0, min(x_map, position.x))
                position.y = max(0, min(y_map, position.y))

        new_eval = path_objective(uavs, sys)
        if new_eval < best_eval:
            best, best_eval = copy.deepcopy(uavs), new_eval

        diff_energy = new_eval - curr_eval
        metropolis = math.exp(-diff_energy / t)
        if diff_energy < 0 or random.random() < metropolis:
            curr, curr_eval = copy.deepcopy(uavs), new_eval

        if live is not None:
            live.update_paths(best, current_uavs=uavs, iteration=iteration)
            live.update_sa_diagnostics(costs, temperatures)

        print(iteration, best_eval)
        t = temperature_linear_update(params.T0, iteration + 1, beta)
        if t <= params.Tf:
            break

    return best, best_eval, costs, temperatures
