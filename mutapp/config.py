"""Shared run-time configuration for metaheuristic path planners."""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class MapConfig:
    x_map: int
    y_map: int
    no_path_points: int = 1

    @property
    def map_dim(self) -> int:
        return self.x_map


@dataclass
class ACOParams:
    number_of_ants: int = 10
    number_of_iterations: int = 50
    initial_pheromone: float = 0.5
    rho: float = 0.5
    alpha: float = 0.7
    beta: float = 0.4
    Q: float = 10.0


@dataclass
class SAPathParams:
    T0: float = 500.0
    Tf: float = 0.1
    i_max: int = 200


@dataclass
class SATaskParams:
    T0: float = 100.0
    Tf: float = 0.1
    beta: float = 0.1
