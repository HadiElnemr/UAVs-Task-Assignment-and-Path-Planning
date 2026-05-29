"""High-level algorithm pipelines used by CLI runners."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import List, Tuple

from mutapp.benchmarks import (
    BENCHMARKS,
    DA_PARAMS,
    HYBRID_DA_PARAMS,
    WOA_PARAMS,
    get_benchmark,
    load_system,
)
from mutapp.config import ACOParams, MapConfig, SAPathParams, SATaskParams
from mutapp.da import DA
from mutapp.ga_path import GeneticAlgorithm
from mutapp import plotting
from mutapp.models import System, UAV
from mutapp.live_plot import LivePathDashboard
from mutapp.sa_path import simulated_annealing_path
from mutapp import task_assignment
from mutapp.woa import WOA


ALGO_SA = "Simulated Annealing (SA)"
ALGO_WOA = "Whale Optimisation Algorithm (WOA)"
ALGO_DA = "Dragonfly Algorithm (DA)"
ALGO_GA = "Genetic Algorithm (GA)"
ALGO_HYBRID_ACO_WOA = "Hybrid: Ant Colony (task) + WOA (path)"
ALGO_HYBRID_ACO_DA = "Hybrid: Ant Colony (task) + DA (path)"
ALGO_PSO = "Particle Swarm Optimisation (PSO) — 2D demo"


def _make_live(
    algorithm: str,
    spec,
    tasks: list,
    *,
    no_path_points: int = 1,
    sa_diagnostics: bool = False,
) -> LivePathDashboard | None:
    return LivePathDashboard(
        algorithm,
        spec,
        tasks,
        spec.x_map,
        spec.y_map,
        no_path_points=no_path_points,
        sa_diagnostics=sa_diagnostics,
    )


def _aco_assign(sys: System, benchmark_index: int) -> List:
    params = ACOParams()
    if benchmark_index >= 3:
        params.number_of_iterations = 100
    result = task_assignment.ACO_TaskAssignment(
        sys,
        params.number_of_ants,
        params.number_of_iterations,
        params.initial_pheromone,
        params.rho,
        params.alpha,
        params.beta,
        params.Q,
    )
    if isinstance(result, tuple):
        return result[0]
    return result


def run_simulated_annealing(
    benchmark: int,
    output_dir: Path,
    *,
    show: bool = False,
    live: bool = False,
) -> dict:
    spec = get_benchmark(benchmark)
    map_cfg = MapConfig(spec.x_map, spec.y_map, no_path_points=1)
    sys = load_system(benchmark)
    live_dash = _make_live(ALGO_SA, spec, sys.list_of_tasks, sa_diagnostics=True) if live else None
    ta_history = task_assignment.simulated_annealing_task_assignment(
        sys, SATaskParams().T0, SATaskParams().Tf, SATaskParams().beta
    )
    plotting.save_task_assignment_convergence(
        ta_history, ALGO_SA, spec, output_dir, method="SA task assignment", show=show
    )
    best_uavs, best_eval, costs, temps = simulated_annealing_path(
        sys, map_cfg, SAPathParams(), live=live_dash
    )
    sys.list_of_UAVs = best_uavs
    if live_dash is not None:
        live_dash.hold_open()
    paths = [
        plotting.save_paths_plot(
            best_uavs, sys.list_of_tasks, map_cfg.map_dim, ALGO_SA, spec, output_dir, show=show
        ),
        plotting.save_sa_diagnostics_plot(costs, temps, ALGO_SA, spec, output_dir, show=show),
    ]
    return {"algorithm": ALGO_SA, "best_path_cost": best_eval, "plots": [str(p) for p in paths]}


def run_woa(
    benchmark: int,
    output_dir: Path,
    *,
    show: bool = False,
    live: bool = False,
    statistical_runs: int = 1,
) -> dict:
    spec = get_benchmark(benchmark)
    n_whale, spiral, n_iter = WOA_PARAMS[benchmark - 1]
    costs = []
    last = None
    tasks = []
    for _run_idx in range(statistical_runs):
        sys = load_system(benchmark)
        sys.assign_random_tasks()
        tasks = sys.list_of_tasks
        live_dash = _make_live(ALGO_WOA, spec, tasks) if live else None
        woa = WOA(n_whale, spiral, n_iter, spec.x_map, sys)
        _, best_fitnesses, best_fitness, best_whale = woa.run(live=live_dash)
        if live_dash is not None:
            live_dash.hold_open()
        costs.append(best_fitness)
        last = (best_fitnesses, best_whale)
    assert last is not None
    best_fitnesses, best_whale = last
    plots = [
        plotting.save_paths_plot(
            best_whale["uavs"],
            tasks,
            spec.x_map,
            ALGO_WOA,
            spec,
            output_dir,
            show=show,
        ),
        plotting.save_convergence_plot(
            best_fitnesses, ALGO_WOA, spec, output_dir, show=show
        ),
    ]
    return {
        "algorithm": ALGO_WOA,
        "best_fitness": costs[-1],
        "mean_fitness": sum(costs) / len(costs),
        "plots": [str(p) for p in plots],
    }


def run_da(
    benchmark: int,
    output_dir: Path,
    *,
    show: bool = False,
    live: bool = False,
    use_aco_task_assignment: bool = False,
) -> dict:
    spec = get_benchmark(benchmark)
    params = HYBRID_DA_PARAMS if use_aco_task_assignment else DA_PARAMS
    n_flies, n_iter = params[benchmark - 1]
    sys = load_system(benchmark)
    if use_aco_task_assignment:
        label = ALGO_HYBRID_ACO_DA
        sys.list_of_UAVs = _aco_assign(sys, benchmark)
    else:
        label = ALGO_DA
        sys.assign_random_tasks()
    live_dash = _make_live(label, spec, sys.list_of_tasks) if live else None
    da = DA(n_flies, n_iter, spec.x_map, sys)
    _, best_fitnesses, best_fitness, best_fly, _ = da.run(live=live_dash)
    if live_dash is not None:
        live_dash.hold_open()
    plots = [
        plotting.save_paths_plot(
            best_fly["uavs"], sys.list_of_tasks, spec.x_map, label, spec, output_dir, show=show
        ),
        plotting.save_convergence_plot(
            best_fitnesses, label, spec, output_dir, show=show
        ),
    ]
    return {"algorithm": label, "best_fitness": best_fitness, "plots": [str(p) for p in plots]}


def run_hybrid_aco_woa(
    benchmark: int, output_dir: Path, *, show: bool = False, live: bool = False
) -> dict:
    spec = get_benchmark(benchmark)
    n_whale, spiral, n_iter = WOA_PARAMS[benchmark - 1]
    sys = load_system(benchmark)
    sys.list_of_UAVs = _aco_assign(sys, benchmark)
    live_dash = _make_live(ALGO_HYBRID_ACO_WOA, spec, sys.list_of_tasks) if live else None
    woa = WOA(n_whale, spiral, n_iter, spec.x_map, sys)
    _, best_fitnesses, best_fitness, best_whale = woa.run(live=live_dash)
    if live_dash is not None:
        live_dash.hold_open()
    plots = [
        plotting.save_paths_plot(
            best_whale["uavs"],
            sys.list_of_tasks,
            spec.x_map,
            ALGO_HYBRID_ACO_WOA,
            spec,
            output_dir,
            show=show,
        ),
        plotting.save_convergence_plot(
            best_fitnesses, ALGO_HYBRID_ACO_WOA, spec, output_dir, show=show
        ),
    ]
    return {
        "algorithm": ALGO_HYBRID_ACO_WOA,
        "best_fitness": best_fitness,
        "plots": [str(p) for p in plots],
    }


def run_genetic_algorithm(
    benchmark: int, output_dir: Path, *, show: bool = False, live: bool = False
) -> dict:
    spec = get_benchmark(benchmark)
    map_cfg = MapConfig(spec.x_map, spec.y_map, no_path_points=1)
    sys = load_system(benchmark)
    uavs, ga_ta_history = task_assignment.ga_task_assignment(
        sys, 0.1, 0.8, 0.1, 20, 500
    )
    sys.update_UAVs(uavs)
    if ga_ta_history:
        plotting.save_task_assignment_convergence(
            ga_ta_history,
            ALGO_GA,
            spec,
            output_dir,
            method="GA task assignment",
            show=show,
        )
    ga = GeneticAlgorithm(
        sys,
        map_cfg,
        n_pop=50,
        max_iter=100,
        p_elite=0.1,
        p_crossover=0.8,
        p_mutation=0.1,
        parents_selection="SUS",
        tournament_size=20,
        mutation_selection="Worst",
        survivors_selection="Age",
    )
    live_dash = _make_live(ALGO_GA, spec, sys.list_of_tasks) if live else None

    def _on_generation(gen_idx: int) -> None:
        if live_dash is not None and ga.best_sols:
            live_dash.update_paths(ga.best_sols[-1], iteration=gen_idx)
            live_dash.update_convergence(ga.best_fvalues)

    ga.perform_algorithm(on_generation=_on_generation if live else None)
    if live_dash is not None:
        live_dash.hold_open()
    best_uavs = ga.best_sols[-1]
    plots = [
        plotting.save_paths_plot(
            best_uavs, sys.list_of_tasks, map_cfg.map_dim, ALGO_GA, spec, output_dir, show=show
        ),
        plotting.save_convergence_plot(
            ga.convergence_history, ALGO_GA, spec, output_dir, phase="path planning", show=show
        ),
    ]
    return {
        "algorithm": ALGO_GA,
        "best_fitness": ga.best_fvalues[-1],
        "plots": [str(p) for p in plots],
    }
