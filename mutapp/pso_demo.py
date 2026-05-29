"""Standalone 2D particle swarm optimisation demo (path-planning toy problem)."""

from __future__ import annotations

import random
from pathlib import Path
from typing import List

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import animation

POPULATION = 100
DIMENSION = 2
POSITION_MIN = -100.0
POSITION_MAX = 100.0
GENERATIONS = 400
FITNESS_CRITERION = 1e-4


def fitness_function(x1: float, x2: float) -> float:
    f1 = x1 + 2 * -x2 + 3
    f2 = 2 * x1 + x2 - 8
    return f1**2 + f2**2


def update_velocity(particle, velocity, pbest, gbest, w_min=0.5, max_v=1.0, c=0.1):
    new_velocity = np.array([0.0 for _ in range(len(particle))])
    r1 = random.uniform(0, max_v)
    r2 = random.uniform(0, max_v)
    w = random.uniform(w_min, max_v)
    for i in range(len(particle)):
        new_velocity[i] = (
            w * velocity[i]
            + c * r1 * (pbest[i] - particle[i])
            + c * r2 * (gbest[i] - particle[i])
        )
    return new_velocity


def update_position(particle, velocity):
    return particle + velocity


def run_pso_demo(
    output_dir: Path,
    *,
    algorithm_label: str = "Particle Swarm Optimisation (PSO) — 2D demo",
    show: bool = False,
) -> dict:
    particles = [
        [random.uniform(POSITION_MIN, POSITION_MAX) for _ in range(DIMENSION)]
        for _ in range(POPULATION)
    ]
    pbest_position = particles
    pbest_fitness = [fitness_function(p[0], p[1]) for p in particles]
    gbest_index = int(np.argmin(pbest_fitness))
    gbest_position = pbest_position[gbest_index]
    velocity = [[0.0 for _ in range(DIMENSION)] for _ in range(POPULATION)]

    for t in range(GENERATIONS):
        if np.average(pbest_fitness) <= FITNESS_CRITERION:
            break
        for n in range(POPULATION):
            velocity[n] = update_velocity(
                np.array(particles[n]),
                np.array(velocity[n]),
                np.array(pbest_position[n]),
                np.array(gbest_position),
            )
            particles[n] = update_position(np.array(particles[n]), velocity[n]).tolist()
        pbest_fitness = [fitness_function(p[0], p[1]) for p in particles]
        gbest_index = int(np.argmin(pbest_fitness))
        gbest_position = pbest_position[gbest_index]

    result = {
        "best_position": gbest_position,
        "best_fitness": float(min(pbest_fitness)),
        "generations": t,
    }

    fig = plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection="3d")
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_zlabel("z")
    x = np.linspace(POSITION_MIN, POSITION_MAX, 80)
    y = np.linspace(POSITION_MIN, POSITION_MAX, 80)
    X, Y = np.meshgrid(x, y)
    Z = fitness_function(X, Y)
    ax.plot_wireframe(X, Y, Z, color="r", linewidth=0.2)
    ax.scatter3D(
        [particles[n][0] for n in range(POPULATION)],
        [particles[n][1] for n in range(POPULATION)],
        [fitness_function(particles[n][0], particles[n][1]) for n in range(POPULATION)],
        c="b",
    )
    ax.set_title(algorithm_label)
    output_dir.mkdir(parents=True, exist_ok=True)
    png = output_dir / "pso_demo_surface.png"
    fig.savefig(png, dpi=150)
    gif_path = output_dir / "pso_demo.gif"
    try:
        images = [
            ax.scatter3D(
                [particles[n][0] for n in range(POPULATION)],
                [particles[n][1] for n in range(POPULATION)],
                [fitness_function(particles[n][0], particles[n][1]) for n in range(POPULATION)],
                c="b",
            )
        ]
        anim = animation.ArtistAnimation(fig, images)
        anim.save(gif_path, writer="pillow")
    except Exception:
        gif_path = None
    if show:
        plt.show()
    else:
        plt.close(fig)
    result["plot_png"] = str(png)
    if gif_path:
        result["plot_gif"] = str(gif_path)
    return result
