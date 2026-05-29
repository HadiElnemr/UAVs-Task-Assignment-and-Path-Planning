"""Domain models for UAVs, tasks, and the MUTAPP system."""

from __future__ import annotations

import copy
import math
import random
from typing import List, Optional


class Point:
    def __init__(self, x: float, y: float) -> None:
        self.x = x
        self.y = y

    def add_shift(self, e_x: float, e_y: float) -> None:
        self.x += e_x
        self.y += e_y

    def get_distance(self, p: Point) -> float:
        return math.sqrt((self.x - p.x) ** 2 + (self.y - p.y) ** 2)

    def dot(self, p: Point) -> float:
        return self.x * p.x + self.y * p.y

    def element_wise_mul(self, p: Point) -> Point:
        return Point(self.x * p.x, self.y * p.y)

    def add(self, p: Point) -> Point:
        return Point(self.x + p.x, self.y + p.y)

    def sub(self, p: Point) -> Point:
        return Point(self.x - p.x, self.y - p.y)

    def mul(self, f: float) -> Point:
        return Point(self.x * f, self.y * f)

    def abs(self) -> Point:
        return Point(abs(self.x), abs(self.y))

    def get_distance_to_closest_on_line(self, a: Point, b: Point) -> float:
        ba = b.sub(a)
        return self.sub(a).dot(ba) / ba.dot(ba)

    @staticmethod
    def rand_position(x_max: int, y_max: int) -> Point:
        return Point(round(x_max * random.random()), round(y_max * random.random()))

    def __str__(self) -> str:
        return f"x: {self.x}, y: {self.y}"

    def __repr__(self) -> str:
        return self.__str__()


def distance(p1: Point, p2: Point) -> float:
    return p1.get_distance(p2)


class Task:
    def __init__(self, position: Point, number: Optional[int] = None) -> None:
        self.position = position
        self.number = 0 if number is None else number

    def __str__(self) -> str:
        return str(self.position)


class UAV:
    def __init__(
        self,
        position: Optional[Point] = None,
        list_of_tasks: Optional[List[Task]] = None,
        number: Optional[int] = None,
        path: Optional[List[Point]] = None,
        max_tasks: int = 3,
    ) -> None:
        self.path: List[Point] = path if path is not None else []
        self.DA_velocities: List[Point] = []
        self.number = 0 if number is None else number
        self.list_of_tasks: List[Task] = list_of_tasks if list_of_tasks is not None else []
        self.position = position
        self.max_tasks = max_tasks

    def number_of_assigned_tasks(self) -> int:
        return len(self.list_of_tasks)

    def distance_of_UAV(self) -> float:
        total = 0.0
        if self.number_of_assigned_tasks() > 0:
            total = self.position.get_distance(self.list_of_tasks[0].position)
            for i in range(1, self.number_of_assigned_tasks()):
                total += self.list_of_tasks[i - 1].position.get_distance(
                    self.list_of_tasks[i].position
                )
        return total

    def path_length(self) -> float:
        length = 0.0
        if self.number_of_assigned_tasks() > 0 and self.path:
            length = self.position.get_distance(self.path[0])
            for i in range(len(self.path) - 1):
                length += self.path[i].get_distance(self.path[i + 1])
        return length

    def __str__(self) -> str:
        tasks = " ".join(f"({t.position.x},{t.position.y})" for t in self.list_of_tasks)
        return f"Position {self.position}\nTasks : {tasks}"

    def __repr__(self) -> str:
        return self.__str__()


class System:
    def __init__(self, list_of_UAVs: List[UAV], list_of_tasks: List[Task]) -> None:
        self.list_of_UAVs = list_of_UAVs
        self.list_of_tasks = list_of_tasks
        self.Weight = 100
        self.uavs_history: list = []
        self.best_Obj = self.cost()
        self.candidate: list = []
        self.candidate_Obj: Optional[float] = None

    def cost(self) -> float:
        total = sum(u.distance_of_UAV() for u in self.list_of_UAVs)
        return total + self.Weight * self.number_of_used_UAVs()

    def candidate_cost(self) -> float:
        total = sum(u.distance_of_UAV() for u in self.candidate)
        return total + self.Weight * self._used_uavs(self.candidate)

    def number_of_used_UAVs(self) -> int:
        return self._used_uavs(self.list_of_UAVs)

    @staticmethod
    def _used_uavs(uavs: List[UAV]) -> int:
        return sum(1 for u in uavs if u.list_of_tasks)

    @staticmethod
    def number_of_used_UAVs_static(uavs: List[UAV]) -> int:
        return System._used_uavs(uavs)

    @staticmethod
    def get_fitness(uavs: List[UAV], weight: float, sys: Optional[System] = None) -> float:
        if sys is not None:
            weight = sys.Weight
        path_lengths = sum(u.path_length() for u in uavs)
        return path_lengths + weight * System._used_uavs(uavs)

    def init_random_path_solution(
        self, x_map: int, y_map: int, no_path_points: int = 1
    ) -> List[UAV]:
        ret_uavs: List[UAV] = []
        for uav in copy.deepcopy(self.list_of_UAVs):
            uav.path = []
            uav.DA_velocities = []
            for task in uav.list_of_tasks:
                for _ in range(no_path_points):
                    p = Point.rand_position(x_map, y_map)
                    uav.path.append(p)
                    uav.DA_velocities.append(copy.deepcopy(p))
                uav.path.append(task.position)
                uav.DA_velocities.append(Point(0, 0))
            ret_uavs.append(uav)
        return ret_uavs

    def initRandomSoln(self) -> None:
        """In-place path init using module defaults (GA legacy API)."""
        raise NotImplementedError("Use init_random_path_solution with explicit map size")

    def assign_random_tasks(self) -> None:
        for uav in self.list_of_UAVs:
            uav.list_of_tasks = []
        list_of_uavs = copy.copy(self.list_of_UAVs)
        tasks = copy.deepcopy(self.list_of_tasks)
        while tasks:
            task_idx = random.randint(0, len(tasks) - 1)
            task = tasks.pop(task_idx)
            uav_idx = random.randint(0, len(list_of_uavs) - 1)
            uav = list_of_uavs[uav_idx]
            if uav.number_of_assigned_tasks() < uav.max_tasks:
                list_of_uavs[uav_idx].list_of_tasks.append(task)
            else:
                tasks.append(task)

    def update_UAVs(self, new_sol: List[UAV]) -> None:
        self.list_of_UAVs = new_sol
        self.best_Obj = self.cost()

    def update_candidate(self) -> None:
        from mutapp import task_assignment

        self.candidate = task_assignment.generate_new_sol(self.list_of_UAVs)
        self.candidate_Obj = self.candidate_cost()


def assign_random_tasks_feasible(list_of_tasks: List[Task], list_of_uavs: List[UAV]) -> List[UAV]:
    from mutapp.task_assignment import is_feasible

    tasks = copy.copy(list_of_tasks)
    while tasks:
        task_idx = random.randint(0, len(tasks) - 1)
        task = tasks.pop(task_idx)
        while True:
            uav_idx = random.randint(0, len(list_of_uavs) - 1)
            list_of_uavs[uav_idx].list_of_tasks.append(task)
            if is_feasible(list_of_uavs):
                break
            list_of_uavs[uav_idx].list_of_tasks.remove(task)
    return list_of_uavs


def path_objective(uavs: List[UAV], sys: System) -> float:
    path_lengths = sum(u.path_length() for u in uavs)
    return path_lengths + sys.Weight * System._used_uavs(uavs)


def temperature_linear_update(T0: float, idx: int, beta: float) -> float:
    return T0 - beta * idx
