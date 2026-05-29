"""Dragonfly algorithm (DA) for UAV path planning."""

from __future__ import annotations

import copy
import random
from math import gamma, sin, pi
from operator import itemgetter
from typing import TYPE_CHECKING, List, Optional

import numpy as np

from mutapp.models import Point, System, UAV

if TYPE_CHECKING:
    from mutapp.live_plot import LivePathDashboard


class DA:
    def __init__(
        self,
        search_agents_no: int,
        max_iter: int,
        map_dim: int,
        sys: System,
        *,
        no_path_points: int = 1,
    ):
        self.number_of_flies = search_agents_no
        self.max_iter = max_iter
        self.map_dim = map_dim
        self.no_path_points = no_path_points

        self.flies: List[dict] = []
        self.neighbourhood_radius = 1000

        self.sys: System = sys
        self.uavs = sys.list_of_UAVs
        self.tasks = sys.list_of_tasks

    def init_flies(self):
        self.flies = []
        for _ in range(self.number_of_flies):
            self.flies.append(
                {
                    'uavs': self.sys.init_random_path_solution(
                        self.map_dim, self.map_dim, self.no_path_points
                    ),
                    'fitness': np.nan,
                }
            )
            self.flies[-1]['fitness'] = System.get_fitness(
                self.flies[-1]['uavs'], 100)
            assert type(self.flies[-1]['uavs']) is list
            assert type(self.flies[-1]['fitness']) is float

    def get_distance_fly_to_fly(self, fly1: dict, fly2: dict):
        uavs1: List[UAV] = fly1['uavs']
        uavs2: List[UAV] = fly2['uavs']
        dist = 0
        for uav_idx, uav in enumerate(uavs1):
            for point_idx, point in enumerate(uav.path):
                if (point_idx + 1) % (self.no_path_points + 1) == 0:
                    continue
                dist += point.get_distance(uavs2[uav_idx].path[point_idx])
        return dist
    
    def get_euclidean_distance_array(self, fly1: dict, fly2: dict):
        uavs1: List[UAV] = fly1['uavs']
        uavs2: List[UAV] = fly2['uavs']
        dist:List[List[Point]] = []
        for i in range(len(uavs1)):
            dist.append([])
            for j in range(len(uavs1[i].path)):
                dist[i].append(uavs1[i].path[j].get_distance(uavs2[i].path[j]))
        return dist
    
    def is_distance_far(self, distance_array):
        for i in range(len(distance_array)):
            for j in range(len(distance_array[i])):
                if distance_array[i][j] > self.neighbourhood_radius:
                    return True
        return False

    def get_best_fly(self):
        return min(self.flies, key=itemgetter('fitness'))

    def get_worst_fly(self):
        return max(self.flies, key=itemgetter('fitness'))

    def cap_position(self, position, map_dim):
        if position.x > map_dim:
            position.x = map_dim - (random.random() * map_dim/4)
        if position.y > map_dim:
            position.y = map_dim - (random.random() * map_dim/4)
        if position.x < 0:
            position.x = 0 + (random.random() * map_dim/4)
        if position.y < 0:
            position.y = 0 + (random.random() * map_dim/4)

    def separation(self, curr_fly: dict, neighbour_flies: List[dict]):
        # S = - Sum(X - Xj)

        S: List[List[Point]] = []
        # Prepare S
        for uav in curr_fly['uavs']:
            S.append([])
            for position in uav.path:
                S[-1].append(Point(0, 0))
        
        if len(neighbour_flies) < 1:
            return S

        # Access: S[uav_idx][pos_idx]

        for neighbour_fly in neighbour_flies:
            # S += X - Xj
            for uav_idx, neighbour_uav in enumerate(neighbour_fly['uavs']):
                for position_idx, neighbour_position in enumerate(neighbour_uav.path):
                    if (position_idx+1) % (self.no_path_points + 1) == 0:
                        continue

                    curr_position: Point = curr_fly['uavs'][uav_idx].path[position_idx]
                    S[uav_idx][position_idx] = S[uav_idx][position_idx].add(
                        curr_position.sub(neighbour_position))

        # Multiply result by -1
        for i in range(len(S)):
            for j in range(len(S[i])):
                S[i][j]: Point = S[i][j].mul(-1)
        return S

    def alignment(self, curr_fly: dict, neighbour_flies: List[dict]):
        # A = Sum(Vj) / N

        A: List[List[Point]] = []

        # Prepare A: for each path point for each uav, there is a velocity for that path point
        for uav in curr_fly['uavs']:
            A.append([])
            for position in uav.path:
                A[-1].append(Point(0, 0))
            assert len(A[-1]) == len(uav.path)
        assert len(A) == len(curr_fly['uavs'])

        if len(neighbour_flies) < 1:
            return A

        # Access: A[uav_idx][vel_idx]

        for neighbour_fly in neighbour_flies:
            # A += Vj
            for uav_idx, neighbour_uav in enumerate(neighbour_fly['uavs']):
                for vel_idx, neighbour_velocity in enumerate(neighbour_uav.DA_velocities):
                    if (vel_idx+1) % (self.no_path_points + 1) == 0:
                        continue
                    A[uav_idx][vel_idx] = A[uav_idx][vel_idx].add(
                        neighbour_velocity)

        # Divide result by N: number of flies
        for i in range(len(A)):
            for j in range(len(A[i])):
                A[i][j]: Point = A[i][j].mul(1/len(neighbour_flies))
        return A

    def cohesion(self, curr_fly: dict, neighbour_flies: List[dict]):
        # C = Sum(Xj) / N - X

        C: List[List[Point]] = []
        # Prepare C
        for uav in curr_fly['uavs']:
            C.append([])
            for position in uav.path:
                C[-1].append(Point(0, 0))

        if len(neighbour_flies) < 1:
            return C

        # Access: C[uav_idx][pos_idx]

        for neighbour_fly in neighbour_flies:
            # C += Xj
            for uav_idx, neighbour_uav in enumerate(neighbour_fly['uavs']):
                for position_idx, neighbour_position in enumerate(neighbour_uav.path):
                    if (position_idx+1) % (self.no_path_points + 1) == 0:
                        continue

                    C[uav_idx][position_idx] = C[uav_idx][position_idx].add(
                        neighbour_position)

        # Divide result by N: number of flies
        for i in range(len(C)):
            for j in range(len(C[i])):
                C[i][j]: Point = C[i][j].mul(1/len(neighbour_flies))

        # Subtract current fly position from result
        # i.e. Result minus current fly position 
        for i in range(len(C)):
            for j in range(len(C[i])):
                C[i][j]:Point = C[i][j].sub(curr_fly['uavs'][i].path[j])
        return C

    def attraction(self, curr_fly: dict, food: dict):
        # F = X+ - X

        F: List[List[Point]] = []
        # Prepare F
        for uav in curr_fly['uavs']:
            F.append([])
            for position in uav.path:
                F[-1].append(Point(0, 0))

        # Access: F[uav_idx][pos_idx]

        for uav_idx, food_uav in enumerate(food['uavs']):
            for position_idx, food_position in enumerate(food_uav.path):
                if (position_idx+1) % (self.no_path_points + 1) == 0:
                    continue

                curr_position: Point = curr_fly['uavs'][uav_idx].path[position_idx]
                F[uav_idx][position_idx] = F[uav_idx][position_idx].add(
                    food_position.sub(curr_position))
        return F

    def distraction(self, curr_fly: dict, enemy: dict):
        # E = X- + X

        E: List[List[Point]] = []
        # Prepare E
        for uav in curr_fly['uavs']:
            E.append([])
            for position in uav.path:
                E[-1].append(Point(0, 0))

        # Access: E[uav_idx][pos_idx]

        for uav_idx, enemy_uav in enumerate(enemy['uavs']):
            for position_idx, enemy_position in enumerate(enemy_uav.path):
                if (position_idx+1) % (self.no_path_points + 1) == 0:
                    continue

                curr_position: Point = curr_fly['uavs'][uav_idx].path[position_idx]
                E[uav_idx][position_idx] = E[uav_idx][position_idx].add(
                    enemy_position.add(curr_position))
        return E

    def levy(self):
        beta=3/2;
        # Eq. (3.10)
        sigma = (gamma(1+beta) * sin(pi*beta/2) / (gamma((1+beta)/2)*beta*2**((beta-1)/2))) ** (1/beta)
        u=np.random.randn()*sigma;
        v=np.random.randn();
        step=u/abs(v)**(1/beta);

        # Eq. (3.9)
        return 0.01*step;

    def update_fly(self, fly_idx, i_curr, s: float, a: float, c: float, f: float, e: float, w:float, food: dict, enemy: dict):
        '''
        update variables for flies

        inputs: fly_idx, i_curr:iteration number, s, a, c, f, e
        description: updates decision variable of a fly
        outputs: fitness of updated fly
        '''

        # find the neighbouring solutions
        neighbours_no = 0
        neighbour_flies = []
        for f2_idx, fly in enumerate(self.flies):
            if f2_idx == fly_idx:
                continue
            if self.is_distance_far(distance_array=self.get_euclidean_distance_array(self.flies[fly_idx], self.flies[f2_idx])):
                neighbours_no += 1
                neighbour_flies.append(self.flies[f2_idx])
            # if self.get_distance_fly_to_fly(self.flies[fly_idx], self.flies[f2_idx]) < self.neighbourhood_radius:
            #     neighbours_no += 1
            #     neighbour_flies.append(self.flies[f2_idx])

        ###### Separation ######
        S = self.separation(self.flies[fly_idx], neighbour_flies)
        ###### Alignment ######
        A = self.alignment(self.flies[fly_idx], neighbour_flies)
        ###### Cohesion ######
        C = self.cohesion(self.flies[fly_idx], neighbour_flies)
        ###### Attraction ######
        F = self.attraction(self.flies[fly_idx], food)
        ###### Distraction ######
        E = self.distraction(self.flies[fly_idx], enemy)

        curr_uavs:List[UAV] = self.flies[fly_idx]['uavs']
        for uav_idx, curr_uav in enumerate(curr_uavs):
            for position_idx, curr_position in enumerate(curr_uav.path):
                if (position_idx+1) % (self.no_path_points + 1) == 0:
                    continue

                curr_uav.DA_velocities[position_idx] = curr_uav.DA_velocities[position_idx].mul(w)\
                                                       .add(S[uav_idx][position_idx].mul(s))\
                                                       .add(A[uav_idx][position_idx].mul(a))\
                                                       .add(C[uav_idx][position_idx].mul(c))

                # if self.get_distance_fly_to_fly(fly1=self.flies[fly_idx], fly2=food) < self.neighbourhood_radius:
                if self.is_distance_far(distance_array=self.get_euclidean_distance_array(self.flies[fly_idx], food)):
                    if neighbours_no > 0:
                        curr_uav.DA_velocities[position_idx] = curr_uav.DA_velocities[position_idx].mul(w)\
                                                       .add(S[uav_idx][position_idx].mul(random.random()))\
                                                       .add(A[uav_idx][position_idx].mul(random.random()))\
                                                       .add(C[uav_idx][position_idx].mul(random.random()))
                        curr_uav.path[position_idx] = curr_uav.path[position_idx].add(curr_uav.DA_velocities[position_idx])
                    else:
                        curr_uav.path[position_idx] = curr_uav.path[position_idx]\
                                                    .add(curr_uav.path[position_idx].element_wise_mul(Point(self.levy(),self.levy())))
                else:
                    curr_uav.DA_velocities[position_idx] = curr_uav.DA_velocities[position_idx].mul(w)\
                                                        .add(S[uav_idx][position_idx].mul(s))\
                                                        .add(A[uav_idx][position_idx].mul(a))\
                                                        .add(C[uav_idx][position_idx].mul(c))\
                                                        .add(F[uav_idx][position_idx].mul(f))\
                                                        .add(E[uav_idx][position_idx].mul(e))
                    curr_uav.path[position_idx] = curr_uav.path[position_idx].add(curr_uav.DA_velocities[position_idx])

                self.cap_position(curr_uav.path[position_idx], self.map_dim)

        self.flies[fly_idx]['uavs'] = curr_uavs
        self.flies[fly_idx]['fitness'] = System.get_fitness(curr_uavs, 100)
        return self.flies[fly_idx]['fitness']

    def run(self, live: Optional["LivePathDashboard"] = None):
        self.init_flies()
        fitness_values = []
        best_fitnesses = []
        best_flies = []
        best_fly = self.get_best_fly()
        best_fitness = best_fly['fitness']
        print('initial best fitness = ', best_fitness)
        if live is not None:
            live.update_paths(best_fly["uavs"])
            live.update_convergence([best_fitness])
        best_fitness_repeated = best_fitness
        best_fitness_repeated_count = 0

        self.neighbourhood_radius = self.map_dim / 10

        for i_curr in range(self.max_iter):
            my_c = max(0.1 - i_curr * ((0.1-0)/(self.max_iter/2)), 0)
            s = 2 * random.random() * my_c
            a = 2 * random.random() * my_c
            c = 2 * random.random() * my_c
            f = 5 * random.random()
            e = my_c
            w = (0.9) - i_curr *((0.9-0.4)/self.max_iter)
            self.neighbourhood_radius = self.map_dim / 4 + self.map_dim * (i_curr / self.max_iter) * 2

            food = self.get_best_fly()
            enemy = self.get_worst_fly()

            fitness_for_iteration = []

            for fly_idx in range(self.number_of_flies):
                fitness = self.update_fly(fly_idx, i_curr, s, a, c, f, e, w, food, enemy)
                fitness_for_iteration.append(fitness)

                if fitness < best_fitness:
                    best_fitness = fitness

                if live is not None:
                    best_fly = self.get_best_fly()
                    live.update_paths(
                        best_fly["uavs"],
                        current_uavs=self.flies[fly_idx]["uavs"],
                        iteration=i_curr,
                    )

            best_fly = self.get_best_fly()
            best_flies.append(copy.deepcopy(best_fly))
            best_fitness = best_fly['fitness']
            best_fitnesses.append(best_fitness)
            fitness_values.append(fitness_for_iteration)
            if live is not None:
                live.update_convergence(best_fitnesses)
            if best_fitness == best_fitness_repeated:
                best_fitness_repeated_count += 1
            else:
                best_fitness_repeated_count = best_fitness
                best_fitness_repeated_count = 0
            if best_fitness_repeated_count == 20:
                break

        best_fly = self.get_best_fly()
        print('final best fitness =', best_fly['fitness'])

        return fitness_values, best_fitnesses, best_fly['fitness'], best_fly, best_flies

