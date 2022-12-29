import numpy as np
from classes import *
from operator import itemgetter
from da_parameters import *
from typing import List
from Test_cases import sys1, sys2, sys3, sys4
from matplotlib import pyplot as plt
import random
from plotting import plot_fitnesses, plot_paths
from statistics import mean, median, stdev
# from ACO.Task_Assignment import ACO_TaskAssignment
import timeit

# from ACO.classes import UAV


class DA:
    def __init__(self, search_agents_no, max_iter, map_dim, sys: System):
        self.number_of_flies = search_agents_no
        # self.spiral_constant = spiral_constant
        self.max_iter = max_iter
        self.map_dim = map_dim

        self.flies: List[dict] = []
        self.neighbourhood_radius = 0

        self.sys: System = sys
        self.uavs = sys.list_of_UAVs
        self.tasks = sys.list_of_tasks

    def init_flies(self):
        self.flies = []
        for _ in range(self.number_of_flies):
            self.flies.append(
                {'uavs': self.sys.initRandomSoln(), 'fitness': np.nan})
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
                if (point_idx+1) % (no_path_points+1) == 0:
                    continue
                dist += point.get_distance(uavs2[uav_idx].path[point_idx])
        return dist

    def get_best_fly(self):
        return min(self.flies, key=itemgetter('fitness'))

    def get_worst_fly(self):
        return max(self.flies, key=itemgetter('fitness'))

    def cap_position(position, map_dim):
        if position.x > map_dim:
            position.x = map_dim - 0.1
        if position.y > map_dim:
            position.y = map_dim - 0.1
        if position.x < 0:
            position.x = 0 + 0.1
        if position.y < 0:
            position.y = 0 + 0.1

    def separation(self, curr_fly: dict, neighbour_flies: List[dict]):
        # S = - Sum(X - Xj)

        S: List[List[Point]] = []
        # Prepare S
        for uav in curr_fly['uavs']:
            S.append([])
            for position_idx, position in enumerate(uav.path):
                # if (position_idx+1) % (no_path_points+1) == 0:
                #     continue
                S[-1].append(Point(0, 0))

        # Access: S[uav_idx][pos_idx]

        for fly_idx, neighbour_fly in enumerate(neighbour_flies):
            # S += X - Xj
            for uav_idx, neighbour_uav in enumerate(neighbour_fly['uavs']):
                for position_idx, neighbour_position in enumerate(neighbour_uav.path):

                    if (position_idx+1) % (no_path_points+1) == 0:
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
            for position_idx, position in enumerate(uav.path):
                # if (position_idx+1) % (no_path_points+1) == 0:
                #     continue
                A[-1].append(Point(0, 0))
            assert len(A[-1]) == len(uav.path)
        assert len(A) == len(curr_fly['uavs'])

        # Access: A[uav_idx][vel_idx]

        for fly_idx, neighbour_fly in enumerate(neighbour_flies):
            # S += X - Xj
            for uav_idx, neighbour_uav in enumerate(neighbour_fly['uavs']):
                for vel_idx, neighbour_velocity in enumerate(neighbour_uav.DA_velocities):
                    if (vel_idx+1) % (no_path_points+1) == 0:
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
            for position_idx, position in enumerate(uav.path):
                # if (position_idx+1) % (no_path_points+1) == 0:
                #     continue
                C[-1].append(Point(0, 0))

        # Access: C[uav_idx][pos_idx]

        for fly_idx, neighbour_fly in enumerate(neighbour_flies):
            # C += Xj
            for uav_idx, neighbour_uav in enumerate(neighbour_fly['uavs']):
                for position_idx, neighbour_position in enumerate(neighbour_uav.path):

                    if (position_idx+1) % (no_path_points+1) == 0:
                        continue

                    # curr_position: Point = curr_fly['uavs'][uav_idx].path[position_idx]
                    C[uav_idx][position_idx] = C[uav_idx][position_idx].add(
                        neighbour_position)

        # Divide result by N: number of flies
        for i in range(len(C)):
            for j in range(len(C[i])):
                C[i][j]: Point = C[i][j].mul(1/len(neighbour_flies))

        # Subtract result from current fly position
        for i, uav in enumerate(C):
            for j, point in enumerate(uav.path):
                C[i][j] -= curr_fly['uavs'][i][j]
        return C

    def attraction(self, curr_fly: dict, food: dict):
        # F = X+ - X

        F: List[List[Point]] = []
        # Prepare F
        for uav in curr_fly['uavs']:
            F.append([])
            for position_idx, position in enumerate(uav.path):
                # if (position_idx+1) % (no_path_points+1) == 0:
                #     continue
                F[-1].append(Point(0, 0))

        # Access: F[uav_idx][pos_idx]

        for uav_idx, food_uav in enumerate(food['uavs']):
            for position_idx, food_position in enumerate(food_uav.path):

                if (position_idx+1) % (no_path_points+1) == 0:
                    continue

                curr_position: Point = curr_fly['uavs'][uav_idx].path[position_idx]
                F[uav_idx][position_idx] = F[uav_idx][position_idx].add(
                    food_position.sub(curr_position))
        return F

    def distraction(self, curr_fly: dict, enemy: dict):
        # E = X- + X

        E: List[List[Point]] = []
        # Prepare F
        for uav in curr_fly['uavs']:
            E.append([])
            for position_idx, position in enumerate(uav.path):
                # if (position_idx+1) % (no_path_points+1) == 0:
                #     continue
                E[-1].append(Point(0, 0))

        # Access: F[uav_idx][pos_idx]

        for uav_idx, enemy_uav in enumerate(enemy['uavs']):
            for position_idx, enemy_position in enumerate(enemy_uav.path):

                if (position_idx+1) % (no_path_points+1) == 0:
                    continue

                curr_position: Point = curr_fly['uavs'][uav_idx].path[position_idx]
                E[uav_idx][position_idx] = E[uav_idx][position_idx].add(
                    enemy_position.add(curr_position))
        return E

    def update_fly(self, fly_idx, i_curr, s: float, a: float, c: float, f: float, e: float, food: dict, enemy: dict):
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
            if self.get_distance_fly_to_fly(self.flies[fly_idx], self.flies[f2_idx]) < self.neighbourhood_radius:
                neighbours_no += 1
                neighbour_flies.append(self.flies[f2_idx])

        uavs = self.flies[fly_idx]['uavs']

        ###### Separation ######
        S = self.separation(self.flies[fly_idx], neighbour_flies)
        ###### Alignment ######
        A = self.alignment(self.flies[fly_idx], neighbour_flies)
        ###### Cohesion ######
        C = self.cohesion(self.flies[fly_idx], food)
        ###### Attraction ######
        A = self.attraction(self.flies[fly_idx], food)
        ###### Distraction ######
        D = self.distraction(self.flies[fly_idx], enemy)


    def run(self):
        self.init_flies()
        fitness_values = []
        best_fitnesses = []
        best_flies = []
        best_fly = self.get_best_fly()
        best_fitness = best_fly['fitness']
        print('initial best fitness = ', best_fitness)

        self.neighbourhood_radius = map_dim

        for i_curr in range(self.max_iter):

            my_c = max(0.1 - i_curr * ((0.1-0)/(self.max_iter/2)), 0)
            s = 2 * random.random() * my_c
            a = 2 * random.random() * my_c
            c = 2 * random.random() * my_c
            f = random.random()
            e = my_c

            self.neighbourhood_radius = map_dim/4 + map_dim * i_curr / self.max_iter * 2

            food = self.get_best_fly()
            enemy = self.get_worst_fly()

            fitness_for_iteration = []

            for fly_idx in range(self.number_of_flies):
                fitness = self.update_fly(fly_idx, i_curr, s, a, c, f, e, food, enemy)
                fitness_for_iteration.append(fitness)

                if fitness < best_fitness:
                    best_fitness = fitness
                    best_fly_idx = fly_idx

            best_fly = self.get_best_fly()
            best_flies.append(copy.deepcopy(best_fly))
            best_fitness = best_fly['fitness']
            best_fitnesses.append(best_fitness)
            fitness_values.append(fitness_for_iteration)

        best_fly = self.get_best_fly()
        print('final best fitness =', best_fly['fitness'])

        return fitness_values, best_fitnesses, best_fly['fitness'], best_fly, best_flies


if __name__ == '__main__':
    number_of_flies = 100
    n_iter = 100
    map_dim = None
    x_map = map_dim
    y_map = map_dim

    map_param = [(10, 10), (100, 100), (100, 100), (1000, 1000)]
    # number_of_flies, spiral_constant, n_iter
    woa_param = [(40, 5, 100), (50, 4, 200), (50, 5, 200), (100, 3, 200)]

    sys_no = int(input('Input Benchmark number: '))  # 1->4

    # params of test cases
    systems = [sys1, sys2, sys3, sys4]
    sys = copy.deepcopy(systems[sys_no - 1])
    x_map = map_param[sys_no - 1][0]
    y_map = map_param[sys_no - 1][1]
    map_dim = x_map
    number_of_flies, spiral_constant, n_iter = woa_param[sys_no - 1]

    tasks = sys.list_of_tasks

    costs = []
    times = []
    for _ in range(1):
        print('iteration no', _)
        # start = timeit.default_timer()
        sys = copy.deepcopy(systems[sys_no - 1])

        # ACO params
        number_of_ants = 10
        number_of_iterations = 50 if (sys_no) < 3 else 100
        initial_phermone = 0.5
        rho = 0.5
        alpha = 0.7
        beta = 0.4
        Q = 10
        sys.list_of_UAVs = ACO_TaskAssignment(
            sys, number_of_ants, number_of_iterations, initial_phermone, rho, alpha, beta, Q)
        # sys.assign_random_tasks()
        # ACO Finished

        woa = DA(number_of_flies, spiral_constant, n_iter, map_dim, sys=sys)
        fitness_values, best_fitnesses, best_fitness, best_fly, best_flies = woa.run()
        costs.append(best_fitness)
        # stop = timeit.default_timer()
        # times.append(stop - start)
        for best_w in best_flies:
            plot_paths(best_w['uavs'], tasks=tasks, map_dim=map_dim)

        # plot_paths(best_fly['uavs'], tasks=tasks, map_dim=map_dim)
        # plot_fitnesses(n_iter, best_fitnesses)

    # print("Mean:", mean(costs))
    # print("Standard Deviation:", stdev(costs))
    # # print("The costs are as follows: ", costs)
    # print("Mean runtime:", mean(times))
    print("Finished")
