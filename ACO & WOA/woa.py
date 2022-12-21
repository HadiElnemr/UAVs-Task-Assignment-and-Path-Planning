import numpy as np
from classes import *
from operator import itemgetter
from woa_parameters import *
from typing import List
from Test_cases import sys1, sys2, sys3, sys4
from matplotlib import pyplot as plt
import random 
from plotting import plot_fitnesses, plot_paths
from statistics import mean, median, stdev
from ACO.Task_Assignment import ACO_TaskAssignment
import timeit

# from ACO.classes import UAV

class WOA:
    def __init__(self, n_whale, spiral_constant, n_iter, map_dim, sys:System):
        self.n_whale = n_whale
        self.spiral_constant = spiral_constant
        self.n_iter = n_iter
        self.map_dim = map_dim
        
        self.whales:List[dict] = []

        self.sys:System = sys
        self.uavs = sys.list_of_UAVs
        self.tasks = sys.list_of_tasks

    def init_whales(self):
        self.whales = []
        for _ in range(self.n_whale):
            self.whales.append({'uavs': self.sys.initRandomSoln(), 'fitness':np.nan})
            self.whales[-1]['fitness'] = System.get_fitness(self.whales[-1]['uavs'], 100)
            assert type(self.whales[-1]['uavs']) is list
            assert type(self.whales[-1]['fitness']) is float
            
    def get_best_whale(self):
        return min(self.whales, key=itemgetter('fitness'))
    
    def cap_position(position, map_dim):
        if position.x > map_dim:
            position.x = map_dim - 0.1
        if position.y > map_dim:
            position.y = map_dim - 0.1
        if position.x < 0:
            position.x = 0 + 0.1
        if position.y < 0:
            position.y = 0 + 0.1
    
    def encircle(self, idx, A, C):
        best_whale = self.get_best_whale()
        best_whale_uavs:List[UAV] = best_whale['uavs']
        uavs:List[UAV] = self.whales[idx]['uavs']
        for uav_idx,uav in enumerate(uavs):
            for position_idx,position in enumerate(uav.path):
                if (position_idx+1)%(no_path_points+1) == 0:
                    continue
                A_np = A[uav_idx][position_idx]
                C_np = C[uav_idx][position_idx]
                A_position = Point(A_np[0], A_np[1])
                C_position = Point(C_np[0], C_np[1])

                position_update = self.encircle_position(position, best_whale_uavs[uav_idx].path[position_idx], A_position, C_position)
                WOA.cap_position(position_update, self.map_dim)
                uav.path[position_idx] = position_update

    def encircle_position(self, position:Point, position_star:Point, A:Point, C:Point):
        '''
        takes position, path position of best whale selected uav, C and A variables in the form of point
        returns the updated uav path position
        '''
        D = C.element_wise_mul(position_star).sub(position).abs()
        return position_star.sub(A.element_wise_mul(D))

    def search_prey(self, idx, A, C):
        random_whale = self.whales[random.choice([w_idx for w_idx in range(n_whale) if w_idx != idx])]
        random_whale_uavs:List[UAV] = random_whale['uavs']
        uavs:List[UAV] = self.whales[idx]['uavs']
        for uav_idx,uav in enumerate(uavs):
            for position_idx,position in enumerate(uav.path):
                if (position_idx+1)%(no_path_points+1) == 0:
                    continue
                A_np = A[uav_idx][position_idx]
                C_np = C[uav_idx][position_idx]
                A_position = Point(A_np[0], A_np[1])
                C_position = Point(C_np[0], C_np[1])

                position_update = self.search_prey_position(position, random_whale_uavs[uav_idx].path[position_idx], A_position, C_position)
                WOA.cap_position(position_update, self.map_dim)
                uav.path[position_idx] = position_update
    
    def search_prey_position(self, position:Point, position_rand:Point, A:Point, C:Point):
        D = C.element_wise_mul(position_rand).sub(position)
        return position_rand.sub(A.element_wise_mul(D))

    def spiral_update(self, idx, A, C):
        best_whale = self.get_best_whale()
        best_whale_uavs:List[UAV] = best_whale['uavs']
        uavs:List[UAV] = self.whales[idx]['uavs']
        
        for uav_idx,uav in enumerate(uavs):
            for position_idx,position in enumerate(uav.path):
                if (position_idx+1)%(no_path_points+1) == 0:
                    continue
                position_update = self.spiral_update_position(position, best_whale_uavs[uav_idx].path[position_idx])
                WOA.cap_position(position_update, self.map_dim)
                uav.path[position_idx] = position_update

    def spiral_update_position(self, position:Point, position_star:Point):
        D_prime = position_star.sub(position).abs()
        l = random.random() * 2 - 1   # -1<l<1 
        exp_calc = math.exp(self.spiral_constant * l)
        cos_calc = math.cos(2*math.pi*l)
        return D_prime.element_wise_mul(Point(x=exp_calc, y=exp_calc)).element_wise_mul(Point(x=cos_calc,y=cos_calc)).add(position_star)

    def update_whale(self, w_idx, i_curr):
        '''
        update variables for whales
        '''
        prev = self.whales[w_idx]['fitness']
        a = 2 - i_curr * (2 / self.n_iter)
        uavs = self.whales[w_idx]['uavs']
        r = []
        A = []
        C = []
        A_vals = np.array([])
        for uav in uavs:
            r_uav = []
            A_uav = []
            C_uav = []
            for p_idx,position in enumerate(uav.path):
                r_uav.append(np.random.rand(2))
                A_uav.append(2 * a * r_uav[-1] - a)
                C_uav.append(2 * r_uav[-1])
                if (p_idx+1)%(no_path_points+1) == 0:
                    continue
                A_vals = np.append(A_vals, A_uav[-1])

            r.append(r_uav)
            A.append(A_uav)
            C.append(C_uav)

        A_abs = np.linalg.norm(A_vals)
        p = random.random()

        if p < 0.5:
            if A_abs < 1: 
            # Encircle prey
                self.encircle(w_idx, A, C)
            else: 
            # Search for prey
                self.search_prey(w_idx, A, C)
        else: 
            # Spiral update
            self.spiral_update(w_idx, A, C)
        self.whales[w_idx]['fitness'] = System.get_fitness(self.whales[w_idx]['uavs'], 100, self.sys)

        return self.whales[w_idx]['fitness']

    def run(self):
        self.init_whales()
        fitness_values = []
        best_fitnesses = []
        best_whales = []
        best_whale = self.get_best_whale()
        best_fitness = best_whale['fitness']
        print('initial best fitness = ', best_fitness)

        for i_curr in range(self.n_iter):
            tasks = sys.list_of_tasks
            # plot(best_whale['uavs'], tasks=tasks)
            # print("Iteration = ", i_curr, "best fitness = ", best_fitness)
            fitness_for_iteration = []

            for w_idx in range(self.n_whale):                
                fitness = self.update_whale(w_idx, i_curr)
                fitness_for_iteration.append(fitness)

                if fitness < best_fitness:
                    best_fitness = fitness
                    best_whale_idx = w_idx
            
            best_whale = self.get_best_whale()
            best_whales.append(copy.deepcopy(best_whale))
            best_fitness = best_whale['fitness']
            best_fitnesses.append(best_fitness)
            fitness_values.append(fitness_for_iteration)
        
        best_whale = self.get_best_whale()
        print('final best fitness =', best_whale['fitness'])
        
        return fitness_values, best_fitnesses, best_whale['fitness'], best_whale, best_whales

if __name__ == '__main__':
    n_whale = 100
    n_iter = 100
    map_dim = None
    x_map = map_dim
    y_map = map_dim

    map_param = [(10, 10), (100, 100), (100, 100), (1000, 1000)]
    woa_param = [(40,5,100),(50,4,200),(50,5,200),(100,3,200)] #n_whale, spiral_constant, n_iter

    sys_no = int(input('Input Benchmark number: ')) # 1->4
    
    # params of test cases
    systems = [sys1, sys2, sys3, sys4]
    sys = copy.deepcopy(systems[sys_no - 1])
    x_map = map_param[sys_no - 1][0]
    y_map = map_param[sys_no - 1][1]
    map_dim = x_map
    n_whale, spiral_constant, n_iter = woa_param[sys_no - 1]


    
    tasks = sys.list_of_tasks

    costs = []
    times = []
    for _ in range(1):
        print('iteration no', _)
        # start = timeit.default_timer()
        sys = copy.deepcopy(systems[sys_no - 1])
        
        ### ACO params
        number_of_ants = 10
        number_of_iterations = 50 if (sys_no ) < 3 else 100
        initial_phermone = 0.5
        rho = 0.5
        alpha = 0.7
        beta = 0.4
        Q = 10
        sys.list_of_UAVs = ACO_TaskAssignment(sys, number_of_ants, number_of_iterations, initial_phermone, rho, alpha, beta, Q)
        # sys.assign_random_tasks()
        ### ACO Finished

        woa = WOA(n_whale, spiral_constant, n_iter, map_dim, sys=sys)
        fitness_values, best_fitnesses, best_fitness, best_whale, best_whales = woa.run()
        costs.append(best_fitness)
        # stop = timeit.default_timer()
        # times.append(stop - start)
        for best_w in best_whales:
            plot_paths(best_w['uavs'], tasks=tasks, map_dim=map_dim)

        # plot_paths(best_whale['uavs'], tasks=tasks, map_dim=map_dim)
        # plot_fitnesses(n_iter, best_fitnesses)

    # print("Mean:", mean(costs))
    # print("Standard Deviation:", stdev(costs))
    # # print("The costs are as follows: ", costs)
    # print("Mean runtime:", mean(times))
    print("Finished")

