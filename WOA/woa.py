import numpy as np
from classes import *
from operator import itemgetter
from woa_parameters import *
from typing import List
from Test_cases import sys1, sys2, sys3, sys4
import random 
# initialise whales, each whale is list of UAVs having path, each whale is a potential solution
#
# rmv obj_func
# understand prey and rmv if not needed
# weight modifying for number of uavs
#
#
#
#

class WOA:
    def __init__(self, n_whale, spiral_constant, n_iter, map_dim, sys, n_uavs): #, lb, ub):
        # self.obj_func = obj_func
        self.n_whale = n_whale
        self.spiral_constant = spiral_constant
        self.n_iter = n_iter
        self.map_dim = map_dim
        
        self.whales = []
        # self.prey = []

        self.uavs = []
        self.tasks = []
        self.sys:System = sys

    def init_whales(self):
        self.whales:List[dict] = []
        for _ in range(self.n_whale):
            self.whales.append({'uavs': self.sys.initRandomSoln(), 'fitness':np.nan})
            self.whales[-1]['fitness'] = System.get_fitness(self.whales[-1]['uavs'], 100)
            assert type(self.whales[-1]['uavs']) is list
            

    def get_best_whale(self):
        return min(self.whales, key=itemgetter('fitness'))

    # def init_prey(self):
    #     tmp = [np.random.uniform(self.lb, self.ub, size=(len(self.lb),))]
    #     self.prey['position'] = np.array(tmp)
    #     self.prey['fitness'] = self.obj_func(self.prey['position'])

    # def update_prey(self):
    #     if self.whale['fitness'].min() < self.prey['fitness'][0]:
    #         self.prey['position'][0] = self.whale['position'][self.whale['fitness'].argmin()]
    #         self.prey['fitness'][0] = self.whale['fitness'].min()

    def search_prey(self, idx, A:Point, C:Point):
        random_whale = self.whales[random.choice([w_idx for w_idx in range(n_whale) if w_idx != idx])]
        random_whale_uavs:List[UAV] = random_whale['uavs']
        uavs:List[UAV] = self.whales[idx]['uavs']
        for uav_idx,uav in enumerate(uavs):
            for position_idx,position in enumerate(uav.path):
                if (position_idx+1)%(no_path_points+1) == 0:
                    continue
                A = 2 a
                position_update = self.search_prey_position(position, random_whale_uavs[uav_idx].path[position_idx], A, C)
                uav.path[position_idx] = position_update
        
        
        
        d = np.abs(C[..., np.newaxis] * random_whale - self.whale['position'][idx])
        self.whale['position'][idx] = np.clip(random_whale - A[..., np.newaxis] * d, self.lb, self.ub)
    
    def search_prey_position(self, position:Point, position_rand:Point, A:Point, C:Point):
        D = C.element_wise_mul(position_rand).sub(position)
        return position_rand.sub(A.element_wise_mul(D))

    def encircle(self, idx, A, C):
        best_whale = self.get_best_whale()
        best_whale_uavs:List[UAV] = best_whale['uavs']
        uavs:List[UAV] = self.whales[idx]['uavs']
        for uav_idx,uav in enumerate(uavs):
            for position_idx,position in enumerate(uav.path):
                if (position_idx+1)%(no_path_points+1) == 0:
                    continue
                
                position_update = self.encircle_position(position, best_whale_uavs[uav_idx].path[position_idx], A=Point(), C=)

        d = np.abs(C[..., np.newaxis] * self.prey['position'] - self.whale['position'][idx])
        self.whale['position'][idx] = np.clip(self.prey['position'][0] - A[..., np.newaxis] * d, self.lb, self.ub)

    def encircle_position(self, position:Point, position_star:Point, A:Point, C:Point):
        '''
        takes position, path position of best whale selected uav, C and A variables in the form of point
        returns the updated uav path position
        '''
        D = C.element_wise_mul(position_star).sub(position).abs()
        return position_star.sub(A.element_wise_mul(D))

    def spiral_update(self, idx, A, C):
        d_prime = np.abs(self.prey['position'] - self.whale['position'][idx])
        l = np.random.uniform(-1, 1, size=len(idx[0]))
        self.whale["position"][idx] = np.clip(
            d_prime * np.exp(self.spiral_constant * l)[..., np.newaxis] * np.cos(2 * np.pi * l)[..., np.newaxis]
            + self.prey["position"],
            self.lb,
            self.ub,
        )

    def optimise_for_whale(self, w_idx, i_curr):
        '''
        update variables for whales
        '''
        
        a = 2 - i_curr * (2 / self.n_iter)
        r = random.random()
        A = 2 * a * r - a ___
        C = 2 * r
        A_abs = A __

        p = random.random()
        if p < 0.5:
            if A_abs < 1: # Encircle prey
                self.encircle(w_idx, A, C__)
            else: # Search for prey
                self.search_prey(w_idx, A=, C=)
        else: # Spiral update
            self.spiral_update(w_idx, A=, C=)

        search_idx = np.where((p < 0.5) & (abs(A) > 1))
        encircle_idx = np.where((p < 0.5) & (abs(A) <= 1))
        bubbleNet_idx = np.where(p >= 0.5)
        self.search(search_idx, A[search_idx], C[search_idx])
        self.encircle(encircle_idx, A[encircle_idx], C[encircle_idx])
        self.bubble_net(bubbleNet_idx)
        self.whale['fitness'] = self.obj_func(self.whale['position'])

    def run(self):
        self.init_whales()
        
        for i_curr in range(self.n_iter):
            #print("Iteration = ", n, " f(x) = ", self.prey['fitness'][0])
            for w_idx in range(self.n_whale):
                dim = n_uavs * n_tasks * no_path_points
                # a = np.full((dim,1), 2 - i_curr * (2 / self.n_iter))
                self.optimise_for_whale(w_idx, i_curr)

            # self.update_prey()
            # f_values.append(self.prey['fitness'][0])
        optimal_x = self.prey['position'].squeeze()
        return f_values, optimal_x


if __name__ == '__main__':
    n_whale = 5
    sys = sys1
    woa = WOA(n_whale, spiral_constant, n_iter, map_dim, sys=sys)
    woa.run()
