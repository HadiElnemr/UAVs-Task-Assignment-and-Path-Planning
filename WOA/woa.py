import numpy as np
from classes import *
from operator import itemgetter
from woa_parameters import *
from typing import List
from Test_cases import sys1, sys2, sys3, sys4
import random 

# weight modifying for number of uavs
#

class WOA:
    def __init__(self, n_whale, spiral_constant, n_iter, map_dim, sys, n_uavs): #, lb, ub):
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
                # A_np = A[uav_idx][position_idx]
                # A_position = Point(A_np[0], A_np[1])
                # A = 2 a
                position_update = self.spiral_update_position(position, best_whale_uavs[uav_idx].path[position_idx])
                uav.path[position_idx] = position_update

    def spiral_update_position(self, position:Point, position_star:Point):
        D_prime = position_star.sub(position).abs()
        l = random.random()
        exp_calc = math.exp(self.spiral_constant * l)
        cos_calc = math.cos(2*math.pi*l)
        return D_prime.element_wise_mul(Point(x=exp_calc, y=exp_calc)).element_wise_mul(Point(x=cos_calc,y=cos_calc)).add(position_star)

    def optimise_for_whale(self, w_idx, i_curr):
        '''
        update variables for whales
        '''
        
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
            for p_idx,position in uav.path:
                r_uav.append(np.random.rand(2))
                A_uav.append(2 * a * r_uav[-1] - a)
                C_uav.append(2 * r_uav[-1])
                if (p_idx+1)%(no_path_points+1) == 0:
                    continue
                A_vals = np.append(A_vals, A_uav[-1])

            r.append(r_uav)
            A.append(A_uav)
            C.append(C_uav)
        
        
        # r = np.random.rand(decision_vars_dim)
        # r = random.random()
        # A = 2 * a * r - a
        # C = 2 * r
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
            # print("Iteration = ", i_curr, "f(x) = ", self.prey['fitness'][0])

            for w_idx in range(self.n_whale):
                
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
