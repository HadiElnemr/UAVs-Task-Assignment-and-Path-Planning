import numpy as np
from classes import *

# initialise whales, each whale is list of UAVs having path, each whale is a potential solution
#
# rmv obj_func
#
#
#

class WOA:
    def __init__(self, obj_func, n_whale, spiral_constant, n_iter,
                lb, ub):
        self.obj_func = obj_func
        self.n_whale = n_whale
        self.spiral_constant = spiral_constant
        self.n_iter = n_iter
        self.lb = lb
        self.ub = ub
        self.whales = []
        self.prey = []
        self.uavs = []
        self.tasks = []

    def init_whales(self):
        sys = System(self.uavs, )
        self.whales = []

        for i in range(self.n_whale):
            self.whales.append()
        # tmp = [np.random.uniform(self.lb, self.ub, size=(len(self.lb),)) for i in range(self.n_whale)]
        
        # self.whale['position'] = np.array(tmp)
        self.whale['fitness'] = self.obj_func(self.whale['position'])

    def init_prey(self):
        tmp = [np.random.uniform(self.lb, self.ub, size=(len(self.lb),))]
        self.prey['position'] = np.array(tmp)
        self.prey['fitness'] = self.obj_func(self.prey['position'])

    def update_prey(self):
        if self.whale['fitness'].min() < self.prey['fitness'][0]:
            self.prey['position'][0] = self.whale['position'][self.whale['fitness'].argmin()]
            self.prey['fitness'][0] = self.whale['fitness'].min()

    def search(self, idx, A, C):
        random_whale = self.whale['position'][np.random.randint(low=0, high=self.n_whale,
                                                                size=len(idx[0]))]
        d = np.abs(C[..., np.newaxis] * random_whale - self.whale['position'][idx])
        self.whale['position'][idx] = np.clip(random_whale - A[..., np.newaxis] * d, self.lb, self.ub)

    def encircle(self, idx, A, C):
        d = np.abs(C[..., np.newaxis] * self.prey['position'] - self.whale['position'][idx])
        self.whale['position'][idx] = np.clip(self.prey['position'][0] - A[..., np.newaxis] * d, self.lb, self.ub)

    def bubble_net(self, idx):
        d_prime = np.abs(self.prey['position'] - self.whale['position'][idx])
        l = np.random.uniform(-1, 1, size=len(idx[0]))
        self.whale["position"][idx] = np.clip(
            d_prime * np.exp(self.spiral_constant * l)[..., np.newaxis] * np.cos(2 * np.pi * l)[..., np.newaxis]
            + self.prey["position"],
            self.lb,
            self.ub,
        )

    def optimize(self, a):

        p = np.random.random(self.n_whale)
        r1 = np.random.random(self.n_whale)
        r2 = np.random.random(self.n_whale)
        A = 2 * a * r1 - a
        C = 2 * r2
        search_idx = np.where((p < 0.5) & (abs(A) > 1))
        encircle_idx = np.where((p < 0.5) & (abs(A) <= 1))
        bubbleNet_idx = np.where(p >= 0.5)
        self.search(search_idx, A[search_idx], C[search_idx])
        self.encircle(encircle_idx, A[encircle_idx], C[encircle_idx])
        self.bubble_net(bubbleNet_idx)
        self.whale['fitness'] = self.obj_func(self.whale['position'])

    def run(self):
        self.init_whales()
        self.init_prey()
        f_values = [self.prey['fitness'][0]]
        for n in range(self.n_iter):
            #print("Iteration = ", n, " f(x) = ", self.prey['fitness'][0])
            a = 2 - n * (2 / self.n_iter)
            self.optimize(a)
            self.update_prey()
            f_values.append(self.prey['fitness'][0])
        optimal_x = self.prey['position'].squeeze()
        return f_values, optimal_x