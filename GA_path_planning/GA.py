import numpy as np
import plotly.graph_objects as go
from classes import *
from parameters import costs, i_max, n_tasks, n_uavs, no_path_points, shift_x, shift_y, shift, tasks, uavs, x_map, y_map, dim_map
class GeneticAlgorithm:
    # replace problem by system
    def __init__(self, system:System, n_pop = 50, max_iter = 100, p_elite = 0.1, p_crossover = 0.8, p_mutation = 0.1, 
                        parents_selection = "Random", tournament_size = 5, mutation_selection = "Worst", survivors_selection = "Fitness"):
        self.system = system
        self.n_pop = n_pop
        self.max_iter = max_iter
        self.p_elite = p_elite
        self.p_crossover = p_crossover
        self.p_mutation = p_mutation
        self.parents_selection = parents_selection
        self.tournament_size = tournament_size if tournament_size < n_pop else n_pop
        self.mutation_selection = mutation_selection
        self.survivors_selection = survivors_selection
        self.gen_sols = None
        self.gen_fvalues = None
        self.gen_ages = None
        self.best_sols = None
        self.best_fvalues = None
    
    def initRandomPopulation(self):
        self.gen_sols:List[List[UAV]] = []
        self.gen_fvalues = []
        self.gen_ages = []
        self.best_sols = []
        self.best_fvalues = []
        for _ in range(self.n_pop):
            # Modify
            self.system.initRandomSoln()
            new_sol = copy.deepcopy(self.system.list_of_UAVs)
            new_fvalue = self.system.cost()
            self.gen_sols.append(new_sol)
            self.gen_fvalues.append(new_fvalue)
            self.gen_ages.append(0)
            if len(self.best_sols) == 0:
                self.best_sols.append(new_sol)
                self.best_fvalues.append(new_fvalue)
            elif (new_fvalue < self.best_fvalues[0]):
                self.best_sols[0], self.best_fvalues[0] = new_sol, new_fvalue
    
    def selectParents(self, numParents, criteria):
        gen_probs = 1 / (1 + np.square(self.gen_fvalues))
        gen_probs = gen_probs / sum(gen_probs)
        lambda_rank = 1.5 # (between 1 and 2) offspring created by best individual
        gen_ranks = list(map(lambda i: np.argwhere(np.argsort(self.gen_fvalues) == i)[0,0], np.arange(self.n_pop)))
        gen_ranks = ((2-lambda_rank) + np.divide(gen_ranks, self.n_pop-1)*(2*lambda_rank-2)) / self.n_pop
        selection_criteria = {
            "Random": lambda n: np.random.choice(self.n_pop, size=(n,), replace=False),
            "RouletteWheel": lambda n: np.random.choice(self.n_pop, size=(n,), replace=True, p=gen_probs),
            "SUS": lambda n: np.random.choice(self.n_pop, size=(n,), replace=False, p=gen_probs),
            "Rank": lambda n: np.random.choice(self.n_pop, size=(n,), replace=False, p=gen_ranks),
            "Tournament": lambda n: np.array([np.amin(list(map(lambda i: [self.gen_fvalues[i], i], 
                                                                np.random.choice(self.n_pop, size=(self.tournament_size,), replace=False))), 
                                                axis=0)[1] for _ in range(n)], dtype=int),
            "Worst": lambda n: np.argsort(self.gen_fvalues)[self.n_pop-n:]
        }
        parents_idx = selection_criteria.get(criteria, selection_criteria["Random"])(numParents)
        return parents_idx

    def crossover(self, p1_idx, p2_idx):
        # Whole Arithmetic Combination
        alpha = np.random.rand() * (0.9 - 0.7) + 0.7
        child_uavs_1 = []
        child_uavs_2 = []
        
        for uav_idx in range(n_uavs):
            # print('uav idx:', uav_idx, len(self.gen_sols[p1_idx]), len(self.gen_sols[p2_idx]))
            # # print(p1_idx,p2_idx)
            # if len(self.gen_sols[p1_idx]) < n_uavs:
            #     print("Errorr")
            # # print(len(self.gen_sols[p1_idx][uav_idx]))
            uav1, uav2 = self.gen_sols[p1_idx][uav_idx], self.gen_sols[p2_idx][uav_idx]
            
            if uav1.number_of_assigned_tasks() > 0:
                path1, path2 = self.uav_crossover(uav1, uav2)
                
                children_uavs = copy.deepcopy(uav1), copy.deepcopy(uav2)
                assert(len(children_uavs) == 2)
                
                children_uavs[0].path, children_uavs[1].path = path1, path2
                child_uavs_1.append(children_uavs[0])
                child_uavs_2.append(children_uavs[1]) 
            else:
                child_uavs_1.append(copy.deepcopy(uav1))
                child_uavs_2.append(copy.deepcopy(uav2))
            
            
        return child_uavs_1, child_uavs_2

    def uav_crossover(self, uav1:UAV, uav2:UAV):
        path1 = []
        path2 = []
        for point_idx in range(len(uav1.path)):
            if (point_idx+1)%(no_path_points+1) == 0: # If this is a task point, do not shift it
                path1.append(uav1.path[point_idx])
                path2.append(uav2.path[point_idx])
                continue
            p1, p2 = self.position_crossover(uav1.path[point_idx], uav2.path[point_idx])
            
            path1.append(p1)
            path2.append(p2)
        return path1, path2

    def position_crossover(self, p1:Point, p2:Point):
        alpha = np.random.rand() * (0.9 - 0.7) + 0.7
        return Point(x = int(alpha * p1.x + (1 - alpha) * p2.x), y = int(alpha * p1.y + (1 - alpha) * p2.y)),\
               Point(x = int((1-alpha) * p1.x + alpha * p2.x), y = int((1-alpha) * p1.y + alpha * p2.y))
    
    def mutation(self, p_idx):                
        child = []
        for uav in self.gen_sols[p_idx]:
            child.append(self.mutate_uav(uav))
        return child
    
    def mutate_uav(self, uav:UAV):
        uav_new = copy.deepcopy(uav)
        for point_idx in range(len(uav.path)):
            if (point_idx+1)%(no_path_points+1) == 0: # If this is a task point, do not shift it
                continue
            uav_new.path[point_idx] = self.mutate_position(uav.path[point_idx])
        return uav_new

    def mutate_position(self, position:Point):
        # Random noise
        r = np.random.rand()
        # child = self.gen_sols[p_idx] + r * (self.problem.ub - self.gen_sols[p_idx]) + (1 - r) * (self.problem.lb - self.gen_sols[p_idx])
        return Point(position.x + r * (x_map - position.x) + (1 - r) * (0 - position.x),
                        position.y + r * (y_map - position.x) + (1 - r) * (0 - position.y))

    def selectSurvivors(self, numSurvivors, criteria):
        selection_criteria = {
            "Age": lambda n: np.argsort(self.gen_ages)[:n],
            "Fitness": lambda n: np.argsort(self.gen_fvalues)[:n]
        }
        survivors_idx = selection_criteria.get(criteria, selection_criteria["Fitness"])(numSurvivors)
        return survivors_idx

    def perform_algorithm(self):
        self.initRandomPopulation()
        print("Best Initial Solution ", self.best_fvalues[0])
        n_crossovers = int(np.ceil(self.p_crossover * self.n_pop / 2))
        n_mutations = int(self.p_mutation * self.n_pop)
        n_elite = int(self.p_elite * self.n_pop)
        n_survivors = self.n_pop - int(self.p_crossover*self.n_pop) - n_mutations - n_elite

        for _ in range(self.max_iter):
            # print('iteration:', _)

            # Crossover and Parents Selection
            parents_idx = self.selectParents(numParents=n_crossovers*2, criteria=self.parents_selection)
            
            
            new_gen_sols = []
            new_gen_fvalues = []
            new_gen_ages = []
            for i in range(0, n_crossovers*2, 2):
                [ch1, ch2] = self.crossover(parents_idx[i], parents_idx[i+1])
                new_gen_sols.append(ch1)
                
                new_gen_fvalues.append(System.get_fitness(ch1, sys))
                new_gen_ages.append(0)
                # if len(new_gen_sols) == int(self.p_crossover * self.n_pop):
                #     break
                new_gen_sols.append(ch2)
                new_gen_fvalues.append(System.get_fitness(ch2, sys))
                new_gen_ages.append(0)
            # Mutation and Parents Selection
            parents_idx = self.selectParents(numParents=n_mutations, criteria=self.mutation_selection)
            for i in range(n_mutations):
                ch = self.mutation(parents_idx[i])
                new_gen_sols.append(ch)
                new_gen_fvalues.append(System.get_fitness(ch, sys))
                new_gen_ages.append(0)
            # Elite Members
            elite_idx = self.selectSurvivors(numSurvivors=n_elite, criteria="Fitness")
            for i in range(n_elite):
                new_gen_sols.append(self.gen_sols[elite_idx[i]])
                new_gen_fvalues.append(self.gen_fvalues[elite_idx[i]])
                new_gen_ages.append(self.gen_ages[elite_idx[i]]+1)
            # Survivors (if any)
            survivors_idx = self.selectSurvivors(numSurvivors=n_survivors, criteria=self.survivors_selection)
            for i in range(n_survivors):
                new_gen_sols.append(self.gen_sols[survivors_idx[i]])
                new_gen_fvalues.append(self.gen_fvalues[survivors_idx[i]])
                new_gen_ages.append(self.gen_ages[survivors_idx[i]]+1)
            assert(len(new_gen_sols) == self.n_pop)
            assert(len(new_gen_fvalues) == self.n_pop)
            assert(len(new_gen_ages) == self.n_pop)
            # New generation becomes current one
            self.gen_sols = new_gen_sols
            self.gen_fvalues = new_gen_fvalues
            self.gen_ages = new_gen_ages
            # update best solution reached so far
            best_idx = np.argmin(self.gen_fvalues)
            if self.gen_fvalues[best_idx] < self.best_fvalues[-1]:
                self.best_sols.append(self.gen_sols[best_idx])
                self.best_fvalues.append(self.gen_fvalues[best_idx])
            else:
                self.best_sols.append(self.best_sols[-1])
                self.best_fvalues.append(self.best_fvalues[-1])
            print(self.best_fvalues)

    def visualize(self):
        # convergence plot
        fig1 = go.Figure(data=go.Scatter(x=np.arange(0, self.max_iter), y=self.best_fvalues, mode="lines"))
        fig1.update_layout(
            title="Convergence Plot",
            xaxis_title="Iteration Number",
            yaxis_title="Fitness Value of Best So Far"
        )
        fig1.show()
        pass


if __name__ == "__main__":
  for ti in range(n_tasks):
    tasks += [Task(Point.rand_position(x_map,y_map))]
  for ui in range(n_uavs):
    uavs += [UAV(Point.rand_position(x_map,y_map))]
  sys = System(uavs, tasks)
#   sys.assign_random_tasks()

  uavs[0].position = Point(0,0)
  uavs[1].position = Point(0,2)
  uavs[2].position = Point(0,7)

  for uav in sys.list_of_UAVs:
    print(uav.number_of_assigned_tasks())

  GA = GeneticAlgorithm(sys, n_pop = 50, max_iter=100, p_elite=0.1, p_crossover=0.8, p_mutation=0.1, 
                                  parents_selection="SUS", tournament_size = 20, mutation_selection = "Worst", survivors_selection = "Age")
  
  GA.perform_algorithm()
  print(GA.best_sols[-1])
  print(GA.best_fvalues[-1])
  GA.visualize()
  
#   print("Finished")
#   while True:
#     pass

