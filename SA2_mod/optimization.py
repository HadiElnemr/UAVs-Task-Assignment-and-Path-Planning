import math
import random
import copy
from matplotlib import pyplot as plt
import time
from classes import *
from visualise import *
from parameters import *
from typing import List
from Test_cases import sys1, sys2, sys3, sys4

def simulated_annealing(sys:System, x_map, y_map, T0, Tf, i_max):

  # paramaters
  shift_x, shift_y = 5, 5
  alpha = 0.7
  beta = (T0-Tf)/i_max
  no_path_points = 1

  costs = []
  temperatures = []


  
  # Assign random tasks to the UAVs
#   sys.assign_random_tasks()
  # To be changed to David's code
  # sys.assign_tasks_SA(Tf,T0,beta)
  
    # UAVs
  uavs:List[UAV] = sys.list_of_UAVs  
  
  # generate random middle points in the path of the UAV between itself and its tasks
  for uav in uavs:
    uav.position = rand_position(x_map, y_map) # initial position
    
    for task in uav.list_of_tasks: # initial path
      for i in range(no_path_points):
        uav.path.append(rand_position(x_map, y_map))
      uav.path.append(task.position)

	# evaluate the initial configuration
  # best_eval = sys.cost()
  best = uavs
  best_eval = objective(uavs, sys)
  # best_uavs_path_costs = path_objective(uavs)
  
  # current working solution
  # curr_costs_list, curr_eval = best_uavs_path_costs, best_eval
  curr, curr_eval = uavs, best_eval
  
  ##################
  # run the algorithm of i_max iterations
  t = T0
  for iter in range(i_max):
    costs += [best_eval]
    temperatures += [t]

    # make position shifts
    feasible = 1 
    for idx, uav in enumerate(uavs):
      for i, position in enumerate(uav.path):
        if (i+1)%(no_path_points+1) == 0: # If this is a task point, do not shift it
          continue
        scaler = 0.2 # number [0-1] to define scale of exploration around current solution

        diff_x = random.random() * scaler * x_map * (1 if random.random() < 0.5 else -1)  
        diff_y = random.random() * scaler * x_map * (1 if random.random() < 0.5 else -1)

        position.add_shift(diff_x, diff_y)
        if position.x < 0:
          position.x = 0
        elif position.x > x_map:
          position.x = x_map
        if position.y < 0:
          position.y = 0
        elif position.y > y_map:
          position.y = y_map
      # # evaluate uav path
      # # curr_costs_list
      # if uav.path_length() < best_uavs_path_costs[idx]:
      #   best_uavs_path_costs[idx] = uav.path_length
      #   best[idx] = copy.deepcopy(uav)
        
    # evaluate step
    # new_eval = sys.cost()
      
    new_eval = objective(uavs, sys)

    # check for new best solution
    if new_eval < best_eval:
      # store new best sys
      best, best_eval = copy.deepcopy(uavs), new_eval
      # report progress
    print(iter, best_eval)


    # difference between candidate and current point evaluation
    diff_energy = new_eval - curr_eval

    # transition probability
    metropolis = math.exp(-diff_energy / t) * feasible
    
    # Accept better solution or worse solution if metropolis < p_random
    if diff_energy < 0 or random.random() < metropolis:
      # store the new current point
      curr, curr_eval = copy.deepcopy(uavs), new_eval
      plot_paths(uavs, x_map, y_map, best)
      for task in tasks:
        ax1.plot(task.position.x,task.position.y, marker="o", markersize=15, markerfacecolor="green")
    # Else, leave as is


    plot_cost(costs, temperatures)
    
    # Update temperature
    t = temperature_linear_update(T0, iter+1, beta)
    # t = temperature_geo_update(T0, iter+1, alpha)
    
    if t <= Tf:
      break
    ax1.set_xlim(0, x_map)
    ax1.set_ylim(0, y_map)
  
  return best, best_eval

if __name__ == "__main__":
  # for _ in range(n_oabstacles):
  #   obstacles += [Obstacle(rand_position(x_map,y_map))]

  sys_no = 1
  systems = [sys1, sys2, sys3, sys4]
  sys = systems[sys_no - 1]

  sa_ta = [(10, 10, 500, 0.02), (100, 100, 100, 0.1), (100, 100, 100, 0.1), (1000, 1000, 1000, 0.1)]
  # (x_map, y_map, T0, Tf)

  # sa_pp = [(10, 10, 50), (100, 100, 100, 0.1), (100, 100, 100, 0.1), (1000, 1000, 1000, 0.1)]
  # (T0, Tf)
  
  x_map = sa_ta[sys_no][1]
  y_map = sa_ta[sys_no][2]

  # simulated_annealing_TaskAssignment(sys, 100, 0.1, 0.1)
  costs = []
  for _ in range(10):
    simulated_annealing_TaskAssignment(sys, sa_ta[sys_no][2], sa_ta[sys_no][3], 0.1)
    costs.append(simulated_annealing(sys, x_map=x_map, y_map=y_map, T0=500, Tf=0.1, i_max=200))
  print("Finished")
  while True:
    pass
