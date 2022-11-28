

import math
import random
import copy
import numpy as np
from matplotlib import pyplot as plt
import time
from parameters import *
from classes import *
# from visualise import * 
from typing import List

def generate_new_sol(list_of_UAVs):
  UAVs=copy.deepcopy(list_of_UAVs)
  while True:
    uav_idx = random.randint(0,len(UAVs)-1)
    uav = UAVs[uav_idx] 
    if uav.number_of_assigned_tasks() > 0:
      break
  task_idx = 0 if len(uav.list_of_tasks)==0 else random.randint(0,len(uav.list_of_tasks)-1)
  task=uav.list_of_tasks.pop(task_idx)
  while True:
    new_uav_idx = random.randint(0,len(UAVs)-1)
    new_uav = UAVs[new_uav_idx]
    new_task_idx = 0 if len(new_uav.list_of_tasks)==0 else random.randint(0,len(new_uav.list_of_tasks)-1)
    new_uav.list_of_tasks.insert(new_task_idx,task)
    
    if isFeasible(UAVs):
      break
    new_uav.list_of_tasks.pop(new_task_idx)  
  
  return UAVs
    


def isFeasible(list_of_UAVs):
  for uav in list_of_UAVs:
    if uav.number_of_assigned_tasks()>uav.max_tasks:
      return False
  return True





def simulated_annealing_TaskAssignment(sys, T0, Tf, beta  ):
#   y=np.array([])
#   x=np.array([])  
  i=1  
  T=T0 - beta*i
  while T >= Tf:
    sys.update_candidate()
    diff=sys.candidate_Obj - sys.best_Obj
    if diff < 0 :
      sys.update_UAVs(copy.deepcopy(sys.candidate))
    else:
      metropolis = math.exp(-diff / T)
      if metropolis > random.random():
        sys.update_UAVs(copy.deepcopy(sys.candidate))
#     print(sys.best_Obj)
#     y=np.append(y,sys.best_Obj)
#     x=np.append(x,T)
    
    i+=1  
    T=T0 - beta*i
    
#   plt.scatter(x,y)  
#   plt.show()
      





