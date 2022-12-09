import math
import random
import copy
import numpy as np

import classes
import plotly.graph_objects as go

# SIMULATED ANNEALING

def generate_new_sol(list_of_UAVs):
    #This function to generate a new solution for the SA Algorithm
    #here we take N number of tasks from a random uav and assign this task to a new random uav
    #N here will be equal number_of_uavs/5, like if we have 50 UAV then we will reassign 10 tasks per new solution.
  UAVs=copy.deepcopy(list_of_UAVs)
  for k in range(0, int(math.ceil(len(list_of_UAVs) / 5))):
      while True:
        uav_idx = random.randint(0, len(UAVs) - 1)  ## select a random position of uav to take a task from it
        uav = UAVs[uav_idx]
        if uav.number_of_assigned_tasks() > 0:  ## to make sure that I can take a task from it
          break
      task_idx = 0 if len(uav.list_of_tasks)==0 else random.randint(0, len(uav.list_of_tasks) - 1) ## select a random task from this uav
      task=uav.list_of_tasks.pop(task_idx)
      while True:
        new_uav_idx = random.randint(0, len(UAVs) - 1) ## select a new random uav to assign this task to it
        new_uav = UAVs[new_uav_idx]
        new_task_idx = 0 if len(new_uav.list_of_tasks)==0 else random.randint(0, len(new_uav.list_of_tasks) - 1) ## select a random position the new task
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


def simulated_annealing_TaskAssignment(sys, T0, Tf, beta):
    sys.update_UAVs(classes.assign_random_tasks(sys.list_of_tasks, sys.list_of_UAVs))
    i=1
    T=T0- beta*i # update the temperature linerally
    print("The initial objective value = ",sys.best_Obj)
    bests=[sys.best_Obj]
    while T >= Tf:
        sys.update_candidate()   #generate a new solution and save it in candidate
        diff=sys.candidate_Obj - sys.best_Obj # compare the candidate solution with the current solution I have
        if diff < 0 :
            sys.update_UAVs(copy.deepcopy(sys.candidate))  # if the candidate is better, the I will take it
        else:
            metropolis = math.exp(-diff / T) # and if not then I will caluculate the p

            if metropolis > random.random(): # then compare the p with a random value r
                sys.update_UAVs(copy.deepcopy(sys.candidate))
        i+=1
        T=T0 - beta*i
        bests.append(sys.best_Obj)

    print("The best objective value = ",sys.best_Obj)
    ## to plot the convergence plot
    fig1 = go.Figure(data=go.Scatter(x=np.arange(0,i), y=bests, mode="lines"))
    fig1.update_layout(
        title="Convergence Plot Of Task Assignment ",
        xaxis_title="Iteration Number",
        yaxis_title="Objective Function Value"
     )
    fig1.show()

