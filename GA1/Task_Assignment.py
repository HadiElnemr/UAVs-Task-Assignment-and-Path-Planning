

import math
import random
import copy
import numpy as np
from matplotlib import pyplot as plt
import time
import parameters
import classes
# from visualise import *
from typing import List


####### SIMULATED ANNEALING

def generate_new_sol(list_of_UAVs):
  UAVs=copy.deepcopy(list_of_UAVs)
  while True:
    uav_idx = random.randint(0, len(UAVs) - 1)
    uav = UAVs[uav_idx] 
    if uav.number_of_assigned_tasks() > 0:
      break
  task_idx = 0 if len(uav.list_of_tasks)==0 else random.randint(0, len(uav.list_of_tasks) - 1)
  task=uav.list_of_tasks.pop(task_idx)
  while True:
    new_uav_idx = random.randint(0, len(UAVs) - 1)
    new_uav = UAVs[new_uav_idx]
    new_task_idx = 0 if len(new_uav.list_of_tasks)==0 else random.randint(0, len(new_uav.list_of_tasks) - 1)
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


########_---------------------------------------------------------------------------------------------------------------------------

#### GENETIC ALGORITH

class child:
    def __init__(self,list_of_UAVs):
        self.list_of_UAVs=list_of_UAVs
        self.fitness=fitness(list_of_UAVs)

    def __str__(self):
        return "( "+ str(round(self.fitness,2)) +" )"
    def __repr__(self):
        return "( "+ str(round(self.fitness,2)) +" )"

    def update_fitness(self):
        self.fitness=fitness(self.list_of_UAVs)
    def show(self):
        i=1
        st=""
        for x in self.list_of_UAVs:
            st+=str(i)  + "- "
            i+=1
            st+=x.__str__() + "\n"
        st += "Fitness = " + str(round(self.fitness,2))
        print(st)

class generation:
    def __init__(self,list_of_children):
        list_of_children.sort(key=lambda x:x.fitness)
        self.list_of_children=list_of_children

    def __str__(self):
        st=""
        for ch in self.list_of_children:
            st += ch.__str__() +"\n"
        return st
    def __repr__(self):
        st=""
        for ch in self.list_of_children:
            st += ch.__str__() +"\n"
        return st

def fitness(list_of_UAVs):
    sum=0
    Weight=len(list_of_UAVs)*10
    for x in list_of_UAVs:
        sum=sum+x.distance_of_UAV()
    sum += Weight * number_of_used_UAVs(list_of_UAVs)
    return sum

def number_of_used_UAVs(list_of_UAVs):
    sum=0
    for x in list_of_UAVs:
      if len(x.list_of_tasks) > 0:
        sum +=1
    return sum

def generate_random_generation(sys1,number_of_population):
    g=[]
    for i in range(number_of_population):
        uavs=copy.deepcopy(sys1.list_of_UAVs)
        classes.assign_random_tasks(sys1.list_of_tasks, uavs)
        ch=child(uavs)
        g.append(ch)
    return generation(g)





def is_task_equal(t1,t2):
    return (t1.position.x== t2.position.x)&(t1.position.y==t2.position.y)
def mutate(sys, parent):
    ch=copy.deepcopy(sys.list_of_UAVs)
    r1=random.randint(0,len(sys.list_of_UAVs)-1)
    r2=random.randint(0,len(sys.list_of_UAVs)-1)
    for i in range(len(sys.list_of_UAVs)): ##### here, it copy the whole uav, not only the tasks assigned to it, I have to handle it
        if (i==r1):
            ch[i].list_of_tasks=parent.list_of_UAVs[r2].list_of_tasks
        elif (i == r2):
            ch[i].list_of_tasks = parent.list_of_UAVs[r1].list_of_tasks
        else:
            ch[i].list_of_tasks=parent.list_of_UAVs[i].list_of_tasks
    return child(ch)



def crossover(sys,parent1,parent2):
    ch1=copy.deepcopy(sys.list_of_UAVs)
    ch2=copy.deepcopy(sys.list_of_UAVs)


    #for ch1 from parent 1 and 2
    while True:
        uav_idx = random.randint(0, len(ch1) - 1)
        uav = parent2.list_of_UAVs[uav_idx]
        if uav.number_of_assigned_tasks() > 0:
            break
    # print(uav_idx)
    task_idx = random.randint(0,len(uav.list_of_tasks)-1)
    # print(task_idx)
    task_parent2_to_ch1=uav.list_of_tasks[task_idx]
    # print(task_parent2_to_ch1.position)

    ch1[uav_idx].list_of_tasks.append(task_parent2_to_ch1)
    for i in range(len(parent1.list_of_UAVs)):
        for j in range(len(parent1.list_of_UAVs[i].list_of_tasks)):
            if not is_task_equal(parent1.list_of_UAVs[i].list_of_tasks[j],task_parent2_to_ch1):
                ch1[i].list_of_tasks.append(parent1.list_of_UAVs[i].list_of_tasks[j])


    # for ch2 from parent 2 and 1

    while True:
        uav_idx = random.randint(0, len(ch1) - 1)
        uav = parent1.list_of_UAVs[uav_idx]
        if uav.number_of_assigned_tasks() > 0:
            break
    # print(uav_idx)
    task_idx = random.randint(0, len(uav.list_of_tasks) - 1)
    # print(task_idx)
    task_parent1_to_ch2 = uav.list_of_tasks[task_idx]
    # print(task_parent1_to_ch2.position)

    ch2[uav_idx].list_of_tasks.append(task_parent1_to_ch2)
    for i in range(len(parent2.list_of_UAVs)):
        for j in range(len(parent2.list_of_UAVs[i].list_of_tasks)):
            if not is_task_equal(parent2.list_of_UAVs[i].list_of_tasks[j], task_parent1_to_ch2):
                ch2[i].list_of_tasks.append(parent2.list_of_UAVs[i].list_of_tasks[j])

    return child(ch1),child(ch2)



def GA_TaskAssignment(sys,percent_elite,percent_crossover,percent_mutant,number_of_population,number_of_generations):
    EliteN=int(percent_elite*number_of_population)
    CrossoverN=int(percent_crossover*number_of_population)
    MutantN=number_of_population-EliteN-CrossoverN
    print(EliteN,CrossoverN,MutantN)
    prev_g=generate_random_generation(sys,number_of_population)
    number_of_generations-=1
    print(prev_g)
    for i in range(number_of_generations):

        print(i)
        curr_g=[]
        for j in range(EliteN):
            curr_g.append(prev_g.list_of_children[j])
            # print("elite")
        crossovered=CrossoverN
        for j in range(1,EliteN+CrossoverN):

            result= crossover(sys,prev_g.list_of_children[0],prev_g.list_of_children[j])
            ch1=result[0]
            ch2=result[1]
            curr_g.append(ch1)
            curr_g.append(ch2)
            crossovered-=2
            # print("crossover")
            if(crossovered<0):
                break
        for j in range(MutantN):
            ch=mutate(sys,prev_g.list_of_children[number_of_population-1-j])
            curr_g.append(ch)
            # print(number_of_population-1-j)
            # print("mutant")
        prev_g=generation(curr_g)
        print(prev_g)
    
    print(prev_g.list_of_children[0].list_of_UAVs[0])
    # return prev_g.list_of_children[0].list_of_UAVs








