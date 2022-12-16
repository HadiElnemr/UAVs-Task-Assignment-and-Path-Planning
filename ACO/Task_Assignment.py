

import math
import random
import copy

import matplotlib.pyplot as plt
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
    prev_g=generate_random_generation(sys,number_of_population)
    number_of_generations-=1
    best=[prev_g.list_of_children[0].fitness]
    print("The initial best fitness = ",prev_g.list_of_children[0].fitness)
    for i in range(number_of_generations):
        # print(i)
        curr_g=[]
        for j in range(EliteN):
            curr_g.append(prev_g.list_of_children[j])
        crossovered=CrossoverN
        for j in range(1,EliteN+CrossoverN):

            result= crossover(sys,prev_g.list_of_children[0],prev_g.list_of_children[j])
            ch1=result[0]
            ch2=result[1]
            curr_g.append(ch1)
            curr_g.append(ch2)
            crossovered-=2
            if(crossovered<0):
                break
        for j in range(MutantN):
            ch=mutate(sys,prev_g.list_of_children[number_of_population-1-j])
            curr_g.append(ch)
        prev_g=generation(curr_g)
        best.append(prev_g.list_of_children[0].fitness)
    print("The best fitness = ",prev_g.list_of_children[0].fitness)
    fig1 = go.Figure(data=go.Scatter(x=np.arange(0, number_of_generations), y=best, mode="lines"))
    fig1.update_layout(
        title="Convergence Plot Of the Task Assignment",
        xaxis_title="Iteration Number",
        yaxis_title="Fitness Value of Best So Far"
    )
    fig1.show()
    return prev_g.list_of_children[0].list_of_UAVs

# ----------------------------------------------------------------------------------------

#ACO
class ant:
    def __init__(self,list_of_UAVs,list_of_tasks):
        self.list_of_UAVs=list_of_UAVs
        self.list_of_tasks=list_of_tasks


def ACO_TaskAssingment(sys,number_of_ants,number_of_iterations,initial_phermone,rho,alpha,beta,Q):
    uavs=copy.deepcopy(sys.list_of_UAVs)
    tasks=copy.deepcopy(sys.list_of_tasks)
    phermones=create_initial_phermones(uavs,tasks,initial_phermone)
    phermones_tasks=phermones[0]
    phermones_uavs=phermones[1]
    best_ant = None
    best_obj = math.inf
    objs=[]
    for i in range(number_of_iterations):
        phermones_flag_tasks = np.zeros((len(tasks), len(tasks)))
        phermones_flag_uavs = np.zeros((len(uavs), len(tasks)))
        ants=[]
        for j in range(number_of_ants):
            ant1=ant(copy.deepcopy(uavs),copy.deepcopy(tasks))
            possible_uavs=copy.deepcopy(ant1.list_of_UAVs)
            tasks=copy.deepcopy(ant1.list_of_tasks)
            random.shuffle(tasks)
            for task in tasks:
                update_possible_uavs(possible_uavs)
                ps=calculate_p(sys,possible_uavs,task,phermones_tasks,phermones_uavs,alpha,beta)
                r=random.random()
                uav_idx=get_idx(ps,r,possible_uavs)
                with_task=False
                ant1.list_of_UAVs[uav_idx].list_of_tasks.append(task)
                if len(ant1.list_of_UAVs[uav_idx].list_of_tasks)>1:
                    with_task=True
                if with_task:
                    t1=task.number
                    t2=ant1.list_of_UAVs[uav_idx].list_of_tasks[len(ant1.list_of_UAVs[uav_idx].list_of_tasks)-2].number
                    phermones_flag_tasks[t1][t2]+=1
                    phermones_flag_tasks[t2][t1] += 1
                else :
                    phermones_flag_uavs[uav_idx][task.number]+=1
            obj=cost(ant1)
            if obj<best_obj:
                best_ant=ant1
                best_obj=obj
            objs.append(best_obj)
            ants.append(ant1)
            print(best_obj)
        [phermones_tasks,phermones_uavs]=Evaporate_phermones(phermones_tasks,phermones_uavs,rho)
        [phermones_tasks,phermones_uavs]=Deposit_phermones(phermones_tasks,phermones_uavs,phermones_flag_tasks,phermones_flag_uavs,Q,sys)
        plt.plot(range(len(objs)), objs)

    return best_ant.list_of_UAVs




def cost(ant):
    sum=0
    for x in ant.list_of_UAVs:
        sum=sum+x.distance_of_UAV()
    return sum
def Deposit_phermones(phermones_tasks,phermones_uavs,phermones_flag_tasks,phermones_flag_uavs,Q,sys):
    for i in range(len(phermones_tasks)):
        for j in range(len(phermones_tasks[i])):
            if i!=j:
                L=get_distance(sys.list_of_tasks[i].position,sys.list_of_tasks[j].position)
                phermones_tasks[i][j]+=(Q*phermones_flag_tasks[i][j])/L


    for i in range(len(phermones_uavs)):
        for j in range(len(phermones_uavs[i])):
            L=get_distance(sys.list_of_UAVs[i].position,sys.list_of_tasks[j].position)
            phermones_uavs[i][j]+=(Q*phermones_flag_uavs[i][j])/L
    return phermones_tasks,phermones_uavs
def Evaporate_phermones(phermones_tasks, phermones_uavs,rho):
    phermones_tasks=(np.array(phermones_tasks)*(1-rho)).tolist()
    phermones_uavs=(np.array(phermones_uavs)*(1-rho)).tolist()
    return phermones_tasks,phermones_uavs
def get_idx(ps,r,possible_uavs):
    for i in range(len(ps)):
        p=ps[i]
        # print(p,r)
        if (r<p):
            return possible_uavs[i].number


def update_possible_uavs(uavs):
    for uav in uavs:
        if len(uav.list_of_tasks)==uav.max_tasks:
            uavs.remove(uav)
def calculate_p(sys,possible_uavs,task,phermones_tasks,phermones_uavs,alpha,beta):
    temp=[]
    for uav in possible_uavs:
        if len(uav.list_of_tasks)==0:
            L=get_distance(uav.position,task.position)
            ph=phermones_uavs[uav.number][task.number]
        else:
            L=get_distance(uav.list_of_tasks[len(uav.list_of_tasks)-1].position,task.position)
            ph=phermones_tasks[task.number][uav.list_of_tasks[len(uav.list_of_tasks)-1].number]
        temp.append(pow(ph,alpha)*pow(1/L,beta))
    p=[]
    for i in temp:
        p.append(i/sum(temp))
    s=0
    p_cumulative=[]
    for i in p:
        s+=i
        p_cumulative.append(s)
    return p_cumulative
def get_distance(p1, p2):
  return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

def create_initial_phermones(uavs,tasks,initial_phermone):
    phermones_tasks=np.ones((len(tasks),len(tasks)))*initial_phermone
    np.fill_diagonal(phermones_tasks,0)
    phermones_uavs=np.ones((len(uavs),len(tasks)))*initial_phermone
    return phermones_tasks,phermones_uavs





