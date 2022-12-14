import math
import random
import copy
from typing import List
from parameters import costs,i_max,n_tasks,n_uavs,no_path_points,shift_x,shift_y,tasks,uavs,x_map,y_map
from matplotlib import pyplot as plt
# from Task_Assignment import *

class Point:
  def __init__(self, x, y):
    self.x = x
    self.y = y
  
  def add_shift(self, e_x, e_y):
    # add noise shift (+ve or -ve). ex: e_x = 0.5  x = (1 + 0.5) * x
    self.x += e_x
    self.y += e_y

  def dot(self, p: 'Point'):
        return self.x * p.x + self.y * p.y

  def add(self, p: 'Point'):
      return Point(self.x + p.x, self.y + p.y)

  def sub(self, p: 'Point'):
      return Point(self.x - p.x, self.y - p.y)

  def mul(self, f: float):
      return Point(self.x * f, self.y * f)
  
  def get_distance_to_closest_on_line(self, a: 'Point', b: 'Point'): # Line ab is considered a line segment, so find the point on that segment
    ba = b.sub(a)   # b with respect to a
    ds = self.sub(a).dot(ba) / ba.dot(ba)     #   (p - a). ba  / ||ba||^2 
    return ds

  def __str__(self):
    return f"x: {self.x}, y: {self.y}"

  def __repr__(self):
    return f"x: {self.x}, y: {self.y}"
  
  def rand_position(x_max, y_max):
    return Point(round(x_max*random.random()), round(y_max*random.random()))

class Obstacles:
  def __init__(self, radius, position=None) -> None:
    self.radius = radius
    self.position: Point = position
    
    if self.position == None:
      self.position = Point.rand_position(x_max=x_map, y_max=y_map)

  def check_collision(self, uavs):
    for uav in uavs:
      for idx, point in enumerate(uav.path):
        if idx == 0:
          continue
        if self.position.get_distance_to_closest_on_line(uav.path[idx-1], point) <= self.radius:
          return True
    return False

class Task:
  def __init__(self, position):
    self.position=position
  
  def __str__(self):
    return str(self.position)

class UAV:
  def __init__(self, position=None, list_of_tasks=None, path=None, max_tasks=5):
    if list_of_tasks is None:
      list_of_tasks = []
    if path is None:
      path = []
    self.path:List[Point] = path
    self.list_of_tasks:List[Task] = list_of_tasks
    self.position:Point = position
    self.max_tasks = 2
  
  def number_of_assigned_tasks(self):
    return len(self.list_of_tasks)
  
  def distance_of_UAV(self):
    sum=0
    if self.number_of_assigned_tasks() > 0:
      sum=get_distance(self.position,self.list_of_tasks[0].position)
      
      for i in range(1,self.number_of_assigned_tasks()):
        sum=sum+get_distance(self.list_of_tasks[i-1].position,self.list_of_tasks[i].position)
    
    return sum
  
  def path_length(self) -> float:
    length = 0
    if self.number_of_assigned_tasks() > 0:
      length = get_distance(self.position, self.path[0])
      
      for i in range(len(self.path)-1):
        length = length + get_distance(self.path[i], self.path[i+1])
    
    return length


  def __str__(self):
    st=""
    for task in self.list_of_tasks:
      st += "("+str(task.position.x) + ","+str(task.position.y)+") "
    return f"Position {self.position} \nTasks : {st}"
    
  def __repr__(self):
     st=""
     for task in self.list_of_tasks:
      st += "("+str(task.position.x) + ","+str(task.position.y)+") "
     return f"Position {self.position} \n Tasks : {st}"
    
def get_distance(p1, p2):
  return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

class System:
  def __init__(self, list_of_UAVs,list_of_tasks):
    self.list_of_UAVs:List[UAV] = list_of_UAVs
    self.list_of_tasks:List[Task] = list_of_tasks
    self.Weight = 100
    self.uavs_history = []
 
    # assign_random_tasks(list_of_tasks,list_of_UAVs)
    
    self.best_Obj=self.cost() ##best objective function for the task assignment
    self.candidate=[] ## the current candidate
    self.candidate_Obj= None  ## the objective for the current candidate
  
    # generate path and random middle points in the path of the UAV between itself and its tasks
  def initRandomSoln(self):
    for uav in self.list_of_UAVs:      
      uav.path = []
      for task in uav.list_of_tasks: # initial path
        for _ in range(no_path_points):
          uav.path.append(Point.rand_position(x_map, y_map))
        uav.path.append(task.position)

  def get_fitness(uavs:List[UAV], sys:'System'):
    path_lengths = 0
    for uav in uavs:
      path_lengths = path_lengths + uav.path_length()
    return path_lengths + sys.Weight * sys.number_of_used_UAVs()


  def cost(self):
    sum = 0
    for x in self.list_of_UAVs:
      sum = sum + x.distance_of_UAV()
    sum += self.Weight * self.number_of_used_UAVs()
    return sum


  def candidate_cost(self):
    sum = 0
    for x in self.candidate:
      sum = sum + x.distance_of_UAV()
    sum += self.Weight * self.number_of_used_UAVs_candidate()
    return sum

  def number_of_used_UAVs(self):
    sum=0
    for x in self.list_of_UAVs:
      if len(x.list_of_tasks) > 0:
        sum +=1
    return sum

  def number_of_used_UAVs_candidate(self):
    sum=0
    for x in self.candidate:
      if len(x.list_of_tasks) > 0:
        sum +=1
    return sum
  
  # Modify
  # def assign_tasks_GA(self,Tf,T0,beta):
  #   simulated_annealing_TaskAssignment(self,Tf,T0,beta)
  
    
    
  #   assign random tasks for random UAVs
  def assign_random_tasks(self):
    for uav in self.list_of_UAVs:
      uav.list_of_tasks = []
    list_of_UAVs = copy.copy(self.list_of_UAVs)
    
    # Choose random task and assign to random UAV
    # Deep copy tasks list to remove chosen task from copied list 
    tasks = copy.deepcopy(self.list_of_tasks)
    
    
    while len(tasks) > 0:
      task_idx = random.randint(0,len(tasks)-1)
      task = tasks.pop(task_idx)
      uav_idx = random.randint(0,len(list_of_UAVs)-1)
      uav = list_of_UAVs[uav_idx] 
      if uav.number_of_assigned_tasks() < uav.max_tasks:
        list_of_UAVs[uav_idx].list_of_tasks.append(task)
      else:
        tasks.append(task)
        continue
    
    # check if the task assignment is not new, if new: proceed
    if list_of_UAVs in self.uavs_history:
      print("searching for a new task assignment")
      self.assign_random_tasks()
    else:
      print("UAVs assigned to tasks")
      self.uavs_history += [list_of_UAVs] 
      self.list_of_UAVs = copy.deepcopy(list_of_UAVs)
    
    
  def update_UAVs(self, newSol):
    self.list_of_UAVs=newSol
    self.best_Obj=self.cost()

  def update_best_Obj(self, Obj):
    self.best_Obj=Obj

  # def update_candidate(self):
  #   self.candidate=generate_new_sol(self.list_of_UAVs)
  #   self.candidate_Obj=self.candidate_cost()


  def plott(self):
    for task in self.list_of_tasks:
      plt.plot(task.position.x,task.position.y, marker="o", markersize=20, markerfacecolor="green")
    for uav in self.list_of_UAVs:
      Xs=[uav.position.x]
      Ys=[uav.position.y]
      for t in uav.list_of_tasks:
        Xs.append(t.position.x)
        Ys.append(t.position.y)
      plt.plot(uav.position.x,uav.position.y, marker="o", markersize=20, markerfacecolor="yellow")
      plt.plot(Xs,Ys)
    plt.show()    
    
def initiate_problem(Nuavs,Ntasks,Xmap,Ymap):
  Uavs=[]
  Tasks=[]
  for i in range(Nuavs):
    p=Point(random.randint(0,Xmap),random.randint(0,Ymap))
    Uavs.append(UAV(p,[]))
  for i in range(Ntasks):
    p=Point(random.randint(0,Xmap),random.randint(0,Ymap))
    Tasks.append(Task(p))
  return Uavs,Tasks
    

def objective(uavs:List[UAV], sys:System):
  path_lengths = 0
  for uav in uavs:
    path_lengths = path_lengths + uav.path_length()
  return path_lengths + sys.Weight * sys.number_of_used_UAVs()


def path_objective(uavs):
  path_lengths = []
  for uav in uavs:
    path_lengths.append(uav.path_length())
  return path_lengths


def assign_random_tasks(list_of_tasks:List[Task], list_of_UAVs:List[UAV]):
  tasks=copy.copy(list_of_tasks)
  while len(tasks)>0:
    task_idx = random.randint(0,len(tasks)-1)
    task = tasks.pop(task_idx)
    uav_idx=random.randint(0,len(list_of_UAVs)-1)
    if list_of_UAVs[uav_idx].max_tasks > len(list_of_UAVs[uav_idx].list_of_tasks):
      list_of_UAVs[uav_idx].list_of_tasks.append(task)
    else:
      tasks.append(task)
      
