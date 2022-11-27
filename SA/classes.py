import math
import random
import copy
from typing import List
from parameters import *

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
      self.position = rand_position(x_max=x_map, y_max=y_map)

  def check_collision(self, uavs:List[uavs]):
    for uav in uavs:
      for idx, point in enumerate(uav.path):
        if idx == 0:
          continue
        if self.position.get_distance_to_closest_on_line(uav.path[idx-1], point) <= self.radius:
          return True
    return True

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
    self.path = path
    self.list_of_tasks = list_of_tasks
    self.position = position
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
  
  def path_length(self):
    length = 0
    if self.number_of_assigned_tasks() > 0:
      length = get_distance(self.position, self.path[0])
      
      for i in range(len(self.path)-1):
        length = length + get_distance(self.path[i], self.path[i+1])
    
    return length

def get_distance(p1, p2):
  return math.sqrt((p1.x - p2.x)**2 + (p1.y - p2.y)**2)

class System:
  def __init__(self, list_of_UAVs ,list_of_tasks):
    self.list_of_UAVs:List[UAV] = list_of_UAVs
    self.list_of_tasks:List[Task] = list_of_tasks
    self.Weight = 100
    self.uavs_history = []

  def cost(self):
    sum=0
    for x in self.list_of_UAVs:
      sum=sum+x.distance_of_UAV()
    sum += self.Weight*self.number_of_used_UAVs()
    return sum

  def number_of_used_UAVs(self):
    sum=0
    for x in self.list_of_UAVs:
      if len(x.list_of_tasks) > 0:
        sum +=1
    return sum

  def assign_tasks(self):
    pass
    # David's part

  # assign random tasks for random UAVs
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
      self.assign_tasks()
    else:
      print("UAVs assigned to tasks")
      self.uavs_history += [list_of_UAVs] 
      self.list_of_UAVs = copy.deepcopy(list_of_UAVs)
    

def temperature_geo_update(T0, idx, alpha):
    return T0 * (alpha ** idx)

def temperature_linear_update(T0, idx, beta):
  return T0 - beta * idx

def objective(uavs, sys):
  path_lengths = 0
  for uav in uavs:
    path_lengths = path_lengths + uav.path_length()
  return path_lengths + sys.Weight * sys.number_of_used_UAVs()

def path_objective(uavs):
  path_lengths = []
  for uav in uavs:
    path_lengths.append(uav.path_length())
  return path_lengths
