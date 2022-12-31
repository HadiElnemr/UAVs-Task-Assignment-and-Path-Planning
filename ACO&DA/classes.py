import math
import random
import copy
from typing import List
# from parameters import costs,i_max,n_tasks,n_uavs,no_path_points,shift_x,shift_y,tasks,uavs,x_map,y_map
from da_parameters import *
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

    def get_distance(self, p):
        return math.sqrt((self.x - p.x)**2 + (self.y - p.y)**2)

    def dot(self, p: 'Point'):
        return self.x * p.x + self.y * p.y

    def element_wise_mul(self, p: 'Point'):
        return Point(self.x*p.x, self.y*p.y)

    def add(self, p: 'Point'):
        return Point(self.x + p.x, self.y + p.y)

    def sub(self, p: 'Point'):
        return Point(self.x - p.x, self.y - p.y)

    def mul(self, f: float):
        return Point(self.x * f, self.y * f)

    def abs(self):
        return Point(abs(self.x), abs(self.y))

    # Line ab is considered a line segment, so find the point on that segment
    def get_distance_to_closest_on_line(self, a: 'Point', b: 'Point'):
        ba = b.sub(a)   # b with respect to a
        ds = self.sub(a).dot(ba) / ba.dot(ba)  # (p - a). ba  / ||ba||^2
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

    # def __init__(self, position):
    #   self.position:Point=position

    def __init__(self, position, number=None):
        self.position: Point = position
        if number == None:
            self.number = 0
        self.number = number

    def __str__(self):
        return str(self.position)


class UAV:
    def __init__(self, position=None, list_of_tasks=None, number=None, path=None, max_tasks=5):
        if list_of_tasks is None:
            list_of_tasks = []
        if path is None:
            path = []
        self.path: List[Point] = path
        self.DA_velocities:List[Point] = []
        
        if number == None:
            self.number = 0
        self.number = number
        self.list_of_tasks: List[Task] = list_of_tasks
        self.position: Point = position
        self.max_tasks = 3
    
    def number_of_assigned_tasks(self):
        return len(self.list_of_tasks)

    def distance_of_UAV(self):
        sum = 0
        if self.number_of_assigned_tasks() > 0:
            sum = self.position.get_distance(self.list_of_tasks[0].position)

            for i in range(1, self.number_of_assigned_tasks()):
                sum = sum + \
                    self.list_of_tasks[i-1].position.get_distance(
                        self.list_of_tasks[i].position)

        return sum

    def path_length(self) -> float:
        length = 0
        if self.number_of_assigned_tasks() > 0:
            length = self.position.get_distance(self.path[0])
            for i in range(len(self.path)-1):
                length = length + self.path[i].get_distance(self.path[i+1])

        return length

    def __str__(self):
        st = ""
        for task in self.list_of_tasks:
            st += "("+str(task.position.x) + ","+str(task.position.y)+") "
        return f"Position {self.position} \nTasks : {st}"

    def __repr__(self):
        st = ""
        for task in self.list_of_tasks:
            st += "("+str(task.position.x) + ","+str(task.position.y)+") "
        return f"Position {self.position} \n Tasks : {st}"


class System:
    def __init__(self, list_of_UAVs, list_of_tasks):
        self.list_of_UAVs: List[UAV] = list_of_UAVs
        self.list_of_tasks: List[Task] = list_of_tasks
        self.Weight = 100
        self.uavs_history = []

        # assign_random_tasks(list_of_tasks,list_of_UAVs)

        # self.best_Obj=self.cost() ##best objective function for the task assignment
        self.candidate = []  # the current candidate
        self.candidate_Obj = None  # the objective for the current candidate

    # generate path and random middle points in the path of the UAV between itself and its tasks
    def initRandomSoln(self) -> List[UAV]:
        ret_uavs:List[UAV] = []
        for uav in copy.deepcopy(self.list_of_UAVs):
            uav.path = []
            uav.DA_velocities = []
            for task in uav.list_of_tasks:  # initial path
                for _ in range(no_path_points):
                    rand_point = Point.rand_position(x_map, y_map)
                    uav.path.append(rand_point)
                    uav.DA_velocities.append(copy.deepcopy(rand_point))
                uav.path.append(task.position)
                uav.DA_velocities.append(Point(0,0))

            ret_uavs.append(uav)
        return ret_uavs

    def get_fitness(uavs: List[UAV], weight, sys: 'System' = None):
        path_lengths = 0
        if sys is not None:
            weight = sys.Weight
        for uav in uavs:
            path_lengths = path_lengths + uav.path_length()
        return path_lengths + weight * System.number_of_used_UAVs(uavs)

    def cost(self):
        sum = 0
        for x in self.list_of_UAVs:
            sum = sum + x.distance_of_UAV()
        sum += self.Weight * self.number_of_used_UAVs()
        return sum

    def number_of_used_UAVs(self):
        sum = 0
        for x in self.list_of_UAVs:
            if len(x.list_of_tasks) > 0:
                sum += 1
        return sum

    def number_of_used_UAVs(uavs: List[UAV]):
        sum = 0
        for uav in uavs:
            if len(uav.list_of_tasks) > 0:
                sum += 1
        return sum

    #   assign random tasks for random UAVs
    def assign_random_tasks(self):
        for uav in self.list_of_UAVs:
            uav.list_of_tasks = []
        list_of_UAVs = copy.copy(self.list_of_UAVs)

        # Choose random task and assign to random UAV
        # Deep copy tasks list to remove chosen task from copied list
        tasks = copy.deepcopy(self.list_of_tasks)

        while len(tasks) > 0:
            task_idx = random.randint(0, len(tasks)-1)
            task = tasks.pop(task_idx)
            uav_idx = random.randint(0, len(list_of_UAVs)-1)
            uav = list_of_UAVs[uav_idx]
            if uav.number_of_assigned_tasks() < uav.max_tasks:
                list_of_UAVs[uav_idx].list_of_tasks.append(task)
            else:
                tasks.append(task)
                continue

    def plott(self):
        for task in self.list_of_tasks:
            plt.plot(task.position.x, task.position.y, marker="o",
                     markersize=20, markerfacecolor="green")
        for uav in self.list_of_UAVs:
            Xs = [uav.position.x]
            Ys = [uav.position.y]
            for t in uav.list_of_tasks:
                Xs.append(t.position.x)
                Ys.append(t.position.y)
            plt.plot(uav.position.x, uav.position.y, marker="o",
                     markersize=20, markerfacecolor="yellow")
            plt.plot(Xs, Ys)
        plt.show()


def initiate_problem(Nuavs, Ntasks, Xmap, Ymap):
    Uavs = []
    Tasks = []
    for i in range(Nuavs):
        p = Point(random.randint(0, Xmap), random.randint(0, Ymap))
        Uavs.append(UAV(p, []))
    for i in range(Ntasks):
        p = Point(random.randint(0, Xmap), random.randint(0, Ymap))
        Tasks.append(Task(p))
    return Uavs, Tasks


def objective(uavs: List[UAV], sys: System):
    path_lengths = 0
    for uav in uavs:
        path_lengths = path_lengths + uav.path_length()
    return path_lengths + sys.Weight * sys.number_of_used_UAVs()
