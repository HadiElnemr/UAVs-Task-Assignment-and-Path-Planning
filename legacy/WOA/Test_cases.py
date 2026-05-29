import copy
from classes import *
# from Task_Assignment import isFeasible


#Test Case 1 : a simple one with 5 UAVs and 5 tasks in 10*10 map
t1 = Task(Point(2, 5))
t2 = Task(Point(1, 7))
t3 = Task(Point(6, 2))
t4 = Task(Point(5, 9))
t5 = Task(Point(1, 1))
tasks = [t1, t2, t3, t4 ,t5]
u1 = UAV(Point(0, 2), [])
u2 = UAV(Point(0, 1), [])
u3 = UAV(Point(1, 8), [])
u4 = UAV(Point(1, 6), [])
u5 = UAV(Point(2, 1), [])
uavs = [u1, u2, u3,u4,u5]
sys1 = System(copy.deepcopy(uavs), copy.deepcopy(tasks))

#Test case 2 : a little complex case with 10 UAVs(all of them on the y-axis) and 15 random tasks in 100*100 map
Uavs = []
Tasks = []
for i in range(10):
    p = Point(0, random.randint(0, 100))
    Uavs.append(UAV(p, []))
for i in range(15):
    p = Point(random.randint(0, 100), random.randint(0, 100))
    Tasks.append(Task(p))

sys2= System(copy.deepcopy(Uavs),copy.deepcopy(Tasks))

#Test case 3 : a more complex case with 10 UAVs and 15 random tasks in 100*100 map totally random position
Uavs = []
Tasks = []
for i in range(10):
    p = Point(random.randint(0,100), random.randint(0, 100))
    Uavs.append(UAV(p, []))
for i in range(15):
    p = Point(random.randint(0, 100), random.randint(0, 100))
    Tasks.append(Task(p))

sys3= System(copy.deepcopy(Uavs),copy.deepcopy(Tasks))


# Test case 4 : a more realistic test case with 50 UAVs with 70 tasks in 1000*1000 map
# the 50 UAVs will be regularly distrubted on the x-axis, but the 70 tasks will be radnomly distrubted in the map

Uavs = []
Tasks = []
for i in range(50):
    p = Point(i*20 ,0 )
    Uavs.append(UAV(p, []))
for i in range(70):
    p = Point(random.randint(0, 1000), random.randint(0, 1000))
    Tasks.append(Task(p))

sys4= System(copy.deepcopy(Uavs),copy.deepcopy(Tasks))


