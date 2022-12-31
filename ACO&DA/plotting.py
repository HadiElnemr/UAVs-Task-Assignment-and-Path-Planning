from matplotlib import pyplot as plt
from classes import *
from typing import List

def plot_fitnesses(n_iter, best_fitnesses):
    fig1, ax1 = plt.subplots()
    ax1.set_xlabel('Iteration no.')
    ax1.set_ylabel('fitness value')
    ax1.plot([i for i in range(1,n_iter+1)], best_fitnesses)
    ax1.set_title('Fitness convergence curve')
    plt.show()

def plot_paths(uavs:List[UAV], tasks:List[Task], map_dim):
    for task in tasks:
        plt.plot(task.position.x, task.position.y, marker="o", markersize=14, markerfacecolor="green", markeredgecolor='green')
    
    for i,uav in enumerate(uavs):
        points_x = [uav.position.x]
        points_y = [uav.position.y]
        plt.plot(uav.position.x, uav.position.y, marker="o", markersize=12, markerfacecolor="red")
        
        for point in uav.path:
            points_x += [point.x]
            points_y += [point.y]
        plt.plot(points_x, points_y, '-o', markersize=5)

    plt.xlim([0, map_dim])
    plt.ylim([0, map_dim])
    plt.xlabel('x')
    plt.ylabel('y')
    plt.title('Task assignment & Path planning results in a 2D map')
    plt.grid()
    plt.show()