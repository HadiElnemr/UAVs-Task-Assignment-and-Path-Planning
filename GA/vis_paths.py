from matplotlib import pyplot as plt
from parameters import *
from typing import List
from classes import UAV

def plot_paths(uavs:List[UAV], x_map, y_map, best_uavs, fig1, ax1):
  for task in tasks:
    ax1.plot(task.position.x,task.position.y, marker="o", markersize=20, markerfacecolor="green")
  fig1.canvas.flush_events()
  ax1.cla()
  for uav in uavs:
    points_x = [uav.position.x]
    points_y = [uav.position.y]
    # plt.plot(uav.position.x, uav.position.y, '-')
    for point in uav.path:
      points_x += [point.x]
      points_y += [point.y]
    ax1.set_xlim(0, x_map)
    ax1.set_ylim(0, y_map)
    ax1.scatter(points_x, points_y)
    ax1.plot(points_x, points_y, '-', alpha=0.2)
  colour_str = 'cmbgryk'
  for idx, uav in enumerate(best_uavs):
    points_x = [uav.position.x]
    points_y = [uav.position.y]
    for point in uav.path:
      points_x += [point.x]
      points_y += [point.y]
    ax1.set_xlim(0, x_map)
    ax1.set_ylim(0, y_map)
    ax1.scatter(points_x, points_y)
    ax1.plot(points_x, points_y, f'{colour_str[idx]}-')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
  fig1.canvas.draw()
  
plt.ion()



