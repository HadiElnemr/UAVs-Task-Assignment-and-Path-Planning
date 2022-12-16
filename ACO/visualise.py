from matplotlib import pyplot as plt
from parameters import *


def plot_paths(uavs, x_map, y_map, best_uavs = None):
  fig1.canvas.flush_events()
  ax1.cla()
  # for uav in uavs:
  #   points_x = [uav.position.x]
  #   points_y = [uav.position.y]
  #   # plt.plot(uav.position.x, uav.position.y, '-')
  #   for point in uav.path:
  #     points_x += [point.x]
  #     points_y += [point.y]
  #   ax1.set_xlim(0, x_map)
  #   ax1.set_ylim(0, y_map)
  #   ax1.scatter(points_x, points_y)
  #   ax1.plot(points_x, points_y, '-', alpha=0.2)
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
    ax1.plot(points_x, points_y, f'{colour_str[idx%7]}-')
    ax1.set_xlabel('x')
    ax1.set_ylabel('y')
  fig1.canvas.draw()
  
def plot_cost(cost, temp):
  fig2.canvas.flush_events()
  ax2.cla()
  ax2.invert_xaxis()
  ax2.scatter(temp, cost)
  ax2.plot(temp, cost, '--')
  fig2.canvas.draw()
  ax2.set_xlabel('Temperature')
  ax2.set_ylabel('Cost')
  ax2.set_ylim(0, 1000)

plt.ion()

fig1, ax1 = plt.subplots()
ax1.set_xlim(0, x_map)
ax1.set_ylim(0, y_map)

fig2, ax2 = plt.subplots()
ax2.set_xlim(0, 1000)
ax2.set_ylim(0, 100)

# mngr = plt.get_current_fig_manager()
# mngr.window.setGeometry(50,100,640, 545)
# mngr = plt.get_current_fig_manager()


for task in tasks:
  ax1.plot(task.position.x,task.position.y, marker="o", markersize=20, markerfacecolor="green")