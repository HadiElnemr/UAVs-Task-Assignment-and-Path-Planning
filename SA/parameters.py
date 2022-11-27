x_map = 100
y_map = 100
T0 = 600
Tf = 0.1
i_max = 500

# paramaters
shift_x, shift_y = 5, 5
alpha = 0.7
beta = (T0-Tf)/i_max
no_path_points = 2

costs = []
temperatures = []

n_tasks = 3
n_uavs = 3
# lists
tasks = []
uavs = []
