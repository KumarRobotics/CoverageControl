import sys
import math
import time
import numpy as np
from sklearn.metrics import pairwise_distances as pwdist
from scipy.optimize import linear_sum_assignment
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import CoverageSystem
from pyCoverageControl import LloydLocalVoronoi, OracleExploreExploit
from multiprocessing import Pool

# We can visualize the map in python
import matplotlib.pylab as plt
import matplotlib.animation as animation
import seaborn as sns
colormap = sns.color_palette("light:b", as_cmap=True)


params_ = pyCoverageControl.Parameters('params/parameters.yaml')

num_gaussians = 20
num_robots = 20

count = 0
robot_id = 0

num_steps = 0
env = CoverageSystem(params_, num_gaussians, num_robots)

# oracle = LloydLocalVoronoi(params_, num_robots, env)
oracle = OracleExploreExploit(params_, num_robots, env)
voronoi_cells = oracle.GetVoronoiCells()
robot_positions = env.GetRobotPositions()
# map = env.GetWorldIDF()
map = oracle.GetOracleMap()

fig = plt.figure("Environment")
ax = sns.heatmap(map.transpose(), vmax=params_.pNorm, cmap=colormap, square=True)
cbar_ax = fig.axes[-1]# retrieve previous cbar_ax (if exists)
ax.invert_yaxis()
nrow, ncol = map.shape
# septicks = 5 ** (math.floor(math.log(nrow, 5)) - 1)
septicks = 10 ** (math.floor(math.log(nrow, 10)) - 1)
plt.xticks(np.arange(0, nrow, septicks), np.arange(0, nrow, septicks))
plt.yticks(np.arange(0, ncol, septicks), np.arange(0, ncol, septicks))

plot_pos_x = np.array([])
plot_pos_y = np.array([])
for pos in robot_positions:
    plot_pos_x = np.append(plot_pos_x, pos[0] / params_.pResolution)
    plot_pos_y = np.append(plot_pos_y, pos[1] / params_.pResolution)
plot_robots, = ax.plot(plot_pos_x, plot_pos_y, 'go')

goals = oracle.GetGoals()
plot_goal_x = np.array([])
plot_goal_y = np.array([])
for pos in robot_positions:
    plot_goal_x = np.append(plot_goal_x, pos[0] / params_.pResolution)
    plot_goal_y = np.append(plot_goal_y, pos[1] / params_.pResolution)
plot_goals, = ax.plot(plot_goal_x, plot_goal_y, 'rx')


curr_pos = np.array([plot_pos_x, plot_pos_y]).transpose()
goals_pos = np.array([plot_goal_x, plot_goal_y]).transpose()

# fig_local = plt.figure("Local Map of Robot" + str(robot_id))
# local_map = env.GetRobotLocalMap(robot_id)
# local_ax = sns.heatmap(data=np.flip(local_map.transpose(),0), vmax=params_.pNorm, cmap=colormap, square=True)
# cbar_ax = fig_local.axes[-1]# retrieve previous cbar_ax (if exists)

cont_flag = True
prev_robot_pos = robot_positions

ax.plot([curr_pos[0][0], goals_pos[0][0]], [curr_pos[0][1], goals_pos[0][1]], 'r')
for i in range(1, num_robots):
    ax.plot([curr_pos[i][0], goals_pos[i][0]], [curr_pos[i][1], goals_pos[i][1]])

def animate(i):
    print(str(i))
    cont_flag = oracle.Step()
    actions = oracle.GetActions()
    robot_positions = env.GetRobotPositions()
    sns.heatmap(oracle.GetOracleMap().transpose(), vmax=params_.pNorm, cmap=colormap, square=True, ax=ax, cbar_ax = cbar_ax)

    for i in range(0, num_robots):
        plot_pos_x[i] =  robot_positions[i][0] / params_.pResolution
        plot_pos_y[i] =  robot_positions[i][1] / params_.pResolution

    goals = oracle.GetGoals()
    for i in range(0, num_robots):
        plot_goal_x[i] =  goals[i][0] / params_.pResolution
        plot_goal_y[i] =  goals[i][1] / params_.pResolution

    plot_robots.set_xdata(plot_pos_x)
    plot_robots.set_ydata(plot_pos_y)
    plot_goals.set_xdata(plot_goal_x)
    plot_goals.set_ydata(plot_goal_y)
    return [plot_robots, plot_goals]

def animate_local(i):
    local_map = env.GetRobotLocalMap(robot_id)
    local_ax.set_title("Robot [" + str(robot_id) + "] position: " + "{:.{}f}".format(robot_positions[robot_id][0], 2) + ", " +  "{:.{}f}".format(robot_positions[robot_id][1], 2))
    sns.heatmap(ax=local_ax,data=np.flip(local_map.transpose(), 0), vmax=params_.pNorm, cmap=colormap, square=True, cbar_ax=cbar_ax, xticklabels=[],yticklabels=[])

# ani = animation.FuncAnimation(fig, animate, interval=0, blit=True)
# ani2 = animation.FuncAnimation(fig_local, animate_local, interval=100, blit=False)


for i in range(0, 20000):
    print(i)
    cont_flag = oracle.Step()
    if cont_flag == False:
        break

animate(0)
plt.show()
