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
from multiprocessing import Pool

params_ = pyCoverageControl.Parameters('params/parameters.yaml')

num_gaussians = 100
num_robots = 20

count = 0
robot_id = 0

actions = PointVector(num_robots)

def GetSquaredNorm(Point2 a, Point2 b):
    diff_x = a[0] - b[0]
    diff_y = a[1] - b[1]
    return diff_x * diff_x + diff_y * diff_y

# Need to modify this function
def write_npz(iRobot):
    np.savez_compressed('../../data/cnn_data_scaled/data_' + f'{(count * num_robots + iRobot):07d}' + '.npz', local_map = env.GetRobotLocalMap(iRobot), communication_map = env.GetCommunicationMap(iRobot), label=np.concatenate((env.GetVoronoiCell(iRobot).centroid, [env.GetVoronoiCell(iRobot).mass])))

while count < 50000:
    num_steps = 0
    print(str(count))
    env = CoverageSystem(params_, num_gaussians, num_robots)
    robot_positions = env.GetRobotPositions()
    voronoi_cells = env.LloydOffline()

    centroid_x = np.array([])
    centroid_y = np.array([])
    for vcell in voronoi_cells:
        centroid_x = np.append(centroid_x, vcell.centroid[0])
        centroid_y = np.append(centroid_y, vcell.centroid[1])
    centroid_pos = np.array([centroid_x, centroid_y]).transpose()
    cost_matrix = pwdist(robot_positions.transpose(), centroid_pos)

    row_ind, col_ind = linear_sum_assignment(cost_matrix)

    cont_flag = True
    while num_steps < params_.pEpisodeSteps:
        robot_global_positions_ = GetRobotPositions()
        for iRobot in range(0, num_robots):
            cont_flag = False
            curr_pos = robot_global_positions_[iRobot]
            goal = centroid_pos[col_ind[i]]
            dist_sqr = GetSquaredNorm(goal, curr_pos)
            if(dist_sqr > params_.pResolution * params_.pResolution):
                cont_flag = True
                dist = math.sqrt(dist_sqr)
                diff = goal - curr_pos
                direction = diff
                direction.normalize()
                speed = dist / params_.pTimeStep
                actions[iRobot] = direction * speed # These are the labels
        if(not cont_flag):
            break
        # positions = env.GetRobotPositions()
        # pool = Pool(num_robots)
        # pool.map(write_npz, range(0, num_robots))
        num_steps = num_steps + 1
    count = count + 1
