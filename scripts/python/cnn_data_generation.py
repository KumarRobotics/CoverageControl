import sys
import math
import time
import numpy as np
import pyCoverageControl # Main library
from pyCoverageControl import Point2 # for defining points
from pyCoverageControl import PointVector # for defining list of points
from pyCoverageControl import CoverageSystem
from multiprocessing import Pool

params_ = pyCoverageControl.Parameters('params/parameters.yaml')

num_gaussians = 100
num_robots = 20
env = CoverageSystem(params_, num_gaussians, num_robots)

robot_id = 0

# We use the function StepDataGenerationLocal to drive the robots around in the world
# The CoverageSystem maintains an oracle_map_ which has the information of the world_idf_ at the locations visited by any of the robots. For locations that haven't been visited yet, the importance is set to a Parameters::pUnknownImportance
# The StepDataGenerationLocal uses an Oracle which decides where the robots should move based on the oracle_map_ described above. It performs a Voronoi Tessalation will ALL the robots, then computes the centroid, solves the assignment problem, and then steps the robots towards their respective assigned centroids.
# The difference from standard methods is that this Oracle does not assume that the entire map is known. It uses only the information that is make available by the robots.
# The StepDataGenerationLocal function has an input num_steps. This allows us to step several times before exchanging data in order to get sufficiently varying data.
# Thus we can simulate several times on the same environment to get different dataset.
# For the training of the CNN, each robot is a unique data
# For each environment, lets say we can simulate upto pEpisodeSteps=1000, as after that the robots might either converge or just osscilate and no new data is generated.
# So for an environment, we take num_steps=10 for pEpisodeSteps/num_steps times. So we will end up with a dataset of num_robots * pEpisodeSteps/num_steps unique data

def write_npz(iRobot):
    np.savez_compressed('../../data/cnn_data/data_' + f'{(iData * 200 + iter * 20 + iRobot):07d}' + '.npz', local_map = env.GetRobotLocalMap(iRobot), communication_map = env.GetCommunicationMap(iRobot), label=np.concatenate((env.GetVoronoiCell(iRobot).centroid, [env.GetVoronoiCell(iRobot).mass])))

for iData in range(0, 500):
    num_steps = 10
    print(iData)
    for iter in range(0, round(params_.pEpisodeSteps/num_steps)):
        # returns true if the robots have converged
        cont_flag = env.StepDataGenerationLocal(num_steps)
        if(not cont_flag):
            break
        positions = env.GetRobotPositions()
        pool = Pool()
        pool.map(write_npz, range(0, num_robots))
