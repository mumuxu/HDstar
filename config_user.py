##########################################################Config user.py####################################################
# Configuration File
from __future__ import division
import math
import collections
import numpy as np

testingMode = False             # suppresses figure generation, outputs from main*.py are not printed  (changed from False)

startWithEmptyMap = True
makeRandObs = False
useMovingGoals = False
restrictVerticalMovement = True
useHierarchicalPlanning = True
numHierLevels = 0

percentFixedRandomObstacles = 0
safetymargin = 1
cX, cY, cZ = 1, 1, 2        # cX and cY currently are unused - modify computeCost if desired
heuristicScale = 1.01

searchRadius = 8
refinementDistance = math.ceil(searchRadius * 1)    # must be an integer
t_max = float('inf')             # Max time to spend on path-finding, in milliseconds. Enter inf to prevent restriction

#was 64
sizeX = 27
sizeY = 10
sizeZ = 4

mapscale = 1 #was 2
start =(1, 1, 1) #was (3*mapscale , 3*mapscale, 6*mapscale)
#want goal at 18,6,1
goals = np.array([[18.,6.,1.,0.]]) #np.array([[fcn.cantor(18,6,1)]]) #([[62., 62., 6.,  0.]])  * mapscale  

# Configure Moving Goals
initX = [60, 60]    #[12, 6]
initY = [50, 49]    #[3, 2]
initZ = [6, 6]      #[4, 7]
T     = [5, 5]      #[5, 2]

# Fixed Individual Obstacles
obstacles = []
"""
# Fixed Rectangular Obstacles
rXstart = [8,  12, 15,  35, 41, 49]
rYstart = [2,  15, 35, 10, 20, 47]
rZstart = [1,  1,  1,  1,  1,  1]
rXdim   = [4,  20, 30, 5,  8,  6]
rYdim   = [9,  12, 8,  5,  8,  6]
rZdim   = [30,  8, 15, 28, 20, 28] """
rXstart = [2, 11, 20] #, 2, 11, 20] #[8, 14, 5]
rYstart = [2, 2, 2, ]#9, 9, 9]   #[2, 5, 2]
rZstart = [2, 2, 2,] # 1, 1, 1]  #[1, 1, 11]
rXdim   = [5, 5, 5,] # 5, 5, 5] #[4, 11, 1]
rYdim   = [8, 8, 8,] # 1, 1, 1] #[9, 15, 6]
rZdim   = [1, 1, 1,] #1, 1, 1] #[1, 11, 2]

# Generate Random Dynamic Obstacles
randomint = np.random.random_integers
minObs = 5
maxObs = 50
maxPercent = 5
seedDyn = np.random.randint(0,1000)
#seedDyn = np.random.randint(0,10)
#seedDyn = 432

# Generate Random Fixed Obstacles
num2gen = int(round(percentFixedRandomObstacles/100 * sizeX*sizeY*sizeZ))
seedStatic = np.random.random_integers(0,1000)
#seedStatic = np.random.random_integers(0,10
#seedStatic = 141

"""
====================================================================================
================== Variables below this line are not user inputs ===================
========== They are here for configuration or to create global variables ===========
====================================================================================
"""

# Modifying by scale factor
initX = [mapscale*point for point in initX]
initY = [mapscale*point for point in initY]
initZ = [mapscale*point for point in initZ]

rXstart = [mapscale*(point) for point in rXstart if point >= 1]
rYstart = [mapscale*(point) for point in rYstart if point >= 1]
rZstart = [point for point in rZstart if point >= 1]                      #why don't we use mapscale for this one?
rXdim = [mapscale*(point) for point in rXdim if point <= sizeX]
rYdim = [mapscale*(point) for point in rYdim if point <= sizeY]
rZdim = [mapscale*(point) for point in rZdim if point <= sizeZ]   

sizeX *= mapscale
sizeY *= mapscale
sizeZ *= mapscale

if testingMode:
    makeFigure = False        
    makeMove = False          

if not useMovingGoals:
    initX = []
    initY = []
    initZ = []
    T = []

goalsVisited, goalhandles, numGoals, goal = [], [], [], []
stepCount = 1              # number of total iterations
number_of_obstacles = 0    # for genRandObs function
numNodes = sizeX*sizeY*sizeZ
goalMoved = False
numlevels = 0

# Set up initial heading angles to factor in direction of travel
oldstart = None

# Set up UAV map and plot
map_ = collections.defaultdict(lambda : 0)
costMatrix = collections.defaultdict(lambda: 1)

# Used to save some variables
hdl = []
closed_list = 0
output = {}

# Additional variables
zf1, zf2 = 1, 0             # provides more flexibility over coarse z-movement; zf1 = multiplier, zf2 = added constant
                                # use (1,0) for default, or (0,x) to set coarse z-successors at a distance of x
distancerequirement =  7     # used in findPath function. determines cluster size used for coarse paths                       
                                # shorter = faster, but may have longer paths
                                # too small and it may not find a path, so >=6 recommended
minclustersize = 4          # represents dimension of smallest cluster in terms of L0 nodes
alpha = 0.5             # use 0.5 for centripetal splines
splinePoints = 5        # Enter 2 to not use splines, otherwise 5 is recommended

