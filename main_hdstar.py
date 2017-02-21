from __future__ import division
import sys
sys.path.append(r"c:\python27\lib")
sys.path.append(r"c:\python27\lib\compiler")
sys.path.append(r"C:\python27\lib\idelib")
sys.path.append(r"C:\python27\lib\importlib")
sys.path.append(r"c:\Users\conna\OneDrive\Documents\Visual Studio 2015\Projects\HDStar\HDStar")

import time
import os
import subprocess
print 'here'
import multiprocessing
#import resource
from math import isinf, sqrt, pi
import numpy as np
import config_user as gl
import config_program
import all_functions as fcn

"""
import sys
import clr
import time
import MissionPlanner #import *
from MissionPlanner import Utilities
"""
print 'Start'

# To reload settings for multiple trials
if gl.testingMode:
    reload(gl)
    reload(config_program)
    reload(fcn)

# Creating local copies of constants
sizeX, sizeY, sizeZ, cX, cY, cZ = gl.sizeX, gl.sizeY, gl.sizeZ, gl.cX, gl.cY, gl.cZ

searchRadius, useMovingGoals = gl.searchRadius, gl.useMovingGoals
makeRandObs, numlevels = gl.makeRandObs, gl.numlevels
minObs, maxObs, maxPercent, seedDyn, seedStatic = gl.minObs, gl.maxObs, gl.maxPercent, gl.seedDyn, gl.seedStatic
initX, initY, initZ, T, rXdim, rYdim, rZdim = gl.initX, gl.initY, gl.initZ, gl.T, gl.rXdim, gl.rYdim, gl.rZdim
rXstart, rYstart, rZstart = gl.rXstart, gl.rYstart, gl.rZstart
refinementDistance = gl.refinementDistance

""" Setup abstract levels and variables for performance testing """
L = fcn.setupLevels()

time_findPath = []             ####################################
total_cost = 0                ##########################################
final_pathX = [gl.start[0]]    ########################################
final_pathY = [gl.start[1]]   #############################3333
final_pathZ = [gl.start[2]]   ############################
tic1 = time.time()            ####################
#time_searh_and_update=[];
""" Begin main algorithm """
for idx in xrange(0, gl.numGoals):                      # for each goal
    xNew, yNew, zNew = gl.start    
                     # get current location
    t=time.clock()
    fcn.searchAndUpdate(xNew,yNew,zNew)                 # search for obstacles
    time_search=time.clock()-t;
   # time_search_and_update.append(time_search)
 ############################################################################################################################################################
    indx=0;
   # Generate GPS coordinates and execute in mission planner or whatever it is                                                                                                                                          
    goalst=gl.goals    
    Xstart,Ystart,Zstart = gl.start                                                                                                                                                     
    xg, yg, zg = goalst[indx, 0], goalst[indx, 1], goalst[indx,2] 
    goalgps=[38.992380,-76.939784]
    xstep=abs((38.992636 - goalgps[0])/(xg-Xstart))
    ystep=abs((-76.939109 - goalgps[1])/(yg-Ystart))
    gps= fcn.n2g(xNew,yNew,xstep,ystep)
    print gps 
  #  clr.AddReference("MissionPlanner.Utilities") # includes the Utilities class
   # time.sleep(10)                                             # wait 10 seconds before starting
    #Script.ChangeMode("Guided")                     # changes mode to "Guided"
   # print 'Guided Mode' 
  #  item = MissionPlanner.Utilities.Locationwp() # creating waypoint
  #  lat = gps(1)                                        # Latitude value
  #  lng = gps(2)                                         # Longitude value
   #alt = 45.720000                                           # altitude value
  #  MissionPlanner.Utilities.Locationwp.lat.SetValue(item,lat)     # sets latitude
  #  MissionPlanner.Utilities.Locationwp.lng.SetValue(item,lng)   # sets longitude
   #MissionPlanner.Utilities.Locationwp.alt.SetValue(item,alt)     # sets altitude
  #  print 'WP set' 
  #  MAV.setGuidedModeWP(item)                                    # tells UAV "go to" the set lat/long @ alt (no alt. in this case for Rover)
  #  print 'Going to WP'
  #  time.sleep(10)                                                            # wait 10 seconds
  #  print 'Ready for next WP'              
############################################################################################################################################################
    while gl.start != gl.goal:
        
        """ Compute path, smooth it, make a spline, and divide into a series of adjacent points to follow """
        tic = time.clock()    # start timer
        path = fcn.findPath(L)
       #path = fcn.postSmoothPath(path)
       #path = fcn.CatmullRomSpline(path)
        path = fcn.simulateUAVmovement(path)
     
        findPathTime = time.clock() - tic   # end timer
        time_findPath.append(findPathTime)  # record time


        if gl.stepCount == 1:
            initialFindPathTime = findPathTime  # record time to find
        xOrig, yOrig, zOrig = gl.start      # to identify when leaving refinement region
        dfs = 0                             # distance from start, used to identify when path needs to be updated
        goalMoved = False                   # indicates whether or not the goal has moved
        validPath = True                    # indicates whether or not path being followed is still valid
        while not goalMoved and validPath and gl.start != gl.goal and path:

            # Follow those points until path is invalidated or we reach end of refinement region
            for point in path:
                
              # Save current position, then move to next point
                xOld, yOld, zOld = xNew, yNew, zNew
                xNew, yNew, zNew = path.pop()
                gl.oldstart = gl.start
                gl.start = (round(xNew), round(yNew), round(zNew))   # update start coordinate
 ############################################################################################################################################################
                # Generate GPS coordinates and execute in mission planner or whatever it is                                                                                                                                          
                goalst=gl.goals     
                Xstart,Ystart,Zstart = gl.start                                                                                                                                                     
                xg, yg, zg = goalst[indx, 0], goalst[indx, 1], goalst[indx,2] 
                xstep=abs((38.992636 - 38.992380)/(xg-1))             
                ystep=abs((-76.939109 - -76.939784)/(yg-1))
                gps= fcn.n2g(xNew,yNew,xstep,ystep)
                print gps
   #             clr.AddReference("MissionPlanner.Utilities") # includes the Utilities class
    #           #time.sleep(10)                                             # wait 10 seconds before starting
     #           Script.ChangeMode("Guided")                     # changes mode to "Guided"
      #          print 'Guided Mode'
       #         item = MissionPlanner.Utilities.Locationwp() # creating waypoint
        #        lat = gps(1)                                        # Latitude value
         #       lng = gps(2)                                         # Longitude value
                #alt = 45.720000                                           # altitude value
          #      MissionPlanner.Utilities.Locationwp.lat.SetValue(item,lat)     # sets latitude
           #     MissionPlanner.Utilities.Locationwp.lng.SetValue(item,lng)   # sets longitude
               #MissionPlanner.Utilities.Locationwp.alt.SetValue(item,alt)     # sets altitude
            #    print 'WP set'
             #   MAV.setGuidedModeWP(item)                                    # tells UAV "go to" the set lat/long @ alt (no alt. in this case for Rover)
             #   print 'Going to WP' 
             #   time.sleep(10)                                                            # wait 10 seconds
            #    print 'Ready for next WP'             
############################################################################################################################################################
                # Update distance from start
                dx, dy, dz = xOrig-xNew, yOrig-yNew, zOrig-zNew
                dfs = sqrt(dx**2 + dy**2 + dz**2)

                # Update total cost of path
                total_cost += L[0].computeCost((xOld, yOld, zOld), (xNew, yNew, zNew), False)

                final_pathX.append(xNew)
                final_pathY.append(yNew)
                final_pathZ.append(zNew)
                 
                
                # Generate random obstacles
                if makeRandObs:
                    fcn.genRandObs(minObs,maxObs,maxPercent,seedDyn)
               
                # Moving goal execution
                if useMovingGoals:
                    for i in xrange(0,len(initX)):  goalMoved = fcn.movingGoal(initX[i], initY[i], initZ[i], T[i])
                
                # Update counter used for the two preceding functions
                gl.stepCount += 1

                # Check if there's any obstacles within search radius if we've moved to a different node
                if gl.oldstart != gl.start and not fcn.searchAndUpdate(xNew, yNew, zNew, path):                                                                              #  %%%%%
                    validPath=False
                    break

                # Check if next movement takes us outside of refinement region
                if dfs+1 >= refinementDistance/2 or len(path)<1:
                    validPath = False
                    break

    indx=indx+1                                                                                                                                                     #INDX IS UPDATED HERE

    if len(gl.goals) > 1:
        print  'finding next goal...'

        # Identify rows in goals array matching current goal
        k1 = np.where(gl.goals[:,0]==gl.goal[0])
        k2 = np.where(gl.goals[:,1]==gl.goal[1])
        k3 = np.where(gl.goals[:,2]==gl.goal[2])

        # Whichever row shows up in k1, k2, and k3 is the row with the current goal
        k = np.intersect1d(np.intersect1d(k1,k2),k3)

        gl.goalsVisited.append(gl.goals[k,3])           # save its goal ID
        gl.goals = np.delete(gl.goals, k, 0)            # delete that row

        # Find next closest goal with respect to straight line distance
        hyp = {}
        for idx, row in enumerate(gl.goals):
            gx, gy, gz = row[0], row[1], row[2]
            xdist, ydist, zdist = gx-gl.start[0], gy-gl.start[1], gz-gl.start[2]
            hyp[idx] = sqrt(xdist**2 + ydist**2 + zdist**2)

        idx, __ = min(hyp.items(), key=lambda x:x[1])
        gl.goal = (gl.goals[idx,0], gl.goals[idx,1], gl.goals[idx,2])

# Get averages, in milliseconds
mean_time_findPath = 1000*sum(time_findPath) / len(time_findPath)
#mean_search_and_update_time=1000*sum(time_search_and_update)/len(time_search_and_update)


def hdstar_outputs():
    return total_cost, gl.closed_list, mean_time_findPath, initialFindPathTime*1000

if not gl.testingMode:
    print 'Run succeeded!\n'
    print 'Elapsed time: ' + str(time.time() - tic1) + ' seconds'
    print'Total number of pathfinding iterations: ',len(time_findPath);
    print 'Total cost: ' + str(total_cost)
    print 'Mean Path-finding Time: ' + str(mean_time_findPath) + ' ms'
    print 'Expanded nodes: ' + str(gl.closed_list)

print 'Final PathX: \n', final_pathX
print 'Final PathY: \n', final_pathY
print 'Final PathZ: \n', final_pathZ

#this is to test to make sure the output works
out=open('testfile','w')
out.write('the run succeeded! \n')
xp=(str (final_pathX))
yp=(str(final_pathY))
zp=(str(final_pathZ))
mpt=(str(mean_time_findPath))
out.write('The x path is: \n')
out.write(xp)
out.write('\nThe y path is: \n')
out.write(yp)
out.write('\nThe z path is: \n')
out.write(zp)
out.write('\nThe mean path finding time was: \n')
out.write(mpt)
out.close()
"""replaced xrange with range--> appears that he used Python 2.x but I/ we expect to use Python 3.x. however
range may be using far to much more memory than xrange. What the hell is going on?

-also replaced cmp(a,b) with (a>b)-(a<b) --> see if comp returned -1,0, or 1 or an actual integer related 
to the  b/t a and b  -->> turns out that this created the issue that False-True in python returns true 
solved this by multiplying by one to force them to be interpreted as integers

-see if any other declarations or other stuff is wasting memory 

-finish setting up the timer to see how long it takes to search and update path 

-sizeX,Y,Z has to be bigger than or equal to minclustersize in config_user because otherwise will get zero for lsizeX,Y,Z when converted to int and will try and divide by zero 
in the declaration of that level L in the CL class declaration 

-turned use hierarchichal planning to false

-added n2g and g2n functions to all_functions.py --> write them to be able to make the conversion using only the initial GPS coordinates, the initial node, and the goal node(s)

-set Mavlink commands with Navio2
"""