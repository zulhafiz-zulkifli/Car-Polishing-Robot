#!/usr/bin/env python3
from math import pi
from spatialmath.base import *
from roboticstoolbox import *
from roboticstoolbox.backends.PyPlot import PyPlot
import matplotlib.pyplot as plt
from robot import ROBOT

if __name__ == '__main__':
    robot = ROBOT()
    robot.q = [0, 0, 0, 0]
    # using joint position from calculation
    deg = pi/180
    d1 = -0.495
    d4 = 0.495
    calc_traj = [[d1,84.824,60,d4], [d1,80.119,60,d4], [d1,68.900,60,d4], 
                 [d1,55.509,60,d4], [d1,44.289,60,d4], [d1,39.584,60,d4], 
                 [d1,32.260,71.912,d4], [d1,24.936,83.849,d4], [d1,30.456,83.849,d4], 
                 [d1,43.632,83.849,d4], [d1,59.357,83.849,d4], [d1,72.532,83.849,d4], 
                 [d1,78.057,83.849,d4], [d1,76.802,93.092,d4], [d1,75.545,102.355,d4], 
                 [d1,68.879,102.374,d4], [d1,53.001,102.374,d4], [d1,34.050,102.374,d4], 
                 [d1,18.171,102.374,d4], [d1,11.509,102.374,d4], [d1,4.600,110.100,d4], 
                 [d1,-2.453,117.747,d4], [d1,5.820,117.664,d4], [d1,25.556,117.664,d4], 
                 [d1,49.114,117.664,d4], [d1,68.850,117.664,d4], [d1,77.122,117.664,d4]]
    for i in calc_traj:
        i[1] = i[1]*deg
        i[2] = i[2]*deg
    # robot environment setup
    fig = plt.figure()
    env = PyPlot()
    env.launch(name='Robotics Project',fig=fig,block=False)
    trplot(transl(-0.5,0.5,0.51), width=0.2,color="black",axislabel=False,length=[0,0.8,0],style="line",originsize=5,projection="persp")
    trplot(transl(0.5,1.3,0.51), width=0.2,color="black",axislabel=False,length=[0,-0.8,0],style="line",originsize=5,projection="persp")
    trplot(transl(0.5,0.5,0.51), width=0.2,color="black",axislabel=False,length=[-1,0,0],style="line",originsize=5,projection="persp")
    trplot(transl(-0.5,1.3,0.51), width=0.2,color="black",axislabel=False,length=[1,0,0],style="line",originsize=5,projection="persp")
    # add robot to the environment
    env.add(robot, jointlabels=False, eeframe=True, name=False)
    # delay for 10 seconds
    for i in range(10):
        env.step(1)
    # calculate trajectories
    for j in calc_traj:
        traj_hood = jtraj(robot.q, j, 15)
        for q in traj_hood.q:
            robot.q = q
            env.step(0.0000001)
    # back to original position
    for q in jtraj(robot.q, [0,0,0,0], 30).q:
        robot.q = q
        env.step(0.000001)
    env.hold()