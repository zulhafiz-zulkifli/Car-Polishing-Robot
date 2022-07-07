#!/usr/bin/env python3
from math import pi
from spatialmath import *
from spatialmath.base import *
from roboticstoolbox import *
from roboticstoolbox.backends.PyPlot import PyPlot
import matplotlib.pyplot as plt
from robot import ROBOT

if __name__ == '__main__': 
    robot = ROBOT()
    robot.q = [0, 0, 0, 0]
    # generate path coordinates
    ik_points = []
    Px = -0.5
    Py = 1.2
    Pz = 0.51
    Xinterval = 5
    Yinterval = 1
    Xstep = 1.0 / Xinterval
    Ystep = 0.2 / Yinterval
    ik_points.append([Px,Py,Pz])
    for i in range(4):
        if (i==0 or i==2):
            while Px < 0.5:
                Px+=Xstep
                ik_points.append([Px,Py,Pz])
        else:
            while Px > -0.5:
                Px-=Xstep
                ik_points.append([Px,Py,Pz])
        Py-=Ystep
        if i!=3:ik_points.append([Px,Py,Pz])
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
    for i in ik_points:
        sol_hood = robot.ikine_LMS(SE3(i[0], i[1], i[2]))
        traj_hood = jtraj(robot.q, sol_hood.q, 15)
        for q in traj_hood.q:
            robot.q = q
            env.step(0.000001)
    # back to original position
    for q in jtraj(robot.q, [0,0,0,0], 30).q:
        robot.q = q
        env.step(0.000001)
    env.hold()