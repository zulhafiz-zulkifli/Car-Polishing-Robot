#!/usr/bin/env python3
from math import pi
from spatialmath import *
from roboticstoolbox import *

class ROBOT(DHRobot):
    def __init__(self):
        deg = pi/180
        L1 = PrismaticDH(theta=0, a=0, alpha=0, qlim=[0,1.5], offset=1.5)
        L2 = RevoluteDH(a=0.8, d=0, alpha=0, qlim=[-180*deg,180*deg], offset=0)
        L3 = RevoluteDH(a=0.7, d=0, alpha=180*deg, qlim=[-180*deg,180*deg], offset=0)
        L4 = PrismaticDH(theta=0, a=0, alpha=0, qlim=[0,1.0], offset=0)
        super().__init__([L1, L2, L3, L4], name='Car Polishing Robot')
        self.base = SE3(0, 0, 0)