import swift
import roboticstoolbox as rtb
from spatialmath import SE3
import time
import numpy as np

# env = swift.Swift()
# env.launch(realtime=True)

def degree_to_rad(degree):
    return degree * (np.pi / 180)

def rad_to_degree(rad):
    return rad * (180 / np.pi)  

def better_ik(current_joint, Tep_goal):
    sol = robot.ik_LM(Tep_goal, q0 = current_joint)  
    q_pickup = sol[0]
    # for i in range(7):
    #     while abs(current_joint[i]-q_pickup[i]) > np.pi:
    #         q_pickup[i] = q_pickup[i] + 2*np.pi * np.sign(current_joint[i]-q_pickup[i])
    return q_pickup

robot = rtb.models.Realman()
print(robot)

Tep = robot.fkine(robot.qil)
print(robot.qil)
print(Tep)

Tep_goal = SE3.Trans(0, 0, -0.1) * Tep
# print(Tep_goal)

time_start = time.time()
q_goal = better_ik(robot.qil, Tep_goal)
print(q_goal)
qt = rtb.jtraj(robot.qil, q_goal, 50)
print("Time: ", time.time() - time_start)

# print("Time: ", time.time() - time_start)
robot.plot(qt.q, backend='pyplot', movie='panda1.gif')
# 把这里的集成到ros_link中，改成四元数的

