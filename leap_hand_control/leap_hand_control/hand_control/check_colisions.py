from urdfpy import URDF
import numpy as np
import os
urdf_path = os.path.abspath("/home/beatrix/ros2_ws/src/leap_control/leap_hand_control/leap_hand_control/data/leap_right/robot.urdf")
robot = URDF.load(urdf_path)

joint_angles = np.ones(16)*np.pi #16 juntas
# fk_result = robot.link_fk(joint_angles)
# #fk = robot.link_fk()
# print(fk_result)

for link in robot.links:
    print(link.name)


for joint in robot.joints:
    print('{} '.format(
        joint.name
    ))

print(robot.base_link.name)

fk = robot.link_fk()
print(fk[robot.links[4]])
print(fk[robot.links[8]])
print(fk[robot.links[12]])
print(fk[robot.links[16]])




# robot.show(cfg={
#     '0': 0.0,
#     '1':0.0

#  })