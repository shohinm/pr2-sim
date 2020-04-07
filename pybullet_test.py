#!/usr/bin/env python3
import numpy as np
import pybullet as p
import pybullet_data
import time
import copy
import pdb
import os

from ss_pybullet.pybullet_tools.utils import joint_from_name, joints_from_names, get_subtree_aabb, get_joints
from ss_pybullet.pybullet_tools.pr2_utils import PR2_GROUPS, REST_LEFT_ARM

# TODO: confirm camera params
FOV, ASPECT, NEARPLANE, FARPLANE = 60, 640.0/480.0, 0.01, 100
# TODO: decide image width, height
IMG_W = 200
IMG_H = 200
CAMERA_HEIGHT = 1.0

def addBox(obj):
    xyz = [obj[0], obj[1], obj[2]]
    rpy = [obj[3], obj[4], obj[5]]
    half_extents = [obj[6], obj[7], obj[8]]
    mass = 0 if obj[9] == 0 else obj[9]

    vis_id = p.createVisualShape(shapeType=p.GEOM_BOX,
                            halfExtents=half_extents,
                            rgbaColor=list(np.random.rand(3)) + [1],
                            specularColor=list(np.random.rand(3)))
    coll_id = p.createCollisionShape(shapeType=p.GEOM_BOX,
                            halfExtents=half_extents)
    body_id = p.createMultiBody(baseMass=mass,
                            baseCollisionShapeIndex=coll_id,
                            baseVisualShapeIndex=vis_id,
                            basePosition=xyz,
                            baseOrientation=p.getQuaternionFromEuler(rpy),
                            baseInertialFramePosition=[0, 0, 0],
                            baseInertialFrameOrientation=[0, 0, 0, 1])
    p.changeDynamics(body_id, -1, lateralFriction=obj[10])

    return body_id

def initImage(table):
    table_xyz = [table[0], table[1], table[2]]
    pos = copy.deepcopy(table_xyz)
    pos[2] += CAMERA_HEIGHT
    quat = p.getQuaternionFromEuler([0, 0, -np.pi/2])
    R = p.getMatrixFromQuaternion(quat)
    R = np.array(R).reshape(3, 3)

    table_height = table[8] * 2.0
    farplane = min(FARPLANE, CAMERA_HEIGHT + table_height)
    p_mat = p.computeProjectionMatrixFOV(
                                        FOV, ASPECT, NEARPLANE, farplane)
    v_mat = p.computeViewMatrix(pos, table_xyz, R.dot([0, 1, 0]))
    img = p.getCameraImage(IMG_W, IMG_H, v_mat, p_mat)







def main():

    PR2_URDF = './ss_pybullet/models/pr2_description/pr2.urdf'

    p.connect(p.GUI)
    p.setGravity(0, 0, -10)
    # Disable auto real-time simulation
    p.setRealTimeSimulation(1)
    # Defualt is 240 Hz, do not change
    p.setTimeStep(1.0/240.0)

    # Add plane
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    plane_id = p.loadURDF("plane.urdf")

    # Add PR2
    start_pos = [0, 0, 0]
    start_orientation = p.getQuaternionFromEuler([0, 0, 0])
    urdf_path =  PR2_URDF
    robot_id = p.loadURDF(urdf_path, start_pos, start_orientation, useFixedBase=1)
    p.stepSimulation()

    # Add table [x, y, z, roll, pitch, yaw, x-size, y-size, z-size, mass, friction]
    # for table, mass = 0 => table is immovable, unaffected by gravity etc.
    table = [0.55, 0.0, 0.6, 0.0, 0.0, 0.0, 0.3, 0.6, 0.02, 0, 1.0]
    table_id = addBox(table)

    # disable collisions with table
    for joint in get_joints(robot_id):
        p.setCollisionFilterPair(robot_id, table_id, joint, -1,
                                                enableCollision=0)

    init_state = [0.16825, -1.69017, 0.0788625, -1.14571, -1.2717, -0.151824, -1.25254, 0.675062]
    init_joints = ['torso_lift_joint', 'r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']

    # Set initial position - helper fuction from ss-pybullet also
    joint_idxs = joints_from_names(robot_id, init_joints)
    p.setJointMotorControlArray(
            robot_id, joint_idxs,
            controlMode=p.POSITION_CONTROL,
            targetPositions=init_state)
    for i in range(400):
        p.stepSimulation()
        if (i % 100) == 0:
            initImage(table)

    # Close gripper
    gripper_joints = joints_from_names(robot_id, PR2_GROUPS['right_gripper'])
    p.setJointMotorControlArray(
            robot_id, gripper_joints,
            controlMode=p.POSITION_CONTROL,
            targetPositions=len(gripper_joints) * [0.0])

    for i in range(400): # TODO: do not hardcode these values!
        p.stepSimulation()
        if (i % 100) == 0:
            initImage(table)

    # Set other arm
    joint_idxs = joints_from_names(robot_id, PR2_GROUPS['left_arm'])
    position = REST_LEFT_ARM
    p.setJointMotorControlArray(
            robot_id, joint_idxs,
            controlMode=p.POSITION_CONTROL,
            targetPositions=position)

    for i in range(400): # TODO: do not hardcode these values!
        p.stepSimulation()
        if (i % 100) == 0:
            initImage(table)

    input("Press key to terminate.")



if __name__ == "__main__":
    main()

