from Pr2Sim import *
import pdb
import numpy as np
from ss_pybullet.pybullet_tools.pr2_utils import PR2_GROUPS, REST_LEFT_ARM, WIDE_LEFT_ARM
from ss_pybullet.pybullet_tools.utils import joint_from_name, joints_from_names, get_subtree_aabb, get_joints



def main():
    sim = Pr2Sim()
    sim.SetAutoSimStep(False)
    pr2_urdf_path = './ss_pybullet/models/pr2_description/pr2.urdf'
    crate_urdf_path = './resources/models/crate.urdf'
    sim.LoadModel('robot', pr2_urdf_path, start_pos = [0, 0, 0])
    sim.LoadModel('crate', crate_urdf_path, start_pos = [0.5, 0, 0])

    table_pose = np.array([0.55, 0.0, 0.6, 0.0, 0.0, 0.0])
    table_size_xyz = 2*np.array([0.3, 0.6, 0.02])
    table_mass = 0
    table_friction = 1.0
    # camera_target_xyz = np.array([table_pose[0], table_pose[1], table_pose[2] + table_size_xyz[2]/2])
    # camera_height = 1.0
    camera_target_xyz = np.array([0.55, 0.0, 0.0])
    camera_height = 1.6

    # bid = sim.AddBox('table', table_pose, table_size_xyz, table_mass, table_friction)
    sim.AddCamera(camera_target_xyz, camera_height)

    pdb.set_trace()

    # init_state = [0.16825, -1.69017, 0.0788625, -1.14571, -1.2717, -0.151824, -1.25254, 0.675062]

    init_state = [1.69017, 0.0788625, -1.14571, -1.2717, -0.151824, -1.25254, 0.675062]

    init_joints = ['r_shoulder_pan_joint', 'r_shoulder_lift_joint', 'r_upper_arm_roll_joint', 'r_elbow_flex_joint', 'r_forearm_roll_joint', 'r_wrist_flex_joint', 'r_wrist_roll_joint']
    sim.SetJointPositions('robot', init_joints, init_state)


    # position = REST_LEFT_ARM
    # sim.SetJointPositions('robot', PR2_GROUPS['left_arm'], position)

    for i in range(400):
        sim.Step()
    
    input("Press key to terminate.")


if __name__ == '__main__':
    main()
