from Pr2Sim import *
import pdb
import numpy as np
from ss_pybullet.pybullet_tools.pr2_utils import PR2_GROUPS, REST_LEFT_ARM
from ss_pybullet.pybullet_tools.utils import joint_from_name, joints_from_names, get_subtree_aabb, get_joints



def main():
    pr2_sim = Pr2Sim()
    pr2_urdf_path = './ss_pybullet/models/pr2_description/pr2.urdf'
    crate_urdf_path = './resources/models/crate.urdf'
    pr2_sim.LoadModel('robot', pr2_urdf_path, start_pos = [0, 0, 0])
    pr2_sim.LoadModel('crate', crate_urdf_path, start_pos = [0.5, 0, 0])

    table_pose = np.array([0.55, 0.0, 0.6, 0.0, 0.0, 0.0])
    table_size_xyz = 2*np.array([0.3, 0.6, 0.02])
    table_mass = 0
    table_friction = 1.0
   
    # camera_target_xyz = np.array([table_pose[0], table_pose[1], table_pose[2] + table_size_xyz[2]/2])
    # camera_height = 1.0
    camera_target_xyz = np.array([0.55, 0.0, 0.0])
    camera_height = 1.6

    # pr2_sim.AddBox(table_pose, table_size_xyz, table_mass, table_friction)
    pr2_sim.AddCamera(camera_target_xyz, camera_height)


    position = REST_LEFT_ARM
    pr2_sim.SetJointPositions('robot', PR2_GROUPS['left_arm'], position)

    pr2_sim.Step()
    input("Press key to terminate.")


if __name__ == '__main__':
    main()
