import pybullet as p
import pybullet_data
import time
import copy
import pdb
import os
import numpy as np
from ss_pybullet.pybullet_tools.utils import joint_from_name, joints_from_names, get_subtree_aabb, get_joints

class Pr2Sim:
    def __init__(self, gui = True): 
        p.connect(p.GUI) if gui else p.connect(p.DIRECT)
        p.setGravity(0, 0, -10)
        # Disable auto real-time simulation
        self.SetAutoSimStep(False)
        # Defualt is 240 Hz, do not change
        p.setTimeStep(1.0/240.0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.model_name_id_dict = {}
        self.LoadModel('plane', 'plane.urdf')

    def SetAutoSimStep(self, auto_step):
        p.setRealTimeSimulation(1) if auto_step else p.setRealTimeSimulation(0) 
             
    def Step(self):
        p.stepSimulation()
        self.GetCameraImage()

    def LoadModel(self, model_name, model_path, start_pos = [0, 0, 0], start_orientation = [0, 0, 0]):
        start_orientation = p.getQuaternionFromEuler(start_orientation)
        self.model_name_id_dict[model_name] = p.loadURDF(model_path, start_pos, start_orientation, useFixedBase=1)

    def AddBox(self, pose, size, mass, friction):
        xyz = [pose[0], pose[1], pose[2]]
        rpy = [pose[3], pose[4], pose[5]]
        half_extents = [size[0]/2, size[1]/2, size[2]/2]
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
        p.changeDynamics(body_id, -1, lateralFriction=friction)
        return body_id    

    def AddCamera(self, target_xyz, camera_height, fov = 60, aspect = 640.0/480.0, near_plane = 0.01, far_plane = 100, img_width = 200, img_height = 200):
        camera_xyz = [target_xyz[0], target_xyz[1], target_xyz[2] + camera_height]
        quat = p.getQuaternionFromEuler([0, 0, -np.pi/2])
        R = p.getMatrixFromQuaternion(quat)
        R = np.array(R).reshape(3, 3)
        far_plane = min(far_plane, camera_height)
        self.camera_img_width = img_width
        self.camera_img_height = img_height
        self.camera_projection_mat = p.computeProjectionMatrixFOV(fov, aspect, near_plane, far_plane)
        self.camera_view_mat = p.computeViewMatrix(camera_xyz, target_xyz, R.dot([0, 1, 0]))
        return self.GetCameraImage()
    
    def GetCameraImage(self):
        return p.getCameraImage(self.camera_img_width, self.camera_img_height, self.camera_view_mat, self.camera_projection_mat)

    def SetJointPositions(self, model_name, joint_names, target_positions):
        p.setJointMotorControlArray(self.model_name_id_dict[model_name], joints_from_names(self.model_name_id_dict[model_name], joint_names), 
            controlMode=p.POSITION_CONTROL, targetPositions = target_positions)
