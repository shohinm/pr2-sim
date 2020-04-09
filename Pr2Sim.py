import pybullet as p
import pybullet_data
import time
import copy
import pdb
import os
import numpy as np
# from .ss_pybullet.pybullet_tools.utils import joint_from_name, joints_from_names, get_subtree_aabb, get_joints
try:
    from .ss_pybullet.pybullet_tools.utils import joint_from_name, joints_from_names, get_subtree_aabb, get_joints
except Exception: #ImportError
    from ss_pybullet.pybullet_tools.utils import joint_from_name, joints_from_names, get_subtree_aabb, get_joints

class Pr2Sim:
    def __init__(self, gui = True, dt = 1.0/240.0): 
        p.connect(p.GUI) if gui else p.connect(p.DIRECT)
        p.setGravity(0, 0, -10)
        # Disable auto real-time simulation
        self.SetAutoSimStep(False)
        # Defualt is 240 Hz, do not change
        p.setTimeStep(1.0/240.0)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        self.model_name_id_dict = {}
        self.model_joint_to_id_dict = {}
        self.model_link_to_id_dict = {}

        self.LoadModel('plane', 'plane.urdf')

    def SetAutoSimStep(self, auto_step):
        p.setRealTimeSimulation(1) if auto_step else p.setRealTimeSimulation(0) 
             
    def Step(self):
        p.stepSimulation()
        self.GetCameraImage()

    def LoadModel(self, model_name, model_path, start_pos = [0, 0, 0], start_orientation = [0, 0, 0]):
        start_orientation = p.getQuaternionFromEuler(start_orientation)
        self.model_name_id_dict[model_name] = p.loadURDF(model_path, start_pos, start_orientation, useFixedBase=1)

        if model_name not in self.model_joint_to_id_dict:
            self.model_joint_to_id_dict[model_name] = {}
        if model_name not in self.model_link_to_id_dict:
            self.model_link_to_id_dict[model_name] = {}
        self.model_link_to_id_dict[model_name] = {p.getBodyInfo(self.model_name_id_dict[model_name])[0].decode('UTF-8'):-1,}

        for _id in range(p.getNumJoints(self.model_name_id_dict[model_name])):
            joint_name = p.getJointInfo(self.model_name_id_dict[model_name], _id)[1].decode('UTF-8')
            link_name = p.getJointInfo(self.model_name_id_dict[model_name], _id)[12].decode('UTF-8')
            self.model_joint_to_id_dict[model_name][joint_name] = _id
            self.model_link_to_id_dict[model_name][link_name] = _id
        
    def AddBox(self, model_name, pose, size, mass, friction):
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
        self.model_name_id_dict[model_name] = body_id
        
        # return body_id    

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

    def MoveJoints(self, model_name, joint_names, target_positions):
        p.setJointMotorControlArray(self.model_name_id_dict[model_name], joints_from_names(self.model_name_id_dict[model_name], joint_names), 
            controlMode=p.POSITION_CONTROL, targetPositions = target_positions)

    def GetNumJoints(self, model_name):
        return p.getNumJoints(self.model_name_id_dict[model_name])

    def GetJointState(self, model_name, joint_name):
        return p.getJointState(self.model_name_id_dict[model_name], self.model_joint_to_id_dict[model_name][joint_name])

    def GetJointStates(self, model_name, joint_names):
        return [self.GetJointState(model_name, joint_name) for joint_name in joint_names] 

    def GetAllLinkNames(self, model_name):
        return self.model_joint_to_id_dict[model_name].keys()

    def GetLinkState(self, model_name, link_name):
        if self.model_link_to_id_dict[model_name][link_name] == -1:
            return p.getBasePositionAndOrientation(self.model_name_id_dict[model_name]) 
        else:
            return p.getLinkState(self.model_name_id_dict[model_name], self.model_link_to_id_dict[model_name][link_name])

    def GetLinkStates(self, model_name, link_names):
        return [self.GetLinkState(model_name, link_name) for link_name in link_names]

    def GetAllLinkNames(self, model_name):
        return self.model_link_to_id_dict[model_name].keys()



