#!/usr/bin/python3


from itertools import chain
import pybullet as p
import numpy as np
from math import sin, cos, pi
import logging, colorlog

class Helper():
    def __init__(self) -> None:
        pass
    
    @staticmethod
    def load_mesh(mesh_setup = None):

        mesh_id = p.loadURDF(mesh_setup["path"], useFixedBase=mesh_setup["fixed"], 
                             basePosition=mesh_setup["position"], 
                             baseOrientation=mesh_setup["orientation"],
                             globalScaling=mesh_setup["scaling"])
        
        return mesh_id


def colored_logging(name:str):
    logger = logging.getLogger(name=name)

    handler = logging.StreamHandler()
    handler.setFormatter(colorlog.ColoredFormatter("\n%(log_color)s%(levelname)s - %(name)s - %(message)s%(reset)s\n\n", log_colors={'DEBUG':'cyan', 'WARNING':'yellow', "ERROR":'red'}))
    logger.addHandler(handler)
    logger.setLevel(logging.ERROR)
    logger.propagate = False
    return logger


# class FingerKinematics():
#     def __init__(self, id_chain: list, body_id) -> None:
#         self.body_id = body_id
#         self.joint_ids = []
#         self.joint_info = {}

#     def setup_kinematics(self, id_chain):
#         base_trans, base_orientation = p.getBasePositionAndOrientation(self.body_id)
#         palm_id = -1
#         self.joint_info[-1] = {"axis_rotation": None,
#                                 "parent_id": "world",
#                                 "translation_from_parent": base_trans,
#                                 "orientation_from_parent": base_orientation }

#         for joint_id in id_chain:
#             self.joint_ids.append(joint_id)
#             info = p.getJointInfo(self.body_id, joint_id)

#             self.joint_info[joint_id] = {
#                                             "axis_rotation": info[-4], # or 13
#                                             "parent_id": info[-1],
#                                             "translation_from_parent": info[-3],
#                                             "orientation_from_parent": info[-2],
#                                             "joint_angle": 0 }
        
#         self.joint_info["distal_link"] = {"translation_to_tip": }
#         self.joint_ids.sort()


#     def update_joints(self, updated_joint_angle:list):
        
#         for i, joint_id in enumerate(self.joint_ids):
#             self.joint_info[joint_id]["joint_angle"] = updated_joint_angle[i]
        

#     def get_finger_tip_loc(self):
#         pass

#     def rotation_matrix(self, angle:float, axis_of_rotation=[0,0,1]):

#         if axis_of_rotation == [0,0,1]:
#             rot = np.array([
#                 [cos(angle), -sin(angle), 0],
#                 [sin(angle), cos(angle), 0],
#                 [0, 0, 1]])
#         elif axis_of_rotation == [1, 0, 0]:
#             rot = np.array([
#                 [1, 0, 0],
#                 [0, cos(angle), -sin(angle)],
#                 [0, sin(angle), cos(angle)]])
#         elif axis_of_rotation == [0, 1, 0]:
#             rot = np.array([
#                 [cos(angle), 0, sin(angle)],
#                 [0, 1, 0],
#                 [-sin(angle), 0, cos(angle)]
#             ])
#         return rot
    
#     def transform_matrix(self, rotation_matrix, translation):
#         transform = np.identity((4,4))
#         transform[:3, :3] = rotation_matrix
#         transform[3, :3] = translation

#         return transform
