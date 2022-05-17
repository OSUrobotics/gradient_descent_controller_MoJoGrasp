#!/usr/bin/python3

import re
import xmltodict
import pprint
import numpy as np
from math import sin, cos, pi


class UrdfToKinematicChain():

    def __init__(self, urdf_file) -> None:
        
        robot_urdf = self.read_urdf(urdf_file=urdf_file)
        # pprint.pprint(robot_urdf['joint'][0], indent=2)
        number_of_joints = len(robot_urdf['joint'])
        self.joint_angles = [0]*number_of_joints
        # print(self.joint_angles)
        self.kinematic_chain = self.intial_kinematic_setup(robot_urdf['joint'])
        # pprint.pprint(self.kinematic_chain, indent=2)
        self.transformation_matrix = {}

        self.calculate_forward_kinematics()
        # pprint.pprint(self.transformation_matrix)
                 
    def update_joint_angles(self, angles:list):
        self.joint_angles = angles

    def calculate_forward_kinematics(self, joint_angles=None):
        if joint_angles == None:
            joint_angles = self.joint_angles
        
        joint_iterator = 0

        for i in range(len(self.kinematic_chain.keys())):
            finger_chain = self.kinematic_chain[f'finger{i}']
            
            previouse_transform = np.identity(4)
            for joint in finger_chain:
                trans = self.translation_matrix(joint['translation'])
                orient = self.rotation_matrix(rotation_axis=joint['orientation'])
                joint_rot = self.rotation_matrix(rotation_axis= joint['joint_axis'], angle=joint_angles[joint_iterator])
                previouse_transform = np.matmul(previouse_transform, np.matmul(trans, np.matmul(orient, joint_rot)))


                joint_iterator += 1

            self.transformation_matrix[f'finger{i}'] = previouse_transform

    def intial_kinematic_setup(self, joint_list):
        kinematic_chain = {}
        
        for i, joint in enumerate(joint_list):
            finger, segment, _ = joint['@name'].split('_')

            translation = joint['origin']['@xyz'].split(' ')
            orientation = joint['origin']['@rpy'].split(' ')
            joint_axis = joint['axis']['@xyz'].split(' ')
            joint_type = joint['@type']
            child_link = joint['child']['@link']
            try:
                kinematic_chain[finger].append({"joint_number": segment[-1],
                                                "translation":translation, 
                                                "orientation":orientation, 
                                                "joint_axis":joint_axis,
                                                "joint_type":joint_type,
                                                "child_link": child_link})
            except:
                kinematic_chain[finger] = [{"joint_number": segment[-1],
                                            "translation":translation, 
                                            "orientation":orientation, 
                                            "joint_axis":joint_axis,
                                            "joint_type":joint_type,
                                            "child_link": child_link}]
        return kinematic_chain


    def rotation_matrix(self, rotation_axis:list, angle=None):

        rot_mat = np.identity(4)

        if angle == None:
            temp_vec = np.array(rotation_axis, dtype=float)
            angle = np.linalg.norm(temp_vec)

            if angle == 0: # no rotation return identity matrix
                return rot_mat
            # print(f'incoming vector: {rotation_axis}, temp_vec: {temp_vec}, angle: {angle}')
            rotation_axis = temp_vec / angle
        # else:
            # print(f'incoming vector: {rotation_axis}, angle: {angle}')
        
        if rotation_axis[0] == 1:
            rot_mat[:3, :3] = np.array([[1, 0, 0],
                                        [0, cos(angle), -sin(angle)],
                                        [0, sin(angle), cos(angle)]])
        elif rotation_axis[1] == 1:
            rot_mat[:3, :3] = np.array([
                                        [cos(angle), 0, sin(angle)],
                                        [0, 1, 0]
                                        [-sin(angle), 0, cos(angle)]])
        elif rotation_axis[2] == 1:
            rot_mat[:3, :3] = np.array([[cos(angle), -sin(angle), 0],
                                        [sin(angle), cos(angle), 0],
                                        [0, 0, 1]])

        return rot_mat

    def translation_matrix(self, translation=list):
        trans_mat = np.identity(4)
        trans_mat[:3, 3] = translation
        # print(trans_mat)
        return trans_mat
        

    def read_urdf(self, urdf_file="./resources/test_hand/test_hand.urdf"):
        with open(urdf_file) as f:
            urdf = xmltodict.parse(f.read())
        
        return urdf['robot']




if __name__ == '__main__':

    kinematics = UrdfToKinematicChain("./resources/test_hand/test_hand.urdf")
    results = np.matmul(kinematics.transformation_matrix['finger0'], np.array([1, 0, 0, 1]))
    print(results)
