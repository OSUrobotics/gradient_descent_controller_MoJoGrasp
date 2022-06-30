#!/usr/bin/python3

from cmath import pi, cos, sin
from multiprocessing.dummy import current_process
from ntpath import join
import pybullet as p

import numpy as np

from modified_mojograsp_classes import UpdatedTwoFingerGripper
from modified_mojograsp_classes import UpdatedObjectBase


class GradientDescent():
    MAX_ERROR = 0.0001 # meters
    STEP_SIZE = pi/50 # radients

    def __init__ (self, hand: UpdatedTwoFingerGripper, finger:str, contact_in_distal:list , goal_contact_pose):
        self.hand = hand
        self.finger = finger
        self.contact_in_distal = contact_in_distal
        self.goal_contact_pose = goal_contact_pose


        self.starting_joint_angles = hand.get_joint_angles()
        self.new_joint_angles = self.starting_joint_angles.copy()
        self.palm_to_world = np.identity(4) 
        self.palm_to_world[0:3, 3] = hand.setup_param["position"]
        self.palm_to_world[0:3, 0:3] = np.reshape(p.getMatrixFromQuaternion(hand.setup_param["orientation"]), (3,3))
 
        

    
    def gradient_calculator(self):
        
        while True:
            previous_delta = self.delta_calculator(self.new_joint_angles)
            if previous_delta < self.MAX_ERROR:
                break
            
            update_joint_angles = self.new_joint_angles.copy()
            
            for joint_index in self.hand.fingers[self.finger]['index_values']:
                step_size = self.STEP_SIZE
                update_joint_angles[joint_index] += 0.00001 # very small step to see the direction to move in.
                
                if self.delta_calculator(update_joint_angles) < previous_delta:
                    step_size *= 1
                else:
                    step_size *= -1

                while True:
                    update_joint_angles[joint_index] = self.new_joint_angles[joint_index] + step_size
                    updated_delta = self.delta_calculator(update_joint_angles)
                    if updated_delta < previous_delta:
                        previous_delta = updated_delta



        return self.new_joint_angles
    

    def delta_calculator(self, joint_angles: list) -> float:
        transform = self.hand.kinematics.calculate_forward_kinematics(joint_angles)
        contact_in_world = np.matmul(self.palm_to_world, np.matmul(transform[self.finger], self.contact_in_distal))
        delta_vector = contact_in_world - self.goal_contact_pose
        return np.linalg.norm(delta_vector)

    # def gradient_calculations(self, next_contact_points: list):

    #     for finger in range(len(self.fingers.keys())):
    #         joint_angles = self.gripper.get_joint_angles(joint_numbers=self.fingers[f'finger{finger}'])
    #         for link in self.fingers[f'finger{finger}']:
    #             pass
