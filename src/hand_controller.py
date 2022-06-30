from multiprocessing.dummy import current_process
import pybullet as p

import numpy as np


from modified_mojograsp_classes import UpdatedObjectBase, UpdatedTwoFingerGripper
from gradient_descent import GradientDescent


class HandController():

    MAX_MOVE = 0.01

    def __init__(self, gripper: UpdatedTwoFingerGripper, cube: UpdatedObjectBase, ) -> None:
        self.hand = gripper
        self.cube = cube

        self.end_effector_links = [self.hand.fingers[finger]["index_values"][-1] for finger in self.hand.fingers.keys()]

        self.goal_position = None
        self.current_cube_pose = None


    def get_current_cube_position(self):
        # this is the location of the origin defined by the urdf in the world frame
        self.current_cube_pose = self.cube.get_curr_pose() 

    def get_next_cube_position(self):
        
        current_position = np.array(self.current_cube_pose[0])

        goal_position = np.array(self.goal_position)
        
        distance_to_subgoal = goal_position - current_position
        euclidean_dist = np.linalg.norm(distance_to_subgoal)
        if euclidean_dist > self.MAX_MOVE:
            next_position = current_position + (distance_to_subgoal * (self.MAX_MOVE / abs(euclidean_dist)))
        else:
            next_position = current_position + distance_to_subgoal


        return next_position, distance_to_subgoal
    
    def get_current_contact_points(self):
        contact_points = {}
        for finger in self.hand.fingers:
            link_index = self.hand.fingers[finger]['index_values'].sort()
            contact_point_info = p.getContactPoints(self.cube.id, self.hand.id, linkIndexB=link_index[-1])
            if contact_point_info:
                contact_points[finger] = np.array(contact_point_info[0][6])
            else:
                return None
        
        return contact_points # global coordinates

    def retry_contact(self):
        pass


    def get_next_contact_points(self, current_contact_points: dict, distance_to_subgoal: list):
        next_contact = {}
        for finger in self.hand.fingers:
            next_contact[finger] = current_contact_points[finger] - distance_to_subgoal
        
        return next_contact # global coordinates
    
    def get_next_link_positions(self, current_contact_points: dict, next_contact_points: dict):
        next_link_positions_global = []
        for finger in self.hand.fingers:  # TODO: change to be numpy matrix math - probably ...
            contact_point = [current_contact_points[finger], (0,0,0,1)]
            finger_pose = [(0,0,0),(0,0,0,1)]
            T_distal_palm = [(0,0,0),(0,0,0,1)]
            # T_palm_world = [(0,0,0),(0,0,0,1)]
            T_palm_to_world = [self.hand.setup_param["position"], self.hand.setup_param["orientation"]]
            T_distal_to_world =  p.multiplyTransforms(T_distal_palm[0], T_distal_palm[1], T_palm_to_world[0], T_palm_to_world[1])
            T_world_to_distal = p.invertTransform(T_distal_to_world)

            contact_point_distal, _ = p.multiplyTransforms(T_world_to_distal, contact_point)

            new_joint_angles = GradientDescent.gradient_calculator(self.hand, finger, contact_point_distal, next_contact_points[finger])







