#!/usr/bin/python3


# Author: Josh Campbell
# Date: 7-6-2022


from multiprocessing.dummy import current_process
import pybullet as p

import numpy as np


from modified_mojograsp_classes import UpdatedObjectBase, UpdatedTwoFingerGripper
from gradient_descent import GradientDescent

import logging, colorlog
import helper_functions as HF

logger = HF.colored_logging(name="hand_controller")


class HandController():


    def __init__(self, gripper: UpdatedTwoFingerGripper, cube: UpdatedObjectBase, ) -> None:
        self.hand = gripper
        self.cube = cube
        self.MAX_MOVE = 0.0005
        self.end_effector_links = [self.hand.fingers[finger]["index_values"][-1] for finger in self.hand.fingers.keys()]
        self.gradient_failed = False
        self.goal_position = None
        self.current_cube_pose = None
        self.prev_distance = 0
        self.distance_count = 0


    def get_current_cube_position(self):
        # this is the location of the origin defined by the urdf in the world frame
        self.current_cube_pose = self.cube.get_curr_pose() 

    def get_next_cube_position(self):
        
        current_position = np.array(self.current_cube_pose[0][:2])

        goal_position = np.array(self.goal_position[:2])
        logger.debug(f'\ngoal_position: {goal_position}\ncurrent_position: {current_position}')
        
        distance_to_subgoal = goal_position - current_position
        euclidean_dist = np.linalg.norm(distance_to_subgoal)
        if euclidean_dist > self.MAX_MOVE:
            distance_to_subgoal = distance_to_subgoal * (self.MAX_MOVE / abs(euclidean_dist))
            next_position = current_position + distance_to_subgoal
            logger.debug(f'distance_moved: {np.linalg.norm(distance_to_subgoal)}')
        else:
            next_position = current_position + distance_to_subgoal
        logger.debug(f'\neuclidean_dist:{euclidean_dist}\nnext_position:{np.around(next_position,5)} \ndistance_to_goal{np.around(distance_to_subgoal,5)}')
        return np.around(next_position,5), np.around(distance_to_subgoal,5)
    
    def get_current_contact_points(self):
        contact_points = {}
        for finger in self.hand.fingers:
            link_index = self.hand.fingers[finger]['index_values']
            link_index.sort()
            contact_point_info = p.getContactPoints(self.cube.id, self.hand.id, linkIndexB=link_index[-1])
            if contact_point_info:
                # print(f"\n\n{contact_points}\n\n")
                contact_points[finger] = np.around(np.array(contact_point_info[0][6]), 5)
            else:
                return None
        
        return contact_points # global coordinates

    def retry_contact(self):
        
        location = np.array(self.current_cube_pose[0])
        next_pose = []
        distal_links = []
        
        for finger in self.hand.fingers:
            link_ids = self.hand.fingers[finger]["index_values"]
            link_ids.sort()
            distal_links.append(link_ids[-1])
            distal_pose = p.getLinkState(self.hand.id, link_ids[-1])
            current_position = np.array(distal_pose[4])
            distance_to_subgoal = location - current_position
            euclidean_dist = np.linalg.norm(distance_to_subgoal)
            if euclidean_dist > self.MAX_MOVE:
                next_pose.append(current_position + (distance_to_subgoal * (self.MAX_MOVE / abs(euclidean_dist))))
            else:
                next_pose.append(current_position + distance_to_subgoal)

        goal_angles = p.calculateInverseKinematics2(bodyUniqueId=self.hand.id, 
                                                    endEffectorLinkIndices=distal_links, 
                                                    targetPositions=next_pose)
        
        # print(f'\n\n{goal_angles}\n\n')
        return goal_angles



    def get_next_contact_points(self, current_contact_points: dict, distance_to_subgoal: list):
        next_contact = {}
        for finger in self.hand.fingers:
            next_contact[finger] = np.around(np.array([current_contact_points[finger][0] + distance_to_subgoal[0],
                                    current_contact_points[finger][1] + distance_to_subgoal[1],
                                    current_contact_points[finger][2]]),5)
        
        return next_contact # global coordinates
    
    def get_next_link_positions(self, current_contact_points: dict, next_contact_points: dict):
        next_link_positions_global = {}
        self.hand.kinematics.update_joint_angles = self.hand.get_joint_angles()
        
        for finger in self.hand.fingers:  # TODO: change to be numpy matrix math - probably ...
            logger.debug(f'{finger}')
            contact_point = np.around(np.array([current_contact_points[finger][0], current_contact_points[finger][1], 
                                        current_contact_points[finger][2], 1]), 5)
            finger_pose = [(0,0,0),(0,0,0,1)]
            T_distal_to_palm = self.hand.kinematics.calculate_forward_kinematics() #[(0,0,0),(0,0,0,1)]
            # orientation_int = np.reshape(T_distal_palm_matrix[0:3, 0:3], (1,9))
            # T_distal_palm = [T_distal_palm_matrix[0:3,3], orientation_int]
            # T_palm_world = [(0,0,0),(0,0,0,1)]
            T_palm_to_world = np.identity(4)
            T_palm_to_world[0:3,3] = self.hand.setup_param["position"]
            T_palm_to_world[0:3,0:3] = np.around(np.reshape(p.getMatrixFromQuaternion(self.hand.setup_param["orientation"]), (3,3)), 5)
            # [self.hand.setup_param["position"], self.hand.setup_param["orientation"]]
            # T_distal_to_world =  p.multiplyTransforms(, T_distal_palm[1], T_palm_to_world[0], T_palm_to_world[1])
            
            logger.debug(f'\n{T_palm_to_world}  \n{T_distal_to_palm[finger]}')
            T_distal_to_world = np.matmul(T_palm_to_world, T_distal_to_palm[finger])
            T_world_to_distal = np.linalg.inv(T_distal_to_world)
            
            
            contact_point_distal = np.matmul(T_world_to_distal, contact_point)
            logger.debug(f'T_world_to_distal \n{T_world_to_distal} \n contact_point\n{contact_point}\ncontact_point_distal\n{contact_point_distal}\nnext_contact_points\n{next_contact_points[finger]}')
            GD = GradientDescent(self.hand, finger, contact_point_distal, next_contact_points[finger])
            new_angles = GD.gradient_calculator()
            if new_angles == False:
                self.gradient_failed = True
                return False
            logger.debug(f'{new_angles}')
            self.hand.kinematics.joint_angles = new_angles
            next_link_positions_global[finger] = new_angles
            # goals.append(new_angles)
                
        return new_angles

    def set_goal_position(self, position:list):
        self.goal_position = np.array(position)

    def check_goal(self):
        distance = np.linalg.norm(self.goal_position[:2] - self.current_cube_pose[0][:2])
        return distance

    def exit_condition(self):
        # checks if we are getting further from goal or closer
        if self.prev_distance < self.check_goal():
            self.distance_count += 1
        else:
            self.distance_count = 0

        # Exits if we lost contact for 5 steps, we are within .01 of our goal, or if our distance has been getting worse for 20 steps
        if self.num_contact_loss > 10:
            logger.debug(f'number of contact loss {self.num_contact_loss}\n\n')
            self.distance_count = 0
            self.num_contact_loss = 0
            self.gradient_failed = False
            return True
        elif self.check_goal() < .01:
            logger.debug('check goal is less than 0.01\n\n')
            self.distance_count = 0
            self.num_contact_loss = 0
            self.gradient_failed = False
            return True

        elif self.distance_count > 20:
            logger.debug(f'distance count {self.distance_count}\n\n')
            self.distance_count = 0
            self.num_contact_loss = 0
            self.gradient_failed = False
            return True
        # sets next previous distance to current distance
        elif self.gradient_failed == True:
            logger.debug(f'gradient failed\n\n')
            self.distance_count = 0
            self.num_contact_loss = 0
            self.gradient_failed = False
            return True
        self.prev_distance = self.check_goal()
        return False


    def get_next_action(self):
        # get current cube position
        self.get_current_cube_position()
        # get next cube position
        next_cube_position, distance_to_subgoal = self.get_next_cube_position()
        # get current contact points
        current_contact_points = self.get_current_contact_points()
        # print(f"\n\n{current_contact_points}\n\n")

        if current_contact_points:
            self.num_contact_loss = 0
            # find next contact points
            next_contact_points = self.get_next_contact_points(
                current_contact_points=current_contact_points, distance_to_subgoal=distance_to_subgoal)
            # get goal link positions
            goal = self.get_next_link_positions(
                current_contact_points=current_contact_points, next_contact_points=next_contact_points)
            if goal == False:
                return False
        else:
            self.num_contact_loss += 1
            goal = self.retry_contact()
        
        return goal

