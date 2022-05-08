#!/usr/bin/python3

from cmath import pi, cos, sin
from multiprocessing.dummy import current_process
from ntpath import join
import pybullet as p

import numpy as np

from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
from mojograsp.simobjects.object_base import ObjectBase


class GradientDescentController():
    MAX_MOVE = 0.01
    
    def __init__(self, gripper: TwoFingerGripper, cube: ObjectBase, data_file: str = None) -> None:
        self.gripper = gripper
        self.cube = cube
        self.path = data_file

        # long term implementation will need these two lines
        self.fingers = {}
        self.setup_hand()

        self.gripper_kinematics = {}

        # short term using current hand using this
        self.end_effector_links = [1, 3] # change this so it can get the distal links for varing amounts of segments

        # world coordinates
        self.goal_position = None
        # world coordinates
        self.current_cube_pose = None

    def setup_hand(self):

        # finger0 is right and finger1 is left or if more than one finger they go from finger0 right in a counterclockwise order.
        # prev_finger_number = ' '
        # prev_segment_number = ' '
        for joint_name in self.gripper.get_joint_names():
            finger_number, segment_number, _ = joint_name.split('_')
            if finger_number in self.fingers:
                self.fingers[finger_number].append(self.gripper.joint_dict[joint_name])
                self.fingers[finger_number].sort()
            else:
                self.fingers[finger_number] = [self.gripper.joint_dict[joint_name]]
    
    def forward_kinematics(self, joint_list):
        
        for finger in self.fingers.keys():
            self.gripper_kinematics[finger] = {}
            for link in self.fingers[finger]:
                self.gripper
                self.gripper_kinematics[finger][link] = self.transform_matrix()

    
    def transform_matrix(self, angle=pi, axis=[0,0,1], translation=[0,0,0]):
        transform = np.identity(4)
        if axis == [0,0,1]:
            rot = np.array([
                [cos(angle), -sin(angle), 0],
                [sin(angle), cos(angle), 0],
                [0, 0, 1]])
        elif axis == [1, 0, 0]:
            rot = np.array([
                [1, 0, 0],
                [0, cos(angle), -sin(angle)],
                [0, sin(angle), cos(angle)]])
        elif axis == [0, 1, 0]:
            rot = np.array([
                [cos(angle), 0, sin(angle)],
                [0, 1, 0],
                [-sin(angle), 0, cos(angle)]
            ])
        transform[:3, :3] = rot
        transform[:3, 3] = translation

        return transform


    def get_current_cube_position(self):
        self.current_cube_pose = self.cube.get_curr_pose()

    def get_next_cube_position(self) -> list:
        # get current (x,y)
        current_x = self.current_cube_pose[0][0]
        current_y = self.current_cube_pose[0][1]
        # get goal (x,y)
        goal_x = self.goal_position[0]
        goal_y = self.goal_position[1]
        # get distance between x1,x2 and y1,y2 and divide by our maximum movement_size to get sample size for linspace
        x_sample_size = round((max(current_x, goal_x) -
                              min(current_x, goal_x)) / self.MAX_MOVE)
        y_sample_size = round((max(current_y, goal_y) -
                              min(current_y, goal_y)) / self.MAX_MOVE)
        # if we are too close to interpolate then set next_cube_x to goal_x otherwise we use linspace
        if x_sample_size <= 1:
            next_cube_x = goal_x
        else:
            next_cube_x = np.linspace(
                current_x, goal_x, x_sample_size, endpoint=False)[1]
        # if we are too close to interpolate then set next_cube_y to goal_y otherwise we use linspace
        if y_sample_size <= 1:
            next_cube_y = goal_y
        else:
            next_cube_y = np.linspace(
                current_y, goal_y, y_sample_size, endpoint=False)[1]
        next_cube_position = [next_cube_x, next_cube_y, 0]
        return next_cube_position

    def get_current_contact_points(self) -> list:
        contact_points = []
        # left distal link
        contact_points_info_left = p.getContactPoints(
            self.cube.id, self.gripper.id, linkIndexB=self.end_effector_links[0])
        if contact_points_info_left:
            contact_points.append(contact_points_info_left[0][6])

        # right distal link
        contact_points_info_right = p.getContactPoints(
            self.cube.id, self.gripper.id, linkIndexB=self.end_effector_links[1])
        if contact_points_info_right:
            contact_points.append(contact_points_info_right[0][6])

        # if either do not have contact we return None
        if len(contact_points) < 2:
            return None
        # world coordinates
        return contact_points

    def retry_contact(self):
        # if no contact attempt to reastablish contact by moving towards cube
        location = self.current_cube_pose[0]
        next_positions = []
        for i in range(len(self.end_effector_links)):
            distal_link = p.getLinkState(
                self.gripper.id, self.end_effector_links[i])
            distal_pos = distal_link[4]
            x_sample_size = round((max(distal_pos[0], location[0]) -
                                   min(distal_pos[0], location[0])) / self.MAX_MOVE)
            y_sample_size = round((max(distal_pos[1], location[1]) -
                                   min(distal_pos[1], location[1])) / self.MAX_MOVE)
            if x_sample_size <= 1:
                next_x = location[0]
            else:
                next_x = np.linspace(
                    distal_pos[0], location[0], x_sample_size, endpoint=False)[1]
            if y_sample_size <= 1:
                next_y = location[1]
            else:
                next_y = np.linspace(
                    distal_pos[1], location[1], y_sample_size, endpoint=False)[1]

            next_positions.append([next_x, next_y, 0])

        goal = p.calculateInverseKinematics2(bodyUniqueId=self.gripper.id,
                                             endEffectorLinkIndices=self.end_effector_links,
                                             targetPositions=next_positions)
        return goal

    def get_next_contact_points(self, current_contact_points: list, next_cube_position: list):
        next_cube_contacts_global = []
        for i in range(len(self.end_effector_links)):
            # get cube pose transform from global to local
            cube_local = p.invertTransform(
                self.current_cube_pose[0], self.current_cube_pose[1])
            # take cube pose and multiply it by current contact points to get them in local frame
            cube_contacts_local = p.multiplyTransforms(
                cube_local[0], cube_local[1], current_contact_points[i], self.current_cube_pose[1])
            # get contact contact points for next cube position using current contact points, returns in global frame
            # ORIENTATION MAY BE IMPORTANT HERE
            next_cube_contacts_global.append(p.multiplyTransforms(next_cube_position, [0, 0, 0, 1],
                                                                  cube_contacts_local[0], cube_contacts_local[1]))
        return next_cube_contacts_global

    def get_next_link_positions(self, current_contact_points: list, next_contact_points: list):
        next_link_positions_global = []
        for i in range(len(self.end_effector_links)):
            # gets contact points into local frame
            contacts_local = p.invertTransform(
                current_contact_points[i], self.current_cube_pose[1])
            # get distal link information
            distal_link = p.getLinkState(
                self.gripper.id, self.end_effector_links[i])
            # get current contact points in relation to distal link
            distal_contacts_local = p.multiplyTransforms(
                contacts_local[0], contacts_local[1], distal_link[4], distal_link[5])
            # get next contact points in global coordinates
            next_link_positions_global.append(p.multiplyTransforms(
                next_contact_points[i][0], next_contact_points[i][1], distal_contacts_local[0], distal_contacts_local[1])[0])



        goal = p.calculateInverseKinematics2(bodyUniqueId=self.gripper.id,
                                             endEffectorLinkIndices=self.end_effector_links,
                                             targetPositions=next_link_positions_global)
        return goal
    
    def gradient_calculations(self, next_contact_points: list):

        for finger in range(len(self.fingers.keys())):
            joint_angles = self.gripper.get_joint_angles(joint_numbers=self.fingers[f'finger{finger}'])
            for link in self.fingers[f'finger{finger}']:
                pass

                


    # def set_goal_position(self, position: list):
    #     # world coordinates
    #     self.goal_position = position

    def move_towards_goal(self):
        self.get_current_cube_position()
        next_cube_position = self.get_next_cube_position()
        current_contact_points = self.get_current_contact_points()

        if current_contact_points:
            next_contact_points = self.get_next_contact_points(
                current_contact_points=current_contact_points, next_cube_position=next_cube_position)
            goal = self.get_next_link_positions(
                current_contact_points=current_contact_points, next_contact_points=next_contact_points)
        else:
            goal = self.retry_contact()

        print(self.gripper.get_joint_angles())
        print(goal)
        p.setJointMotorControlArray(self.gripper.id, jointIndices=self.gripper.get_joint_numbers(),
                                    controlMode=p.POSITION_CONTROL, targetPositions=goal)



class SegmentKinematics():

    def __init__(self, hand_id, joint_id, hand_dict):
        
        joint_info = p.getJointInfo(hand_id, joint_id)
        self.joint_axis = joint_info[13]
        self.joint_location = joint_info[14]
        pass
    
    
    def setup_kinematics(self):
        pass

    def update_joints(self, joints):
        pass

    def predict_location(self, joints):
        pass

    def get_finger_tip_loc(self):
        pass
    
    def rotation_matrix(self, angle=pi, axis=[0,0,1]):
        if axis == [0,0,1]:
            rot = np.array([
                [cos(angle), -sin(angle), 0],
                [sin(angle), cos(angle), 0],
                [0, 0, 1]])
        elif axis == [1, 0, 0]:
            rot = np.array([
                [1, 0, 0],
                [0, cos(angle), -sin(angle)],
                [0, sin(angle), cos(angle)]])
        elif axis == [0, 1, 0]:
            rot = np.array([
                [cos(angle), 0, sin(angle)],
                [0, 1, 0],
                [-sin(angle), 0, cos(angle)]
            ])
        return rot


    def transform_matrix(self, angle=pi, axis=[0,0,1], translation=[0,0,0]):
        transform = np.identity((4,4))
        if axis == [0,0,1]:
            rot = np.array([
                [cos(angle), -sin(angle), 0],
                [sin(angle), cos(angle), 0],
                [0, 0, 1]])
        elif axis == [1, 0, 0]:
            rot = np.array([
                [1, 0, 0],
                [0, cos(angle), -sin(angle)],
                [0, sin(angle), cos(angle)]])
        elif axis == [0, 1, 0]:
            rot = np.array([
                [cos(angle), 0, sin(angle)],
                [0, 1, 0],
                [-sin(angle), 0, cos(angle)]
            ])
        transform[:3, :3] = rot
        transform[3, :3] = translation

        return transform