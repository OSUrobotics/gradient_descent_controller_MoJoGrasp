#!/usr/bin/python3

from ntpath import join
import pybullet as p

from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
from mojograsp.simobjects.object_base import ObjectBase
from urdf_to_kinematic_chain import UrdfToKinematicChain



class UpdatedTwoFingerGripper(TwoFingerGripper):
    def __init__(self, id: int = None, path: str = None, name: str = "two_finger_gripper", setup_parameters = None) -> None:
        super().__init__(id = id, path = path, name = name)
        self.setup_param = setup_parameters

        self.fingers = self.setup_fingers()
        self.kinematics = UrdfToKinematicChain(urdf_file=path)
        self.kinematics.update_joint_angles(setup_parameters["starting_joint_angles"])

    def setup_fingers(self):
        # joint_finger = {}
        fingers_dict = {}
        for i in range(p.getNumJoints(self.id)):

            joint_info = p.getJointInfo(self.id, i)
            # print(joint_info)

            finger_num, _ = joint_info[12].decode('UTF-8').split('_')
            if joint_info[-1] == -1:  # * Currently gradient controller is expecting the first to be for the first joint/link not the palm which is index -1
                fingers_dict[finger_num] = {"index_values" : [joint_info[0]], "joint_names": [joint_info[1].decode('UTF-8')], "link_names":[joint_info[12].decode('UTF-8')]}
            else:
                fingers_dict[finger_num]["index_values"].append(joint_info[0])
                fingers_dict[finger_num]["link_names"].append(joint_info[12].decode('UTF-8'))
                fingers_dict[finger_num]["joint_names"].append(joint_info[1].decode('UTF-8'))
        
        return fingers_dict


class UpdatedObjectBase(ObjectBase):
    def __init__(self, id: int = None, path: str = None, name: str = None, setup_parameters = None) -> None:
        super().__init__(id = id, path = path, name = name)
        self.setup_param = setup_parameters
