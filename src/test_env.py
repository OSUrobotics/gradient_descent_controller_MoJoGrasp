#!/usr/bin/python3


import pybullet as p
import pybullet_data

from urdf_to_kinematic_chain import UrdfToKinematicChain
from pprint import pprint
import pathlib
import os 
from math import pi
from helper_functions import Helper as HF
from modified_mojograsp_classes import UpdatedTwoFingerGripper, UpdatedObjectBase
from hand_controller import HandController

if __name__ == '__main__':

    current_path = pathlib.Path(__file__).parent.resolve()


    # print(pathlib.Path().resolve())
    # print(current_path / "resources/test_hand/test_hand.urdf")
    # os.chdir(current_path / "resources/test_hand/")
    # print(pathlib.Path().resolve())
    
    
    env_setup = {"hand": {"path": str(current_path / "resources/test_hand/test_hand.urdf"),
                        "position": [0.0, 0.0, 0.04],
                        "orientation": p.getQuaternionFromEuler([0, pi/2, pi/2]), # [0, pi/2, pi/2]
                        "scaling": 1.0, #0.25,
                        "fixed": True,
                        "starting_joint_angles": [0,0,0,0],
                        "palm_color": [0.3, 0.3, 0.3, 1],
                        "segment_colors":[[1, 0.5, 0, 1], [0.3, 0.3, 0.3, 1], [1, 0.5, 0, 1], [0.3, 0.3, 0.3, 1]]},
                 "object": {"path": str(current_path /  "resources/object_models/2v2_mod/2v2_mod_cuboid_small.urdf"),
                        "position": [0.0, .09, .05],
                        "orientation": [0, 0, 0, 1],
                        "scaling": 1,
                        "fixed": False,
                        "color": [0.3, 0.3, 0.3, 1]},
                 "trial" : {"episode_number" : 2}}
    
    hand_setup = env_setup['hand']
    obj_setup = env_setup['object']
    
    # physics_client = p.connect(p.GUI)
    physics_client = p.connect(p.DIRECT)
    # p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
                                cameraTargetPosition=[0, 0.1, 0.5])

    hand_id = HF.load_mesh(hand_setup)
    

    hand = UpdatedTwoFingerGripper(hand_id, path=hand_setup["path"], setup_parameters=hand_setup, name=hand_setup['path'].split('/')[-1].split('.')[0])
    obj_id = HF.load_mesh(obj_setup)
    obj = UpdatedObjectBase(id=obj_id, path=obj_setup["path"], setup_parameters=obj_setup)

    # for finger in hand.fingers.keys():
    #     print(hand.fingers[finger])


    a = obj.get_curr_pose()
    print(a)


    # controller = HandController(hand, obj)
    
    # print(controller.end_effector_links)
    # num_joints = p.getNumJoints(hand)
    



    # print("\n\n\n")
    # for i in range(num_joints):
    #     print(p.getJointInfo(hand, i)[12].decode('UTF-8'))

    # kc = UrdfToKinematicChain('/home/jcampbell/git_repos/optimize_hand_design/src/resources/test_hand/test_hand.urdf')


    # a = kc.calculate_forward_kinematics(joint_angles=[0,3.14, 0, 3.14])

    # pprint(a['finger0'])
    # pprint(a['finger1'])
