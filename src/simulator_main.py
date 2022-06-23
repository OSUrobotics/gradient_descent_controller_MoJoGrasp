from distutils.core import setup
from matplotlib.pyplot import close
from mojograsp.simcore import record_data
import pybullet as p
import pybullet_data
import pathlib
import close_hand_phase
import manipulation_phase
import asterisk_env
from mojograsp.simcore.sim_manager import SimManagerDefault
from mojograsp.simcore.state import StateDefault
from mojograsp.simcore.reward import RewardDefault
from mojograsp.simcore.environment import EnvironmentDefault
from mojograsp.simcore.record_data import RecordDataJSON
from modified_mojograsp_classes import UpdatedObjectBase, UpdatedTwoFingerGripper
from helper_functions import Helper as HF
from math import pi




# class simulation_main:
#     def __init__(self, env_setup):
        
    
def asterisk_simulation(env_setup):
    hand_setup = env_setup["hand"]
    obj_setup = env_setup["object"]
    trial_setup = env_setup["trial"]
    # start pybullet
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
                                cameraTargetPosition=[0, 0.1, 0.5])
    
    # load meshes
    plane_id = p.loadURDF("plane.urdf")
    hand_id = HF.load_mesh(hand_setup)
    hand = UpdatedTwoFingerGripper(hand_id, path=hand_setup["path"], setup_parameters=hand_setup)
    obj_id = HF.load_mesh(obj_setup)
    obj = UpdatedObjectBase(id=obj_id, path=obj_setup["path"], setup_parameters=obj_setup)
    # change visual of gripper
    p.changeVisualShape(hand_id, -1, rgbaColor=hand_setup["palm_color"])
    for segment_number in range(len(hand_setup["segment_colors"])):
        p.changeVisualShape(hand_id, segment_number, rgbaColor=hand_setup["segment_colors"][segment_number])
    
    # state and reward
    state = StateDefault(objects=[hand, obj])
    # reward = RewardDefault()

    # data recording
    # record = RecordDataJSON(data_path=data_path, state=state, save_all=True)

    # environment and recording
    env = asterisk_env.AsteriskEnv(hand=hand, obj=obj)

    # sim manager
    manager = SimManagerDefault(num_episodes=trial_setup["episode_number"], env=env)

    close_hand = close_hand_phase.CloseHand(hand, obj=obj)
    manipulation = manipulation_phase.AstriskManipulation(hand, obj=obj)
    manager.add_phase("close", close_hand)
    manager.add_phase("manipulation", manipulation)

    manager.run()
    manager.stall()



if __name__ == '__main__':

    current_path = str(pathlib.Path().resolve())

    env_setup = {"hand": {"path": current_path+"/resources/test_hand/test_hand.urdf",
                        "position": [0.0, 0.0, 0.04],
                        "orientation": p.getQuaternionFromEuler([0, pi/2, pi/2]), # [0, pi/2, pi/2]
                        "scaling": 1.0, #0.25,
                        "fixed": True,
                        "distal_joints":[0,2],
                        "distal_links": [],
                        "starting_joint_angles": [0,0,0,0],
                        "palm_color": [0.3, 0.3, 0.3, 1],
                        "segment_colors":[[1, 0.5, 0, 1], [0.3, 0.3, 0.3, 1], [1, 0.5, 0, 1], [0.3, 0.3, 0.3, 1]]},
                 "object": {"path":current_path + "/resources/object_models/2v2_mod/2v2_mod_cuboid_small.urdf",
                        "position": [0.0, .09, .05],
                        "orientation": [0, 0, 0, 1],
                        "scaling": 1,
                        "fixed": False,
                        "color": [0.3, 0.3, 0.3, 1]},
                 "trial" : {"episode_number" : 2}}

    asterisk_simulation(env_setup= env_setup)


    
# # load objects
# plane_id = p.loadURDF("plane.urdf")
# hand_id = p.loadURDF(hand_path, useFixedBase=True,
#                      basePosition=[0.0, 0.0, 0.04],
#                      baseOrientation=p.getQuaternionFromEuler([0, pi/2, pi/2]), globalScaling=0.25)

# hand = TwoFingerGripper(hand_id, path=hand_path)

# cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.16, .05])
# cube = ObjectBase(cube_id, path=cube_path)




# p.changeVisualShape(hand_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
# p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
# p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
# p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
# p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])
