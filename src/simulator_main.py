#!/usr/bin/python3
from distutils.core import setup
from matplotlib.pyplot import close
from mojograsp.simcore import record_data
import manipulation_phase
import asterisk_env
from demos.expert_demo import expert_action
from demos.expert_demo import expert_reward
from mojograsp.simcore.sim_manager import SimManagerDefault
from mojograsp.simcore.state import StateDefault
from mojograsp.simcore.reward import RewardDefault
from mojograsp.simcore.environment import EnvironmentDefault
from mojograsp.simcore.record_data import RecordDataJSON
from modified_mojograsp_classes import UpdatedObjectBase, UpdatedTwoFingerGripper
import helper_functions as HF
from numpy import pi
import pybullet as p
import pybullet_data
import pathlib
import time



logger = HF.colored_logging("simulator_main")
    
def asterisk_simulation(env_setup, gui):
    """Initiate and run mojograsp trials.

    Args:
        env_setup (dict): Dictionary containing three setup dictionaries with the keys of 'hand', 'object', and 'trial' see bottom of script or src/README.md for an example.
    """
    # breaking up setup parameters
    hand_setup = env_setup["hand"]
    obj_setup = env_setup["object"]
    trial_setup = env_setup["trial"]
    
    # start pybullet
    if gui == True:
        physics_client = p.connect(p.GUI)
    elif gui == False:
        physics_client = p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -10)
    
    # gui view port
    p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
                                cameraTargetPosition=[0, 0.05, 0.5])
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
    # load meshes
    plane_id = p.loadURDF("plane.urdf")
    hand_id = HF.load_mesh(hand_setup)
    hand = UpdatedTwoFingerGripper(hand_id, path=hand_setup["path"], setup_parameters=hand_setup)
    obj_id = HF.load_mesh(obj_setup)
    obj = UpdatedObjectBase(id=obj_id, path=obj_setup["path"], setup_parameters=obj_setup)
    
    # change visual of gripper and set initial joint angles
    p.changeVisualShape(hand_id, -1, rgbaColor=hand_setup["palm_color"])
    for segment_number in range(len(hand_setup["segment_colors"])):
        p.changeVisualShape(hand_id, segment_number, rgbaColor=hand_setup["segment_colors"][segment_number])
        p.resetJointState(hand_id, segment_number, hand_setup["starting_joint_angles"][segment_number])
    
    # state and reward
    state = StateDefault(objects=[hand, obj])
    action = expert_action.ExpertAction()
    reward = expert_reward.ExpertReward()

    # data recording
    record = RecordDataJSON(data_path=trial_setup["data_path"], state=state, action=action, reward=reward, save_all=True)

    # environment and recording
    env = asterisk_env.AsteriskEnv(hand=hand, obj=obj)

    # sim manager
    manager = SimManagerDefault(num_episodes=len(trial_setup["goal_locations"]["y"]), env=env, record_data=record)

    # close_hand = close_hand_phase.CloseHand(hand, obj=obj)
    manipulation = manipulation_phase.AstriskManipulation(hand, obj=obj, 
        x_goals = trial_setup["goal_locations"]["x"], y_goals=trial_setup["goal_locations"]["y"],
        state=state, action=action, reward=reward)

    manager.add_phase("manipulation", manipulation, start=True)

    # input('Press "Enter" to Start Trial')
    manager.run()
    # manager.stall()



if __name__ == '__main__':

    current_path = str(pathlib.Path(__file__).parent.resolve())
    
    # example of enviroment setup directory.
    env_setup = {"hand": {"path": current_path+"/resources/2v2_Demo/hand/2v2_Demo.urdf",
                        "position": [0.0, 0.0, 0.03],
                        "orientation": p.getQuaternionFromEuler([0, 0, 0]), # [0, pi/2, pi/2]
                        "scaling": 1.0, #0.25,
                        "fixed": True,
                        "starting_joint_angles": [-.695, 1.487, 0.695, -1.487],
                        "palm_color": [0.3, 0.3, 0.3, 1],
                        "segment_colors":[[1, 0.5, 0, 1], [0.3, 0.3, 0.3, 1], [1, 0.5, 0, 1], [0.3, 0.3, 0.3, 1]]},
                 "object": {"path":current_path + "/resources/2v2_Demo/object/2v2_Demo_cuboid_small.urdf",
                        "position": [0.0, .1067, .05],
                        "orientation": [0, 0, 0, 1],
                        "scaling": 1,
                        "fixed": False,
                        "color": [0.3, 0.3, 0.3, 1]},
                 "trial" : {"data_path" : current_path + '/data/',
                            "goal_locations" : {
                                "x" : [0,      0.1,    0.1,    0.1,      0,      -0.1,      -0.1,   -0.1],  # full astrisk test
                                "y" : [0.2067, 0.2067, 0.1067, 0.0067, 0.0067,   0.0067,   0.1067,  0.2067]
                            }}}


    # start_time = time.time()
    # asterisk_simulation(env_setup= env_setup, gui=True)
    # gui_time = time.time() - start_time
    # print(f"\n\nTime for a gui run: {gui_time}")
    start_time2 = time.time()
    asterisk_simulation(env_setup= env_setup, gui=False)
    nongui_time = time.time() - start_time2
    # \nTime for a gui run: {gui_time}
    print(f"\n\nTime for a non-gui run: {nongui_time}")