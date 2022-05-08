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
from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
from mojograsp.simobjects.object_base import ObjectBase
from math import pi
# resource paths
current_path = str(pathlib.Path().resolve())
# hand_path = current_path+"/resources/2v2_nosensors/2v2_nosensors.urdf"
hand_path = current_path+"/resources/test_hand/test_hand.urdf" # josh's rescent one
#hand_path = current_path+"/resources/hand_models/2v2_mod/2v2_mod.urdf"
cube_path = current_path + \
    "/resources/object_models/2v2_mod/2v2_mod_cuboid_small.urdf"
# cube_path = current_path + \
# "/resources/2v2_nosensors_objects/2v2_nosensors_cuboid_small.urdf"
data_path = current_path+"/data/"

# start pybullet
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
p.resetDebugVisualizerCamera(cameraDistance=.02, cameraYaw=0, cameraPitch=-89.9999,
                             cameraTargetPosition=[0, 0.1, 0.5])

# load objects
plane_id = p.loadURDF("plane.urdf")
hand_id = p.loadURDF(hand_path, useFixedBase=True,
                     basePosition=[0.0, 0.0, 0.04],
                     baseOrientation=p.getQuaternionFromEuler([0, pi/2, pi/2]))

hand = TwoFingerGripper(hand_id, path=hand_path)
p.resetJointState(hand_id, 0, 1.57)
p.resetJointState(hand_id, 2, 1.57)
cube_id = p.loadURDF(cube_path, basePosition=[0.0, 0.16, .05])
cube = ObjectBase(cube_id, path=cube_path)


# change visual of gripper
p.changeVisualShape(hand_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])


# state and reward
state = StateDefault(objects=[hand, cube])
reward = RewardDefault()

# data recording
record = RecordDataJSON(data_path=data_path, state=state, save_all=True)

# environment and recording
env = asterisk_env.AsteriskEnv(hand=hand, obj=cube)

# sim manager
manager = SimManagerDefault(num_episodes=1, env=env, record=record)

close_hand = close_hand_phase.CloseHand(hand, cube)
manipulation = manipulation_phase.Manipulation(hand, cube)
manager.add_phase("close", close_hand)
# manager.add_phase("manipulation", manipulation)

manager.run()
manager.stall()
