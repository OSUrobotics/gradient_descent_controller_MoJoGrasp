from mojograsp.simcore.environment import Environment
# from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
# from mojograsp.simobjects.object_base import ObjectBase
from modified_mojograsp_classes import UpdatedObjectBase, UpdatedTwoFingerGripper
import pybullet as p
from helper_functions import Helper as HF

class AsteriskEnv(Environment):
    def __init__(self, hand: UpdatedTwoFingerGripper, obj: UpdatedObjectBase):
        self.hand = hand
        self.obj = obj

    def reset(self):

        hand_setup = self.hand.setup_param
        obj_setup = self.obj.setup_param
        p.resetSimulation()
        p.setGravity(0, 0, -10)

        plane_id = p.loadURDF("plane.urdf")
        hand_id = HF.load_mesh(hand_setup)
        self.hand.id = hand_id

        obj_id = HF.load_mesh(obj_setup)
        self.obj.id = obj_id
        # change visual of gripper
        p.changeVisualShape(hand_id, -1, rgbaColor=hand_setup["palm_color"])
        for segment_number in range(len(hand_setup["segment_colors"])):                
            p.changeVisualShape(hand_id, segment_number, rgbaColor=hand_setup["segment_colors"][segment_number])
            p.resetJointState(hand_id, segment_number, hand_setup["starting_joint_angles"][segment_number])


    def setup(self):
        super().setup()

    def step(self):
        super().step()
