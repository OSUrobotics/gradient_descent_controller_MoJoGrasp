from mojograsp.simcore.environment import Environment
from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
from mojograsp.simobjects.object_base import ObjectBase
import pybullet as p


class AsteriskEnv(Environment):
    def __init__(self, hand: TwoFingerGripper, obj: ObjectBase):
        self.hand = hand
        self.obj = obj

    def reset(self):
        p.resetSimulation()
        plane_id = p.loadURDF("plane.urdf")
        hand_id = p.loadURDF(self.hand.path, useFixedBase=True,
                             basePosition=[0.0, 0.0, 0.08])
        p.resetJointState(hand_id, 0, 1.57)
        p.resetJointState(hand_id, 2, -1.57)
        obj_id = p.loadURDF(self.obj.path, basePosition=[0.0, 0.16, .05])
        self.hand.id = hand_id
        self.obj.id = obj_id
        p.changeVisualShape(hand_id, -1, rgbaColor=[0.3, 0.3, 0.3, 1])
        p.changeVisualShape(hand_id, 0, rgbaColor=[1, 0.5, 0, 1])
        p.changeVisualShape(hand_id, 1, rgbaColor=[0.3, 0.3, 0.3, 1])
        p.changeVisualShape(hand_id, 2, rgbaColor=[1, 0.5, 0, 1])
        p.changeVisualShape(hand_id, 3, rgbaColor=[0.3, 0.3, 0.3, 1])

    def setup(self):
        super().setup()

    def step(self):
        super().step()
