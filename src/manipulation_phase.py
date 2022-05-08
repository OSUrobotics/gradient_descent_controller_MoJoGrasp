import pybullet as p
from mojograsp.simcore.phase import Phase
from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
from mojograsp.simobjects.object_base import ObjectBase
import expert_controller
import gradient_controller
from math import isclose


class Manipulation(Phase):

    def __init__(self, hand: TwoFingerGripper, obj: ObjectBase):
        self.name = "close"
        #self.target_pos = [0.75, -1.4, -0.75, 1.4]
        self.target_pos = [7.4, 14, 7.4, 14]
        self.hand = hand
        self.obj = obj
        self.terminal_step = 1000
        self.timestep = 0

        # TESTING
        self.controller = expert_controller.ExpertController(hand, obj)
        # self.controller = gradient_controller.GradientDescentController(hand, obj)

    def setup(self):
        self.timestep = 0

    def execute_action(self):
        self.controller.set_goal_position([-.08, .15, 0])
        self.controller.move_towards_goal()
        self.timestep += 1

    def exit_condition(self) -> bool:
        if self.timestep < self.terminal_step:
            return False
        print(self.obj.get_curr_pose())
        return True

    def next_phase(self) -> str:
        return None
