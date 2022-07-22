
from mojograsp.simcore import episode
import pybullet as p
from mojograsp.simcore.phase import Phase

from modified_mojograsp_classes import UpdatedObjectBase, UpdatedTwoFingerGripper

from mojograsp.simcore.state import State
from mojograsp.simcore.reward import Reward
from mojograsp.simcore.action import Action
import hand_controller
from math import isclose
import helper_functions as HF

logger = HF.colored_logging(name="manipulation_phase")

class AstriskManipulation(Phase):

    def __init__(self, hand: UpdatedTwoFingerGripper, obj: UpdatedObjectBase, x_goals, y_goals, state: State, action:Action, reward:Reward):
        self.name = "manipulation"
        self.hand = hand
        self.obj = obj
        
        self.state = state
        self.action = action
        self.reward = reward
        self.terminal_step = 400
        self.timestep = 0
        self.episode = 0
        self.x_goals = x_goals
        self.y_goals = y_goals
        self.goal_position = None

    def setup(self):
        self.controller = hand_controller.HandController(self.hand, self.obj)
        self.timestep = 0
        self.controller.num_contact_loss = 0
        logger.info(f'episode #: {self.episode}\nCube Goal Position:\nx: {self.x_goals[self.episode]}\ny: {self.y_goals[self.episode]}')
        self.goal_position = [float(self.x_goals[self.episode]), 
            float(self.y_goals[self.episode]), 0]
        self.controller.set_goal_position(self.goal_position)
    
    def pre_step(self):

        int_val = self.controller.get_next_action()
        if int_val == False:
            pass
        else:
            self.target = int_val
        
        self.action.set_action(self.target)
        self.state.set_state()
    
    def execute_action(self):
        p.setJointMotorControlArray(self.hand.id, jointIndices=self.hand.get_joint_numbers(),
                                    controlMode=p.POSITION_CONTROL, targetPositions=self.target)
        self.timestep += 1
        
    def post_step(self):
        if self.reward == None:
            pass
        else:
            self.reward.set_reward(self.goal_position, self.obj)

    def exit_condition(self) -> bool:
        if self.timestep > self.terminal_step:
            logger.warning("Time step limit reached\n\n")
            return True
        elif self.controller.exit_condition():
            logger.warning("Controller exit condition\n\n")
            return True
        return False
    
    def next_phase(self) -> str:
        self.episode += 1
        return None

    
