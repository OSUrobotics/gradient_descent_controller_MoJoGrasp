from mojograsp.simobjects.two_finger_gripper import TwoFingerGripper
from mojograsp.simobjects.object_base import ObjectBase



class UpdatedTwoFingerGripper(TwoFingerGripper):
    def __init__(self, id: int = None, path: str = None, name: str = "two_finger_gripper", setup_parameters = None) -> None:
        super().__init__(id = id, path = path, name = name)
        self.setup_param = setup_parameters

class UpdatedObjectBase(ObjectBase):
    def __init__(self, id: int = None, path: str = None, name: str = None, setup_parameters = None) -> None:
        super().__init__(id = id, path = path, name = name)
        self.setup_param = setup_parameters
