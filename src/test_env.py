#!/usr/bin/python3


from urdf_to_kinematic_chain import UrdfToKinematicChain
from pprint import pprint



if __name__ == '__main__':

    kc = UrdfToKinematicChain('/home/jcampbell/git_repos/optimize_hand_design/src/resources/test_hand/test_hand.urdf')


    a = kc.calculate_forward_kinematics(joint_angles=[0,3.14, 0, 3.14])

    pprint(a['finger0'])
    pprint(a['finger1'])
