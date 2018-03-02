#! /usr/bin/env python
"""A test program to test action servers for the JACO and MICO arms."""

import math
import numpy as np
import roslib; roslib.load_manifest('kinova_demo')
import rospy

import tf
from robot_control_modules import *

prefix = 'j2n6s300_'
nbJoints = 6

if __name__ == '__main__':
    try:
        rospy.init_node('test_action_servers')

        initial_position = [0.0, -0.5, 0.4]

        orientation = [math.pi, 0, 0]
        result = gripper_client([0, 0, 0], prefix)

        for i in range(100):
            maybe_move_or_go_home(initial_position, orientation, prefix)
            x = 0.1568450084279111
            y = -0.43456870558539773
            # x = np.random.uniform(-0.3, 0.3)
            # y = np.random.uniform(-0.6, -0.4)
            z = 0.01

            maybe_move_or_go_home([x, y, z], orientation, prefix)

            flag_grasp = False
            for j in range(3000, 6500, 200):
                result = gripper_client([j, j, j], prefix)
                if max(abs(result.fingers.finger1 - j), abs(result.fingers.finger2 - j), abs(result.fingers.finger3 - j)) > 100:
                    flag_grasp = True
                    break

            print(flag_grasp)
            if flag_grasp:
                print(x, y)

            maybe_move_or_go_home(initial_position, orientation, prefix)
            result = gripper_client([0, 0, 0], prefix)

        print("done!")

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
