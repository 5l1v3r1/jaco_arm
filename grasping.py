#! /usr/bin/env python
"""A test program to test action servers for the JACO and MICO arms."""

import numpy as np
import roslib; roslib.load_manifest('kinova_demo')
import rospy

from robot_control_modules import *

prefix = 'j2n6s300_'
nbJoints = 6

def get_distance(vec, position):
    return np.sqrt(pow(vec[0] - position.x, 2) +
                   pow(vec[1] - position.y, 2) +
                   pow(vec[2] - position.z, 2))

if __name__ == '__main__':
    try:
        rospy.init_node('test_action_servers')


        initial_position = [0.0, -0.5, 0.4]
        pi = 3.1415
        a0 = pi # np.random.uniform(-pi, pi)
        a1 = 0 # np.random.uniform(-pi, pi)
        a2 = 0 # np.random.uniform(-pi, pi)

        result = gripper_client([0, 0, 0], prefix)

        for i in range(1000):
            #set cartesian pose 1
            quaternion = tf.transformations.quaternion_from_euler(a0, a1, a2,'rxyz')
            result = cartesian_pose_client(initial_position, quaternion, prefix)

            x = np.random.uniform(-0.3, 0.3)
            y = np.random.uniform(-0.6, -0.4)
            z = 0.01

            result = cartesian_pose_client([x, y, z], quaternion, prefix)
            print(result.pose.pose)

            new_pos = result.pose.pose.position

            dist = get_distance([x, y, z], new_pos)
            if dist > 2e-2:
                homeRobot(prefix)
            else:
                result = gripper_client([4100, 4100, 4100], prefix)

                print(dist)

                #n = raw_input()

                result = cartesian_pose_client(initial_position, quaternion, prefix)

                result = gripper_client([0, 0, 0], prefix)

        print("done!")

    except rospy.ROSInterruptException:
        print("program interrupted before completion")
