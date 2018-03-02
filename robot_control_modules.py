#! /usr/bin/env python
"""A set of example functions that can be used to control the arm"""
import rospy
import actionlib

import tf
import kinova_msgs.msg
import geometry_msgs.msg
import std_msgs.msg
from kinova_msgs.srv import *
import numpy as np

def cartesian_pose_client(position, orientation, prefix):
    """Send a cartesian goal to the action server."""
    action_address = '/' + prefix + 'driver/pose_action/tool_pose'
    client = actionlib.SimpleActionClient(action_address, kinova_msgs.msg.ArmPoseAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.ArmPoseGoal()
    goal.pose.header = std_msgs.msg.Header(frame_id=(prefix + 'link_base'))
    goal.pose.pose.position = geometry_msgs.msg.Point(
        x=position[0], y=position[1], z=position[2])
    goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        x=orientation[0], y=orientation[1], z=orientation[2], w=orientation[3])

    client.send_goal(goal)

    if client.wait_for_result(rospy.Duration(200.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        print('        the cartesian action timed-out')
        return None


def get_distance(vec, position):
    return np.sqrt(pow(vec[0] - position.x, 2) +
                   pow(vec[1] - position.y, 2) +
                   pow(vec[2] - position.z, 2))


def maybe_move_or_go_home(position, orientation, prefix, threshold=5e-2):
    quaternion = tf.transformations.quaternion_from_euler(orientation[0], orientation[1], orientation[2], 'rxyz')
    result = cartesian_pose_client(position, quaternion, prefix)

    new_position = result.pose.pose.position
    dist = get_distance(position, new_position)

    if dist > threshold:
        # homeRobot(prefix)
        return True
    else:
        return True


def gripper_client(finger_positions, prefix):
    """Send a gripper goal to the action server."""
    action_address = '/' + prefix + 'driver/fingers_action/finger_positions'

    client = actionlib.SimpleActionClient(action_address,
                                          kinova_msgs.msg.SetFingersPositionAction)
    client.wait_for_server()

    goal = kinova_msgs.msg.SetFingersPositionGoal()
    goal.fingers.finger1 = float(finger_positions[0])
    goal.fingers.finger2 = float(finger_positions[1])
    goal.fingers.finger3 = float(finger_positions[2])
    client.send_goal(goal)
    if client.wait_for_result(rospy.Duration(50.0)):
        return client.get_result()
    else:
        client.cancel_all_goals()
        rospy.WARN('        the gripper action timed-out')
        return None


def homeRobot(prefix):
	service_address = '/' + prefix + 'driver/in/home_arm'
	rospy.wait_for_service(service_address)
        try:
           home = rospy.ServiceProxy(service_address, HomeArm)
           home()
           return None
        except rospy.ServiceException, e:
           print("Service call failed: %s" % e)
