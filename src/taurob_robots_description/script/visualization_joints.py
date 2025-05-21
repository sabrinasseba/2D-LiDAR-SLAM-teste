#!/usr/bin/env python

# Copyright (c) 2021 taurob GmbH. All rights reserved. Confidential.
# Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com

# This node calculates the "visualization joints" of the OPERATOR robot model
# and publishes them.

import math

import rospy
from sensor_msgs.msg import JointState

class OPERATORVisualizationJointPublisher:

    RIGHT_JOINT_NAMES = (
        "flipper_right_joint",
        "flipper_right_front_visualization_middle_joint",
        "flipper_right_front_visualization_front_joint"
    )

    LEFT_JOINT_NAMES = (
        "flipper_left_joint",
        "flipper_left_front_visualization_middle_joint",
        "flipper_left_front_visualization_front_joint"
    )

    # The distance between the flipper axle and the middle wheel.
    MIDDLE_WHEEL_DISTANCE = 0.176

    # The distance between the flipper axle and the front wheel.
    FRONT_WHEEL_DISTANCE = 0.49

    def __init__(self):

        self._joint_states_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)
        self._joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_states_callback)


    def joint_states_callback(self, msg):

        if len(msg.name) != len(msg.position):
            return

        new_msg = JointState()
        new_msg.header.stamp = msg.header.stamp

        for i in range(len(msg.name)):
            if msg.name[i] == OPERATORVisualizationJointPublisher.RIGHT_JOINT_NAMES[0]:
                new_msg.position.extend(self.calculate_joint_positions(msg.position[i]))
                new_msg.name.extend(OPERATORVisualizationJointPublisher.RIGHT_JOINT_NAMES[1:])
            elif msg.name[i] == OPERATORVisualizationJointPublisher.LEFT_JOINT_NAMES[0]:
                new_msg.position.extend(self.calculate_joint_positions(msg.position[i]))
                new_msg.name.extend(OPERATORVisualizationJointPublisher.LEFT_JOINT_NAMES[1:])

        if len(new_msg.name):
            self._joint_states_pub.publish(new_msg)


    def calculate_joint_positions(self, flipper_position):
        """Calculate the visualization joints for a given flipper joint position.

        Returns a tuple (middle, front) joint position.
        """

        a = OPERATORVisualizationJointPublisher.FRONT_WHEEL_DISTANCE
        b = OPERATORVisualizationJointPublisher.MIDDLE_WHEEL_DISTANCE

        gamma = flipper_position

        # The distance between the front and the middle wheel.
        c = math.sqrt(a**2 + b**2 - 2 * a * b * math.cos(gamma))
        alpha = math.acos((b**2 + c**2 - a**2) / (2 * b * c))
        beta = math.acos((a**2 + c**2 - b**2) / (2 * a * c))

        if gamma > 0:
            return math.pi - alpha, beta
        else:
            return -(math.pi - alpha), -beta


if __name__ == "__main__":
    rospy.init_node("visualization_joints")

    publisher = OPERATORVisualizationJointPublisher()

    rospy.spin()
