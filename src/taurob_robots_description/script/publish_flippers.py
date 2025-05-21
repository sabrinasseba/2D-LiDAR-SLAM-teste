#!/usr/bin/env python

# Copyright (c) 2021 taurob GmbH. All rights reserved. Confidential.
# Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com

# Demo script that moves the flippers of the OPERATOR robot up and down.

import math

import rospy
from sensor_msgs.msg import JointState


UPPER_LIMIT = 69 * math.pi / 180.0
LOWER_LIMIT = -30 * math.pi / 180.0


if __name__ == "__main__":

    left_position = right_position = LOWER_LIMIT + (UPPER_LIMIT - LOWER_LIMIT) / 2.0
    right_step = -0.01
    left_step = 0.01

    rospy.init_node("flipper_joint_publisher")

    joint_states_pub = rospy.Publisher("/joint_states", JointState, queue_size=1)

    r = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.name.extend(["flipper_right_joint", "flipper_left_joint"])
        msg.position.extend([right_position, left_position])

        joint_states_pub.publish(msg)

        right_position += right_step
        if right_position > UPPER_LIMIT:
            right_position = UPPER_LIMIT
            right_step = -right_step
        elif right_position < LOWER_LIMIT:
            right_position = LOWER_LIMIT
            right_step = -right_step

        left_position += left_step
        if left_position > UPPER_LIMIT:
            left_position = UPPER_LIMIT
            left_step = -left_step
        elif left_position < LOWER_LIMIT:
            left_position = LOWER_LIMIT
            left_step = -left_step

        r.sleep()
