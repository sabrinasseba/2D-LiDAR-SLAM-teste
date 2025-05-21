#!/usr/bin/env python

# Copyright (c) 2021 taurob GmbH. All rights reserved. Confidential.
# Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com

# This node takes the array from the joint_state_publisher_gui topic containing the joint position and
# publishes the array as individual floats so that the values can be used in the rqt_plot function

import rospy
import std_msgs.msg
import sensor_msgs.msg

class Republisher:

    def __init__(self):
        self._joint_states_sub = rospy.Subscriber("/joint_states", sensor_msgs.msg.JointState, self.callback)
        self._publishers = {
            "arm_joint_1": rospy.Publisher("/joint1_position", std_msgs.msg.Float64, queue_size=1),
            "arm_joint_2": rospy.Publisher("/joint2_position", std_msgs.msg.Float64, queue_size=1),
            "arm_joint_3": rospy.Publisher("/joint3_position", std_msgs.msg.Float64, queue_size=1),
            "arm_joint_4": rospy.Publisher("/joint4_position", std_msgs.msg.Float64, queue_size=1),
            "arm_joint_5": rospy.Publisher("/joint5_position", std_msgs.msg.Float64, queue_size=1),
        }

    def callback(self, msg):

        if len(msg.name) != len(msg.position):
            return

        for i in range(len(msg.name)):
            name = msg.name[i]
            position = msg.position[i]

            if name in self._publishers:
                self._publishers[name].publish(position)

if __name__ == '__main__':
    try:
        rospy.init_node('rqt_plot_data_publisher')
        r = Republisher()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
