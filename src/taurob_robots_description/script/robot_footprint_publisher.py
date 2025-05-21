#!/usr/bin/env python

# Copyright (c) 2023 taurob GmbH. All rights reserved. Confidential.
# Perfektastrasse 57/7, 1230 Wien, Austria. office@taurob.com

# Script to publish robot global and local footprint for visualization in rviz

from geometry_msgs.msg import PolygonStamped, Point32
import rospy
import json

NODE_NAME = "robot_footprint_publisher"

class FootprintPublisher(object):
    def __init__(self):
        self.footprint_pub_global = rospy.Publisher("robot_footprint/global", PolygonStamped, queue_size=1)
        self.footprint_pub_local = rospy.Publisher("robot_footprint/local", PolygonStamped, queue_size=1)

        #read parameters from parameter server
        self.footprint_global = json.loads(rospy.get_param(NODE_NAME + "/footprint_global/footprint"))
        self.footprint_local  = json.loads(rospy.get_param(NODE_NAME + "/footprint_local/footprint"))

    def _publish_global(self):
        msg = PolygonStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"
        
        msg.polygon.points = [Point32(point[0], point[1], 0.0) for point in self.footprint_global]

        self.footprint_pub_global.publish(msg)
        
    def _publish_local(self):
        msg = PolygonStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        msg.polygon.points = [Point32(point[0], point[1], 0.0) for point in self.footprint_local]

        self.footprint_pub_local.publish(msg)
    
    def publish(self):
        self._publish_global()
        self._publish_local()

if __name__ == "__main__":
    try:
        rospy.init_node(NODE_NAME)
        footprint_pub = FootprintPublisher()
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            footprint_pub.publish()
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
