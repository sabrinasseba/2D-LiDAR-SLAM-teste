#!/usr/bin/env python3

import rospy
from gazebo_msgs.msg import ModelStates
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class DynamicObjectTracker:
    def __init__(self):
        rospy.init_node("gazebo_object_tracker")

        self.prev_positions = {}
        self.target_models = ["moving_box"]  # name(s) of the object(s) in Gazebo to track
        self.marker_pub = rospy.Publisher("/object_markers", Marker, queue_size=1)

        rospy.Subscriber("/gazebo/model_states", ModelStates, self.callback)
        rospy.loginfo("Tracking object positions in Gazebo...")
        rospy.spin()

    def callback(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.target_models:
                position = msg.pose[i].position

                # Detect movement
                moved = False
                if name in self.prev_positions:
                    prev = self.prev_positions[name]
                    dx = abs(prev.x - position.x)
                    dy = abs(prev.y - position.y)
                    dz = abs(prev.z - position.z)
                    moved = dx > 0.01 or dy > 0.01 or dz > 0.01

                self.prev_positions[name] = position

                self.publish_marker(name, position, moved)

    def publish_marker(self, name, pos, moved):
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "tracked_objects"
        marker.id = hash(name) % 1000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position = pos
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.a = 1.0
        marker.color.r = 1.0 if moved else 0.0
        marker.color.g = 0.0 if moved else 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)

if __name__ == "__main__":
    DynamicObjectTracker()
