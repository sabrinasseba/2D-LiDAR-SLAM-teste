#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math

class MovingBox:
    def _init_(self, model_name, pattern, center_x, center_y, distance=3.0, speed=0.03, angular_speed=0.02):
        self.model_name = model_name
        self.pattern = pattern
        self.center_x = center_x
        self.center_y = center_y
        self.distance = distance
        self.speed = speed
        self.angular_speed = angular_speed
        self.theta = 0.0
        self.z_pos = 0.0

        if self.pattern in ['square', 'triangle']:
            self.init_waypoints()
            self.current_target = 0
            self.x_pos, self.y_pos = self.waypoints[self.current_target]
        else:
            raise ValueError(f"Padrão de movimento '{self.pattern}' não suportado.")

    def init_waypoints(self):
        if self.pattern == 'square':
            self.waypoints = [
                (self.center_x + self.distance, self.center_y + self.distance),
                (self.center_x - self.distance, self.center_y + self.distance),
                (self.center_x - self.distance, self.center_y - self.distance),
                (self.center_x + self.distance, self.center_y - self.distance)
            ]
        elif self.pattern == 'triangle':
            angle_offset = math.radians(90)
            self.waypoints = []
            for i in range(3):
                angle = angle_offset + i * (2 * math.pi / 3)
                x = self.center_x + self.distance * math.cos(angle)
                y = self.center_y + self.distance * math.sin(angle)
                self.waypoints.append((x, y))

    def update_position(self):
        target_x, target_y = self.waypoints[self.current_target]
        delta_x = target_x - self.x_pos
        delta_y = target_y - self.y_pos
        dist = (delta_x * 2 + delta_y * 2) ** 0.5
        if dist > 0:
            move_x = (delta_x / dist) * self.speed
            move_y = (delta_y / dist) * self.speed
        else:
            move_x, move_y = 0.0, 0.0

        self.x_pos += move_x
        self.y_pos += move_y

        if abs(self.x_pos - target_x) < 0.05 and abs(self.y_pos - target_y) < 0.05:
            self.current_target = (self.current_target + 1) % len(self.waypoints)

    def get_model_state_msg(self):
        state = ModelState()
        state.model_name = self.model_name
        state.pose.position.x = self.x_pos
        state.pose.position.y = self.y_pos
        state.pose.position.z = self.z_pos
        return state


class MultiBoxController:
    def _init_(self):
        rospy.init_node('multi_box_controller', anonymous=True)
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Apenas caixas com movimento em triângulo e quadrado
        self.boxes = [
            MovingBox('box_triangle', 'triangle', center_x=-10.0, center_y=0.0),
            MovingBox('box_square', 'square', center_x=0.0, center_y=0.0)
        ]

        rospy.loginfo("MultiBoxController is running! Boxes will move in triangle and square patterns.")
        self.run_loop()

    def run_loop(self):
        rate = rospy.Rate(25)
        while not rospy.is_shutdown():
            for box in self.boxes:
                box.update_position()
                state_msg = box.get_model_state_msg()
                try:
                    self.set_model_state(state_msg)
                except rospy.ServiceException as e:
                    rospy.logerr(f"Failed to move {box.model_name}: {e}")
            rate.sleep()


if _name_ == "_main_":
    try:
        MultiBoxController()
    except rospy.ROSInterruptException:
        pass