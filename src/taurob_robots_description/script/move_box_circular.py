#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState
import math

class MoveBox:
    def __init__(self):
        rospy.init_node('move_box_node', anonymous=True)

        # Aguarda o serviço do Gazebo estar disponível
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Parâmetros do movimento circular
        self.center_x = 0.0   # Centro do círculo no eixo X
        self.center_y = 0.0   # Centro do círculo no eixo Y
        self.radius = 4.0     # Raio do movimento circular
        self.angular_speed = 0.02  # Velocidade angular (radianos por ciclo)
        self.theta = 0.0      # Ângulo inicial

        # Define a posição fixa no solo
        self.z_pos = 0.0  # Mantém a caixa no chão

        rospy.loginfo("MoveBox Node is running! The box will move in a circle around the center.")

        self.move_box_loop()

    def move_box_loop(self):
        rate = rospy.Rate(15)  # Taxa de atualização (15 Hz)

        while not rospy.is_shutdown():
            # Calcula a nova posição com base no ângulo
            self.x_pos = self.center_x + self.radius * math.cos(self.theta)
            self.y_pos = self.center_y + self.radius * math.sin(self.theta)
            self.theta += self.angular_speed

            # Define a nova posição da caixa no Gazebo
            box_state = ModelState()
            box_state.model_name = "moving_box"
            box_state.pose.position.x = self.x_pos
            box_state.pose.position.y = self.y_pos
            box_state.pose.position.z = self.z_pos  # Mantém a caixa no solo

            try:
                self.set_model_state(box_state)
                rospy.loginfo("Box moved to: X={:.2f}, Y={:.2f}, Z={:.2f}".format(self.x_pos, self.y_pos, self.z_pos))
            except rospy.ServiceException as e:
                rospy.logerr("Failed to move box: {}".format(e))

            rate.sleep()  # Aguarda até o próximo ciclo

if __name__ == "__main__":
    try:
        MoveBox()
    except rospy.ROSInterruptException:
        pass
