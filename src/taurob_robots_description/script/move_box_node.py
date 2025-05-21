#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SetModelState
from gazebo_msgs.msg import ModelState

class MoveBox:
    def __init__(self):
        rospy.init_node('move_box_node', anonymous=True)

        # Aguarda o serviço do Gazebo estar disponível
        rospy.wait_for_service('/gazebo/set_model_state')
        self.set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)

        # Definição do centro e tamanho do quadrado
        self.center_x = 0.0   # Centro do movimento no eixo X
        self.center_y = 0.0   # Centro do movimento no eixo Y
        self.distance = 4.0   # Distância do centro até os vértices do quadrado
        self.speed = 0.02     # Velocidade do movimento

        # Definir a trajetória do quadrado (4 vértices)
        self.waypoints = [
            (self.center_x + self.distance, self.center_y + self.distance),  # Canto superior direito
            (self.center_x - self.distance, self.center_y + self.distance),  # Canto superior esquerdo
            (self.center_x - self.distance, self.center_y - self.distance),  # Canto inferior esquerdo
            (self.center_x + self.distance, self.center_y - self.distance)   # Canto inferior direito
        ]
        
        self.current_target = 0  # Índice do waypoint atual
        self.x_pos, self.y_pos = self.waypoints[self.current_target]  # Começa no primeiro ponto
        
        # Define a posição fixa no solo
        self.z_pos = 0.0  # Mantém a caixa no chão

        rospy.loginfo("MoveBox Node is running! The box will move in a square around the center.")

        self.move_box_loop()

    def move_box_loop(self):
        rate = rospy.Rate(25)  # Taxa de atualização

        while not rospy.is_shutdown():
            # Obtém o próximo ponto alvo (waypoint)
            target_x, target_y = self.waypoints[self.current_target]

            # Calcula os deslocamentos necessários
            delta_x = target_x - self.x_pos
            delta_y = target_y - self.y_pos

            # Normaliza os deslocamentos para manter a velocidade constante
            dist = (delta_x ** 2 + delta_y ** 2) ** 0.5
            if dist > 0:
                move_x = (delta_x / dist) * self.speed
                move_y = (delta_y / dist) * self.speed
            else:
                move_x, move_y = 0.0, 0.0

            # Atualiza a posição
            self.x_pos += move_x
            self.y_pos += move_y

            # Verifica se chegou no waypoint
            if abs(self.x_pos - target_x) < 0.05 and abs(self.y_pos - target_y) < 0.05:
                self.current_target = (self.current_target + 1) % len(self.waypoints)  # Avança para o próximo ponto

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
