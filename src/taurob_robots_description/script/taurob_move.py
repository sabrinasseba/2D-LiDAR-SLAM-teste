#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def girar_continuamente(pub, vel_angular):
    movimento = Twist()
    movimento.angular.z = vel_angular
    rate = rospy.Rate(15)  # 10 Hz

    while not rospy.is_shutdown():
        pub.publish(movimento)
        rate.sleep()

if _name_ == '_main_':
    rospy.init_node('giro_continuo_taurob')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.sleep(2)  # Aguarda o publisher se conectar

    vel_angular = 0.05  # rad/s (ajuste conforme desejar)
    girar_continuamente(pub, vel_angular)