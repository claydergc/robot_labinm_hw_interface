#!/usr/bin/env python3

import rospy
import time
from odrive_ros_control.msg import od_dual_control

if __name__ == '__main__':

    rospy.init_node('test_parada_emergencia')
    
    
    
    pub_axis = rospy.Publisher('odrive_dual_control', od_dual_control, queue_size=10)
    
    msg_dual_control = od_dual_control();
    #msg_dual_control.stamp=rospy.Time.now()
    msg_dual_control.Odrive1.Parada_emergencia = False    
    msg_dual_control.Odrive2.Parada_emergencia = False
    msg_dual_control.Odrive1.Vel_ax0 = 360*3;
    msg_dual_control.Odrive1.Vel_ax1 = 360*3;
    msg_dual_control.Odrive2.Vel_ax0 = 360*3;
    msg_dual_control.Odrive2.Vel_ax1 = 360*3;
    pub_axis.publish(msg_dual_control)
    print("velocidad de 3 RPS")
    time.sleep(1.0)

    msg_dual_control.Odrive1.Parada_emergencia = False    
    msg_dual_control.Odrive2.Parada_emergencia = False
    msg_dual_control.Odrive1.Vel_ax0 = 360*3;
    msg_dual_control.Odrive1.Vel_ax1 = 360*3;
    msg_dual_control.Odrive2.Vel_ax0 = 360*3;
    msg_dual_control.Odrive2.Vel_ax1 = 360*3;
    pub_axis.publish(msg_dual_control)
    print("velocidad de 3 RPS")
    time.sleep(5.0)
    
    #msg_dual_control.stamp=rospy.Time.now()
    msg_dual_control.Odrive1.Vel_ax0 = 360*-5;
    msg_dual_control.Odrive1.Vel_ax1 = 360*-5;
    msg_dual_control.Odrive2.Vel_ax0 = 360*-5;
    msg_dual_control.Odrive2.Vel_ax1 = 360*-5;
    msg_dual_control.Odrive1.Parada_emergencia = False    
    msg_dual_control.Odrive2.Parada_emergencia = False
    #msg_dual_control.Odrive1.Parada_emergencia = True
    #msg_dual_control.Odrive2.Parada_emergencia = True
    pub_axis.publish(msg_dual_control)
    print("velocidad de -5 RPS")
    time.sleep(2.0)

    msg_dual_control.Odrive1.Parada_emergencia = True    
    msg_dual_control.Odrive2.Parada_emergencia = True
    pub_axis.publish(msg_dual_control)
    print("Parada de emergencia activa")
    time.sleep(5.0)

    msg_dual_control.Odrive1.Parada_emergencia = False    
    msg_dual_control.Odrive2.Parada_emergencia = False
    pub_axis.publish(msg_dual_control)
    print("Parada de emergencia desactiva")
    time.sleep(5.0)

    msg_dual_control.Odrive1.Vel_ax0 = 360*5;
    msg_dual_control.Odrive1.Vel_ax1 = 360*5;
    msg_dual_control.Odrive2.Vel_ax0 = 360*5;
    msg_dual_control.Odrive2.Vel_ax1 = 360*5;
    msg_dual_control.Odrive1.Parada_emergencia = False    
    msg_dual_control.Odrive2.Parada_emergencia = False
    pub_axis.publish(msg_dual_control)
    print("velocidad de 5 RPS")
    time.sleep(10.0)

    msg_dual_control.Odrive1.Parada_emergencia = False    
    msg_dual_control.Odrive2.Parada_emergencia = False
    msg_dual_control.Odrive1.Vel_ax0 = 0
    msg_dual_control.Odrive1.Vel_ax1 = 0
    msg_dual_control.Odrive2.Vel_ax0 = 0
    msg_dual_control.Odrive2.Vel_ax1 = 0
    pub_axis.publish(msg_dual_control)
    print("velocidad de 0RPS")