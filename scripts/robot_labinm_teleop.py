#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from odrive_ros_control.msg import od_dual_control

class Teleop:
    def __init__(self):
        rospy.init_node('robot_labinm_teleop')
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size = 5)
        self.pub_vel = rospy.Publisher('odrive_dual_control', od_dual_control, queue_size=5)
        rospy.Subscriber("joy", Joy, self.callback)
        self.rate = rospy.Rate(rospy.get_param('~hz', 20))
        #rospy.spin()
        self.cmd = Twist()
        self.linear_x = 0
        self.angular_z = 0
        
        self.msg_dual_control = od_dual_control();
        
        while not rospy.is_shutdown():
            self.cmd.linear.x = self.linear_x
            self.cmd.angular.z = self.angular_z
            self.cmd_pub.publish(self.cmd)
            self.rate.sleep()            

    def callback(self, data:Joy):
        """ Receive joystick data, formulate Twist message.
        Use planner if a secondary button is pressed """
        #cmd = Twist()
        #self.cmd.linear.x = 0
        #self.cmd.angular.z = 0
        
        self.linear_x = 0
        self.angular_z = 0
	
        #print(data.axes[0], ', ', data.axes[1])

        if data.buttons[5]:            
            self.msg_dual_control.Odrive1.Parada_emergencia = True
            self.msg_dual_control.Odrive2.Parada_emergencia = True
            self.pub_vel.publish(self.msg_dual_control)
            print("Boton RB activo")
        if data.buttons[4]:            
            self.msg_dual_control.Odrive1.Parada_emergencia = False
            self.msg_dual_control.Odrive2.Parada_emergencia = False
            self.pub_vel.publish(self.msg_dual_control)
            print("Boton LB activo")
        
        if data.axes[3] or data.axes[1]:
            #self.cmd.linear.x = data.axes[1]*5
            #self.cmd.angular.z = data.axes[0]*2
            #self.linear_x = data.axes[1]*3.0
            #self.angular_z = data.axes[0]*5
            
            #self.linear_x = data.axes[1]*3.0
            #self.angular_z = data.axes[3]*4.0
            
            self.linear_x = data.axes[1]*0.5
            self.angular_z = data.axes[3]*0.5

        

if __name__ == "__main__": Teleop()
