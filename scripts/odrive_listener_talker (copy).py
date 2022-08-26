#!/usr/bin/env python3
"""
Example usage of the ODrive python library to monitor and control ODrive devices
"""

from __future__ import print_function

import odrive
from odrive.enums import *
import time
import rospy
from std_msgs.msg import Int32
from std_msgs.msg import Float32

from odrive_ros_control.msg import od_dual_supervision
from odrive_ros_control.msg import od_dual_control

odrive0 = odrive.find_any(serial_number="20863696424D")
odrive0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrive0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

odrive1 = odrive.find_any(serial_number="20713694424D")
odrive1.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
odrive1.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL


vl = 0
vr = 0
isInsideCallbackLeftWheel = False
isInsideCallbackRightWheel = False

def callbackLeftWheel(data):

    global isInsideCallbackLeftWheel
    global vl
    #print('Entré')
    #rospy.loginfo(rospy.get_caller_id() + "I heard %d and %d", data.data)
    
    vl = float(data.data) / 360.0 #RPS
    #vl = data.data
    print('vl(grados/s): ', data.data, 'vl(RPS): ', vl)
    #vr = data.data[1]
    isInsideCallbackLeftWheel = True
    #delta_angle = 5
    
    #delta_angle = (360.0*(encoderPos-lastpos))/CPR; #change in position in degrees of the wheel
    #lastpos=encoderPos;
    
    #pub.publish(delta_angle)
    
def callbackRightWheel(data):

    global isInsideCallbackRightWheel
    global vr
      
    #rospy.loginfo(rospy.get_caller_id() + "I heard %d and %d", data.data)
    vr = float(data.data) / 360.0 #RPS
    
    print('vl(grados/s): ', data.data, 'vr(RPS): ', vr)
    isInsideCallbackRightWheel = True



if __name__ == '__main__':

    rospy.init_node('odrive_listener_talker', anonymous=True)

    rospy.Subscriber("left_wheel_speed", Float32, callbackLeftWheel)
    rospy.Subscriber("right_wheel_speed", Float32, callbackRightWheel)
    pubLeftWheelDeltaAngle = rospy.Publisher('left_wheel_delta_angle', Int32, queue_size=10)
    pubRightWheelDeltaAngle = rospy.Publisher('right_wheel_delta_angle', Int32, queue_size=10)
    
    #rospy.spin()
    
    '''line = input('Ingrese velocidad: ')
    vel_cmd_str = line
    vl = float(vel_cmd_str)
    odrive1.axis0.controller.input_vel = vl'''
    
    encoderPos = 0
    lastPos=0
    
    nroRevsActualLeftWheel1 = 0
    nroRevsLastLeftWheel1 = 0
    nroRevsActualLeftWheel2 = 0
    nroRevsLastLeftWheel2 = 0
    
    nroRevsActualRightWheel1 = 0
    nroRevsLastRightWheel1 = 0
    nroRevsActualRightWheel2 = 0
    nroRevsLastRightWheel2 = 0
    
    left_wheel_delta_angle1 = 0
    left_wheel_delta_angle2 = 0
    right_wheel_delta_angle1 = 0
    right_wheel_delta_angle2 = 0
    
    left_wheel_delta_angle = 0
    left_wheel_delta_angle = 0
    
    lastTime = 0
    dt = 1.0
    
    
    
    while not rospy.is_shutdown():
       now = time.time()
       timeChange = now - lastTime
	    
       if timeChange>=dt:            
            nroRevsActualLeftWheel1 = odrive0.axis0.encoder.pos_estimate_counts
            diffPosLeftWheel1 = nroRevsActualLeftWheel1 - nroRevsLastLeftWheel1                         
            left_wheel_delta_angle1 = (360.0*diffPosLeftWheel1)/(now - lastTime)
            
            nroRevsActualLeftWheel2 = odrive1.axis0.encoder.pos_estimate_counts
            diffPosLeftWheel2 = nroRevsActualLeftWheel2 - nroRevsLastLeftWheel2                         
            left_wheel_delta_angle2 = (360.0*diffPosLeftWheel2)/(now - lastTime)
            
            left_wheel_delta_angle = (left_wheel_delta_angle1+left_wheel_delta_angle2)/2.0
            
            nroRevsActualRightWheel1 = odrive0.axis1.encoder.pos_estimate_counts
            diffPosRightWheel1 = nroRevsActualRightWheel1 - nroRevsLastRightWheel1                         
            right_wheel_delta_angle1 = (360.0*diffPosRightWheel1)/(now - lastTime)
            
            nroRevsActualRightWheel2 = odrive1.axis1.encoder.pos_estimate_counts
            diffPosRightWheel2 = nroRevsActualRightWheel2 - nroRevsLastRightWheel2                         
            right_wheel_delta_angle2 = (360.0*diffPosRightWheel2)/(now - lastTime)
            
            right_wheel_delta_angle = (right_wheel_delta_angle1+right_wheel_delta_angle2)/2.0
            
            lastTime=now;
            nroRevsLastLeftWheel1 = nroRevsActualLeftWheel1
            nroRevsLastLeftWheel2 = nroRevsActualLeftWheel2
            nroRevsLastRightWheel1 = nroRevsActualRightWheel1
            nroRevsLastRightWheel2 = nroRevsActualRightWheel2
            
       if isInsideCallbackLeftWheel == True:
            #print(vl)
            #print('vl: ', vl)
            odrive0.axis0.controller.input_vel = vl
            odrive1.axis0.controller.input_vel = vl
            pubLeftWheelDeltaAngle.publish(int(left_wheel_delta_angle))
            isInsideCallbackLeftWheel = False
            
       if isInsideCallbackRightWheel == True:
            #print(vl)
            #print('vr: ', vr)            
            odrive0.axis1.controller.input_vel = vr
            odrive1.axis1.controller.input_vel = vr
            pubRightWheelDeltaAngle.publish(int(right_wheel_delta_angle))
            isInsideCallbackRighttWheel = False
            
