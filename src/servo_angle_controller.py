#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from arm_control.msg import ServoCommand
from Arm_Lib import Arm_Device
import time
from math import radians, sin, cos, pi

Arm = Arm_Device()
time.sleep(0.1)

DEFAULT_TIME = 500

def check(angles):
    L1 = 10
    L2 = 10
    L3 = 18
    x1, y1, z1 = 0, 0, L1
    radian = []
    
    print(x1, y1, z1)
    
    for i in angles:
        radian.append(radians(i))
    
    x2 = x1 + L2 * cos(radian[2] + pi/2)*cos(radian[1])
    #y2 = y1 + L2 * cos(radian[2] + pi/2)*sin(radian[1])
    z2 = z1 + L2 * sin(radian[2] + pi/2)
    
    # print(x2, y2, z2)
    
    x3 = x2 + L3 * cos(radian[3] + pi/2)*cos(radian[1])
    #y3 = y2 + L3 * cos(radian[3] + pi/2)*sin(radian[1])
    z3 = z2 + L3 * sin(radian[3] + pi/2)
    
    print("x=" + str(x3), "z=" + str(z3))
    
    if (x3 > 7 and z3 > -5):
        return True
    return False
    

def servo_callback(msg):
    if 1 <= msg.id <= 6 and 0 <= msg.angle <= 180:
        angles = []
        for i in range(1,7):
            angle = Arm.Arm_serial_servo_read(i)
            angles.append(angle)

        angles[msg.id-1] = msg.angle 
        
        if not check(angles):
            rospy.logerr("Недопустимое положение")
            return
        
        rospy.loginfo("Установка мотора %d в угол %.1f", msg.id, msg.angle)
        Arm.Arm_serial_servo_write(msg.id, msg.angle, DEFAULT_TIME)
    else:
        rospy.logwarn("Неверный id (%d) или угол (%.1f)", msg.id, msg.angle)

def main():
    rospy.init_node("servo_command_listener")
    rospy.Subscriber("/servo_command", ServoCommand, servo_callback)
    rospy.loginfo("Ожидаю команды на /servo_command")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
