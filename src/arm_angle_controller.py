#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float32MultiArray
from Arm_Lib import Arm_Device
import time
from math import radians, sin, cos, pi

Arm = Arm_Device()
time.sleep(0.1)

MOVE_TIME = 500  # время движения для всех моторов

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
    

def angle_callback(msg):
    angles = msg.data
    if len(angles) != 6:
        rospy.logwarn("Ожидается 6 углов, получено %d", len(angles))
        return
    
    if not check(angles):
        rospy.logerr("Недопустимое положение")
        return

    for i in range(6):
        angle = angles[i]
        if 0 > angle > 180:
            rospy.logerr("Недопустимый угол %.1f для мотора %d", angle, servo_id)
            return
        servo_id = i + 1
        rospy.loginfo("Установка мотора %d в угол %.1f", servo_id, angle)
        Arm.Arm_serial_servo_write(servo_id, angle, MOVE_TIME)
        time.sleep(0.01)

def main():
    rospy.init_node("arm_angle_controller")
    rospy.Subscriber("/set_angles", Float32MultiArray, angle_callback)
    rospy.loginfo("Готов к приёму углов на /set_angles")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass