#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from arm_control.msg import ColorCommand
from Arm_Lib import Arm_Device
import time

Arm = Arm_Device()
time.sleep(0.1)

DEFAULT_TIME = 1000
GRAB_TIME = 1500  
LET_TIME = 1500

CLAW_TIME = 150

BASE_POSITION = [90, 152, 24, 22, 90]
BLACK_POSITION = [90, 48, 35, 30, 90]
GREEN_POSITION = [136, 66, 20, 29, 90]
RED_POSITION_GRAB = [115, 19, 66, 56, 90]
RED_POSITION_LET = [115, 19, 66, 56, 90]
YELLOW_POSITION_GRAB = [61, 22, 64, 56, 90]
YELLOW_POSITION_LET = [61, 22, 64, 56, 90]
BLUE_POSITION = [44, 66, 20, 28, 90]
GRAB_HANDLE = 150
LET_HANDLE = 0

def change_position(position, time_move=DEFAULT_TIME):
    rospy.loginfo("Перемещние в позицию" + str(position))
    for i in range(5):
        Arm.Arm_serial_servo_write(i+1, position[i], time_move)
        time.sleep(0.01)
        
def grab():
    rospy.loginfo(f"Захват: {GRAB_HANDLE}" )
    Arm.Arm_serial_servo_write(6, GRAB_HANDLE, CLAW_TIME)
    time.sleep(0.01)

def let():
    rospy.loginfo(f"Расхват: {LET_HANDLE}" )
    Arm.Arm_serial_servo_write(6, LET_HANDLE, CLAW_TIME)
    time.sleep(0.01)
        
def move_callback(msg):
    let()
    change_position(BASE_POSITION)
    
    time.sleep(1)
    
    rospy.loginfo("Цвет отправления: " + msg.source_color.lower())
    rospy.loginfo("Цвет назначения: " + msg.destination_color.lower())
    
    if (msg.source_color.lower() == 'black'):
        change_position(BLACK_POSITION, GRAB_TIME)
    elif(msg.source_color.lower() == 'green'):
        change_position(GREEN_POSITION, GRAB_TIME)
    elif(msg.source_color.lower() == 'red'):
        change_position(RED_POSITION_GRAB, GRAB_TIME)
    elif(msg.source_color.lower() == 'yellow'):
        change_position(YELLOW_POSITION_GRAB, GRAB_TIME)
    elif(msg.source_color.lower() == 'blue'):
        change_position(BLUE_POSITION, GRAB_TIME)
    
    time.sleep(2)
    
    grab()
        
    time.sleep(1)
    
    change_position(BASE_POSITION)
    
    time.sleep(1)
    
    if (msg.destination_color.lower() == 'black'):
        change_position(BLACK_POSITION, LET_TIME)
    elif(msg.destination_color.lower() == 'green'):
        change_position(GREEN_POSITION, LET_TIME)
    elif(msg.destination_color.lower() == 'red'):
        change_position(RED_POSITION_LET, LET_TIME)
    elif(msg.destination_color.lower() == 'yellow'):
        change_position(YELLOW_POSITION_LET, LET_TIME)
    elif(msg.destination_color.lower() == 'blue'):
        change_position(BLUE_POSITION, LET_TIME)
        
    time.sleep(2)
    
    let()
        
    time.sleep(1)

def main():
    rospy.init_node("command_command_listener")
    rospy.Subscriber("/move_command", ColorCommand, move_callback)
    rospy.loginfo("Ожидаю команды на /move_command")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass