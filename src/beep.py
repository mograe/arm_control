#!/usr/bin/env python3
#coding=utf-8
import time
import sys
from std_msgs.msg import Float32
import rospy
from Arm_Lib import Arm_Device
Arm = Arm_Device()
time.sleep(.1)
def beep_callback(msg):
    rospy.loginfo("beep")
    b_time = msg.data
    Arm.Arm_Buzzer_On()
    time.sleep(float(b_time))
    Arm.Arm_Buzzer_Off()
def main():
    rospy.init_node("arm_beep")
    rospy.Subscriber("beep_command", Float32, beep_callback)
    rospy.spin()
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass