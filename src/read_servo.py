#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from Arm_Lib import Arm_Device

Arm = Arm_Device()

for i in range(1,7):
    angle = Arm.Arm_serial_servo_read(i)
    print(angle)

