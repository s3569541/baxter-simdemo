#!/usr/bin/python

import socket
import math
import time
import Vectors
import struct
import threading
import Queue
import rospy
from std_msgs.msg import String
import sys

# local collection of library functions (refactored)
import locallib
import moveit_baxter

locallib.init()
moveit_baxter.init()

stack = ''
if len(sys.argv)>1:
  stack = (sys.argv[1])

# 4 in a row with yaw
if stack == 'yaw':
    moveit_baxter.move_arm('right', 0.7, -0.05, -0.11)

# for pyramid of 4
elif stack == 'pymd':
    moveit_baxter.move_arm('right', 0.6, -0.2, -0.11, -0.475801111046, 0.520671447619, 0.488470343501, 0.513722950511)

# for two stacks of two
elif stack == '2of2':
    moveit_baxter.move_arm('right', 0.6, -0.24, -0.09, -0.475801111046, 0.520671447619, 0.488470343501, 0.513722950511)
    # moveit_baxter.move_arm('left',  0.6, -0.1, -0.09, 0.475801111046, 0.520671447619, -0.488470343501, 0.513722950511)

# for tower of 4
elif stack == 'tower':
    moveit_baxter.move_arm('right', 0.6, -0.2, -0.09, -0.475801111046, 0.520671447619, 0.488470343501, 0.513722950511)

#open grippers
elif stack == 'open':
    rgripper = locallib.init_gripper("right")
    lgripper = locallib.init_gripper("left")

else:
    moveit_baxter.move_arm("left", 0.4, 0.6, 0.1)
    moveit_baxter.move_arm('right', 0.4, -0.6, 0.1)

moveit_baxter.terminate()
