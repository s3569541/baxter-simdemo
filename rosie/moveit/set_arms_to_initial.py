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
# this is to ensure gripper is open
rgripper = locallib.init_gripper("right")

# following position if good for tower of 4 spawn
moveit_baxter.move_arm('left', 0.6, 0.2, 0.1)
moveit_baxter.move_arm('right', 0.57, -0.55, -0.09, 0.475801111046, 0.520671447619, -0.488470343501, 0.513722950511)

moveit_baxter.terminate()
