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

moveit_baxter.move_arm('left', 0.6, 0.2, 0.1)
moveit_baxter.move_arm('right', 0.6, -0.2, 0.1)



moveit_baxter.terminate()
