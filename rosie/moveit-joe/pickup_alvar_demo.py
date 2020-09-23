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

initial_pose = {} # to be populated from left hand camera
goal_pose = {} # to be populated from right hand camera

moveit_baxter.init()
locallib.init()

global mylimb
mylimb = 'left'

markers = {}
# the following will put the right arm in a good position for pyramid stack
# print("Move arm to initial scanning position")
# moveit_baxter.move_arm('right', 0.57, -0.55, -0.12, 0.475801111046, 0.520671447619, -0.488470343501, 0.513722950511)

while markers == {}:
    rospy.sleep(2)
    markers = locallib.getTopMarkerNumbers()

print "\nDetected Marker IDs"
print markers, "\n"

# Get locations of all blocks in right arm stack
for marker in markers:
  # occasionally the alvar systems thinks it sees 255 when it doesnt exist
  if marker != 255:
    # moveit_baxter.locateBlock(marker)
    print "Getting location of marker: ", marker
    locallib.getavgpos('right_hand_camera', marker)

# sort the right stack block ids by z axis value so the lowest blocks get stacked first
rightstack = locallib.getRightBlockPositions()
stackOrder = []
for marker in rightstack:
  if marker != 255:
    stackOrder.append((rightstack[marker]['AvgPos'].z, marker))

def sortSecond(val): 
  return val[0]
stackOrder.sort(key=sortSecond) 

for marker in stackOrder:
  # occasionally the alvar systems thinks it sees 255 when it doesnt exist
  if marker[1] != 255:
    moveit_baxter.pick(marker[1])
    moveit_baxter.place(marker[1], locallib.getRightBlockPositions()[marker[1]]['AvgPos'], locallib.getRightBlockPositions()[marker[1]]['avgyaw'])

# moveit_baxter.move_arm(mylimb, 0.6, 0.0, 0.1)

moveit_baxter.terminate()
