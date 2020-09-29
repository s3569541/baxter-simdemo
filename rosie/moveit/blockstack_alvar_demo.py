#!/usr/bin/python

import socket
import math
import time
import Vectors
import struct
import threading
import Queue
import copy 
import rospy
from std_msgs.msg import String
import sys

# local collection of library functions (refactored)
import locallib
import moveit_baxter

global avgpos
global avgyaw
global last_seen
global pos_in_frame
global topbot
global center_pose

locallib.init(nodename='pickup_alvar_demo')
moveit_baxter.init()

# range of marker blocks spawned in sim:
availableMarkers = [0,10,20,30]

markers = {}
while markers == {}:
    rospy.sleep(0.5)
    markers = copy.deepcopy(locallib.avgmarkerpos)

print "\nKnown Block Numbers"
for marker in markers:
  print marker

rightStackMarkers = {}
# Get locations of all blocks in right arm stack
for marker in markers:
  if marker in availableMarkers:
    avgpos,avgyaw = moveit_baxter.locateBlock(marker)
    if avgpos != 0:
      rightStackMarkers[marker] = {"AvgPos": avgpos, "avgyaw" : avgyaw}

for marker in rightStackMarkers:
  print marker
  print rightStackMarkers[marker], "\n"

# sort the right stack block ids by z axis value so the lowest blocks get stacked first
stackOrder = []
for marker in rightStackMarkers:
  stackOrder.append((rightStackMarkers[marker]['AvgPos'].z, marker))

def sortSecond(val): 
  return val[0]
stackOrder.sort(key=sortSecond) 

for marker in stackOrder:
  moveit_baxter.pick(marker[1])
  moveit_baxter.place(marker[1], rightStackMarkers[marker[1]]['AvgPos'], rightStackMarkers[marker[1]]['avgyaw'])

moveit_baxter.terminate()
