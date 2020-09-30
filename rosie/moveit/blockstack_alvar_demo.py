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
import json
import sys

print 'argv',len(sys.argv)
global twins
twins=False
global primary
primary=False
if len(sys.argv)>1:
  twins=True
  primary=(sys.argv[1]=='primary')
print 'twins',twins,'primary',primary

# local collection of library functions (refactored)
import locallib
import moveit_baxter

def scanBlockStack(availableMarkers):
  # range of marker blocks spawned in sim:
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
      avgpos,avgyaw = moveit_baxter.locateBlock('right_hand_camera', marker)
      rightStackMarkers[marker] = ({"AvgPos": {"x": avgpos.x, "y": avgpos.y, "z": avgpos.z}, "avgyaw" : avgyaw})

  # sort the right stack block ids by z axis value so the lowest blocks get stacked first
  stackOrder = []
  for marker in rightStackMarkers:
    stackOrder.append((rightStackMarkers[marker]['AvgPos']["z"], marker))

  def sortSecond(val): 
    return val[0]
  
  stackOrder.sort(key=sortSecond) 

  final = {}
  for marker in stackOrder:
    final[marker[1]] = (rightStackMarkers[marker[1]])
  # print "!!final!!\n", final
  return final

def createStack(stackOrder):
  blockorder = json.loads(stackOrder)
  for marker in blockorder:
    avgpos = moveit_baxter.pick('left', int(marker))
    pose = blockorder[marker]['AvgPos']
    avgpos.x = pose["x"]
    avgpos.y = pose["y"]
    avgpos.z = pose["z"]
    moveit_baxter.place('left', avgpos, blockorder[marker]['avgyaw'])

def callback(data):
  rospy.loginfo(rospy.get_caller_id() + "\nI heard %s", data.data)
  createStack(data.data)

if twins:
  if primary:
    locallib.init('scanner')
    # uncomment this if you need to move the arm to scan:
    # moveit_baxter.init()
    pub = rospy.Publisher('/rosie/blockstack', String, queue_size=10)

    stackOrder = scanBlockStack([0,10])#,20,30])
    json_mylist = json.dumps(stackOrder, separators=(',', ':'))

    while not rospy.is_shutdown():
      print "\nSending blockstack data:"
      rospy.loginfo(stackOrder)
      pub.publish(json_mylist)
      rospy.sleep(2.0)

  else:
    locallib.init('stacker')
    moveit_baxter.init()
    rospy.Subscriber('/rosie/blockstack', String, callback)
    rospy.spin()

else:
  locallib.init(nodename='pickup_alvar_demo')
  moveit_baxter.init()
  blockStack = scanBlockStack([0,10,20,30])
  createStack(blockStack)

moveit_baxter.terminate()
