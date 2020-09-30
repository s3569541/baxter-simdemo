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

  return rightStackMarkers

def createStack(stackOrder):
  blockdata = json.loads(stackOrder)

  stackOrder = []
  for marker in blockdata:
    stackOrder.append((blockdata[marker]['AvgPos']["z"], marker))

  def sortSecond(val): 
    return val[0]
  
  stackOrder.sort(key=sortSecond) 

  finalOrder = []
  for marker in stackOrder:
    # print "Marker:", marker[1], " \n", blockdata[marker[1]], "\n"
    finalOrder.append((marker[1],  blockdata[marker[1]]))

  print "stack order\n", stackOrder  
  for marker in finalOrder:
    print marker[0]
    print marker[1]['AvgPos']

  for marker in finalOrder:
    avgpos = moveit_baxter.pick('left', int(marker[0]))
    avgpos.x = marker[1]['AvgPos']['x']
    avgpos.y = marker[1]['AvgPos']['y']
    avgpos.z = marker[1]['AvgPos']['z']
    moveit_baxter.place('left', avgpos, int(marker[1]['avgyaw']))

  moveit_baxter.terminate()

def callback(data):
  rospy.loginfo(rospy.get_caller_id() + "\nI heard %s", data.data)
  print data.data
  createStack(data.data)
  
if twins:
  if primary:
    locallib.init('scanner')
    # uncomment this if you need to move the arm to scan:
    # moveit_baxter.init()
    pub = rospy.Publisher('/rosie/blockstack', String, queue_size=10)

    stackOrder = scanBlockStack([0,10,20,30])
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
