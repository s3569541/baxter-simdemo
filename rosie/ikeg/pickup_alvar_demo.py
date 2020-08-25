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

from ar_track_alvar_msgs.msg import AlvarMarkers
from gazebo_msgs.srv import GetModelState
#import tf

# for Euler
import tf.transformations
from tf.transformations import *

from trac_ik_python.trac_ik import IK

#import random

import tf2_ros
import tf2_geometry_msgs

import geometry_msgs.msg

# refer to http://sdk.rethinkrobotics.com/wiki/IK_Service_Example

from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        )
from std_msgs.msg import Header

#from baxter_core_msgs.srv import (
#        SolvePositionIK,
#        SolvePositionIKRequest,
#        )
#import baxter_interface
#import baxter_external_devices
# from baxter_interface import CHECK_VERSION

### Needed?
#mutex = threading.Lock()

locallib.init()

#global tf_buffer
#tf_buffer = locallib.tf_buffer

global target_cube
target_cube = 1
global target_object
target_object = 'marker'+str(target_cube)
global target_marker_id
target_marker_id = 5

#######################################################################################
# Optional: for testing: do we have Baxter or Rosie? (only available during simulation)
#######################################################################################

from gazebo_msgs.srv import GetWorldProperties
parent_frame='head'
try:
    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    props = get_world_properties()
    for name in props.model_names:
      if name=='mobility_base' or name=='baxter':
        parent_model = name + "::head"
except rospy.ServiceException as e:
    print e
    sys.exit(1)

##
## This data is only available from simulation
##

from gazebo_msgs.srv import GetModelState
model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
print 'get position of marker wrt ',parent_model
coords = model_coordinates(target_object, parent_model)
blockpose = coords.pose
if coords.success:
  print('block pose',blockpose,'relative to',parent_model)
else:
  print('could not get marker1 position wrt',parent_model)

truepos = blockpose.position
tori = blockpose.orientation
# There are several different ROS implementations of ``quaternion'' depending on library/fn
# Here we use tf.transformations.euler_from_quaternion which requires a vector (x,y,y,w)
# (https://answers.ros.org/question/69754/quaternion-transformations-in-python/)
tquat = (tori.x,tori.y,tori.z,tori.w)
(trueroll,truepitch,trueyaw) = tf.transformations.euler_from_quaternion(tquat)
print 'true RPW',trueroll,truepitch,trueyaw

global mylimb
mylimb = 'left'

print 'calibrate / open gripper'
gripper = locallib.init_gripper(mylimb)
gripper.calibrate()
gripper.open()
rospy.sleep(1)
print 'gripper open'

global avgpos
global avgyaw
avgpos = None
avgyaw = None

def getavgpos():
    global avgpos
    global avgyaw
    (avgpos,avgyaw) = locallib.getavgpos(target_marker_id)
    print 'demo:','avgyaw',avgyaw

print '******* move to basic sighting pos ********'
print 'look for cube from pos1'
locallib.solve_move_trac(mylimb, locallib.make_pose_stamped(Point(x=0.49, y=-0.02, z=0.0), frame_id='base'))
print 'look for cube from pos2'
locallib.solve_move_trac(mylimb, locallib.make_pose_stamped(Point(x=0.49, y= 0.02, z=0.0), frame_id='base'))

# should have a sighting by now...?
while avgpos == None:
    rospy.sleep(2)
    getavgpos()

print '******* move to proper sighting pos ********'
locallib.solve_move_trac(mylimb, locallib.make_pose_stamped_yaw(Point(x=avgpos.x+0.1, y=avgpos.y, z=avgpos.z+0.20), frame_id='head', yaw=avgyaw))
rospy.sleep(2)
getavgpos()
print 'move to pregrab'
locallib.solve_move_trac(mylimb, locallib.make_pose_stamped_yaw(Point(x=avgpos.x, y=avgpos.y, z=avgpos.z+0.12), frame_id='head', yaw=avgyaw))

rospy.sleep(2)
getavgpos()
print 'move to grab'
locallib.solve_move_trac(mylimb, locallib.make_pose_stamped_yaw(Point(x=avgpos.x, y=avgpos.y, z=avgpos.z+0.04), frame_id='head', yaw=avgyaw))

###
### Close grippers
### 

print 'close (grab)'
rospy.sleep(1.0)
gripper.close()
rospy.sleep(1.0)

print 'lift'
locallib.solve_move_trac(mylimb, locallib.make_pose_stamped(Point(x=avgpos.x, y=avgpos.y, z=avgpos.z+0.20), frame_id='head'))

print 'get new position of marker'

coords = model_coordinates(target_object, parent_model)
if coords.success:
  print('block original position',truepos,'relative to',parent_model)
  pose = coords.pose
  newpos = pose.position
  print('block current position',newpos)
  # should be 15cm higher
  # note: more properly we would expect the cube to be approx 5cm below the 'frame' (camera) position
  if (abs(newpos.x - truepos.x) < 0.04 and abs(newpos.y - truepos.y) < 0.04 and abs((newpos.z - truepos.z) - 0.14) < 0.04):
    print 'success'
    sys.exit(0)
  else:
    print 'failure, block is at',newpos,'vs originally at',truepos
else:
  print('could not get ',target_object,' position wrt',parent_model)

print 'failure'
sys.exit(1)
