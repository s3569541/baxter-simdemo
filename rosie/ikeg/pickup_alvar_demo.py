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

import random

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

# TO DO: use laser rangefinder to improve z co-ordinate of block

def this_marker(marker):
    # TO DO: handle markers other than the current known-top marker
    # effectively marker.id is now (block number - 1) with bundles
    result = (marker.header.frame_id != 'head_camera') and (marker.id == target_marker_id)
    return result

locallib.init(nodename='pickup_alvar_demo',target_marker_fn=this_marker)

#global tf_buffer
#tf_buffer = locallib.tf_buffer

global target_cube
target_cube = 1
global target_object
target_object = 'marker'+str(target_cube)
global target_marker_id
target_marker_id = 0

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
global otherlimb
otherlimb = 'right'

print 'init / open gripper'
gripper = locallib.init_gripper(mylimb)
gripper.open()
rospy.sleep(1)
print 'gripper open'

global avgpos
avgpos = None
global avgyaw
avgyaw = None
global last_seen 
last_seen = None
global marker
marker = None
global pos_in_frame
pos_in_frame = None

def getavgpos(left_only=False):
    global avgpos
    global avgyaw
    global last_seen
    global pos_in_frame
    global topbot
    global center_pose
    avgmarkerpos = locallib.avgmarkerpos
    d0 = {}
    d = {}
    print 'awaiting Alvar avg'
    rospy.sleep(1)
    if target_marker_id in avgmarkerpos:
        d0 = avgmarkerpos[target_marker_id]
    if 'right_hand_camera' in d0 and not left_only:
        d = d0['right_hand_camera']
    if 'left_hand_camera' in d0:
        d = d0['left_hand_camera']
    if 'avg' in d:
        marker = d['marker']
        (topbot,center_pose) = locallib.get_top_or_bot_blockface(marker,d['avg_master_pose'])
        #avgpos = d['avg']
        avgpos = center_pose.pose.position
        #avgyaw = d['avg_rpy'][2]
        rpy = locallib.euler_from_quat(topbot.pose.orientation)
        print 'avgpos',avgpos,'rpy',rpy
        avgyaw = rpy[2]
        # experimental: canonicalise yaw to improve stability of path to pickup (suspect yaw sometimes "flips" to different representation of same canonical angle??)
        while avgyaw < 0:
            avgyaw = avgyaw + math.pi
        while avgyaw > math.pi:
            avgyaw = avgyaw - math.pi
        last_seen = d['last_seen']
        pos_in_frame = marker.pose.pose.position
        frame = d['frame']
        print '- getavgpos','avg pos',avgpos,'avg rpy',d['avg_rpy'],'last sighting',rospy.get_time() - last_seen,'s ago'
        print '- marker wrt ',frame,':',pos_in_frame.x,pos_in_frame.y,pos_in_frame.z
        return True
    return False

# FIXME: back off and try from more positions
# FIXME: point right camera at best guess position
# FIXME: give up if the marker is consistently not found

otherpose = locallib.get_frame(otherlimb+'_gripper','base')
print 'camera on',otherlimb,'...'
locallib.solve_move_trac(otherlimb, locallib.make_pose_stamped_rpy(otherpose.transform.translation, frame_id='base', r=3.3, p=0.0))

grab = False
stage = 0

def implies(p,q):
    return not(p) or q

# assume in a fixed frame, such as 'head', so we can check for discontinuities
pos_by_stage = {}

global last_seen_yaw
last_seen_yaw = 0

while not grab:
    print 'stage',stage
    if stage == 0:
        # No sighting available
        print '******* move to fallback sighting pos ********'
        locallib.solve_move_trac(mylimb, locallib.make_pose_stamped(Point(x=0.49, y=0.0 + random.random() * 0.4, z=0.2), frame_id='base'))
        last_seen = 0
        last_seen_sighting = 0
    if stage == 1:
        # Use whatever average position we have as a basis for proper sighting
        print '******* move to sighting pos ********'
        locallib.solve_move_trac(mylimb, locallib.make_pose_stamped_yaw(Point(x=avgpos.x+0.1, y=avgpos.y, z=avgpos.z+0.22), frame_id='head', yaw=avgyaw))
    if stage == 2:
        print '********* move to pregrab **********'
        locallib.solve_move_trac(mylimb, locallib.make_pose_stamped_yaw(Point(x=avgpos.x, y=avgpos.y, z=avgpos.z+0.18), frame_id='head', yaw=avgyaw))
        rospy.sleep(1)
    if stage == 3:
        print '********* move to grab **********'
        locallib.solve_move_trac(mylimb, locallib.make_pose_stamped_yaw(Point(x=avgpos.x, y=avgpos.y, z=avgpos.z+0.08), frame_id='head', yaw=avgyaw))
        grab = True
    # update vision QOS info
    # PASSING: getavgpos(left_only=(stage>=1))
    # DELAY here
    getavgpos(left_only=(stage>=2))
    pos_by_stage[stage] = avgpos
    # for stage 3, pre: gripper centred on the marker
    centred = pos_in_frame and abs(pos_in_frame.x)<0.03 and abs(pos_in_frame.y)<0.1
    fresh = last_seen > last_seen_sighting
    marker_stable = True
    yaw_stable = abs(last_seen_yaw - avgyaw) < (math.pi/20)
    print 'stage:', stage, 'fresh?', fresh, 'pos in frame', pos_in_frame
    if stage>0:
        marker_movement = locallib.diff(pos_by_stage[stage], pos_by_stage[stage - 1])
        marker_stable = (marker_movement.x < 0.02) and (marker_movement.y < 0.02) and (marker_movement.z < 0.02)
        print '-', 'marker movement since last stage', marker_movement
    #vision_valid = fresh and implies(stage == 2, centred and marker_stable)
    vision_valid = fresh and implies(stage == 2, centred and yaw_stable)
    #vision_valid = fresh and implies(stage == 2, marker_stable)
    #vision_valid = fresh
    last_seen_sighting = last_seen
    last_seen_yaw = avgyaw
    # if still have vision, proceed, otherwise back up
    if vision_valid:
        stage = stage + 1
    else:
        if (stage == 2) and not centred:
            print 'stage 2 and not centred'
        stage = max(stage - 1, 0)
        print '-> stage',stage

if not grab:
    print 'unknown failure'
    sys.exit(1)

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
