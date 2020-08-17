#!/usr/bin/python
# Baxter boogie REAM team 2018 Semester 1

import socket
import math
import time
import Vectors
import struct
import threading
import Queue
import rospy
from std_msgs.msg import String
from ar_track_alvar_msgs.msg import AlvarMarkers
import tf
from tf.transformations import *

from trac_ik_python.trac_ik import IK

import random

import tf2_ros
import tf2_geometry_msgs

rospy.init_node("marker_tracking",anonymous=True)

tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

# refer to http://sdk.rethinkrobotics.com/wiki/IK_Service_Example

from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        )
from std_msgs.msg import Header

from baxter_core_msgs.srv import (
        SolvePositionIK,
        SolvePositionIKRequest,
        )
import baxter_interface
import baxter_external_devices

from baxter_interface import CHECK_VERSION

direction = 1

mutex = threading.Lock()

last_data = None;
dz = 0.0;

global posedebug
posedebug = rospy.Publisher('/red/pose_debug', PoseStamped, queue_size=2)
global marker_topic
marker_topic = rospy.Publisher('/red/ikeg/target_marker_pose', PoseStamped, queue_size=5)
global target_topic
target_topic = rospy.Publisher('/red/ikeg/target_gripper_pose', PoseStamped, queue_size=5)

global redebug
redebug = rospy.Publisher('/red/debug', String, queue_size=1)

# Something in here doesn't work unless Baxter is the master
#rs = baxter_interface.RobotEnable(CHECK_VERSION)
#init_state = rs.state().enabled
#print 'initial state', init_state
#rs.enable()


##################################
# TRAC IK solver for Baxter
##################################

global mylimb
mylimb = 'left'

# ps: PoseStamped
def trac_ik_solve(limb, ps):
	print 'publish',ps
        target_topic.publish(ps)
        local_base_frame = limb+"_arm_mount"
        ik_solver = IK(local_base_frame,
                       limb+"_wrist",
                       urdf_string=urdf_str)
        seed_state = [0.0] * ik_solver.number_of_joints
        # canonical pose in local_base_frame
        #hdr = Header(stamp=rospy.Time.now(), frame_id=from_frame)
        #ps = PoseStamped(
        #        header=hdr,
        #        pose=pose,
        #        )
        p = translate_frame(ps,local_base_frame)
        #print 'translated frame',p
        soln = ik_solver.get_ik(seed_state,
                        p.pose.position.x,p.pose.position.y,p.pose.position.z,  # X, Y, Z
                        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w,  # QX, QY, QZ, QW
                        0.01,0.01,0.01,
                        0.1,0.1,0.1,

        )
        print 'trac soln',soln
        return soln

# translate PoseStamped arg:ps into Pose with respect to arg:frame
def translate_frame(ps,frame):
    #print 'translate_frame',pose
    global tf_buffer
    # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
    transform = tf_buffer.lookup_transform(frame,
      ps.header.frame_id, #source frame
      rospy.Time(0), #get the tf at first available time
      rospy.Duration(2.0)) #wait for 2 seconds
    pose_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)
    #print 'pose in baxter frame',pose_transformed,' ', frame_to
    return pose_transformed

from gazebo_msgs.srv import GetModelState

avgpos = None
blockspos = [None,None,None,None,None,None]

def gazebo_pos_marker(blocknr):
    print 'finding block',blocknr
    global blockpos
    model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
    blockname = 'marker'+str(blocknr)
    coords = model_coordinates(blockname, 'baxter::base')
    blockpose = coords.pose
    if coords.success:
      print('block pose',blockpose)
    else:
      print('could not get ',blockname,'position wrt base frame')
    blockpos = blockpose.position
    blockspos[blocknr] = blockpos
    return blockpos

def gazebo_pos_marker1():
    return gazebo_pos_marker(1)

def gazebo_pos_markerblocks():
    gazebo_pos_marker(1)
    gazebo_pos_marker(2)
    gazebo_pos_marker(3)
    gazebo_pos_marker(4)
    gazebo_pos_marker(5)

def makepose(pos, orientation=Quaternion(x=0, y=1, z=0, w=0)):
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x,
                y=pos.y,
                z=pos.z,
                ),
            orientation=orientation,
            )
    return pose_right

def make_pose_stamped(pos,frame_id='base', orientation=Quaternion(x=0, y=1, z=0, w=0)):
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=makepose(pos, orientation),
  )

avgspos = [None,None,None,None,None,None,None]
avgmarkerpos = {}

def pairavg(a,b):
    return (a + b) / 2

def rnd(v):
    return math.floor(v * 1000) / 1000

def dst(a,b):
    diff = abs(a - b)
    return rnd(diff)

def marker_callback(data):
  global avgpos
  global blockpos
  global avgmarkerpos
  if data.markers:
      for marker in data.markers:
          if marker.header.frame_id != 'head_camera':
              blocknr = int(marker.id / 10) + 1
              print 'block',blocknr,'marker',marker.id,'frame',marker.header.frame_id
              print '- marker',marker.id
              basepose = translate_frame(make_pose_stamped(marker.pose.pose.position,marker.header.frame_id), 'base')
              pos = basepose.pose.position
              #print '- pos',pos
              if not (marker.id in avgmarkerpos):
                  avgmarkerpos[marker.id] = pos
              avgpos = avgmarkerpos[marker.id]
              avgmarkerpos[marker.id] = Point(x = pairavg(avgpos.x, pos.x), y = pairavg(avgpos.y, pos.y), z = pairavg(avgpos.z, pos.z))
              blockpos = blockspos[blocknr] 
              print '- average pos',avgpos
              print '- block pos',blockpos
              print '- error',dst(avgpos.x,blockpos.x),dst(avgpos.y,blockpos.y),dst(avgpos.z,blockpos.z)

gazebo_pos_markerblocks()
rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_callback, queue_size=1)

rospy.spin()
