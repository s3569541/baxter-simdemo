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

#time.sleep(2)

print 'init publishers...'
global posedebug
posedebug = rospy.Publisher('/red/pose_debug', PoseStamped, queue_size=1)

global marker0_topic
marker0_topic = rospy.Publisher('/rosie/marker0', PoseStamped, queue_size=1)
global marker1_topic
marker1_topic = rospy.Publisher('/rosie/marker1', PoseStamped, queue_size=1)
global marker2_topic
marker2_topic = rospy.Publisher('/rosie/marker2', PoseStamped, queue_size=1)

global target_topic
target_topic = rospy.Publisher('/red/ikeg/target_gripper_pose', PoseStamped, queue_size=1)
print 'init publishers done'

global redebug
redebug = rospy.Publisher('/red/debug', String, queue_size=1)

# translate a pose in terms of its "own" rotational axes (as if it had its own frame)
def translate_pose_in_own_frame(ps,localname,dx,dy,dz):
  m = geometry_msgs.msg.TransformStamped()
  m.child_frame_id = localname
  m.header.frame_id = ps.header.frame_id
  m.header.stamp = rospy.Time.now()
  m.transform.translation.x=ps.pose.position.x
  m.transform.translation.y=ps.pose.position.y
  m.transform.translation.z=ps.pose.position.z
  m.transform.rotation.x=ps.pose.orientation.x
  m.transform.rotation.y=ps.pose.orientation.y
  m.transform.rotation.z=ps.pose.orientation.z
  m.transform.rotation.w=ps.pose.orientation.w
  tf_buffer.set_transform(m,"")
  return translate_in_frame(ps,localname,dx,dy,dz)

# get target_frame with respect to source_frame
def get_frame(target_frame,source_frame):
    #print 'translate_frame',pose
    global tf_buffer
    # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
    transform = tf_buffer.lookup_transform(source_frame,
      target_frame,
      rospy.Time(0), #get the tf at first available time
      rospy.Duration(2.0)) #wait for 2 seconds
    #print 'pose of',target_frame,'wrt',source_frame,':',transform
    return transform

# translate PoseStamped arg:ps into Pose with respect to arg:frame
def translate_frame(ps,frame):
    #print 'translate_frame',pose
    global tf_buffer
    # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
    transform = tf_buffer.lookup_transform(frame, # source frame
      ps.header.frame_id, # target frame
      rospy.Time(0), #get the tf at first available time
      rospy.Duration(2.0)) #wait for 2 seconds
    pose_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)
    #print 'pose in baxter frame',pose_transformed,' ', frame_to
    return pose_transformed

##################################
# TRAC IK solver for Baxter
##################################

# make search start position from current joint states
def make_seed(limb):
        arm = baxter_interface.Limb(limb)
        state = arm.joint_angles()
        #print 'solve current:',arm.joint_angles()
        jointnames = ['s0','s1','e0','e1','w0','w1','w2']
        seed = []
        for i in range(0, len(jointnames)):
            key = limb+"_"+jointnames[i]
            seed.append(state[key])
        return seed

def trac_ik_solve(limb, ps):
        print 'trac_ik_solve',limb,ps
        target_topic.publish(ps)
        seed1 = make_seed(limb)
        print 'candidate seed based on current position',seed1
        local_base_frame = limb+"_arm_mount"
        soln1 = trac_ik_solve_with_seed(limb, ps, seed1)
        if soln1:
            print 'found solution searching from current position'
            return soln1
        print 'no solution found searching from current position',seed1
        ik_solver = IK(local_base_frame,
                       #limb+"_wrist",
                       limb+"_gripper",
                       urdf_string=urdf_str)
        seed2 = [0.0] * len(seed1)
        print 'candidate zero position seed',seed2
        soln2 = trac_ik_solve_with_seed(limb, ps, seed2)
        if soln2:
            print 'found solution searching from zero position'
            return soln2
        print 'no solution found searching from zero position'
        return None

# ps: PoseStamped
def trac_ik_solve_with_seed(limb, ps, seed):
        local_base_frame = limb+"_arm_mount"
        ik_solver = IK(local_base_frame,
                       #limb+"_wrist",
                       limb+"_gripper",
                       urdf_string=urdf_str)
	gripper_ps = translate_pose_in_own_frame(ps,'gripper_target',0,0,0.05)
        p = translate_frame(gripper_ps,local_base_frame)
        #print 'translated frame',p
        soln = ik_solver.get_ik(seed,
                        p.pose.position.x,p.pose.position.y,p.pose.position.z,  # X, Y, Z
                        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w,  # QX, QY, QZ, QW
                        0.01,0.01,0.01,
                        0.1,0.1,0.1,

        )
        return soln

print 'get_param /robot_description...'
# Get your URDF from somewhere
urdf_str = rospy.get_param('/robot_description')
print 'get_param /robot_description done'

def ik_solve(limb,pose,frame):
    return trac_ik_solve(limb,pose,frame)

# joints: 
#  - 
#    header: 
#      seq: 0
#      stamp: 
#        secs: 0
#        nsecs:         0
#      frame_id: ''
#    name: [right_s0, right_s1, right_e0, right_e1, right_w0, right_w1, right_w2]
#    position: [0.2561138207510154, -0.6456376771715344, 0.6654558305409705, 0.6501751367692917, -0.5169514656977631, 1.6621515293700257, -0.13885424105804758]
#    velocity: []
#    effort: []
#isValid: [True]
#result_type: [2]

def make_move_baxter(msg, limb, speed):
    print 'make_move',msg
    SPEED_SCALE = 1
    speed = speed * SPEED_SCALE
    arm = baxter_interface.Limb(limb)
    print 'arm',arm
    lj = arm.joint_names()
    command = {}
    for i in range(0, len(msg.joints[0].name)):
        command[msg.joints[0].name[i]] = msg.joints[0].position[i]
    print 'current:',arm.joint_angles()
    print 'make_move: speed',speed
    arm.set_joint_position_speed(speed)
    print 'make_move_baxter: posns',command
    #arm.set_joint_positions(command)
    arm.move_to_joint_positions(command)
    print 'make_move: done'

# msg: 7-vector of positions corresponding to joints
# limb: 'left'|'right'
def make_move_trac(msg, limb, speed):
    if msg==None:
        raise BaseException('no solution was provided')
    print 'make_move_trac',msg
    SPEED_SCALE = 1
    speed = speed * SPEED_SCALE
    arm = baxter_interface.Limb(limb)
    print 'make_move_trac arm',arm
    lj = arm.joint_names()
    command = {}
    # for i in range(0, len(msg.joints[0].name)):
    jointnames = ['s0','s1','e0','e1','w0','w1','w2']
    for i in range(0, len(jointnames)):
        command[limb+"_"+jointnames[i]] = msg[i]
    print 'current:',arm.joint_angles()
    print 'make_move: speed',speed
    arm.set_joint_position_speed(speed)
    print 'make_move_trac: posns',command
    #arm.set_joint_positions(command)
    arm.move_to_joint_positions(command)
    print 'make_move: done'

# move limb-gripper to given PoseStamped (absolute)
# limb: 'left'|'right'
# ps: PoseStamped including header.frame_id
def solve_move_trac(limb,ps):
    soln = trac_ik_solve(limb,ps)
    make_move_trac(soln, limb, 0.8)

def recv_data(data):
    '''
    Take udp data and break into component messages
    '''
    
    # split on token
    list_data = data.split(',')
    
    # Convert next beat time
    list_data[0] = float(list_data[0])

    # Convert and normalize energy
    list_data[1] = float(list_data[1])
    list_data[1] = min(3000, list_data[1])
    list_data[1] /= 3000

    list_data[2] = float(list_data[2]);

    dict_data = {'beat': list_data[0], 'energy': list_data[1], 'onset': list_data[2]}

    return dict_data

def decide_increment(data): 
    #TODO - decide based on data
    mutex.acquire()
    global direction
    
    global dz

    global last_data;
    if last_data is None:
        last_data = data;

    if (data is not None):
        wow = data['onset'];
        dwow = data['onset'] - last_data['onset'];
        if abs(dwow) > 0.01:
            dz = dwow / 10.0;
        print "dwow", dwow;

    print "dz: ", dz

    inc = Vectors.V4D(0.0, direction * 0.05, dz , 0.0)
    mutex.release()

    last_data = data;

    return inc;

# Quaternion -> Quaternion
def normalize(quat):
 	quatNorm = math.sqrt(quat.x * quat.x + quat.y *
                        quat.y + quat.z * quat.z + quat.w * quat.w)
        normQuat = Quaternion(quat.x / quatNorm,
                              quat.y / quatNorm,
                              quat.z / quatNorm,
                              quat.w / quatNorm)
	return normQuat

# translate PoseStamped ps with respect to a given frame
def translate_in_frame(ps,frame,dx,dy,dz):
    ps_f = translate_frame(ps,frame)
    result = PoseStamped(
	header=ps_f.header,
	pose=Pose(
            position=Point(
                x=ps_f.pose.position.x + dx,
                y=ps_f.pose.position.y + dy,
                z=ps_f.pose.position.z + dz
                ),
            orientation=ps_f.pose.orientation
    ))
    return result

# translate PoseStamped ps on Z axis by dz in base_link frame
def lift_in_base_frame(ps,dz):
    return translate_in_frame(ps,'base_link',0,0,dz)

# translate PoseStamped ps on Z axis by dz in base_link frame (legacy)
def lift_in_base_frame_old(ps,dz):
    psbl = translate_frame(ps,'base_link')
    result = PoseStamped(
	header=psbl.header,
	pose=Pose(
            position=Point(
                x=psbl.pose.position.x,
                y=psbl.pose.position.y,
                z=psbl.pose.position.z + dz
                ),
            orientation=psbl.pose.orientation
    ))
    return result

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
            #orientation=Quaternion( x=0, y=1, z=0, w=0), # straight up and down
            #orientation=Quaternion( x=0.0462177008579, y=0.889249134439, z=0.0227669346795, w=0.454512450557), # toed-in for right camera
            )
    return pose_right

def make_pose_stamped_from_pose(pose, frame_id='base'):
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=pose
  )

def make_pose_stamped(pos,frame_id='base', ori=Quaternion(x=0, y=1, z=0, w=0)):
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=makepose(pos, ori),
  )

def make_pose_stamped_rpy(pos=(0,0,0), frame_id='base', r=3.14, p=0.0, y=0.0):
  print 'make_pose_stamped_rpy',r,p,y
  ori = quaternion_from_euler(r, p, y)
  print 'ori',ori
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=makepose(pos, Quaternion(x = ori[0], y=ori[1], z=ori[2], w=ori[3])),
  )

def make_pose_stamped_yaw(pos, frame_id='base', yaw=0):
  # yaw pitch roll
  print 'make_pose_stamped yaw',yaw
  ori = quaternion_from_euler(3.14, 0, yaw)
  print 'ori',ori
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=makepose(pos, Quaternion(x = ori[0], y=ori[1], z=ori[2], w=ori[3])),
  )

###
### Library
###

# Ensure that dict contains an entry for key, using dict[key]=default
# Return the value of dict[key]
def init_key(dict,key,default):
  if not (key in dict):
    dict[key]=default
  return dict[key]

def square(v1):
  return v1*v1

def dist3d(pt1,pt2):
  return round(math.sqrt(square(pt1.x - pt2.x) + square(pt1.y - pt2.y) + square(pt1.z - pt2.z)))

def pairavg(a,b):
    if not (type(a) is int or type(a) is float):
        print 'a is non numeric ',str(a)
    if not (type(b) is int or type(b) is float):
        print 'b is non numeric ',str(b)
    return (a + b) / 2

def diff(a,b):
    return Point(x=abs(a.x - b.x), y=abs(a.y - b.y), z=abs(a.z - b.z))

def round(v):
  return math.floor(v * 1000) / 1000

#time.sleep(2)

###
### Alvar marker / block tracking
### TODO: collapse into Object
###

#global target_cube
#target_cube = 1
#global target_object
#target_object = 'marker'+str(target_cube)
#global target_marker_id
#target_marker_id = 5

## marker id -> frame id -> {avg(marker pose in base frame):, err: int(camera to block dist) -> int(distance)}
avgmarkerpos = {}

#('block pose', position:
#  x: 0.44074679349
#  y: 0.0302300076348
#  z: -0.881031211204
#orientation:
#  x: -8.26386015882e-05
#  y: 3.71056433154e-05
#  z: 0.342937175775
#  w: 0.939358336986, 'relative to', 'baxter::head')

def any_marker(marker):
  return True

global _target_marker_fn
_target_marker_fn = any_marker

def block_nr(marker):
    return int(marker.id / 10) + 1

# child frame id is required, not parent frame id, because PoseStamped is anonymous and defined in terms of another PARENT frame
def new_frame(ps,child_id):
  m = geometry_msgs.msg.TransformStamped()
  m.child_frame_id = child_id
  m.header.stamp = ps.header.stamp
  m.header.frame_id = ps.header.frame_id
  m.transform.translation.x=ps.pose.position.x
  m.transform.translation.y=ps.pose.position.y
  m.transform.translation.z=ps.pose.position.z
  m.transform.rotation.x=ps.pose.orientation.x
  m.transform.rotation.y=ps.pose.orientation.y
  m.transform.rotation.z=ps.pose.orientation.z
  m.transform.rotation.w=ps.pose.orientation.w
  tf_buffer.set_transform(m,node_name)

def quat_from_euler(r,p,y):
    ori = quaternion_from_euler(r,p,y)
    return Quaternion(x = ori[0], y=ori[1], z=ori[2], w=ori[3])

def euler_from_quat(ori):
  # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
  quat = (ori.x,ori.y,ori.z,ori.w)
  return tf.transformations.euler_from_quaternion(quat)

def block_master_frame_id(blocknr):
  return 'block'+str(blocknr)+'_master'

# get Top or Bottom marker to find yaw info for gripper pose
def get_top_or_bot_blockface(marker,master_pose):
  print 'get_top_or_bot_blockface'
  frame = marker.header.frame_id
  blocknr = block_nr(marker)
  #master_pose = translate_frame(make_pose_stamped(marker.pose.pose.position,ori=marker.pose.pose.orientation,frame_id=frame), 'head')
  master_frame_id = block_master_frame_id(blocknr)
  # master currently to Rosie's right
  print 'marker frame',frame,'master pose',master_pose,'master_frame_id',master_frame_id
  master_frame = new_frame(master_pose, master_frame_id)
  center_pose = translate_frame(translate_pose_in_own_frame(master_pose,'center',0,0,-0.02),'head')
  ws = 0.02
  pi = math.pi
  qt = pi/2
  #side1_id = 'block'+str(blocknr)+'_side1'
  # face offset 1 in bundle (currently back)
  side1 = PoseStamped(header=Header(frame_id=master_frame_id, stamp=rospy.Time.now()),pose=Pose(position=Point(x=ws,y=0,z=-ws),orientation=quat_from_euler(0,qt,0)))
  side1_headpose = translate_frame(side1,'head')
  #side2_id = 'block'+str(blocknr)+'_side2'
  # face offset 4 in bundle (currently 'bot')
  side2 = PoseStamped(header=Header(frame_id=master_frame_id, stamp=rospy.Time.now()),pose=Pose(position=Point(0,-ws,-ws),orientation=quat_from_euler(0,qt,-qt)))
  side2_headpose = translate_frame(side2,'head')
  print 'master pose',master_pose
  print 'side1 pose',side1_headpose
  print 'side2 pose',side2_headpose
  marker0_topic.publish(master_pose)
  marker1_topic.publish(side1_headpose)
  marker2_topic.publish(side2_headpose)
  print 'rpy: master',euler_from_quat(master_pose.pose.orientation),'side1',euler_from_quat(side1_headpose.pose.orientation),'side2',euler_from_quat(side2_headpose.pose.orientation)
  mz = master_pose.pose.position.z
  s1z = side1_headpose.pose.position.z
  s2z = side2_headpose.pose.position.z
  #
  # find distinguished side - attempt 2 - find side with most different Z ordinate
  #
  dm1 = abs(mz - s1z)
  dm2 = abs(mz - s2z)
  d12 = abs(s1z - s2z)
  #
  # assume master
  mindiff = d12
  commonz = s1z
  other = master_pose
  otherz = mz
  # side 2
  if dm1 < mindiff:
      mindiff = dm1
      commonz = mz
      other = side2_headpose
      otherz = s2z
  # side 1
  if dm2 < mindiff:
      mindiff = dm2
      commonz = mz
      other = side1_headpose
      otherz = s1z
  if otherz == None:
      print 'WARNING: assertion failure'
  if otherz > commonz:
      print 'top',otherz,'vs',commonz
  else:
      print 'bot',otherz,'vs',commonz
  topbot = other
  return (topbot,center_pose)

def process_alvar(data):
  global avgmarkerpos
  global _target_marker_fn
  if data.markers:
      for marker in data.markers:
          frame = marker.header.frame_id
          #blocknr = int(marker.id / 10) + 1
          blocknr = block_nr(marker)
          if _target_marker_fn(marker): #frame != 'head_camera' and blocknr != 5:
              #if blocknr == target_cube:
              #print 'alvar marker:',marker.id,'blocknr',blocknr,'frame',frame
                #print '- pose',marker
              # Apparently we don't have enough time to do this before the next callback; offload to client
              ### (topbot,center_pose) = get_top_or_bot_blockface(marker)
              ### pose = center_pose
              ### ori = topbot.pose.orientation
              pose = translate_frame(make_pose_stamped(marker.pose.pose.position,ori=marker.pose.pose.orientation,frame_id=frame), 'head')
              ori = pose.pose.orientation
              pos = pose.pose.position
              quat = (ori.x,ori.y,ori.z,ori.w)
              # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
              (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(quat)
              framedict=init_key(avgmarkerpos,marker.id,{})
              d=init_key(framedict,frame,{})
              avgpos = init_key(d,'avg',pos)
              avgrpy = init_key(d,'avg_rpy',(roll,pitch,yaw))
              frame = init_key(d,'frame','head')
              err=init_key(d,'err',{})
              avgpos = Point(x = pairavg(avgpos.x, pos.x), y = pairavg(avgpos.y, pos.y), z = pairavg(avgpos.z, pos.z))
              avgrpy = (pairavg(avgrpy[0], roll), pairavg(avgrpy[1], pitch), pairavg(avgrpy[2], yaw))
              d['avg_master_pose']=PoseStamped(header=Header(frame_id='head', stamp=rospy.Time.now()),pose=Pose(position=avgpos,orientation=quat_from_euler(avgrpy[0],avgrpy[1],avgrpy[2])))
              d['avg'] = avgpos
              d['avg_rpy'] = avgrpy
              d['last_seen'] = rospy.get_time()
              d['marker'] = marker
              avgpos = d['avg']
              avgyaw = d['avg_rpy']
              #print '-','marker','avgpos',avgpos.x,avgpos.y,avgpos.z,'avg_rpy',avgrpy

global gripper

def init_gripper(mylimb):
    global gripper
    print 'init gripper'
    success = False
    while not success:
      try:
	gripper = baxter_interface.Gripper(mylimb, CHECK_VERSION)
	print 'gripper initialised'
	success = True
      except OSError:
	print 'OSError'
	success = False
    gripper.calibrate()
    return gripper

global avgpos
global avgyaw
avgpos = None
avgyaw = None

def getavgpos(target_marker_id):
    global avgpos
    global avgyaw
    d5 = {}
    d = {}
    print 'awaiting Alvar avg'
    rospy.sleep(3)
    if target_marker_id in avgmarkerpos:
        d5 = avgmarkerpos[target_marker_id]
    if 'head_camera' in d5:
        d = d5['head_camera']
    if 'right_hand_camera' in d5:
        d = d5['right_hand_camera']
    if 'left_hand_camera' in d5:
        d = d5['left_hand_camera']
    if 'avg' in d:
        avgpos = d['avg']
        avgyaw = d['avg_rpy'][2]
    print '- alvar','avg pos',avgpos,'avg yaw',avgyaw
    return (avgpos,avgyaw)

global node_name

def init(nodename='locallib', target_marker_fn=any_marker):
    global node_name
    node_name = nodename

    global _target_marker_fn
    _target_marker_fn = target_marker_fn
    print 'init node...'
    rospy.init_node(nodename, anonymous=True)
    print 'init node done'

    print 'init TF'
    global tf_buffer
    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    print 'init TF done'

    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, process_alvar, queue_size=1)
