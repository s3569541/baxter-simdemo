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

#import tf

# for Euler
from tf.transformations import *

from trac_ik_python.trac_ik import IK

import random

import tf2_ros
import tf2_geometry_msgs

print 'init node...'
rospy.init_node("marker_ik_example")
print 'init node done'

print 'init TF'
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
print 'init TF done'

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
global marker_topic
marker_topic = rospy.Publisher('/red/ikeg/target_marker_pose', PoseStamped, queue_size=1)
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

# give PoseStamped ps in frame-of-refernce frame
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

##################################
# TRAC IK solver for Baxter
##################################

# make search start position from current joint states
def make_seed(limb):
        arm = baxter_interface.Limb(limb)
        state = arm.joint_angles()
        print 'solve current:',arm.joint_angles()
        jointnames = ['s0','s1','e0','e1','w0','w1','w2']
        seed = []
        for i in range(0, len(jointnames)):
            key = limb+"_"+jointnames[i]
            seed.append(state[key])
        return seed

# ps: PoseStamped
def trac_ik_solve(limb, ps):
	time.sleep(1)
	target_topic.publish(ps)
	time.sleep(1)
        command = make_seed(limb)
	print 'candidate seed',command
        local_base_frame = limb+"_arm_mount"
        ik_solver = IK(local_base_frame,
                       #limb+"_wrist",
                       limb+"_gripper",
                       urdf_string=urdf_str)
        #seed_state = [0.0] * ik_solver.number_of_joints
        seed_state = command
        # canonical pose in local_base_frame
        #hdr = Header(stamp=rospy.Time.now(), frame_id=from_frame)
        #ps = PoseStamped(
        #        header=hdr,
        #        pose=pose,
        #        )
	#gripper_ps = translate_in_frame(ps,'right_wrist',0,0,0)
	#gripper_ps = translate_pose_in_own_frame(ps,'gripper_target',0.015,-0.02,-0.2)
	gripper_ps = translate_pose_in_own_frame(ps,'gripper_target',0,0,0.05)
        p = translate_frame(gripper_ps,local_base_frame)
        #print 'translated frame',p
        soln = ik_solver.get_ik(seed_state,
                        p.pose.position.x,p.pose.position.y,p.pose.position.z,  # X, Y, Z
                        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w,  # QX, QY, QZ, QW
                        0.01,0.01,0.01,
                        0.1,0.1,0.1,

        )
        print 'trac soln',soln
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
    print 'make_move_trac',msg
    SPEED_SCALE = 1
    speed = speed * SPEED_SCALE
    arm = baxter_interface.Limb(limb)
    print 'arm',arm
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
    print '*********************'
    print 'solve_move_trac',limb,ps
    soln = trac_ik_solve(limb,ps)
    if not soln:
	print '***** NO SOLUTION ******',soln
    else:
        print 'soln',soln
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

def clamp(p, inner, outer):
    if p.x() > outer.x():
        p = Vectors.V4D(outer.x(), p.y(), p.z(), p.w())
    elif p.x() < inner.x():
        p = Vectors.V4D(inner.x(), p.y(), p.z(), p.w())
    if p.y() > outer.y():
        p = Vectors.V4D(p.x(), outer.y(), p.z(), p.w())
    elif p.y() < inner.y():
        p = Vectors.V4D(p.x(), inner.y(), p.z(), p.w())
    if p.z() > outer.z():
        p = Vectors.V4D(p.x(), p.y(), outer.z(), p.w())
    elif p.z() < inner.z():
        p = Vectors.V4D(p.x(), p.y(), inner.z(), p.w())
    if p.w() > outer.w():
        p = Vectors.V4D(p.x(), p.y(), p.z(), outer.w())
    elif p.w() < inner.w():
        p = Vectors.V4D(p.x(), p.y(), p.z(), inner.w())

    return p



def jitter():
    return random.uniform(-0.05, 0.05)

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

# turn PoseStamped of a marker into target gripper pose
# assuming marker is 'level'
# was "hack_pose"
def target_from_marker(ps):
    #print 'hack_pose',ps
    quaternion = (
	ps.pose.orientation.x,
	ps.pose.orientation.y,
	ps.pose.orientation.z,
	ps.pose.orientation.w)
    # Leroy's guess
    #q_rot = quaternion_from_euler(math.pi/2, math.pi, 0.0, 'rzyx')
    q_rot = quaternion_from_euler(0.0, math.pi, math.pi/2)
    #q_rot1 = quaternion_from_euler(0.0, 0.0, math.pi, 'rzyx')
    #q_rot = quaternion_from_euler(math.pi/2, 0.0, 0.0, 'rzyx')
    #q_new = quaternion_from_euler(0, 0, 0)
    #q_new = quaternion
    #q_new = quaternion_multiply(q_rot, quaternion)
    q_new = quaternion_multiply(quaternion, q_rot)
    pose_rot = PoseStamped(
	header=ps.header,
	pose=Pose(
            position=Point(
                x=ps.pose.position.x,
                y=ps.pose.position.y,
                z=ps.pose.position.z
                ),
            orientation=normalize(Quaternion(
		x=q_new[0],
		y=q_new[1],
		z=q_new[2],
		w=q_new[3]
            ))
	    # This won't work in camera frame
	    #orientation=Quaternion(
            #                 x=-0.0249590815779,
            #                 y=0.999649402929,
            #                 z=0.00737916180073,
            #                 w=0.00486450832011)
    ))
    return pose_rot

def right_arm(pos):
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x(),
                y=pos.y(),
                z=pos.z(),
                ),
            orientation=Quaternion(
                x=0,
                y=1,
                z=0,
                w=0.5
                ),
            )
    return pose_right


def makepose(pos, orientation=Quaternion(x=0, y=1, z=0, w=0)):
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x(),
                y=pos.y(),
                z=pos.z(),
                ),
	    orientation=orientation,
            #orientation=Quaternion( x=0, y=1, z=0, w=0), # straight up and down
            #orientation=Quaternion( x=0.0462177008579, y=0.889249134439, z=0.0227669346795, w=0.454512450557), # toed-in for right camera
            )
    return pose_right

def make_pose_stamped(pos,frame_id='base', orientation=Quaternion(x=0, y=1, z=0, w=0)):
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=makepose(pos, orientation),
  )

#time.sleep(2)

###
### Example proper
###

print 'init gripper'
mylimb = 'left'

success = False
while not success:
  try:
    gripper = baxter_interface.Gripper(mylimb, CHECK_VERSION)
    print 'gripper initialised'
    success = True
  except OSError:
    print 'OSError'
    success = False

rospy.sleep(3.0)
gripper.close()
#gripper.calibrate()
gripper.open()
##gripper.calibrate()

solve_move_trac(mylimb, make_pose_stamped(Vectors.V4D(0.47, -0.0, 0.0, 0), frame_id='base'))
solve_move_trac(mylimb, make_pose_stamped(Vectors.V4D(0.47, -0.0, -0.1, 0), frame_id='base'))
solve_move_trac(mylimb, make_pose_stamped(Vectors.V4D(0.47, -0.0, -0.2, 0), frame_id='base'))

gripper.close()

solve_move_trac(mylimb, make_pose_stamped(Vectors.V4D(0.47, -0.01, 0.6, 0), frame_id='base'))

#while True:
#    rospy.sleep(1.0)
