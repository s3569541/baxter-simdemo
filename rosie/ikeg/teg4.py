#!/usr/bin/python

# Use feedback from gripper camera to guide pickup
# - some issues with feedback loops in z axis

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

tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

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

rospy.init_node("marker_ik_example")
#time.sleep(2)

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

#global gframe_name
#global gframe_parent
#def frame_pub():
#thread = threading.Thread(target=publisher)
#thread.start()

global gripper
gripper = baxter_interface.Gripper('left', CHECK_VERSION)
gripper.calibrate()
#gripper.close()
gripper.open(block=True)
#gripper.calibrate()

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

global mylimb
mylimb = 'left'

# ps: PoseStamped
def trac_ik_solve(limb, ps, seed_as_posn=True,posn_tol=0.01):
	arm = baxter_interface.Limb(limb)
	state = arm.joint_angles()
	print 'solve current:',arm.joint_angles()
	jointnames = ['s0','s1','e0','e1','w0','w1','w2']
        command = []
	for i in range(0, len(jointnames)):
	    key = limb+"_"+jointnames[i]
	    command.append(state[key])
	print 'candidate seed',command
        local_base_frame = limb+"_arm_mount"
        ik_solver = IK(local_base_frame,
                       #limb+"_wrist",
                       limb+"_gripper",
                       urdf_string=urdf_str)
	if seed_as_posn:
	    seed_state = command
	else:
	    seed_state = [0.0] * ik_solver.number_of_joints
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
                        posn_tol,posn_tol,posn_tol,
                        0.1,0.1,0.1,

        )
        #print 'trac soln',soln
        return soln

# Get your URDF from somewhere
urdf_str = rospy.get_param('/robot_description')

def ik_solve(limb,pose,frame):
    #baxter_ik_solve(limb,pose,frame)
    return trac_ik_solve(limb,pose,frame)

def baxter_ik_solve(limb, pose, frame):
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    #hdr = Header(stamp=rospy.Time.now(), frame_id='reference/right_hand_camera')
    hdr = Header(stamp=rospy.Time.now(), frame_id=frame)
    ps = PoseStamped(
            header=hdr,
            pose=pose,
            )
    global posedebug
    target_topic.publish(ps)
    ikreq.pose_stamp.append(ps)
    try:
        #print 'ik_solve: waiting...'
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1
    #print 'ik_solve: done...'

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
            resp.result_type)
    if (resp_seeds[0] != resp.RESULT_INVALID):
        seed_str = {
                ikreq.SEED_USER: 'User Provided Seed',
                ikreq.SEED_CURRENT: 'Current Joint Angles',
                ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                }.get(resp_seeds[0], 'None')
        #print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" % (seed_str,))
        # Format solution into Limb API-compatible dictionary
        limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
        print 'ik_solve: solution found...',limb_joints
        return resp
    else:
        print("INVALID POSE - No Valid Joint Solution Found.")
        return None

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
    #print 'make_move_trac',msg
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
    arm.move_to_joint_positions(command,timeout=5.0)
    print 'make_move: done'

# move limb-gripper to given PoseStamped (absolute)
# limb: 'left'|'right'
# ps: PoseStamped including header.frame_id
def solve_move_trac(limb,ps):
    #print '*********************'
    #print 'solve_move_trac',limb,ps
    print 'solve_move_trac: (solve) ',limb
    #time.sleep(1)
    #target_topic.publish(ps)
    #time.sleep(1)
    soln = trac_ik_solve(limb,ps,posn_tol=0.005)
    if not soln:
	print '***** solve_move_trac no soln, retry with zero seed ******',soln
	soln = trac_ik_solve(limb,ps,seed_as_posn=False)
	if not soln:
	    print '***** solve_move_trac: NO SOLUTION ******',soln
	    return False
    else:
        print 'solve_move_trac: soln',soln
    print 'solve_move_trac: (move) ',limb
    make_move_trac(soln, limb, 0.8)
    return True

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


def pose2(pos):
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
                w=0
                ),
            )
    return pose_right

#time.sleep(2)

init_pos = Vectors.V4D(0.8,
        -0.47,
        0.2, 0)

bound = Vectors.V4D(0.656982770038,
        -0.252598021641,
        0.5388609422173, 0)

init_pos2 = Vectors.V4D(0.656982770038,
        -0.35,
        0.1, 0)

init_pos3 = Vectors.V4D(0.656982770038,
        -0.35,
        0.4, 0)

# this works on command line but not in rospy!?
# rostopic pub /red/pose_debug geometry_msgs/PoseStamped '{header: {stamp: now, frame_id: "map"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}'
#time.sleep(2)
myps = PoseStamped(
			header=Header(stamp=rospy.Time.now(), frame_id='base'),
			#pose=Pose(position=Point(1.0,0.0,0.0),orientation=Quaternion(0.0,0.0,0.0,1.0))
			pose=pose2(init_pos),
)
#time.sleep(2)
#print 'posedebug test'
#print 'posedebug test done'

global moved
moved = False

time.sleep(1)
posedebug.publish(myps)
time.sleep(1)

print 'initialisation move...'
#resp = trac_ik_solve(mylimb, myps)
#if resp is not None:
#    print 'moving...'
#    make_move_trac(resp, mylimb, 0.4)
#else:
#    print 'IK error'
#time.sleep(1)
#print 'initialisation move...'

global markerglobal
markerglobal = None

global poseglobal
poseglobal = None

global since_left
since_left = 0

# only return right pose after <right_threshold> right poses since last left pose
def callback(data):
  right_threshold = 10
  global gripper
  global moved
  global posedebug
  global since_left
  foundmarker = False
  global poseglobal
  global markerglobal
  if not moved:
    if data.markers:
      for marker in data.markers:
        if marker.id != 255 and marker.id != 0:
          #if not foundmarker:
		#print 'markers:',
          foundmarker = True
	  #print marker.id,
	  if (marker.id == 2 or marker.id == 4 or marker.id == 5 or marker.id == 13 or marker.id == 14 or marker.id == 10):
	      #print 'found marker',marker.id,marker.header.frame_id
              # check pose is suitable i.e. z axis is pointing upward in base frame...
	      marker_pose = marker.pose.pose
              ps = PoseStamped(
			  header=marker.header,
			  pose=marker.pose.pose,
		      )
	      # marker pose in base_link frame
 	      psbl = translate_frame(ps,'base_link')
	      orientation_q = psbl.pose.orientation
    	      orientation_l = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    	      (roll,pitch,yaw) = euler_from_quaternion(orientation_l)
	      #print ' rpy ',roll,pitch,yaw,marker.header.frame_id
	      if abs(roll)<0.25 and abs(pitch)<0.25:
		  #print '*** roughly level ',roll,pitch,' ***'
		  marker_topic.publish(ps)
		  if marker.header.frame_id == 'reference/left_hand_camera':
			  print 'left'	
			  poseglobal = target_from_marker(ps)
			  markerglobal = marker
			  since_left = 0
		  else:
			  since_left = since_left + 1
			  if since_left > right_threshold:
				  print 'right'
				  poseglobal = target_from_marker(ps)
				  since_left = 0
              #else:
	          #print 'not level',roll,pitch
  if not foundmarker:
      poseglobal = None
		  
def pickup():
    global poseglobal
    global markerglobal
    gripper.open(block=True)
    pose = poseglobal
    if pose == None:
        return
    print 'move to hover...'
    abovepose = lift_in_base_frame(pose,0.08)
    resp = solve_move_trac('left', abovepose)
    if not resp:
        print "can't go to hover"
        return
    time.sleep(0.8)
    # this pose is important because it's the last time we see the marker before pickup move
    pose = poseglobal
    if pose == None:
        print "lost block before pickup"
	return
    print 'do pre-grab...'
    pregrabpose = lift_in_base_frame(pose,0.05)
    finalpose = lift_in_base_frame(pose,0.15)
    desc = False
    while True:
        m = markerglobal
        if m!= None and m.header.frame_id == 'reference/left_hand_camera':
	    marker_pose = m.pose.pose
	    ps = PoseStamped(
			header=m.header,
			pose=m.pose.pose,
		    )
            mg = translate_frame(ps,'left_gripper')
            # gripper model in URDF is longer than actual gripper
	    ztarget = -0.02
	    # for mg.pose.position: x,y target is 0. z target is "ztarget"
	    if (
		abs(mg.pose.position.x)>0.4 and
		abs(mg.pose.position.y)>0.4
	        ):
		print '...servo abort'
		resp = solve_move_trac('left', abovepose)
		return
	    if (
		abs(mg.pose.position.x)<0.01 and
		abs(mg.pose.position.y)<0.01
	        ):
		print '...move down to grab'
                desc = True
	    if (
		abs(mg.pose.position.x)<0.015 and
		abs(mg.pose.position.y)<0.015 and
		abs(mg.pose.position.z - ztarget)<0.02
	        ):
		print '...grab at',mg.pose.position
                break
            print "*** servo, with target ",mg.pose.position
	    if desc:
                iztarget = mg.pose.position.z + ztarget
            else:
	        iztarget = 0
            fixedpregrabpose = translate_in_frame(pregrabpose,'left_gripper', mg.pose.position.x,  mg.pose.position.y, iztarget)
	    resp = solve_move_trac('left', fixedpregrabpose)
            # one attempt only to descend to avoid vertical inaccuracy with marker recognition!
	    if not resp:
	        print "servo failed"
	    	return
    time.sleep(0.5)
    gripper.close(block=True)
    time.sleep(0.5)
    resp = solve_move_trac('left', finalpose)
    time.sleep(0.5)
    gripper.open(block=True)
    if not resp:
        print "can't go to drop"
	return

rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback, queue_size=1)

#time.sleep(240)
#rospy.spin()

while True:
    pickup()
