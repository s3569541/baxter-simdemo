#!/usr/bin/env python

from trac_ik_python.trac_ik import IK
import rospy
import time

import baxter_interface


from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        )

import tf2_ros
import tf2_geometry_msgs

tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
rospy.init_node("tractest", anonymous=True)

tf_listener = tf2_ros.TransformListener(tf_buffer)

from std_msgs.msg import Header

#time.sleep(2)

global posedebug
posedebug = rospy.Publisher('/red/trac_eg/pose_debug', PoseStamped, queue_size=1)
global marker_topic
marker_topic = rospy.Publisher('/red/trac_eg/target_marker_pose', PoseStamped, queue_size=1)
global target_topic
target_topic = rospy.Publisher('/red/trac_eg/target_gripper_pose', PoseStamped, queue_size=1)

def frange(x, y, jump):
  while x < y:
    yield x
    x += jump

# Get your URDF from somewhere
urdf_str = rospy.get_param('/robot_description')

ik_solver = IK("right_arm_mount",
               "right_wrist",
               urdf_string=urdf_str)

seed_state = [0.0] * ik_solver.number_of_joints

#print ik_solver.number_of_joints

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

# ps: PoseStamped
def trac_ik_solve(limb, ps):
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

# ps: PoseStamped including header.frame_id
def solve_move_trac(limb,ps):
    print '*********************'
    print 'solve_move_trac',limb,ps
    time.sleep(1)
    target_topic.publish(ps)
    time.sleep(1)
    soln = trac_ik_solve(limb,ps)
    print 'soln',soln
    make_move_trac(soln, limb, 1.0)

def mkPS(pos,ori,frame):
    hdr = Header(stamp=rospy.Time.now(), frame_id=frame)
    return PoseStamped(
	header=hdr,
	pose=Pose(
	    position=pos,
	    orientation=ori
	)
    )

# Base: X towards viewer, Y to Baxter's left, Z up
or1=Quaternion(x = 0.0, y=1.0, z=0.0, w=0.5)
ori_su=Quaternion(x = 0.0, y=0.0, z=0.0, w=-1.0)
ori_sd=Quaternion(x = 0.0, y=1.0, z=0.0, w=0.0)
ori_so=Quaternion(x = 0.0, y=1.0, z=0.0, w=1.0)

# Baxter's front right
solve_move_trac('right',mkPS(Point(x=0.7,y=-0.5,z=0.0),ori_sd,'base'))
time.sleep(1)
solve_move_trac('right',mkPS(Point(x=0.8,y=-0.2,z=0.0),ori_sd,'base'))
time.sleep(1)
# above front left
solve_move_trac('right',mkPS(Point(x=0.4,y=-0.5,z=0.5),ori_su,'base'))
time.sleep(1)
# directly in front
solve_move_trac('right',mkPS(Point(x=0.5,y=0.0,z=0.0),ori_sd,'base'))
time.sleep(1)
# above in front
solve_move_trac('right',mkPS(Point(x=0.5,y=0.0,z=0.5),ori_so,'base'))
time.sleep(1)

#for xa in frange(0.5, 0.51, 0.5):
#    for ya in frange(0.5, 0.51, 0.5):
#        for za in frange(0.5, 0.51, 0.5):
#	    solve_move_trac('right',mkPS(Point(x=xa,y= - ya,z=za),or1,'base'))

print 'finished'
rospy.spin()
