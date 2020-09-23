#!/usr/bin/env python

# https://github.com/ravijo/baxter_moveit_tutorial/blob/master/scripts/example.py

# Steps to run this code
# 1) roslaunch baxter_moveit_tutorial moveit_init.launch
# 2) rosrun baxter_moveit_tutorial example.py

import sys
import copy
import rospy
import moveit_commander
import geometry_msgs.msg

import baxter_interface
from   baxter_interface import CHECK_VERSION

import baxter_external_devices

# for Euler
import tf.transformations
from tf.transformations import *

from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        )
from std_msgs.msg import Header

import tf2_ros
import tf2_geometry_msgs

from ar_track_alvar_msgs.msg import AlvarMarkers

# initialize moveit_commander and rospy.
joint_state_topic = ['joint_states:=/robot/joint_states']
moveit_commander.roscpp_initialize(joint_state_topic)
rospy.init_node('moveit_baxter_example', anonymous=True)

# Instantiate a RobotCommander object.  This object is
# an interface to the robot as a whole.
robot = moveit_commander.RobotCommander()
group = moveit_commander.MoveGroupCommander("both_arms")
group.set_max_velocity_scaling_factor(0.07)

## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
## to the world surrounding the robot:
## Needs to be constructed after robot and group, or "can't get joint states"
global scene
scene = moveit_commander.PlanningSceneInterface()

print 'init TF'
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)
print 'init TF done'

global lgripper
global rgripper

global avgmarkerpos
global target_cube
global avgpos
global avgyaw

global d5
global d
global target_marker_id
global pos

def main():
    global avgpos
    global avgyaw
    global target_cube
    global avgmarkerpos

    global pos
    global lgripper

    target_cube = 3
    global target_object
    target_object = 'marker'+str(target_cube)
    global target_marker_id
    target_marker_id = 25

    # wait_for_state_update(obj_name='box',box_is_known=True)

    avgmarkerpos = {}

    # print 'calibrate / open gripper'
    # init_gripper()

    # rospy.Subscriber('/ar_pose_marker', AlvarMarkers, marker_callback, queue_size=1)
    
    avgpos = None
    avgyaw = None
    pos = None
    d5 = {}
    d = {}

    # while avgpos == None:
    #     rospy.sleep(2)
    #     getavgpos() 
    #     print pos, "!!!!!!!!"
    # while pos == None:
    #     rospy.sleep(2)
    
    

    # coord = avgpos
    # moveit_baxter_example('left', 0.6, 0.3, 0.1)
    # rospy.sleep(2)
    moveit_baxter_example('left', 0.8, 0.3, 0.1)
    # rospy.sleep(2)

    # coord = pos
    # print coord
    # print "~~~~~~~~~~~~~~~~"

    # moveit_baxter_example('left', coord.x, coord.y+0.02, coord.z+0.02)
    # rospy.sleep(1)
    # lgripper.close()
    # rospy.sleep(1)
    # moveit_baxter_example('left', 0.6, 0.1, 0.1)
    # moveit_baxter_example('left', 0.8, 0.1, pos.z+0.04)
    # lgripper.open()
    # moveit_baxter_example('left', 0.6, 0.1, 0.1)
    # moveit_baxter_example('right', 0.6, -0.1, 0.1)

    # moveit_baxter_example('left', pos.x, pos.y, pos.z)

    # When finished shut down moveit_commander.
    # moveit_commander.roscpp_shutdown()
    # moveit_commander.os._exit(0)

def init_gripper():
    global lgripper
    global rgripper
    print 'init gripper'
    lgripper = baxter_interface.Gripper('left', CHECK_VERSION)
    rgripper = baxter_interface.Gripper('right', CHECK_VERSION)
    lgripper.calibrate()
    rgripper.calibrate()
    lgripper.close()
    rgripper.close()
    rospy.sleep(1)
    lgripper.open()
    rgripper.open()
    rospy.sleep(1)
    print 'gripper open'
    print 'gripper initialised'

# ensure that dict contains key by priming with val if necessary
# return dict[key]
def init_key(dict,key,val):
  if not (key in dict):
    dict[key]=val
  return dict[key]

def pairavg(a,b):
    return (a + b) / 2

def getavgpos():
    global avgmarkerpos
    global avgpos
    global avgyaw
    global d5
    global d
    global target_marker_id
    print 'awaiting Alvar avg'
    rospy.sleep(3)
    # if target_marker_id in avgmarkerpos:
    #     d5 = avgmarkerpos[target_marker_id]
    # if 'right_hand_camera' in d5:
    #     d = d5['right_hand_camera']
    # if 'left_hand_camera' in d5:
    #     d = d5['left_hand_camera']
    # if 'avg' in d:
    #     avgpos = d['avg']
    #     avgyaw = d['avgyaw']
    # print '- alvar avg pos',avgpos,'yaw',avgyaw #,'truepos',truepos,'trueyaw',trueyaw

def process_alvar(data):
  global avgmarkerpos
  global target_cube
  global pos
  if data.markers:
      for marker in data.markers:
          frame = marker.header.frame_id
          blocknr = int(marker.id / 10) + 1
          # process only "small" grab-able blocks
        #   print 'marker',marker.id,'blocknr',blocknr,'frame',frame
          if frame != 'head_camera' and blocknr != 5:
              if blocknr == target_cube:
                print 'marker',marker.id,'blocknr',blocknr,'frame',frame
                pose = translate_frame(make_pose_stamped_yaw(marker.pose.pose.position,frame), 'base')
                pos = pose.pose.position
                print pos
                # ori = marker.pose.pose.orientation
                # quat = (ori.x,ori.y,ori.z,ori.w)
            #   # https://answers.ros.org/question/69754/quaternion-transformations-in-python/
            #   (roll,pitch,yaw) = tf.transformations.euler_from_quaternion(tquat)
            #   yaw = 
            #   framedict=init_key(avgmarkerpos,marker.id,{})
            #   d=init_key(framedict,frame,{})
            #   avgpos = init_key(d,'avg',pos)
            #   avgyaw = init_key(d,'avgyaw',ori.w)
            #   err=init_key(d,'err',{})
            #   d['avg'] = Point(x = pairavg(avgpos.x, pos.x), y = pairavg(avgpos.y, pos.y), z = pairavg(avgpos.z, pos.z))
            #   d['avgyaw'] = pairavg(avgyaw, ori.w)
            #   avgpos = d['avg']
            #   avgyaw = d['avgyaw']
            #   campos = translate_frame(make_pose_stamped(marker.pose.pose.position,frame), frame).pose.position

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

def make_pose_stamped(pos,frame_id='base', ori=Quaternion(x=0, y=1, z=0, w=0)):
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=makepose(pos, ori),
  )

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

def make_pose_stamped_yaw(pos, frame_id='base', yaw=0):
  # yaw pitch roll
  print 'make_pose_stamped yaw',yaw
  ori = quaternion_from_euler(yaw, -0.0, 3.14, 'rzyx')
  print 'ori',ori
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=makepose(pos, Quaternion(x = ori[0], y=ori[1], z=ori[2], w=ori[3])),
  )

def marker_callback(data):
  global avgmarkerpos
  global target_cube
  global truepos
  global trueroll, truepitch, trueyaw
  process_alvar(data)

def moveit_baxter_example(limb, x, y, z):

    # Planning to a Pose goal
    left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    right_current_pose = group.get_current_pose(end_effector_link='right_gripper').pose

    #initialise taget pose variables
    left_target_pose = left_current_pose
    right_target_pose = right_current_pose

    if limb == 'left':
        left_target_pose.position.x = left_current_pose.position.x + 0.4
        left_target_pose.position.y = left_current_pose.position.y
        left_target_pose.position.z = left_current_pose.position.z
        group.set_pose_target(left_target_pose, end_effector_link='left_gripper')

    if limb == 'right':
        right_target_pose.position.x = x
        right_target_pose.position.y = y
        right_target_pose.position.z = z
        group.set_pose_target(right_target_pose, end_effector_link='right_gripper')
    
    plan = group.plan()

    if not plan.joint_trajectory.points:
        print "[ERROR] No trajectory found"
    else:
        group.go(wait=True)

def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4, obj_name='box'):
    
    # Wait on group/scene?
    
    global scene
    box_pose = geometry_msgs.msg.PoseStamped()

    box_pose.header.frame_id = "head"
    #box_pose.pose.position.x =  1.12790490938
    box_pose.pose.position.x =  1.08
    box_pose.pose.position.y = -0.00919116929443
    box_pose.pose.position.z = -1.3
    box_pose.pose.orientation.w = 1.0
    box_name = "box"

    scene.add_box(box_name, box_pose, size=(1.5, 0.8, 0.73))

    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([obj_name])
      is_attached = len(attached_objects.keys()) > 0
      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = obj_name in scene.get_known_object_names()
      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True
      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()
    # If we exited the while loop without returning then we timed out
    return False


if __name__ == '__main__':
    try:
        main()
        # moveit_baxter_example()
    except rospy.ROSInterruptException:
        pass

