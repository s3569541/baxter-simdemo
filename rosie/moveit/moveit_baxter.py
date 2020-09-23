#!/usr/bin/env python

# https://github.com/ros-planning/moveit_tutorials/blob/kinetic-devel/doc/move_group_python_interface/scripts/move_group_python_interface_tutorial.py
# https://github.com/ravijo/baxter_moveit_tutorial/blob/master/scripts/example.py

# Steps to run this code
# 1) roslaunch baxter_moveit_tutorial moveit_init.launch
# 2) rosrun baxter_moveit_tutorial example.py
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

import locallib

import socket
import math
import time
import Vectors
import struct
import threading
import Queue

import moveit_baxter

from ar_track_alvar_msgs.msg import AlvarMarkers
from gazebo_msgs.srv import GetModelState
#import tf

# for Euler
import tf.transformations
from tf.transformations import *

from trac_ik_python.trac_ik import IK

import tf2_ros
import tf2_geometry_msgs

# refer to http://sdk.rethinkrobotics.com/wiki/IK_Service_Example

from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        )
from std_msgs.msg import Header

global lgroup
global rgroup
lgroup = None, 
rgroup = None
global robot
global mylimb
mylimb = ''
global lgripper
global display_trajectory_publisher

def init():
    global lgroup
    global rgroup
    global robot
    global mylimb
    global display_trajectory_publisher

    mylimb = 'left'
    # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    # rospy.init_node('moveit_baxter_example', anonymous=True)
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                               moveit_msgs.msg.DisplayTrajectory,
                                               queue_size=20)

    robot = moveit_commander.RobotCommander()
    lgroup = moveit_commander.MoveGroupCommander("left_arm")
    rgroup = moveit_commander.MoveGroupCommander("right_arm")

    # group.set_max_velocity_scaling_factor(0.07)
    lgroup.set_max_acceleration_scaling_factor(0.01)
    rgroup.set_max_acceleration_scaling_factor(0.01)

    print 'calibrate / open gripper'
    global lgripper
    lgripper = locallib.init_gripper(mylimb)
    lgripper.calibrate()
    lgripper.open()
    rospy.sleep(1)
    print 'gripper open'

    # scene = moveit_commander.PlanningSceneInterface()
    # rospy.sleep(2)
    # scene.remove_world_object("table")

    print("~~~~~~init complete~~~~~~~~~~")

def locateBlock(marker):
    
    print "\nLocating marker: ", marker
    while(locallib.getavgpos('right_hand_camera', marker)==False):
        rospy.sleep(2)

    avgpos,avgyaw = locallib.getavgpos('right_hand_camera', marker)

    print "New Pos: ", avgpos, "\nYaw: ",avgyaw
    # mylimb =  'right'
    # newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    # pos = newPoseBase.pose.position
    # ori = quaternion_from_euler(3.14, 0, avgyaw)

    # print 'Scanning......'
    # move_arm(mylimb, pos.x, pos.y+0.05, pos.z+0.08, 0.521617116774, 0.474482274666, -0.513181385153, 0.489312804297)

    # print("Updating marker: ", marker, " position...", )
    # avgpos,avgyaw = locallib.getavgpos('right_hand_camera', marker)
    # newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    # pos = newPoseBase.pose.position
    # ori = quaternion_from_euler(3.14, 0, avgyaw)
    # move_arm(mylimb, pos.x, pos.y+0.06, pos.z+0.05, 0.521617116774, 0.474482274666, -0.513181385153, 0.489312804297)
    # print("Updating marker: ", marker, " position...")
    # avgpos,avgyaw = locallib.getavgpos('right_hand_camera', marker)

    return avgpos,avgyaw

def pick(target_marker_id):

    # Move to idea scan position
    # move_arm( mylimb, 0.6, 0.7, -0.1)
    move_arm('left', 0.65, 0.7, -0.1)

    global lgripper
    global mylimb

    while(not locallib.getavgpos('left_hand_camera', target_marker_id)):
        rospy.sleep(1)

    avgpos, avgyaw = locallib.getavgpos('left_hand_camera', target_marker_id)

    print '******* move to proper sighting pos ********'
    newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    pos = newPoseBase.pose.position
    ori = quaternion_from_euler(3.14, 0, avgyaw)

    move_arm(mylimb, pos.x, pos.y, pos.z+0.12)#, ori[0], ori[1], ori[2], ori[3])

    # terminate()

    rospy.sleep(2)
    avgpos,avgyaw = locallib.getavgpos('left_hand_camera', target_marker_id)

    newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    pos = newPoseBase.pose.position
    ori = quaternion_from_euler(3.14, 0, avgyaw)

    move_arm(mylimb, pos.x, pos.y, pos.z+0.07)#, ori[0], ori[1], ori[2], ori[3])
    # drop_arm(mylimb, pos.x, pos.y, pos.z+0.05)

    # terminate()

    rospy.sleep(2)
    avgpos,avgyaw = locallib.getavgpos('left_hand_camera', target_marker_id)

    print 'move to grab'
    newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    pos = newPoseBase.pose.position
    ori = quaternion_from_euler(3.14, 0, avgyaw)

    drop_arm(mylimb, pos.x, pos.y, pos.z+0.02)

    ### Close grippers
    print 'close (grab)'
    rospy.sleep(1.0)
    lgripper.close()
    print 'lift'
    raise_arm(0.1)

def place(marker, avgpos, avgyaw):
    global mylimb
    global lgripper

    newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    pos = newPoseBase.pose.position
    ori = quaternion_from_euler(3.14, 0, avgyaw)

    print 'Stack'
    move_arm(mylimb, pos.x, pos.y*-1, pos.z+0.04)#, ori[0], ori[1], ori[2], ori[3])

    rospy.sleep(1.0)
    lgripper.open()
    rospy.sleep(1.0)

    raise_arm(0.1)

def move_arm(limb, x, y, z, ox = 0.0, oy = 1.0, oz = 0.0, ow = 0.0):
    global lgroup
    global rgroup
    global robot
    global display_trajectory_publisher
    # Planning to a Pose goal
    # print "\n\nLeft gripper current state"
    # print rgroup.get_current_pose(end_effector_link='right_gripper')
    # terminate()
    
    if(limb == 'left'):
        left_current_pose = lgroup.get_current_pose(end_effector_link='left_gripper').pose
        left_target_pose = left_current_pose
        left_target_pose.position.x = x
        left_target_pose.position.y = y
        left_target_pose.position.z = z
        left_target_pose.orientation.x = ox
        left_target_pose.orientation.y = oy
        left_target_pose.orientation.z = oz
        left_target_pose.orientation.w = ow
        lgroup.set_pose_target(left_target_pose, end_effector_link='left_gripper')
        plan = lgroup.plan()
        if not plan.joint_trajectory.points:
            print "[ERROR] No trajectory found"
            # terminate()
        else:
            lgroup.go(wait=True)
            rospy.sleep(0.5)

    if(limb == 'right'):
        right_current_pose = rgroup.get_current_pose(end_effector_link='right_gripper').pose
        right_target_pose = right_current_pose
        right_target_pose.position.x = x
        right_target_pose.position.y = y
        right_target_pose.position.z = z
        right_target_pose.orientation.x = ox
        right_target_pose.orientation.y = oy
        right_target_pose.orientation.z = oz
        right_target_pose.orientation.w = ow
        rgroup.set_pose_target(right_target_pose, end_effector_link='right_gripper')
        plan = rgroup.plan()
        if not plan.joint_trajectory.points:
            print "[ERROR] No trajectory found"
            # terminate()
        else:
            rgroup.go(wait=True)
            rospy.sleep(0.5)

    # display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    # display_trajectory.trajectory_start = robot.get_current_state()
    # display_trajectory.trajectory.append(plan)
    # # Publish
    # display_trajectory_publisher.publish(display_trajectory);

def drop_arm(mylimb, x, y, z):
    # adapted from: 
    # https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#getting-started
    global lgroup
    global rgroup
    link = ''

    if mylimb == 'left':
        group = lgroup
        link = 'left_gripper'
    else:
        group = rgroup
        link = 'right_gripper'

    waypoints = []
    depth = 0.05

    wpose = group.get_current_pose(end_effector_link=link).pose

    wpose.position.x = x
    wpose.position.y = y
    wpose.position.z = z+depth
    for i in range(100):
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.z -= (depth/100)

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    # return plan, fraction 

    group.execute(plan, wait=True)

def raise_arm(height):
    # adapted from: 
    # https://ros-planning.github.io/moveit_tutorials/doc/move_group_python_interface/move_group_python_interface_tutorial.html#getting-started
    global lgroup
    waypoints = []

    wpose = lgroup.get_current_pose(end_effector_link='left_gripper').pose

    for i in range(100):
        wpose.position.z += height/100
        waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = lgroup.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01,        # eef_step
                                    0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    # return plan, fraction 

    lgroup.execute(plan, wait=True)

def update_scene(blocks):
    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    ## Needs to be constructed after robot and group, or "can't get joint states"
    scene = moveit_commander.PlanningSceneInterface()
    # Wait on group/scene?
    # FIXME: is there a proper way to do this
    rospy.sleep(2)
    scene.remove_world_object("table")
    scene.remove_world_object("box_1")
    scene.remove_world_object("box_2")
    scene.remove_world_object("box_3")

    #adding planning scene objects, initially to avoid collisions has resulted in error and issues and complexity in motion planning, 
    # seems to work better if you simply set way points to aid planning and collision avoidance

    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    ## Needs to be constructed after robot and group, or "can't get joint states"
    # scene = moveit_commander.PlanningSceneInterface()
    # # Wait on group/scene?
    # # FIXME: is there a proper way to do this
    # rospy.sleep(2)
    
    # scene.remove_world_object("table")
    # table = geometry_msgs.msg.PoseStamped()   
    # table.header.frame_id = "head"
    # table.pose.position.x =  1.08
    # table.pose.position.y = -0.00919116929443
    # table.pose.position.z = -1.3
    # table.pose.orientation.w = 1.0
    # box_name = "table"
    # scene.add_box(box_name, table, size=(1.5, 0.8, 0.82))
    # wait_for_state_update(obj_name='table',box_is_known=True)

    # rospy.sleep(2) 

    for block in blocks:
        newPoseBase = locallib.translate_frame(locallib.make_pose_stamped(blocks[block]['AvgPos'], 'head', locallib.return_yaw(blocks[block]['avgyaw'])), 'base')
        pos = newPoseBase.pose.position
        ori = newPoseBase.pose.orientation
        box_name = "box_"+str(block)
        box = geometry_msgs.msg.PoseStamped()   
        box.header.frame_id = "base"
        box.pose.position.x = pos.x
        box.pose.position.y = pos.y
        box.pose.position.z = pos.z
        box.pose.orientation.w = 1.0
        scene.add_box(box_name, box, size=(0.04, 0.04, 0.04))
        wait_for_state_update(obj_name=box_name,box_is_known=True)
    
def delete_scene():
    scene = moveit_commander.PlanningSceneInterface()
    scene.remove_world_object("box_1")
    scene.remove_world_object("box_2")
    scene.remove_world_object("box_3")

def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4, obj_name='box'):
    scene = moveit_commander.PlanningSceneInterface()
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

def terminate():
    global lgroup
    global rgroup
    # When finished shut down moveit_commander.
    print 'Robot controller terminated.'
    lgroup.stop()
    lgroup.clear_pose_targets()
    rgroup.stop()
    rgroup.clear_pose_targets()
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)