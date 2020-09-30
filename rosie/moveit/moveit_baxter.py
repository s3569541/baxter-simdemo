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
global robot
global lgripper
global rgripper
global display_trajectory_publisher

def init():
    global robot
    global lgroup
    global rgroup
    global display_trajectory_publisher

    lgroup = None
    rgroup = None

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
    trys = 5
    while lgroup == None and trys > 0:
        try:
            lgroup = moveit_commander.MoveGroupCommander("left_arm")
        except:
            print "Time out error inisialising moveit_commander('left_arm')"
            trys-=1
    while rgroup == None and trys > 0:
        try:
            rgroup = moveit_commander.MoveGroupCommander("right_arm")
        except:
            print "Time out error inisialising moveit_commander('right_arm')"
            trys-=1

    if rgroup == None or lgroup == None:
        print "Failed to initialise moveit_commander, please try again."
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)
    # this is necessaary to reduce number pf joint failures to increase precision of movements
    lgroup.set_max_acceleration_scaling_factor(0.5)
    rgroup.set_max_acceleration_scaling_factor(0.5)

    print 'calibrate / open left gripper'
    global lgripper
    lgripper = locallib.init_gripper('left')

    print("~~~~~~ init and calibration complete ~~~~~~~~~~")

def locateBlock(camera, marker):
    
    print "\nLocating marker:", marker, "from" , camera

    while(locallib.getavgpos(camera, marker)==False):
        rospy.sleep(0.5)
    
    avgpos,avgyaw = locallib.getavgpos(camera, marker)

    print "New Pos:\n", avgpos, "\nYaw:",avgyaw
    return avgpos,avgyaw

def pick(mylimb, target_marker_id):

    move_arm(mylimb, 0.65, 0.7, -0.1)
    print "Moving to scan position"

    global lgripper
    
    avgpos, avgyaw = locateBlock('left_hand_camera', target_marker_id)

    print '******* move to proper sighting pos ********'
    newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    pos = newPoseBase.pose.position
    ori = quaternion_from_euler(3.14, 0, avgyaw)

    move_arm(mylimb, pos.x, pos.y, pos.z+0.08)#, ori[0], ori[1], ori[2], ori[3])

    rospy.sleep(0.5)
    avgpos,avgyaw = locateBlock('left_hand_camera', target_marker_id)

    newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    pos = newPoseBase.pose.position
    ori = quaternion_from_euler(3.14, 0, avgyaw)

    # move_arm(mylimb, pos.x, pos.y, pos.z+0.07)#, ori[0], ori[1], ori[2], ori[3])
    drop_arm(mylimb, pos.x, pos.y, pos.z+0.07)

    rospy.sleep(0.5)
    avgpos,avgyaw = locateBlock('left_hand_camera', target_marker_id)

    print 'move to grab'
    newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    pos = newPoseBase.pose.position
    ori = quaternion_from_euler(3.14, 0, avgyaw)

    drop_arm(mylimb, pos.x, pos.y, pos.z+0.005)

    ### Close grippers
    print 'close (grab)'
    rospy.sleep(0.5)
    lgripper.close()
    print 'lift'
    raise_arm(0.1)
    return avgpos

def place(mylimb, avgpos, avgyaw):
    global lgripper

    newPoseBase = locallib.translate_frame(locallib.make_pose_stamped_yaw(avgpos, 'head', avgyaw), 'base')
    pos = newPoseBase.pose.position
    ori = quaternion_from_euler(3.14, 0, avgyaw)

    print 'Stack'
    move_arm(mylimb, pos.x, pos.y*-1, pos.z+0.04)#, ori[0]), ori[1], ori[2], ori[3])

    rospy.sleep(0.5)
    lgripper.open()
    rospy.sleep(0.5)

    raise_arm(0.1)

def move_arm(limb, x, y, z, ox = 0.0, oy = 1.0, oz = 0.0, ow = 0.0):
    global lgroup
    global rgroup
    global robot
    global display_trajectory_publisher
    # Planning to a Pose goal
    
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
        else:
            lgroup.go(wait=True)

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
        else:
            rgroup.go(wait=True)

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