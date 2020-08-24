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
import geometry_msgs.msg

def wait_for_state_update(box_is_known=False, box_is_attached=False, timeout=4, obj_name='box'):
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

def moveit_baxter_example():
    # initialize moveit_commander and rospy.
    joint_state_topic = ['joint_states:=/robot/joint_states']
    moveit_commander.roscpp_initialize(joint_state_topic)
    rospy.init_node('moveit_baxter_example', anonymous=True)

    # Instantiate a RobotCommander object.  This object is
    # an interface to the robot as a whole.
    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander("both_arms")
    ## Instantiate a `PlanningSceneInterface`_ object.  This object is an interface
    ## to the world surrounding the robot:
    ## Needs to be constructed after robot and group, or "can't get joint states"
    global scene
    scene = moveit_commander.PlanningSceneInterface()
    # Wait on group/scene?
    # FIXME: is there a proper way to do this
    rospy.sleep(2)

    box_pose = geometry_msgs.msg.PoseStamped()

    box_pose.header.frame_id = "head"
    #box_pose.pose.position.x =  1.12790490938
    box_pose.pose.position.x =  1.08
    box_pose.pose.position.y = -0.00919116929443
    box_pose.pose.position.z = -1.3
    box_pose.pose.orientation.w = 1.0
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(1.5, 0.8, 0.73))
    wait_for_state_update(obj_name='box',box_is_known=True)

    # Planning to a Pose goal
    left_current_pose = group.get_current_pose(end_effector_link='left_gripper').pose
    right_current_pose = group.get_current_pose(end_effector_link='right_gripper').pose

    left_target_pose = left_current_pose
    left_target_pose.position.x = left_current_pose.position.x - 0.1  # 0.1m = 10 cm
    left_target_pose.position.y = left_current_pose.position.y + 0.0  # 0.1m = 10 cm
    left_target_pose.position.z = left_current_pose.position.z + 0.0

    right_target_pose = right_current_pose
    right_target_pose.position.x = right_current_pose.position.x - 0.1
    right_target_pose.position.y = right_current_pose.position.y + 0.0
    right_target_pose.position.z = right_current_pose.position.z + 0.0

    group.set_pose_target(left_target_pose, end_effector_link='left_gripper')
    group.set_pose_target(right_target_pose, end_effector_link='right_gripper')

    plan = group.plan()

    if not plan.joint_trajectory.points:
        print "[ERROR] No trajectory found"
    else:
        group.go(wait=True)

    # When finished shut down moveit_commander.
    moveit_commander.roscpp_shutdown()
    moveit_commander.os._exit(0)


if __name__ == '__main__':
    try:
        moveit_baxter_example()
    except rospy.ROSInterruptException:
        pass

