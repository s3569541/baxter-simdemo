#!/usr/bin/env python

import rospy
import actionlib
import argparse

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

DOOR_X = -0.25
DOOR_Y = 5.5
ORIENT_Z = 0.7
ORIENT_W = 0.7

parser = argparse.ArgumentParser(prog='move_to.py', description='Navigation to location')
#parser.add_argument("-p", '--pushButton', action="store_true", help="")
#args = parser.parse_args();

def movebase_client():

    client = actionlib.SimpleActionClient('/rosie/move_base',MoveBaseAction)
    rospy.loginfo("Waiting for server...")
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = DOOR_X
    goal.target_pose.pose.position.y = DOOR_Y
    goal.target_pose.pose.orientation.z = ORIENT_Z
    goal.target_pose.pose.orientation.w = ORIENT_W

    client.send_goal(goal)
    rospy.loginfo("Waiting for response...")
    result = client.wait_for_result()
    if not result:
        rospy.logerr("Action server not available!")
        rospy.signal_shutdown("Action server not available!")
    else:
        return client.get_result()

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_py', anonymous=True)
        result = movebase_client()
        if result:
            rospy.loginfo("Goal execution done")
        #    if args.pushButton:
        #         rospy.loginfo("Calling beerPusher.py to push the button...")
        #         beerPusher.main()
        else:
            rospy.loginfo("Goal execution unsuccessful!")
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")
