#!/usr/bin/python
#testing of how the marker subscriber/listener system works
import rospy
from ar_track_alvar_msgs.msg import AlvarMarkers

def callback(data):
    for marker in data.markers:
        print marker.pose.pose
    
def listener():
    rospy.init_node('node_name')
    rospy.Subscriber("/ar_pose_marker", AlvarMarkers, callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

listener()
rospy.spin()
