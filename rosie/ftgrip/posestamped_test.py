#!/usr/bin/python
#script simply used to work out how the posestamped system worked.
import baxter_interface
import rospy
import tf
from std_msgs.msg import Header
from visualization_msgs.msg import Marker
from geometry_msgs.msg import (
    PoseStamped,
        Pose,
            Point,
                Quaternion,
                )

rospy.init_node("test4")

#append the information from pdict into pose
def pose_stamped(pdict):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id =  'reference/left_gripper'
        pose.pose.position.x = pdict['position'].x
        pose.pose.position.y = pdict['position'].y
        pose.pose.position.z = pdict['position'].z

        pose.pose.orientation.x = pdict['orientation'].x
        pose.pose.orientation.y = pdict['orientation'].y
        pose.pose.orientation.z = pdict['orientation'].z
        pose.pose.orientation.w = pdict['orientation'].w
        return pose 

'''
        pos:
          x: -0.158249041885
          y: -0.069605939989
          z: 0.203944316739
        orientation: 
          x: 0.635492567941
          y: 0.770855668572
          z: 0.0305417346856
          w: 0.0315901371407
'''
left = baxter_interface.Limb('left')
pdict=left.endpoint_pose()
pdict['position'] = Point(x= -0.158249041885, y= -0.069605939989, z= 0.203944316739)

pdict['orientiation'] = Quaternion( x= 0.635492567941,y=0.770855668572,z= 0.0305417346856,w= 0.0315901371407)


listener = tf.TransformListener()
rate = rospy.Rate(10.0)
listener.waitForTransform("/base", "/reference/left_hand_camera", rospy.Time(), rospy.Duration(4.0))
pose_dict=left.endpoint_pose()

pstamped = pose_stamped(pdict)
#pstamped = PoseStamped()
#pstamped.pose.postion = left.endpoint_pose()['position']
#pstamped.pose.orientation = left.endpoint_pose()['orientation']
#hdr = Header(stamp=rospy.Time.now(), frame_id='reference/left_hand_camera')
#pstamped.header = hdr
rospy.sleep(0.1)
print pstamped
posetf = listener.transformPose("/base", pstamped)
print posetf
posetf = listener.transformPose("/reference/left_hand_camera", pstamped)
print posetf
print Marker()
