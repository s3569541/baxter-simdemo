#!/usr/bin/python

import math
import rospy
import baxter_interface
import tf2_ros, tf2_geometry_msgs
from std_msgs.msg import String, Header
from ar_track_alvar_msgs.msg import AlvarMarkers
from trac_ik_python.trac_ik import IK
from tf.transformations import *

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion
    )

from std_msgs.msg import Header

def set_speed(limb, speed):
    arm = baxter_interface.Limb(limb)
    arm.set_joint_position_speed(speed)

if __name__ == "__main__":
    rospy.init_node("set_speed", anonymous=True)
    speed = 2.0
    set_speed("left", speed)
    set_speed("right", speed)
