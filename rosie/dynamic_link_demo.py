#!/usr/bin/env python

# ~/gazebo_link_attacher_ws/src/gazebo_ros_link_attacher/scripts/attach.py

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


if __name__ == '__main__':
    rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    rospy.loginfo("Attaching cube1 and cube2")
    req = AttachRequest()
    req.model_name_1 = "mobility_base"
    req.link_name_1 = "base_footprint"
    req.model_name_2 = "cafe_table2"
    req.link_name_2 = "link"

    attach_srv.call(req)
    # From the shell:
    """
rosservice call /link_attacher_node/attach "model_name_1: 'cube1'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"
    """
