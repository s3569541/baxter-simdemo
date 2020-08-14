#!/usr/bin/python

import sys

from gazebo_msgs.srv import GetModelState
import rospy

def show_gazebo_models(blockName, relativeName):
    try:
        model_coordinates = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        print("getcoords",blockName, relativeName)
        resp_coordinates = model_coordinates(blockName, relativeName)
        print 'Status.success = ', resp_coordinates.success
        print("pose: " + str(resp_coordinates.pose.position))
        print("orientation: " + str(resp_coordinates.pose.orientation))

    except rospy.ServiceException as e:
        rospy.loginfo("Get Model State service call failed:  {0}".format(e))


if __name__ == '__main__':
    show_gazebo_models(sys.argv[1], sys.argv[2])
