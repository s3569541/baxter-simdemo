#! /usr/bin/env python

from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import GetModelState
import rospy

def show_gazebo_models():
    try:
        get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
        props = get_world_properties()
        for name in props.model_names:
            print "'"+name+"'"

    except rospy.ServiceException as e:
        print e

if __name__ == '__main__':
    show_gazebo_models()
