#!/usr/bin/python
import sys

blocknr=int(sys.argv[1])
print 'blocknr',blocknr
objname='marker'+str(blocknr)

from gazebo_msgs.srv import GetWorldProperties
from gazebo_msgs.srv import GetModelState
from gazebo_msgs.srv import DeleteModel
import rospy
import random

try:
    get_world_properties = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
    props = get_world_properties()
    models = props.model_names
except rospy.ServiceException as e:
    print e

print 'delete?',(objname in models)

if objname in models:
    try:
	delete_model = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
	delete_model(objname)
    except rospy.ServiceException as e:
	print e

# [0...10cm]
x = 0.5 + 10 * (random.random() / 100)
# [-10 ... +10cm]
y = 0 + 10 * (2 * random.random() - 1) / 100
z = 0.79
yaw = random.random()

bashCommand = 'rosrun gazebo_ros spawn_model -sdf -file /root/rosie/markerblock_'+str(blocknr)+'/model.sdf -model marker'+str(blocknr)+' -x '+str(x)+' -y '+str(y)+' -z '+str(z)+' -Y '+str(yaw)
print bashCommand
import subprocess
cmd = bashCommand.split()
print cmd
process = subprocess.Popen(bashCommand.split(), stdout=subprocess.PIPE)
output, error = process.communicate()

print error,output
