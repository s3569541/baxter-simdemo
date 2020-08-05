#!/usr/bin/env python

# https://answers.ros.org/question/79851/python-odometry/

import numpy as np
import rospy
import roslib
import tf
import PyKDL as kdl
import math
import json
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import Imu

import datetime

# Messages
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion

#roslib.load_manifest('odom_publisher')

class RosOdomPublisher:
  gps_ekf_odom_pub = rospy.Publisher('/rosie/odom', Odometry)
  tf_br = tf.TransformBroadcaster()

  publish_odom_tf = True

  frame_id = '/odom'
  child_frame_id = '/base_footprint'

  # Covariance
  P = np.mat(np.diag([0.0]*3))

  def __init__(self):
      self.x = 0
      self.y = 0
      self.yaw = 0
      # Attempt to read from file
#      try:
#        with open('/home/mb/base_position.json', 'r') as f:
#       	  print 'file',f
#          data = json.load(f)
#      	  print 'data',data
#          self.x = data['x']
#          self.y = data['y']
#          self.yaw = data['yaw']
#      except:
#        print 'No stored position, starting at 0,0 angle 0' 
      print 'No stored position, starting at 0,0 angle 0' 
      # (not used)
      self.z = 0
      self.roll = 0
      self.pitch = 0

  def publish_odom(self):
      msg = Odometry()
      msg.header.stamp = rospy.Time.now()
      msg.header.frame_id = self.frame_id # i.e. '/odom'
      msg.child_frame_id = self.child_frame_id # i.e. '/base_footprint'

      msg.pose.pose.position = Point(self.x, self.y, self.z)
      msg.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(self.roll, self.pitch, self.yaw).GetQuaternion()))

      p_cov = np.array([0.0]*36).reshape(6,6)

      # position covariance
      p_cov[0:2,0:2] = self.P[0:2,0:2]
      # orientation covariance for Yaw
      # x and Yaw
      p_cov[5,0] = p_cov[0,5] = self.P[2,0]
      # y and Yaw
      p_cov[5,1] = p_cov[1,5] = self.P[2,1]
      # Yaw and Yaw
      p_cov[5,5] = self.P[2,2]

      msg.pose.covariance = tuple(p_cov.ravel().tolist())

      pos = (msg.pose.pose.position.x,
             msg.pose.pose.position.y,
             msg.pose.pose.position.z)

      ori = (msg.pose.pose.orientation.x,
             msg.pose.pose.orientation.y,
             msg.pose.pose.orientation.z,
             msg.pose.pose.orientation.w)

      # Publish odometry message
      self.gps_ekf_odom_pub.publish(msg)

      # Write to file
      #with open('/home/mb/base_position.json', 'w') as f:
      #  json.dump({'x':self.x, 'y':self.y, 'yaw':self.yaw}, f)

      # Also publish tf if necessary
      if self.publish_odom_tf:
          self.tf_br.sendTransform(pos, ori, msg.header.stamp, msg.child_frame_id, msg.header.frame_id)

pub = RosOdomPublisher()

# Last time cmdvel was called
lastcmdvel = datetime.datetime.now()

# Previous twist message (0 on initialization)
lasttwist = Twist()

def mypub_cmdvel(msg):
  global lasttwist
  twist = msg.twist
  # Calculate elapsed time since last twist in microseconds
  global lastcmdvel
  newcmdvel = datetime.datetime.now()
  dt = (newcmdvel - lastcmdvel)
  micros = dt.microseconds
  #print 'twist',twist
  #print 'dt',dt.seconds,'new',newcmdvel,'last',lastcmdvel
  # Optimisation---normally skip intervals where velocity is zero, except to maintain TF every few seconds
  tf_timeout = dt.seconds > 1
  if tf_timeout:
    rospy.loginfo('TF timeout:')
  if not tf_timeout and lasttwist.linear.x==0 and lasttwist.linear.y==0 and lasttwist.angular.z==0 and twist.linear.x==0 and twist.linear.y==0 and twist.angular.z==0:
    # skip null twist messages
    #####rospy.loginfo('Odometry: skipping msg'+str(msg.twist))
    return
  #rospy.loginfo('Updating based on delta-Twist')
  lasttwist = twist
  lastcmdvel = newcmdvel

  # Calculate new position in terms of delta-Twist
  #pub.dx = twist.linear.x * 0.4
  #pub.dy = twist.linear.y * 0.4
  pub.dx = twist.linear.x * 0.4
  pub.dy = twist.linear.y * 0.4

  # MAGIC NUMBER
  #pub.dyaw = twist.angular.z / 1.55
  ######pub.dyaw = twist.angular.z * 0.35
  pub.dyaw = twist.angular.z * 0.4
  #pub.dyaw = twist.angular.z * 1.0

  c1 = (pub.dx * micros / 1000000)
  c2 = (pub.dy * micros / 1000000)

  pub.x = pub.x + c1 * math.cos(pub.yaw) + c2 * math.sin(pub.yaw)
  pub.y = pub.y + c1 * math.sin(pub.yaw) + c2 * math.cos(pub.yaw)

  # ROS: yaw increases in an anti-clockwise direction
  # (http://www.ros.org/reps/rep-0103.html)
  pub.yaw = pub.yaw + pub.dyaw * micros / 1000000
  #print 'yaw',pub.yaw
  #print 'dx',pub.dx,'dy',pub.dy,'dyaw',pub.dyaw
  #print 'x',pub.x,'y',pub.y,'z',pub.z,'yaw',pub.yaw
  pub.publish_odom()

rospy.init_node('dataspeed_base_odometry', anonymous=False)
#rospy.Subscriber('/mobility_base/cmd_vel', Twist, mypub_cmdvel)
rospy.Subscriber('/mobility_base/twist', TwistStamped, mypub_cmdvel)
#rospy.Subscriber('/mobility_base/imu/data_raw', Imu, mypub_imu)
rospy.loginfo("Mobility base odometry.py running")
while True:
  rospy.sleep(1.0)
rospy.loginfo("Mobility base odometry.py closing")
