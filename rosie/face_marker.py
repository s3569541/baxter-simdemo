#!/usr/bin/env python

# Software License Agreement (BSD License)
#
# Copyright (c) 2014-2017, Dataspeed Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
#     * Redistributions of source code must retain the above copyright notice,
#       this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright notice,
#       this list of conditions and the following disclaimer in the documentation
#       and/or other materials provided with the distribution.
#     * Neither the name of Dataspeed Inc. nor the names of its
#       contributors may be used to endorse or promote products derived from this
#       software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist

import locallib

def timer_callback(event):
  pub.publish(twist_output)

def motion(twist, duration):
  global twist_output
  #
  if rospy.is_shutdown():
    return
  #
  twist_output = twist
  rospy.sleep(duration)
  twist_output = zero_twist
  rospy.sleep(0.5)

def simple_motion(twist, duration):
  global twist_output
  #
  opposite_twist = Twist()
  opposite_twist.linear.x = -twist.linear.x
  opposite_twist.linear.y = -twist.linear.y
  opposite_twist.angular.z = -twist.angular.z
  #
  if rospy.is_shutdown():
    return
  #
  twist_output = twist
  rospy.sleep(duration)
  twist_output = zero_twist
  rospy.sleep(1.0)
  twist_output = opposite_twist
  rospy.sleep(duration)
  twist_output = zero_twist
  rospy.sleep(1.0)

def avgpos():
  l1 = locallib.avgmarkerpos
  l2 = None
  if 33 in l1:
      l2 = l1[33]['head_camera']
  if 35 in l1:
      l2 = l1[35]['head_camera']
  if l2 == None:
      return None
  return l2['avg'] 

def motion_demo():
  global pub
  pub = rospy.Publisher('/mobility_base/cmd_vel', Twist, queue_size=1)
  #rospy.init_node('motion_demo');
  #
  global zero_twist
  zero_twist = Twist()
  zero_twist.linear.x = 0
  zero_twist.linear.y = 0
  zero_twist.angular.z = 0
  #
  global twist_output
  twist_output = Twist()
  #
  rospy.Timer(rospy.Duration(0.1), timer_callback)
  #

  speed = 0.1

  ccw = Twist()
  ccw.linear.x = 0
  ccw.linear.y = 0
  ccw.angular.z = -speed

  cw = Twist()
  cw.linear.x = 0
  cw.linear.y = 0
  cw.angular.z = speed

  while avgpos() == None:
      print 'waiting for marker'
      rospy.sleep(1)

  while True:
      pos = avgpos()
      print 'seen at',pos
      if (pos.y < 0):
        motion(cw, 0.01)
      if (pos.y > 0):
        motion(ccw, 0.01)

def this_marker(marker):
  return marker.header.frame_id == 'head_camera' and locallib.block_nr(marker) == 4

if __name__ == '__main__':
  locallib.init('face_marker', target_marker_fn=this_marker)
  motion_demo()
