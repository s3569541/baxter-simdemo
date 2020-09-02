#!/usr/bin/python

import rospy
import locallib
import math

from geometry_msgs.msg import (
        PoseStamped,
        Pose,
        Point,
        Quaternion,
        TransformStamped,
        )

import os
import sys

#baseval = int(sys.argv[1])

print 'init node...'
rospy.init_node('make_alvar_marker_bundles', anonymous=True)
print 'init node done'

import tf2_ros
import tf2_geometry_msgs

global tf_buffer
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

# translate PoseStamped arg:ps into Pose with respect to arg:frame
def translate_frame(ps,frame):
    #print 'translate_frame',pose
    global tf_buffer
    # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
    transform = tf_buffer.lookup_transform(frame, # source frame
      ps.header.frame_id, # target frame
      rospy.Time(0), #get the tf at first available time
      rospy.Duration(2.0)) #wait for 2 seconds
    pose_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)
    #print 'pose in baxter frame',pose_transformed,' ', frame_to
    return pose_transformed

import tf.transformations
from tf.transformations import *
from std_msgs.msg import Header

def makepose(pos, orientation=Quaternion(x=0, y=1, z=0, w=0)):
    '''
    Create goal Pose and call ik move
    '''
    pose_right = Pose(
            position=Point(
                x=pos.x,
                y=pos.y,
                z=pos.z,
                ),
            orientation=orientation,
            )
    return pose_right

def make_pose_stamped_rpy(pos=(0,0,0), frame_id='base', r=3.14, p=0.0, y=0.0):
  print 'make_pose_stamped_rpy',r,p,y
  ori = quaternion_from_euler(r, p, y)
  print 'ori',ori
  return PoseStamped(
        header=Header(stamp=rospy.Time.now(), frame_id=frame_id),
        pose=Pose(position=pos, orientation=Quaternion(x = ori[0], y=ori[1], z=ori[2], w=ori[3])),
  )

broadcaster = tf2_ros.StaticTransformBroadcaster()
from geometry_msgs.msg import TransformStamped

for i in range(1):
  baseval = i*10
  # generate png
  print 'generating bundle for cube #',i
  # http://wiki.ros.org/ar_track_alvar#ar_track_alvar.2Fpost-fuerte.Detecting_multi-tag_bundles
  # positive-z comes out of the front of the tag toward the viewer, positive-x is to the right, and positive-y is up
  with open('block'+str(i)+'bundle.xml', 'w') as f:
    masterFrame = TransformStamped()
    masterFrame.header=Header(stamp=rospy.Time.now(), frame_id = 'world')
    masterFrameId = 'master'
    masterFrame.child_frame_id = masterFrameId
    quat = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
    masterFrame.transform.rotation.x = quat[0]
    masterFrame.transform.rotation.y = quat[1]
    masterFrame.transform.rotation.z = quat[2]
    masterFrame.transform.rotation.w = quat[3]
    print 'rot',masterFrame.transform.rotation
    masterFrame.transform.translation = Point(x=0,y=0,z=0)
    broadcaster.sendTransform(masterFrame)
    def doMarker(id,pos,r,p,y):
      markerFrame = TransformStamped()
      markerFrame.header=Header(stamp=rospy.Time.now(), frame_id = masterFrameId)
      markerFrameId = 'marker'+str(id)
      markerFrame.child_frame_id = markerFrameId
      markerFrame.transform.translation = pos
      quat = tf.transformations.quaternion_from_euler(r,p,y)
      markerFrame.transform.rotation.x = quat[0]
      markerFrame.transform.rotation.y = quat[1]
      markerFrame.transform.rotation.z = quat[2]
      markerFrame.transform.rotation.w = quat[3]
      broadcaster.sendTransform(markerFrame)
      o = -0.017
      f.write('<marker index="'+str(id)+'" status="1">\n')
      def myround(x):
        return math.floor(x * 1000) / 1000
      def tr(cx,cy,cz):
        corner = PoseStamped(
                header=Header(stamp=rospy.Time.now(), frame_id='marker'+str(id)),
                pose=Pose(position=Point(x=cx, y=cy, z=cz))
                )
        result = translate_frame(corner, masterFrameId).pose.position
        return Point(x=myround(result.x), y=myround(result.y), z=myround(result.z))
      p = tr(-o, y, 0)
      f.write('  <corner x="'+str(p.x)+'" y="'+str(p.y)+'" z="'+str(p.z)+'" />\n')
      p = tr( o, y, 0)
      f.write('  <corner x="'+str(p.x)+'" y="'+str(p.y)+'" z="'+str(p.z)+'" />\n')
      p = tr( o,-o, 0)
      f.write('  <corner x="'+str(p.x)+'" y="'+str(p.y)+'" z="'+str(p.z)+'" />\n')
      p = tr(-o,-o, 0)
      f.write('  <corner x="'+str(p.x)+'" y="'+str(p.y)+'" z="'+str(p.z)+'" />\n')
      f.write('</marker>\n')
    #writeMarker(0+baseval,20,0)
    #writeMarker(1+baseval,20,10) 
    #writeMarker(2+baseval,20,20) 
    #writeMarker(3+baseval,20,30) 
    #writeMarker(4+baseval,10,10) 
    #writeMarker(5+baseval,30,10) 
    # euler angles yaw, pitch, and roll about Z, Y, X axes respectively
    # quarter turn euler angle
    pi = math.pi
    qt = 2*pi/4
    doMarker(0+baseval,Point(x=0,y=0,z=0),0,    0,    0)
    doMarker(1+baseval,Point(x=0,y=0,z=0),0,    qt,   0)
    doMarker(2+baseval,Point(x=0,y=0,z=0),0,    2*qt, 0)
    doMarker(3+baseval,Point(x=0,y=0,z=0),0,    3*qt, 0)
    doMarker(4+baseval,Point(x=0,y=0,z=0),0,    +qt,    0)
    doMarker(5+baseval,Point(x=0,y=0,z=0),0,    -qt,    0)
  stream = os.popen('/opt/ros/melodic/lib/ar_track_alvar/createMarker -p < /tmp/marker.in')
  output = stream.read()
  output
