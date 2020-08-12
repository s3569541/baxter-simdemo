#!/usr/bin/python2

# Copyright (c) 2013-2015, Rethink Robotics
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import argparse
import socket
import sys

import rospy
import rosgraph

import std_srvs.srv

from baxter_core_msgs.srv import (
    ListCameras,
)
from baxter_interface.camera import CameraController


def list_cameras(*_args, **_kwds):
    ls = rospy.ServiceProxy('cameras/list', ListCameras)
    rospy.wait_for_service('cameras/list', timeout=10)
    resp = ls()
    if len(resp.cameras):
        # Find open (publishing) cameras
        master = rosgraph.Master('/rostopic')
        resp.cameras
        cam_topics = dict([(cam, "/cameras/%s/image" % cam)
                               for cam in resp.cameras])
        open_cams = dict([(cam, False) for cam in resp.cameras])
        try:
            topics = master.getPublishedTopics('')
            for topic in topics:
                for cam in resp.cameras:
                    if topic[0] == cam_topics[cam]:
                        open_cams[cam] = True
        except socket.error:
            raise ROSTopicIOException("Cannot communicate with master.")
        for cam in resp.cameras:
            print("%s%s" % (cam, ("  -  (open)" if open_cams[cam] else "")))
    else:
        print ('No cameras found')


def reset_cameras(*_args, **_kwds):
    reset_srv = rospy.ServiceProxy('cameras/reset', std_srvs.srv.Empty)
    rospy.wait_for_service('cameras/reset', timeout=10)
    reset_srv()


def enum_cameras(*_args, **_kwds):
    try:
        reset_cameras()
    except:
        srv_ns = rospy.resolve_name('cameras/reset')
        rospy.logerr("Failed to call reset devices service at %s", srv_ns)
        raise
    else:
        list_cameras()


def open_camera(camera, res, *_args, **_kwds):
    cam = CameraController(camera)
    cam.resolution = res
    cam.open()


def close_camera(camera, *_args, **_kwds):
    cam = CameraController(camera)
    cam.close()

def main():
    camname = 'right_hand_camera'
    ####cam = CameraController('head_camera')
    ####cam.close()
    #reset_cameras()
    cam = CameraController(camname)
    cam.open()
    print 'camera ',camname
    print 'gain ',cam.gain
    print 'res ',cam.resolution
    cam.close()

if __name__ == '__main__':
    sys.exit(main())
