#!/usr/bin/python
import collections
from scipy import stats
import rospy
import struct
import baxter_interface
import numpy as np
import matplotlib.pyplot as plt
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from baxter_core_msgs.msg import EndpointState
from ar_track_alvar_msgs.msg import AlvarMarkers
from sensor_msgs.msg import (JointState,
                            Range,
)

from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)

from tf.transformations import *

from std_msgs.msg import Header
from std_msgs.msg import UInt8

from sensor_msgs.msg import (JointState,
                            Range,
)

from trac_ik_python.trac_ik import IK

from baxter_interface import CHECK_VERSION
import tf2_ros
import tf2_geometry_msgs

import random as rd
rospy.init_node("marker_ik_example_movement",disable_signals=True)
global tf_buffer
tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0)) #tf buffer length


right = baxter_interface.Limb('right')
right.set_joint_position_speed(0.5)

left = baxter_interface.Limb('left')
left.set_joint_position_speed(0.5)
global prev_time
prev_time = 0
global range_array
range_array = collections.deque(5*[(0.0,0.0)], 5)
global endpoint_array
endpoint_array = []
global marker_array
marker_array = []
global last_left_ar_posns
last_left_ar_posns = collections.deque(20*[(0.0,0.0)], 20)
global ar_pos_left
ar_pos_left = (0.0,0.0)
global recently_moved
recently_moved = False
global poseglobal
poseglobal = None

def main():
    rospy.sleep(.2)
    global camdata
    #print "camdata is" ,camdata
    global right
    global lg
    global left

    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.Subscriber('/robot/range/left_hand_range/state', Range, callback_range,queue_size=1)
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback_marker,queue_size=1)
    rospy.Subscriber('/robot/limb/left/endpoint_state',EndpointState,callback_endpoint,queue_size=1)
    camdata = Point()
    lg = baxter_interface.Gripper('left')
    
    yfactor = .000667917
    xfactor = .0008135938

    if lg.calibrated() == False:
        lg.calibrate()
    #rg.calibrate()
    lg.open()
    quit = "n"
    

    pos = Point(x=0.50,
    y=0.182539067108,
    z=0.0998704377885)
    quat = Quaternion(x=0.0,
    y=1.0,
    z=0.0,
    w=0.0)
    pos_start = Point(x=0.578951563602,
    y=0.182539067108,
    z=0.3998704377885)

    # hard coded joint postion used for demos, not needed
    # high_joints = {'left_w0': 0.20747090156150222, 'left_w1': 1.3169225063996277, 
    #    'left_w2': -0.37314082665312687, 'left_e0': -0.7098496095939753, 
    #    'left_e1': 1.5523885573400387, 'left_s0': -0.4111068511532909, 
    #    'left_s1': -1.2532623037023831} 
    #pos_start = Point(x=0.102390108213,
    #        y=0.899521549527,
    #        z=0.344839245)
    
    joints = ik_solve('left',pos_start,quat)
    print 'move to start position'
    left.move_to_joint_positions(joints)
    
    # Uncomment as apppropriate, find_surface takes some time so its best to
    # re-use this value for demos/debugging/repeated runs
    # Find surface_z and set with rangefinder
    #surface_z = find_surface(pos_start, quat)
    # lab box height
    #surface_z = -0.314529563082
    # lab small table height
    surface_z = -0.0791295619731

        
    pos = Point(x=0.102390108213,
            y=0.899521549527,
            z=0.344839245)
    # front table
    pre_pose_high = Point(x=0.50,
    y=0.272539067108,
    z=0.3998704377885)
    #side table
    #pre_pose_high = Point(x=0.102390108213,
    #        y=0.899521549527,
    #        z=0.344839245)
    print "pos at start"
    print pos_start.z
    print surface_z
    global poseglobal

 
    
    rd.seed(5)
    # Open grippers
    lg.open()
    # Main Grab loop
    for i in range(0, 100):
        print i
        # Allow time for grippers to drop a block if holding on 
        rospy.sleep(0.5)

        # Move arm to a start position above the surface 
        print 'move to start position in the general area'
        joints = ik_solve('left',pos,quat)
        if surface_z > -0.10:
            print 'high'
            high_joints = ik_solve('left',pre_pose_high,quat)
            left.move_to_joint_positions(high_joints)
        else:
            print 'low'
            left.move_to_joint_positions(joints)

        # Wait to for some position data to come in from a marker topic
        rospy.sleep(2)
        pos1 = update_arm_pos(pos)
        # construct a PoseStamped from pos1
        pose_sm = Point(x=pos1.x+0.05,y=pos1.y,z=surface_z + 0.15) 
        ps = makePoseStamped(pose_sm,quat,"base")

        # wait for more data move to hover pose for better accuracy
        rospy.sleep(1)
        pos1 = update_arm_pos(pos)
        pose_sm = Point(x=pos1.x+ 0.05,y=pos1.y,z=surface_z + 0.4) 
        joints_hover = ik_solve('left',pose_sm,quat)
        print("Move to hover above block");
        left.move_to_joint_positions(joints_hover)

        rospy.sleep(1)
        pos2 = update_arm_pos(pos)
        pose_sm = Point(x=pos2.x+ 0.05,y=pos2.y,z=surface_z + 0.3) 
        joints_hover2 = ik_solve('left',pose_sm,quat)
        print("Move to second hover position above block");
        left.move_to_joint_positions(joints_hover2)

        rospy.sleep(1)
        pos3 = update_arm_pos(pos)
        pose_sm = Point(x=pos3.x+ 0.05,y=pos3.y,z=surface_z + 0.15) 
        joints_pre = ik_solve('left',pose_sm,quat)
        print("Move to pre-grab position above block");
        left.move_to_joint_positions(joints_pre)

        # wait for data in pre-grap pose, move to grab pose
        rospy.sleep(1)
        pose_grip = Point(x=pos3.x+0.01,y=pos3.y,z=surface_z + 0.12) 
        joints_grab = ik_solve('left',pose_grip,quat)
        print("Moving to grab block");
        left.move_to_joint_positions(joints_grab)
        # Grab block 
        print("Closing grippers");
        lg.close()
        rospy.sleep(0.2)
        print("Moving back to pre-grab");
        left.move_to_joint_positions(joints_pre)
        rospy.sleep(0.2)
        print("Moving back to hover position");
        left.move_to_joint_positions(joints_hover)

        # Move to randomised drop position magic numbers represent guesses for
        # sensible x and y bounds in base frame reference
        '''
        dropx = rd.uniform(0.40, 0.60) 
        dropy = rd.uniform(0.18, 0.35)
        pose_drop = Point(x=dropx,y=dropy,z=surface_z + 0.15) 
        joints = ik_solve('left',pose_drop,quat)
        '''
        joints ={'left_w0': -0.008436894333369775, 'left_w1': 1.5056021433095337, 
            'left_w2': 0.9629564395950685, 'left_e0': 0.2500388684253224, 
            'left_e1': 1.3640924156271041, 'left_s0': 0.009587379924283835, 
            'left_s1': -1.345301150975508}
        print("Moving to jointspace position ",joints);
        left.move_to_joint_positions(joints)
        joints ={'left_w0': 0.2972087776527989, 'left_w1': 0.14764565083397108, 
            'left_w2': 2.8624081501941823, 'left_e0': -0.004601942363656241, 
            'left_e1': 0.7359272829880272, 'left_s0': -0.9387962421858732, 
            'left_s1': -0.06404369789421602} 
        print("Moving to front of robot position",joints);
        left.move_to_joint_positions(joints)
        rospy.sleep(0.5)
        # Open grippers
        print("Dropping");
        lg.open()

        rospy.sleep(1)
        joints ={'left_w0': 0.002684466378799474, 'left_w1': 1.3690778531877317, 
			'left_w2': 3.025010113710036, 'left_e0': 0.018791264651596317, 
			'left_e1': 1.0431069357620812, 'left_s0': -0.8045729232458995, 
			'left_s1': -0.9023641984735946}
 
        left.move_to_joint_positions(joints)
        print("Moving to front of robot position",joints);
        print "clear"
        pos_start = Point(x=0.102390108213,
            y=0.899521549527,
            z=0.344839245)
        last_left_ar_posns.clear()
        high_joints = ik_solve('left',pre_pose_high,quat)
        print("Moving to high_joints position",high_joints);
        left.move_to_joint_positions(high_joints)
        break


def find_surface(pos, quat):
    global range_array
    # sleep until range_array is full
    pos_new = move_to_low_hover(pos, quat)
    rospy.sleep(2)
    mean_centered = np.mean(range_array)
    mean_left, mean_front = check_front_right(pos_new, quat)
    ranges = []
    # exclude ranges off edge of table
    for i in [mean_centered, mean_left,mean_front]:
        if i < 1.0:
            ranges.append(i)
    # Larges of remaining ranges should be surface
    print "pos z and max at find surface"
    print pos.z
    print max(ranges)
    print pos.z - max(ranges)
    return pos_new.z -  max(ranges)

# Move arm around a bit, make sure range finder has surface height not block height
def check_front_right(pos, quat):
    pos = Point(x=pos.x + 0.1,y=pos.y,z=pos.z)
    joints = ik_solve('left',pos,quat)
    left.move_to_joint_positions(joints)
    rospy.sleep(2)
    front_mean = np.mean(range_array)
    
    pos = Point(x=pos.x - 0.1,y=pos.y - 0.1,z=pos.z)
    joints = ik_solve('left',pos,quat)
    left.move_to_joint_positions(joints)
    rospy.sleep(2)
    right_mean = np.mean(range_array)
    
    return right_mean, front_mean

# Arm is above some surface. Move to hover above it.
def move_to_low_hover(pos, quat):
    global range_array
    while np.mean(range_array) > 65:
        print "coarse"
        pos = Point(x=pos.x,y=pos.y,z=(pos.z - 0.2))
        joints = ik_solve('left',pos,quat)
        left.move_to_joint_positions(joints)
        rospy.sleep(3)
        print range_array
        print np.mean(range_array)
    while np.mean(range_array) > 0.30:
        print "finer"
        pos = Point(x=pos.x,y=pos.y,z=(pos.z - 0.05))
        joints = ik_solve('left',pos,quat)
        left.move_to_joint_positions(joints)
        rospy.sleep(3)
        print range_array
        print np.mean(range_array)
    while np.mean(range_array) > 0.20:
        print "finest"
        pos = Point(x=pos.x,y=pos.y,z=(pos.z - 0.03))
        joints = ik_solve('left',pos,quat)
        left.move_to_joint_positions(joints)
        rospy.sleep(1)
        print range_array
        print np.mean(range_array)
    return pos


def update_arm_pos(pos):
    global ar_pos_left
    # Do not bother moving if there is invalid data or arm is in position

#    if abs(ar_pos_left[0]) < 0.05 and (ar_pos_left[1] < 0.05):
#        return pos
    if ar_pos_left[0] != 0.0:
        return Point(x=ar_pos_left[0], y=ar_pos_left[1], z=pos.z)
    else: 
        print("no vision data: return arbitrary pos");
        return Point(x=0.578951563602, y=0.182539067108, z=0.0998704377885)

# updates the x and y co-ordinates. Tries to handle some noise.
def update_position(pose):
    global ar_pos_left
    global last_left_ar_posns
    
    x = pose.position.x
    y = pose.position.y
    last_left_ar_posns.appendleft((x,y))
    x_data = [i[0] for i in last_left_ar_posns]
    y_data = [i[1] for i in last_left_ar_posns]
    x_zscore = stats.zscore(x_data)
    y_zscore = stats.zscore(y_data)
    # Do not update position until deque is full
    error_count = 0
    clean_x = []
    clean_y = []
    if last_left_ar_posns[len(last_left_ar_posns) - 1][0] != 0.0:
        for i in range(0,len(last_left_ar_posns)):
            #marker1 = marker1[(np.abs(stats.zscore(marker1)) < 6).all(axis=1)]
            if (abs(x_zscore[i]) < 1):
                clean_x.append(last_left_ar_posns[i][0])
            else:
                error_count += 1
            
            if (abs(y_zscore[i]) < 1):
                clean_y.append(last_left_ar_posns[i][1])
            else:
                error_count += 1

    else:
        ar_pos_left = (0.0,0.0)
    if error_count > len(last_left_ar_posns):
        ar_pos_left = (0.0,0.0)
    elif (len(clean_x) > 0 and len(clean_y) > 0):
        ar_pos_left =  (np.mean(clean_x),np.mean(clean_y))
    else:
        ar_pos_left = (0.0,0.0)

        
    #print ar_pos_left 


def callback_range(data):
    global range_array
    range_array.append(data.range)

def callback_endpoint(data):
    global endpoint_array
    outstr =  str(data.header.stamp) + "," + str(data.pose.position.x)+ "," + str(data.pose.position.y) + ","+ str(data.pose.position.z)
    endpoint_array.append(outstr)

def callback_marker(data):
    global marker_array
    global poseglobal
    global prev_time
    for marker in data.markers:
        if marker.id != 255 and marker.id != 0:
            if not "right" in marker.header.frame_id:
                tf_pos = translate_frame(PoseStamped(header=marker.header,pose=marker.pose.pose), "base")
                update_position(tf_pos.pose)
                ps = PoseStamped(
                header=marker.header,
                pose=marker.pose.pose,)
                poseglobal = target_from_marker(ps)

### ALL FROM TEG FROM HERE DOWN ###
def translate_frame(ps,frame):
    #print 'translate_frame',pose
    global tf_buffer
    # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
    transform = tf_buffer.lookup_transform(frame,
      #ps.header.frame_id, #source frame
      "reference/left_hand_camera",
      rospy.Time(0), #get the tf at first available time
      rospy.Duration(2.0)) #wait for 2 seconds
    pose_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)
    return pose_transformed


# translate PoseStamped ps with respect to a given frame
def translate_in_frame(ps,frame,dx,dy,dz):
    ps_f = translate_frame(ps,frame)
    result = PoseStamped(
    header=ps_f.header,
    pose=Pose(
            position=Point(
                x=ps_f.pose.position.x + dx,
                y=ps_f.pose.position.y + dy,
                z=ps_f.pose.position.z + dz
                ),
            orientation=ps_f.pose.orientation
    ))
    return result

# translate PoseStamped ps on Z axis by dz in base frame
def lift_in_base_frame(ps,dz):
    return translate_in_frame(ps,'base',0,0,dz)


# Quaternion -> Quaternion
def normalize(quat):
    quatNorm = math.sqrt(quat.x * quat.x + quat.y *
                        quat.y + quat.z * quat.z + quat.w * quat.w)
    normQuat = Quaternion(quat.x / quatNorm,
                              quat.y / quatNorm,
                              quat.z / quatNorm,
                              quat.w / quatNorm)
    return normQuat

# turn PoseStamped of a marker into target gripper pose
# assuming marker is 'level'
# was "hack_pose"
def target_from_marker(ps):
    #print 'hack_pose',ps
    quaternion = (
    ps.pose.orientation.x,
    ps.pose.orientation.y,
    ps.pose.orientation.z,
    ps.pose.orientation.w)
    # Leroy's guess
    q_rot = quaternion_from_euler(0.0, math.pi, math.pi/2)
    q_new = quaternion_multiply(quaternion, q_rot)
    pose_rot = PoseStamped(
    header=ps.header,
    pose=Pose(
            position=Point(
                x=ps.pose.position.x,
                y=ps.pose.position.y,
                z=ps.pose.position.z
                ),
            orientation=normalize(Quaternion(
        x=q_new[0],
        y=q_new[1],
        z=q_new[2],
        w=q_new[3]
            ))
        # This won't work in camera frame
        #orientation=Quaternion(
            #                 x=-0.0249590815779,
            #                 y=0.999649402929,
            #                 z=0.00737916180073,
            #                 w=0.00486450832011)
    ))
    return pose_rot



# translate a pose in terms of its "own" rotational axes (as if it had its own frame)
def translate_pose_in_own_frame(ps,localname,dx,dy,dz):
  m = geometry_msgs.msg.TransformStamped()
  m.child_frame_id = localname
  m.header.frame_id = ps.header.frame_id
  m.header.stamp = rospy.Time.now()
  m.transform.translation.x=ps.pose.position.x
  m.transform.translation.y=ps.pose.position.y
  m.transform.translation.z=ps.pose.position.z
  m.transform.rotation.x=ps.pose.orientation.x
  m.transform.rotation.y=ps.pose.orientation.y
  m.transform.rotation.z=ps.pose.orientation.z
  m.transform.rotation.w=ps.pose.orientation.w
  tf_buffer.set_transform(m,"")
  return translate_in_frame(ps,localname,dx,dy,dz)


def makePoseStamped(p,q,frame):
    hdr = Header(stamp=rospy.Time.now(), frame_id=frame)
    return PoseStamped(header=hdr,pose=Pose(position=p,orientation=q)),

def ik_solve(limb,p,q):
    #rospy.init_node("node1")
    global right
    global left
    ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
    iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
    ikreq = SolvePositionIKRequest()
    
    
    hdr = Header(stamp=rospy.Time.now(), frame_id='base')
    
    poses = {
        'left': PoseStamped(header=hdr,pose=Pose(position=p,orientation=q)),
        'right': PoseStamped(header=hdr,pose=Pose(position=p,orientation=q))        
    }
    
    rs = baxter_interface.RobotEnable(CHECK_VERSION)
    init_state = rs.state().enabled 

    ikreq.pose_stamp.append(poses[limb])
    
    try:
        rospy.wait_for_service(ns, 5.0)
        resp = iksvc(ikreq)
            
    except (rospy.ServiceException, rospy.ROSException), e:
        rospy.logerr("Service call failed: %s" % (e,))
        return 1

    # Check if result valid, and type of seed ultimately used to get solution
    # convert rospy's string representation of uint8[]'s to int's
    for i in range(50):
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type),
                               resp.result_type)
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                    ikreq.SEED_USER: 'User Provided Seed',
                    ikreq.SEED_CURRENT: 'Current Joint Angles',
                    ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                    }.get(resp_seeds[0], 'None')
            #print("SUCCESS - Valid Joint Solution Found from Seed Type: %s" %
                  #(seed_str,))
        # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            #print "\nIK Joint Solution:\n", limb_joints
            #print "------------------"
            #print "Response Message:\n", resp
            break
        else:
            print("INVALID POSE - No Valid Joint Solution Found.")
            
            
            noise= np.random.normal(0,0.25,7)
            
            
            js = JointState()
            js.header = hdr 
            i = 0
            for key,val in right.joint_angles().iteritems():
                js.name.append(key)
                js.position.append(val+noise[i])
                i += 1
           
            
            ikreq.seed_angles = [js]
            
            resp = iksvc(ikreq)
    try:
        return limb_joints
    except:
        mess = 1
        print "FATALITY" 
main()

