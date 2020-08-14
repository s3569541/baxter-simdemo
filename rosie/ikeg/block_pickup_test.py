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

def callback(data):
    global right_marker
    global left_marker

    for marker in data.markers:
        if marker.header.frame_id == "reference/left_hand_camera":
            left_marker = marker
        if marker.header.frame_id == "reference/right_hand_camera":
            right_marker = marker

    #Timeout after 5s
    rospy.sleep(5)
    right_marker = None
    left_marker = None

def move_to(hover_height):
    global left_marker
    global right_marker
    target = None
    if right_marker is not None and left_marker is None:
        target = createIKTarget(right_marker)
    if left_marker is not None:
        target = createIKTarget(left_marker)

    if target == None:
        print "No valid target visible"
        print left_marker
        print right_marker
        return
    target.pose.position.z = target.pose.position.z + hover_height
    IKMoveTo(target, "left")

def IKMoveTo(pose_stamped, limb):
    joint_solution = trac_ik_solve(limb, pose_stamped)
    # print "\nSoln:\t",joint_solution

    if joint_solution is not None:
        print "Moving to solution:\t",pose_stamped.pose.position.z,pose_stamped.header.frame_id
        make_move_trac(joint_solution, limb, speed=0.5)
        # rospy.signal_shutdown("Intended")
        # exit()

# msg: 7-vector of positions corresponding to joints
# limb: 'left'|'right'
def make_move_trac(msg, limb, speed):
    #print 'make_move_trac',msg
    SPEED_SCALE = 1
    speed = speed * SPEED_SCALE
    arm = baxter_interface.Limb(limb)
    #print 'arm',arm
    lj = arm.joint_names()
    command = {}
    # for i in range(0, len(msg.joints[0].name)):
    jointnames = ['s0','s1','e0','e1','w0','w1','w2']
    for i in range(0, len(jointnames)):
        command[limb+"_"+jointnames[i]] = msg[i]
    #print 'current:',arm.joint_angles()
    #print 'make_move: speed',speed
    arm.set_joint_position_speed(speed)
    #print 'make_move_trac: posns',command
    #arm.set_joint_positions(command)
    arm.move_to_joint_positions(command,timeout=5.0)


def trac_ik_solve(limb, ps, seed_as_posn=True,posn_tol=0.01):

    local_base_frame = limb+"_arm_mount"
    ik_solver = IK(
        local_base_frame,
        limb+"_gripper",
        urdf_string=urdf_str
    )

    state = baxter_interface.Limb(limb).joint_angles()
    #print 'solve current:',state
    jointnames = ['s0','s1','e0','e1','w0','w1','w2']
    success = False
    while not success:
        command = []
        success = True
        for i in range(0, len(jointnames)):
            key = limb+"_"+jointnames[i]
            if not key in state:
                success = False
                break
            command.append(state[key])
    #print 'candidate seed',command

    if seed_as_posn:
        seed_state = command
    else:
        seed_state = [0.0] * ik_solver.number_of_joints

    # gripper_ps = translate_pose_in_own_frame(ps,'gripper_target',0,0,0.05)
    p = translate_frame(ps,local_base_frame)

    soln = ik_solver.get_ik(
        seed_state,
        p.pose.position.x,
        p.pose.position.y,
        p.pose.position.z,
        p.pose.orientation.x, 
        p.pose.orientation.y, 
        p.pose.orientation.z, 
        p.pose.orientation.w,
        posn_tol,
        posn_tol,
        posn_tol,
        0.1,
        0.1,
        0.1,
    )
    return soln

# give PoseStamped ps in frame-of-refernce frame
def translate_frame(ps,frame):
    # print 'translate_frame',frame, ps
    global tf_buffer
    # https://answers.ros.org/question/222306/transform-a-pose-to-another-frame-with-tf2-in-python/
    transform = tf_buffer.lookup_transform(
        frame,
        ps.header.frame_id, #source frame
        rospy.Time(0), #get the tf at first available time
        rospy.Duration(2.0)  #2 seconds timeout on wait for TF
    )
    pose_transformed = tf2_geometry_msgs.do_transform_pose(ps, transform)
    # print 'pose in baxter frame',pose_transformed,' ', frame_to
    print "target:\t",ps,"\ntransformed:\t", pose_transformed
    return pose_transformed

#Need to flip the pose of the input marker
def createIKTarget(marker):
    quaternion = (
        marker.pose.pose.orientation.x,
        marker.pose.pose.orientation.y,
        marker.pose.pose.orientation.z,
        marker.pose.pose.orientation.w
        )

    q_rot = quaternion_from_euler(0.0, math.pi, math.pi/2)
    q_new = quaternion_multiply(quaternion, q_rot)

    return PoseStamped(
        header=marker.header,
        pose=Pose(
            position=Point(
                x=marker.pose.pose.position.x,
                y=marker.pose.pose.position.y,
                z=marker.pose.pose.position.z
                ),
            orientation=normalize(Quaternion(
                x=q_new[0],
                y=q_new[1],
                z=q_new[2],
                w=q_new[3]
                )
            )
        )
    )

# Quaternion -> Quaternion
def normalize(quat):
    quatNorm = math.sqrt(   quat.x * quat.x + 
                            quat.y * quat.y + 
                            quat.z * quat.z + 
                            quat.w * quat.w
                            )
    normQuat = Quaternion(  quat.x / quatNorm,
                            quat.y / quatNorm,
                            quat.z / quatNorm,
                            quat.w / quatNorm
                            )
    return normQuat

def pickup():
    global left_marker
    global right_marker
    if left_marker is not None or right_marker is not None:
        #Hover at 15cm
        move_to(-0.15)
        #Go to block position
        rospy.sleep(3)
        move_to(0)
        gripper.close(block=True)
        move_to(-0.20)
        gripper.open(block=True)
    else:
        print "No visible markers\t",left_marker,right_marker

if __name__ == "__main__":
    global urdf_str
    global gripper
    global left_marker
    global right_marker
    left_marker = None
    right_marker = None
    #Disable signals allows for KeyboardInterrupts to not be exclusively captured by the rospy node
    rospy.init_node("block_pickup", disable_signals=True)
    rospy.Subscriber('/ar_pose_marker', AlvarMarkers, callback, queue_size=1)
    urdf_str = rospy.get_param('/robot_description')

    tf_buffer = tf2_ros.Buffer(rospy.Duration(1200.0))
    tf_listener = tf2_ros.TransformListener(tf_buffer) #TF semantics require establishment of a TF Listener to work as intended
    gripper = baxter_interface.Gripper('left', baxter_interface.CHECK_VERSION)
    gripper.calibrate()
    gripper.open(block=True)
    try:
        while True:
            # pickup()
            rospy.sleep(0.2)
    except KeyboardInterrupt:
        #rospy needs to be manually signalled to initiate shutdown routine if disable_signals=True on node initialisation
        rospy.signal_shutdown("User requested shutdown")