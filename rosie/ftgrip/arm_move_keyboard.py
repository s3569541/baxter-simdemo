#!/usr/bin/python
#This file is used to move the arm a certain distance using keyboard inputs
import rospy
import struct
import baxter_interface
import numpy as np
from baxter_core_msgs.srv import (
    SolvePositionIK,
    SolvePositionIKRequest,
)
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)


from std_msgs.msg import Header
from std_msgs.msg import UInt8

from sensor_msgs.msg import (JointState,
)

from baxter_interface import CHECK_VERSION
rospy.init_node('marker_ik_example_movement')
#setting limb speed
right = baxter_interface.Limb('right')
right.set_joint_position_speed(1.0)

left = baxter_interface.Limb('left')
left.set_joint_position_speed(1.0)
#################################
#pre existing solver from baxter
#################################
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
##############################

def get_user_input(pos,userin):
    mov_factor = 0.05
    #get user input in a certain form to get it to move
    #userin = raw_input("move (wasd or lp) or quit(q)")
    for userin_next in userin:
        print pos
        if (userin_next == "q"):
            return None, None, "y"
        elif (userin_next == "w"):
            pos = Point(x=pos.x + mov_factor,y=pos.y, z=pos.z)
        elif (userin_next == "a"):
            pos = Point(x=pos.x, y=pos.y + mov_factor, z=pos.z)
        elif (userin_next == "s"):
            pos = Point(x=pos.x - mov_factor, y=pos.y, z=pos.z)
        elif (userin_next == "d"):
            pos = Point(x=pos.x, y=pos.y - mov_factor, z=pos.z)
        elif (userin_next == "p"):
            pos = Point(x=pos.x, y=pos.y, z=pos.z + mov_factor)
        elif (userin_next == "l"):
            pos = Point(x=pos.x, y=pos.y, z=pos.z - mov_factor)
    print pos
    return pos, "n"


def main():
    rospy.sleep(.2)
    global camdata
    #print "camdata is" ,camdata
    global right
    global lg
    global left
    global smile
    #set arm speed
    right = baxter_interface.Limb('right')
    right.set_joint_position_speed(1.0)
    left = baxter_interface.Limb('left')
    left.set_joint_position_speed(1.0)
    camdata = Point()
    lg = baxter_interface.Gripper('left')
    
    yfactor = .000667917
    xfactor = .0008135938

    if lg.calibrated() == False:
        lg.calibrate()
    #rg.calibrate()
    #input certain set values of where the arm will start and go 
    Q = Quaternion(x=0.7523426889287905, y=-0.6584930265055371, z=0.0010142237493953393, w=0.019141154854433382)
    S1p = Point(x=.55,y=-.569769,z=.12)
    S2p = Point(.70,-.2651429,.12)
    S3p = Point(.55,.00738,.12)
    goalP =  Point(x=0.6586872879757675, y=0.1567994652872984, z=0.28566015793134736)
    Q =  Quaternion(x=1, y=0, z=0, w=0)

    goalP1 =  Point(x=0.6586872879757675, y=0.3567994652872984, z=0.28566015793134736)
    Q1 =  Quaternion(y=0, z=0, x=0.707, w=0.707)



    #rospy.sleep(.15)
    S1j = ik_solve('left',goalP1,Q)
    left.move_to_joint_positions(S1j)
    print "1"
    quit = "n"
    
    #pos and quat to solve for 
    pos = Point(x=0.578951563602,
    y=0.182539067108,
    z=0.0998704377885)
    quat = Quaternion(x=0.142665026585,
    y=0.989469457861,
    z=0.00906167097681,
    w=0.0226885052115)
    # do a certain prescripted movement.
    while (quit != "y"):
        for i in ["lllll","pp",
        "ll","pp",
        "ll","pp",
        "ll","pp",
        "ll","pp",
        "ll","pp",
        "ll","pp",
        "ll","pp","q"]:
            joints = ik_solve('left',pos,quat)
            left.move_to_joint_positions(joints)
            pos,quit = get_user_input(pos, i)
            rospy.sleep(1)

main()

