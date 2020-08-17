#!/usr/bin/python
#show and tell is for the table height to give the cube to some one
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
rospy.init_node('marker_ik_example',disable_signals=True)
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
###########################

def main():
    rospy.sleep(.2)
    global camdata
    #print "camdata is" ,camdata
    global right
    global lg
    global left
    global smile

    right = baxter_interface.Limb('right')
    right.set_joint_position_speed(1.0)
    left = baxter_interface.Limb('left')
    left.set_joint_position_speed(1.0)
    camdata = Point()
    lg = baxter_interface.Gripper('left')
    
    yfactor = .000667917
    xfactor = .0008135938


    #rg.calibrate()
    #positions for the arm to move
    Q = Quaternion(x=0.7523426889287905, y=-0.6584930265055371, z=0.0010142237493953393, w=0.019141154854433382)
    S1p = Point(x=.55,y=-.569769,z=.12)
    S2p = Point(.70,-.2651429,.12)
    S3p = Point(.55,.00738,.12)
    #initial postion
    goalP =  Point(x=0.6586872879757675, y=0.1567994652872984, z=0.28566015793134736)
    Q =  Quaternion(x=1, y=0, z=0, w=0)

    #goal 1
    goalP1 =  Point(x=0.6586872879757675, y=0.3567994652872984, z=0.28566015793134736)
    Q1 =  Quaternion(y=0, z=0, x=0.707, w=0.707)


    #goal 2
    goalP2 =  Point(x=0.246586872879757675, y=0.3567994652872984, z=0.28566015793134736)
    Q2 =  Quaternion(y=0, z=0, x=-0.707, w=0.707)


    #goal 3
    goalP3 =  Point(x=1.2586872879757675, y=0.3567994652872984, z=0.08566015793134736)
    Q3 =  Quaternion(x=0, y=0, z=-0.707, w=0.707)

    #rospy.sleep(.15)
    S1j = ik_solve('left',goalP1,Q)
    print "1"
    
    #left.move_to_joint_positions(S1j)
    gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    print 'calibrate'
    #gripper.close()

    # alternate position for the arms to move

    #rospy.sleep(2)
    #S2j = ik_solve('left',goalP1,Q1)
    print "2"
    #left.move_to_joint_positions(S2j)
    #rospy.sleep(2)
    #left.move_to_joint_positions(S1j)
    #S3j = ik_solve('left',goalP2,Q)
    print "3"
    #left.move_to_joint_positions(S3j)
    #rospy.sleep(2)
    #left.move_to_joint_positions(S1j)
    #S4j = ik_solve('left',goalP3,Q)
    print "4"
    #left.move_to_joint_positions(S4j)
    #rospy.sleep(2)
    #left.move_to_joint_positions(S1j)
    
    #set positions for the joints to move to
    pregrab = {'left_w0': 0.6396699885482175, 'left_w1': 1.2191312311719327, 'left_w2': -0.8559612796400609, 'left_e0': -0.9621894492011258, 'left_e1': 1.5316798167035857, 'left_s0': -0.5288398766234964, 'left_s1': -0.8655486595643447} 


    grab = {'left_w0': 0.6795534890332383, 'left_w1': 1.0768545130955605, 'left_w2': -0.85289331806429, 'left_e0': -0.8548107940491468, 'left_e1': 1.5930390482190022, 'left_s0': -0.5867476513661708, 'left_s1': -0.8011214664731573}

    give = {'left_w0': 0.7589369948063085, 'left_w1': 0.786548648988246, 'left_w2': 0.0011504855909140602, 'left_e0': -0.7650729179578502, 'left_e1': -0.049854375606275945, 'left_s0': -0.9042816744584514, 'left_s1': -0.40650490878963463}
    
    #make the movement of the arms to certain positions for the block and give.
    left.move_to_joint_positions(S1j)
    rospy.sleep(0.5)
    left.move_to_joint_positions(pregrab)
    rospy.sleep(1)
    left.move_to_joint_positions(grab)
    gripper.close(block=True)
    left.move_to_joint_positions(pregrab)
    rospy.sleep(1)
    left.move_to_joint_positions(give)
    gripper.open(block=True)
    rospy.sleep(2)
    left.move_to_joint_positions(S1j)

main()

