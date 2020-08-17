#!/usr/bin/python
#show and tell box is for the use of a prescripted movement to show the baxter
#robot grabbing a block and giving it to some one
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
rospy.init_node('marker_ik_example')
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
########################

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

    if lg.calibrated() == False:
        lg.calibrate()
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
    
    left.move_to_joint_positions(S1j)
    gripper = baxter_interface.Gripper('left', CHECK_VERSION)
    print 'calibrate'
    gripper.calibrate()
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
    
    #goal
    goalP1 = Point(x=1.2583525662953068, y=0.32040226742002664, z=0.2788726696784476)
    goalQ1 = Quaternion(x=0.7631680830198564, y=-0.02755537954971319, z=0.6430040504122455, w=-0.05797386713956011)
    goal = ik_solve('left',goalP1,goalQ1)
    
    #pregoal
    pregoalP1 = Point(x=1.2167327379279873, y=0.24181852256605696, z=0.6219109786693553)
    pregoalQ1 = Quaternion(x=0.728887712337905, y=-0.03934813515099991, z=0.6776550752614304, w=-0.08920776891829153)
    pregoal = ik_solve('left',pregoalP1,pregoalQ1)
    #pregrip
    pregripP1 = Point(x=0.7564160720416103, y=0.19026210082775052, z=-0.11989789999384434)
    pregripQ1 = Quaternion(x=0.9996463609432708, y=0.023193564174948188, z=0.008769810118763982, w=0.009607396328842483)
    pregrip = ik_solve('left',pregripP1,pregripQ1)
    #grip
    gripP1 = Point(x=0.7509983017123646, y=0.18440098049341058, z=-0.1922153309208344)
    gripQ1 = Quaternion(x=0.9993950013795004, y=0.0038452097622827075, z=0.022759519354074668, w=0.026016338295845813)
    grip = ik_solve('left',gripP1,gripQ1)
    #safetyup
    safupP = Point(x=0.8302343717427884, y=0.23686341493294455, z=0.3954032327879931)
    safupQ =  Quaternion(x=0.9908886961272979, y=0.06226809909369729, z=0.10950946265922441, w=-0.047644027017968923)
    safup = ik_solve('left',safupP,safupQ)


    #do all the movements that were specified
    left.move_to_joint_positions(pregrip)
    rospy.sleep(3)
    left.move_to_joint_positions(grip)
    gripper.close(block=True)
    rospy.sleep(0.5)
    left.move_to_joint_positions(safup)
    left.move_to_joint_positions(pregoal)
    left.move_to_joint_positions(goal)
    gripper.open(block=True)
    rospy.sleep(1)
    left.move_to_joint_positions(S1j)


main()


