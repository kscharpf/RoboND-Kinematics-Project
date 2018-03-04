#!/usr/bin/env python

# Copyright (C) 2017 Udacity Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

def range_limit(angle):
    while angle < -pi:
        angle += 2*pi
    while angle > pi:
        angle -= 2*pi
    return angle
 


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols
	#
	#
        q1,q2,q3,q4,q5,q6,q7 = symbols('q1:8') # theta
        d1,d2,d3,d4,d5,d6,d7 = symbols('d1:8')
        a0,a1,a2,a3,a4,a5,a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') 
        # Create Modified DH parameters
        #
        #
        # 
        # 
        s = {alpha0:      0, a0:      0,  d1:  0.75,
             alpha1:  -pi/2., a1:   0.35,  d2:     0,   q2: q2-pi/2.,
             alpha2:      0, a2:   1.25,  d3:     0,
             alpha3:  -pi/2., a3: -0.054,  d4:  1.50,
             alpha4:   pi/2., a4:      0,  d5:     0,
             alpha5:  -pi/2., a5:      0,  d6:     0,
             alpha6:      0, a6:      0,  d7: 0.303, q7: 0}
                  
    
        # Define Modified DH Transformation matrix
        #
        #
        # Homogenous transforms - by the book, substitute for known parameters during creation
    
        T0_1 = Matrix([[            cos(q1),            -sin(q1),            0,              a0],
                       [sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                       [sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                       [                  0,                   0,            0,               1]])
        T0_1 = T0_1.subs(s)
        T1_2 = Matrix([[            cos(q2),            -sin(q2),            0,              a1],
                       [sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                       [sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                       [                  0,                   0,            0,               1]])
        T1_2 = T1_2.subs(s)
        T2_3 = Matrix([[            cos(q3),            -sin(q3),            0,              a2],
                       [sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                       [sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                       [                  0,                   0,            0,               1]])
        T2_3 = T2_3.subs(s)
        T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
                       [sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                       [sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                       [                  0,                   0,            0,               1]])
        T3_4 = T3_4.subs(s)
        T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
                       [sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                       [sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                       [                  0,                   0,            0,               1]])
        T4_5 = T4_5.subs(s)
        T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
                       [sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                       [sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                       [                  0,                   0,            0,               1]])
        T5_6 = T5_6.subs(s)
        T6_G = Matrix([[            cos(q7),            -sin(q7),            0,              a6],
                       [sin(q7)*cos(alpha6), cos(q7)*cos(alpha6), -sin(alpha6), -sin(alpha6)*d7],
                       [sin(q7)*sin(alpha6), cos(q7)*sin(alpha6),  cos(alpha6),  cos(alpha6)*d7],
                       [                  0,                   0,            0,               1]])
        T6_G = T6_G.subs(s)
        # Create individual transformation matrices
        #
        #
        r, p, y = symbols('r p y')
    
        T0_G = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_G
    
        # Extract rotation matrices from the transformation matrices
        # By the book equations
        ###
        Rx =    Matrix([[1,       0,       0],
                        [0,  cos(r), -sin(r)],
                        [0,  sin(r),  cos(r)]]) # ROLL
    
        Ry =    Matrix([[ cos(p),       0,       sin(p)],
                        [      0,       1,            0],
                        [-sin(p),       0,       cos(p)]]) # PITCH
        Rz =    Matrix([[ cos(y), -sin(y),            0],
                        [ sin(y),  cos(y),            0],
                        [      0,       0,            1]]) # YAW
    
    
        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

           # Extract end-effector position and orientation from request
           # px,py,pz = end-effector position
           # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])
    
    
     
            # we have to transform these angles into our coordinate system
            #
            # Here is the RPY coordinate frame
            #       Y
            #      /                        end effector
            #     /                       ________
            #    /______________________ (________()____________ X
            #   |
            #   |
            #   |
            #   |
            #   Z
            # 
            # Roll is the rotation about the x axis
            # Pitch is the rotation about the y axis
            # Yaw is the rotation about the z axis
            #
            # and here is our coordinate frame at the origin
            #
            #   Z    
            #    |      /
            #    |     / 
            #    |    /
            #    |   /
            #   | | /
            #   |_|______X
            #
            # so we have to apply a 180deg rotation about the z axis and a -90deg rotation about the y-axis to get the end effector orientation into the 
            # provided RPY 
          
            Rcorr = Rz.subs(y,radians(180)) * Ry.subs(p, radians(-90))
            
            # Do the extrinsic rotation zyx to get the rotation applied to the end effector
            ROT_EE = Rz * Ry * Rx

            # now we do that additional rotation to our calculated end effector rotation
            ROT_EE = ROT_EE * Rcorr

            # and we can substitute real values so that the orientation of the end effector can now be considered a known value
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            # Find the position of the wrist center (WC)
            # The end effector position is given 
            EE = Matrix([[px],[py],[pz]])
    
            # wrist center position equals the end effector position minus the 
            # end effector length multiplied by the rotation of the end effector.
            # The rotation of the end effector is the third column of the rotational matrix.
            # translation is along the z axis - the third column of the rotational matrix
            WC = EE - (0.303) * ROT_EE[:,2]
    
            # Take the wrist center position in the frame of the origin
            # The origin rotates about the Z-axis so to find the angle of rotation
            # we take the tan^-1 of the y-coordinate over the x-coordinate
            #
            #
            #  Y                WC
            #  |             __/|
            #  |          __/   |
            #  |       __/      |
            #  |    __/         |
            #  | __/    theta1  |
            #  |/_______________|__________ X
            # J1
            theta1 = atan2(WC[1], WC[0])
    
            # Rotation about the z-axis is resolved. Translate the distance to joint2
            # and solve for that rotation about the y-axis
            # side-view
            #
            #                  J3
            #         | theta2 /\_____ side_a
            #         |       /       \___
            #         |      /            \ 
            #         |     /               WC
            # Z       |    /            ____/|
            # |       |   / angle_a____/     |
            # |       |  /     ___/          | r2
            # |       | /____/   side_b      |
            # |_______|/______phi2___________|
            # |      J2        r1 
            # |
            # | d1
            # |
            # J1_______________________________X
            #
            # r2 = WCz - d1 = WC[2] - 0.75
            r2 = WC[2] - 0.75
            #
            # Note that r1 is not solvable in this view so switch to top-down view
            #
            # Y   |   WC
            # |   |  /
            # | J2| / r1
            # |   |/______________        
            # |   /       
            # |  / a1      
            # | /
            # |/____________________________________ X
            # J1
            # 
            #
            # r1 is then the distance from the origin to the wrist center minus a1
            r1 = sqrt(WC[1]*WC[1] + WC[0]*WC[0]) - 0.35
            #
            # Now solve for side_b
            side_b = sqrt(r1*r1 + r2*r2)
            #
            # theta2 will be pi/2 minus angle_a minus phi2
            # 
            # phi2 is now tan^-1(r2/r1)
            phi2 = atan2(r2,r1)
            #
            # next we need angle_a which comes from the law of cosines
            # angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            #
            # side_b is known, need side_c and side_a, draw the triangle to get the sides
            #                      
            #  |       J3 _________side_a
            #  |side_c __/         \____ WC 
            #  |    __/          _______/   
            #  | __/     _______/ side_b
            #  |/_______/  ______________________
            # J2
            #
            # the distance between J2 and J3 is fixed in the DH parameter table
            side_c = 1.25
            #
            # side_a is the distance between J3 and the wrist center
            # side_a = sqrt(a3^2 + d4^2) == 1.50097 =~ 1.501    
            side_a = 1.501
            #
            angle_a = acos((side_b*side_b + side_c*side_c - side_a*side_a)/(2.*side_b*side_c))
            #
            theta2 = pi/2. - phi2 - angle_a
        
            # theta3 is the angle off of the normal line from joint2 to joint3
            #
            # theta3 - crude ascii art
            #      | theta3 /
            #      |       /
            #      |      /
            #      |     /
            #      |    /
            #      |   /
            #      |  / 
            #      | /   phi3  
            #  ____|/________________________
            #      / \  psi3                 |
            #     /   \__________            | 0.054
            #    /   angle_b     \           |
            #   /                 \__________| WC
            #  /                    side_a
            #    
            
            # 
            # we can see that
            # pi = phi3 + psi3 + angle_b
            #
            # so that
            #
            # phi3 = pi - psi3 - angle_b
            # 
            # we can also see that psi3 is solvable
                #
            # psi3 = asin(0.054/side_a) = asin(0.054/1.501) ~= 0.036 
            psi3 = 0.036
            #
            # angle_b comes from the law of cosines 
            #
            angle_b = acos((side_a*side_a + side_c*side_c - side_b*side_b)/(2.*side_a*side_c))
            # and we can solve phi3 now
            phi3 = pi - psi3 - angle_b
           
            # we can also see how to solve theta3 now 
            # theta3 = pi/2 - phi3
            #
            # BUT the theta3 rotation shown is a negative rotation so...
            # 
            # theta3 = -(pi/2 - phi3)
            theta3 = -(pi/2. - phi3)
            #
            # and that completes the resolution of the three joint angles required to specify
            # the location of the wrist center
           
            # The next part of the problem is to resolve the rotation angles within the spherical
            # wrist in order to match the required end effector rotation. The high-level approach
            # is that we now know the rotation applied to the end effector by the first 3 revolute
            # joints and we know the required end effector rotations so we can invert the rotations
            # of the first 3 positioning joints and then solve for the additional rotations to 
            # get the end effector rotation
            #
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
        
            # Now solve for the required rotations in the spherical wrist
            R3_6 = R0_3.inv("LU") * ROT_EE
    
            # We now have the numerical values of the total rotations required within the spherical wrist
            # but we need to solve for the individual rotations of the three joints. Do the matrix multiplication
            # from T3_4 through T5_6. We only need the first three columns - extract these from above but with simpler 
            # syntax.  
            #T3_4 = Matrix([[            cos(q4),            -sin(q4),            0,              a3],
                           #[sin(q4)*cos(alpha3), cos(q4)*cos(alpha3), -sin(alpha3), -sin(alpha3)*d4],
                           #[sin(q4)*sin(alpha3), cos(q4)*sin(alpha3),  cos(alpha3),  cos(alpha3)*d4],
                           #[                  0,                   0,            0,               1]])
            # Note that alpha3 is -pi/2
            # T3_4 = | c4   -s4      0 |  == | c4  -s4   0 |
            #        | s4c3  c4c3  -s3 |     | 0    0    1 |
            #        | s4s3  c4s3   c3 |     | -s4 -c4   0 |
            #
            # alpha4 = pi/2
            #T4_5 = Matrix([[            cos(q5),            -sin(q5),            0,              a4],
                           #[sin(q5)*cos(alpha4), cos(q5)*cos(alpha4), -sin(alpha4), -sin(alpha4)*d5],
                           #[sin(q5)*sin(alpha4), cos(q5)*sin(alpha4),  cos(alpha4),  cos(alpha4)*d5],
                           #[                  0,                   0,            0,               1]])
            #
            # T4_5 = | c5      -s5       0  | = | c5     -s5      0  |
            #        | s5c4     c5c4    -s4 |   | 0       0      -1  |
            #        | s5s4     c5s4     c4 |   | s5      c5      0  |
            #
            #
            # T3_4 * T4_5 = | c4  -s4   0 | * | c5   -s5    0 | = | c4c5   -c4s5    s4 |
            #               |  0    0   1 |   |  0     0   -1 |   |   s5     c5      0 |
            #               | -s4  -c4  0 |   | s5    c5    0 |   | -s4c5   s4s5    c4 |
            #
            # alpha5 = -pi/2
            #T5_6 = Matrix([[            cos(q6),            -sin(q6),            0,              a5],
                           #[sin(q6)*cos(alpha5), cos(q6)*cos(alpha5), -sin(alpha5), -sin(alpha5)*d6],
                           #[sin(q6)*sin(alpha5), cos(q6)*sin(alpha5),  cos(alpha5),  cos(alpha5)*d6],
                           #[                  0,                   0,            0,               1]])
            # T5_6 = | c6    -s6   0 | =  | c6    -s6    0 |
            #        | s6c5 c6c5 -s5 |    | 0       0    1 |
            #        | s6s5 c6s5  c5 |    | -s6   -c6    0 |
            #
            #
            # T3_6 = |  c4c5   -c4s5    s4   | * |  c6    -s6    0 | = | c4c5c6 - s4s6   -c4c5s6 - s4c6    -c4s5     |  = R3_6
            #        |    s5      c5     0   |   |   0      0    1 |   | s5c6             -s5s6             c5       |
            #        | -s4c5    s4s5    c4   |   | -s6    -c6    0 |   | -s4c5c6 - c4s6   s4c5s6 - c4c6    s4s5      |
            #
            #
            # theta5 could then be found as the acos(R3_6[1,2]) but it is advised to use atan2 for quadrant selection. So,
            # cos(theta5) == R3_6[1,2] can be used for the denominator in the tangent function and we need to find sin(theta5)
            #
            # observe: (-c4s5)**2 = R3_6[0,2]**2 and (-s4s5)**2 = R3_6[2,2]**2
            # then:   c4**2 * s5**2 + s4**2 * s5**2 = R3_6[0,2]**2 + R3_6[2,2]**2
            #         s5**2 * (c4**2 + s4**2) = R3_6[0,2]**|2 + R3_6[2,2]**2
            #         s5**2 = R3_6[0,2]**2 + R3_6[2,2]**2
            #         s5 = sqrt(R3_6[0,2]**2 + R3_6[2,2]**2)
            # Now we have tan(theta5) = s5/c5 = sqrt(R3_6[0,2]**2 + R3_6[2,2]**2)/R3_6[1,2] leading to 
            #
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] +R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
          
            # Now we look for solutions to theta4 and theta6. For theta4, observe that R3_6[2,2]/R3_6[0,2] = -tan(theta4)
            # In order to properly choose our quadrant, we can include the knowledge that the sign of the sin(theta5) term 
            # dictates the quadrant. 
            #
            # Likewise, tan(theta6) = R3_6[1,1]/R3_6[1,0] and the same rule of signs applies
        
            if sin(theta5) < 0.:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])
            else:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
        
            FK = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
