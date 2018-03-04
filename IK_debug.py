from sympy import *
from time import time
from mpmath import radians
import numpy as np
import tf
import sys

'''
Format of test case is [ [[EE position],[EE orientation as quaternions]],[WC location],[joint angles]]
You can generate additional test cases by setting up your kuka project and running `$ roslaunch kuka_arm forward_kinematics.launch`
From here you can adjust the joint angles to find thetas, use the gripper to extract positions and orientation (in quaternion xyzw) and lastly use link 5
to find the position of the wrist center. These newly generated test cases can be added to the test_cases dictionary.
'''

test_cases = {1:[[[2.16135,-1.42635,1.55109],
                  [0.708611,0.186356,-0.157931,0.661967]],
                  [1.89451,-1.44302,1.69366],
                  [-0.65,0.45,-0.36,0.95,0.79,0.49]],
              2:[[[-0.56754,0.93663,3.0038],
                  [0.62073, 0.48318,0.38759,0.480629]],
                  [-0.638,0.64198,2.9988],
                  [-0.79,-0.11,-2.33,1.94,1.14,-3.68]],
              3:[[[-1.3863,0.02074,0.90986],
                  [0.01735,-0.2179,0.9025,0.371016]],
                  [-1.1669,-0.17989,0.85137],
                  [-2.99,-0.12,0.94,4.06,1.29,-4.12]],
              4:[[[0.80245,-0.019778,-0.46804],
                  [0.0091962,0.62935,-0.54015,0.55863]],
                 [0.91629,0.15957,-0.25197],
                 [0.17,1.27,0.68,-4.40,-0.58,5.55]],
              5:[]}

def range_limit(angle):
    if angle < -pi:
        angle += 2*pi
    if angle > pi:
        angle -= 2*pi
    return angle

#def myround(v):
    #outv = v*1000
    #outv = int(v + 0.5)
    #return outv/1000.


def test_code(test_case):
    ## Set up code
    ## Do not modify!
    x = 0
    class Position:
        def __init__(self,EE_pos):
            self.x = EE_pos[0]
            self.y = EE_pos[1]
            self.z = EE_pos[2]
    class Orientation:
        def __init__(self,EE_ori):
            self.x = EE_ori[0]
            self.y = EE_ori[1]
            self.z = EE_ori[2]
            self.w = EE_ori[3]

    position = Position(test_case[0][0])
    orientation = Orientation(test_case[0][1])

    class Combine:
        def __init__(self,position,orientation):
            self.position = position
            self.orientation = orientation

    comb = Combine(position,orientation)

    class Pose:
        def __init__(self,comb):
            self.poses = [comb]

    req = Pose(comb)
    start_time = time()
    
    ########################################################################################
    ## 

    ## Insert IK code here!
    
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




    # 
    # Extract the x/y/z coordinates of this pose

    px = req.poses[x].position.x
    py = req.poses[x].position.y
    pz = req.poses[x].position.z

    # convert the quaternion coordinates into roll pitch and yaw

    roll, pitch, yaw = tf.transformations.euler_from_quaternion(
        [req.poses[x].orientation.x, req.poses[x].orientation.y,
         req.poses[x].orientation.z, req.poses[x].orientation.w])
   
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

    # Do the extrinsic rotation zyx to get the rotation applied to the end effector
    ROT_EE = Rz * Ry * Rx
 
    # but the problem is not done - we have to transform these angles into our coordinate system
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
    # | J2| / r2
    # |   |/______________        
    # |   /       
    # |  / a1      
    # | /
    # |/____________________________________ X
    # J1
    # 
    #
    # r2 is then the distance from the origin to the wrist center minus a1
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
    # What is the rotation applied at the end of the positioning joints (R0_3)?

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


    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    FK = T0_G.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})

    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [WC[0], WC[1], WC[2]]
    your_ee = [FK[0,3],FK[1,3],FK[2,3]]
    #your_wc = [1,1,1] # <--- Load your calculated WC values in this array
    #your_ee = [1,1,1] # <--- Load your calculated end effector value from your forward kinematics
    ########################################################################################

    ## Error analysis
    print ("\nTotal run time to calculate joint angles from pose is %04.4f seconds" % (time()-start_time))

    # Find WC error
    if not(sum(your_wc)==3):
        wc_x_e = abs(your_wc[0]-test_case[1][0])
        wc_y_e = abs(your_wc[1]-test_case[1][1])
        wc_z_e = abs(your_wc[2]-test_case[1][2])
        wc_offset = sqrt(wc_x_e**2 + wc_y_e**2 + wc_z_e**2)
        print ("\nWrist error for x position is: %04.8f" % wc_x_e)
        print ("Wrist error for y position is: %04.8f" % wc_y_e)
        print ("Wrist error for z position is: %04.8f" % wc_z_e)
        print ("Overall wrist offset is: %04.8f units" % wc_offset)

    # Find theta errors
    t_1_e = abs(theta1-test_case[2][0]) 
    #if t_1_e > pi/2:
        #t_1_e = pi - t_1_e
    t_2_e = abs(theta2-test_case[2][1]) 
    #if t_2_e > pi/2:
        #t_2_e = pi - t_2_e
    t_3_e = abs(theta3-test_case[2][2])
    #if t_3_e > pi/2:
        #t_3_e = pi - t_3_e
    #print "theta4: %f exp: %f" % (theta4,test_case[2][3])
    t_4_e = abs(theta4-test_case[2][3]) 
    #if t_4_e > pi/2:
        #t_4_e = pi - t_4_e
    t_5_e = abs(theta5-test_case[2][4]) 
    #if t_5_e > pi/2:
        #t_5_e = pi - t_5_e
    #print "theta6: %f exp: %f" % (theta6,test_case[2][5])
    t_6_e = abs(theta6-test_case[2][5]) 
    #if t_6_e > pi/2:
        #t_6_e = pi - t_6_e
    print ("\nTheta 1 error is: %04.8f" % t_1_e)
    print ("Theta 2 error is: %04.8f" % t_2_e)
    print ("Theta 3 error is: %04.8f" % t_3_e)
    print ("Theta 4 error is: %04.8f" % t_4_e)
    print ("Theta 5 error is: %04.8f" % t_5_e)
    print ("Theta 6 error is: %04.8f" % t_6_e)
    print ("\n**These theta errors may not be a correct representation of your code, due to the fact \
           \nthat the arm can have muliple positions. It is best to add your forward kinmeatics to \
           \nconfirm whether your code is working or not**")
    print (" ")

    # Find FK EE error
    if not(sum(your_ee)==3):
        ee_x_e = abs(your_ee[0]-test_case[0][0][0])
        ee_y_e = abs(your_ee[1]-test_case[0][0][1])
        ee_z_e = abs(your_ee[2]-test_case[0][0][2])
        ee_offset = sqrt(ee_x_e**2 + ee_y_e**2 + ee_z_e**2)
        print ("\nEnd effector error for x position is: %04.8f" % ee_x_e)
        print ("End effector error for y position is: %04.8f" % ee_y_e)
        print ("End effector error for z position is: %04.8f" % ee_z_e)
        print ("Overall end effector offset is: %04.8f units \n" % ee_offset)




if __name__ == "__main__":
    # Change test case number for different scenarios
    test_case_number = int(sys.argv[1])
    print "Executing test case: %d" % test_case_number
    test_code(test_cases[test_case_number])
