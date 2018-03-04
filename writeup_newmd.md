## Project: Kinematics Pick & Place
### Writeup Template: You can use this file as a template for your writeup if you want to submit it as a markdown file, but feel free to use some other method and submit a pdf if you prefer.

---


**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/side_view.jpg
[image2]: ./misc_images/top_view.jpg
[image3]: ./misc_images/origin_frame.png
[image4]: ./misc_images/ee_frame.png
[image5]: ./misc_images/theta1.png
[image6]: ./misc_images/theta2.png
[image7]: ./misc_images/theta2_top.png
[image8]: ./misc_images/theta3.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

We follow the rules for generating the DH parameters. The urdf indicates joint1 has a z offset of 0.33 and joint2 has a z offset of 0.42 and an x offset of 0.35. We move the joint1 origin to the accumulated z to provide a d1 of 0.75. We then twist the joint -90deg and translate 0.35 along the x axis resulting in alpha1=-pi/2 and a1=0.35, d2=0. We translate along the z axis 1.25 to reach joint3 with no rotation. Joints 4,5,6 are will have a coincident origin as they form the spherical wrist. We rotate -pi/2 to orient in the joint4 configuration and apply a translation of 0.96 + 0.54 for d4 and a3 has an offset of -0.054. A twist of +pi/2 moves from joint4 to joint5. A final twist of -pi/2 moves from joint5 to joint6. Finally, the end effector has d7 of 0.193 + 0.11 or 0.303 from the urdf.

#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | 0
1->2 | - pi/2 | 0.35 | 0 | -pi/2 + q2
2->3 | 0 | 1.25 | 0 | 0
3->4 |  -pi/2 | -0.054 | 1.50 | 0
4->5 | pi/2 | 0 | 0 | 0
5->6 | -pi/2 | 0 | 0 | 0
6->EE | 0 | 0 | 0.303 | 0

The generic transform looks like 
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/e14085f2f4ef9761c8276e41643c3b5a.svg?invert_in_darkmode" align=middle width=671.81235pt height=78.794265pt/></p>
 
For the T<sub>0,1</sub> matrix we substitute alpha_<sub>0</sub>=0, a<sub>0</sub>=0,d<sub>1</sub>=0.75 noting that cos(0) = 1 and sin(0) = 0 yields
 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/68cfaddf0a18ac2c8c33ea45a956b5ce.svg?invert_in_darkmode" align=middle width=281.86455pt height=78.794265pt/></p>
 
 For the T<sub>1,2</sub> matrix we substitute alpha<sub>1</sub>=-pi/2, a<sub>1</sub>=0.35,d<sub>2</sub>=0 noting that cos(-pi/2) = 0 and sin(-pi/2) = -1 yields


<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/ccb81345288f361ee17a8837806ad957.svg?invert_in_darkmode" align=middle width=294.6504pt height=78.794265pt/></p>
 


 For the T<sub>2,3</sub> matrix we substitute alpha<sub>2</sub>=-0, a<sub>2</sub>=1.25,d<sub>3</sub>=0 noting that cos(0) = 1 and sin(0) = 0 yields
 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/bfe617bff329c2fcb8018edf589e79f5.svg?invert_in_darkmode" align=middle width=281.86455pt height=78.794265pt/></p>
 
  For the T<sub>3,4</sub> matrix we substitute alpha<sub>3</sub>=-pi/2, a<sub>3</sub>=-0.054,d<sub>4</sub>=1.50 noting that cos(-pi/2) = 0 and sin(-pi/2) = -1 yields
  
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/c85a72d2723ba550935828289042d6c6.svg?invert_in_darkmode" align=middle width=315.6549pt height=78.794265pt/></p>
 
   For the T<sub>4,5</sub> matrix we substitute alpha<sub>4</sub>=pi/2, a<sub>4</sub>=0,d<sub>5</sub>=0 noting that cos(pi/2) = 0 and sin(pi/2) = 1 yields
   
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/f3af93561037d030c0e46e599f06d8d0.svg?invert_in_darkmode" align=middle width=273.6459pt height=78.794265pt/></p>
 
 For the T<sub>5,6</sub> matrix we substitute alpha<sub>5</sub>=-pi/2, a<sub>5</sub>=0,d<sub>6</sub>=0 noting that cos(-pi/2) = 0 and sin(-pi/2) = -1 yields
   
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/3f9b31d26eb433c994179550371d0a0c.svg?invert_in_darkmode" align=middle width=273.6459pt height=78.794265pt/></p>
  
 For the T<sub>6,G</sub> matrix we substitute alpha<sub>6</sub> = 0, a<sub>6</sub>=0,d<sub>7</sub>=0.303 noting that cos(0) = 1 and sin(0) = 0 yields
 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/3e01e4c6ebed410ed0ccb6c5114bb029.svg?invert_in_darkmode" align=middle width=293.766pt height=78.794265pt/></p>
 
 The transform from the base link to the end effector is then
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/dad78a8626b6ebb5090edff289a4ede8.svg?invert_in_darkmode" align=middle width=247.5462pt height=15.885705pt/></p>
 
The input into the system is the roll, pitch, and yaw of the end effector. We will transform the rotations about the end effector into the total rotation required from the origin of our robot. Here is the coordinate frame of our system:

![alt text][image3]

and here is the coordinate frame of the end effector:

![alt text][image4]
 
 To transform the orientation of our world frame into that of the input end effector rotations, we need to rotate 180 degrees about the z axis to get the coordinate frames equivalent. Additionally, a rotation of -90 degrees about the y axis is required from the base revolute joint such that the normal is in the direction of the end effector frame. We call this matrix <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/5dde2efb0811e83c5b3226ba58b805eb.svg?invert_in_darkmode" align=middle width=37.619835pt height=13.656621pt/></p>.
 
 We define a sequence of extrinsic rotations in order to calculate the end effector rotation.
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/407a0a77127e7b9f94745708f2c537c4.svg?invert_in_darkmode" align=middle width=154.36938pt height=15.885705pt/></p>
 
 The rotations about <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/476d8b3d1647f5f93a0bc315e187ac59.svg?invert_in_darkmode" align=middle width=101.09583pt height=16.0677pt/></p> are by the book and are:
 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/b2380288bb34a602d0c3a19e54d297bc.svg?invert_in_darkmode" align=middle width=212.96385pt height=59.068185pt/></p>
 where *r* is the input roll.
 
  <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/b8cea39aa656ffbb15a6cde3debc5943.svg?invert_in_darkmode" align=middle width=213.3846pt height=59.068185pt/></p>
 where *p* is the input pitch.
 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/6d210a9be1152a792d33a7ee6c1c4df7.svg?invert_in_darkmode" align=middle width=213.8136pt height=59.068185pt/></p>
 where *y* is the input yaw.
 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/5c134f1fb417bc045e61b1341daf1a30.svg?invert_in_darkmode" align=middle width=224.3604pt height=15.885705pt/></p>
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/fbd794cbe11149993ac24912818d25dd.svg?invert_in_darkmode" align=middle width=214.33665pt height=59.068185pt/></p>
 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/c71eb1fc383683c5e984e982ed2a41fd.svg?invert_in_darkmode" align=middle width=205.2963pt height=59.068185pt/></p>
 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/53aa80eb6a8847540edade6b181d3a84.svg?invert_in_darkmode" align=middle width=159.43785pt height=59.068185pt/></p>
 
 at this point we have <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/837f1978b45a2b95d8f815ca99541953.svg?invert_in_darkmode" align=middle width=32.92674pt height=13.656621pt/></p> in the world coordinates and the end effector position is also part of the input. Given the rotation of the end effector and the length of the end effector, we can now define the coordinates of the wrist center (WC). Lesson 15 defines the wrist center equations as 
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/e4120e2f3e078152c842d857be948961.svg?invert_in_darkmode" align=middle width=157.10574pt height=16.376943pt/></p>
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/401df9903fad3ffd06c8fe7d1f81650e.svg?invert_in_darkmode" align=middle width=155.98275pt height=16.97751pt/></p>
 <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/dd5942ad1b6b5406beddaac517e9e799.svg?invert_in_darkmode" align=middle width=155.00232pt height=16.376943pt/></p>
 
 where the **n** vector may be extracted from the just calculated <p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/837f1978b45a2b95d8f815ca99541953.svg?invert_in_darkmode" align=middle width=32.92674pt height=13.656621pt/></p> to give the resulting **WC** position vector.
 
 
#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

We solve the inverse kinematics problem by considering the first three joints as those that resolve the place the wrist center in the correct position and the last three joints are those that specify the orientation of the end effector. We will start by solving for theta1, which happens to be the only joint to provide any rotation about the z axis. We start by taking a top-down view of the robot in order to calculate the required rotation about z in order to calculate theta1.

![alt text][image5]

From here it is easy to solve for theta1.
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/a838d652c9bef3b51c76ba0fef7083d0.svg?invert_in_darkmode" align=middle width=163.355445pt height=36.05316pt/></p>

Next up, we move to theta2 and we will start with a side-view.

![alt text][image6]

From this drawing we can see that
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/2c5449615d1f7d692d523c42526cd478.svg?invert_in_darkmode" align=middle width=214.6551pt height=16.376943pt/></p>

Furthermore we can see that
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/b2514efe1fba72c3958398ec5a252b02.svg?invert_in_darkmode" align=middle width=127.38594pt height=32.950665pt/></p>
and from the law of cosines
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/052282aff4dbed322bae94b4af97ba35.svg?invert_in_darkmode" align=middle width=227.43105pt height=35.749725pt/></p>

Some of these components are immediately solvable:
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/f3db9530e8b0376562ad49f994eb4d4b.svg?invert_in_darkmode" align=middle width=122.297175pt height=14.55729pt/></p>
and extracted from the DH table is
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/94745abd184dfab98c6c641c4388b4d3.svg?invert_in_darkmode" align=middle width=63.86523pt height=11.190894pt/></p>

<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/e5afd2f95d30a66b6e74230b4a3ffe20.svg?invert_in_darkmode" align=middle width=131.122365pt height=14.55729pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/9c1fa03d44b8f8f6b5a050e549f3adb2.svg?invert_in_darkmode" align=middle width=177.6687pt height=19.654965pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/fc539b2550686e5b1ea7a9f1ca95cb60.svg?invert_in_darkmode" align=middle width=76.93521pt height=11.190894pt/></p>

<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/dc94a0487535061e04d132c66b0ff9bc.svg?invert_in_darkmode" align=middle width=110.0286pt height=13.838616pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/ffe9ae78ebd6e369c9861137ac080000.svg?invert_in_darkmode" align=middle width=124.09122pt height=13.656621pt/></p>

to solve for B and r1 requires we return to the top view.

![alt text][image7]

In this coordinate frame we see that r1 is now the remaining distance to the wrist center from joint1 once the length of a1 has been subtracted.

<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/1462fabdccba3f592e4cb33a20f83ceb.svg?invert_in_darkmode" align=middle width=187.5357pt height=29.48121pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/a84e3042371e9e344c61225096b97851.svg?invert_in_darkmode" align=middle width=118.579395pt height=19.654965pt/></p>

and now all required elements to solve for theta2 have been resolved.

We advance to joint3 to solve for theta3 in a similar manner.

![alt text][image8]

From this drawing we can see that
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/acf20ec90c76998b942450dff6f5beeb.svg?invert_in_darkmode" align=middle width=134.557995pt height=33.197505pt/></p>

meanwhile we can see that
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/7f2e059b8baefe9e12abfdac3d2611be.svg?invert_in_darkmode" align=middle width=180.9456pt height=14.55729pt/></p>
or
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/ff152bf73356c74f9d7c9379cc986f15.svg?invert_in_darkmode" align=middle width=180.9456pt height=14.55729pt/></p>
and
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/ed569009cbce56a898bce6074587766a.svg?invert_in_darkmode" align=middle width=125.057955pt height=33.769395pt/></p>
where d4 is from the table and A was previously resolved.
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/2b54815f8343c33415c890dfebda106e.svg?invert_in_darkmode" align=middle width=145.725855pt height=32.950665pt/></p>
resulting in 
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/a64204aad9ce5122db3bed38c0f52dbe.svg?invert_in_darkmode" align=middle width=85.396905pt height=13.9854165pt/></p>

We an solve angle_b from the law of cosines
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/e2788e9087bcc37a405945df5c4df7c7.svg?invert_in_darkmode" align=middle width=226.08135pt height=35.749725pt/></p>
and all unknowns are now resolved for theta3. However, the sign of theta3 needs to be inverted to reflect the negative rotation. This completes the inverse kinematics code for position determination.

We now move to the inverse kinematics of the orientation of the spherical wrist. First, we already have the total orientation required and we know the orientation applied to the end effector by the first three joints. So, we will first solve for the required orientation applied by the spherical wrist alone.
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/ee3a04764b46f426a5956738d37435ab.svg?invert_in_darkmode" align=middle width=115.3284pt height=21.032385pt/></p>
where
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/ddb2385c3dd9af0257a6bc0249b68317.svg?invert_in_darkmode" align=middle width=326.2677pt height=15.885705pt/></p>
Here we just applied the homogenous transforms using the solved rotational angles for the first three joints to find the total orientation contribution of the first three joints. Therefore, we have also found the total orientation contribution required by the last three joints and the problem is to now solve for the angles on each of the last three joints.

The transform from joint3 to the wrist center is:
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/241514948843874dd24b5d35d14db03b.svg?invert_in_darkmode" align=middle width=130.590075pt height=15.885705pt/></p>
to make the matrix multiplications less daunting we will write "c4" to mean cos(theta4) and "s4" to mean sin(theta4).
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/824da39cc9e34bdac48ebdad8ef71aeb.svg?invert_in_darkmode" align=middle width=103.18869pt height=15.885705pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/a0e4471158b986e6aad00d585e58d594.svg?invert_in_darkmode" align=middle width=109.0947pt height=15.885705pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/21f531434c87f4d80622c0b374291955.svg?invert_in_darkmode" align=middle width=214.8729pt height=59.068185pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/e35b42a27905a623b887a5400aa883c8.svg?invert_in_darkmode" align=middle width=376.9458pt height=59.068185pt/></p>

theta5 could be easily plucked out of the T3_6[1,2] entry but the guidance is to use the tangent allowing for the specification of quadrants. Observe:
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/91f458cf04887af71b4acd3aada0f637.svg?invert_in_darkmode" align=middle width=155.41878pt height=18.869895pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/e348ba4ffb203c498cfd159ceadae849.svg?invert_in_darkmode" align=middle width=156.00816pt height=18.869895pt/></p>
then
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/f64ae536bea8f53063b9354235e697f0.svg?invert_in_darkmode" align=middle width=294.28905pt height=18.869895pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/19d83fe482b6d4526f6fc3d6bc0eeec9.svg?invert_in_darkmode" align=middle width=283.7868pt height=18.869895pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/1bbc0fa3370d3fe9810494a14b29d751.svg?invert_in_darkmode" align=middle width=215.39265pt height=29.48121pt/></p>
leading to
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/bd511b637a47be2a9fb98e6a101a57a7.svg?invert_in_darkmode" align=middle width=295.77405pt height=42.10767pt/></p>
theta4 and theta6 are easier to resolve as
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/1392d00021d2bd05961e097c251f1628.svg?invert_in_darkmode" align=middle width=202.31145pt height=39.374115pt/></p>
<p align="center"><img src="https://rawgit.com/in	git@github.com:kscharpf/RoboND-Kinematics-Project/master/svgs/590134f91aef6e646a8d3b4f0e1264b6.svg?invert_in_darkmode" align=middle width=202.31145pt height=39.374115pt/></p>

and all angles have now been resolved. The inverse kinematics problem is complete.






### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 


The result of this implementation was 8/10 successes. These results can be seen in the following two images.
![alt text][image1]
![alt text][image2]

The first image shows a target object that has fallen on its side and the second image shows one target object not in the bin. The reason behind the second failure is that the robot did not account for the length of the target while orienting into the bin. The result was that the target hit the side of the bin during one of the manuevers moving the bin and the target fell to the ground. The first failure was a similar result. The arm hit the target during orientation - before the grasping stage. Both of these could be accounted for but are difficult to consider in this scenario where we are given the required pose but not the contextual information such as "am I already holding the target". 




