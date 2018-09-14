## Project: Kinematics Pick & Place

---
**Steps to complete the project:**  


1. Set up your ROS Workspace.
2. Download or clone the [project repository](https://github.com/udacity/RoboND-Kinematics-Project) into the ***src*** directory of your ROS Workspace.  
3. Experiment with the forward_kinematics environment and get familiar with the robot.
4. Launch in [demo mode](https://classroom.udacity.com/nanodegrees/nd209/parts/7b2fd2d7-e181-401e-977a-6158c77bf816/modules/8855de3f-2897-46c3-a805-628b5ecf045b/lessons/91d017b1-4493-4522-ad52-04a74a01094c/concepts/ae64bb91-e8c4-44c9-adbe-798e8f688193).
5. Perform Kinematic Analysis for the robot following the [project rubric](https://review.udacity.com/#!/rubrics/972/view).
6. Fill in the `IK_server.py` with your Inverse Kinematics code. 


[//]: # (Image References)

[image1]: ./misc_images/forward_kinematics.png
[image2]: ./misc_images/fk_links_stick_figure.png
[image3]: ./misc_images/fk_links_diagram.png
[image4]: ./misc_images/DH_diagram.png
[image5]: ./misc_images/total_homogeneous_transform.png
[image6]: ./misc_images/WC_equations.png
[image7]: ./misc_images/first_three_joints_diagram.png
[image8]: ./misc_images/first_three_joints_equations.png
[image9]: ./misc_images/R3_6.png
[image10]: ./misc_images/perfect_score_pick_and_place.png


## [Rubric](https://review.udacity.com/#!/rubrics/972/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one. You can submit your writeup as markdown or pdf.

You're reading it!

### Kinematic Analysis
#### 1. Run the forward_kinematics demo and evaluate the kr210.urdf.xacro file to perform kinematic analysis of Kuka KR210 robot and derive its DH parameters.

I used script `forward_kinematics.launch` that also runs the **joint_state_publisher** node:


```
$ roslaunch kuka_arm forward_kinematics.launch
```

Below is an image of the robot in RViz, showing its links and joint frames in reset mode, according the to its URDF. Notice that in reset mode (when the robot is **centered** using the **joint_state_publisher**), all of its frames have the same orientation as that of its **base_link**; that is, all frames are parallel to that of the **base_link**.

![alt text][image1]


Using the **joint_state_publisher** interface, I verified how each joint moves when given a positive or negative value. This was needed in the derivation of IK formulas later on. 
The following image shows a stick figure of the robot in its reset configuration. 

![alt text][image2]


Notice that in a DH diagram, we choose the Z axis of a rotating joint to be its rotation axis (observing the right hand rule), and that of a prismatic joint to be its translation axis (observing the direction of positive translation). For that, the URDF frames need to be rotated as shown in the following diagram. We also notice that the center of the URDF frames do not match the robot's DH diagram.

![alt text][image3]


To derive the DH diagram of the robot, I followed the steps in the lecture videos, and the following image from the videos perfectly shows the final result. Any parameter not shown in the image has a value of zero.

![alt text][image4]


The following is a reminder for how DH parameters are defined:
- qi (joint angle) = angle between Xi-1 to Xi measured about Zi in a right handed sense.
- di (link offset) = signed distance between Xi-1 to Xi, measured along Zi.
- ai-1 (link length) = distance from Zi-1 to Zi measured along Xi-1. Xi-1 is perpendicular to both Zi-1 and Zi.
- alphai-1 (twist angle) = angle between Zi-1 and Zi measured about Xi-1, in a right hand sense.


Thus we can derive DH parameters for KR210 as shown below (joint_i parameters are from the URDF file):
- a0 = alpha0 = 0, since Z0 and Z1 in DH are coincident.
- d1 = joint_1.z + joint_2.z
- a1 = joint_2.x
- alpha1 = -pi/2, based on the DH diagram
- d2 = 0, since X1 and X2 cross each other at O2 of DH
- q2_DH = q2_URDF - pi/2. Because while the DH diagram shows a joint angle of -pi/2, the corresponding joint anlge of the robot is actually 0.
- Since X2, X3, X4, X5, X6, and XG in the DH diagram are all parallel when robot is in its reset configuration (accoriding to URDF file), then in that configuration we have q3 = q4 = q5 = q6 = 0, which matches the reset config. So, unlike the case of q2, there is no offset between these qs in URDF versus the DH coordinates.
- a2 = joint_3.z
- alpha2 = 0, because Z2 and Z3 of DH are parallel
- d3 = 0, since X2 and X3 of DH are aligned
- a3 = joint_4.z
- alpha3 = -pi/2, based on the DH diagram
- d4 = joint_4.x + joint_5.x
- a4 = 0, because Z4 and Z5 of DH cross each other at O4 = O5
- alpha4 = pi/2, based on the DH diagram
- d5 = 0, because O4 and O5 are coincident in the DH diagram
- a5 = 0, because Z5 and Z6 cross each other at O5 = O6
- alpha5 = -pi/2, based on the DH diagram
- d6 = 0,  because X5 and X6 cross each other at O5 = O6
- a6 = 0, because Z6 and ZG cross each other at OG
- alpha6 = 0, based on the DH diagram
- d7 =  joint_6.x + gripper_joint.x
- q7 = 0, and is fixed, according to the URDF file


The following is the DH table based on the above equations, replacing values from the `kr210.urdf.xacro` file:

Links | alpha(i-1) | a(i-1) | d(i-1) | theta(i)
--- | --- | --- | --- | ---
0->1 | 0 | 0 | 0.75 | q1
1->2 | - pi/2 | 0.35 | 0 | - pi/2 + q2
2->3 | 0 | 1.25 | 0 | q3
3->4 | - pi/2 | -0.054 | 1.50 | q4
4->5 | pi/2 | 0 | 0 | q5
5->6 | - pi/2 | 0 | 0 | q6
6->EE | 0 | 0 | 0.303 | 0


#### 2. Using the DH parameter table you derived earlier, create individual transformation matrices about each joint. In addition, also generate a generalized homogeneous transform between base_link and gripper_link using only end-effector(gripper) pose.

I used a symbolic representation of a single DH transformation matrix about a given joint, as shown below:

```
       # Define Modified DH Transformation matrix
        Ti = Matrix([[            cos(q),           -sin(q),           0,             a],
                     [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                     [                 0,                 0,           0,             1]])
```

To derive individual transformations per each joint (e.g., Ti_j's), I simply replaced the parameters of the corresponding joint/link in the above formula. For example, to derive the transformation matrix for joint 2, I did the following:

```
        T1_2 = Ti.subs({q: q2, d: d2, alpha: alpha1, a: a1})
        T1_2 = T1_2.subs(s_list)
```

where `s_list = s.items()` and `s` is the dictionary for the DH table. With this, I avoided repeating the definition of the DH matrix 6 more times.

The following are the resulting transformations:

```
T0_1 = [[cos(q1), -sin(q1), 0,    0],
        [sin(q1),  cos(q1), 0,    0],
        [      0,        0, 1, 0.75],
        [      0,        0, 0,    1]]

T1_2 = [[sin(q2),  cos(q2), 0, 0.35],
        [      0,        0, 1,    0],
        [cos(q2), -sin(q2), 0,    0],
        [      0,        0, 0,    1]]

T2_3 = [[cos(q3), -sin(q3), 0, 1.25],
        [sin(q3),  cos(q3), 0,    0],
        [      0,        0, 1,    0],
        [      0,        0, 0,    1]]

T3_4 = [[ cos(q4), -sin(q4), 0, -0.054],
        [       0,        0, 1,    1.5],
        [-sin(q4), -cos(q4), 0,      0],
        [       0,        0, 0,      1]]

T4_5 = [[cos(q5), -sin(q5),  0, 0],
        [      0,        0, -1, 0],
        [sin(q5),  cos(q5),  0, 0],
        [      0,        0,  0, 1]]

T5_6 = [[ cos(q6), -sin(q6), 0, 0],
        [       0,        0, 1, 0],
        [-sin(q6), -cos(q6), 0, 0],
        [       0,        0, 0, 1]]

T6_G = [[1, 0, 0,     0],
        [0, 1, 0,     0],
        [0, 0, 1, 0.303],
        [0, 0, 0,     1]]
```

To compute T0_j matrices, that define the transformation matrix from the base_link to link j, I used the following recursive formula, for j from 0 to the end effector (note that this is not a line of code, and is only to illustrate the recursion):

```
        T0_j = T0_(j-1) * T(j-1)_j # base_link to link_j
```

As an example, T0_2 is coded as below:

```
        T0_2 = T0_1 * T1_2 # base_link to link_2
```

Finally, to accommodate for the difference between the gripper frame in DH versus the one in URDF, we need a corrective rotation matrix as a 180 degree rotation around Z followed by a -90 degree rotation around the Y axis (applied to T0_G of DH). T_Toal would then be the total homogeneous transformation matrix between the base_link and the gripper_link.  

```
        R_z = Matrix([[     cos(pi),   -sin(pi),             0],
                      [     sin(pi),    cos(pi),             0],
                      [           0,          0,             1]])
        R_y = Matrix([[  cos(-pi/2),          0,    sin(-pi/2)],
                      [           0,          1,             0],
                      [ -sin(-pi/2),          0,    cos(-pi/2)]])
        R_corr = simplify(R_z * R_y)

        # Total homogeneous transformation between base_link and
        # gripper_link with orientation correction applied.
        T_corr = R_corr.row_join(Matrix([0, 0, 0])).col_join(Matrix([[0, 0, 0, 1]]))
        T_total = T0_G * T_corr
```

Given a 4x4 T_Total transformation matrix as above, its top left 3x3 matrix determines the orientation of the gripper, and its last column ([px, py, pz]) determines the position of the gripper-link. See the image below.

![alt text][image5]


Meanwhile, the orientation of the tool_tip is typically specified using three Euler angles (roll about the X axis, pitch about the Y axis, and yaw about the Z axis, in an extrinsic X_Y_Z rotation sequence). So, given T_Total above, its 3x3 composite rotation matrix is equivalent to the R_rpy matrix below (where R_corr was explained before):

```
        R_y_pitch = Matrix([[  cos(_pitch),            0,    sin(_pitch)],
                            [            0,            1,              0],
                            [ -sin(_pitch),            0,    cos(_pitch)]])

        R_z_yaw   = Matrix([[    cos(_yaw),   -sin(_yaw),              0],
                            [    sin(_yaw),    cos(_yaw),              0],
                            [            0,            0,              1]])

        R_x_roll = Matrix([[             1,            0,              0],
                           [             0,   cos(_roll),    -sin(_roll)],
                           [             0,   sin(_roll),     cos(_roll)]])
        R_rpy = R_z_yaw * R_y_pitch * R_x_roll * R_corr
```

which can be computed given a triplet of roll-pitch-yaw rotation angles for the gripper_link. Thus, given the position and orientation of the gripper_link, we can reconstruct T_total, the homogeneous transformation matrix from the base_link to the gripper_link.


#### 3. Decouple Inverse Kinematics problem into Inverse Position Kinematics and inverse Orientation Kinematics; doing so derive the equations to calculate all individual joint angles.

Since we have a spherical wrist, and O4 = O5 = O6 in our DH diagram, the inverse kinematics problem gets simplified:
1- Given the gripper-link orientation, we can solve the last three joints of the robot.
2- Given the gripper-link position, can derive wrist center (O4 = O5 = O6), and from that we can solve the first three joints of the robot. 

To derive Wrist Center (WC), we have the following equation (where d = d6 in the DH table, R0_6 is the 3x3 rotation matrix R_rpy describes before, and [px, py, pz] is the position of the gripper. 

![alt text][image6]


Once we have the coordinates of WC (O4 = O5 = O6), and given the following diagram, we can solve for the first three angles. 
![alt text][image7]


Below are the equations for solving the first three angles. Notice that since a3 is negative, we have to use its absolute value.

![alt text][image8]


Finally, to compute the last three angles, we have the following equation:

![alt text][image9]


The right hand side of this equation can be computed given the orientation of the gripper (R0_6), and the first three joint angles (after we solve for them given the rotation of the gripper). Then, given the symbolic form of R3_6 (extracted from the symbolic form of T3_6), we can solve for the last three joint as shown below:


```
        T3_6 = simplify(T3_4 * T4_5 * T5_6)
        R3_6 = T3_6[0:3, 0:3]
        # Print the symbolic form of R3_6, which will be used to compute the last three joints:
        print("R3_6 = ", R3_6)
        # The result of the print will be the following:
        'R3_6 = ', Matrix([
        [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
        [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
        [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]]))
        # Solving the equations, we find the following, where rij's are elements of the above matrix.
        q4 = atan2(r33, -r13)
        q6 = atan2(-r22, r21)
        q5 = atan2(sqrt(r13^2 + r33^2), r23)
```

Note that if theta5 = 0, then we have a singularity, and there are infinite ways to satisfy theta4 and theta6, as we will have a constraint on their sum. This would be a singularity, and the above equations will not produce valid values for q4 and q6. In that case, R3_6 can be simplified as below:

```
        'R3_6 = ', Matrix([
        [-sin(q4)*sin(q6) + cos(q4)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4),          0],
        [                                 0,                                  0,          1],
        [-sin(q4)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6) - cos(q4)*cos(q6),          0]]))
        # Further simplifying we have:
        'R3_6 = ', Matrix([
        [ cos(q4+q6),  -sin(q4+q6),          0],
        [          0,            0,          1],
        [-sin(q4+q6),  -cos(q4+q6),          0]]))
        # which gives us the following:
        q5 = 0
        q4+q6 = atan2(-r12, r11)
```

In this case, the choice of q4 and q6 is rather arbitrary, as long as their sum has the correct value. When we get into this situation, we can keep one of the two angles fixed (retaining its value from the previous step), and change the other one, or use some other strategy to change the two together. 

I use the joint values of the previous iteration, and also valid joint limits, to guide the choice of values for the next iteration of the algorithm, to reduce the movement of each joint. For example, if the value of a joint was `0` in the previous cycle, and I compute `2*pi - epsilon` for it in the next cycle, I replace it with `-epsilon`. Similarly, if and when `theta5 = 0`, I use a series of tests to choose values for `theta4` and `theta6`. Please see documentation in the code for more details.


### Project Implementation

#### 1. Fill in the `IK_server.py` file with properly commented python code for calculating Inverse Kinematics based on previously performed Kinematic Analysis. Your code must guide the robot to successfully complete 8/10 pick and place cycles. Briefly discuss the code you implemented and your results. 

My code for `IK_server.py` is well documented, and includes much of the points that I have discussed in the previous sections of this write up. The code achieves a high rate of success (better than 8/10 rate, rarely failing). I have solved the IK problem assuming an elbow up configuration for the robot similar to the one suggested in the lectures. Once I compute values for the six joints, I check that they are all within their corresponding joint limits, and if not, I adjust them (by adding or subtracting 2*pi). 

Looking at the joint limits of Kuka, one notices that some of its joints can reach an angle either making a rotation in the positive direction, or in the negative direction. That is, there is some redundancy. I have leveraged this redundancy to minimize the magnitude of joint movements over subsequent points along a requested path. That is, if to reach from a point to the next, a given joint can move in either of the its two positive or negative directions, I choose the one that results in a smaller move. 

Still, if a requested path has points on it that are not reachable with the given elbow up configuration (or are just out of the workspace of the robot), then the implementation may/will not generate correct results, and thus would fail for such points (leading to failure in grasping the object, or dropping it early, etc).

Before addressing the singularities of joints 4 and 6, sometimes I would see the robot do a lot of seemingly unnecessary rotations around those joints along its path. By handling the singularities, and also checking for joint limits, I have reduced such incidents. They can stil happen at the very beginning of a requested path, or infrequently in the middle of a path, because I guess the issue is hard to perfectly resolve, and/or likely there are more robust solutions than the one I have implemented.


Finally, the following is a snapshot of a perfect 10/10 pick&place run:

![alt text][image10]

