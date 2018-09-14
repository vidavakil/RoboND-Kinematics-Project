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

# Use this flag to enable FK and measuring of gripper position error.
# Note that FK significantly lowers the speed!
enable_FK = False

def simplify_theta(index, theta):
    # Given an input angle theta, return an equivalent angle that has
    # the smallest possible absolute size.
    new_theta = theta
    theta_small = theta - 2*pi
    theta_big = theta + 2*pi
    theta_min = min(abs(theta_small).evalf(), abs(theta_big).evalf())
    theta_min = min(abs(theta), theta_min.evalf())
    if theta_min == abs(theta_small).evalf():
        new_theta = theta_small
    elif theta_min == abs(theta_big).evalf():
        new_theta = theta_big
    else:
        return theta
    # print "For step " + str(index) + " changed theta from" \
    #    + str(theta) + " to new_theta = " + str(new_theta)
    return new_theta

def optimize(index, new_positions, previous_positions, lower, upper):
    # Given new and old values for the 6 joints and their lower and
    # upper limits, compute valid alternative values for the new
    # joint values that minimize joint movements and are within range.
    theta = []
    changed = False
    for idx, new_theta in enumerate(new_positions):
        old_theta = previous_positions[idx]
        diff = abs(new_theta - old_theta).evalf()
        big_diff = abs(new_theta + 2*pi - old_theta).evalf()
        small_diff = abs(new_theta - 2*pi - old_theta).evalf()
        best_diff = min(diff, small_diff)
        best_diff = min(best_diff, big_diff)
        if best_diff == small_diff:
            new_theta = new_theta - 2*pi
            changed = True
        elif best_diff == big_diff:
            new_theta = new_theta + 2*pi
            changed = True
        else:
            new_theta = new_theta
        if new_theta < lower[idx]:
            new_theta = new_theta + 2*pi
            changed = True
        elif new_theta > upper[idx]:
            new_theta = new_theta - 2*pi
            changed = True
        theta.append(new_theta)
    # if changed:
    #   print "For step " + str(index) + " changed theta"
    #   print(previous_positions)
    #   print(new_positions)
    #   print(theta)
    #   print "\n"
    return theta


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:

        ### Your FK code here
        # Create symbols for joint variables
        T3_4 = T4_5 = T5_6 = T6_G = 0
        T0_4 = T0_5 = T0_6 = T0_G = 0
        forwards_p = 0

        # Joint ranges:
        # Note that joints 1, 4, and 6 can reach some angles from two
        # directions. For joint 1, the overlap is 10 degrees. For joints
        # 4 and 6, the overlap is 335 degrees!
        lower = [-185, -45, -210, -350, -125, -350]
        upper = [ 185,  85,   65,  350,  125,  350]

        lower = [i * pi/180 for i in lower]
        upper = [i * pi/180 for i in upper]

        # qi (joint angle) = angle between Xi-1 to Xi measured about Zi
        #    in a right handed sense.
        # di (link offset) = signed distance between Xi-1 to Xi ,
        #    measured along Zi.
        # ai-1 (link length) = distance from Zi-1 to Zi measured along
        #    Xi-1. Xi-1 is perpendicular to both Zi-1 and Zi.
        # alphai-1 (twist angle) = angle between Zi-1 and Zi measured
        #    about Xi-1, in a right hand sense.
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        q = symbols('q')
        d = symbols('d')
        a = symbols('a')
        alpha = symbols('alpha')

        # Create Modified DH parameters for KR210
        # a0 = alpha0 = 0, since Z0 and Z1 in DH are coincident.
        # d1 = joint_1.z + joint_2.z
        # a1 = joint_2.x
        # alpha1 = -pi/2, based on the DH diagram
        # d2 = 0, since X1 and X2 cross each other at O2 of DH
        # q2_DH = q2_URDF - pi/2. Because while the DH diagram shows
        #    a joint angle of -pi/2, the corresponding joint anlge
        #    of the robot is actually 0.
        # Since X2, X3, X4, X5, X6, and XG in the DH diagram are all
        #    parallel when robot is in its reset configuraation
        #    (according to URDF file), then in that configuration
        #    we have q3 = q4 = q5 = q6 = 0, which matches the reset
        #    config. SO, unlike the case of q2, there is no offset
        #    between these qs in URDF versus the DH coordinates.
        # a2 = joint_3.z
        # alpha2 = 0, because Z2 and Z3 of DH are parallel
        # d3 = 0, since X2 and X3 of DH are aligned
        # a3 = joint_4.z
        # alpha3 = -pi/2, based on the DH diagram
        # d4 = joint_4.x + joint_5.x
        # a4 = 0, because Z4 and Z5 of DH cross each other at O4 = O5
        # alpha4 = pi/2, based on the DH diagram
        # d5 = 0, because O4 and O5 are coincident in the DH diagram
        # a5 = 0, because Z5 and Z6 cross each other at O5 = O6
        # alpha5 = -pi/2, based on the DH diagram
        # d6 = 0,  because X5 and X6 cross each other at O5 = O6
        # a6 = 0, because Z6 and ZG cross each other at OG
        # alpha6 = 0, based on the DH diagram
        # d7 =  joint_6.x + gripper_joint.x
        # q7 = 0, and is fixed, according to the URDF file

        s = {alpha0:     0,  a0:      0,  d1:  0.75,
             alpha1: -pi/2,  a1:   0.35,  d2:     0,  q2: q2 - pi/2,
             alpha2:     0,  a2:   1.25,  d3:     0,
             alpha3: -pi/2,  a3: -0.054,  d4:  1.50,
             alpha4:  pi/2,  a4:      0,  d5:     0,
             alpha5: -pi/2,  a5:      0,  d6:     0,
             alpha6:     0,  a6:      0,  d7: 0.303,  q7: 0}

        # Define Modified DH Transformation matrix
        Ti = Matrix([[            cos(q),           -sin(q),           0,             a],
                     [ sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                     [ sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                     [                 0,                 0,           0,             1]])

        # Create individual transformation matrices
        s_list = s.items()
        T0_1 = Ti.subs({q: q1, d: d1, alpha: alpha0, a: a0})
        T0_1 = T0_1.subs(s_list)

        T1_2 = Ti.subs({q: q2, d: d2, alpha: alpha1, a: a1})
        T1_2 = T1_2.subs(s_list)

        T2_3 = Ti.subs({q: q3, d: d3, alpha: alpha2, a: a2})
        T2_3 = T2_3.subs(s_list)

        # Since we don't need the rest of Ti_j transforms for IK,
        # comment below lines.
        if enable_FK:
            T3_4 = Ti.subs({q: q4, d: d4, alpha: alpha3, a: a3})
            T3_4 = T3_4.subs(s_list)

            T4_5 = Ti.subs({q: q5, d: d5, alpha: alpha4, a: a4})
            T4_5 = T4_5.subs(s_list)

            T5_6 = Ti.subs({q: q6, d: d6, alpha: alpha5, a: a5})
            T5_6 = T5_6.subs(s_list)

            T6_G = Ti.subs({q: q7, d: d7, alpha: alpha6, a: a6})
            T6_G = T6_G.subs(s_list)

            print("T0_1 = ", T0_1)
            print("T1_2 = ", T1_2)
            print("T2_3 = ", T2_3)
            print("T3_4 = ", T3_4)
            print("T4_5 = ", T4_5)
            print("T5_6 = ", T5_6)
            print("T6_G = ", T6_G)

        T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
        T0_3 = simplify(T0_2 * T2_3) # base_link to link_3

        # Since we don't need the rest of T0_i transforms for IK,
        # comment below lines.
        if enable_FK:
            T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
            T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
            T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
            T0_G = simplify(T0_6 * T6_G) # base_link to end effector
            forwards_p = T0_G[0:3, 3]

        # The gripper position based on forwards kinematics

        # Correction needed to account for orentation difference
        # between the definition of gripper_link in URDF versus
        # our DH convention

        R_z = Matrix([[     cos(pi),   -sin(pi),             0],
                      [     sin(pi),    cos(pi),             0],
                      [           0,          0,             1]])
        R_y = Matrix([[  cos(-pi/2),          0,    sin(-pi/2)],
                      [           0,          1,             0],
                      [ -sin(-pi/2),          0,    cos(-pi/2)]])
        R_corr = simplify(R_z * R_y)

        # Numerically evaluate transforms, and compare with the
        # output of tf_echo:
        # print("T0_1 = ", T0_1.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
        # print("T0_2 = ", T0_2.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
        # print("T0_3 = ", T0_3.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
        # print("T0_4 = ", T0_4.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
        # print("T0_5 = ", T0_5.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
        # print("T0_6 = ", T0_6.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))
        # print("T0_G = ", T0_G.evalf(subs={q1: 0, q2: 0, q3:0, q4:0, q5:0, q6:0}))

        # Total homogeneous transformation between base_link and
        # gripper_link with orientation correction applied.
        # Since we don't need T_total for IK, comment below lines.
        # T_corr = R_corr.row_join(Matrix([0, 0, 0])).col_join(Matrix([[0, 0, 0, 1]]))
        # T_total = simplify(T0_G * T_corr)
        # print("R_corr = ", R_corr)
        # print("T_total = ", T_total)

        # Extract rotation matrices from the transformation matrices
        R0_3 = T0_3[0:3, 0:3]
        # Print the symbolic form of R0_3
        # print("R0_3 = ", R0_3)

        # T3_6 = simplify(T3_4 * T4_5 * T5_6)
        # R3_6 = T3_6[0:3, 0:3]
        # Print the symbolic form of R3_6, which will be used to compute
        # the last three joints
        # print("R3_6 = ", R3_6)
        # 'R3_6 = ', Matrix([
        # [-sin(q4)*sin(q6) + cos(q4)*cos(q5)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4)*cos(q5), -sin(q5)*cos(q4)],
        # [                           sin(q5)*cos(q6),                           -sin(q5)*sin(q6),          cos(q5)],
        # [-sin(q4)*cos(q5)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6)*cos(q5) - cos(q4)*cos(q6),  sin(q4)*sin(q5)]]))
        # Solving the equations, we find the following
        # q4 = atan2(r33, -r13)
        # q6 = atan2(-r22, r21)
        # q5 = atan2(sqrt(r13^2 + r33^2), r23)
        #
        # Note that if q5 == 0, then we have a singularity, and the
        # above equations will not produce valid values for q4 and q6.
        # In that case, R3_6 can be simplified as below:
        # 'R3_6 = ', Matrix([
        # [-sin(q4)*sin(q6) + cos(q4)*cos(q6), -sin(q4)*cos(q6) - sin(q6)*cos(q4),          0],
        # [                                 0,                                  0,          1],
        # [-sin(q4)*cos(q6) - sin(q6)*cos(q4),  sin(q4)*sin(q6) - cos(q4)*cos(q6),          0]]))
        #
        # 'R3_6 = ', Matrix([
        # [ cos(q4+q6),  -sin(q4+q6),          0],
        # [          0,            0,          1],
        # [-sin(q4+q6),  -cos(q4+q6),          0]]))
        # which gives us the following:
        # q5 = 0
        # q4+q6 = atan2(-r12, r11)
        #
        # In this case, the choice of q4 and q6 is rather arbitrary,
        # as long as their sum is right. During trajectory planning,
        # if we get into this situation, we can keep one of them fixed
        # (retaining its value from the previous step), and change the
        # other one, or use some other strategy to change the two together.

        # Compute symbolic form of R_rpy, to be later used in IK
        _yaw   = symbols('yaw')
        _pitch = symbols('pitch')
        _roll  = symbols('roll')
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


        # Initialize service response
        joint_trajectory_list = []
        previous_thetas = []
        p_error = []
        for index in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[index].position.x
            py = req.poses[index].position.y
            pz = req.poses[index].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[index].orientation.x, req.poses[index].orientation.y,
                    req.poses[index].orientation.z, req.poses[index].orientation.w])

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_rpy_eval = R_rpy.evalf(subs={_yaw: yaw, _pitch: pitch, _roll: roll})
            EE_Z = R_rpy_eval[0:3, 2] # Z axis of EE
            P_EE = Matrix([px, py, pz])
            WC_0 = P_EE - s[d7] * EE_Z # Wrist coordinate w.r.t the base_link
            WC_x = WC_0[0]
            WC_y = WC_0[1]
            WC_z = WC_0[2]
            # Calculate joint angles using Geometric IK method.
            # Validated joint direction using IK_debug.py. Could have used rViz.
            theta1 = atan2(WC_y, WC_x)
            y = WC_z - s[d1]
            x = sqrt(WC_x * WC_x + WC_y * WC_y) - s[a1]
            B_2 = y * y + x * x
            B = sqrt(B_2)
            A_2 = s[d4] * s[d4] + s[a3] * s[a3]
            A = sqrt(A_2)
            C = s[a2]
            gama = atan2(y, x)
            B1 = (C * C - A_2 + B_2) / (2 * B)
            D = sqrt(C * C - B1 * B1)
            a = atan2(D, B1)
            theta2 = pi/2 - gama - a
            B2 = B - B1
            c = atan2(D, B2)
            b = pi - c - a
            # Notice that since a3 is negative, we have to take its
            # absolute to make it work in the formula below.
            theta3 = -(b - pi/2 + atan2(abs(s[a3]), s[d4]))

            ###
            R0_3_eval = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6_eval = R0_3_eval.T * R_rpy_eval

            r33 = R3_6_eval[2, 2]
            r13 = R3_6_eval[0, 2]
            r22 = R3_6_eval[1, 1]
            r21 = R3_6_eval[1, 0]
            r23 = R3_6_eval[1, 2]
            theta5 = atan2(sqrt(r13 * r13 + r33 * r33), r23)
            # If theta5 == 0, then we have a singularity and have to
            # choose theta4 and theta6 so that they satisfy a sum.
            if (theta5 != 0):
                theta4 = atan2(r33, -r13)
                theta6 = atan2(-r22, r21)
            else:
                r12 = R3_6_eval[0, 1]
                r11 = R3_6_eval[0, 0]
                theta4_6 = atan2(-r12, r11) # q4+q6

                # Adjust theta4_6
                theta4_6 = simplify_theta(theta4_6)
                if theta4_6 > upper[5]:
                    theta4_6 -= 2*pi
                if theta4_6 < lower[5]:
                    theta4_6 += 2*pi

                # Choose values for theta4 and theta6, based on whether
                # this is the start of the path, or the middle of it
                if index == 0:
                    theta4 = 0
                    theta6 = theta4_6
                else:
                    old_theta4 = previous_thetas[3]
                    old_theta6 = previous_thetas[5]
                    old_theta4_6 = old_theta4 + old_theta6
                    diff = theta4_6 - old_theta4_6

                    # Try assigning half of the change in the sum of
                    # theta4 and theta6 since last step to each of them,
                    # but account for joint limits.
                    theta4 = max(min(old_theta4 + diff/2, upper[3]), lower[3])
                    new_diff = diff - (theta4 - old_theta4)
                    theta6 = old_theta6 + new_diff

                    # Adjust theta6 one more time.
                    if theta6 > upper[5]:
                        theta6 -= 2*pi
                    elif theta6 < lower[5]:
                        theta6 += 2*pi


            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            new_thetas = [theta1, theta2, theta3, theta4, theta5, theta6]
            if (index != 0):
                new_thetas = optimize(index, new_thetas, previous_thetas, lower, upper)
            joint_trajectory_point.positions = list(new_thetas)
            joint_trajectory_list.append(joint_trajectory_point)
            previous_thetas = list(new_thetas)

            # Compute error, if FK is enabled.
            if enable_FK:
                forwards_p_eval = forwards_p.evalf(subs={q1: theta1, q2: theta2,
                    q3:theta3, q4: theta4, q5: theta5, q6: theta6})
                x_error = px - forwards_p_eval[0]
                y_error = py - forwards_p_eval[1]
                z_error = pz - forwards_p_eval[2]
                p_error.append(sqrt(x_error * x_error + y_error * 
                               y_error + z_error * z_error))
                print "Error in position at step " + str(index) + " is " \
                    + str(p_error[index])

        # Compute average error over the whole sequence.
        if enable_FK:
            print "Average position error over " + str(len(p_error)) \
                + " steps is " + str(sum(p_error) / float(len(p_error)))
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


