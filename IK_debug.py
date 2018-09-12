from sympy import *
from time import time
from mpmath import radians
import tf

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
              4:[],
              5:[]}

def correct_theta(theta):
    theta_small = theta - 2*pi
    theta_big = theta + 2*pi
    theta_min = min(abs(theta_small), abs(theta_big))
    theta_min = min(abs(theta), theta_min)
    if theta_min == abs(theta):
	return theta
    elif theta_min == abs(theta_small):
	return theta_small
    else:
	return theta_big

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

    ## Insert FK here!
    ## (OPTIONAL) YOUR CODE HERE!
    if True:
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i   
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') 
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        q = symbols('q')
        d = symbols('d')
        a = symbols('a')
        alpha = symbols('alpha')


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

        T3_4 = Ti.subs({q: q4, d: d4, alpha: alpha3, a: a3})
        T3_4 = T3_4.subs(s_list)

        T4_5 = Ti.subs({q: q5, d: d5, alpha: alpha4, a: a4})
        T4_5 = T4_5.subs(s_list)

        T5_6 = Ti.subs({q: q6, d: d6, alpha: alpha5, a: a5})
        T5_6 = T5_6.subs(s_list)

        T6_G = Ti.subs({q: q7, d: d7, alpha: alpha6, a: a6})
        T6_G = T6_G.subs(s_list)

        T0_2 = simplify(T0_1 * T1_2) # base_link to link_2
        T0_3 = simplify(T0_2 * T2_3) # base_link to link_3
        T0_4 = simplify(T0_3 * T3_4) # base_link to link_4
        T0_5 = simplify(T0_4 * T4_5) # base_link to link_5
        T0_6 = simplify(T0_5 * T5_6) # base_link to link_6
        T0_G = simplify(T0_6 * T6_G) # base_link to end effector


        R_z = Matrix([[     cos(pi),   -sin(pi),             0],
                      [     sin(pi),    cos(pi),             0],
                      [           0,          0,             1]])
        R_y = Matrix([[  cos(-pi/2),          0,    sin(-pi/2)],
                      [           0,          1,             0],
                      [ -sin(-pi/2),          0,    cos(-pi/2)]])
        R_corr = simplify(R_z * R_y)

        # Total homogeneous transformation between base_link and gripper_link
        # with orientation correction applied
        T_corr = R_corr.row_join(Matrix([0, 0, 0])).col_join(Matrix([[0, 0, 0, 1]]))
        T_total = simplify(T0_G * T_corr)
  
        # Extract rotation matrices from the transformation matrices
        R0_3 = T0_3[0:3, 0:3]

        # T3_6 = simplify(T3_4 * T4_5 * T5_6)
        # R3_6 = T3_6[0:3, 0:3]

    ## Insert IK code here!
        if True:
    
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

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

            ### Your IK code here
            # Compensate for rotation discrepancy between DH parameters and Gazebo
            R_rpy_eval = R_rpy.evalf(subs={_yaw: yaw, _pitch: pitch, _roll: roll})
            EE_Z = R_rpy_eval[0:3, 2] # Z axis of EE
            P_EE = Matrix([px, py, pz])
            WC_0 = P_EE - s[d7] * EE_Z # Wrist coordinate w.r.t the base_link
            WC_x = WC_0[0]
            WC_y = WC_0[1]
            WC_z = WC_0[2]
            # Calculate joint angles using Geometric IK method
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
            theta2 = pi/2 - gama - a  # check the direction of theta1/2/3 in Rviz
            B2 = B - B1
            c = atan2(D, B2)
            b = pi - c - a
            # Notice that since a3 is negative, we have to take its absolute to make it work in the formula below.
            theta3 = - (b - pi/2 + atan2(abs(s[a3]), s[d4])) # this may have a +/- multiplier. Check the direction of angles in rviz.

            ###
            R0_3_eval = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6_eval = R0_3_eval.T * R_rpy_eval

            r33 = R3_6_eval[2, 2]
            r13 = R3_6_eval[0, 2]
            r22 = R3_6_eval[1, 1]
            r21 = R3_6_eval[1, 0]
            r23 = R3_6_eval[1, 2]
            theta4 = atan2(r33, -r13)
            theta6 = atan2(-r22, r21)
            theta5 = atan2(sqrt(r13 * r13 + r33 * r33), r23)

    ## 
    ########################################################################################
    
    ########################################################################################
    ## For additional debugging add your forward kinematics here. Use your previously calculated thetas
    ## as the input and output the position of your end effector as your_ee = [x,y,z]

    ## (OPTIONAL) YOUR CODE HERE!
    T_total_eval = T_total.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    T0_6_eval = T0_6.evalf(subs={q1: theta1, q2: theta2, q3: theta3, q4: theta4, q5: theta5, q6: theta6})
    ## End your code input for forward kinematics here!
    ########################################################################################

    ## For error analysis please set the following variables of your WC location and EE location in the format of [x,y,z]
    your_wc = [T0_6_eval[0, 3], T0_6_eval[1, 3], T0_6_eval[2, 3]] # <--- Load your calculated WC values in this array
    your_ee = [T_total_eval[0, 3], T_total_eval[1, 3], T_total_eval[2, 3]] # <--- Load your calculated end effector value from your forward kinematics
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
    t_2_e = abs(theta2-test_case[2][1])
    t_3_e = abs(theta3-test_case[2][2])
    t_4_e = abs(theta4-test_case[2][3])
    t_5_e = abs(theta5-test_case[2][4])
    t_6_e = abs(theta6-test_case[2][5])
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
    test_case_number = 3

    test_code(test_cases[test_case_number])
