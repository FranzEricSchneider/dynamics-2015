# UR5 inverse kinematics paper
# smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf
# 3R planar manipulator IK paper
# http://www.seas.upenn.edu/~meam520/notes/planar.pdf

import numpy as np
from numpy.linalg import inv
import rospy 
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from forward_kinematics import UR5ForwardKinematics
from ur5_model.msg import JointAngles


def main():
    rospy.init_node('test_inverse_kinematics')
    pub = rospy.Publisher('/ur5_model/joint_command', JointAngles, queue_size=10)
    pose_debug_pub = rospy.Publisher('effector_pose', Marker, queue_size=1)
    rospy.sleep(1.0)
    IK = InverseKinematics()
    # These variables are used to make the test path of the arm
    counter = 0
    scalar = 10.0
    radius = 0.6
    while not rospy.is_shutdown():
        effector_pose = InitialEffectorPose(radius * np.sin(counter / scalar),
                                            radius * np.cos(counter / scalar),
                                            radius * np.cos(counter / scalar) / 2,
                                            0, counter * np.pi/32, 0)
        # This publisher publishes a marker pointing to where the end effector should be
        pose_debug_pub.publish(create_effector_marker(effector_pose.px, effector_pose.py, effector_pose.pz))
        # Calculates all eight joint permutations given the effector pose
        joint_permutations = IK.calculate_joints(effector_pose)
        for permutation in joint_permutations:
            pub.publish(permutation)
            rospy.sleep(0.25)
        rospy.loginfo('Published an effector pose')
        counter += 1


class InverseKinematics():
    def __init__(self):
        self.FK = UR5ForwardKinematics()
        pass

    def calculate_joints(self, effector_pose):
        '''
        Calculates the joint possibilities for the UR5 arm as laid out in the UR5 inverse kinematics paper, then
        returns all possible permutations of those joint commands
        '''
        theta1 = self.calculate_theta_1(effector_pose)
        theta5 = self.calculate_theta_5(effector_pose, theta1)
        theta6 = self.calculate_theta_6(effector_pose, theta1, theta5)
        theta2, theta3, theta4 = self.calculate_theta_234(effector_pose,
                                                          theta1, theta5, theta6)
        return(self.create_joint_permutations(theta1, theta2, theta3, theta4, theta5, theta6))

    def calculate_theta_1(self, effector_pose):
        '''
        -- Calculates the theta1 angle as laid out in the UR5 inverse kinematics paper, by stepping from the 06 origin
        back to the 05 origin and then using the knowledge that with the UR5 geometry the length d4 will always
        be parallel to y1
        '''
        # Gets the global distance between the 05 and 06 origins by taking a d6 vector in the 06 frame and using the
        # R06 rotation matrix to express that vector in the global 00 frame
        offset56 = inv(effector_pose.R06) * np.matrix([0, 0, self.FK.d6]).transpose()
        # The global location of the 05 origin
        p05x = effector_pose.px - offset56[0, 0]
        p05y = effector_pose.py - offset56[1, 0]
        p05z = effector_pose.pz - offset56[2, 0]
        # The distance from global origin to the 05 origin, in the global (x, y) plane
        R = np.sqrt( p05x**2 + p05y**2 )
        # Implements the math in the paper, doesn't have special meaning in an of itself
        alpha1 = np.arctan2( p05y, p05x )
        alpha2 = np.arccos( self.FK.d4 / R )
        # Returns the two possible theta1 values
        return( [alpha1 + alpha2 + np.pi / 2, alpha1 - alpha2 + np.pi / 2] )

    def calculate_theta_5(self, effector_pose, theta1):
        '''
        -- Calculates the theta5 angle as laid out in the UR5 inverse kinematics paper, by measuring how much the y5
        axis is lined up with the y1 axis
        -- Assumes that the theta1 passed in is a list with two angles (floats) inside
        '''
        # Implements the math in the paper, doesn't have special meaning in an of itself
        numerator = effector_pose.px * np.sin(theta1) - effector_pose.py * np.cos(theta1) - self.FK.d4
        abs_theta5 = np.arccos( numerator * np.sign(self.FK.d6) / self.FK.d6 )
        # Returns the four possible theta5 values
        return( [ [abs_theta5[0], -abs_theta5[0]], [abs_theta5[1], -abs_theta5[1]] ] )

    def calculate_theta_6(self, effector_pose, theta1, theta5):
        '''
        -- Calculates the theta6 angle as laid out in the UR5 inverse kinematics paper
        -- Assumes that the theta1 passed in is a list with two angles (floats) inside and that the theta5 passed in
        has four angles and is a list of lists of floats like so: [[a, b],[c, d]]
        '''
        th6 = []
        for th1, th5 in zip(theta1, theta5):
            # The paper walks through how these statements solve for theta6 as part of a spherical joint with theta5
            y_term = (- effector_pose.DH06[0, 1] * np.sin(th1) + effector_pose.DH06[1, 1] * np.cos(th1))
            x_term = -(- effector_pose.DH06[0, 0] * np.sin(th1) + effector_pose.DH06[1, 0] * np.cos(th1))
            th6.append([np.arctan2(y_term / np.sign(np.sin(th5[0])), x_term / np.sign(np.sin(th5[0]))),
                        np.arctan2(y_term / np.sign(np.sin(th5[1])), x_term / np.sign(np.sin(th5[1])))])
        # Returns the four possible theta6 values
        return(th6)

    def calculate_theta_234(self, effector_pose, global_theta1, global_theta5, global_theta6):
        '''
        -- Calculates the theta2/3/4 angles as laid out in the separate 3R planar manipulator IK paper. All variables
        are named to match those in the 3R paper, for easier reading, so theta1/2/3 are solved for (because that's
        what's in the paper) even though on the UR5 arm those are angles theta2/3/4
        -- Assumes that the theta1 passed in is a list with two angles (floats) inside and that the theta5/6 passed in
        have four angles in a list of lists of floats like so: [[a, b],[c, d]]
        '''
        # These are empty variables that will be populated in the for loop. Each angle will eventually have 8 values
        theta1 = [[[0, 0], [0, 0]], [[0, 0], [0, 0]]]
        theta2 = [[[0, 0], [0, 0]], [[0, 0], [0, 0]]]
        theta3 = [[[0, 0], [0, 0]], [[0, 0], [0, 0]]]
        for i in range(2):
            for j in range(2):
                # Computes the transformation matrix from the 01 frame to the 04 frame. We get this transformation
                # matrix by starting from DH06 and canceling the transformations that we've already calculated the
                # angles for. We know what DH06 is from the origin effector pose, and we know that in theory DH06 is
                # equal to [DH56 * DH45 * DH34 * DH23 * DH12 * DH01]
                DH14 = inv(self.FK.DH45(global_theta5[i][j])) *\
                       inv(self.FK.DH56(global_theta6[i][j])) *\
                       effector_pose.DH06 *\
                       inv(self.FK.DH01(global_theta1[i]))
                # We want the translation from the 01 origin to the 04 origin, and the translation in DH14 is from the
                # 04 origin to 01 origin, so we can use the rotation matrix from 01 to 04 to get what we want
                translation14 = inv(DH14[0:3, 0:3]) * -DH14[0:3, 3]
                # x_prime and y_prime represent the second joint, in our case the 04 origin. See paper for details
                x_prime = translation14[0, 0]
                y_prime = translation14[2, 0]
                # R_prime is the planar distance from the 01 origin to the second joint (04 origin)
                R_prime = np.sqrt( x_prime**2 + y_prime**2 )
                # R14_planar allows us to calculate the angle from x1 to d5, which is necessary in order to solve the
                # 3R system. It is not strictly the 01 to 04 rotation matrix. There is a pi/2 rotation around the x1
                # axis to make z1 parallel to the 3R plane. There is also a -pi/2 rotation around z4 to account for the
                # fact that d5 is offset from x4 by -pi/2 radians
                R14_planar = np.matrix([[np.cos(-np.pi/2),  np.sin(-np.pi/2), 0],
                                        [-np.sin(-np.pi/2), np.cos(-np.pi/2), 0],
                                        [0,                 0,                1]]) *\
                             DH14[0:3, 0:3] *\
                             inv(np.matrix([[1, 0, 0],
                                            [0, np.cos(np.pi/2),  np.sin(np.pi/2)],
                                            [0, -np.sin(np.pi/2), np.cos(np.pi/2)]]))
                # Phi is the angle of d5 measured against x1, and is calculated using the planar rotation matrix that
                # goes from 01 to d5. See 3R paper for details
                phi = np.arccos(R14_planar[0, 0]) * np.sign(R14_planar[0, 1])
                # These L variables aren't strictly necessary, but they match the paper and make it easier to read
                l1 = self.FK.a2
                l2 = self.FK.a3
                l3 = self.FK.d5
                # lambda is a temporary variable in the paper, I made up alpha to use as another temporary variable
                val_lambda = np.arctan2( -y_prime / R_prime, -x_prime / R_prime )
                val_alpha = np.arccos( -(x_prime**2 + y_prime**2 + l1**2 - l2**2) / (2 * l1 * R_prime) )
                # Two theta2/3/4 values are calculated for each theta1/5/6 combination, elbow up/down configurations
                theta1[i][j] = [val_lambda + val_alpha, val_lambda - val_alpha]
                theta2[i][j] = [np.arctan2((y_prime - l1 * np.sin(th1)) / l2,
                                           (x_prime - l1 * np.cos(th1)) / l2) - th1 for th1 in theta1[i][j]]
                # The pi/2 term is added to reflect the fact that d5 is -pi/2 off from x4 and phi was calculated using
                # the direction d5 was pointing in
                theta3[i][j] = [phi + np.pi/2 - (th1 + th2) for th1, th2 in zip(theta1[i][j], theta2[i][j])]
        return(theta1, theta2, theta3)

    def create_joint_permutations(self, th1, th2, th3, th4, th5, th6):
        '''
        -- Takes the calculated joint angles and creates all possible permutations, b/c the arm can generally reach a
        given position in multiple ways
        -- Assumes (based on the paper) that there are two theta1 possibilities, two theta5/6 possibilities for each
        theta1, and two theta2/3/4 possibilities for each theta5/6 value. In the ideal case, where the arm can reach
        the desired point in all orientations, these values will all exist. When the arm can't reach some or all of
        these permutations the code might break, but I'm not sure if we'll deal with those cases
        '''
        permutations = []
        for i in range(2):
            for j in range(2):
                for k in range(2):
                    permutations.append(JointAngles([th1[i],
                                                     th2[i][j][k],
                                                     th3[i][j][k],
                                                     th4[i][j][k],
                                                     th5[i][j],
                                                     th6[i][j]]))
        return(permutations)


class InitialEffectorPose():
    def __init__(self, x, y, z, phi, theta, psi):
        '''
        -- Takes in the desired (x,y,z) global position of the effector, and the desired 313 Euler angles specifying
        the desired starting orientation of the effector
        -- Contains the position, the rotation matrix, and the Denavit-Hartenberg transformation matrix from the 00
        reference frame to the 06 reference frame
        '''
        # Global position of the effector
        self.px = x
        self.py = y
        self.pz = z
        # 313 Euler angles for the effector
        self.phi = phi
        self.theta = theta
        self.psi = psi
        # Used to calculate the rotation matrix from 00 reference frame to the 06 reference frame
        Rphi = np.matrix([[np.cos(phi),  np.sin(phi), 0],
                        [-np.sin(phi), np.cos(phi), 0],
                        [0,            0,           1]])
        Rtheta = np.matrix([[1, 0,              0],
                          [0, np.cos(theta),  np.sin(theta)],
                          [0, -np.sin(theta), np.cos(theta)]])
        Rpsi = np.matrix([[np.cos(psi),  np.sin(psi), 0],
                        [-np.sin(psi), np.cos(psi), 0],
                        [0,            0,           1]])
        # Rotation matrix from 0 reference frame to the 6 reference frame
        self.R06 = Rpsi * Rtheta * Rphi
        # Components used to build up the 00->06 transformation matrix
        # In a Denavit-Hartenberg matrix the translation portion is from the goal origin (in this case the 06 origin)
        #   to the base origin (in this case the 00 origin), IN THE GOAL REFERENCE FRAME (in this case the 06
        #   reference frame). That means that b/c we are given the global (x,y,z) coordinates, in order to build the
        #   Denavit-Hartenberg matrix we need to invert the vector and express it in the 06 frame using the R06 matrix
        translation = self.R06 * -np.matrix([x, y, z]).transpose()
        bottom_layer = np.matrix([0, 0, 0, 1])
        # The 4x4 Denavit-Hartenberg transformation matrix from the 00 reference frame to the 06 reference frame
        self.DH06 = np.vstack( [np.hstack([self.R06, translation]), bottom_layer] )


def create_effector_marker(x, y, z):
    '''
    Creates a single orange marker in the workspace of the arm pointing at an x, y, z point
    '''
    marker = Marker()
    marker.header.frame_id = 'ur5/base_link'
    marker.header.stamp = rospy.Time.now()
    marker.ns = ''
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD;
    marker.points = []
    marker.points.append( Point() )
    marker.points.append( Point(x=-x, y=-y, z=z) )
    marker.scale.x = 0.03;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.6;
    return(marker)


if __name__ == '__main__':
    main()
