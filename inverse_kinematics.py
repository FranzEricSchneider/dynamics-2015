# UR5 inverse kinematics paper
# smartech.gatech.edu/bitstream/handle/1853/50782/ur_kin_tech_report_1.pdf
# 3R planar manipulator IK paper
# http://www.seas.upenn.edu/~meam520/notes/planar.pdf

import numpy as np
from numpy.linalg import inv
import rospy 

from forward_kinematics import UR5ForwardKinematics
from ur5_model.msg import JointAngles


def main():
    IK = InverseKinematics()
    # effector_pose = InitialEffectorPose(0.8, 0, 0,
    #                                     0, 0, 0)
    effector_pose = InitialEffectorPose(IK.FK.a2 + IK.FK.a3,
                                        -IK.FK.d4 - IK.FK.d6,
                                        IK.FK.d1 - IK.FK.d5,
                                        0, np.pi/2, 0)
    IK.calculate_joints(effector_pose)
    # pub = rospy.Publisher('/ur5_model/joint_command', JointAngles, queue_size=10)
    # rospy.init_node('test_inverse_kinematics')
    # rospy.sleep(1.0)
    # a = JointAngles([theta1[0], 0, 0, 0, theta5[0][0], theta6])
    # pub.publish(a)
    # rospy.spin()


class InverseKinematics():
    def __init__(self):
        self.FK = UR5ForwardKinematics()
        pass

    def calculate_joints(self, effector_pose):
        # Tentatively checked off logically
        theta1 = self.calculate_theta_1(effector_pose)
        # Tentatively checked off logically
        theta5 = self.calculate_theta_5(effector_pose, theta1)
        # Checked off logically
        theta6 = self.calculate_theta_6(effector_pose)
        # 
        theta2, theta3, theta4 = self.calculate_theta_234(effector_pose,
                                                          theta1, theta5, theta6)

    def calculate_theta_1(self, effector_pose):
        offset56 = inv(effector_pose.R06) * np.matrix([0, 0, self.FK.d6]).transpose()
        p05x = effector_pose.px - offset56[0, 0]
        p05y = effector_pose.py - offset56[1, 0]
        p05z = effector_pose.pz - offset56[2, 0]
        R = np.sqrt( p05x**2 + p05y**2 )
        alpha1 = np.arctan2( p05y, p05x )
        alpha2 = np.arccos( self.FK.d4 / R )
        return( [alpha1 + alpha2 + np.pi / 2, alpha1 - alpha2 + np.pi / 2] )

    def calculate_theta_5(self, effector_pose, theta1):
        # Remember that theta1 is an array with no values
        numerator = effector_pose.px * np.sin(theta1) - effector_pose.py * np.cos(theta1) - self.FK.d4
        abs_theta5 = np.arccos( numerator / self.FK.d6 )
        return( [ [abs_theta5[0], -abs_theta5[0]], [abs_theta5[1], -abs_theta5[1]] ] )

    def calculate_theta_6(self, effector_pose):
        return(effector_pose.psi)

    def calculate_theta_234(self, effector_pose, theta1, theta5, theta6):
        # DH14 = inv(self.FK.DH45(theta5[0][0])) *\
        #        inv(self.FK.DH56(theta6)) *\
        #        effector_pose.DH06 *\
        #        inv(self.FK.DH01(theta1[0]))
        DH14 = inv(self.FK.DH45(0)) *\
               inv(self.FK.DH56(0)) *\
               effector_pose.DH06 *\
               inv(self.FK.DH01(0))
        # The translation in DH14 is from 4 to 1 in the 4 reference frame, so we can undo that
        translation14 = inv(DH14[0:3, 0:3]) * -DH14[0:3, 3]
        # We use this x and y because it's the frame that defines the RRR linkage. See paper for details
        x = translation14[0, 0]
        y = translation14[2, 0]
        # This gets a rotation matrix from 1 to 4 assuming that 1 is rotated by 90 deg around its x-axis,
        # therefore all planar. This is basically a global reference frame in the plane of the arm
        R14_planar = inv(np.matrix([[1, 0, 0],
                                    [0, np.cos(np.pi/2),  np.sin(np.pi/2)],
                                    [0, -np.sin(np.pi/2), np.cos(np.pi/2)]])) *\
                     DH14[0:3, 0:3]
        # Phi is the 4 reference frame measured from a global frame. See paper for details
        phi = np.arccos(R14_planar[0, 0]) * np.sign(R14_planar[0, 1])
        # These L variables aren't strictly necessary, but they match the paper notation and make it easier to read
        l1 = self.FK.a2
        l2 = self.FK.a3
        l3 = self.FK.d5
        # x_prime and y_prime are the x, y positions of the third link, as in the paper
        x_prime = x - l3 * np.cos(phi)
        y_prime = y - l3 * np.sin(phi)
# Debugging this right here
        print('l1 + l2')
        print(l1 + l2)
        print(x_prime)
        print(y_prime)
        R_prime = np.sqrt( x_prime**2 + y_prime**2 )
        # lambda is a temporary variable in the paper, I made up alpha to break things up
        val_lambda = np.arctan2( -y_prime / R_prime, -x_prime / R_prime )
        val_alpha = np.arccos( -(x_prime**2 + y_prime**2 + l1**2 - l2**2) / (2 * l1 * R_prime) )
        # theta1 is the theta1 from the paper, which is theta2 in actuality
        theta1 = [val_lambda + val_alpha, val_lambda - val_alpha]
        # Same story for theta2 - it's theta3 in actuality
        theta2 = [np.arctan2((y_prime - l1 * np.sin(th1)) / l2,
                             (x_prime - l1 * np.cos(th1)) / l2) - th1 for th1 in theta1]
        theta3 = [phi - (th1 + th2) for th1, th2 in zip(theta1, theta2)]
        return(theta1, theta2, theta3)

    def create_joint_permutations(self):
        pass


class InitialEffectorPose():
    def __init__(self, x, y, z, phi, theta, psi):
        self.px = x
        self.py = y
        self.pz = z
        self.phi = phi
        self.theta = theta
        self.psi = psi
        Rphi = np.matrix([[np.cos(phi),  np.sin(phi), 0],
                        [-np.sin(phi), np.cos(phi), 0],
                        [0,            0,           1]])
        Rtheta = np.matrix([[1, 0,              0],
                          [0, np.cos(theta),  np.sin(theta)],
                          [0, -np.sin(theta), np.cos(theta)]])
        Rpsi = np.matrix([[np.cos(psi),  np.sin(psi), 0],
                        [-np.sin(psi), np.cos(psi), 0],
                        [0,            0,           1]])
        self.R06 = Rpsi * Rtheta * Rphi
        # This is necessary because the translation column should be from 6 to 0, in the 6 reference frame
        translation = self.R06 * -np.matrix([x, y, z]).transpose()
        bottom_layer = np.matrix([0, 0, 0, 1])
        self.DH06 = np.vstack( [np.hstack([self.R06, translation]), bottom_layer] )

if __name__ == '__main__':
    main()
