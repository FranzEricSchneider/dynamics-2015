import numpy as np
from numpy.linalg import inv

from forward_kinematics import UR5ForwardKinematics
from ur5_model.msg import JointAngles


def main():
    IK = InverseKinematics()
    effector_pose = InitialEffectorPose(1, 0, 0, -np.pi/2, -np.pi/2, 0)
    IK.calculate_joints(effector_pose)


class InverseKinematics():
    def __init__(self):
        self.FK = UR5ForwardKinematics()
        pass

    def calculate_joints(self, effector_pose):
        # Tentatively checked off logically
        theta1 = self.calculate_theta_1(effector_pose)

    def calculate_theta_1(self, effector_pose):
        offset56 = inv(effector_pose.R06) * np.matrix([0, 0, self.FK.d6]).transpose()
        p05x = effector_pose.px - offset56[0, 0]
        p05y = effector_pose.py - offset56[1, 0]
        p05z = effector_pose.pz - offset56[2, 0]
        R = np.sqrt( p05x**2 + p05y**2 )
        alpha1 = np.arctan2( p05y, p05x )
        alpha2 = np.arccos( self.FK.d4 / R )
        return([alpha1 + alpha2 + np.pi / 2, alpha1 - alpha2 + np.pi / 2])

    def calculate_theta_5(self):
        pass

    def calculate_theta_6(self):
        pass

    def calculate_theta_234(self):
        pass

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


if __name__ == '__main__':
    main()
