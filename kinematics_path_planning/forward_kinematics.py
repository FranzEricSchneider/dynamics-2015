from numpy import pi
from numpy import cos
from numpy import sin
from numpy import arccos
from numpy import arctan2
from numpy import sign
from numpy import array
from numpy import matrix
from numpy.linalg import inv


class UR5ForwardKinematics():
    def __init__(self):
        # Taken from ur5.urdf.xacro
        self.d1 = 0.089159
        self.a2 = 0.42500
        self.a3 = 0.39225
        self.d4 = 0.10915
        self.d5 = 0.09465
        self.d6 = 0.0823
        # Used when calculating Euler angles
        self.ZERO_THRESH = 1e-4

    def DH01(self, theta1):
        return( make_DH(theta1, self.d1, 0, 0) )

    def R01(self, theta1):
        return( self.DH01(theta1)[0:3, 0:3] )

    def DH12(self, theta2):
        return( make_DH(theta2, 0, 0, pi / 2) )

    def R12(self, theta2):
        return( self.DH12(theta2)[0:3, 0:3] )

    def DH23(self, theta3):
        return( make_DH(theta3, 0, self.a2, 0) )

    def R23(self, theta3):
        return( self.DH23(theta3)[0:3, 0:3] )

    def DH34(self, theta4):
        return( make_DH(theta4, self.d4, self.a3, 0) )

    def R34(self, theta4):
        return( self.DH34(theta4)[0:3, 0:3] )

    def DH45(self, theta5):
        return( make_DH(theta5, self.d5, 0, pi / 2) )

    def R45(self, theta5):
        return( self.DH45(theta5)[0:3, 0:3] )

    def DH56(self, theta6):
        return( make_DH(theta6, self.d6, 0, -pi / 2) )

    def R56(self, theta6):
        return( self.DH56(theta6)[0:3, 0:3] )

    def DH06(self, joint_angles):
        """ ang is the a JointAngles variable """
        ang = joint_angles.arm_angle
        DH06 = self.DH56(ang[5]) * self.DH45(ang[4]) * self.DH34(ang[3]) *\
               self.DH23(ang[2]) * self.DH12(ang[1]) * self.DH01(ang[0])
        return(DH06)

    def R06(self, joint_angles):
        """ ang is the a JointAngles variable """
        ang = joint_angles.arm_angle
        R06 = self.R56(ang[5]) * self.R45(ang[4]) * self.R34(ang[3]) *\
               self.R23(ang[2]) * self.R12(ang[1]) * self.R01(ang[0])
        return(R06)

    def effectorXYZ(self, joint_angles):
        """ Takes a ur5_model/JointAngles variable and calculates the effector XYZ position """
        DH06 = self.DH06(joint_angles)
        # A zero vector in the 06 reference frame
        zero_vector = matrix('0;0;0;1')
        effector_vector = array((inv(DH06) * zero_vector).transpose())[0]
        return(effector_vector[0:3])

    def effectorEulerAngles(self, joint_angles):
        """ Takes joint angles in and returns phi, theta, psi 313 Euler angles """
        # R313 = cphi cpsi - ctheta sphi spsi,  cpsi sphi + ctheta cphi spsi,  stheta spsi;
        #        -cphi spsi - ctheta cpsi sphi, -sphi spsi + ctheta cphi cpsi, cpsi stheta;
        #        stheta sphi,                   -cphi stheta,                  ctheta
        # theta = 0
        # cphi cpsi - sphi spsi   [0, 0]
        #     = c(psi-phi)
        # cpsi sphi + cphi spsi   [0, 1]
        #     = s(psi-phi)
        # theta = pi
        # cphi cpsi + sphi spsi   [0, 0]
        #     = c(psi-phi)
        # cpsi sphi - cphi spsi   [0, 1]
        #     = -s(psi-phi)
        R06 = self.R06(joint_angles)
        c_theta = R06[2, 2]
        theta = [arccos(c_theta), -arccos(c_theta)]
        if abs(sin(theta[0])) > self.ZERO_THRESH:
            phi = [arctan2(R06[2, 0] / sin(theta[0]), -R06[2, 1] / sin(theta[0])),
                   arctan2(R06[2, 0] / sin(theta[1]), -R06[2, 1] / sin(theta[1]))]
            psi = [arctan2(R06[0, 2] / sin(theta[0]), R06[1, 2] / sin(theta[0])),
                   arctan2(R06[0, 2] / sin(theta[1]), R06[1, 2] / sin(theta[1]))]
        else:
            # We are in gimbal lock (sin(theta) = 0), set phi to anything
            # The two values are for when theta = pi and theta = 0
            phi = [0.0, 0.0]
            psi = [-arctan2(R06[0, 1], R06[0, 0]), arctan2(R06[0, 1], R06[0, 0])]
        return([phi, theta, psi])


# See Theory of Applied Robotics pg. 242
# Uses the Denavit-Hartenberg method of reference frames
def make_DH(theta, d, a, alpha):
    # First, rotate by alpha around the x-axis
    R_x = matrix([[1, 0, 0, 0],
                  [0, cos(alpha), sin(alpha), 0],
                  [0, -sin(alpha), cos(alpha), 0],
                  [0, 0, 0, 1]])
    # Then travel by a along the x-axis
    D_x = matrix([[1, 0, 0, -a],
                  [0, 1, 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    # Then rotate by theta around the new z-axis
    R_z = matrix([[cos(theta), sin(theta), 0, 0],
                  [-sin(theta), cos(theta), 0, 0],
                  [0, 0, 1, 0],
                  [0, 0, 0, 1]])
    # Then travel by d along the z-axis
    D_z = matrix([[1, 0, 0, 0],
                  [0, 1, 0, 0],
                  [0, 0, 1, -d],
                  [0, 0, 0, 1]])
    return D_z * R_z * D_x * R_x
