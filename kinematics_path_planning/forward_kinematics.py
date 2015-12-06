from numpy import pi
from numpy import cos
from numpy import sin
from numpy import matrix


class UR5ForwardKinematics():
    def __init__(self):
            # Taken from ur5.urdf.xacro
            self.d1 = 0.089159
            self.a2 = 0.42500
            self.a3 = 0.39225
            self.d4 = 0.10915
            self.d5 = 0.09465
            self.d6 = 0.0823

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
