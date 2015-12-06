import numpy as np

class EffectorPose():
    def __init__(self, x, y, z, phi, theta, psi):
        """
        -- Takes in the desired (x,y,z) global position of the effector, and the desired 313 Euler angles specifying
        the desired starting orientation of the effector
        -- Contains the position, the rotation matrix, and the Denavit-Hartenberg transformation matrix from the 00
        reference frame to the 06 reference frame
        """
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
