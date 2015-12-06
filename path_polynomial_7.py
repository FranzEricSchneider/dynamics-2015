import numpy as np

from forward_kinematics import UR5ForwardKinematics
from ur5_model.msg import JointAngles


class PathPolynomial7():
    """ Makes a path from pose to pose that uses a 7-degree polynomial path """
    def __init__(self, arg):
        self.FK = UR5ForwardKinematics()
        # There are 8 coefficients in a 7-degree polynomial, and there is a 7-degree
        # polynomial for every joint
        self.coefficients = [np.arange(8) for i in range(6)]

    def calculate_coefficients(self, q0, Dq0, DDq0, DDDq0, qf, Dqf, DDqf, DDDqf, tf):
    """
    Takes in joint positions and derivaties from initial to final, as well as final time
    Inspiration taken from pg 737 of the Theory of Applied Robotics
    """
        RHS = np.matrix([q0, Dq0, DDq0, DDDq0, qf, Dqf, DDqf, DDDqf]).transpose()
        LHS = np.matrix([1, 0, 0, 0, 0, 0, 0, 0],
                        [0, 1, 0, 0, 0, 0, 0, 0],
                        [0, 0, 2, 0, 0, 0, 0, 0],
                        [0, 0, 0, 6, 0, 0, 0, 0],
                        [1, t, t**2, t**3, t**4, t**5, t**6, t**7],
                        [0, 1, 2*t, 3*t**2, 4*t**3, 5*t**4, 6*t**5, 7*t**6],
                        [0, 0, 2, 6*t, 12*t**2, 20*t**3, 30*t**4, 42*t**5],
                        [0, 0, 0, 6, 24*t, 60*t**2, 120*t**3, 210*t**4])
        np.linalg.inv(LHS)