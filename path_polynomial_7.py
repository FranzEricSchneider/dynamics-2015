import numpy as np
from matplotlib import pyplot

from forward_kinematics import UR5ForwardKinematics
from ur5_model.msg import JointAngles


def main():
    P7 = PathPolynomial7()
    tf = 1.0
    coefficients = P7.calculate_coefficients(0, 0, 0, 0,
                                             35, 0, 0, 0, tf)
    t = np.arange(0, tf, 0.01)
    q = P7.angle_equation(coefficients, t)
    Dq = P7.velocity_equation(coefficients, t)
    DDq = P7.acceleration_equation(coefficients, t)
    DDDq = P7.jerk_equation(coefficients, t)
    print(type(DDDq))
    pyplot.plot(t, q, color='b')
    pyplot.plot(t, Dq/2.0, color='g')
    pyplot.plot(t, DDq/10.0, color='k')
    pyplot.plot(t, DDDq/100.0, color='r')
    pyplot.show()


class PathPolynomial7():
    """ Makes a path from pose to pose that uses a 7-degree polynomial path """
    def __init__(self):
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
        LHS = np.matrix([[1, 0, 0, 0, 0, 0, 0, 0],
                         [0, 1, 0, 0, 0, 0, 0, 0],
                         [0, 0, 2, 0, 0, 0, 0, 0],
                         [0, 0, 0, 6, 0, 0, 0, 0],
                         [1, tf, tf**2, tf**3, tf**4, tf**5, tf**6, tf**7],
                         [0, 1, 2*tf, 3*tf**2, 4*tf**3, 5*tf**4, 6*tf**5, 7*tf**6],
                         [0, 0, 2, 6*tf, 12*tf**2, 20*tf**3, 30*tf**4, 42*tf**5],
                         [0, 0, 0, 6, 24*tf, 60*tf**2, 120*tf**3, 210*tf**4]])
        coefficients = np.array((np.linalg.inv(LHS) * RHS).transpose())[0]
        return(coefficients)

    def angle_equation(self, cf, t):
        """
        Given coefficients (numpy array) and time vector (numpy array), calculates the
        angle position through time (q)
        """
        q = cf[0] + cf[1]*t + cf[2]*t**2 + cf[3]*t**3 + cf[4]*t**4 + cf[5]*t**5 + cf[6]*t**6 + cf[7]*t**7
        return(q)

    def velocity_equation(self, cf, t):
        """ Given coefficients and time vector, calculates the velocity through time (Dq) """
        Dq = cf[1] + 2*cf[2]*t + 3*cf[3]*t**2 + 4*cf[4]*t**3 + 5*cf[5]*t**4 + 6*cf[6]*t**5 + 7*cf[7]*t**6
        return(Dq)

    def acceleration_equation(self, cf, t):
        """ Given coefficients and time vector, calculates the acceleration through time (Dq) """
        DDq = 2*cf[2] + 6*cf[3]*t + 12*cf[4]*t**2 + 20*cf[5]*t**3 + 30*cf[6]*t**4 + 42*cf[7]*t**5
        return(DDq)

    def jerk_equation(self, cf, t):
        """ Given coefficients and time vector, calculates the jerk through time (Dq) """
        DDDq = 6*cf[3] + 24*cf[4]*t + 60*cf[5]*t**2 + 120*cf[6]*t**3 + 210*cf[7]*t**4
        return(DDDq)

if __name__ == '__main__':
    main()
