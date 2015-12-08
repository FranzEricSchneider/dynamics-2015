import rospy
import numpy as np
from matplotlib import pyplot

from effector_pose import EffectorPose
from forward_kinematics import UR5ForwardKinematics
from path_polynomial_7 import PathPolynomial7
from ur5_model.msg import JointAngles


def main():
    rospy.init_node('PathCartesion')
    PC = PathCartesion()
    P7 = PathPolynomial7()
    # Let the ROS node start up
    rospy.sleep(1.0)
    tf = 2.0
    dt = 0.05
    pose1 = JointAngles([0, 1.8, -0.46, 0, 0, 0])
    pose2 = JointAngles([-0.5, 1, -1, 0, -2, -4])
    pose3 = JointAngles([-3, 0.5, -0.5, -2, -2, -4])
    pose4 = JointAngles([-2, 0.5, -2, 1, 0, -4])
    P7.calculate_and_publish_end_stop_path(pose1, pose2, dt, tf)
    rospy.sleep(0.5)
    P7.calculate_and_publish_end_stop_path(pose2, pose3, dt, tf)
    rospy.sleep(0.5)
    P7.calculate_and_publish_end_stop_path(pose3, pose4, dt, tf)
    rospy.sleep(0.5)
    P7.calculate_and_publish_end_stop_path(pose4, pose1, dt, tf)


class PathCartesion():
    """ Makes a path from pose to pose that uses a 7-degree polynomial path """
    def __init__(self):
        self.FK = UR5ForwardKinematics()
        # There are 8 coefficients in a 7-degree polynomial, and there is a 7-degree
        # polynomial for every joint
        self.coefficients = np.arange(8)
        self.joint_pub = rospy.Publisher('/ur5_model/joint_command', JointAngles, queue_size=10)

    def calculate_and_publish_end_stop_path(self, pose1, pose2, dt, tf):
        t = np.arange(0, tf, dt)
        arm_coefficients = self.calculate_arm_coefficients_end_stops(pose1, pose2, tf)
        arm_angle_path = self.calculate_arm_angles(arm_coefficients, t)
        self.publish_arm_path(arm_angle_path, dt, tf)

    def calculate_arm_coefficients_end_stops(self, pose1, pose2, tf):
        """
        Takes the start and poses of arm motion, and time to complete path, and
        returns 7-poly coefficients for a path
        """
        # Turn the JointAngles variables into numpy arrays
        pose1 = np.array(pose1.arm_angle)
        pose2 = np.array(pose2.arm_angle)
        # Loop through the arm joints calculate coefficients for each joint
        for i in range(6):
            # Calculates the coefficients assuming 0 velocity, acceleration, and jerk at endpoints
            self.coefficients[i] = self.calculate_coefficients(pose1[i], 0, 0, 0,
                                                               pose2[i], 0, 0, 0, tf)
        return(self.coefficients)

    def calculate_arm_angles(self, cf, t):
        """ Given coefficients and time vector, calculate paths for all arm angles """
        angles = [np.arange(len(t)) for i in range(6)]
        for i in range(6):
            angles[i] = self.angle_equation(cf[i], t)
        return(angles)

    def publish_arm_path(self, ang, dt, tf):
        """ Given angles and a time step, will try to publish the arm positions in time """
        counter = 0.0
        for i in range(len(ang[0])):
            joint = JointAngles([ang[0][i], ang[1][i], ang[2][i], ang[3][i], ang[4][i], ang[5][i]])
            self.joint_pub.publish(joint)
            # rospy.loginfo('Publishing, time elapsed = %.2f / %.2f', counter * dt, tf)
            rospy.sleep(dt)
            counter += 1

    def calculate_xyz_path(self, pose1, pose2, t, tf):
        """ Takes in pose1 and pose2 and generates a 7-polynomial xyz path with zero initial v, a, j """
        xyz1 = self.FK.effectorXYZ(pose1)
        xyz2 = self.FK.effectorXYZ(pose2)
        x_cf = self.P7.calculate_coefficients(xyz1[0], 0, 0, 0, xyz2[0], 0, 0, 0, tf)
        X = self.P7.angle_equation(x_cf, t)
        Y = np.array([xyz1[1] + (xyz2[1] - xyz1[1]) / (xyz2[0] - xyz1[0]) * (x - xyz1[0]) for x in X])
        Z = np.array([xyz1[2] + (xyz2[2] - xyz1[2]) / (xyz2[0] - xyz1[0]) * (x - xyz1[0]) for x in X])
        return([X, Y, Z])

    def calculate_angle_time_series(self, start_pose, XYZ):
        """ Takes a start_pose and an XYZ path, then calculates IK along that path """
        first_step = self.IK.calculate_closest_permutation(EffectorPose(), start_pose)
        poses = [self.IK.calculate_closest_permutation()]

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
