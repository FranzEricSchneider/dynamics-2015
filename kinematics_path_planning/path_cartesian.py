import rospy
import numpy as np
from matplotlib import pyplot

from effector_pose import EffectorPose
from forward_kinematics import UR5ForwardKinematics
from inverse_kinematics import InverseKinematics
from path_polynomial_7 import PathPolynomial7
from ur5_model.msg import JointAngles


def main():
    rospy.init_node('PathCartesian')
    PC = PathCartesian()
    # Let the ROS node start up
    rospy.sleep(1.0)
    tf = 2.0
    dt = 0.1
    ep1 = EffectorPose(0.4, 0, 0, 0, np.pi/4, 0)
    ep2 = EffectorPose(0.4, 0.4, 0.4, 0, 0, 0)
    ep3 = EffectorPose(-0.4, 0.4, 0, 0, 0, 0)
    start_pose = PC.IK.calculate_joints(ep1)[0]
    end_pose = PC.calculate_and_publish_path(start_pose, ep1, ep2, dt, tf)
    PC.calculate_and_publish_path(end_pose, ep2, ep3, dt, tf)


class PathCartesian():
    """ Makes a path from pose to pose that uses a 7-degree polynomial path """
    def __init__(self):
        self.FK = UR5ForwardKinematics()
        self.IK = InverseKinematics()
        self.P7 = PathPolynomial7()
        self.joint_pub = rospy.Publisher('/ur5_model/joint_command', JointAngles, queue_size=10)

    def calculate_and_publish_path(self, start_pose, effector_pose1, effector_pose2, dt, tf):
        """ Publishes the path and returns the last pose JointAngles pose """
        t = np.arange(0, tf, dt)
        XYZ = self.calculate_xyz_path(effector_pose1, effector_pose2, t, tf)
        arm_angle_path = self.calculate_angle_time_series(start_pose, XYZ)
        self.publish_arm_path(arm_angle_path, dt, tf)
        return(arm_angle_path[-1])

    def publish_arm_path(self, ang, dt, tf):
        """
        Given angles and a time step, will try to publish the arm positions in time
        This function is slightly different than the one in Polynomial Path 7
        """
        counter = 0.0
        for i in range(len(ang)):
            self.joint_pub.publish(ang[i])
            # rospy.loginfo('Publishing, time elapsed = %.2f / %.2f', counter * dt, tf)
            rospy.sleep(dt)
            counter += 1

    def calculate_xyz_path(self, effector_pose1, effector_pose2, t, tf):
        """ Takes in pose1 and pose2 and generates a 7-polynomial xyz path with zero initial v, a, j """
        xyz1 = [effector_pose1.px, effector_pose1.py, effector_pose1.pz]
        xyz2 = [effector_pose2.px, effector_pose2.py, effector_pose2.pz]
        if abs(xyz1[0] - xyz2[0]) > self.FK.ZERO_THRESH:
            x_cf = self.P7.calculate_coefficients(xyz1[0], 0, 0, 0, xyz2[0], 0, 0, 0, tf)
            X = self.P7.angle_equation(x_cf, t)
            Y = np.array([xyz1[1] + (xyz2[1] - xyz1[1]) / (xyz2[0] - xyz1[0]) * (x - xyz1[0]) for x in X])
            Z = np.array([xyz1[2] + (xyz2[2] - xyz1[2]) / (xyz2[0] - xyz1[0]) * (x - xyz1[0]) for x in X])
        elif abs(xyz1[1] - xyz2[1]) > self.FK.ZERO_THRESH:
            y_cf = self.P7.calculate_coefficients(xyz1[1], 0, 0, 0, xyz2[1], 0, 0, 0, tf)
            Y = self.P7.angle_equation(y_cf, t)
            X = np.array([xyz1[0] + (xyz2[0] - xyz1[0]) / (xyz2[1] - xyz1[1]) * (y - xyz1[1]) for y in Y])
            Z = np.array([xyz1[2] + (xyz2[2] - xyz1[2]) / (xyz2[1] - xyz1[1]) * (y - xyz1[1]) for y in Y])
        elif abs(xyz1[2] - xyz2[2]) > self.FK.ZERO_THRESH:
            z_cf = self.P7.calculate_coefficients(xyz1[2], 0, 0, 0, xyz2[2], 0, 0, 0, tf)
            Z = self.P7.angle_equation(z_cf, t)
            X = np.array([xyz1[0] + (xyz2[0] - xyz1[0]) / (xyz2[2] - xyz1[2]) * (z - xyz1[2]) for z in Z])
            Y = np.array([xyz1[1] + (xyz2[1] - xyz1[1]) / (xyz2[2] - xyz1[2]) * (z - xyz1[2]) for z in Z])
        else:
            return([xyz1, xyz2])
        return([X, Y, Z])

    def calculate_angle_time_series(self, start_pose, XYZ):
        """ Takes a start_pose and an XYZ path, then calculates IK along that path """
        euler_init = self.FK.effectorEulerAngles(start_pose)
        ep_init = EffectorPose(XYZ[0][0], XYZ[1][0], XYZ[2][0], euler_init[0][0], euler_init[1][0], euler_init[2][0])
        first_pose_step = self.IK.calculate_closest_permutation(ep_init, start_pose)
        joint_angles = range(len(XYZ[0]))
        for i in range(len(XYZ[0])):
            if i == 0:
                joint_angles[i] = first_pose_step
            else:
                joint_angles[i] = self.IK.calculate_closest_permutation(EffectorPose(XYZ[0][i], XYZ[1][i], XYZ[2][i],
                                                                              euler_init[0][0],
                                                                              euler_init[1][0],
                                                                              euler_init[2][0]),
                                                                        joint_angles[i-1])
        return(joint_angles)


if __name__ == '__main__':
    main()
