import rospy
import numpy as np

from effector_pose import EffectorPose
from inverse_kinematics import InverseKinematics
from path_cartesian import PathCartesian
from path_polynomial_7 import PathPolynomial7
from ur5_model.msg import JointAngles


def main():
    rospy.init_node('Demo')
    pub = rospy.Publisher('/ur5_model/joint_command', JointAngles, queue_size=10)
    start_pose = JointAngles([0, 1.8, 5.82, 0, 0, 0])
    IK = InverseKinematics()
    P7 = PathPolynomial7()
    PC = PathCartesian()
    dt = 0.05
    # Let ROS node start up
    rospy.sleep(1.0)

    radius = 0.5
    scalar = 1.0
    counter = 0
    extension = 0.25
    tf1 = 1.5
    tf2 = 0.35
    while not rospy.is_shutdown():
        x = radius * np.sin(counter / scalar)
        y = radius * np.cos(counter / scalar)
        z = 0.1
        phi = np.arctan2(y, x)
        theta = -4*np.pi/5
        ep1 = EffectorPose(x, y, z, phi - np.pi / 2, theta, 0)
        ep2 = EffectorPose(np.cos(phi) * abs(np.sin(theta)) * extension + x,
                           np.sin(phi) * abs(np.sin(theta)) * extension + y,
                           np.cos(theta) * extension + z,
                           phi, theta, 0)
        pose = IK.calculate_closest_permutation(ep1, start_pose)
        out_pose = P7.calculate_and_publish_end_stop_path(start_pose, pose, dt, tf1)
        rospy.sleep(0.5)
        out_pose = PC.calculate_and_publish_path(out_pose, ep1, ep2, dt, tf2)
        rospy.sleep(0.25)
        PC.calculate_and_publish_path(out_pose, ep2, ep1, dt, tf2)
        rospy.sleep(0.5)
        P7.calculate_and_publish_end_stop_path(pose, start_pose, dt, tf1)
        rospy.sleep(0.5)
        counter += 1

if __name__ == '__main__':
    main()
