import rospy
import numpy as np

from effector_pose import EffectorPose
from inverse_kinematics import InverseKinematics
from path_polynomial_7 import PathPolynomial7
from ur5_model.msg import JointAngles


def main():
    rospy.init_node('Demo')
    pub = rospy.Publisher('/ur5_model/joint_command', JointAngles, queue_size=10)
    start_pose = JointAngles([0, 1.8, 5.82, 0, 0, 0])
    IK = InverseKinematics()
    P7 = PathPolynomial7()
    dt = 0.05
    # Let ROS node start up
    rospy.sleep(1.0)

    ep1 = EffectorPose(0.3, -0.5, 0.2, 0.2, 0, 0)
    pose1 = IK.calculate_closest_permutation(ep1, start_pose)
    pub.publish(pose1)
    tf1 = 2.0
    # P7.calculate_and_publish_end_stop_path(start_pose, pose1, dt, tf1)


if __name__ == '__main__':
    main()
