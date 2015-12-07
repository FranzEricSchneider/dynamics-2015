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

    radius = 0.4
    scalar = 1.0
    counter = 0
    while not rospy.is_shutdown():
        ep1 = EffectorPose(radius * np.sin(counter / scalar),
                           radius * np.cos(counter / scalar),
                           0, 
                           np.pi/2, np.pi - 0.25, 0)
        pose1 = IK.calculate_closest_permutation(ep1, start_pose)
        tf1 = 1.0
        P7.calculate_and_publish_end_stop_path(start_pose, pose1, dt, tf1)
        rospy.sleep(0.5)
        P7.calculate_and_publish_end_stop_path(pose1, start_pose, dt, tf1)
        rospy.sleep(0.5)
        counter += 1

if __name__ == '__main__':
    main()
