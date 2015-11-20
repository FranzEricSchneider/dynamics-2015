from math import pi
import sympy as sp

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from forward_kinematics import UR5ForwardKinematics
from ur5_model.msg import JointAngles


def main():
    arm_pub = rospy.Publisher('/ur5_model/joint_command', JointAngles, queue_size=1)
    marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
    origin_pubs = [rospy.Publisher(s + '_axis', Marker, queue_size=1) for s in ['x', 'y', 'z']]
    rospy.init_node('test_forward_kinematics')
    FK = UR5ForwardKinematics()
    actual_pose = JointAngles([0, 0, 0, 0, 0, 0])
    B01_result = return_vector_to_reference_frame(FK, FK.B01, actual_pose.arm_angle)
    B02_result = return_vector_to_reference_frame(FK, FK.B02, actual_pose.arm_angle)
    B03_result = return_vector_to_reference_frame(FK, FK.B03, actual_pose.arm_angle)
    B04_result = return_vector_to_reference_frame(FK, FK.B04, actual_pose.arm_angle)
    B05_result = return_vector_to_reference_frame(FK, FK.B05, actual_pose.arm_angle)
    B06_result = return_vector_to_reference_frame(FK, FK.B06, actual_pose.arm_angle)
    B = B01_result
    B = B.row_join(B02_result)
    B = B.row_join(B03_result)
    B = B.row_join(B04_result)
    B = B.row_join(B05_result)
    B = B.row_join(B06_result)

    rate = rospy.Rate(2)
    while not rospy.is_shutdown():
        # rospy.loginfo("Publishing, cntl-c to quit")
        arm_pub.publish(actual_pose)
        marker_pub.publish(create_marker(B))
        create_origin(origin_pubs)
        rate.sleep()


def return_vector_to_reference_frame(FK, transfer_matrix, thetas):
    B = transfer_matrix
    # Taken from ur5.urdf.xacro
    B_w_lengths = B.subs([(FK.d1, 0.089159),
                          (FK.a2, 0.42500),
                          (FK.a3, 0.39225),
                          (FK.d4, 0.10915),
                          (FK.d5, 0.09465),
                          (FK.d6, 0.0823)])
    B_w_zero_angles = B_w_lengths.subs([(FK.th1, thetas[0]),
                                        (FK.th2, thetas[1]),
                                        (FK.th3, thetas[2]),
                                        (FK.th4, thetas[3]),
                                        (FK.th5, thetas[4]),
                                        (FK.th6, thetas[5])])
    initial_vector = sp.Matrix([0.0, 0, 0, 1.0])
    return(B_w_zero_angles.inv() * initial_vector)


def create_marker(vector):
    marker = Marker()
    marker.header.frame_id = 'ur5/base_link'
    marker.header.stamp = rospy.Time.now()
    marker.ns = ''
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD;
    marker.points = []
    for i in range(len(vector[0, :])):
        marker.points.append( Point() )
        marker.points.append( Point(x=-vector[0, i], y=-vector[1, i], z=vector[2, i]) )
    # marker.points = [Point(), Point(x=1, y=1, z=1)]
    marker.scale.x = 0.005;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker


def create_origin(origin_pubs):
    x_axis = create_marker(sp.Matrix([1, 0, 0]))
    y_axis = create_marker(sp.Matrix([0, 1, 0]))
    z_axis = create_marker(sp.Matrix([0, 0, 1]))
    x_axis.color.r = 1.0; x_axis.color.g = 0.0; x_axis.color.b = 0.0;
    y_axis.color.r = 0.0; y_axis.color.g = 1.0; y_axis.color.b = 0.0;
    z_axis.color.r = 0.0; z_axis.color.g = 0.0; z_axis.color.b = 1.0;
    origin_pubs[0].publish(x_axis)
    origin_pubs[1].publish(y_axis)
    origin_pubs[2].publish(z_axis)


def create_header(seq):
    header = Header()
    header.seq = seq
    header.stamp = rospy.Time.now()
    header.frame_id = 'ur5/base_link'
    return header


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

