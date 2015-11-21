from math import pi
import numpy as np
import sympy as sp

import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Header

from forward_kinematics import UR5ForwardKinematics
from ur5_model.msg import JointAngles


class TestForwardKinematics():
    def __init__(self):
        rospy.init_node('test_forward_kinematics')
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=1)
        self.origin_pubs = [rospy.Publisher(s + '_axis', Marker, queue_size=1) for s in ['x', 'y', 'z']]
        rospy.Subscriber("/ur5_model/joint_command", JointAngles, self.arm_callback)
        self.FK = UR5ForwardKinematics()
        rospy.spin()

    def arm_callback(self, actual_pose):
        rospy.loginfo("Recieved angles")
        B01_result = self.FK.return_vector_to_reference_frame(self.FK.B01, actual_pose.arm_angle)
        B02_result = self.FK.return_vector_to_reference_frame(self.FK.B02, actual_pose.arm_angle)
        B03_result = self.FK.return_vector_to_reference_frame(self.FK.B03, actual_pose.arm_angle)
        B04_result = self.FK.return_vector_to_reference_frame(self.FK.B04, actual_pose.arm_angle)
        B05_result = self.FK.return_vector_to_reference_frame(self.FK.B05, actual_pose.arm_angle)
        B06_result = self.FK.return_vector_to_reference_frame(self.FK.B06, actual_pose.arm_angle)
        B = [B01_result]
        B.append( B02_result )
        B.append( B03_result )
        B.append( B04_result )
        B.append( B05_result )
        B.append( B06_result )
        self.marker_pub.publish(create_marker(B))
        create_origin(self.origin_pubs)
        rospy.loginfo('Finished callback')


def create_marker(matrices):
    marker = Marker()
    marker.header.frame_id = 'ur5/base_link'
    marker.header.stamp = rospy.Time.now()
    marker.ns = ''
    marker.id = 0
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD;
    marker.points = []
    for i in range(len(matrices)):
        marker.points.append( Point() )
        marker.points.append( Point(x=-matrices[i][0, 0], y=-matrices[i][1, 0], z=matrices[i][2, 0]) )
    # marker.points = [Point(), Point(x=1, y=1, z=1)]
    marker.scale.x = 0.01;
    marker.color.a = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    return marker


def create_origin(origin_pubs):
    x_axis = create_marker([np.matrix('1; 0; 0')])
    y_axis = create_marker([np.matrix('0; 1; 0')])
    z_axis = create_marker([np.matrix('0; 0; 1')])
    x_axis.color.r = 1.0; x_axis.color.g = 0.0; x_axis.color.b = 0.0;
    x_axis.scale.x = 0.005;
    y_axis.color.r = 0.0; y_axis.color.g = 1.0; y_axis.color.b = 0.0;
    y_axis.scale.x = 0.005;
    z_axis.color.r = 0.0; z_axis.color.g = 0.0; z_axis.color.b = 1.0;
    z_axis.scale.x = 0.005;
    origin_pubs[0].publish(x_axis)
    origin_pubs[1].publish(y_axis)
    origin_pubs[2].publish(z_axis)


def create_header(seq):
    header = Header()
    header.seq = seq
    header.stamp = rospy.Time.now()
    header.frame_id = 'ur5/base_link'
    return header


def main():
    TFK = TestForwardKinematics()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

