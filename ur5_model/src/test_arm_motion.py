#!/usr/bin/env python

import rospy
import math
from ur5_model.msg import JointAngles


def sine_publisher():
    pub = rospy.Publisher('/ur5_model/joint_command', JointAngles, queue_size=10)
    rospy.init_node('sine_publisher', anonymous=True)
    rate = rospy.Rate(50)    # 50 Hz
    counter = 0
    scalar = 50.0
    while not rospy.is_shutdown():
        deg = counter / scalar
        cmd = JointAngles([math.sin(deg), math.cos(deg)] * 3)
        pub.publish(cmd)
        rate.sleep()
        counter += 1

if __name__ == '__main__':
    try:
        sine_publisher()
    except rospy.ROSInterruptException:
        pass