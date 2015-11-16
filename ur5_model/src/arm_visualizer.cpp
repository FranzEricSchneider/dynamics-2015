#include <string>

#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/JointState.h>

#include <ur5_model/JointAngles.h>
#include "arm_visualizer.h"

using namespace std;


ros::Publisher joint_pub;
sensor_msgs::JointState joint_state;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_visualizer");
  ros::NodeHandle n;
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);

  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] ="shoulder_pan_joint";
  joint_state.name[1] ="shoulder_lift_joint";
  joint_state.name[2] ="elbow_joint";
  joint_state.name[3] ="wrist_1_joint";
  joint_state.name[4] ="wrist_2_joint";
  joint_state.name[5] ="wrist_3_joint";

  ros::Publisher cmd_pub = n.advertise<ur5_model::JointAngles>("/ur5_model/joint_command", 10);
  ros::Subscriber arm_sub = n.subscribe("/ur5_model/joint_command", 10, publish_arm_to_rviz);
  
  // Zero the hand and make it appear open. The sleeps are to let RVIZ start
  ros::Duration(2.0).sleep();
  ur5_model::JointAngles base_arm_state;
  for (int i=0; i<10; i++) {
    cmd_pub.publish(base_arm_state);
    ros::Duration(0.5).sleep();
  }

  ros::spin();
  return 0;
}

void publish_arm_to_rviz(const ur5_model::JointAnglesConstPtr& arm) {
  joint_state.header.stamp = ros::Time::now();
  joint_state.position[0] = arm->arm_angle[0];
  joint_state.position[1] = arm->arm_angle[1];
  joint_state.position[2] = arm->arm_angle[2];
  joint_state.position[3] = arm->arm_angle[3];
  joint_state.position[4] = arm->arm_angle[4];
  joint_state.position[5] = arm->arm_angle[5];

  joint_pub.publish(joint_state);
}
