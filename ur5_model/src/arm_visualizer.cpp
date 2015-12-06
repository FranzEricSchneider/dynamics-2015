#include "arm_visualizer.h"


ros::Publisher joint_pub;
ros::Publisher x_pub;
ros::Publisher y_pub;
ros::Publisher z_pub;
sensor_msgs::JointState joint_state;


int main(int argc, char **argv)
{
  ros::init(argc, argv, "arm_visualizer");
  ros::NodeHandle n;
  joint_pub = n.advertise<sensor_msgs::JointState>("joint_states", 1);
  // CoG_pub = n.advertise<visualization_msgs::MarkerArray>("CoG_array", 1);

  joint_state.name.resize(6);
  joint_state.position.resize(6);
  joint_state.name[0] ="ur5/shoulder_pan_joint";
  joint_state.name[1] ="ur5/shoulder_lift_joint";
  joint_state.name[2] ="ur5/elbow_joint";
  joint_state.name[3] ="ur5/wrist_1_joint";
  joint_state.name[4] ="ur5/wrist_2_joint";
  joint_state.name[5] ="ur5/wrist_3_joint";

  ros::Publisher cmd_pub = n.advertise<ur5_model::JointAngles>("/ur5_model/joint_command", 10);
  x_pub = n.advertise<visualization_msgs::Marker>("/ur5_model/x_axis", 1);
  y_pub = n.advertise<visualization_msgs::Marker>("/ur5_model/y_axis", 1);
  z_pub = n.advertise<visualization_msgs::Marker>("/ur5_model/z_axis", 1);
  ros::Subscriber arm_sub = n.subscribe("/ur5_model/joint_command", 10, publish_arm_to_rviz);
  ros::Subscriber origin_sub = n.subscribe("/ur5_model/joint_command", 10, create_origin);
  
  // Zero the hand and make it appear open. The sleeps are to let RVIZ start
  ros::Duration(2.0).sleep();
  ur5_model::JointAngles base_arm_state;
  for (int i=0; i<4; i++) {
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

void create_origin(const ur5_model::JointAnglesConstPtr& arm) {
  visualization_msgs::Marker x_axis = create_marker(0.4, 0, 0);
  visualization_msgs::Marker y_axis = create_marker(0, 0.4, 0);
  visualization_msgs::Marker z_axis = create_marker(0, 0, 0.4);
  x_axis.color.r = 1.0; x_axis.color.g = 0.0; x_axis.color.b = 0.0;
  x_axis.scale.x = 0.01;
  y_axis.color.r = 0.0; y_axis.color.g = 1.0; y_axis.color.b = 0.0;
  y_axis.scale.x = 0.01;
  z_axis.color.r = 0.0; z_axis.color.g = 0.0; z_axis.color.b = 1.0;
  z_axis.scale.x = 0.01;
  x_pub.publish(x_axis);
  y_pub.publish(y_axis);
  z_pub.publish(z_axis);
}

visualization_msgs::Marker create_marker(float x, float y, float z) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "ur5/base_link";
  marker.header.stamp = ros::Time::now();
  marker.ns = "axes";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  geometry_msgs::Point p1, p2;
  p1.x = 0; p1.y = 0; p1.z = 0;
  p2.x = -x; p2.y = -y; p2.z = z;
  marker.points.push_back(p1);
  marker.points.push_back(p2);
  marker.scale.x = 0.01;
  marker.color.a = 1.0;
  return marker;
}
