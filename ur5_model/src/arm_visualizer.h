#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <ur5_model/JointAngles.h>

void publish_arm_to_rviz(const ur5_model::JointAnglesConstPtr& arm);
void create_origin(const ur5_model::JointAnglesConstPtr& arm);
visualization_msgs::Marker create_marker(float x, float y, float z);
