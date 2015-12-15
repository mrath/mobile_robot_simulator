#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Twist.h"
#include "tf/LinearMath/Transform.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#ifndef MOBILE_ROBOT_SIMULATOR
#define MOBILE_ROBOT_SIMULATOR

namespace mob_sim {

ros::Time last_vel; // last incoming velocity command
ros::Time measure_time; // this incoming velocity command
bool message_received = false;

nav_msgs::Odometry odom; // odometry message
tf::StampedTransform odom_trans;

tf::StampedTransform map_trans; // transformation from odom to map

// current pose (only need yaw, rest is calculated)
double th = 0.0;

// update the odometry info based on velocity and duration
void update_odom_from_vel(geometry_msgs::Twist vel, ros::Duration time_diff);

// generate transform from odom
void get_tf_from_odom(nav_msgs::Odometry odom);

// callback function for velocity
void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);

// initial pose callback function
void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg);

} // end mob_sim

#endif
