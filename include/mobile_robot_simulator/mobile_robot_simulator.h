#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_broadcaster.h>

namespace mob_sim {

ros::Time last_vel; // last incoming velocity command
ros::Time measure_time; // this incoming velocity command

nav_msgs::Odometry odom; // odometry message

// current pose (only need yaw, rest is calculated)
double th = 0.0;

// update the odometry info based on velocity and duration
void update_odom_from_vel(geometry_msgs::Twist vel, ros::Duration time_diff);

// generate transform from odom
geometry_msgs::TransformStamped get_tf_from_odom(nav_msgs::Odometry odom);

// callback function for velocity
void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);

} // end mob_sim
