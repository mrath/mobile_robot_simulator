#include "ros/ros.h"
#include "ros/console.h"

#include "geometry_msgs/Twist.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include <tf/transform_broadcaster.h>

namespace mob_sim {

ros::Time last_vel;
ros::Time measure_time;

nav_msgs::Odometry odom;

// current pose
double th = 0.0;

void update_odom_from_vel(geometry_msgs::Twist vel, ros::Duration time_diff);

geometry_msgs::TransformStamped get_tf_from_odom(nav_msgs::Odometry odom);

void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);

} // end mob_sim
