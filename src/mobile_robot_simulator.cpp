#include "ros/ros.h"

#include "mobile_robot_simulator/mobile_robot_simulator.h"

namespace mob_sim { 

void update_odom_from_vel(geometry_msgs::Twist vel, ros::Duration time_diff)
{
    ROS_DEBUG_STREAM("Velocity - x: " << vel.linear.x << " y: " << vel.linear.y << " th: " << vel.angular.z);
    //compute odometry in a typical way given the velocities of the robot
    double delta_x = (vel.linear.x * cos(th) - vel.linear.y * sin(th)) * time_diff.toSec();
    double delta_y = (vel.linear.x * sin(th) + vel.linear.y * cos(th)) * time_diff.toSec();
    double delta_th = vel.angular.z * time_diff.toSec();
    ROS_DEBUG_STREAM("Delta - x: " << delta_x << " y: " << delta_y << " th: " << delta_th);
    
    // update odometry
    odom.header.stamp = measure_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x += delta_x;
    odom.pose.pose.position.y += delta_y;
    // generate quaternion based on current yaw
    th += delta_th;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
    // set velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist = vel;
    ROS_DEBUG_STREAM("Odometry - x: " << odom.pose.pose.position.x << " y: " << odom.pose.pose.position.y << " th: " << th);
    return;
}

void get_tf_from_odom(nav_msgs::Odometry odom)
{
    geometry_msgs::TransformStamped odom_tmp;
    // copy from odmoetry message
    odom_tmp.header = odom.header;
    odom_tmp.child_frame_id = odom.child_frame_id;
    odom_tmp.transform.translation.x = odom.pose.pose.position.x;
    odom_tmp.transform.translation.y = odom.pose.pose.position.y;
    odom_tmp.transform.translation.z = 0.0;
    odom_tmp.transform.rotation = odom.pose.pose.orientation;
    // convert and update
    tf::transformStampedMsgToTF(odom_tmp, odom_trans);
    return;
}

void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_DEBUG("Received message on cmd_vel");
    measure_time = ros::Time::now();
    ros::Duration dt = measure_time - last_vel;
    last_vel = measure_time;
    if (dt >= ros::Duration(0.5)) dt = ros::Duration(0.1);
    message_received = true;
    geometry_msgs::Twist vel = *msg;
    update_odom_from_vel(vel,dt);
    return;
}

void init_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
{
    if (msg->header.frame_id != "map") {
        ROS_ERROR("Initial pose not specified in map frame, ignoring");
        return;
    }
    ROS_INFO("Received pose estimate of mobile base");
    
    // msg is map -> base_link
    tf::StampedTransform msg_t;
    msg_t.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    msg_t.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
    ROS_DEBUG_STREAM("map -> base_link - x: " << msg_t.getOrigin().getX() << " y: " << msg_t.getOrigin().getY());
    // get odom -> base_link
    ROS_DEBUG_STREAM("odom -> base_link - x: " << odom_trans.getOrigin().getX() << " y: " << odom_trans.getOrigin().getY());
    // calculate map -> odom and save as stamped
    tf::StampedTransform map_t = tf::StampedTransform(msg_t * odom_trans.inverse(), msg->header.stamp, "map", "odom");
    ROS_DEBUG_STREAM("map -> odom - x: " << map_t.getOrigin().getX() << " y: " << map_t.getOrigin().getY());
    map_trans = map_t;    
    return;
}

} // end namespace



