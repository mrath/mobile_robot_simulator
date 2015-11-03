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
    // copy from odmoetry message
    odom_trans.header = odom.header;
    odom_trans.child_frame_id = odom.child_frame_id;
    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom.pose.pose.orientation;
    return;
}

void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_INFO("Received message on cmd_vel");
    measure_time = ros::Time::now();
    ros::Duration dt = measure_time - last_vel;
    last_vel = measure_time;
    if (dt >= ros::Duration(0.2)) dt = ros::Duration(0.1);
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
    ROS_INFO("Received pose estimate");
    
    // msg is map -> base_link
    tf::StampedTransform msg_t;
    msg_t.setOrigin(tf::Vector3(msg->pose.pose.position.x,msg->pose.pose.position.y,msg->pose.pose.position.z));
    msg_t.setRotation(tf::Quaternion(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w));
    ROS_DEBUG_STREAM("map -> base_link - x: " << msg_t.getOrigin().getX() << " y: " << msg_t.getOrigin().getY());
    // get odom -> base_link
    tf::StampedTransform odom_t;
    tf::transformStampedMsgToTF(odom_trans, odom_t);
    ROS_DEBUG_STREAM("odom -> base_link - x: " << odom_t.getOrigin().getX() << " y: " << odom_t.getOrigin().getY());
    // calculate map -> odom and save as stamped
    tf::StampedTransform map_t = tf::StampedTransform(msg_t * odom_t.inverse(), msg->header.stamp, "map", "odom");
    ROS_DEBUG_STREAM("map -> odom - x: " << map_t.getOrigin().getX() << " y: " << map_t.getOrigin().getY());
    // convert and update
    tf::transformStampedTFToMsg(map_t, map_trans);    
    return;
}

} // end namespace

int main(int argc, char **argv)
{
    ros::init(argc,argv, "mobile_robot_simulator");
    ros::NodeHandle nh;
    // publishers, subscribers
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",50); // odometry publisher
    tf::TransformBroadcaster tf_broadcaster; // tf publisher
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel",5,mob_sim::vel_callback); // velocity subscriber
    ros::Subscriber init_pose_sub = nh.subscribe("initialpose",5,mob_sim::init_pose_callback); // initial pose callback
    // load parameters
    
    // initialize timers
    ros::Time last_update = ros::Time::now();
    mob_sim::last_vel = last_update - ros::Duration(0.1);
    // initialize forst odom message
    mob_sim::update_odom_from_vel(geometry_msgs::Twist(), ros::Duration(0.1));
    mob_sim::odom.header.stamp = last_update;
    mob_sim::get_tf_from_odom(mob_sim::odom);
    // Initialize tf from map to odom
    mob_sim::map_trans.header.frame_id = "/map";
    mob_sim::map_trans.header.stamp = last_update;
    mob_sim::map_trans.child_frame_id = "/odom";
    mob_sim::map_trans.transform.rotation.w = 1.0;    
    
    ROS_INFO("Started mobile robot simulator, listening on cmd_vel topic");
    
    ros::Rate r(10.0);
    while(nh.ok()){
        ros::spinOnce();
        last_update = ros::Time::now();
        //ROS_INFO_STREAM("time diff " << (last_update - mob_sim::last_vel).toSec());
        //ROS_INFO_STREAM("last_vel: " << mob_sim::last_vel.toSec());
        //ROS_INFO_STREAM("cmd_vel received: " << mob_sim::message_received);
        // If we didn't receive a message, send the old odometry info with a new timestamp
        if (!mob_sim::message_received)
        {
            mob_sim::odom.header.stamp = last_update;
            mob_sim::odom_trans.header.stamp = last_update;
        }
        // publish odometry and tf
        odom_pub.publish(mob_sim::odom);
        mob_sim::get_tf_from_odom(mob_sim::odom);
        tf_broadcaster.sendTransform(mob_sim::odom_trans);
        mob_sim::map_trans.header.stamp = last_update;
        tf_broadcaster.sendTransform(mob_sim::map_trans);
        
        r.sleep();
   
        mob_sim::message_received = false;
    }
    
    return 0;
    
} // end main

// end namespace
