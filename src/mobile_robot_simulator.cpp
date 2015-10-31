#include "ros/ros.h"

#include "mobile_robot_simulator/mobile_robot_simulator.h"

namespace mob_sim { 

void update_odom_from_vel(geometry_msgs::Twist vel, ros::Duration time_diff)
{
    //compute odometry in a typical way given the velocities of the robot
    double delta_x = (vel.linear.x * cos(th) - vel.linear.y * sin(th)) * time_diff.toSec();
    double delta_y = (vel.linear.x * sin(th) + vel.linear.y * cos(th)) * time_diff.toSec();
    double delta_th = vel.angular.z * time_diff.toSec();
    // update odometry
    odom.header.stamp = measure_time;
    odom.header.frame_id = "odom";
    odom.pose.pose.position.x += delta_x;
    odom.pose.pose.position.y += delta_y;
    th += delta_th;
    odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(th);
    // set velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist = vel;
    
    return;
}

geometry_msgs::TransformStamped get_tf_from_odom(nav_msgs::Odometry odom)
{
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header = odom.header;
    odom_trans.child_frame_id = odom.child_frame_id;

    odom_trans.transform.translation.x = odom.pose.pose.position.x;
    odom_trans.transform.translation.y = odom.pose.pose.position.y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom.pose.pose.orientation;
    return odom_trans;
}

void vel_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
    ROS_DEBUG("Received message on cmd_vel");
    measure_time = ros::Time::now();
    ros::Duration dt = measure_time - last_vel;
    geometry_msgs::Twist vel = *msg;
    update_odom_from_vel(vel,dt);
    last_vel = measure_time;
    return;
}

} // end namespace

int main(int argc, char **argv)
{
    ros::init(argc,argv, "mobile_robot_simulator");
    ros::NodeHandle nh;
    
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",50);
    tf::TransformBroadcaster odom_broadcaster;
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel",50,mob_sim::vel_callback);
    
    ros::Time last_update = ros::Time::now();
    mob_sim::last_vel = last_update;
    
    mob_sim::update_odom_from_vel(geometry_msgs::Twist(), ros::Duration(0));
    
    mob_sim::odom.header.stamp = last_update;
    
    ROS_INFO("Started mobile robot simulator, listening on cmd_vel topic");
    
    ros::Rate r(5.0);
    while(nh.ok()){
        ros::spinOnce();
        ROS_INFO_STREAM("last_update: " << last_update.toSec());
        ROS_INFO_STREAM("last_vel: " << mob_sim::last_vel.toSec());
        if (last_update >= mob_sim::last_vel)
        {
            last_update = ros::Time::now();
            mob_sim::odom.header.stamp = last_update;
        }
        odom_pub.publish(mob_sim::odom);
        odom_broadcaster.sendTransform(mob_sim::get_tf_from_odom(mob_sim::odom));
        r.sleep();
    }
    
    return 0;
    
} // end main

// end namespace
