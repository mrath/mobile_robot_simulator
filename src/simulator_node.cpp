#include "ros/ros.h"

#include "mobile_robot_simulator/mobile_robot_simulator.h"
#include "mobile_robot_simulator/laser_simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "mobile_robot_simulator");
    ros::NodeHandle nh;
    // publishers, subscribers
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom",50); // odometry publisher
    tf::TransformBroadcaster tf_broadcaster; // tf publisher
    tf::TransformListener tf_listener; 
    ros::Subscriber vel_sub = nh.subscribe("cmd_vel",5,mob_sim::vel_callback); // velocity subscriber
    ros::Subscriber init_pose_sub = nh.subscribe("initialpose",5,mob_sim::init_pose_callback); // initial pose callback
    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("scan",10); // odometry publisher
    // load parameters
    //TODO 
    
    ROS_INFO("Started mobile robot simulator, listening on cmd_vel topic");
    // global rate
    float rate = 10.0;
    
    // initialize timers
    ros::Time last_update = ros::Time::now();
    mob_sim::last_vel = last_update - ros::Duration(0.1);
    // initialize forst odom message
    mob_sim::update_odom_from_vel(geometry_msgs::Twist(), ros::Duration(0.1));
    mob_sim::odom.header.stamp = last_update;
    mob_sim::get_tf_from_odom(mob_sim::odom);
    // Initialize tf from map to odom
    mob_sim::map_trans.frame_id_ = "/map";
    mob_sim::map_trans.stamp_ = last_update;
    mob_sim::map_trans.child_frame_id_ = "/odom";
    mob_sim::map_trans.setIdentity();    
    
    // set laser parameters
    double l_x, l_y, l_theta;
    laser_sim::set_laser_params("base_link", M_PI, 100,25.0, 0.0, rate);
    laser_sim::last_scan = last_update;
    // get map
    laser_sim::get_map();
    // do the tf lookup only once for the laser scanner
    tf::StampedTransform rob_laser_tf;
    if (laser_sim::l_frame == "base_link") rob_laser_tf.setIdentity();
    else
    {
        tf_listener.waitForTransform("/base_link",laser_sim::l_frame,ros::Time(0),ros::Duration(1.5));
        tf_listener.lookupTransform("/base_link",laser_sim::l_frame,ros::Time(0),rob_laser_tf);
    }
    
    ros::Rate r(rate);
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
            mob_sim::odom_trans.stamp_ = last_update;
        }
        
        // publish odometry and tf
        odom_pub.publish(mob_sim::odom);
        mob_sim::get_tf_from_odom(mob_sim::odom);
        tf_broadcaster.sendTransform(mob_sim::odom_trans); // odom -> base_link
        mob_sim::map_trans.stamp_ = last_update;
        tf_broadcaster.sendTransform(mob_sim::map_trans); // map -> odom
        // update laser scan
        // first, get the pose of the laser in the map frame
        tf::Transform this_laser_tf = mob_sim::map_trans * mob_sim::odom_trans * rob_laser_tf;
        l_x = this_laser_tf.getOrigin().getX();
        l_y = this_laser_tf.getOrigin().getY();
        l_theta = tf::getYaw(this_laser_tf.getRotation());
        //ROS_INFO_STREAM_THROTTLE(2,"x: " << l_x << " y: " << l_y << " theta: " <<  l_theta);
        laser_sim::update_scan(l_x,l_y,l_theta,&laser_sim::output_scan);
        laser_sim::output_scan.header.stamp = last_update;
        laser_pub.publish(laser_sim::output_scan);
        
        
        r.sleep();
   
        mob_sim::message_received = false;
    }
    
    return 0;
    
} // end main
