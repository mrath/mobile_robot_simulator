#include "ros/ros.h"

#include "mobile_robot_simulator/laser_simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "laser_simulator");
    ros::NodeHandle nh;
        
    LaserScannerSimulator laser_sim(&nh);
    
    ROS_INFO("--- Starting LaserScanner simulator");
    
    ros::Duration(0.5).sleep();
    
    laser_sim.start();
    
    while (nh.ok()) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }

    laser_sim.stop();
    
    return 0;
    
} // end main
