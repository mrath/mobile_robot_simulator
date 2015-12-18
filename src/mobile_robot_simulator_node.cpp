#include "ros/ros.h"

#include "mobile_robot_simulator/mobile_robot_simulator.h"

int main(int argc, char **argv)
{
    ros::init(argc,argv, "mobile_robot_simulator");
    ros::NodeHandle nh;
       
    MobileRobotSimulator mob_sim(&nh);
    
    ROS_INFO("--- Starting MobileRobot simulator");
    
    ros::Duration(0.5).sleep();
     
    mob_sim.start();
    
    while (nh.ok()) {
        ros::spinOnce();
        ros::Duration(0.01).sleep();
    }
    
    mob_sim.stop();
    
    return 0;
    
} // end main
