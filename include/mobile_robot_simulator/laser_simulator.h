#include "ros/ros.h"
#include "ros/console.h"

#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/GetMap.h"
#include "sensor_msgs/LaserScan.h"

#include "tf/transform_datatypes.h"
#include <tf/transform_listener.h>

#include <math.h>

#ifndef LASER_SIMULATOR
#define LASER_SIMULATOR

namespace laser_sim {

// map 
nav_msgs::OccupancyGrid map; //map data
nav_msgs::OccupancyGrid * map_ptr; 
bool have_map;

// laser
std::string l_frame;
float l_fov; // field of view, centered at pose of laser
unsigned int l_beams; // number of beams per scan
float l_max_range; // max range of the laser scan
float l_min_range; // min range of the laser scan
float l_frequency; // frequency of laser scans
ros::Time last_scan;

// output
sensor_msgs::LaserScan output_scan;

/*! gets the current map */
void get_map();

/*! updates the laser scanner parameters */
void set_laser_params(std::string frame_id, float fov, unsigned int beam_count, float max_range, float min_range, float l_frequency);

/*! finds the pose of the laser in the map frame */
void get_laser_pose(tf::TransformListener * tl, double * x, double * y, double * theta);

/*! updates the laser scan based on current 2D pose of the scanner in map coordinates */
void update_scan(double x, double y, double theta, sensor_msgs::LaserScan * scan);

/*! raytracing, calculates intersection with the map for a single ray */
double find_map_range(double x, double y, double theta);

/*! get map cell corresponding to real-world coordinates */
void get_world2map_coordinates(double x, double y, int * map_x, int * map_y);

/*! get real-world coordinates of map cell */
void get_map2world_coordinates(int map_x, int map_y, double * x, double * y);

/*! get occupancy of specified cell */
int get_map_occupancy(int x, int y);

}

#endif
