# mobile_robot_simulator
A simple ROS simulator for mobile robots. Subscribes to incoming velocity commands, and updates odometry based on this. Also publishes localization on tf.

This is useful if some high-level simulation is needed, inspired by the industrial_robot_simulator package in ROS Industrial.

Sensor information is not simulated, so if this driver is used with the regular ROS navigation stack, localization must be turned off, and local costmap set to the same as the global costmap to avoid local plans that are invalid.

## Subscriptions
- `/cmd_vel` (geometry_msgs/Twist) - velocity commands 
- `/initialpose` (geometry_msgs/PoseWithCovarianceStamped) - current pose estimate of the robot with respect to the /map frame

## Publications
- `/odom` (nav_msgs/Odometry) - odometry of the mobile robot, calculated based on the incoming velocity commands
- `/tf` - publishes 2 transforms: /odom -> /base_link and /map -> /odom
