#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"


/** @brief This node generates a gaussian noisce to the pointcloud raw data
 * 
 * The simulated rgbd camera in the Gazebo are noise-free. A Gaussian noise is added to the raw data
 * to generate a more realistic depth information. The node subscribes the topic from the rgbd camera
 * ,add the noise to the raw data and publish the depth_noise topic to the ROS master.
 * 
 */
int main(int argc, char* argv[])
{

}