#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"

#include "pcl_conversions/pcl_conversions.h"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/Dense>
#include <random>

/** @brief The callback process the raw depth information to find the possible doorway position.
 * 
 */
void doorwayCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    
}


/** @brief This node process the depth data and find the doorway start and end points in camera frame.
 * 
 * This node process the depth data with the following steps to find the possible doorway starting and end points
 * 1. Downsampling with the Voxelgrid
 * 2. 
 */
int main(int argc, char **argv)
{
    /**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
    ros::init(argc, argv, "doorway_detection_node");
    ROS_INFO("[DEBUG] noise_node is now running...");

    /**
	 * NodeHandle is the main access point to communications with the ROS system.
	 * The first NodeHandle constructed will fully initialize this node, and the last
	 * NodeHandle destructed will close down the node.
	 */
    ros::NodeHandle nh;

    /**
  	 * The subscribe() call is how you tell ROS that you want to receive messages
  	 * on a given topic.  This invokes a call to the ROS
	 * master node, which keeps a registry of who is publishing and who
	 * is subscribing.  Messages are passed to a callback function, here
	 * called chatterCallback.  subscribe() returns a Subscriber object that you
	 * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
	 * object go out of scope, this callback will automatically be unsubscribed from
	 * this topic.
 	 *
	 * The second parameter to the subscribe() function is the size of the message
	 * queue.  If messages are arriving faster than they are being processed, this
	 * is the number of messages that will be buffered up before beginning to throw
	 * away the oldest ones.
	 */
    ros::Subscriber sub = nh.subscribe("depth_noise", 1, doorwayCallback);

    return 0;
}