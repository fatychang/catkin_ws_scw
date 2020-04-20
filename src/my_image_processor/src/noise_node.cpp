#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"

#include "pcl_conversions/pcl_conversions.h"
#include "cv_bridge/cv_bridge.h"

#include <Eigen/Dense>
#include <random>


ros::Publisher pub;


/** @brief callback for the noise node.
 * 
 * The nose callback add a Gaussian noise to the pointcloud raw data.
 */
void noiseCallback(const sensor_msgs::PointCloud2 &cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);

    // Convert from ROS sensor_msg::POintCloud2 to pcl_pointcloud2
    pcl_conversions::toPCL(cloud_msg, pcl_pc2);
    // Convert from pcl_pointCloud2 to pcl::pointCloud<pointXYZ>
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_ptr);

    // Define random generator with Gaussian distribution
    const float mean = 0.0;
    const float stddev = 0.1;
    std::default_random_engine generator;
    std::normal_distribution<float> dist(mean, stddev);
    


    // Extract the points from the cloud and store in a Eigen::Vector
    int numOfPoints = cloud_ptr->size();
    std::vector<Eigen::Vector3f> pts;
    for (size_t i=0; i<numOfPoints; ++i)
    {
        cloud_ptr->points[i].x += dist(generator);
        cloud_ptr->points[i].y += dist(generator);
        cloud_ptr->points[i].z += dist(generator);
    }
    // ROS_INFO("Data Acquired.");
    // std::cout << "pts: " << pts[10] << std::endl;

    // Convert to ROS data type 
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cloud_ptr.get(), output);

    // Publish the processed data
    pub.publish(output);



}


/** @brief This node generates a gaussian noisce to the pointcloud raw data
 * 
 * The simulated rgbd camera in the Gazebo are noise-free. A Gaussian noise is added to the raw data
 * to generate a more realistic depth information. The node subscribes the topic from the rgbd camera
 * ,add the noise to the raw data and publish the depth_noise topic to the ROS master.
 * 
 */
int main(int argc, char* argv[])
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
    ros::init(argc, argv, "noise_node");
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
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, noiseCallback);


	 /**
	  * The advertise() function is how you tell ROS that you want to
	  * publish on a given topic name. This invokes a call to the ROS
	  * master node, which keeps a registry of who is publishing and who
	  * is subscribing. After this advertise() call is made, the master
	  * node will notify anyone who is trying to subscribe to this topic name,
	  * and they will in turn negotiate a peer-to-peer connection with this
	  * node.  advertise() returns a Publisher object which allows you to
	  * publish messages on that topic through a call to publish().  Once
	  * all copies of the returned Publisher object are destroyed, the topic
	  * will be automatically unadvertised.
	  *
	  * The second parameter to advertise() is the size of the message queue
	  * used for publishing messages.  If messages are published more quickly
	  * than we can send them, the number here specifies how many messages to
	  * buffer up before throwing some away.
	  */
    pub = nh.advertise<sensor_msgs::PointCloud2>("depth_noise", 1);


    ros::spin();

    return 0;

}