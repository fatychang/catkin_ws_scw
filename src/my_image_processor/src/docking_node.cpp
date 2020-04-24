#include "ros/ros.h"

// message
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"


// image processing
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/filters/passthrough.h"

// Declare publisher
ros::Publisher pub, pub2;

// Parameters for image processing
float leaf_size = 0.1;
int meanK = 50;
float epsAngle = 30.0f;

/** @brief The callback process the raw depth information to find the suitable docking location.
 * 
 */
void dockingCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        table_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	
	std_msgs::Float32MultiArray PtsMsg;		//message that will be published to ROS master
	PtsMsg.data.clear();
	// std::vector<float> pts;						//points that will be added to the messages and published
	// pts.clear();

    
    // Convert the ROS message to the PCLPointCloud2
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    // Convert from the PCLPointCloud2 to pcl::PointXYZ
    pcl::fromPCLPointCloud2(pcl_pc2, *origin_cloud);

    // Downsample - VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(origin_cloud);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox.filter(*filtered_cloud);
    std::cout << "[DEBUG]: the number of points:" << filtered_cloud->points.size() << std::endl;

    // Remove the ground plane with pass through filter (can use transfer function to replace the min_y part)
    float min_y = filtered_cloud -> points[0].y;
    int min_y_id;
    for (int i=1; i<filtered_cloud->points.size(); ++i)
    {
        if(filtered_cloud->points[i].y < min_y)
        {
            min_y = filtered_cloud->points[i].y;
            min_y_id = i;
        }
    }
    std::cout << "[DEBUG]: the minimum y:" << min_y << " at " << min_y_id << std::endl;
	PtsMsg.data.push_back(filtered_cloud->points[min_y_id].x), PtsMsg.data.push_back(filtered_cloud->points[min_y_id].y), PtsMsg.data.push_back(filtered_cloud->points[min_y_id].z);
	PtsMsg.data.push_back(filtered_cloud->points[0].x), PtsMsg.data.push_back(filtered_cloud->points[0].y), PtsMsg.data.push_back(filtered_cloud->points[0].z);


    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(filtered_cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits((min_y + 0.3f), 5);      // higher than ground for 30cm
    // pass.filter(*filtered_cloud);
    // std::cout << "[DEBUG]: the number of points (pass):" << filtered_cloud->points.size() << std::endl;


    // Convert to ROS message data type and publish
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*filtered_cloud.get(), output);
    pub.publish(output);

	// publish std_message::Float32Array containing the points information
	pub2.publish(PtsMsg);
}

/** @brief Shows all the parse message usage.
 * 
 */
static void showUsage(std::string name)
{
	std::cerr << "Usage: " << name << "option(s) SOURCES"
			<< "Options:\n"
			<< "\t -h, --help \t\t Show this help message\n"
			<< "\t -l, --leaf \t\t leaf_size size of the VoxelGrid filter (Default is 0.1)\n"
			<< "\t -a, --angle \t\t eps angle for the plane detection (Default is 30 degs.)\n" 
            << "\t -k, --meanK \t\t meanK for the statistical outlier removal (Default is 50)"<<std::endl;
}

/** @brief This node processes the depth input (ploint cloud) and to find the suitable docking location.
 * 
 * This node process the depth input (ploint cloud) from an rgbd camera to locate the possible docking 
 * location (usually in front of the table) and publish the location in terms of the camera frame.
 * 
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
    ros::init(argc, argv, "docking_node");
    ROS_INFO("[DEBUG] docking_node is now running...");

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
    ros::Subscriber sub = nh.subscribe("depth_noise", 1, dockingCallback);    


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
     pub = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
	 pub2 = nh.advertise<std_msgs::Float32MultiArray>("pointInfo", 1);

     ros::spin();

     return 0;
}