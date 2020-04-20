#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"

// image processing
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/project_inliers.h"



#include <Eigen/Dense>
#include <cmath>

ros::Publisher pub;

// Parameters for image processing
float leave_size = 0.05;
float epsAngle = 30.0f;

/** @brief The callback process the raw depth information to find the possible doorway position.
 * 
 */
void doorwayCallback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        door_cloud (new pcl::PointCloud<pcl::PointXYZ>);

    // Convert from ROS message to pcl::pointcloud2
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    // Convert from pcl::pointcloud2 to pcl::pointcloud<PointXYZ>
    pcl::fromPCLPointCloud2(pcl_pc2, *origin_cloud);

    //  Downsample - VoxelGrid   //
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(origin_cloud);
    vox.setLeafSize(leave_size, leave_size, leave_size);
    vox.filter(*filtered_cloud);
    // ROS_INFO("[DEBUG] Size of filtered cloud:%d", filtered_cloud->size());

    //  Wall Detection - RANSAC //
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    Eigen::Vector3f axis = Eigen::Vector3f(0.0, 0.0, 1.0);      //search plane that is perpendicular to the axis
                                         
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setAxis(axis);
    seg.setEpsAngle(epsAngle * M_PI/180.f);
    seg.setDistanceThreshold(0.2);
    seg.setInputCloud(filtered_cloud);
    seg.segment(*inliers, *coefficients);

    // filter the outliers
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    if(inliers->indices.size() == 0)
    {
        ROS_INFO("Wall not found. Keep Searching...");
    }
    else
    {
        // extract the inliers (keep the inliers)
        extract.setInputCloud(filtered_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*filtered_cloud);

        // project the inliers onto the plane
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(filtered_cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*filtered_cloud);

        int numOfPoints = filtered_cloud->points.size();


        // Convert to ROS message data type
        sensor_msgs::PointCloud2 output;
        pcl::toROSMsg(*filtered_cloud.get(), output);
        pub.publish(output);
    }
    









                                

}

/** @brief Shows all the parse message usage.
 * 
 */
static void showUsage(std::string name)
{
	std::cerr << "Usage: " << name << "option(s) SOURCES"
			<< "Options:\n"
			<< "\t -h, --help \t\t Show this help message\n"
			<< "\t -l, --leave \t\t Leave size of the VoxelGrid filter (Default is 0.05)\n"
			<< "\t -a, --angle \t\t eps angle for the plane detection (Default is 30 degs.)\n" <<std::endl;
}


/** @brief This node process the depth data and find the doorway start and end points in camera frame.
 * 
 * This node process the depth data with the following steps to find the possible doorway starting and end points
 * 1. Downsampling with the Voxelgrid
 * 2. Find the plane (inliers) with RANSAC
 * 3. Project the inliers to the plance
 */
int main(int argc, char **argv)
{

    // Parse the commend line
	for (int i=0; i<argc; ++i)
	{
		std::string arg = argv[i];

		if ((arg == "-h") || (arg == "--help"))
		{
			showUsage(argv[0]);
			return 0;
		}
		else if((arg == "-l") || (arg == "-leave"))
		{
			leave_size = std::stof(argv[i+1]);
		}
		else if((arg == "-a") || (arg == "-angle"))
		{
			epsAngle = std::stof(argv[i+1]);
		}
	}

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

    ros::spin();

    return 0;
}