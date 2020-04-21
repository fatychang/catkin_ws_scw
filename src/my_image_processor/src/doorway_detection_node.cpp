#include "ros/ros.h"

#include "sensor_msgs/PointCloud2.h"

// image processing
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include "pcl/filters/project_inliers.h"
#include "pcl/kdtree/kdtree_flann.h"
#include "pcl/common/geometry.h"

#include "visualization_msgs/Marker.h"
#include "std_msgs/Float32MultiArray.h"

#include <Eigen/Dense>
#include <cmath>

ros::Publisher pub1, pub2;

// Parameters for image processing
float leave_size = 0.05;
int meanK = 50;
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

    std_msgs::Float32MultiArray gapPtsMsg;


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
        // ROS_INFO("[DEBUG]: The number of inliers:%d", inliers->indices.size() );

        // extract the inliers (keep the inliers)
        extract.setInputCloud(filtered_cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*door_cloud);

        // remove the outliers with statistical outlier removal filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(door_cloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(1.0);
        sor.filter(*door_cloud);


        // project the inliers onto the plane
        pcl::ProjectInliers<pcl::PointXYZ> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(door_cloud);
        proj.setModelCoefficients(coefficients);
        proj.filter(*door_cloud);

        int numOfPoints = door_cloud->points.size();
        ROS_INFO("[DEBUG]: The number of inliers:%d", numOfPoints);



        //  Stripe Scan //
        // find the minimal x and maxmum x to mark the start and end of the stripe scan
        float min_x = door_cloud -> points[0].x, max_x = door_cloud -> points[0].x;
        int min_x_id = 0, max_x_id = 0;
        for (int i=1; i<numOfPoints; ++i)
		{
			if(door_cloud->points[i].x < min_x)
			{
				min_x = door_cloud->points[i].x;
				min_x_id = i;
			}
			if(door_cloud->points[i].x > max_x)
			{
				max_x = door_cloud->points[i].x;
				max_x_id = i;
			}
		}
		// ROS_INFO("[DEBU]: min_x:%f at %d",min_x, min_x_id);
		// ROS_INFO("[DEBU]: max_x:%f at %d",max_x, max_x_id);

        //  Find door   //
        // k-d tree neighboring search
        pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
        kdtree.setInputCloud(door_cloud);
        float search_radius = 0.05;
        float terminate_threshold = 0.01;
        float step_size = 0.05;

        pcl::PointXYZ searchPoint, endPoint, searchPointPrev;
        int numOfNeighbors = 1, numOfNeighborsPrev = 1;
        searchPoint = door_cloud->points[min_x_id];
        endPoint = door_cloud->points[max_x_id];

        float door_width_min = 0.82;     //[m] follows American's with Disabilities Act (ADA)
        float door_width_max = 1.64;     //[m]

        pcl::PointXYZ gapStart, gapEnd;
        gapPtsMsg.data.clear();
        
        // search for the gap
       while (endPoint.x - searchPoint.x > terminate_threshold)
       {
            std::vector<int> pointId;       //stores the id of the poins that are within the radius
            std::vector<float> pointDist;   //stores the squre distance of the points that are within the raduis

            numOfNeighborsPrev = numOfNeighbors;
            numOfNeighbors = kdtree.radiusSearch(searchPoint, search_radius, pointId, pointDist);

            // ROS_INFO("[DEBUG] numOfNeighbors:%d", numOfNeighbors);
            // break;

            // check whether a door is found
            if(numOfNeighbors == 0 && numOfNeighborsPrev != 0)
            {
                // get the gap start position
                gapStart = searchPoint;
                // ROS_INFO("gap_start_pt: (%f, %f, %f)",gapStart.x, gapStart.y, gapStart.z);
            }
            else if(numOfNeighbors !=0 && numOfNeighborsPrev == 0)
            {
                // get the gap end position
                gapEnd = searchPoint;
                // ROS_INFO("gap_end_pt: (%f, %f, %f)",gapEnd.x, gapEnd.y, gapEnd.z);

                gapPtsMsg.data.push_back(gapStart.x), gapPtsMsg.data.push_back(gapStart.y),gapPtsMsg.data.push_back(gapStart.z);
                gapPtsMsg.data.push_back(gapEnd.x), gapPtsMsg.data.push_back(gapEnd.y),gapPtsMsg.data.push_back(gapEnd.z);


                // calculate the width of the gap whether within the door rang
                float gapWidth = pcl::geometry::distance(gapEnd, gapStart);
                // ROS_INFO("[DEBUG] gap width:%f", gapWidth);
                
                break;
            }

            // update the previous point and the next search point
            searchPointPrev = searchPoint;
            searchPoint.x += step_size * coefficients->values[2];
            searchPoint.z -= step_size * coefficients->values[0];
       }



    }

    // Convert to ROS message data type
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*door_cloud.get(), output);
    pub1.publish(output);
    pub2.publish(gapPtsMsg);
    









                                

}

/** @brief Shows all the parse message usage.
 * 
 */
static void showUsage(std::string name)
{
	std::cerr << "Usage: " << name << "option(s) SOURCES"
			<< "Options:\n"
			<< "\t -h, --help \t\t Show this help message\n"
			<< "\t -l, --leave \t\t Leave size of the VoxelGrid filter (Default is 0.08)\n"
			<< "\t -a, --angle \t\t eps angle for the plane detection (Default is 30 degs.)\n" 
            << "\t -k, --meanK \t\t meanK for the statistical outlier removal (Default is 50)"<<std::endl;
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
        else if((arg == "-k") || (arg == "-meanK"))
		{
			meanK = std::stof(argv[i+1]);
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
    ROS_INFO("[DEBUG] doorway_detection_node is now running...");

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
    // ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, doorwayCallback);

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
    pub1 = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    pub2 = nh.advertise<std_msgs::Float32MultiArray>("pointInfo",1);

    ros::spin();

    return 0;
}