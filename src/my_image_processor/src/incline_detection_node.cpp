#include "ros/ros.h"

// message
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"

// image processing
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/region_growing.h>

ros::Publisher pub_obj, pub_filtered, pub_pts,pub_normals, pub_pts2; 

// Parameters for image processing
float leaf_size = 0.1;
int meanK = 50;
float radius = 30.0f;
float p = 1.0;
float k = 1.0;

/** @brief The callback process the raw depth information to decide whether the ramp is safe to transver.
 */
void inclineCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
	pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        incline_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	
	std_msgs::Float32MultiArray ptsMsg, ptsMsg2, lineMsg, normalsMsg;		//message that will be published to ROS master
	ptsMsg.data.clear();
	ptsMsg2.data.clear();
	lineMsg.data.clear();
	normalsMsg.data.clear();


	// Convert the ROS message to the PCLPointCloud2
    pcl_conversions::toPCL(*cloud_msg, pcl_pc2);
    // Convert from the PCLPointCloud2 to pcl::PointXYZ
    pcl::fromPCLPointCloud2(pcl_pc2, *origin_cloud);

	// Downsample - VoxelGrid
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(origin_cloud);
    vox.setLeafSize(leaf_size, leaf_size, leaf_size);
    vox.filter(*filtered_cloud);
    //std::cout << "[DEBUG]: the number of points:" << filtered_cloud->points.size() << std::endl;

	// remove the outliers with statistical outlier removal filter
	pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
	sor.setInputCloud(filtered_cloud);
	sor.setMeanK(meanK);
	sor.setStddevMulThresh(1.0);
	sor.filter(*filtered_cloud);
    // std::cout << "[DEBUG]: the number of points:" << filtered_cloud->points.size() << std::endl;


	////////// Estimate the normals
	// create the normal estimation class and pass the input dataset to it
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	ne.setInputCloud(filtered_cloud);
	// create an empty kdtree representation and pass it to the normal estimation object
	// its content will be filled inside the object, based on the given dataset
	pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());
	ne.setSearchMethod(tree);
	// output datasets
	pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
	// use all neighbors in a sphere of radius 0.1 m (for noise-free data)
	ne.setRadiusSearch(0.1);
	// compute the features
	ne.compute(*cloud_normals);
    //std::cout << "[DEBUG]: the number of points in normal cloud" << cloud_normals->points.size() << std::endl;
    // std::cout << "[DEBUG]: the normal of points in normal cloud" << cloud_normals->points[98].normal_x <<" " << cloud_normals->points[98].normal_y
	// << " "<< cloud_normals->points[98].normal_z << std::endl;
	// std::cout << "[DEBUG]: the position of points in normal cloud" << filtered_cloud->points[98].x<<" " << filtered_cloud->points[98].y
	// << " "<< filtered_cloud->points[98].z << std::endl;

	// visualize the normals
	for (int i=0; i< filtered_cloud->points.size(); ++i)
	{
		float len = 0.5;
		normalsMsg.data.push_back(filtered_cloud->points[i].x);
		normalsMsg.data.push_back(filtered_cloud->points[i].y);
		normalsMsg.data.push_back(filtered_cloud->points[i].z);
		normalsMsg.data.push_back(cloud_normals->points[i].normal_x * len + filtered_cloud->points[i].x);
		normalsMsg.data.push_back(cloud_normals->points[i].normal_y * len + filtered_cloud->points[i].y);
		normalsMsg.data.push_back(cloud_normals->points[i].normal_z * len + filtered_cloud->points[i].z);
	}

	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(20);
	reg.setMaxClusterSize(1000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(radius);
	reg.setInputCloud(filtered_cloud);
	reg.setInputNormals(cloud_normals);
	reg.setSmoothModeFlag(0.5 / 180 * M_PI);
	reg.setCurvatureThreshold(0.5);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	int numOfClusters = clusters.size();
	std::cout << "number of clusters: " << numOfClusters << std::endl;
	// std::cout << "number of points in the clusters: " << clusters[0].indices.size() << std::endl;
	// std::cout << "number of points in the clusters: " << clusters[1].indices.size() << std::endl;

	pcl::ExtractIndices<pcl::PointXYZ> extract;
	std::vector<Eigen::Vector3f> avgNormals;
	for (int i=0; i<numOfClusters; ++i)
	{
		//Extract the indices from the clusters
		pcl::PointIndices::Ptr tmp_cluster (new pcl::PointIndices);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		tmp_cluster->indices = clusters[i].indices;
		extract.setInputCloud(filtered_cloud);
		extract.setIndices(tmp_cluster);
		extract.setNegative(false);
		extract.filter(*incline_cloud);

		std::cout << "number of points in the clusters" << i << " " << clusters[i].indices.size() << std::endl;


		// calculate the average normals of the clusters
		Eigen::Vector3f tmp = Eigen::Vector3f::Zero();
		int counter=0;
		for (int j=0; j<clusters[i].indices.size(); ++j)
		{
			// std::cout << " " << cloud_normals->points[clusters[i].indices[j]].normal_x << std::endl;
			if ( !isnan(cloud_normals->points[clusters[i].indices[j]].normal_x))
			{
				if (!isnan(cloud_normals->points[clusters[i].indices[j]].normal_y))
				{
					if (!isnan(cloud_normals->points[clusters[i].indices[j]].normal_z))
					{
						tmp(0) += cloud_normals->points[clusters[i].indices[j]].normal_x;
						tmp(1) += cloud_normals->points[clusters[i].indices[j]].normal_y;
						tmp(2) += cloud_normals->points[clusters[i].indices[j]].normal_z;
						counter ++;
						//std::cout << "( " << cloud_normals->points[clusters[i].indices[j]].normal_x << ", " << cloud_normals->points[clusters[i].indices[j]].normal_y << ", " << cloud_normals->points[clusters[i].indices[j]].normal_z << std::endl;
					}					
				}
			}
		}
		// std::cout << "counter: " << counter << std::endl;
		avgNormals.push_back(tmp /= counter);
		std::cout << "avg Normal in cluster " << i << " :" << avgNormals[i] << std::endl;

		// std::vector<int> vote_clusters;
		// for(int i=0; i<avgNormals.size(); ++i)
		// {
		// 	vote_clusters.push_back(0);
		// 	if(avgNormals[i](0) > 0.3)
		// 		vote_clusters[i] ++;
		// 	if(avgNormals[i](1) > 0.3)
		// 		vote_clusters[i] ++;
		// 	if(avgNormals[i](2) > 0.3)
		// 		vote_clusters[i] ++;											
		// }



		//cluster visualization
		if(i==0)
		{
			// std::cout << "number of points in the " << i << " clusters: " << incline_cloud->points.size() << std::endl;
			for (int j=0; j<incline_cloud->points.size(); ++j)
			{
				ptsMsg.data.push_back(incline_cloud->points[j].x);
				ptsMsg.data.push_back(incline_cloud->points[j].y);
				ptsMsg.data.push_back(incline_cloud->points[j].z);
			}
		}
		else if(i==1)
		{
			// std::cout << "number of points in the " << i << " clusters: " << incline_cloud->points.size() << std::endl;
			for (int j=0; j<incline_cloud->points.size(); ++j)
			{
				ptsMsg2.data.push_back(incline_cloud->points[j].x);
				ptsMsg2.data.push_back(incline_cloud->points[j].y);
				ptsMsg2.data.push_back(incline_cloud->points[j].z);
			}
		}
		else
		{
			std::cout << "Unable to visualize the cluster >=3" << ", now is cluster:" << i << std::endl;
		}		
	}
	
	


	
	



	// Convert to ROS message data type and publish
    sensor_msgs::PointCloud2 output, output2;
    pcl::toROSMsg(*filtered_cloud.get(), output);
    pcl::toROSMsg(*incline_cloud.get(), output2);
    pub_filtered.publish(output);
    pub_obj.publish(output2);

	// // publish std_message::Float32Array containing the points information
	pub_pts.publish(ptsMsg);
	pub_normals.publish(normalsMsg);
	pub_pts2.publish(ptsMsg2);
	// pub_line.publish(lineMsg);

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
			<< "\t -r, --radius \t\t \n" 
            << "\t -p, --param \t\t tmp param for tuning params"<<std::endl;
}

/** @brief Incline detection node identifies the location of the ramp and decides whether it's safe to traversal.
 * 
 * 
 */
int main (int argc, char **argv)
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
		else if((arg == "-l") || (arg == "-leaf"))
		{
			leaf_size = std::stof(argv[i+1]);
		}
		else if((arg == "-r") || (arg == "-radius"))
		{
			radius = std::stof(argv[i+1]);
		}
        else if((arg == "-p") || (arg == "-param"))
		{
			p = std::stof(argv[i+1]);
		}
		else if((arg == "-k") || (arg == "-kparam"))
		{
			k = std::stof(argv[i+1]);
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
    ros::init(argc, argv, "incline_detection_node");
    ROS_INFO("[DEBUG] incline_detection_node is now running...");

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
    // ros::Subscriber sub = nh.subscribe("depth_noise", 1, inclineCallback);    
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, inclineCallback);

    

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
    pub_obj = nh.advertise<sensor_msgs::PointCloud2>("object_cloud", 1);
    pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
    pub_pts= nh.advertise<std_msgs::Float32MultiArray>("pointInfo",1);
	pub_pts2 = nh.advertise<std_msgs::Float32MultiArray>("pointInfo2", 1);
	pub_normals = nh.advertise<std_msgs::Float32MultiArray>("normalsInfo", 1);
	// pub_line = nh.advertise<std_msgs::Float32MultiArray>("lineInfo", 1);


    ros::spin();

    return 0;
}