#include "ros/ros.h"

// message
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Float32MultiArray.h"


// image processing
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/segmentation/sac_segmentation.h"
#include "pcl/filters/extract_indices.h"
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"


//#include "pcl/filters/passthrough.h"

// others
#include <Eigen/Dense>

// Declare publisher
ros::Publisher pub_obj, pub_filtered, pub_pts;

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
    //std::cout << "[DEBUG]: the number of points:" << filtered_cloud->points.size() << std::endl;

    // Remove the ground plane with pass through filter (can use transfer function to replace the min_y part)
    // [NOTE 1] The image from the camera is required to roate 90 degress to match the actual world axis. 
	// Therefor, the minimun of y is now the maximun of y.
	// [Note 2] This only works that the camera has zero pitch.
	float max_y = filtered_cloud -> points[0].y;
    int max_y_id;
    for (int i=1; i<filtered_cloud->points.size(); ++i)
    {
        if(filtered_cloud->points[i].y > max_y)
        {
            max_y = filtered_cloud->points[i].y;
            max_y_id = i;
        }
    }
    //std::cout << "[DEBUG]: the minimum y:" << max_y << " at " << max_y_id << std::endl;
	//PtsMsg.data.push_back(filtered_cloud->points[max_y_id].x), PtsMsg.data.push_back(filtered_cloud->points[max_y_id].y), PtsMsg.data.push_back(filtered_cloud->points[max_y_id].z);


    // pcl::PassThrough<pcl::PointXYZ> pass;
    // pass.setInputCloud(filtered_cloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-5, max_y-0.01f);      // higher than ground for 30cm
    // pass.filter(*filtered_cloud);
    //std::cout << "[DEBUG]: the number of points (pass):" << filtered_cloud->points.size() << std::endl;

	// RANSAC to search the potential table surface
	pcl::ModelCoefficients::Ptr coefficient (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	Eigen::Vector3f axis = Eigen::Vector3f(0.0, 1.0, 0.0);

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setAxis(axis);
	//seg.setEpsAngle()
	seg.setDistanceThreshold(0.2);
	seg.setInputCloud(filtered_cloud);
	seg.segment(*inliers, *coefficient);
	//std::cout << "[DEBUG]: coefficient: (" << coefficient->values[0] << " " << coefficient->values[1] << " " << coefficient->values[2] << std::endl;

	// filter the outliers
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	if(inliers->indices.size() == 0)
	{
		ROS_INFO("Table not found. Keep searching...");
	}
	else
	{
        // ROS_INFO("[DEBUG]: The number of inliers:%d", inliers->indices.size() );

		// extract the inliers (keep the inliers)
		extract.setInputCloud(filtered_cloud);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*table_cloud);

		// Check whether the candidate satisfies the following thress constrains.
		if(fabs(coefficient->values[1]) > 0.98) 							// 1. plane is perpendicular to the ground
		{
			int minimul_num_of_points = 100;
			if(table_cloud->points.size() > minimul_num_of_points)			// 2. the number of points exceed a certain threshold
			{
				// calculate the average height (y) of all the points in the plane
				float avgTableHeight = 0.0;
				for (int i=0; i< table_cloud->points.size(); ++i)
				{
					avgTableHeight+=table_cloud->points[i].y;
				}
				avgTableHeight /= table_cloud->points.size();
				// std::cout << "[DEBUG]: avg height: " << avgTableHeight << std::endl;

				float camera_height = 1.3;
				float minimul_table_height = 0.7112;  //around 28'
				if(avgTableHeight < camera_height - minimul_table_height)	// 3. the height of the table exceeds the threshold
				{
					// project the model inliers to the plane
					pcl::ProjectInliers<pcl::PointXYZ> proj;
					proj.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
					proj.setModelCoefficients(coefficient);
					proj.setInputCloud(table_cloud);
					proj.filter(*table_cloud);

					// create a Convex Hull representation
					pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud (new pcl::PointCloud<pcl::PointXYZ>);
					pcl::ConvexHull<pcl::PointXYZ> hull;
					hull.setInputCloud(table_cloud);
					hull.reconstruct(*hull_cloud);

					// update points to the message for visualization
					for(int i=0; i<hull_cloud->points.size(); ++i)
					{
						pcl::PointXYZ p;
						p.x = hull_cloud->points[i].x; p.y = hull_cloud->points[i].y; p.z = hull_cloud->points[i].z; 
						PtsMsg.data.push_back(p.x); PtsMsg.data.push_back(p.y); PtsMsg.data.push_back(p.z); 						
					}


				}
			}

		}
	}




    // Convert to ROS message data type and publish
    sensor_msgs::PointCloud2 output, output2;
    pcl::toROSMsg(*filtered_cloud.get(), output);
    pcl::toROSMsg(*table_cloud.get(), output2);
    pub_filtered.publish(output);
    pub_obj.publish(output2);

	// publish std_message::Float32Array containing the points information
	pub_pts.publish(PtsMsg);
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
     pub_obj = nh.advertise<sensor_msgs::PointCloud2>("object_cloud", 1);
     pub_filtered = nh.advertise<sensor_msgs::PointCloud2>("filtered_cloud", 1);
	 pub_pts = nh.advertise<std_msgs::Float32MultiArray>("pointInfo", 1);

     ros::spin();

     return 0;
}