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
#include "pcl/surface/convex_hull.h"
#include <pcl/common/common.h>		//for distance.h

#define PI 3.14159265

ros::Publisher pub_obj, pub_filtered, pub_pts,pub_normals, pub_pts2, pub_line; 

// Parameters for image processing
float leaf_size = 0.1;
int meanK1 = 50;
int meanK2 = 10;
float radius = 30.0f;
float p = 10.0;
float k = 1.0;

/** @brief return the index of the minimul value
 * 
 * @param vect a std vector of floating points 
 * @return the index of the minimul value is returned
 */
int findMinIndex(std::vector<float> vect)
{
	int id=0;
	float min=vect[0];
	for(int i=1; i<vect.size(); ++i)
	{
		if (vect[i] < min)
		{
			id = i;
			min =vect[i];
		}
	}
	return id;
}

/** @brief return the index of the minimul value.
 * 
 * @param vect a std vector of pcl::PointXYZ
 * @param axis the axis where we are comparing. the argument is expected to 'x'(0), 'y'(1) or 'z'(2)
 * @return the index of the minimul value is returned
 */
int findMinIndex(std::vector<pcl::PointXYZ> vect, int axis)
{
	int id=0;
	if (axis=0)
	{
		float min = vect[0].x;
		for(int i=1; i<vect.size(); i++)
		{
			if (vect[i].x < min)
			{
				min = vect[i].x;
				id = i;
			}
		}
	}
	else if (axis=1)
	{
		float min = vect[0].y;
		for(int i=1; i<vect.size(); i++)
		{
			if (vect[i].y < min)
			{
				min = vect[i].y;
				id = i;
			}
		}
	}
	else if (axis=2)
	{
		float min = vect[0].z;
		for(int i=1; i<vect.size(); i++)
		{
			if (vect[i].z < min)
			{
				min = vect[i].z;
				id = i;
			}
		}
	}
	else
	{
		std::cout << "[ERROR] Invalid axis argument. It must be either 'x', 'y' or 'z'." << std::endl;
		id = -1;
	}

	return id;
}

/** @brief return the euclidean distance between two given points in 3D.
 * 
 * @param p1 points1 in the type of pcl::PointXYZ
 * @param p2 points1 in the type of pcl::PointXYZ
 * @return the euclidean distance between two points in 3D.
 */
float findDistance(pcl::PointXYZ p1, pcl::PointXYZ p2)
{
	float diff_x = p2.x - p1.x, diff_y = p2.y - p1.y, diff_z = p2.z - p1.z;
	return (sqrt(diff_x*diff_x + diff_y*diff_y + diff_z*diff_z));
}

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
	sor.setMeanK(meanK1);
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


	// // visualize the normals
	// for (int i=0; i< filtered_cloud->points.size(); ++i)
	// {
	// 	// std::cout << "[DEBUG]: (" << cloud_normals->points[i].normal_x <<" " << cloud_normals->points[i].normal_y
	// 	// 	<< " "<< cloud_normals->points[i].normal_z << ")" << std::endl;
		
	// 	if(isnan(cloud_normals->points[i].normal_x) || isnan(cloud_normals->points[i].normal_y) || isnan(cloud_normals->points[i].normal_z))
	// 		continue;
		
	// 	float len = 0.5;
	// 	normalsMsg.data.push_back(filtered_cloud->points[i].x);
	// 	normalsMsg.data.push_back(filtered_cloud->points[i].y);
	// 	normalsMsg.data.push_back(filtered_cloud->points[i].z);
	// 	normalsMsg.data.push_back(cloud_normals->points[i].normal_x * len + filtered_cloud->points[i].x);
	// 	normalsMsg.data.push_back(cloud_normals->points[i].normal_y * len + filtered_cloud->points[i].y);
	// 	normalsMsg.data.push_back(cloud_normals->points[i].normal_z * len + filtered_cloud->points[i].z);
	// }


	// Merge points that are close in terms of smoothness and curvature
	pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
	reg.setMinClusterSize(20);
	reg.setMaxClusterSize(1000);
	reg.setSearchMethod(tree);
	reg.setNumberOfNeighbours(radius);
	reg.setInputCloud(filtered_cloud);
	reg.setInputNormals(cloud_normals);
	reg.setSmoothModeFlag(5 / 180 * M_PI);
	reg.setCurvatureThreshold(2);

	std::vector<pcl::PointIndices> clusters;
	reg.extract(clusters);

	std::cout << "number of clusters: " << clusters.size() << std::endl;
	// std::cout << "number of points in the clusters: " << clusters[1].indices.size() << std::endl;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_cloud (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	std::vector<Eigen::Vector3f> avgNormals;
	int groundCluster, inclineCluster;
	groundCluster = inclineCluster = -1;

	for (int i=0; i<clusters.size(); ++i)
	{
		std::cout << "[DEBUG] # of points in the clusters " << i << ": "<< clusters[i].indices.size() << std::endl;

		//Extract the indices from the clusters
		pcl::PointIndices::Ptr tmp_cluster (new pcl::PointIndices);
		// pcl::PointCloud<pcl::PointXYZ>::Ptr tmp_cloud (new pcl::PointCloud<pcl::PointXYZ>);
		tmp_cluster->indices = clusters[i].indices;
		extract.setInputCloud(filtered_cloud);
		extract.setIndices(tmp_cluster);
		extract.setNegative(false);
		extract.filter(*cluster_cloud);

		// if (cluster_cloud->points.size() == 0)
		// 	continue;


		// // [DEBUG] cluster visualization
		// if(i==0)
		// {
		// 	// std::cout << "number of points in the " << i << " clusters: " << cluster_cloud->points.size() << std::endl;
		// 	for (int j=0; j<cluster_cloud->points.size(); ++j)
		// 	{
		// 		ptsMsg.data.push_back(cluster_cloud->points[j].x);
		// 		ptsMsg.data.push_back(cluster_cloud->points[j].y);
		// 		ptsMsg.data.push_back(cluster_cloud->points[j].z);
		// 	}
		// }
		// else if(i==1)
		// {
		// 	// std::cout << "number of points in the " << i << " clusters: " << cluster_cloud->points.size() << std::endl;
		// 	for (int j=0; j<cluster_cloud->points.size(); ++j)
		// 	{
		// 		ptsMsg2.data.push_back(cluster_cloud->points[j].x);
		// 		ptsMsg2.data.push_back(cluster_cloud->points[j].y);
		// 		ptsMsg2.data.push_back(cluster_cloud->points[j].z);
		// 	}
		// }
		// else
		// {
		// 	std::cout << "Unable to visualize the cluster >=3" << ", now is cluster:" << i << std::endl;
		// }	


		// calculate the average normals of the clusters
		Eigen::Vector3f tmp = Eigen::Vector3f::Zero();
		int counter=0;
		for (int j=0; j<clusters[i].indices.size(); ++j)
		{
			// Exclude points that have nan normal
			if(isnan(cloud_normals->points[clusters[i].indices[j]].normal_x) || isnan(cloud_normals->points[clusters[i].indices[j]].normal_y) || isnan(cloud_normals->points[clusters[i].indices[j]].normal_z))
				continue;
			
			tmp(0) += cloud_normals->points[clusters[i].indices[j]].normal_x;
			tmp(1) += cloud_normals->points[clusters[i].indices[j]].normal_y;
			tmp(2) += cloud_normals->points[clusters[i].indices[j]].normal_z;
			counter ++;
			// std::cout <<"normal:(" <<cloud_normals->points[clusters[i].indices[j]].normal_x << "," 
			// << cloud_normals->points[clusters[i].indices[j]].normal_y<< "," 
			// << cloud_normals->points[clusters[i].indices[j]].normal_z << std::endl;  
		}
		// std::cout << "counter: " << counter << std::endl;
		avgNormals.push_back(tmp /= counter);
		std::cout << "[DEBUG] avg Normal in cluster " << i << " :(" << avgNormals[i][0] << "," << avgNormals[i][1] << "," << avgNormals[i][2] << ")" << std::endl;

		// Detect the ground plane
		if(fabs(avgNormals[i][1]) > 0.8 && fabs(avgNormals[i][0]) < 0.1 && fabs(avgNormals[i][2]) < 0.1)
		{
			std::cout << "[DEBUG] Ground plane removed. cluster: " << i << std::endl;
			groundCluster = i;
			continue;
		}

		std::cout << std::endl;
		
		//Detect the possible inclined plane
		if(fabs(avgNormals[i][1]) < 0.8)	// normal is not perpendicular to the ground
		// if(fabs(avgNormals[i][1]) > 0.8 && fabs(avgNormals[i][0]) < 0.1 && fabs(avgNormals[i][2]) < 0.1)
		{
			// Fit the plane with RANSAC
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    		pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    		pcl::SACSegmentation<pcl::PointXYZ> seg;

			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			// seg.setAxis(axis);
			// seg.setEpsAngle(epsAngle * M_PI/180.f);
			seg.setDistanceThreshold(0.2);
			seg.setInputCloud(cluster_cloud);
			seg.segment(*inliers, *coefficients);

			// std::cout <<"[DEBUG] # of inliers:" << inliers->indices.size() << "		coefficient:(" 
			// << coefficients->values[0] <<"," <<coefficients->values[1] << "," <<coefficients->values[2] << std::endl;

			// Extracts the inliers (keep only the inliers)
			if (inliers->indices.size() < 10)
				continue;

    		pcl::ExtractIndices<pcl::PointXYZ> ext;

			// keep the inliers
			std::cout << "[DEBUG] potential inclined plane found. Cluster:" << i << std::endl;
			inclineCluster = i;
			ext.setInputCloud(cluster_cloud);
			ext.setIndices(inliers);
			ext.setNegative(false);
			ext.filter(*incline_cloud);

			// remove the outliers with statistical outlier removal filter
			pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
			sor.setInputCloud(incline_cloud);
			sor.setMeanK(meanK2);
			sor.setStddevMulThresh(1.0);
			sor.filter(*incline_cloud);

			//create a Convex Hull representation
			pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ConvexHull<pcl::PointXYZ> hull;
			hull.setInputCloud(incline_cloud);
			hull.reconstruct(*hull_cloud);
			std::cout <<"[DEBUG] # of hull points:" << hull_cloud->points.size() << std::endl;

			if(hull_cloud->points.size() == 0)
				continue;

			// visualize the convex hull (contour and points)
			for(int i=0; i<hull_cloud->points.size(); ++i)
			{
				ptsMsg2.data.push_back(hull_cloud->points[i].x); ptsMsg2.data.push_back(hull_cloud->points[i].y); ptsMsg2.data.push_back(hull_cloud->points[i].z); 						
				lineMsg.data.push_back(hull_cloud->points[i].x); lineMsg.data.push_back(hull_cloud->points[i].y); lineMsg.data.push_back(hull_cloud->points[i].z); 	
			}

			// calculate the slope between two consecutive hull points
			std::vector<Eigen::Vector3f> directions;
			for(int i=0; i<hull_cloud->points.size(); ++i)
			{
				Eigen::Vector3f direction;
				pcl::PointXYZ point1, point2;				
				if (i==hull_cloud->points.size()-1)
				{
					point2 = hull_cloud->points[0];
					point1 = hull_cloud->points[i];				
				}
				else
				{
					point2 = hull_cloud->points[i+1];
					point1 = hull_cloud->points[i];
				}
				float mag = sqrt(pow(point2.x-point1.x, 2)+pow(point2.y-point1.y, 2)+pow(point2.z-point1.z, 2));
				direction << point2.x-point1.x, point2.y-point1.y, point2.z-point1.z;
				direction /= mag;
				directions.push_back(direction);
			}

			// calculate the dot of the two consecutive directions
			std::vector<float> dots;
			for(int i=0; i<directions.size(); ++i)
			{
				float product;
				if (i==directions.size()-1)
				{
					product = directions[i].dot(directions[0]);			
				}
				else
				{
					product = directions[i].dot(directions[i+1]);			

				}
				dots.push_back(product);
				//std::cout << i << "dot:" << product << std::endl;
			}
			// for(int i=0; i<tmp.size();++i)
			// {
			// 	std::cout << i <<":" << tmp[i] <<std::endl;
			// }
			// std::cout <<findMinIndex(tmp);


			std::vector<int> cornerId;
			std::vector<float> tmp(dots);
			for(int i=0; i<4; ++i)
			{
				cornerId.push_back(findMinIndex(tmp));			
				tmp[cornerId[i]] = 100;
			}

			// update the corners
			std::vector<pcl::PointXYZ> corners;
			for (int i=0; i<cornerId.size(); ++i)
			{
				if(cornerId[i] == hull_cloud->points.size()-1)
					corners.push_back(hull_cloud->points[0]);
				else
					corners.push_back(hull_cloud->points[cornerId[i]+1]);
				
				// visualize the corner
				ptsMsg.data.push_back(corners[i].x); ptsMsg.data.push_back(corners[i].y); ptsMsg.data.push_back(corners[i].z); 						;
			}

			// get the width of the ramp
			std::vector<float> rampLengths;
			rampLengths.push_back(findDistance(corners[1], corners[0]));
			rampLengths.push_back(findDistance(corners[2], corners[1]));
			rampLengths.push_back(findDistance(corners[3], corners[2]));
			rampLengths.push_back(findDistance(corners[0], corners[3]));
			std::cout << "[DEBUG] The length of the rectangle ramp: (" << rampLengths[0] << ", " << rampLengths[1] << ", "
						<< rampLengths[2] << ", " << rampLengths[3] << std::endl;;


			// identify the two corner points that are closer to the camera (at the loanding position)
			// NOTED, the y value is upside down in the rviz therefore minimul y means highest point in the view
			std::vector<int> landingId, rampId;		// stores the id of the corner points which have the smaller y (at the ramp)
			std::vector<pcl::PointXYZ> tmp2(corners);
			pcl::PointXYZ discardPt; discardPt.x=1000, discardPt.y=1000, discardPt.z=1000;
			for (int i=0; i<2; ++i)
			{
				landingId.push_back(findMinIndex(tmp2, 1)); // the second argument is to indicate the axis. x=0, y=1, z=2
				tmp2[landingId[i]] = discardPt;
				//std::cout << "index: " << landingId[i] << std::endl;
			}
			
			// calculate the centers Cn, Cf of the flush transitions of the ramp to landings (simplified)
			Eigen::Vector3f cn, cf;
			cf << (corners[landingId[0]].x + corners[landingId[1]].x)/2, 
					(corners[landingId[0]].y + corners[landingId[1]].y)/2, 
					(corners[landingId[0]].z + corners[landingId[1]].z)/2;
			ptsMsg.data.push_back(cf(0)); ptsMsg.data.push_back(cf(1)); ptsMsg.data.push_back(cf(2));
			
			for(int i=0; i<4; ++i)
			{
				if(i == landingId[0] || i== landingId[1])
					continue;
				rampId.push_back(i);
			}			 			
			cn << (corners[rampId[0]].x + corners[rampId[1]].x)/2, 
					(corners[rampId[0]].y + corners[rampId[1]].y)/2, 
					(corners[rampId[0]].z + corners[rampId[1]].z)/2;	
			ptsMsg.data.push_back(cn(0)); ptsMsg.data.push_back(cn(1)); ptsMsg.data.push_back(cn(2));


			////// Finding Navigable Ramp
			Eigen::Vector3f rampSlope, rampRun, mag;
			float angleSlope;
			mag << pow((cn(0)-cf(0)),2), pow((cn(1)-cf(1)),2), pow((cn(2)-cf(2)),2);
			rampSlope = (cf-cn)/ sqrt(mag(0)+mag(1)+mag(2));
			rampRun << cn(0), cn(1), cn(2)+1;
			rampRun = rampRun/ sqrt(rampRun(0)+rampRun(1)+rampRun(2));
			angleSlope = acos(rampSlope.dot(rampRun));  		//[rad]
			std::cout << "angleSlope (deg.):" << angleSlope*180/PI << std::endl;

			// visualize the vector rampRun and rampSlope
			normalsMsg.data.push_back(cn(0)), normalsMsg.data.push_back(cn(1)), normalsMsg.data.push_back(cn(2));
			normalsMsg.data.push_back(cn(0)+rampSlope(0)), normalsMsg.data.push_back(cn(1)+rampSlope(1)), normalsMsg.data.push_back(cn(2)+rampSlope(2));
			normalsMsg.data.push_back(cn(0)), normalsMsg.data.push_back(cn(1)), normalsMsg.data.push_back(cn(2));
			normalsMsg.data.push_back(cn(0)+rampRun(0)), normalsMsg.data.push_back(cn(1)+rampRun(1)), normalsMsg.data.push_back(cn(2)+rampRun(2));

			// further checking of the ramp candidate
			




		}

		std::cout <<std::endl;
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
	pub_line.publish(lineMsg);

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
            << "\t -k1, --meanK1 \t\t mean K for the first round outliers removal (Default is 50)"
            << "\t -k2, --meanK2 \t\t mean K for the second round outliers removal (Default is 10)"<<std::endl;
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
		else if((arg == "-k1") || (arg == "-meanK1"))
		{
			meanK1 = std::stof(argv[i+1]);
		}
		else if((arg == "-k2") || (arg == "-meanK2"))
		{
			meanK2 = std::stof(argv[i+1]);
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
	pub_line = nh.advertise<std_msgs::Float32MultiArray>("lineInfo", 1);


    ros::spin();

    return 0;
}