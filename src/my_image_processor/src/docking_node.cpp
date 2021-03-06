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
#include "pcl/filters/project_inliers.h"
#include "pcl/surface/convex_hull.h"


//#include "pcl/filters/passthrough.h"

// others
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

// Declare publisher
ros::Publisher pub_obj, pub_filtered, pub_pts, pub_pts2, pub_line, pub_normals;

// Parameters for image processing
float leaf_size = 0.1;
int meanK = 50;
float epsAngle = 30.0f;
int p_param = 0;
int k_param = 1;

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

/** @brief calculate the minimum or maxmimun of the given point cloud and return the index
 * 
 * @param cloud_points the points required to be sorted
 * @param mode find minimum =0; find maximum =1
 * @param axis the axis to be sorted
 * 
 * @return the index of the min/max point is returned
 */
int calculateMinMax(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int mode, char axis)
{
	float required_value=-1;
	int returned_index=0;
	float point=-1;


	if(axis == 'x')
		required_value =cloud->points[0].x;
	else if (axis == 'y')
		required_value = cloud->points[0].y;
	else if (axis == 'z')
		required_value = cloud->points[0].z;
	else
	{
		ROS_ERROR("Please provide a valid axis. (x, y, or z) with small letters");
		return -1;
	}

	if(mode > 2)
	{
		ROS_ERROR("Please provide a valid mode. 0=min, 1=max");
		return -1;
	}
	
	for(int i=1; i<cloud->points.size(); ++i)
	{
		if(axis == 'x')
			point =cloud->points[i].x;
		else if (axis == 'y')
			point = cloud->points[i].y;
		else if (axis == 'z')
			point = cloud->points[i].z;

		// compare
		if(mode == 0)	//min
		{
			if(point < required_value)
			{
				required_value = point;
				returned_index = i;
			}
		}
		else			//max
		{
			if(point > required_value)
			{
				required_value = point;
				returned_index = i;
			}
		}		
	}

	return returned_index;
}

/** Compare two values and return both with the smaller in the first. 
 * 
 * @param v1 one of the value required to be compared
 * @param v2 another value required to be compared
 * @return return both value and the smaller value in the first index
 */
Eigen::Vector2f compareValues(float v1, float v2)
{
	Eigen::Vector2f result;
	if(v1 < v2)
		result << v1, v2;
	else
		result << v2, v1;
	return result;
}

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


/** @brief Calculate the intersection of two lines.
 * 
 *  @param pt1 starting point of line 1
 *  @param pt2 end point of line 1
 *  @param pt3 starting point of line 2
 *  @param pt4 end point of line 2
 *  @return intersection of the lines
 */
Eigen::Vector2f calculateIntersection(Eigen::Vector2f pt1, Eigen::Vector2f pt2, Eigen::Vector2f pt3, Eigen::Vector2f pt4)
{
	Eigen::Vector2f intersect;

	// Line 1 represented as a1x + b1y = c1
	float a1 = pt2(1) - pt1(1);
	float b1 = pt2(0) - pt1(0);
	float c1 = a1 * pt1(0) + b1 * pt2(1); 

	// Line 2 represented as a2x + b2y = c2
	float a2 = pt4(1) - pt3(1);
	float b2 = pt4(0) - pt3(0);
	float c2 = a2 * pt3(0) + b2 * pt4(1); 

	float determinant = a1*b2 - a2*b1;

	if (determinant == 0)
		intersect << NAN, NAN;
	else
		intersect << (b2*c1 - b1*c2)/determinant, (a1*c2 - a2*c1)/determinant;

	return intersect;
}

/** @brief The callback process the raw depth information to find the suitable docking location.
 * 
 */
void dockingCallback(const sensor_msgs::PointCloud2::ConstPtr &cloud_msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::PointCloud<pcl::PointXYZ>::Ptr origin_cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        filtered_cloud (new pcl::PointCloud<pcl::PointXYZ>),
                                        table_cloud (new pcl::PointCloud<pcl::PointXYZ>);

	
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
	// std::cout << "[DEBUG]: coefficient: (" << coefficient->values[0] << " " << coefficient->values[1] << " " << coefficient->values[2] << std::endl;

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
		extract.filter(*filtered_cloud);

		// remove the outliers with statistical outlier removal filter
        pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        sor.setInputCloud(filtered_cloud);
        sor.setMeanK(meanK);
        sor.setStddevMulThresh(1.0);
        sor.filter(*filtered_cloud);

		// Check whether the candidate satisfies the following thress constrains.
		// 1. plane is perpendicular to the ground
		// 2. the number of points exceed a certain threshold
		// 3. the height of the table exceeds the threshold
		float perpendicular = 0.98;
		int minimul_num_of_points = 100;
		float camera_height = 1.3;
		float minimul_table_height = 0.7112;  //around 28'

		// calculate the average height (y) of all the points in the plane
		float avgTableHeight = 0.0;
		for (int i=0; i< filtered_cloud->points.size(); ++i)
		{
			avgTableHeight+=filtered_cloud->points[i].y;
		}
		avgTableHeight /= filtered_cloud->points.size();

		if((fabs(coefficient->values[1]) > perpendicular) && (filtered_cloud->points.size() > minimul_num_of_points) && (avgTableHeight < camera_height - minimul_table_height))
		{
			std::cout << "[DEBUG] Table candidate found." << std::endl;

			// project the model inliers to the plane
			pcl::ProjectInliers<pcl::PointXYZ> proj;
			proj.setModelType(pcl::SACMODEL_PLANE);
			proj.setIndices(inliers);
			proj.setModelCoefficients(coefficient);
			proj.setInputCloud(filtered_cloud);
			proj.filter(*table_cloud);
			std::cout << "plane coefficient: (" <<coefficient->values[0] <<"," << coefficient->values[1] << "," << coefficient->values[2] <<")" << std::endl;
			float planeY = table_cloud->points[5].y;

			//create a Convex Hull representation
			pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud (new pcl::PointCloud<pcl::PointXYZ>);
			pcl::ConvexHull<pcl::PointXYZ> hull;
			hull.setInputCloud(table_cloud);
			hull.reconstruct(*hull_cloud);

			// Detect the corner of rectangle table by the dot product of the two consecutive convex hull
			if(hull_cloud->points.size() > 0)
			{
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


				std::vector<float> tableDim;
				tableDim.push_back(findDistance(corners[1], corners[0]));
				tableDim.push_back(findDistance(corners[2], corners[1]));
				tableDim.push_back(findDistance(corners[3], corners[2]));
				tableDim.push_back(findDistance(corners[0], corners[3]));
				std::cout << "[DEBUG] The length of the rectangle ramp: (" << tableDim[0] << ", " << tableDim[1] << ", "
						<< tableDim[2] << ", " << tableDim[3] << std::endl;;

				//find the center of the table of docking position [simplified.]
				// should consider the obstable under the table in the future
				// identify the two corner points that are closer to the camera (the side where we want to dock)
				// NOTED, the y value is upside down in the rviz therefore minimul y means highest point in the view
				std::vector<int> furtherId, closerId;		// stores the id of the corner points which have the smaller y (at the ramp)
				std::vector<pcl::PointXYZ> tmp2(corners);
				pcl::PointXYZ discardPt; discardPt.x=1000, discardPt.y=1000, discardPt.z=1000;
				for (int i=0; i<2; ++i)
				{
					closerId.push_back(findMinIndex(tmp2, 2)); // the second argument is to indicate the axis. x=0, y=1, z=2
					tmp2[closerId[i]] = discardPt;
					//std::cout << "index: " << landingId[i] << std::endl;
				}
				
				// calculate the centers cc, cf of the side which is closer or rfurther away from the camera (in terms of z value)
				Eigen::Vector3f cc, cf;
				cf << (corners[closerId[0]].x + corners[closerId[1]].x)/2, 
						(corners[closerId[0]].y + corners[closerId[1]].y)/2, 
						(corners[closerId[0]].z + corners[closerId[1]].z)/2;
				ptsMsg.data.push_back(cf(0)); ptsMsg.data.push_back(cf(1)); ptsMsg.data.push_back(cf(2));

			}
			else
			{
				ROS_INFO("Unable to detect the contour.");
			}

		





			// // [Paper method] Generate 2D grid for Hough Transfer -> Not used.
			// if(hull_cloud->points.size() > 0)
			// {
			// 	// visualize the convex hull (contour and points)
			// 	for(int i=0; i<hull_cloud->points.size(); ++i)
			// 	// for(int i=k_param; i<p_param+1; ++i)
			// 	{
			// 		ptsMsg.data.push_back(hull_cloud->points[i].x); ptsMsg.data.push_back(hull_cloud->points[i].y); ptsMsg.data.push_back(hull_cloud->points[i].z); 						
			// 		lineMsg.data.push_back(hull_cloud->points[i].x); lineMsg.data.push_back(hull_cloud->points[i].y); lineMsg.data.push_back(hull_cloud->points[i].z); 	
			// 		// std::cout << "hull " << i << " (" << hull_cloud->points[i].x << "," << hull_cloud->points[i].z << std::endl;
			// 	}
			// 	//std::cout << "[DEBUG]: the number of points in the hull: " << hull_cloud->points.size() << std::endl;\

			// 	//find the min & max in the x and z direction 
			// 	int min_x_id = calculateMinMax(table_cloud, 0, 'x');
			// 	int max_x_id = calculateMinMax(table_cloud, 1, 'x');
			// 	int min_z_id = calculateMinMax(table_cloud, 0, 'z');
			// 	int max_z_id = calculateMinMax(table_cloud, 1, 'z');
			// 	float min_x = table_cloud->points[min_x_id].x;
			// 	float max_x = table_cloud->points[max_x_id].x;
			// 	float min_z = table_cloud->points[min_z_id].z;
			// 	float max_z = table_cloud->points[max_z_id].z;
			// 	//visualize the min/max boundaries
			// 	// ptsMsg2.data.push_back(table_cloud->points[min_x_id].x); ptsMsg2.data.push_back(table_cloud->points[min_x_id].y); ptsMsg2.data.push_back(table_cloud->points[min_x_id].z); 						
			// 	// ptsMsg2.data.push_back(table_cloud->points[max_x_id].x); ptsMsg2.data.push_back(table_cloud->points[max_x_id].y); ptsMsg2.data.push_back(table_cloud->points[max_x_id].z); 						
			// 	// ptsMsg2.data.push_back(table_cloud->points[min_z_id].x); ptsMsg2.data.push_back(table_cloud->points[min_z_id].y); ptsMsg2.data.push_back(table_cloud->points[min_z_id].z); 						
			// 	// ptsMsg2.data.push_back(table_cloud->points[max_z_id].x); ptsMsg2.data.push_back(table_cloud->points[max_z_id].y); ptsMsg2.data.push_back(table_cloud->points[max_z_id].z); 
				
			// 	float step_size = 0.05;
			// 	int grid_x = (max_x - min_x) / step_size, grid_z = (max_z - min_z) / step_size;	
			// 	// std::cout << "[DEBUG] grid_x:" << grid_x << "	grid_z:" << grid_z << std::endl;
			// 	Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> grid(grid_z+2, grid_x+2);
			// 	grid.setZero();


			// 	int edge_x = grid_x/5, edge_z = grid_z/5;
			// 	int tmpCount = 0;
			// 	for(int i=0; i < grid_x + 2; ++i)
			// 	{					
			// 		for(int j=0; j < grid_z + 2; ++j)
			// 		{	
			// 			float tmp_x = min_x + i * step_size, tmp_z = min_z + j * step_size;
			// 			float tmp_x2 = min_x + (i+1) * step_size, tmp_z2 = min_z + (j+1) * step_size;
			// 			// std::cout << "tmp: (" << tmp_x << ", " << tmp_z << std::endl;

			// 			for (int k=0; k < hull_cloud->points.size(); ++k)	
			// 			// for (int k=k_param; k < p_param; ++k)	
			// 			{
			// 				// Mark the convex hull points on the grid
			// 				if((hull_cloud->points[k].z >= tmp_z) && (hull_cloud->points[k].z < tmp_z2))
			// 				{
			// 					if((hull_cloud->points[k].x >= tmp_x) && (hull_cloud->points[k].x < tmp_x2))
			// 					{
			// 						grid(j, i) = 1;
			// 						// visualize only the contour point
			// 						ptsMsg2.data.push_back(min_x + i*step_size); ptsMsg2.data.push_back(planeY); ptsMsg2.data.push_back(min_z + j * step_size);
			// 						break;
			// 					}
			// 				}

			// 				// Mark the contour on the grid
			// 				// skip points that are not even close to the boundary
			// 				Eigen::Vector2f boundX, boundZ;
			// 				// int cnt = -1;
			// 				if(k!=hull_cloud->points.size()-1)
			// 				{
			// 					boundX = compareValues(hull_cloud->points[k].x, hull_cloud->points[k+1].x);
			// 					boundZ = compareValues(hull_cloud->points[k].z, hull_cloud->points[k+1].z);
			// 				}
			// 				else
			// 				{
			// 					boundX = compareValues(hull_cloud->points[k].x, hull_cloud->points[0].x);
			// 					boundZ = compareValues(hull_cloud->points[k].z, hull_cloud->points[0].z);
			// 				}
			// 				// std::cout << "boundX:(" << boundX(0) << "," << boundX(1) << ")" << std::endl; 
			// 				// std::cout << "boundz:(" << boundZ(0) << "," << boundZ(1) << ")" << std::endl; 
			// 				// if(tmp_x < boundX(0)-step_size || tmp_x > boundX(1)+step_size)
			// 				// 	continue;
			// 				// if(tmp_z < boundZ(0)-step_size || tmp_z > boundZ(1)+step_size)							
			// 				// 	continue;	
			// 				// if(tmp_x < boundX(0)|| tmp_x > boundX(1))
			// 				// 	continue;
			// 				// if(tmp_z < boundZ(0)|| tmp_z > boundZ(1))							
			// 				// 	continue;	
			// 				// cnt++;
			// 				//std::cout << k << ":" << cnt << std::endl;
			// 				if(tmp_x < boundX(0)-step_size*0.5 || tmp_x > boundX(1)+step_size*0.5)
			// 					continue;
			// 				if(tmp_z < boundZ(0)-step_size*0.5 || tmp_z > boundZ(1)+step_size*0.5)							
			// 					continue;															

							


			// 				// // calculate the slope
			// 				float slope;
			// 				if(k < hull_cloud->points.size()-1)
			// 				{
			// 					slope = (hull_cloud->points[k+1].z - hull_cloud->points[k].z)/(hull_cloud->points[k+1].x - hull_cloud->points[k].x);
			// 					// visualize the convux hull
			// 					// ptsMsg2.data.push_back(hull_cloud->points[k].x), ptsMsg2.data.push_back(hull_cloud->points[k].y), ptsMsg2.data.push_back(hull_cloud->points[k].z);
			// 					// ptsMsg2.data.push_back(hull_cloud->points[k+1].x), ptsMsg2.data.push_back(hull_cloud->points[k+1].y), ptsMsg2.data.push_back(hull_cloud->points[k+1].z);
			// 					// std::cout << "hull point " << k << " (" << hull_cloud->points[k].x << "," << hull_cloud->points[k].z << std::endl;
			// 					// std::cout << "hull point " << k+1 << " (" << hull_cloud->points[k+1].x << "," << hull_cloud->points[k+1].z << std::endl;
			// 					// std::cout << "[DEBUG]: slope for point " << k << " : " << slope << std::endl;
			// 				}
			// 				else
			// 				{
			// 					slope = (hull_cloud->points[0].z - hull_cloud->points[k].z)/(hull_cloud->points[0].x - hull_cloud->points[k].x);
			// 					//visualize the convux hull
			// 					// ptsMsg2.data.push_back(hull_cloud->points[k].x), ptsMsg2.data.push_back(hull_cloud->points[k].y), ptsMsg2.data.push_back(hull_cloud->points[k].z);
			// 					// ptsMsg2.data.push_back(hull_cloud->points[0].x), ptsMsg2.data.push_back(hull_cloud->points[0].y), ptsMsg2.data.push_back(hull_cloud->points[0].z);
			// 					// std::cout << "hull point " << k << " (" << hull_cloud->points[k].x << "," << hull_cloud->points[k].z << std::endl;
			// 					// std::cout << "hull point 0 (" << hull_cloud->points[0].x << "," << hull_cloud->points[0].z << std::endl;
			// 					// std::cout << "[DEBUG]: slope for point " << k << " : " << slope << std::endl;
			// 				}	
							
							
			// 				float tor = 1;
			// 				if(fabs(slope) > 10)
			// 				{
			// 					if(tmp_x - hull_cloud->points[k].x <= step_size * tor && tmp_x - hull_cloud->points[k].x >= -step_size * tor )
			// 					{
			// 						// visualize only the contour points
			// 						if(!grid(j, i))
			// 						{
			// 							grid(j, i) = 1;	
			// 							ptsMsg2.data.push_back(min_x + i*step_size); ptsMsg2.data.push_back(planeY); ptsMsg2.data.push_back(min_z + j * step_size);								
			// 						}
			// 					}
			// 				}
			// 				else
			// 				{
			// 					if(slope * (tmp_x - hull_cloud->points[k].x) - tmp_z + hull_cloud->points[k].z <= step_size * tor && slope * (tmp_x - hull_cloud->points[k].x) - tmp_z + hull_cloud->points[k].z > -step_size * tor)
			// 					{
			// 						// visualize only the contour points
			// 						if(!grid(j, i))
			// 						{
			// 							grid(j, i) = 1;	
			// 							ptsMsg2.data.push_back(min_x + i*step_size); ptsMsg2.data.push_back(planeY); ptsMsg2.data.push_back(min_z + j * step_size);								
			// 						}
			// 					}
			// 				}							
			// 			}							
			// 			// grid visualization
			// 			// ptsMsg.data.push_back(min_x + i*step_size); ptsMsg.data.push_back(table_cloud->points[max_z_id].y); ptsMsg.data.push_back(min_z + j * step_size); 
			// 			// int scale = 100;
			// 			// ptsMsg.data.push_back(min_x + i*coefficient->values[2]*scale); ptsMsg.data.push_back(table_cloud->points[max_z_id].y); ptsMsg.data.push_back(min_z + j * coefficient->values[0]*scale); 
			// 		}
			// 	}	// end of generating the grid

			// 	///////// Detect the shape: retangle or circle 
			// 	// convert Eigen::Matrix to cv type
			// 	cv::Mat_<int> cMat = cv::Mat_<int>::zeros(grid_z+1, grid_x+1);
			// 	cv::eigen2cv(grid, cMat);
			// 	cv::Mat cMat_8U;
			// 	cMat.convertTo(cMat_8U,CV_8UC1);
				
			// 	// Hough transfer to detect lines
			// 	std::vector<cv::Vec4i> lines;
			// 	cv::HoughLinesP(cMat_8U, lines, 1, CV_PI/180, 10, 0, 0);
			// 	std::cout << "[DEBUG]: the number of lines detected:" << lines.size() << std::endl;
			// 	// Visualize the lines and print it
			// 	for (int i=0; i<lines.size(); ++i)
			// 	{
			// 		float x1 = min_x + lines[i][0] * step_size, z1 = min_z + lines[i][1] * step_size;
			// 		float x2 = min_x + lines[i][2] * step_size, z2 = min_z + lines[i][3] * step_size;
			// 		normalsMsg.data.push_back(x1), normalsMsg.data.push_back(table_cloud->points[max_z_id].y), normalsMsg.data.push_back(z1);	// (x1, z1)
			// 		normalsMsg.data.push_back(x2), normalsMsg.data.push_back(table_cloud->points[max_z_id].y), normalsMsg.data.push_back(z2);	// (x2, z2)					
			// 		std::cout << "line " << i << ":(" << lines[i][0] << "," << lines[i][1] << ")	" << "(" << lines[i][2] << "," << lines[i][3] << ")" << std::endl;																													
			// 	}

			// 	// // calculate the intersect
			// 	// std::vector<Eigen::Vector2f> intersects;
			// 	// for (int i=0; i<lines.size(); ++i)
			// 	// {
			// 	// 	Eigen::Vector2f pt1, pt2, pt3, pt4;

			// 	// 	pt1 << min_x + lines[i][0] * step_size, min_z + lines[i][1] * step_size;
			// 	// 	pt2 << min_x + lines[i][2] * step_size, min_z + lines[i][3] * step_size;
			// 	// 	for (int j=i+1; j<lines.size(); ++j)
			// 	// 	{
			// 	// 		pt3 << min_x + lines[j][0] * step_size, min_z + lines[j][1] * step_size;
			// 	// 		pt4 << min_x + lines[j][2] * step_size, min_z + lines[j][3] * step_size;
			// 	// 		intersects.push_back(calculateIntersection(pt1, pt2, pt3, pt4));
			// 	// 	}
			// 	// }
			// 	// // std::cout << "The number of intersects: " << intersects.size() << std::endl;
			// 	// // for (int i=0; i<intersects.size(); ++i)
			// 	// // {
			// 	// // 	std::cout << "Intersected " << i << ":(" << intersects[i][0] << "," << intersects[i][1] << std::endl;
			// 	// // }


			// 	// // Extract the corners from the intersected points
			// 	// std::vector<Eigen::Vector3f> corners;
			// 	// for (int i=0; i< intersects.size(); ++i)
			// 	// {
			// 	// 	if(!isnan(intersects[i](0)) || !isnan(intersects[i](1)))
			// 	// 	{
			// 	// 		// // Visualize valid intersected points
			// 	// 		// ptsMsg.data.push_back(intersects[i](0)), ptsMsg.data.push_back(table_cloud->points[max_z_id].y), ptsMsg.data.push_back(intersects[i](1));									

			// 	// 		if(intersects[i](0) >= max_x-step_size && intersects[i](0) <= max_x+step_size || intersects[i](0) <= min_x + step_size && intersects[i](0) >= min_x - step_size)
			// 	// 		{
			// 	// 			if(intersects[i](1) >= max_z-step_size && intersects[i](1) <= max_z+step_size || intersects[i](1) <= min_z + step_size && intersects[i](1) >= min_z - step_size)
			// 	// 			{
			// 	// 				Eigen::Vector3f corner (intersects[i](0), table_cloud->points[max_z_id].y, intersects[i](1));
			// 	// 				if (corners.size() == 0)
			// 	// 				{
			// 	// 					corners.push_back(corner);
			// 	// 					// corner visualization
			// 	// 					ptsMsg2.data.push_back(intersects[i](0)), ptsMsg2.data.push_back(table_cloud->points[max_z_id].y), ptsMsg2.data.push_back(intersects[i](1));									
			// 	// 					continue;
			// 	// 				}	
			// 	// 				bool isRepeat = false;									
			// 	// 				for (int j=0; j<corners.size(); ++j)
			// 	// 				{
			// 	// 					if(corner(0) >= corners[j](0) - 2*step_size && corner(0) <= corners[j](0) + 2*step_size && corner(2) >= corners[j](2) - 2*step_size && corner(2) <= corners[j](2) + 2*step_size)
			// 	// 					{
			// 	// 						isRepeat = true;
			// 	// 						break;
			// 	// 					}	
			// 	// 				}		
			// 	// 				if(!isRepeat)
			// 	// 				{
			// 	// 					corners.push_back(corner);
			// 	// 					// corner visualization
			// 	// 					ptsMsg2.data.push_back(intersects[i](0)), ptsMsg2.data.push_back(table_cloud->points[max_z_id].y), ptsMsg2.data.push_back(intersects[i](1));																				
			// 	// 				}							
			// 	// 			}
			// 	// 		}
			// 	// 	}
			// 	// }	//end of corner extraction
			// 	// std::cout << "[DEBUG] The number of coners: " << corners.size() << std::endl; 
			// 	// // for(int i=0; i< corners.size();++i)
			// 	// // {
			// 	// // 	std::cout << "[DEBUG] corner " << i << ":(" << corners[i](0) << "," << corners[i](2) << ")\n";
			// 	// // }


			// }
			// else
			// {
			// 	std::cout << "[DEBUG] Unable to detect the contour of the table" << std::endl;				
			// }
		}
		else
		{
			std::cout << "[DEBUG] Unable to find a table fits the constrains. Please check the type of table." << std::endl;
		}
	}	//end of table cloud




    // Convert to ROS message data type and publish
    sensor_msgs::PointCloud2 output, output2;
    pcl::toROSMsg(*filtered_cloud.get(), output);
    pcl::toROSMsg(*table_cloud.get(), output2);
    pub_filtered.publish(output);
    pub_obj.publish(output2);

	// publish std_message::Float32Array containing the points information
	pub_pts.publish(ptsMsg);
	pub_pts2.publish(ptsMsg2);
	pub_line.publish(lineMsg);
	pub_normals.publish(normalsMsg);
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
            << "\t -p, --param \t\t tmp param for tuning params"<<std::endl;
}

/** @brief This node processes the depth input (ploint cloud) and to find the suitable docking location.
 * 
 * This node process the depth input (ploint cloud) from an rgbd camera to locate the possible docking 
 * location (usually in front of the table) and publish the location in terms of the camera frame.
 * 
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
		else if((arg == "-l") || (arg == "-leaf"))
		{
			leaf_size = std::stof(argv[i+1]);
		}
		else if((arg == "-a") || (arg == "-angle"))
		{
			epsAngle = std::stof(argv[i+1]);
		}
        else if((arg == "-p") || (arg == "-param"))
		{
			p_param = std::stof(argv[i+1]);
		}
		else if((arg == "-k") || (arg == "-kparam"))
		{
			k_param = std::stof(argv[i+1]);
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
    // ros::Subscriber sub = nh.subscribe("depth_noise", 1, dockingCallback);  
    ros::Subscriber sub = nh.subscribe("/camera/depth/points", 1, dockingCallback);



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
	pub_pts2 = nh.advertise<std_msgs::Float32MultiArray>("pointInfo2", 1);
	pub_line = nh.advertise<std_msgs::Float32MultiArray>("lineInfo", 1);
	pub_normals = nh.advertise<std_msgs::Float32MultiArray>("normalsInfo", 1);


     ros::spin();

     return 0;
}