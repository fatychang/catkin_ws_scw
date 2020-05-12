#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float32MultiArray.h"


ros::Publisher marker_pub, marker_pub2;
std::vector<float> points, points2;
float scale = 0.03f;

/** @brief This callback updates the point position received from the topic.
 *  
 */
void clbk(const std_msgs::Float32MultiArray::ConstPtr &point_msg)
{
    // clear the vector
    points.clear();

    int numOfMsgs = point_msg ->data.size();
    ROS_INFO("numOfPoints in point1: %d", numOfMsgs/3);
    for (size_t i=0; i< numOfMsgs; ++i)
    {
        points.push_back(point_msg->data[i]);
    }
    

    visualization_msgs::Marker marker_pts;

    marker_pts.header.frame_id = "/camera_link";
    marker_pts.action = visualization_msgs::Marker::ADD;
    marker_pts.pose.orientation.w =  1.0;
    marker_pts.id = 0;
    marker_pts.type = visualization_msgs::Marker::POINTS;
    marker_pts.scale.x = 0.03;
    marker_pts.scale.y = 0.03;
    marker_pts.color.a = 1.0;
    marker_pts.color.r = 1.0;

    

    // add the vertices
    for(int i=0; i< numOfMsgs; i+=3)
    {
        geometry_msgs::Point p;
        p.x = points[i];
        p.y = points[i+1];
        p.z = points[i+2];
        marker_pts.points.push_back(p);
        // std::cout << "[DEBUG] point:" << p.x << " " << p.y << " " << p.z << std::endl;
   }

    marker_pub.publish(marker_pts);

}

void clbk2(const std_msgs::Float32MultiArray::ConstPtr &point_msg)
{
    // clear the vector
    points2.clear();

    int numOfMsgs = point_msg ->data.size();
    // ROS_INFO("numOfPoints in point2: %d", numOfMsgs/3);
    for (size_t i=0; i< numOfMsgs; ++i)
    {
        points2.push_back(point_msg->data[i]);
    }
    

    visualization_msgs::Marker marker_pts;
    marker_pts.header.frame_id = "/camera_link";
    marker_pts.action = visualization_msgs::Marker::ADD;
    marker_pts.pose.orientation.w = 1.0;
    marker_pts.id = 0;
    marker_pts.type = visualization_msgs::Marker::POINTS;
    marker_pts.scale.x = scale;
    marker_pts.scale.y = scale;
    marker_pts.color.a = 1.0;
    marker_pts.color.b = 1.0;

    // add the vertices
    for(int i=0; i< numOfMsgs; i+=3)
    {
        geometry_msgs::Point p;
        p.x = points2[i];
        p.y = points2[i+1];
        p.z = points2[i+2];
        marker_pts.points.push_back(p);
        // std::cout << "[DEBUG] point:" << p.x << " " << p.y << " " << p.z << std::endl;
    }

    marker_pub2.publish(marker_pts);
}


/** @brief Shows all the parse message usage.
 * 
 */
static void showUsage(std::string name)
{
	std::cerr << "Usage: " << name << "option(s) SOURCES"
			<< "Options:\n"
			<< "\t -h, --help \t\t Show this help message\n"
			<< "\t -s, --scale \t\t scale of the points (Default is 0.05)\n" << std::endl;
}


/** @brief The points visualizer node is responsible for visualize the markers (points) in rviz.
 *  The point visualizer node subscribes the pointInfo from other nodes and visualize them in the rviz.
 *  The scale (size) of the points is adjustable using comments -s
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
		else if((arg == "-s") || (arg == "-scale"))
		{
			scale = std::stof(argv[i+1]);
		}
	}

    //ROS and node handle initialize 
    ros::init(argc, argv, "points_visualizer_node");
    ros::NodeHandle nh;

    ROS_INFO("[DEBUG] points_visualizer_node is running...");

    // Declare the publisher
    marker_pub = nh.advertise<visualization_msgs::Marker> ("points_visualizer1", 10);
    marker_pub2 = nh.advertise<visualization_msgs::Marker> ("points_visualizer2", 10);

    // restrict the frequency to allow the user to rotate the view in rviz.
    ros::Rate(1);

    // Subscribe the pose of the points
    ros::Subscriber sub = nh.subscribe("pointInfo", 1, clbk);
    ros::Subscriber sub2 = nh.subscribe("pointInfo2", 1, clbk2);

    ros::spin();

    return 0;
}