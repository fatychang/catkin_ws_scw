#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float32MultiArray.h"

ros::Publisher marker_pub;
std::vector<float> points;

float x_scale = 0.005;
float y_scale = 0.005;
float z_scale = 0.1;

void clbk(const std_msgs::Float32MultiArray::ConstPtr &arrow_msg)
{
    // clear the vector
    points.clear();

    int numOfMsgs = arrow_msg ->data.size();
    ROS_INFO("numOfPoints: %d", numOfMsgs/6);
    for (size_t i=0; i< numOfMsgs; ++i)
    {
        points.push_back(arrow_msg->data[i]);
    }

    visualization_msgs::Marker marker_arrow;
    marker_arrow.header.frame_id = "/camera_link";
    marker_arrow.action = visualization_msgs::Marker::ADD;
    marker_arrow.id = 1;
    marker_arrow.type = visualization_msgs::Marker::LINE_STRIP;
    marker_arrow.scale.x = x_scale;
    marker_arrow.scale.y = y_scale;
    marker_arrow.scale.z = z_scale;
    // marker_arrow.pose.orientation.w =  1.0;    
    marker_arrow.color.a = 1.0;
    marker_arrow.color.g = 1.0;

    // geometry_msgs::Point p;
    // p.x = 0.0, p.y = 0.0, p.z = 0.0;
    // marker_arrow.points.push_back(p);
    // p.x = 1.0, p.y = 1.0, p.z = 1.0;
    // marker_arrow.points.push_back(p);



    //add the vertices
    for(int i=0; i< numOfMsgs; i+=6)
    {
        geometry_msgs::Point p;
        p.x = points[i];
        p.y = points[i+1];
        p.z = points[i+2];
        marker_arrow.points.push_back(p);
        p.x = points[i+3];
        p.y = points[i+4];
        p.z = points[i+5];
        marker_arrow.points.push_back(p);

        // std::cout << "[DEBUG] point:" << p.x << " " << p.y << " " << p.z << std::endl;
   }

    marker_pub.publish(marker_arrow);
}

/** @brief The nomal visualizer node is responsible for visualize the normal of a surface with arrow
 *  The nomal visualizer node subscribes the arrowInfo from other nodes and visualize them in the rviz.
 */
int main(int argc, char **argv)
{   
    // Parse the commend line
	for (int i=0; i<argc; ++i)
	{
		std::string arg = argv[i];

		if((arg == "-x") || (arg == "-leaf"))
		{
			x_scale = std::stof(argv[i+1]);
		}
		else if((arg == "-y") || (arg == "-angle"))
		{
			y_scale = std::stof(argv[i+1]);
		}
        		else if((arg == "-z") || (arg == "-angle"))
		{
			z_scale = std::stof(argv[i+1]);
		}
        
	}

    //ROS and node handle initialize 
    ros::init(argc, argv, "normal_visualization_node");
    ros::NodeHandle nh;

    ROS_INFO("[DEBUG] normal_visualization_node is running...");

    // Declare the publisher
    marker_pub = nh.advertise<visualization_msgs::Marker> ("normal_visualizer", 10);

    // restrict the frequency to allow the user to rotate the view in rviz.
    ros::Rate(1);

    // Subscribe the pose of the points
    ros::Subscriber sub = nh.subscribe("normalsInfo", 1, clbk);

    ros::spin();

    return 0;
}