#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float32MultiArray.h"

ros::Publisher marker_pub;
std::vector<float> points;

void clbk(const std_msgs::Float32MultiArray::ConstPtr &lines_msg)
{
    // clear the vector
    points.clear();

    int numOfMsgs = lines_msg ->data.size();
    // ROS_INFO("numOfPoints: %d", numOfMsgs/3);
    for (size_t i=0; i< numOfMsgs; ++i)
    {
        points.push_back(lines_msg->data[i]);
    }

    visualization_msgs::Marker marker_lineList;
    marker_lineList.header.frame_id = "/camera_link";
    marker_lineList.action = visualization_msgs::Marker::ADD;
    marker_lineList.id = 1;
    marker_lineList.type = visualization_msgs::Marker::LINE_LIST;
    marker_lineList.scale.x = 0.03;
    marker_lineList.pose.orientation.w =  1.0;    
    marker_lineList.color.a = 1.0;
    marker_lineList.color.g = 1.0;

    // add the vertices
    for(int i=0; i< numOfMsgs; i+=3)
    {
        geometry_msgs::Point p;
        if (i < numOfMsgs - 3)
        {  
            p.x = points[i];
            p.y = points[i+1];
            p.z = points[i+2];
            marker_lineList.points.push_back(p);
            p.x = points[i+3];
            p.y = points[i+4];
            p.z = points[i+5];
            marker_lineList.points.push_back(p);
        }
        else
        {
            p.x = points[i];
            p.y = points[i+1];
            p.z = points[i+2];
            marker_lineList.points.push_back(p);
            p.x = points[0];
            p.y = points[1];
            p.z = points[2];
            marker_lineList.points.push_back(p);

        }
        // std::cout << "[DEBUG] point:" << p.x << " " << p.y << " " << p.z << std::endl;
   }

    marker_pub.publish(marker_lineList);
}

/** @brief The lines visualizer node is responsible for visualize the markers (lines) in rviz.
 *  The lines visualizer node subscribes the pointInfo from other nodes and visualize them in the rviz.
 */
int main(int argc, char **argv)
{   

    //ROS and node handle initialize 
    ros::init(argc, argv, "lines_visualizer_node");
    ros::NodeHandle nh;

    ROS_INFO("[DEBUG] lines_visualizer_node is running...");

    // Declare the publisher
    marker_pub = nh.advertise<visualization_msgs::Marker> ("lines_visualizer1", 10);

    // restrict the frequency to allow the user to rotate the view in rviz.
    ros::Rate(1);

    // Subscribe the pose of the points
    ros::Subscriber sub = nh.subscribe("lineInfo", 1, clbk);

    ros::spin();

    return 0;
}