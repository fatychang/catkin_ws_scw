#include "ros/ros.h"
#include "visualization_msgs/Marker.h"
#include "std_msgs/Float32MultiArray.h"


ros::Publisher marker_pub1, marker_pub2;
std::vector<float> points;
/** @brief This callback updates the point position received from the topic.
 * 
 */
void clbk(const std_msgs::Float32MultiArray::ConstPtr &point_msg)
{
    // clear the vector
    points.clear();

    int numOfMsgs = point_msg ->data.size();
    //ROS_INFO("numOfPoints: %d", numOfMsgs/3);
    for (size_t i=0; i< numOfMsgs; ++i)
    {
        points.push_back(point_msg->data[i]);
    }
    // std::cout << "points:(" << points[0] << "," << points[1] << "," << points[1] << std::endl;

    visualization_msgs::Marker marker_pts1, marker_pts2;
    marker_pts1.header.frame_id = marker_pts2.header.frame_id = "/camera_link";
    marker_pts1.ns = marker_pts2.ns = "points";
    marker_pts1.action = marker_pts2.action = visualization_msgs::Marker::ADD;
    marker_pts1.pose.orientation.w = marker_pts2.pose.orientation.w = 1.0;
    marker_pts1.id = 0;
    marker_pts2.id = 1;
    marker_pts1.type = marker_pts2.type = visualization_msgs::Marker::SPHERE;

    marker_pts1.scale.x = marker_pts2.scale.x = 0.1;
    marker_pts1.scale.y = marker_pts2.scale.y = 0.1;
    marker_pts1.color.a = marker_pts2.color.a =1.0;

    marker_pts1.color.r = 1.0f;
    marker_pts2.color.b = 1.0f;


    marker_pts1.pose.position.x = points[0], marker_pts1.pose.position.y = points[1], marker_pts1.pose.position.z = points[2];
    marker_pts2.pose.position.x = points[3], marker_pts2.pose.position.y = points[4], marker_pts2.pose.position.z = points[5];
    //marker_pts2.pose.position.x = (points[0] + points[3])/2, marker_pts2.pose.position.y = (points[1]+ points[4])/2, marker_pts2.pose.position.z = (points[2]+points[5])/2;

    marker_pub1.publish(marker_pts1);
    marker_pub2.publish(marker_pts2);

}


/** @brief The points visualizer node is responsible for visualize the markers in rviz.
 * 
 */
int main(int argc, char **argv)
{   
    //ROS and node handle initialize 
    ros::init(argc, argv, "points_visualizer_node");
    ros::NodeHandle nh;
    marker_pub1 = nh.advertise<visualization_msgs::Marker> ("points_visualizer1", 10);
    marker_pub2 = nh.advertise<visualization_msgs::Marker> ("points_visualizer2", 10);
    ros::Rate(30);

    // Subscribe the pose of the points
    ros::Subscriber sub = nh.subscribe("pointInfo", 1, clbk);

    ros::spin();

    return 0;
}