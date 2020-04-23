# Git repo for ROS shared control wheelchair
## Doorway Detection
Doorway detection is one the commonly seen tasks required for a shared control wheelchair to assist the user in manuvering around in the indoor enviornment. The project is implemented in the ROS environment with the simulated environment by Gazebo.The doorway detection is achieved with the depth information to minimize the effect of the light in different scenario.

The method is inspired from the Automated Doorway Detection for Assistive Shared-Control Wheelchair by Mattew Derry and Brenna Argall with some modification to fit the ROS and Gazebo simulation. Further improvements are required to implement to the real-life environment.

### my_image_processor
The **my_image_processor** contains the necessary nodes to achieve the doorway detection task. **noise_node** is responsible to add the Gaussian noise to the simulated depth information (point cloud data) to create a more realistic sensory input for the image processing later.
**Doorway_detection_node** runs the algorithm to detect the potential doorway in the environment and publish the resulting pointcloud and the location of the door (center point of the doorway) to the ROS master.
**points_visualizer_node** visualizes the points as requested to the RVIZ. One thing worth mentioning is that the frame for all the aboved images is defined as **camera_link**. 
