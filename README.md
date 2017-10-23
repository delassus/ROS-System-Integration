# Overview
This is the repo for the final project of the Udacity Self-Driving Car Nanodegree: Programming a Real Self-Driving Car.
It is a team project. contributed to this project:

### TEAM :
| Name | Email | Slack |
| --- | --- | --- |
| Shyam Jagannathan | shyam.jagannathan@gmail.com | @shyam.jagannathan |
| Hubert de Lassus | hubert.delassus@gmail.com | @delassus |
| Haitham Khedr | haithamkhedr@gmail.com | @haithamkhedr |
| Ganesh Setty R | setty.ganesh@gmail.com | @gsetty |
| Nishanth | nishanth.jois@gmail.com | @nishanthjois |



# System Architecture Diagram

![final-project-ros-graph-v2](https://user-images.githubusercontent.com/6969317/31867863-46a81e98-b74b-11e7-857f-e55a429671f8.png)

For this project, I will be be writing ROS nodes to implement core functionality of the autonomous vehicle system, including traffic light detection, control, and waypoint following. I will test the code using a simulator, and finally, the project will be run on "Carla" the Udacity Lincoln self driving car.

The following is a system architecture diagram showing the ROS nodes and topics used in the project. The ROS nodes and topics shown in the diagram are described briefly in the Code Structure section below.

# Code Structure
Below is a brief overview of the repo structure, along with descriptions of the ROS nodes. The code for the project is contained entirely within the (path_to_project_repo)/ros/src/ directory. Within this directory, you will find the following ROS packages:

# Traffic Light Detector
(path_to_project_repo)/ros/src/tl_detector/
This package contains the traffic light detection node: tl_detector.py. This node takes in data from the /image_color, /current_pose, and /base_waypoints topics and publishes the locations to stop for red traffic lights to the /traffic_waypoint topic.
![tl-detector-ros-graph](https://user-images.githubusercontent.com/6969317/31867909-0d975d5c-b74c-11e7-9944-0371ba77aa60.png)


# Current Pose Topic
The /current_pose topic provides the vehicle's current position, and /base_waypoints provides a complete list of waypoints the car will be following.

This project implements both a traffic light detection node and a traffic light classification node. Traffic light detection takes place within tl_detector.py, whereas traffic light classification takes place within ../tl_detector/light_classification_model/tl_classfier.py.

# Way Point Updater
(path_to_project_repo)/ros/src/waypoint_updater/
This package contains the waypoint updater node: waypoint_updater.py. The purpose of this node is to update the target velocity property of each waypoint based on traffic light and obstacle detection data. This node subscribes to the /base_waypoints, /current_pose, /obstacle_waypoint, and /traffic_waypoint topics, and publish a list of waypoints ahead of the car with target velocities to the /final_waypoints topic.
![waypoint-updater-ros-graph](https://user-images.githubusercontent.com/6969317/31867951-bc7811d6-b74c-11e7-9eed-bdbc4a7528a1.png)

(path_to_project_repo)/ros/src/twist_controller/

# Drive By Wire
Carla the Udacity Lincoln self driving car is equipped with a drive-by-wire (dbw) system, meaning the throttle, brake, and steering have electronic control. This package contains the files that are responsible for control of the vehicle: the node dbw_node.py and the file twist_controller.py, along with a pid and lowpass filter. The dbw_node subscribes to the /current_velocity topic along with the /twist_cmd topic to receive target linear and angular velocities. Additionally, this node will subscribe to /vehicle/dbw_enabled, which indicates if the car is under dbw or driver control. This node will publish throttle, brake, and steering commands to the /vehicle/throttle_cmd, /vehicle/brake_cmd, and /vehicle/steering_cmd topics.
![dbw-node-ros-graph](https://user-images.githubusercontent.com/6969317/31867963-01ad7a52-b74d-11e7-997e-a131dccb89c6.png)

# Additional packages
In addition to these packages you will find the following. The styx and styx_msgs packages are used to provide a link between the simulator and ROS, and to provide custom ROS message types:

(path_to_project_repo)/ros/src/styx/
A package that contains a server for communicating with the simulator, and a bridge to translate and publish simulator messages to ROS topics.
(path_to_project_repo)/ros/src/styx_msgs/
A package which includes definitions of the custom ROS message types used in the project.
(path_to_project_repo)/ros/src/waypoint_loader/
A package which loads the static waypoint data and publishes to /base_waypoints.
(path_to_project_repo)/ros/src/waypoint_follower/
A package containing code from Autoware which subscribes to /final_waypoints and publishes target vehicle linear and angular velocities in the form of twist commands to the /twist_cmd topic.


# Implementation
**Waypoint Updater Node** (partial): Completes a waypoint updater which subscribes to /base_waypoints and /current_pose and publishes to /final_waypoints.

**DBW Node**: Once the waypoint updater is publishing /final_waypoints, the waypoint_follower node will start publishing messages to the/twist_cmd topic. At this point, we have everything needed to build the dbw_node. After completing this step, the car drives in the simulator, ignoring the traffic lights.

**Traffic Light Detection**: This is split into 2 parts:
Detection: Detect the traffic light and its color from the /image_color. The topic /vehicle/traffic_lights contains the exact location and status of all traffic lights in simulator, so we can test our output.

**Waypoint publishing**: Once we have correctly identified the traffic light and determined its position, we can convert it to a waypoint index and publish it.
Waypoint Updater (Full): we use /traffic_waypoint to change the waypoint target velocities before publishing to /final_waypoints. The car now stops at red traffic lights and moves when they are green.


 
### Installation 

* Be sure that your workstation is running Ubuntu 16.04 Xenial Xerus or Ubuntu 14.04 Trusty Tahir. [Ubuntu downloads can be found here](https://www.ubuntu.com/download/desktop). 
* If using a Virtual Machine to install Ubuntu, use the following configuration as minimum:
  * 2 CPU
  * 2 GB system memory
  * 25 GB of free hard drive space
  
  The Udacity provided virtual machine has ROS and Dataspeed DBW already installed, so you can skip the next two steps if you are using this.

* Follow these instructions to install ROS
  * [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu) if you have Ubuntu 16.04.
  * [ROS Indigo](http://wiki.ros.org/indigo/Installation/Ubuntu) if you have Ubuntu 14.04.
* [Dataspeed DBW](https://bitbucket.org/DataspeedInc/dbw_mkz_ros)
  * Use this option to install the SDK on a workstation that already has ROS installed: [One Line SDK Install (binary)](https://bitbucket.org/DataspeedInc/dbw_mkz_ros/src/81e63fcc335d7b64139d7482017d6a97b405e250/ROS_SETUP.md?fileviewer=file-view-default)
* Download the [Udacity Simulator](https://github.com/udacity/CarND-Capstone/releases/tag/v1.2).

### Usage

1. Clone the project repository
```bash
git clone https://github.com/udacity/CarND-Capstone.git
```

2. Install python dependencies
```bash
cd CarND-Capstone
pip install -r requirements.txt

3. Download and install Tensorflow checkpoint
```bash
download the tensorflow checkpoint and put it in ros/src/tl_detector/ckpt/
https://drive.google.com/file/d/0B2K7eATT8qRAY0g0aWhjdkw0bEU/view
file: tiny-yolov2-udacity.tar.gz (size 222MB)
Move the downloaded tar file to Vidyut-CarND-Capstone-master/ros/src/tl_detector/ckpt/
tar xzf tiny-yolov2-udacity.tar.gz
move the files in tiny-yolo-v2 directory one level up to ckpt directory
cd Vidyut-CarND-Capstone-master/ros/src/tl_detector/ckpt/
\cp -rf tiny-yolo-v2/* .
cd ../../../../

```

3. Compile tl_detector package Cython files
```bash
cd ros
cd src
cd tl_detector
cd cython_utils
python setup.py build_ext --inplace
cd ../../../../
```

4. Make and run styx
```bash
cd ros
catkin_make
source devel/setup.sh
roslaunch launch/styx.launch
```
5. Run the simulator

### Real world testing
1. Download [training bag](https://drive.google.com/file/d/0B2_h37bMVw3iYkdJTlRSUlJIamM/view?usp=sharing) that was recorded on the Udacity self-driving car
2. Unzip the file
```bash
unzip traffic_light_bag_files.zip
```
3. Play the bag file
```bash
rosbag play -l traffic_light_bag_files/loop_with_traffic_light.bag
```
4. Launch your project in site mode
```bash
cd CarND-Capstone/ros
roslaunch launch/site.launch
```

