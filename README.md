# Fetch_Robot
Autonomously controlling fetch robot through visual servoing method


# Fetch Robot Grasping and Manipulation

This project involves the development of a robotic system using the Fetch robot to grasp and manipulate objects of various shapes and sizes. The project utilizes the ROS Melodic framework on Ubuntu 18.04 and integrates various components such as computer vision, depth sensing, grasp planning, and motion control.

## Table of Contents
- [Installation](#installation)
- [Dependencies](#dependencies)
- [Camera Calibration](#camera-calibration)
- [Usage](#usage)
- [Components](#components)
- [Safety Precautions](#safety-precautions)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Installation
1. Clone the repository:
   ```bash
   git clone https://github.com/ZebraDevs/fetch_ros.git
   cd fetch_ros
   git clone --recursive https://github.com/leggedrobotics/darknet_ros.git

   ```

2. Install required dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   sudo apt-get install ros-melodic-fetch-gazebo-demo
   wget https://pjreddie.com/media/files/yolov3.weights
   ```

3. Build the workspace:
   ```bash
   catkin_make
   ```

4. Source the workspace:
   ```bash
   source devel/setup.bash
   ```

## Dependencies
- ROS Melodic
- Gazebo for simulation
- OpenCV for computer vision tasks
- MoveIt for motion planning
- cv_bridge for converting between ROS and OpenCV Image formats
- darknet_ros for object detection

## Camera Calibration
Follow the [Intel RealSense D435 Calibration Guide](https://github.com/IntelRealSense/librealsense/blob/master/doc/calibration.md) to calibrate the camera. Ensure that the calibration parameters are correctly set in the respective ROS topics or configuration files.

## Usage
1. Launch the Fetch robot in Gazebo:
   ```bash
   roslaunch fetch_gazebo simulation.launch
  ```
2. ```bash
   roslaucnh darknet_ros darknet_ros.launch
   ```

3. Launch the object detection node:
   ```bash
   roslaunch fetch_motion object_detection.launch
   ```

4. Launch the grasp planning node:
   ```bash
   roslaunch fetch_motion grasp_planning.launch
   ```

5. Launch the visual servoing node:
   ```bash
   roslaunch fetch_motion visual_servoing.launch
   ```

6. To start the grasping process, send a command or use a relevant ROS tool.

## Components
- Object Detection: Utilizes YOLO through the darknet_ros package for detecting objects and their bounding boxes.
- Depth Sensing: Uses the RealSense D435 camera to obtain depth images of the scene.
- Grasp Planning: Implements a grasp planning algorithm to determine the best grasp positions on detected objects.
- Visual Servoing: Utilizes visual feedback to align the robot's end effector with the target object for precise grasping.
- Motion Control: Uses MoveIt for planning and executing movements of the robot's arm.




