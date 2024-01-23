# Tello ROS driver 

ROS driver used in the course MTRX5700 - Experimental robotics @ The University of Sydney.
This driver was forked from https://github.com/TIERS/tello-driver-ros. We have updated the driver to be compatable with ROS Noetic which uses Python3 only. Other changes may also be implemented to meet course requirements.

## Installation

If you don't have it, create your workspace and clone this repo
```
cd catkin_ws/src
git clone --recursive https://github.com/ACFR-RPG/tello-driver-ros
git clone https://github.com/ACFR-RPG/camera_info_manager_py
```

Install dependencies
```
sudo apt install ros-noetic-codec-image-transport python-catkin-tools python3-dev python3-pip
pip3 install --upgrade pip
pip3 install https://github.com/damiafuentes/DJITelloPy/archive/master.zip
```

Build the workspace
```
cd ..
catkin init
catkin build
source devel/setup.bash
```

## Launch for Tello with its own Wi-Fi AP

- Turn on Tello drone
- Connect to drone's WiFi access point (`TELLO_XXXXXX`)

Then launch the driver
```
roslaunch tello_driver tello_node.launch tello_ip:="192.168.10.1"
```

If you run into decoding errors, please try tello_node_decode.launch instead


## ROS Node - Tello Node

### Subscribed topics
* ```/tello/cmd_vel``` [geometry_msgs/Twist](http://docs.ros.org/api/geometry_msgs/html/msg/Twist.html)
* ```/tello/emergency``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/fast_mode``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/flattrim``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/flip``` [std_msgs/Uint8](http://docs.ros.org/api/std_msgs/html/msg/UInt8.html)
* ```/tello/land``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/palm_land``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/manual_takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)
* ```/tello/throw_takeoff``` [std_msgs/Empty](http://docs.ros.org/api/std_msgs/html/msg/Empty.html)

### Published topics
* ```/tello/camera/camera_info``` [sensor_msgs/CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html)
* ```/tello/image_raw``` [sensor_msgs/Image](http://docs.ros.org/api/sensor_msgs/html/msg/Image.html)
* ```/tello/imag/raw/h264``` [h264_image_transport/H264Packet](https://github.com/tilk/h264_image_transport/blob/master/msg/H264Packet.msg)
* ```/tello/odom``` [nav_msgs/Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
* ```/tello/imu``` [sensor_msgs/Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
* ```/tello/status``` [tello_driver/TelloStatus](https://github.com/appie-17/tello_driver/blob/development/msg/TelloStatus.msg)

### Parameters
* ```~/tello_driver_node/connect_timeout_sec```
* ```~/tello_driver_node/fixed_video_rate```
* ```~/tello_driver_node/local_cmd_client_port```
* ```~/tello_driver_node/local_vid_server_port```
* ```~/tello_driver_node/stream_h264_video```
* ```~/tello_driver_node/tello_cmd_server_port```
* ```~/tello_driver_node/tello_ip```
* ```~/tello_driver_node/vel_cmd_scale```
* ```~/tello_driver_node/video_req_sps_hz```
* ```~/tello_driver_node/altitude_limit```
* ```~/tello_driver_node/attitude_limit```
* ```~/tello_driver_node/low_bat_threshold```

## Manual Control
The drone subscribes to `geometry_msgs/Twist` messages that can be used for manual control. Install the `teleop_twist_keyboard` node:
```
sudo apt install ros-noetic-teleop-twist-keyboard
```
And run it with:
```
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/tello/cmd_vel
```

## Video Glitch Filtering
Glitch will appear if the connection between the tello and your device is not good. A filtering mechanism is added. To apply this, go to /nodes and read the instruction

## IMU
The update rate of the IMU on tello is 15Hz. It is too low for any Visual-Inertial estimator like (VINS-Mono or Orb-SLAM3)


