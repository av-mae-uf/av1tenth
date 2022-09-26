# EML4930: Autonomous Vehicles 1/10th Car Driver and Sensor Base Code
Autonomous Vehicles package for testing and creating nodes for preperation for GPS and LIDAR based autonomous navigation
### Installation

To install this package create a workspace, and call it whatever you want and change directory into the workspace

```
mkdir av1tenth_ws && cd av1tenth_ws && mkdir src && cd src
```

Then clone the github repository into the workspace

```
git clone https://github.com/av-mae-uf/av1tenth.git
```

To build go back to the 
## Requirements

This package requires the following softwares

- Ubuntu 20.04 Focal
- ROS2 Foxy
- Python 3 (ROS2 Requires this and our code will require certain packages from it)

### Ubuntu
You can download and install Ubuntu's 20.04 LTS release from their [website](https://releases.ubuntu.com/20.04/). If implementing on single-board computers (Odroid, RaspberryPi, Nvidia Jetson) it will require Armbian, or similar such software to run ROS2. This package was tested on an Odroid N2+ with [Armbian](https://www.armbian.com/odroid-n2/) focal

### ROS2
Install [ROS2 Foxy](https://docs.ros.org/en/foxy/index.html). Install from debian packages to install the packages that follow. This has only been tested on Ubuntu 20.04. Other OS may work, but additional troubleshooting may be required.

This package requires the rplidar-ros package. It can be added to your ROS2 Foxy installation through the following command:

```
sudo apt install ros-foxy-rplidar-ros
```

To run the Slamtech RPLidar and view it in Rviz, type in the following command:

```
ros2 launch rplidar_ros view_rplidar.launch.py
```

### Python

The `gps_publisher` package requires the `utm` python package which can be installed with the command:
```
python3 -m pip install utm
```
and

```
python3 -m pip install adafruit-circuitpython-gps
```
If it throws and error while trying to install with pip, install pip:
```
sudo apt install python3-pip
```

