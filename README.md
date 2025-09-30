# Sensor-Based Collision Prev
This repository contains the `robot_no_crash` package developed. The package is designed to prevent the collision during robot movement. This functionality is developed in both 2D (the STDR) and 3D (Gazebo). 

---

> **Note:** This package is intended for ROS Noetic on Ubuntu Focal. It is recommended to have basic knowledge of ROS nodes, services, and tf transformations to use this package effectively.

---
## Table of Contents

1. [Package Structure](#package-structure)
2. [Installation of Required Packages](#installation-of-required-packages)
    - [Installation of ros-noetic-turtlebot-2d-world](#installation-of-ros-noetic-turtlebot-2d-world)
    - [Installation of STDR repository from GitHub](#installation-of-stdr-repository-from-github)
3. [Clone This Repository](#clone-this-repository)
4. [Launching the Package](#launching-the-package)
5. [Video for Running Experiment](#video-for-running-experiment)
6. [Links and Resources](#links-and-resources)

---
## Package Structure
```
  robot_no_crash
  ├── CMakeLists.txt
  ├── package.xml
  ├── launch
  │    ├── robot_no_crash.launch # for 2D STDR 
  │    └── robot_no_crash_gazebo.launch # for 3D gazebo
  ├── config
  │    ├── safety.yaml # for 2D STDR 
  │    └── safety_gazebo.yaml # for 3D gazebo
  ├── scripts
  │   └──robot_no_crash.py
  ├── src
  └── README.md
```
## Installation of Required Packages

To use this package, ensure the following dependencies are installed:

Retrieve and prepare the STDR for building.
Make a catkin workspace and make it the current working directory.

```bash
mkdir ‐p ~/stdr_ws/src
cd ~/stdr_ws/src
```
### Installation of ros-noetic-turtlebot-2d-world
Install the ros-noetic-turtlebot-2d-world package from the course repository. This package
depends on a Turtlebot 2 simulation and a Yujin Kobuki mobile base simulation. 

```bash
 wget -q https://cwru-ecse-376.github.io/cwru-ecse-376.asc -O - | sudo apt-key add -
 sudo apt update
 sudo apt install  ros-noetic-turtlebot-2d-world
```
Configure to use ROS manually

```bash
source /opt/ros/noetic/setup.bash
```

Ensure all the dependencies are installed.

```bash
rosdep install -‐from‐paths src ‐i ‐y ‐r
```

Build the packages

```bash
catkin_make
```
Add the newly built packages to the environment

```bash
source devel/setup.bash
```

To make sure it is installed search this in your computer you installed.

```bash
    dpkg -L ros-noetic-turtlebot-2d-world
```
To test it run the following launch.
To make sure it is installed search it in your computer you installed.

```bash
     roslaunch turtlebot_2d_world turtlebot_stdr_gazebo.launch
```
I got the following error in my case. If you also got the same error, please do the following steps:

The error:
```bash
     ... logging to /home/ilkekas/.ros/log/fecffa60-daa5-11ef-98e7-554aad097450/roslaunch-ilke-kas-335806.log
        Checking log directory for disk usage. This may take a while.
        Press Ctrl-C to interrupt
        Done checking log file disk usage. Usage is <1GB.


        RLException: Invalid roslaunch XML syntax: not well-formed (invalid token): line 8, column 37
        The traceback for the exception was written to the log file

```
How to fix? 

```bash
     sudo nano /opt/ros/noetic/share/turtlebot_2d_world/launch/turtlebot_stdr_gazebo.launch
```
Then change the "value-" to "value=" in the launch file and save it. Then rerun the following roslaunch command.

```bash
     roslaunch turtlebot_2d_world turtlebot_stdr_gazebo.launch
```

### Installation of STDR repository from GitHub

Clone the STDR repository from GitHub (Noetic is the default branch).

```bash
git clone ‐b noetic-devel / https://github.com/cwru‐ecse‐376/stdr_simulator.git
```
Prepare to make the packages
Change to the base of the catkin workspace.

```bash
cd ../
```
Configure to use ROS manually

```bash
source /opt/ros/noetic/setup.bash
```

Ensure all the dependencies are installed.

```bash
rosdep install -‐from‐paths src ‐i ‐y ‐r
```

Build the packages

```bash
catkin_make
```
Add the newly built packages to the environment

```bash
source devel/setup.bash
```

Start the STDR to test the installation.

```bash
roslaunch stdr_launchers server_with_map_and_gui_plus_robot
```

### Clone This Repository

```bash
    git clone https://github.com/cwru-courses/csds_476_s25_ixk238_robot_no_crash.git
```

- Compile the workspace

```bash
    catkin_make
```

- Run workspace configuration to be used by ROS

```bash
    source devel/setup.bash
```

## Launching the Package

You can launch the package in 2 different ways by using two different 

- **Launch the robot_no_crash package with rqt_robot_steering by using robot_no_crash.launch :**

```bash
 roslaunch robot_no_crash robot_no_crash.launch
  ```

- **Launch the robot_no_crash package with rqt_robot_steering by using robot_no_crash_gazebo.launch :**

```bash
 roslaunch robot_no_crash robot_no_crash_gazebo.launch
  ```
After you run both of these launch files, one needs to manually change the position of the robot, you need to change the topic to "/des_vel".

## Video for Running Experiment

### With 2D STDR


https://github.com/user-attachments/assets/90852dde-d370-406f-977c-453a1f5697a0



### With 3D Gazebo


https://github.com/user-attachments/assets/278a190e-ba2e-4e15-9dc8-445228463214




## Links and Resources
- [CWRU ECSE 476 Course Page](https://cwru-ecse-376.github.io/)
- [The physical specifications for Kobuki Base](https://kobuki.readthedocs.io/en/devel/conversions.html)
