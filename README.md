\# Indoor Autonomous Navigation System Using SLAM and Nav2



This repository contains the ROS 2 source code and configuration files for an indoor autonomous navigation robot developed as a graduation project in Electrical and Electronics Engineering.



The project focuses on building a differential-drive mobile robot capable of mapping an indoor environment using 2D LiDAR-based SLAM, estimating its motion using sensor fusion, and performing autonomous point-to-point navigation using the ROS 2 Navigation Stack.



\---



\## Project Overview



The system was designed around a differential-drive mobile robot equipped with a 2D LiDAR, IMU, wheel encoders, motor driver, and an onboard NVIDIA Jetson Xavier NX. The robot used ROS 2 as the main software framework, with SLAM Toolbox for mapping, an Extended Kalman Filter for localization and odometry fusion, and Nav2 for autonomous navigation.



The project was developed to demonstrate the complete workflow of an indoor mobile robot:



1\. Sensor data acquisition

2\. Odometry estimation

3\. EKF-based sensor fusion

4\. SLAM-based map generation

5\. Map saving and reuse

6\. Autonomous navigation using Nav2

7\. Path planning, local control, and recovery behavior



\---



\## Main Features



\* ROS 2-based mobile robot software architecture

\* 2D LiDAR mapping using SLAM Toolbox

\* Differential-drive wheel odometry

\* IMU and odometry fusion using EKF

\* Nav2-based autonomous navigation

\* DWB local controller for path tracking

\* Global path planning on a saved occupancy grid map

\* Custom velocity management layer

\* Motor serial communication driver

\* Robot description and TF frame structure

\* Modular package-based project organization



\---



\## Hardware Platform



The physical robot platform used during development included:



\* NVIDIA Jetson Xavier NX

\* RPLIDAR A1M8 2D LiDAR

\* IMU module

\* Wheel encoders

\* Differential-drive DC motors

\* Motor driver

\* 12 V battery system

\* Custom mobile robot chassis



\---



\## Software Stack



The project was implemented using:



\* ROS 2

\* SLAM Toolbox

\* Nav2 Navigation Stack

\* robot\_localization EKF

\* RViz2

\* Python

\* C++

\* Linux-based robotics development environment



\---



\## Repository Structure



```text

indoor-autonomous-navigation-slam-nav2/

├── config/

│   └── slam\_toolbox/

│       └── SLAM Toolbox configuration files

│

└── src/

&#x20;   ├── differential\_odom/

&#x20;   │   └── Differential-drive odometry node

&#x20;   │

&#x20;   ├── gp\_bringup\_v/

&#x20;   │   └── Main robot bringup launch files

&#x20;   │

&#x20;   ├── gp\_description/

&#x20;   │   └── Robot description and TF-related files

&#x20;   │

&#x20;   ├── gp\_localization/

&#x20;   │   └── EKF localization configuration

&#x20;   │

&#x20;   ├── gp\_nav2/

&#x20;   │   └── Nav2 launch and parameter files

&#x20;   │

&#x20;   ├── gp\_velocity\_manager/

&#x20;   │   └── Custom velocity management node

&#x20;   │

&#x20;   └── motor\_serial\_driver/

&#x20;       └── Motor communication driver

```



\---



\## System Architecture



The robot software architecture follows the standard ROS 2 mobile robot pipeline:



```text

LiDAR Scan  ───────────────► SLAM Toolbox / Nav2 Costmaps

Wheel Encoders ───────────► Differential Odometry

IMU Data ─────────────────► EKF Sensor Fusion

EKF Output ───────────────► Localization / Odometry

Saved Map ────────────────► Nav2 Global Planning

Nav2 Controller ──────────► cmd\_vel

Velocity Manager ────────► Motor Driver

Motor Driver ────────────► Robot Motion

```



The main TF structure used in the project was:



```text

map → odom → base\_footprint → base\_link → laser

&#x20;                                     └── imu\_link

```



\---



\## Main ROS 2 Topics



Common topics used in the system included:



```text

/scan              LiDAR scan data

/imu/data\_raw      Raw IMU data

/vel\_raw           Raw wheel velocity / encoder-based data

/odom\_raw          Differential-drive odometry

/odometry/filtered EKF-filtered odometry

/cmd\_vel           Navigation velocity command

/cmd\_vel\_motor     Motor driver velocity command

/tf                Transform tree

```



\---



\## SLAM Workflow



Mapping was performed using SLAM Toolbox with 2D LiDAR scan data and odometry information. The robot explored the indoor environment while SLAM Toolbox generated an occupancy grid map.



The generated map was then saved and reused later for autonomous navigation.



General workflow:



```text

Robot Bringup → LiDAR + Odometry + IMU → EKF → SLAM Toolbox → Occupancy Grid Map → Save Map

```



\---



\## Navigation Workflow



After saving the map, the robot used Nav2 for point-to-point navigation. The navigation stack used the saved map, localization data, LiDAR scans, costmaps, global planning, and local control to generate safe motion commands.



General workflow:



```text

Saved Map → Localization → Global Planner → Local Costmap → DWB Controller → cmd\_vel → Motor Driver

```



The system also included recovery behavior through Nav2 behavior trees when the robot encountered local planning difficulty, obstacle blockage, or path execution failure.



\---



\## Build Instructions



Clone the repository into a ROS 2 workspace and build it using `colcon`:



```bash

git clone https://github.com/ARaafat237/indoor-autonomous-navigation-slam-nav2.git

cd indoor-autonomous-navigation-slam-nav2

colcon build --symlink-install

source install/setup.bash

```



\---



\## Example Launch Commands



Main robot bringup:



```bash

ros2 launch gp\_bringup\_v robot\_bringup\_v.launch.py

```



Navigation bringup:



```bash

ros2 launch gp\_nav2 gp\_nav2\_bringup.launch.py use\_supervisor:=true

```



SLAM Toolbox configuration files are located in:



```text

config/slam\_toolbox/

```



Nav2 configuration files are located in:



```text

src/gp\_nav2/

```



\---



\## Project Results



The project successfully demonstrated the main stages of an indoor autonomous navigation system:



\* Robot sensor bringup

\* LiDAR scan acquisition

\* Wheel odometry generation

\* IMU and odometry fusion using EKF

\* Indoor map generation using SLAM Toolbox

\* Saved-map navigation using Nav2

\* Global path planning

\* Local trajectory following using DWB

\* Recovery behavior during navigation difficulties



The system was tested in indoor environments with open areas, corridors, turns, and obstacle conditions.



\---



\## Limitations



The project was completed as an academic prototype. Some limitations were observed during testing:



\* Navigation performance was affected in narrow passages and tight turning areas.

\* The Jetson Xavier NX environment had software and support limitations due to the older platform setup.

\* Local planning performance depended strongly on controller frequency, costmap tuning, and CPU load.

\* Camera-based perception was not integrated in the final version.

\* The physical robot platform is no longer available, so this repository is maintained as a documented final source-code archive and portfolio reference.



\---



\## Future Improvements



Possible improvements for future development include:



\* Migrating the system to a newer ROS 2 distribution

\* Using a newer embedded AI computer

\* Adding RGB-D or stereo camera perception

\* Improving local planning performance

\* Testing alternative Nav2 controllers

\* Adding semantic obstacle detection

\* Improving navigation in narrow indoor spaces

\* Adding a more complete simulation environment



\---



\## Project Status



This repository represents the final academic implementation of the graduation project. The physical robot platform is no longer available after project completion, so the repository is maintained as a clean source-code archive, documentation reference, and GitHub portfolio project.



\---



\## Author



\*\*Ahmed Raafat\*\*

B.Sc. Electrical and Electronics Engineering

İstanbul Kültür Üniversitesi



GitHub: \[ARaafat237](https://github.com/ARaafat237)



