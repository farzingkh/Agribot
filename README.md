# Service Robot
This project simulates a four wheeled service robot that collects an item from one points and drops it in another location. The project is implemented using ROS localization and slam packages as seen below:

### Robot 
The robot is created using urdf files and xacro and simulated in Gazebo. It uses Gazebo camera and lidar plugins along with skid steer drive controller plugin for control. 

### Mapping
The mapping is done using [`gmapping`](http://wiki.ros.org/gmapping) ros package which is a wrapper for [`OpenSlam Gmapping`](https://openslam-org.github.io/gmapping.html) library that uses lidar and robot pose from wheel odometry to create a 2D occupancy grid map of robots environement. Gmapping uses Rao-Blackwellized particle filters for occupancy grid mapping. In their approach each particle carries an individual map of the evironment and the number of particles are minimized by marginalization. This package is used to generate an occupany grid map of the gazebo world and fed into the localization package.

### Localization 
The localization is done using the [`AMCL`](http://wiki.ros.org/amcl) localization package. AMLC package takes in an occupancy grid map and uses laser scan to estimate robot's pose. This package has three categories of parameters for tuning: Overall filter parameters, Laser model parameters, Odometry model parameters. The odom model used is *diff-corrected* and the laser model is *likelihood-field*. 

### Navigation
The navigation is performed using [`move_base`](http://wiki.ros.org/move_base?distro=noetic) package. It uses a global and local planner that each use an individual cost map to navigate the robot to a goal point in the environment. It can use different local and global planners that adhere to *nav_core::BaseGlobalPlanner* interface. Here the *navfn* is used for global planner that uses Dijkstra's algorithm. *base_local_planner* is used for the local planner that uses *Trajectory Rollout* to estimate velocity commands required to move the robot in the right direction. Trajectory Rollout is based on discrete sampling of the robot's control space (velocities) and perform forward simulation for each sample to find the best trajectory using different measures like proximity to obstacles, global path, or etc. These criteria can be fine-tuned based on the robot's environment. 

## Steps to launch the simulation

#### Step 1 clone the repository and build it
```sh
$ git clone https://github.com/farzingkh/Service-Robot.git
$ cd Service-Robot/catkin_ws
$ catkin_make
$ source devel/setup.bash
```

#### Step 3 Run bash scripts to launch the robot 
```sh
$ ./src/service.sh
```
Use other provided scripts for testing and fine tuning the parameters.

## Directory Structure

<pre><font color="#3465A4"><b>catkin_ws/src</b></font>
├── <font color="#3465A4"><b>add_markers</b></font>
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── <font color="#3465A4"><b>src</b></font>
│       └── add_markers.cpp
├── <font color="#3465A4"><b>amcl_localization</b></font>
│   ├── CMakeLists.txt
│   ├── <font color="#3465A4"><b>config</b></font>
│   │   ├── base_local_planner_params.yaml
│   │   ├── costmap_common_params.yaml
│   │   ├── global_costmap_params.yaml
│   │   ├── local_costmap_params.yaml
│   │   └── <font color="#3465A4"><b>__MACOSX</b></font>
│   ├── <font color="#3465A4"><b>launch</b></font>
│   │   └── amcl.launch
│   ├── <font color="#3465A4"><b>maps</b></font>
│   │   ├── <font color="#75507B"><b>map.pgm</b></font>
│   │   └── map.yaml
│   └── package.xml
├── <font color="#3465A4"><b>maps</b></font>
│   ├── CMakeLists.txt
│   ├── <font color="#75507B"><b>mymap.pgm</b></font>
│   ├── mymap.yaml
│   └── package.xml
├── <font color="#3465A4"><b>pick_objects</b></font>
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── <font color="#3465A4"><b>src</b></font>
│       └── pick_objects.cpp
├── <font color="#3465A4"><b>robot</b></font>
│   ├── CMakeLists.txt
│   ├── <font color="#3465A4"><b>launch</b></font>
│   │   ├── robot_description.launch
│   │   └── world.launch
│   ├── <font color="#3465A4"><b>meshes</b></font>
│   │   └── hokuyo.dae
│   ├── package.xml
│   ├── <font color="#3465A4"><b>rviz</b></font>
│   │   └── RViz.rviz
│   ├── <font color="#3465A4"><b>urdf</b></font>
│   │   ├── my_robot.gazebo
│   │   └── my_robot.xacro
│   └── <font color="#3465A4"><b>world</b></font>
│       ├── empty.world
│       └── office.world
├── <font color="#3465A4"><b>rvizConfig</b></font>
│   ├── CMakeLists.txt
│   ├── package.xml
│   └── RViz.rviz
├── <font color="#3465A4"><b>scripts</b></font>
│   ├── <font color="#4E9A06"><b>add_markers.sh</b></font>
│   ├── <font color="#4E9A06"><b>pick_objects.sh</b></font>
│   ├── <font color="#4E9A06"><b>test_navigation.sh</b></font>
│   └── <font color="#4E9A06"><b>test_slam.sh</b></font>
├── <font color="#3465A4"><b>slam_gmapping</b></font>
│   ├── <font color="#3465A4"><b>gmapping</b></font>
│   └── <font color="#3465A4"><b>slam_gmapping</b></font>
└── <font color="#3465A4"><b>teleop_twist_keyboard</b></font>


</pre>

## Output 
![ouput](./image/out.gif)

