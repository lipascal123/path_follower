path_controller

## Overview

This package takes as input the estimated pose of the robot in the world frame, the reference path and the
semantic elevation map. Based on these inputs the robot is steered along the reference path provided the path is
safe.

### License

**Author:** Pascasl Lieberherr <br />
**Project:** Lonomy, ETH Zürich <br />
**Created:** September 2022

The lonomy_path_controller package has been tested under ROS Noetic and Ubuntu 20.04

## Installation
Follow the instructions in the read me in the repository lonomy_navigation.

## Usage

To start the robot in teach mode in simulation:

```
  roslaunch lonomy_path_controller lonomy_teach_repeat.launch launch_simulation:='true' repeat_mode:='false'   
```

To start the robot in repeat mode in simulation:

```
  roslaunch lonomy_path_controller lonomy_teach_repeat.launch launch_simulation:='true' repeat_mode:='true'   
```

To start the real robot in teach mode:

```
  roslaunch lonomy_path_controller lonomy_teach_repeat.launch launch_simulation:='false' repeat_mode:='false'   
```

To start the real robot in repeat mode:

```
  roslaunch lonomy_path_controller lonomy_teach_repeat.launch launch_simulation:='false' repeat_mode:='true'   
```


## Nodes
### lonomy_path_controller_node

**Subscribed Topics**

* **`/odom`** ([nav_msgs::Odometry])

  Returns the robot odometry from simulation.


* **`/rowesys/estimator/pose_fused`** ([geometry_msgs::PoseStamped])

  Returns the robot pose from real robot.


* **`/rowesys/robot_autonomous_mode`** ([rowesys_navigation_msgs::AutonomousMode])

  Returns the mode of the robot e.g. manual mode or autonomous mode.


* **`/elevation_mapping/elevation_map_recordable`** ([grid_map_msgs::GridMap])

  Returns the semantic elevation map.


**Published Topics**

* **`/rowesys/robot_twist`** ([geometry_msgs::Twist])

    Returns the twist command for the robot


* **`/lonomy/path_controller/swath`** ([nav_msgs::Path])

    Returns the swath along the predicted path which will be checked for collision.

\
For visualization:
* **`/rowesys/ref_path`** ([nav_msgs::Path])

    Returns the entire reference path.


* **`/rowesys/lookahead_marker`** ([visualization_msgs::Marker])

    Returns a marker at the lookahead point of the pure pursuit controller.


* **`/rowesys/footprint`** ([nav_msgs::Path])

    Returns the footprint of the robot.


* **`/rowesys/predicted_path`** ([nav_msgs::Path])

    Returns the predicted path of the robot up to the end of the map.


* **`/rowesys/implement/teach_repeat_control`** ([td_msgs::Bool])

    Returns the implement status.


  Returns the recorded waypoints.

**Services**

No services

**Actions**

No actions

**Parameters**
* See config.yaml file in config folder
