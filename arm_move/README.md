# Interbotix PX100 Motion Planning using MoveIt
## Overview

This package uses MoveIt to control Interbotix PX100 robot arm (in Simulation as well as in real world).

* User may use services listed below individually to *reset robot*, execute a *step* or make the robot follow a pre-loaded series of *waypoints*
* User may use services in combination by:
    * Resetting robot to a `Goal_Pose` pre-defined position, set-up `box_position` and `box_orientation` of the box obstacle to scene and optionally set `clear_waypoints` to delete previously collected waypoints
    * Execute step service multiple times with different `goal_position`, `goal_orientation` and `Gripper_state` as input as desired. User may choose to `record` the waypoint. The waypoint will only be recorded to rosparam server if plan _succeeds_ **and** user set `record` to _true_.
    * Once user collects multiple waypoints using step service, user may use follow service to execute entire sequence and `repeat` if desired. 
    (Note: `Follow service` calls step service in a loop but doesn't `record` in each iteration)
* The user has an option to run the code on actual *Interbotix PX100 robot arm* or in Simulation. User may visualize the robot in RVIZ optionally in both the cases.

## Dependencies
Title | Link
------------ | -------------
MoveIt| [ROS Wiki](http://wiki.ros.org/moveit)
interbotix_ros_manipulators | [Github](https://github.com/Interbotix/interbotix_ros_manipulators)


## Installation Instructions
* Create a catkin workspace
    ```
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src/
    ```
* Clone the repo into `catkin_ws/src`
    ```
    git clone https://github.com/ME495-EmbeddedSystems/homework-3-whomihirpatel.git
    ```
* Build the workspace and activate it
    ```
    cd ~/catkin_ws/
    catkin_make
    source ~/catkin_ws/devel/setup.zsh
    ```

## Usage Instructions
* To run the code on actual Interbotix PX100 robot arm:
    ```
    roslaunch arm_move arm.launch actual:=true
    ```
    * This command also launches RVIZ to visualize actual robot in action.
    * Note: By default, all `args` in ` roslaunch arm_move arm.launch` are set to `False`. Set **only one** `arg` to `True`. *Check below for all configurations options availabe*.
    
    * The waypoints are pre-loaded to `rosparam` server. To make the robot follow these series of waypoints use service below
      (Note: service input `repeat: 0` will make the robot follow series of waypoints once, i.e. repeat `0` times)
        ```
        rosservice call /px100/follow "repeat: 2"
        ```
     * Check services below to plan, execute, record step sequence or reset robot as well as add box to planning scene

* To run the code in RViz:
    ```
    roslaunch arm_move arm.launch use_fake:=true
    ```
* Services:
    * __*Reset*__
    ```
    rosservice call /px100/reset "box_position:                                                          
        x: 0.2
        y: 0.0
        z: 0.01
        box_orientation:
        x: 0.0
        y: 0.0
        z: 1.0
        w: 1.0
        Goal_Pose: 'Home'
        clear_waypoints: true"
    ```
    * __*Step*__
    ```
    rosservice call /px100/step "goal_position:                                                              
        x: 0.2
        y: 0.0
        z: 0.1
        goal_orientation:
        x: 0.0
        y: 0.0
        z: 0.0
        w: 1.0
        Gripper_state: 'Closed'
        record: true"        
    ```
    * __*Follow*__
    ```
    rosservice call /px100/follow "repeat: 2"  
    ```

* Waypoints structure:
### Configurations Instructions

* `use_actual:=true` option to run the code on actual Interbotix PX100 robot arm
* `use_fake:=true ` option to run the code in RViz
* `use_gazebo:=true ` option to run the code in Gazebo
* `rvizconfig` option to load custom rviz configuration
*  user may alter `waypoints`, `gripper_state`, as well as `frequency` parameters, from `config/waypoints.yaml`

    Set only one of the above mentioned `args` to to `True` in launch file.
    
* Demo: Click on the image below to view video: *Turtlebot3 Following Figure-8 Trajectory: Visualization in GAZEBO*

    [![Watch the video](https://img.youtube.com/vi/h6xmesHbBHA/maxresdefault.jpg )](https://youtu.be/h6xmesHbBHA)

* Demo: Click on the image below to view video: *Turtlebot3 Following Figure-8 Trajectory: Visualization in RVIZ*

    [![Watch the video](https://img.youtube.com/vi/DA9sDGc_mRw/maxresdefault.jpg )](https://youtu.be/DA9sDGc_mRw)

---
