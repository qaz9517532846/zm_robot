# zm_robot

The zm robot is a autonomous mobile robot by 4 mecanum wheel driving under Gazebo simulation. It has two 2D-Didar, RGB-D camera and Imu sensor. The purpose of this project is to make it easy for people to understand the control method of Omnidirectional Wheel and the establishment of mobile robots in the environment of ROS and Gazebo.

------

## Built with

- ROS Foxy under Ubuntu 20.04 LTS

------

## Getting Started

### Installation

- Installation ros package.

    ``` $ sudo apt-get install ros-foxy-gazebo-ros-pkgs ros-foxy-gazebo-ros ros-foxy-gazebo-dev ```

    ``` $ sudo apt-get install -y libgazebo11-dev ```
    
    ``` $ sudo apt-get install -y gazebo11 ```

    ``` $ sudo apt-get install -y ros-foxy-cartographer-ros ```
  
    ``` $ sudo apt-get install -y ros-foxy-navigation2 ```
    
    ``` $ sudo apt-get install -y ros-foxy-nav2-bringup ```
    
  - You may need to source Gazebo's setup file if you're having difficulty finding plugins and other resources. 
  
    ``` $ source /usr/share/gazebo/setup.sh ```

- clone gazebo_mecanum_plugins package.

``` bash
$ git clone -b ros2-foxy https://github.com/qaz9517532846/gazebo_mecanum_plugins.git
```

- clone AWS RoboMaker Small Warehouse World package.

``` bash
$ git clone -b ros2 https://github.com/aws-robotics/aws-robomaker-small-warehouse-world
```

### Run

- The zm_robot 3d model xaro file into Rviz.

``` bash
$ ros2 launch zm_robot_description zm_robot_demo.launch.py
```

- The zm_robot 3d model xaro file into Gazebo.

``` bash
$ ros2 launch zm_robot_gazebo zm_robot_empty_world.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2/image/zm_robot_empty_world.png)

- This is a zm_robot control using a keyboard.

``` bash
$ ros2 run zm_robot_teleop zm_robot_teleope_key
```

- The zm_robot warehouse environment under Gazebo.

``` bash
$ ros2 launch zm_robot_gazebo zm_robot_aws_warehouse.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2/image/zm_robot_aws_warehouse.png)

- The zm_robot create a map at warehouse.

``` bash
$ ros2 launch zm_robot_navigation zm_robot_cartographer.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2/image/zm_robot_cartographer.png)

- Cartographer save map command.

``` bash
$ ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id : 0}"
$ ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename : '${HOME}/zm_robot_cartographer_map.pbstream'}"
$ ros2 run nav2_map_server map_saver_cli -f ~/map
```

- The zm_robot can do navigation and aviod obstacles at warehouse.

``` bash
$ ros2 launch zm_robot_navigation zm_robot_navigation2.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2/image/zm_robot_navigation2.png)

------

## zm_robot topic

| Topic                              | Description                                                       |
| ---                                | ---                                                               | 
| cmd_vel                            | zm_robot input to move velocity.                                  |
| joint_states                       | zm_robot joint status topic.                                      |
| odom                               | zm_robot odomentry topic.                                         |
| sick_s30b/laser/scan0              | sick laser1 Laserscan.                                            |
| sick_s30b/laser/scan1              | sick laser2 Laserscan.                                            |
| kinect_v2/color/image_raw          | Kinect V2 RGB image.                                              |
| kinect_v2/depth/image_raw          | Kinect V2 Depth image.                                            |
| zm_robot_imu                       | zm_robot IMU sensor.                                              ||

------

## History:

| Branch         | Description                                                                     | File Creation |
| ---            | ---                                                                             | ---           |
| ros1-original  | zm_robot original version.                                                      | July, 2020.   |
| ros1           | zm_robot ros1 version for zm_robot programing simply.                           | July, 2021.   |
| ros1-main      | zm_robot ros1-main version add safety function           .                      | August, 2021. |
| ros1-pid       | zm_robot ros1-pid version using PID controller for zm_robot programing simply.  | July, 2021.   |
| ros2           | zm_robot ros2 versoin under ROS 2 Foxy environment.                             | August, 2021. ||

------

## Reference:

[1]. turtlebot3_teleop, https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop

[2]. mecanum_sim, https://github.com/joelillo/mecanum_sim

[3]. Gazebo tutorial - Control plugin. http://gazebosim.org/tutorials?cat=guided_i&tut=guided_i5

[4]. Gazebo tutorial - Sensor plugin. http://gazebosim.org/tutorials?tut=ros_gzplugins&cat=connect_ros

[5]. aws-robomaker-small-warehouse-world. https://github.com/aws-robotics/aws-robomaker-small-warehouse-world

[6]. gazebo_mecanum_plugins. https://github.com/qaz9517532846/gazebo_mecanum_plugins

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2021 ZM Robotics Software Laboratory.
