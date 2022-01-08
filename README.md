# zm_robot

The zm robot is a autonomous mobile robot by 4 mecanum wheel driving under Gazebo simulation. It has two 2D-Didar, RGB-D camera and Imu sensor. The purpose of this project is to make it easy for people to understand the control method of Omnidirectional Wheel and the establishment of mobile robots in the environment of ROS and Gazebo.

------

## Built with

- ROS Melodic Morenia under Ubuntu 18.04 LTS

- ROS Noetic Ninjemys under Ubuntu 20.04 LTS

------

## Getting Started

### Installation

- Installation ros package.

    ``` $ sudo apt-get install ros-<distro>-gazebo-ros-pkgs ros-<distro>-gazebo-ros-control ros-<distro>-ros-controllers```

    ``` $ sudo apt-get install -y libgazebo11-dev ```
    
    ``` $ sudo apt-get install -y gazebo11 ```

    ``` $ sudo apt-get install -y ros-<distro>-openslam-gmapping ros-<distro>-navigation ```
  
    ``` $ sudo apt-get install -y ros-<distro>-amcl ros-<distro>-move-base ```

- clone gazebo_mecanum_plugins package.

``` bash
$ git clone https://github.com/qaz9517532846/gazebo_mecanum_plugins.git
```

- clone Install ira_laser_tools package.

``` bash
$ git clone https://github.com/iralabdisco/ira_laser_tools.git
```

### Run

The zm_robot 3d model xaro file into Rviz.

``` bash
$ roslaunch zm_robot_description zm_robot_demo.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros1-original/image/zm_robot_rviz.png)

- The zm_robot 3d model xaro file into Gazebo.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_world.launch
```
![image](https://github.com/qaz9517532846/zm_robot/blob/ros1-original/image/zm_robot_gazebo.png)

- The zm_robot wheel controllers in rviz and gazebo.

``` bash
$ roslaunch zm_robot_control zm_robot_control_rviz.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros1-original/image/zm_robot_control.png)

- This is a zm_robot control using a keyboard.

``` bash
$ cd <catkin_workspace>/src/zm_robot/zm_robot_teleop_key/scripts
```

``` bash
$ chmod +x zm_robot_teleope_key.py
```

``` bash
$ rosrun zm_robot_teleop_key zm_robot_teleope_key.py
```

- The zm_robot using PID controller.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_pid.launch
```

``` bash
$ roslaunch zm_robot_pid_control zm_robot_pid_rviz.launch
```

``` bash
$ rosrun rqt_reconfigure rqt_reconfigure
```

``` bash
$ rosrun rqt_gui rqt_gui
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros1-original/image/zm_robot_pid_control.png)

![image](https://github.com/qaz9517532846/zm_robot/blob/ros1-original/image/PID.png)

- The zm_robot warehouse environment under Gazebo.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_warehouse.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros1-original/image/zm_robot_in_warehouse.png)

- The zm_robot create a map at warehouse.

``` bash
$ roslaunch zm_robot_navigation zm_robot_gmapping.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros1-original/image/zm_robot_gmapping.png)

- The zm_robot can do navigation and aviod obstacles at warehouse.

``` bash
$ roslaunch zm_robot_navigation zm_robot_navigation.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros1-original/image/zm_robot_navigation.png)

- The zm_robot can do navigation and aviod obstacles at warehouse using programing.

``` bash
$ roslaunch zm_robot_navigation zm_robot_navigation.launch
```

``` bash
$ rosrun zm_robot_programing zm_robot_move
```

The zm_robot_move.cpp example.

``` bash
#include <zm_robot_programing/zm_robot_move_function.h>

void spinThread(){
  ros::spin();
}

int main(int argc, char** argv) 
{
  ros::init(argc, argv, "zm_robot_move"); 
  ros::NodeHandle n;

  boost::thread spin_thread = boost::thread(boost::bind(&spinThread));

  zm_robot my_zm_robot;

  ////// zm_robot programing control start //////

  my_zm_robot.move_map(1.0, 3.0, 1.5708);
  
  my_zm_robot.move_base(1.0, 3.0, 1.5708);

  ////// END //////

  return 0;
}
```

## illustration:

| Function                           | Description                                                |
| ---                                | ---                                                        | 
| my_zm_robot.move_map(x, y, theta)  | zm_robot can move to designated location relative to map.  |
| my_zm_robot.move_base(x, y, theta) | zm_robot can move to designated location relative to base. ||

------

## zm_robot topic

| Topic                              | Description                                                       |
| ---                                | ---                                                               | 
| cmd_vel                            | zm_robot input to move velocity.                                  |
| wheel1_velocity                    | zm_robot mecanum whee1 1 input velocity.                          |
| wheel2_velocity                    | zm_robot mecanum whee1 2 input velocity.                          |
| wheel3_velocity                    | zm_robot mecanum whee1 3 input velocity.                          |
| wheel4_velocity                    | zm_robot mecanum whee1 4 input velocity.                          |
| wheel1_PID                         | zm_robot mecanum whee1 1 PID parameter.                           |
| wheel2_PID                         | zm_robot mecanum whee1 2 PID parameter.                           |
| wheel3_PID                         | zm_robot mecanum whee1 3 PID parameter.                           |
| wheel4_PID                         | zm_robot mecanum whee1 4 PID parameter.                           |
| wheel1_output_vel                  | zm_robot mecanum whee1 1 output velocity using PID controller.    |
| wheel2_output_vel                  | zm_robot mecanum whee1 2 output velocity using PID controller.    |
| wheel3_output_vel                  | zm_robot mecanum whee1 3 output velocity using PID controller.    |
| wheel4_output_vel                  | zm_robot mecanum whee1 4 output velocity using PID controller.    |
| zm_robot_position                  | zm_robot position in Gazebo.                                      |
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

[5]. warehouse_simulation_toolkit. https://github.com/wh200720041/warehouse_simulation_toolkit

[6]. ira_laser_tools. http://wiki.ros.org/ira_laser_tools

[7]. dynamic_reconfigure. http://wiki.ros.org/dynamic_reconfigure/Tutorials

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2020 ZM Robotics Software Laboratory.
