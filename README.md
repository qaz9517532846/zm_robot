# zm_robot

The zm robot is a autonomous mobile robot by 4 mecanum wheel driving under Gazebo simulation. It has two 2D-Lidar, RGB-D camera and Imu sensor. The purpose of this project is to make it easy for people to understand the control method of Omnidirectional Wheel and the establishment of mobile robots in the environment of ROS and Gazebo.

------

## Built with

- ROS Jazzy under Ubuntu 24.04 LTS

------

## Getting Started

### Installation

- Installation Gazebo Harmonic

    - Gazebo Harmonic Install Tutorial - https://gazebosim.org/docs/harmonic/install_ubuntu/

- Installation ros package.

    ``` $ sudo apt-get install ros-jazzy-ros-gz-sim ros-jazzy-xacro```

    ``` $ sudo apt-get install -y ros-jazzy-cartographer-ros ```
  
    ``` $ sudo apt-get install -y ros-jazzy-navigation2 ```
    
    ``` $ sudo apt-get install -y ros-jazzy-nav2-bringup ```

### Run

- The zm_robot 3d model xaro file into Rviz.

``` bash
$ ros2 launch zm_robot_description zm_robot_demo.launch.py
```

- The zm_robot 3d model xaro file into Gazebo.

``` bash
$ ros2 launch zm_robot_gazebo zm_robot_empty_world.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2-jazzy/image/zm_robot_empty_world.png)

- This is a zm_robot control using a keyboard.

``` bash
$ ros2 run zm_robot_teleop zm_robot_teleop_key
```

- The zm_robot warehouse environment under Gazebo.

``` bash
$ ros2 launch zm_robot_gazebo zm_robot_warehouse.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2-jazzy/image/zm_robot_aws_warehouse.png)

- The zm_robot create a map at warehouse.

``` bash
$ ros2 launch zm_robot_navigation zm_robot_cartographer.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2-jazzy/image/zm_robot_cartographer.png)

- Cartographer save map command.

``` bash
$ ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename : '${HOME}/zm_robot_cartographer_map.pbstream'}"
$ ros2 run nav2_map_server map_saver_cli -f ~/map
```

- The zm_robot navigation using cartographer localization.

``` bash
$ ros2 launch zm_robot_navigation zm_robot_cartographer_navigation2.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2-jazzy/image/zm_robot_navigation2.png)

------

- The zm_robot can do navigation and aviod obstacles at warehouse using programing.

``` bash
$ ros2 run zm_robot_programing zm_robot_move
```

The zm_robot_move.cpp example.

``` bash
import rclpy
from zm_robot_programing.zm_robot_action import zm_robot_cmd

def main(args=None):
    rclpy.init(args=args)
    zm_robot = zm_robot_cmd()
    zm_robot.move_map(1.0, 1.0, 0.0)
    zm_robot.move_map(1.0, 0.0, 0.0)
    zm_robot.move_base(-1.0, 0.0, 0.0)
    exit(0)


if __name__ == '__main__':
    main()
```

## illustration:

| Function                           | Description                                                |
| ---                                | ---                                                        | 
| zm_robot.move_map(x, y, theta)  | zm_robot can move to designated location relative to map.     |
| zm_robot.move_base(x, y, theta) | zm_robot can move to designated location relative to base.    ||

------

## zm_robot topic

| Topic                              | Description                                                       |
| ---                                | ---                                                               | 
| cmd_vel                            | zm_robot input to move velocity.                                  |
| joint_states                       | zm_robot joint status topic.                                      |
| odomentry                          | zm_robot odomentry topic.                                         |
| sick_lidar0/scan                   | sick laser1 Laserscan.                                            |
| sick_lidar1/scan                   | sick laser2 Laserscan.                                            |
| kinect_v2/color                    | Kinect V2 RGB image.                                              |
| kinect_v2/depth                    | Kinect V2 Depth image.                                            |
| imu                                | zm_robot IMU sensor.                                              ||

------

## History:

| Branch         | Description                                                                     | File Creation |
| ---            | ---                                                                             | ---           |
| ros1-original  | zm_robot original version.                                                      | July, 2020.   |
| ros1           | zm_robot ros1 version for zm_robot programing simply.                           | July, 2021.   |
| ros1-main      | zm_robot ros1-main version add safety function           .                      | August, 2021. |
| ros1-pid       | zm_robot ros1-pid version using PID controller for zm_robot programing simply.  | July, 2021.   |
| ros2-foxy      | zm_robot ros2 versoin under ROS 2 Foxy environment.                             | August, 2021. |
| ros2-jazzy     | zm_robot ros2 versoin under ROS 2 Jazzy environment.                            | February, 2025.   ||

------

## Reference:

[1]. turtlebot3_teleop, https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop

[2]. Gazebo - Get started. https://gazebosim.org/docs/latest/getstarted/

[3]. aws-robomaker-small-warehouse-world. https://github.com/Juancams/aws-robomaker-small-warehouse-world

[4]. Cartographer ROS2. https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html

[5]. navigation2. https://github.com/ros-planning/navigation2

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2025 ZM Robotics Software Laboratory.
