# zm_robot

The zm robot is a autonomous mobile robot by 4 mecanum wheel driving under NVIDIA isaac sim. It has two 2D-Lidar, RGB-D camera and Imu sensor. The purpose of this project is to make it easy for people to understand the control method of Omnidirectional Wheel and the establishment of mobile robots in the environment of ROS and NVIDIA isaac sim.

------

## System Requirements

- OS : Ubuntu 24.04
- CPU : Intel Core i7 (7th Generation) / AMD Ryzen 5
- Cores : 4
- RAM : 32 GB
- Storage : 50GB SSD
- GPU : GeForce RTX 4080
- VRAM : 16GB
- Driver : Linux: 580.65.06

------

## Built with

- ROS Jazzy under Ubuntu 24.04 LTS

------

## Getting Started

### Installation

- Installation NVIDIA Isaac Sim

    - NVIDIA Isaac Sim Install - https://docs.isaacsim.omniverse.nvidia.com/5.1.0/installation/download.html

- Installation ros package.

    ``` $ sudo apt-get install ros-Jazzy-xacro```

    ``` $ sudo apt-get install -y ros-Jazzy-cartographer-ros ```
  
    ``` $ sudo apt-get install -y ros-Jazzy-navigation2 ```
    
    ``` $ sudo apt-get install -y ros-Jazzy-nav2-bringup ```

- Installation NVIDIA VPI
    - NVIDIA VPI - https://docs.nvidia.com/vpi/installation.html

### Run

- Open zm_robot_flat_grid.usd after launching NVIDIA Isaac Sim.

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2-isaac-sim/image/zm_robot_flat_grid.png)

- This is a zm_robot control using a keyboard.

``` bash
$ ros2 run zm_robot_teleop zm_robot_teleop_key
```

- The zm_robot simple environment under NVIDIA Isaac Sim.

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2-isaac-sim/image/zm_robot_simple_room.png)


- The zm_robot create a map at simple room.

``` bash
$ ros2 launch zm_robot_navigation zm_robot_cartographer.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2-isaac-sim/image/zm_robot_cartographer.png)

- Cartographer save map command.

``` bash
$ ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename : '${HOME}/zm_robot_cartographer_map.pbstream'}"
$ ros2 run nav2_map_server map_saver_cli -f ~/map
```

- The zm_robot navigation using cartographer localization.

``` bash
$ ros2 launch zm_robot_navigation zm_robot_cartographer_navigation2.launch.py
```

![image](https://github.com/qaz9517532846/zm_robot/blob/ros2-isaac-sim/image/zm_robot_navigation2.png)

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
| odom                               | zm_robot odomentry topic.                                         |
| sick_lidar0/scan                   | sick laser1 Laserscan.                                            |
| sick_lidar1/scan                   | sick laser2 Laserscan.                                            |
| kinect_v2/color/image_raw          | Kinect V2 RGB image.                                              |
| kinect_v2/depth/image_raw          | Kinect V2 Depth image.                                            |
| imu                                | zm_robot IMU sensor.                                              ||

------

## 🎬 Simulation Demo (YouTube)

Click the image below to watch the demo video:

[![Isaac Sim Mecanum Robot Demo](https://img.youtube.com/vi/rabQwstnbsQ/maxresdefault.jpg)](https://youtu.be/rabQwstnbsQ)

[![Isaac Sim Mecanum Robot Mapping](https://img.youtube.com/vi/QpQLJSyjY1c/maxresdefault.jpg)](https://youtu.be/QpQLJSyjY1c)

[![Isaac Sim Mecanum Robot Nav](https://img.youtube.com/vi/EszqV1yoW10/maxresdefault.jpg)](https://youtu.be/EszqV1yoW10)


------

## History:

| Branch         | Description                                                                     | File Creation |
| ---            | ---                                                                             | ---           |
| ros1-original  | zm_robot original version.                                                      | July, 2020.   |
| ros1           | zm_robot ros1 version for zm_robot programing simply.                           | July, 2021.   |
| ros1-main      | zm_robot ros1-main version add safety function           .                      | August, 2021. |
| ros1-pid       | zm_robot ros1-pid version using PID controller for zm_robot programing simply.  | July, 2021.   |
| ros2-foxy      | zm_robot ros2 versoin under ROS 2 Foxy environment.                             | August, 2021. |
| ros2-jazzy     | zm_robot ros2 versoin under ROS 2 Jazzy environment.                            | February, 2025.   |
| ros2-isaac-sim | zm_robot ros2 versoin under ROS 2 Jazzy and NVIDIA isaac sim                    | February, 2026.   ||

------

## Reference:

[1]. turtlebot3_teleop. https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop

[2]. Isaacsim ROS2 tutorial. https://docs.isaacsim.omniverse.nvidia.com/5.1.0/ros2_tutorials/index.html

[4]. Cartographer ROS2. https://ros2-industrial-workshop.readthedocs.io/en/latest/_source/navigation/ROS2-Cartographer.html

[5]. navigation2. https://github.com/ros-planning/navigation2

[6]. Isaac ROS. https://nvidia-isaac-ros.github.io/index.html

------

This repository is for your reference only. copying, patent application, academic journals are strictly prohibited.

Copyright © 2026 ZM Robotics Software Laboratory.
