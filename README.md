# zm_robot (updating)
The zm_robot is a AGV using four Mecanum wheel.

Software: Robot Operating System.

Version: kinetic and melodic.

Your PC need to install ros package.

if your pc used melodic version.

``` bash
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-controllers
```

``` bash
$ sudo apt-get install -y libgazebo9-dev
```

``` bash
$ sudo apt-get install -y gazebo9
```
------


if your pc used kinetic version.

``` bash
$ sudo apt-get install ros-kinetic-gazebo-ros-pkgs ros-kinetic-gazebo-ros-control ros-kinetic-ros-controllers
```

``` bash
$ sudo apt-get install -y libgazebo7-dev
```

``` bash
$ sudo apt-get install -y gazebo7
```

------

The zm_robot 3d model xaro file into Rviz.

``` bash
$ roslaunch zm_robot_description zm_robot_demo.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_rviz_update.png)

------

The zm_robot 3d model xaro file into Gazebo.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_world.launch
```
![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_gazebo_update.png)

------

This is a zm_robot control using a keyboard.

``` bash
$ cd <catkin_workspace>/src/zm_robot/zm_robot_teleop_key/scripts
```

``` bash
$ chmod +x zm_robot_teleope_key.py
```

``` bash
$ rosrun zm_robot_teleop_key zm_robot_teleope_key.py
```

------

The zm_robot wheel controllers in rviz and gazebo.

``` bash
$ roslaunch zm_robot_control zm_robot_control_rviz.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_controller_rviz.png)

------

The zm_robot using PID controller.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_pid.launch
```

``` bash
$ roslaunch zm_robot_pid_control zm_robot_pid_rviz.launch
```
------

Reference:

[1]. turtlebot3_teleop, https://github.com/ROBOTIS-GIT/turtlebot3/tree/master/turtlebot3_teleop

[2]. mecanum_sim, https://github.com/joelillo/mecanum_sim
