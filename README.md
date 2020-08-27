# zm_robot (updating)
The zm_robot is a AGV using four Mecanum wheel.

Software: Robot Operating System.

Version: kinetic and melodic.

Your PC need to install ros package.

``` bash
$ sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control ros-melodic-ros-controllers
```

``` bash
$ sudo apt-get install -y libgazebo9-dev
```

``` bash
$ sudo apt-get install -y gazebo9
```

The zm_robot 3d model xaro file into Rviz.

``` bash
$ roslaunch zm_robot_description zm_robot_demo.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_rviz.png)

The zm_robot 3d model xaro file into Gazebo.

``` bash
$ roslaunch zm_robot_gazebo zm_robot_world.launch
```

![image](https://github.com/qaz9517532846/zm_robot/blob/master/image/zm_robot_gazebo.png)

The zm_robot four Mecanum wheel controllers.

``` bash
$ roslaunch zm_robot_control zm_robot_control.launch
```
