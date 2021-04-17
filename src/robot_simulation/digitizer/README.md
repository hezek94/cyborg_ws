# DIGITIZER SIMULATION
![](../images/logo3.png)

## Prerequisites
- Ubuntu 16.04 or newer (Ubuntu 18.04 recommended)
- [ROS Kinetic ](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Ubuntu 16.04) or [ROS Melodic ](http://wiki.ros.org/melodic/Installation/Ubuntu) (Ubuntu 16.04)
- [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)


## Build
Build all the packages by running this inside your workspace
```sh
$ catkin build digitizer
$ source devel/setup.bash
```

## Launch the world
This will create an world with the arena and spawn the mobile_robot
```sh
$ roslaunch digitizer gazebo.launch
```

### Todos
 - [Link for the digitizer problem statement](https://drive.google.com/file/d/1TRfSipJfPAEazeUMHq2HEIN2zoGUkVlZ/view)
 - Event Coordinators: Abhijeet Tripathy & Abhilash Mishra

License
----

MIT

