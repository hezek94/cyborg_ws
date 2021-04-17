# LINE FOLLOWER SIMULATION
![](../images/logo3.png)

## Prerequisites
- Ubuntu 16.04 or newer (Ubuntu 18.04 recommended)
- [ROS Kinetic ](http://wiki.ros.org/kinetic/Installation/Ubuntu) (Ubuntu 16.04) or [ROS Melodic ](http://wiki.ros.org/melodic/Installation/Ubuntu) (Ubuntu 16.04)
- [Catkin Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)


## Build
Build all the packages by running this inside your workspace
```sh
$ catkin build line_follower
$ source devel/setup.bash
```

## Extract model files
Extract models.zip to home/.gazebo/
```sh
$ roscd line_follower
$ unzip models.zip -d ~/.gazebo/models/
```

## Utilities
Teleop twist keyboard:
```sh
$ https://github.com/ros-teleop/teleop_twist_keyboard.git
```

### Todos

 - Simulate a camera based line following robot. The robot should subscribe to camera/image_raw, perform image processing to detect the track/line, then use some control algorithms to align itself with the track.

License
----

MIT

