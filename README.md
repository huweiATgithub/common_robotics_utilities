# common_robotics_utilities
Common utility functions and algorithms for robotics work used by ARC &amp; ARM labs and TRI.

## Setup

`common_robotics_utilities` is a ROS package.

Thus, it is best to build it within a ROS workspace:

```sh
mkdir -p ~/ws/src
cd ~/ws/src
git clone https://github.com/calderpg/common_robotics_utilities.git
```

## Building

Use [`catkin_make`](http://wiki.ros.org/catkin/commands/catkin_make) or
[`colcon`](https://colcon.readthedocs.io/en/released/) accordingly.

*For ROS 1 Kinetic+*
```sh
cd ~/ws
catkin_make  # the entire workspace
catkin_make --pkg common_robotics_utilities  # the package only
```

*For ROS 2 Galactic+*
```sh
cd ~/ws
colcon build  # the entire workspace
colcon build --packages-select common_robotics_utilities  # the package only
```

## Testing
