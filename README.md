# only_add_mm
This repo is only for adding mobile manipulator support to the original REMANI-Planner.

## ISSUES
1. The inported ARX arm always collides with something. I have tried to adjust the collision spheres but it still collides. Get error 'KinoAstar: start (1) is not free!'

## Setup

Compiling tests passed on Ubuntu 20.04 with ROS installed.

### Prerequisites

- [ROS](http://wiki.ros.org/ROS/Installation) (tested with Noetic)

```
sudo apt install libompl-dev libeigen3-dev
cd /usr/include
sudo ln -sf eigen3/Eigen Eigen
sudo ln -sf eigen3/unsupported unsupported
```

### Compiling and Running

```
cd ${your catkin workspace}/src
git clone -b master --single-branch https://github.com/SYSU-STAR/REMANI-Planner.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
```

1. Navigating in dense cuboids map

```
source devel/setup.bash
roslaunch remani_planner exp0.launch
```

You should see the simulation in rviz. You can use the `2D Nav Goal` to send a trigger to start navigation.

<p align="center">
  <img src="./attachment/exp0_0.gif"/>
</p>


2. Navigating through a bridge

```
source devel/setup.bash
roslaunch remani_planner exp1.launch
```

<p align="center">
  <img src="./attachment/exp0_1.gif"/>
</p>


## Customize your own Mobile Manipulator (MM)

1. Make the following adjustments in the `remani_planner/mm_config/src/mm_config.cpp` file:
   - Modify the `getAJointTran` function to calculate the homogeneous transformation between different frames of the manipulator.
   - Modify the `setLinkPoint` function to set the position of collision spheres in the respective frame.
   - Modify the `getMMMarkerArray` function based on the urdf file to generate the marker array for MM visualization.
2. Adapt the `mm_param.yaml` file located in the `remani_planner/remani_planner/config/` directory to configure parameters specific to your MM.



**Note**: We have provided an example for the [UR5](https://www.universal-robots.com/products/ur5-robot/) in our code. To use the UR5, simply follow these steps:

1. Open the `mm_param.yaml` file located in the `remani_planner/plan_manage/config/` directory.
2. Locate the `parameter` for `FastArmer` and comment it out by adding a "#" symbol at the beginning of the line.
3. Uncomment the `parameter` for `UR5` by removing the "#" symbol at the beginning of the line.
4. You are now ready to conduct above experiments with a mobile base incorporating the UR5 configuration.

<p align="center">
  <img src="./attachment/exp1_0.gif" width = "400" height = "225"/>
  <img src="./attachment/exp1_1.gif" width = "400" height = "225"/>
</p>

