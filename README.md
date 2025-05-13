# AwsimRvizPlugins
Rviz plugin for operating AWSIM environment.

The following features are implemented:
- EgoPlacement: Set AWSIM EGO pose from Rviz GUI tool

## Features

### EgoPlacement
Set AWSIM EGO pose from Rviz GUI tool.

`rviz_common::Tool` named `awsim_rviz_plugins::EgoPlacement` is implemented.
`awsim_rviz_plugins::EgoPlacement` gets position and orientation by dragging on the map displayed in Rviz, and publishes those as a `/awsim/awsim_rviz_plugin/ego_placement/pose_with_covariance` topic (as `geometry_msgs::msg::PoseWithCovarianceStamped`).

AWSIM subscribes this topic and updates the coordinates of the EGO.

#### How to use
1. Click the plus button on the toolbar and select `awsim_rviz_plugins/EgoPlacement` from the list.
2. Click on `AWSIM EGO Placement` button from the toolbar and select it.
3. On the map displayed in Rviz, drag the cursor to the location and orientation where you want to move the EGO.

If `pilot-auto` is running with, press the `Initialize with GNSS` button to perform localilization again.

## Installation
### Install only this plugins to Rviz
1. clone this repository
```
git clone git@github.com:tier4/AwsimRvizPlugins.git
```
2. build package
```
cd AwsimRvizPlugins
colcon build
```
3. source package
You must `source` each time you launch a terminal.
```
source install/setup.sh
```
4. launch Rviz and use tools
```
# rviz2
# ros2 launch ...
```

## TODO
### Update contents
- LICENSE
- CmakeLists.txt
- awsim_rviz_plugins-extras.cmake

### Modify sentences / coding style
- plugins_description.xml
- CHANGELOG.rst
- ego_placement.cpp / .hpp
