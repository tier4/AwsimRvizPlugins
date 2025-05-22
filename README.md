# AwsimRvizPlugins
Rviz plugin for operating AWSIM environment.

The following features are implemented:
- 2dPoseTeleport: Teleport AWSIM EGO pose from Rviz GUI tool

## Features

### 2DPoseTeleport
Set AWSIM EGO pose from Rviz GUI tool.

`rviz_common::Tool` named `awsim_rviz_plugins/2dPoseTeleport` is implemented.
`awsim_rviz_plugins/2dPoseTeleport` gets position and orientation by dragging on the map displayed in Rviz, and publishes those as a `/awsim/awsim_rviz_plugin/pose_teleport/pose_with_covariance` topic (as `geometry_msgs::msg::PoseWithCovarianceStamped`).

AWSIM subscribes this topic and updates the coordinates of the EGO.

#### How to use
1. Click the plus button on the toolbar and select `awsim_rviz_plugins/2dPoseTeleport` from the list.
2. Click on `2D Pose Teleportt` button from the toolbar and select it.
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

### Install with Autoware (pilot-auto)
1. clone Autoware repository
2. add description of AwsimRvizPlugins to autoware.repos
```
repositories:
  # please add the following statement to autoware.repos to use AwsimRvizPlugins
  simulator/awsim_rviz_plugins:
    type: git
    url: git@github.com:tier4/AwsimRvizPlugins.git
```
3. Introduce `Autoware (pilot-auto)` according to [Autoware Documentation](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/).
