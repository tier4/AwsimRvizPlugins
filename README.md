# AwsimRvizPlugins
Rviz plugin for operating AWSIM environment.

The following features are implemented:
- EgoPlacement: Set AWSIM EGO pose from Rviz GUI tool
- NpcSpawner: Spawn AWSIM Npc from Rviz GUI tool

## Features

### EgoPlacement
Set AWSIM EGO pose from Rviz GUI tool.

`rviz_common::Tool` named `awsim_rviz_plugins/EgoPlacement` is implemented.
`awsim_rviz_plugins/EgoPlacement` gets position and orientation by dragging on the map displayed in Rviz, and publishes those as a `/awsim/awsim_rviz_plugin/ego_placement/pose_with_covariance` topic (as `geometry_msgs::msg::PoseWithCovarianceStamped`).

AWSIM subscribes this topic and updates the coordinates of the EGO.

#### How to use
1. Click the plus button on the toolbar and select `awsim_rviz_plugins/EgoPlacement` from the list.
2. Click on `AWSIM EGO Placement` button from the toolbar and select it.
3. On the map displayed in Rviz, drag the cursor to the location and orientation where you want to move the EGO.

If `pilot-auto` is running with, press the `Initialize with GNSS` button to perform localilization again.

### NpsSpawner
Spawn AWSIM Npc from Rviz GUI tool.
Type and velocity of spawned Npc is specified from Rviz display.

`rviz_common::Tool` named `awsim_rviz_plugins/NpsSpawner` is implemented.
`awsim_rviz_plugins/NpsSpawner` gets position and orientation by dragging on the map displayed in Rviz, and publishes those as a `/awsim/awsim_rviz_plugin/npc_spawner/pose_with_covariance` topic (as `geometry_msgs::msg::PoseWithCovarianceStamped`).

AWSIM subscribes this topic and spawn NPC on the coordinates.

`rviz_common::Display` named `awsim_rviz_plugins/NpsSpawnerStatus` is implemented.
`awsim_rviz_plugins/NpsSpawnerStatus` is entered Npc type (drop down list) and velocity, and publishes those as a `/awsim/awsim_rviz_plugin/npc_spawner/npc_name` and `/awsim/awsim_rviz_plugin/npc_spawner/npc_velocity` topic.

AWSIM subscribes those topics and specify type and velocity of spawned NPC.

#### How to use
1. Click the plus button on the toolbar and select `awsim_rviz_plugins/EgoPlacement` from the list.
2. Click on `AWSIM EGO Placement` button from the toolbar and select it.
3. On the map displayed in Rviz, drag the cursor to the location and orientation where you want to move the EGO.

If you want to change type and velocity of spawned NPC, do the following:
1. Click the `Add` button on the `Display` panel and select `awsim_rviz_plugins/NpcSpawner` from the list.
2. Change the value of `NPC Type` and `Velocity [km/h]`.


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

## TODO
### Update contents
- LICENSE
- CmakeLists.txt