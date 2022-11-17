# ros_pose_tracking
ROS2 package for target pose tracking with MoveIt's move group interface framework.
The node listens to the commanding topic /tracking_pose of type geometry_msgs/msg/PoseStamped.
When a commanding topic is received, it plots desired pose first for visulization, then initiates motion planning and execution.
<img width="50%" src="docs/ros_pose_tracking.gif" alt="motion planning/execution"/>

## Setup
- [Humble]  
It seems the node must run on the same machine where MoveIt runs, otherwise certain moveit components may crash.

- [Foxy]  
It should build and run out of box.  
Although it runs, it is much more complicated than [Humble] to configure, i.e., it needs launch files and corresponding robot model parameters.

## Build (same on both [Humble] and [Foxy])
I used the following command to build, as personal preference, but any colcon command should do:  
`colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ros_pose_tracking --cmake-args=" --log-level=STATUS" --event-handlers console_direct+`

## Run
### 7DOF Panda ARM
#### Start RViz with Moveit
- [Humble]  
[Humble] introduces some significant moveit configuration and launch enhancements.  
If you have moveit2_tutorial package installed, you can directly launch RViz with:  
`ros2 launch moveit2_tutorials demo.launch.py`  
There is no extra step needed. Robot model is automatically published.  
If you don't have moveit2_tutorial package, or if you prefer to use your own config/launch setup, follow the instruction for [Foxy]

- [Foxy]  
Use provided launch script to start Moveit:    
`ros2 launch ros_pose_tracking move_group.panda.launch.py`

#### Start ros_pose_tracking node
- [Humble]  
If you're using moveit2_tutorial package, run the node as:  
`ros2 run ros_pose_tracking ros_pose_tracking`  
If you're using provided launch script, follow the instruction for [Foxy]

- [Foxy]  
`ros2 launch ros_pose_tracking pose_tracking.panda.launch.py`

#### Adding visualization tool's topic to RViz
ros_pose_tracking node publishes visualization topic to ask RViz to perform visualization rendering. So we need to add this topic in RViz in order to show commanding poses.  
<img width="50%" src="docs/rviz_visual_tools.topic.png" alt="visulization tool setup"/>

#### Commanding robot ARM to track target poses
Experiments show [Humble] can do any (if not all) motion planning when it operates in ARM's workspace, and meets joint limits requirement.  
But unfortunately it is not the case for [Foxy]. There are couple of instances that motion from pose A to pose B can be performed, but not from pose B back to pose A.  

Here's the CLI I used to command ARM from another sourced shell console:  
`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.28, y: -0.2, z: 0.5}, orientation: {x: 0.924135, y: -0.382065, z: -0.00018, w: -0.00048516}}}' -1`

`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.28, y: -0.2, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}}}' -1`

`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.30744, y: -0.0000178, z: 0.5903}, orientation: {x: 0.96296, y: -0.38249, z: 0.0000226, w: -0.000029}}}' -1`

`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.316581, y: 0.515877, z: 0.388593}, orientation: {x: 0.924, y: -0.382405, z: -0.0000199, w: 0.0000177}}}' -1`

# this causing problem
`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: -0.316581, y: -0.515877, z: 0.388593}, orientation: {x: 0.924, y: -0.382405, z: -0.0000199, w: 0.0000177}}}' -1`

`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.316581, y: -0.515877, z: 0.388593}, orientation: {x: 0.924, y: -0.382405, z: -0.0000199, w: 0.0000177}}}' -1`

`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: -0.316581, y: 0.515877, z: 0.388593}, orientation: {x: 0.924, y: -0.382405, z: -0.0000199, w: 0.0000177}}}' -1`

# can't move it (UR)
`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.106936, y: -0.000008, z: 1.12102}, orientation: {x: 0.65324, y: -0.27045, z: 0.65344, w: -0.02705}}}' -1`

`ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.40589, y: 0.357684, z: 0.75333}, orientation: {x: 0.747626, y: 0.65632, z: 0.10129, w: -0.00635}}}' -1`

You can write a testing node to cycle through the above poses.  
(I probably will do it later)

### Other Robots
- [UR10e] is work in progress
#### Start UR client and controllers
#### Start RViz with Moveit for Physical ARM or URSim
- [Humble]  
Use your preferred moveit config/launch script, but make sure robot model is published by adding the following two extra parameters to move group node:  
`
 'publish_robot_description': True,
 'publish_robot_description_semantic': True,  
`
#### Start ros_pose_tracking node
Same as Panda ARM

#### Adding visualization tool's topic to RViz
Same as Panda ARM

#### Commanding robot ARM to track target poses
Same as Panda ARM
