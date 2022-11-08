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

## Build
I used the following command to build, as personal preference, but any colcon command should do:  
`
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --packages-select ros_pose_tracking --cmake-args=" --log-level=STATUS" --event-handlers console_direct+
`

## Run
### Start RViz with Moveit
- [For Panda ARM]  
The following moveit config/launch script has proper setup done, so no extra steps are needed:  
`
ros2 launch moveit2_tutorials demo.launch.py
`
- [For other robot ARM]  
Use your preferred moveit config/launch script, but make sure robot model is published by adding the following two extra parameters to move group node:  
`
 'publish_robot_description_semantic': True,  
 'publish_robot_description': True,
`

### Start ros_pose_tracking node
- [For Panda ARM]  
After a sourcing is done, run the code as following for Panda ARM:  
`
ros2 run ros_pose_tracking ros_pose_tracking
`
- [For other robot ARM]  
If you're working with other robot ARM, such Universal Robots' UR10e, then 
`
ros2 run ros_pose_tracking ros_pose_tracking -g ur_manipulator
`
where  
`-g`
is used to specify move group's name.  
You can find this information from your ARM's SRDF.

If there is error complaining it could not find parameter robot_description and/or robot_description_semantic, then it means when your moveit launch script doesn't publish the parameters mentioned earlier, you can either:  
1 - update moveit config launch script to enable publishing the parameters, or  
2 - provide the parameters directly to ros_pose_tracking node by using launch script

### Adding visualization tool's topic to RViz
Add ros_pose_tracking node's visualization tool's topic to RViz such that the tool shows commanding pose.
<img width="50%" src="docs/rviz_visual_tools.topic.png" alt="visulization tool setup"/>

### Commanding robot ARM to track target poses
Here's the CLI I used to command ARM:  
`
ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.28, y: -0.2, z: 0.5}, orientation: {x: 0.924135, y: -0.382065, z: -0.00018, w: -0.00048516}}}' -1
`

`
ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.28, y: -0.2, z: 0.5}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1}}}' -1
`

`
ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.307, y: 0.0, z: 0.59023}, orientation: {x: 0.924, y: -0.382, z: -0.0002, w: 0.00048}}}' -1
`

`
ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.307, y: 0.3, z: 0.59023}, orientation: {x: 0.924, y: -0.382, z: -0.0002, w: 0.00048}}}' -1
`

`
ros2 topic pub /tracking_pose geometry_msgs/msg/PoseStamped '{header: {stamp: {sec: 0, nanosec: 0}, frame_id: "map"}, pose: {position: {x: 0.307, y: -0.3, z: 0.59023}, orientation: {x: 0.924, y: -0.382, z: -0.0002, w: 0.00048}}}' -1
`
