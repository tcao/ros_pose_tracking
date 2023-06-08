# WMD2023 (Humble/RViz/MoveIt/MediaPipe/TensorFlow)
The project intends to drive robot arm with hand gestures.
C++ node ros_pose_tracking from package ros_pose_tracking (same name) listens to the commanding topic **/tracking_pose** of type **geometry_msgs/msg/PoseStamped**.  
When a commanding topic is received, it plots desired pose first for visulization, then initiates motion planning and execution.  

Python node wmd2023 intends to estimate hand gestures, and to send pose request over **/tracking_pose** when a gesture is identified.
Machine learning inference is done with MediaPipe framework. The simple (less than 40 samples) training was done in TensorFlow.
The original code works good in standalone. It doesn't work yet when it is used directly in a ROS 2 python package, because of ROS 2 special python building/running requirements, causing runtime errors without able to find dependencies, or python wrapper for MediaPipe is compatible with actual installed inference engine.

# Demo
The video shows RViz in action with a Panda ARM, ros_pose_tracking node runs in the background, another C++ node ros_pose_tracking_test requests goals repeatly.  
See main branch's README about settings and runnings.  
ML node wmd2023 performs hand gesture estimation from live camera feeds, and it doesn't yet send pose goal request because above mentioned issues.
