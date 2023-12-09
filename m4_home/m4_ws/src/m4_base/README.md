# m4_base
Python based ROS2 package for sending commands to the PX4 autopilot from offboard computers. 
The usage is the following. There should be a high-level ROS2 (foxy) workspace named ws_offboard_control. 
In ws_offboard_control/src this package should be included as well as the px4_msgs package.

Finally by properly sourcing everything the user can call ros2 run px4_offboard offboard_control
