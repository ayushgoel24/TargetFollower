# TargetFollower

This is a RaspberryPi-based robot which chases a target using PID control and a combination of sensor data.

I created a ROS package that consists of three different functions. The first node subscribed to the RaspberryPI camera node and published the location of the centre of the target being tracked. The second node publishes the angular position and distance of the target using a combination of LIDAR and cameraa data. The third node implements the PID control, which calculates the angular error that makes the robot face the target and also the linear error which keeps it at a certain distance from the target. This node publishes velocity commands for the robot to follow
